#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{AnyPin, Level, Output},
    i2c::{self, Blocking, I2c},
    peripherals::I2C0,
};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{block_for, Duration, Timer};
use static_cell::StaticCell;

/// GPIO pin (bank 0) wired to the panic-indicator LED.
const PANIC_LED_PIN: u8 = 14;

/// Panic handler: blink the LED on [`PANIC_LED_PIN`] at 1 Hz forever.
///
/// By the time we land here the async executor is gone, so we can't use
/// `Timer`/spawning. Instead we steal the pin (nothing else owns GPIO14)
/// and use `block_for`, a busy-wait against the still-running hardware
/// timer. A 1 Hz blink is a full on+off cycle per second, i.e. toggle
/// every 500 ms.
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let pin = unsafe { AnyPin::steal(PANIC_LED_PIN) };
    let mut led = Output::new(pin, Level::Low);
    loop {
        led.toggle();
        block_for(Duration::from_millis(500));
    }
}

use crate::drivers::oled::OLED;

mod bluetooth;
mod drivers;
mod motor;
mod xbox;

const RESET_REASON_ROW: i32 = 0;
const POWER_STATUS_ROW: i32 = 1;
const STATUS_ROW: i32 = 2;

pub type SharedOled =
    Mutex<CriticalSectionRawMutex, RefCell<OLED<'static, I2c<'static, I2C0, Blocking>>>>;

#[embassy_executor::task]
async fn power_task(oled: &'static SharedOled) -> ! {
    loop {
        // Check status of regulator
        let is_ok = embassy_rp::pac::VREG_AND_CHIP_RESET.vreg().read().rok();
        if is_ok {
            oled.lock(|o| o.borrow_mut().write_text("vreg: ok", 0, POWER_STATUS_ROW));
        } else {
            oled.lock(|o| o.borrow_mut().write_text("vreg: bad", 0, POWER_STATUS_ROW));
        }

        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // OLED on I2C0: SDA = GPIO8, SCL = GPIO9 (GPIO16/17 are now the motor PWM
    // pins). The bus lives in a RefCell so the ssd1306 driver can share it via
    // embedded-hal-bus. Both the bus and the OLED are promoted to 'static (via
    // StaticCell) so power_task can hold a &'static reference to the display.
    let i2c_bus = I2c::new_blocking(p.I2C0, p.PIN_9, p.PIN_8, i2c::Config::default());
    static I2C_CELL: StaticCell<RefCell<I2c<'static, I2C0, Blocking>>> = StaticCell::new();
    let refcell_i2c = I2C_CELL.init(RefCell::new(i2c_bus));
    let oled = OLED::new(refcell_i2c);

    static OLED_CELL: StaticCell<SharedOled> = StaticCell::new();
    let shared_oled: &'static SharedOled = OLED_CELL.init(Mutex::new(RefCell::new(oled)));

    // Check if reset was caused by brownout
    let had_debug_port_reset = embassy_rp::pac::VREG_AND_CHIP_RESET
        .chip_reset()
        .read()
        .had_psm_restart();
    let had_run_pin_reset = embassy_rp::pac::VREG_AND_CHIP_RESET
        .chip_reset()
        .read()
        .had_run();
    let had_brownout_reset = embassy_rp::pac::VREG_AND_CHIP_RESET
        .chip_reset()
        .read()
        .had_por();

    shared_oled.lock(|o| {
        let mut oled = o.borrow_mut();
        if had_debug_port_reset {
            oled.write_text("Debug Port Reset", 0, RESET_REASON_ROW);
        } else if had_run_pin_reset {
            oled.write_text("Run Pin Reset", 0, RESET_REASON_ROW);
        } else if had_brownout_reset {
            oled.write_text("PwrOn|BrownOut", 0, RESET_REASON_ROW);
        } else {
            oled.write_text("Invalid Reset Reason", 0, RESET_REASON_ROW);
        }
        oled.write_text("Connecting...", 0, STATUS_ROW);
    });

    spawner.spawn(power_task(shared_oled).unwrap());

    // Motors: TB6612FNG on PWMA=GP16, PWMB=GP17 (PWM slice 0), AI1=GP21,
    // AI2=GP22, BI1=GP19, BI2=GP18, STBY=GP20. The drive task reacts to the
    // shared controller state and halts on signal loss.
    let motors = motor::MotorController::init(
        p.PWM_SLICE0,
        p.PIN_16,
        p.PIN_17,
        p.PIN_21,
        p.PIN_22,
        p.PIN_19,
        p.PIN_18,
        p.PIN_20,
    );
    spawner.spawn(motor::drive_task(motors).unwrap());

    // Bring up the cyw43 chip's Bluetooth and run the controller link forever.
    bluetooth::run(
        spawner,
        p.PIN_23,
        p.PIN_24,
        p.PIN_25,
        p.PIN_29,
        p.PIO0,
        p.DMA_CH0,
        p.DMA_CH1,
        p.FLASH,
        shared_oled,
    )
    .await;
}
