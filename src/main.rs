#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_rp::{
    i2c::{self, Blocking, I2c},
    peripherals::I2C0,
};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::Timer;
use panic_halt as _;
use static_cell::StaticCell;

use crate::drivers::oled::OLED;

mod drivers;
mod wireless;

const RESET_REASON_ROW: i32 = 0;
const POWER_STATUS_ROW: i32 = 1;
const IP_ROW: i32 = 2;

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

    // OLED on I2C0: SDA = GPIO16, SCL = GPIO17. The bus lives in a RefCell so
    // the ssd1306 driver can share it via embedded-hal-bus. Both the bus and
    // the OLED are promoted to 'static (via StaticCell) so power_task can hold
    // a &'static reference to the shared display.
    let i2c_bus = I2c::new_blocking(p.I2C0, p.PIN_17, p.PIN_16, i2c::Config::default());
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
            oled.write_text("PwrOn | BrownOut", 0, RESET_REASON_ROW);
        } else {
            oled.write_text("No Valid Reset Reason", 0, RESET_REASON_ROW);
        }
        oled.write_text("Connecting...", 0, IP_ROW);
    });

    spawner.spawn(power_task(shared_oled).unwrap());

    // Bring up the wireless chip and join the network. Returns the IP
    // address (or a failure reason) ready to display.
    let status = wireless::init(
        spawner,
        p.PIN_23,
        p.PIN_24,
        p.PIN_25,
        p.PIN_29,
        p.PIO0,
        p.DMA_CH0,
        shared_oled,
    )
    .await;

    shared_oled.lock(|o| {
        let mut oled = o.borrow_mut();
        oled.clear_row(IP_ROW);
        oled.write_text(status.as_str(), 0, IP_ROW);
    });

    loop {
        Timer::after_secs(1).await;
    }
}
