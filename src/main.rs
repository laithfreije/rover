#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use hal::pac;
use panic_halt as _;
use rp2040_hal::{self as hal, Clock, I2C, fugit::{self, ExtU32}, gpio::{FunctionSioInput, Pin, PullUp, bank0::Gpio5}, timer::{Alarm, Alarm0}};

use crate::{control::irsensor::{IRSensor, SAMPLE_HZ}, drivers::oled::OLED};
use core::fmt::Write;   // <-- this line

mod drivers;
mod control;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

static IR: Mutex<RefCell<Option<(IRSensor, IrPin, Alarm0)>>> = Mutex::new(RefCell::new(None));

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
const I2C_BUS_FREQUENCY_KHZ: u32 = 100;

type IrPin = Pin<Gpio5, FunctionSioInput, PullUp>;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led = pins.gpio25.into_push_pull_output();

    let i2c_bus = I2C::i2c0(
        pac.I2C0,
        pins.gpio16.reconfigure(),
        pins.gpio17.reconfigure(),
        fugit::RateExtU32::kHz(I2C_BUS_FREQUENCY_KHZ),
        &mut pac.RESETS,
        fugit::RateExtU32::Hz(XOSC_CRYSTAL_FREQ),
    );

    let refcell_i2c = RefCell::new(i2c_bus);

    let mut oled_driver = OLED::new(&refcell_i2c);

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm = timer.alarm_0().unwrap();
    let ir_pin: IrPin = pins.gpio5.reconfigure();

    let ir = IRSensor::new();

    alarm.schedule((1_000_000 / SAMPLE_HZ).micros()).unwrap();
    alarm.enable_interrupt();

    critical_section::with(|cs| {
        IR.borrow(cs).replace(Some((ir, ir_pin, alarm)));
    });

    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0); }

    loop {
        led.set_high().unwrap();
        delay.delay_ms(500);
        led.set_low().unwrap();
        delay.delay_ms(500);
    }
}
