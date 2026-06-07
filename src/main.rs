#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use hal::pac;
use panic_halt as _;
use rp2040_hal::{self as hal, fugit, Clock, I2C};

use crate::drivers::oled::{OLED};

mod drivers;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
const I2C_BUS_FREQUENCY_KHZ: u32 = 100;

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

    oled_driver.write_text("New text", 0, 0);

    oled_driver.write_text("Other text", 64, 128);

    loop {
        led.set_high().unwrap();
        delay.delay_ms(500);
        led.set_low().unwrap();
        delay.delay_ms(500);
    }
}
