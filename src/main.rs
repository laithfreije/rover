#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use hal::pac;
use panic_halt as _;
use rp2040_hal::{self as hal, Clock};

use crate::{mech_hal::mech_gpio::MechGPIO, pico_drivers::pico_gpio::PicoGPIO};

mod mech_hal;
mod pico_drivers;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

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

    let pico_gpio: RefCell<PicoGPIO> = RefCell::new(PicoGPIO::new(pac.SIO, pac.IO_BANK0, pac.PADS_BANK0, &mut pac.RESETS));
       
    pico_gpio.borrow_mut().set_output(25, true).unwrap();

    loop {
        pico_gpio.borrow_mut().set_level(25, true).unwrap();
        delay.delay_ms(500);
        pico_gpio.borrow_mut().set_level(25, false).unwrap();
        delay.delay_ms(500);
    }
}
