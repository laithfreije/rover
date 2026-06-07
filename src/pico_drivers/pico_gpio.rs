use rp2040_hal::{gpio::Pins, pac::{IO_BANK0, PADS_BANK0, RESETS, SIO}};

use crate::mech_hal::mech_gpio::MechGPIO;


pub struct PicoGPIO {
    pins: Pins
}

impl PicoGPIO{
    pub fn new(sio: SIO, bank0: IO_BANK0, pads: PADS_BANK0, resets: &mut RESETS) -> Self {
        let sio: rp2040_hal::Sio = rp2040_hal::Sio::new(sio);
        let pins = rp2040_hal::gpio::Pins::new(
            bank0,
            pads,
            sio.gpio_bank0,
            resets,
        );

        Self{pins}
    }
}

impl MechGPIO for PicoGPIO {
    fn set_output(self, num: u8, is_output: bool) -> Result<(), crate::mech_hal::mech_gpio::GPIOError> {
        todo!()
    }

    fn set_level(self, num: u8, is_high: bool) -> Result<(), crate::mech_hal::mech_gpio::GPIOError> {
        todo!()
    }

    fn get_level(self, num: u8) -> Result<bool, crate::mech_hal::mech_gpio::GPIOError> {
        todo!()
    }

    fn set_function(self, num: u8, function: crate::mech_hal::mech_gpio::MechGPIOFunc) -> Result<(), crate::mech_hal::mech_gpio::GPIOError> {
        todo!()
    }
}