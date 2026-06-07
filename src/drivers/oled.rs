use core::cell::RefCell;

use embedded_hal::i2c::{AddressMode, Error, I2c};

enum OLEDCommands {
    DisplayOff = 0xAE,
    DisplayOn = 0xAF,
    ChargePumpSelect = 0x8D,
    AddressMode = 0x20,
    ColumnRange = 0x21,
    PageRange = 0x22
}

enum OLEDValues {
    ChargePumpOn = 0x14,
    HorizontalAddressMode = 0x00,
    VerticalAddressMode = 0x01,
}

pub struct OLED<'a, I>{
    i2c: &'a RefCell<I>,
    address: u8,
}

impl<'a, I: embedded_hal::i2c::I2c> OLED<'a, I> {
    fn send_command(&mut self, command: OLEDCommands, values: &[u8]) {

    }

    pub fn new(i2c: &'a RefCell<I>, address: u8) -> Self {
        // Turn on charge pump
        let mut oled = Self{i2c, address};

        oled.send_command(OLEDCommands::DisplayOff, &[]);
        oled.send_command(OLEDCommands::ChargePumpSelect, &[OLEDValues::ChargePumpOn as u8]);
        oled.send_command(OLEDCommands::AddressMode, &[OLEDValues::HorizontalAddressMode as u8]);
        oled.send_command(OLEDCommands::ColumnRange, &[0, 127]);
        oled.send_command(OLEDCommands::PageRange, &[0, 7]);
        oled.send_command(OLEDCommands::DisplayOn, &[]);
        oled
    }
}