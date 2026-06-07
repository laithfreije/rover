use core::cell::RefCell;

use embedded_hal::i2c::{AddressMode, Error, ErrorType, I2c};

const OLED_ADDRESS: u8 = 0x3C;

enum OLEDControlBytes {
    CommandFlow = 0x00,
    DataFlow = 0x40,
}

enum OLEDCommands {
    DisplayOff = 0xAE,
    DisplayOn = 0xAF,
    ChargePumpSelect = 0x8D,
    AddressMode = 0x20,
    ColumnRange = 0x21,
    PageRange = 0x22,
}

enum OLEDValues {
    ChargePumpOn = 0x14,
    HorizontalAddressMode = 0x00,
    VerticalAddressMode = 0x01,
}

pub struct OLED<'a, I> {
    i2c: &'a RefCell<I>,
}

impl<'a, I: embedded_hal::i2c::I2c> OLED<'a, I> {
    fn send_command(
        &mut self,
        command: OLEDCommands,
        values: &[u8],
    ) -> Result<(), <I as ErrorType>::Error> {
        let mut buf = [0u8; 8];
        buf[0] = OLEDControlBytes::CommandFlow as u8;
        buf[1] = command as u8;
        let len = 2 + values.len();
        buf[2..len].copy_from_slice(values);
        self.i2c.borrow_mut().write(OLED_ADDRESS, &buf)
    }

    fn send_data(&mut self, values: &[u8]) -> Result<(), <I as ErrorType>::Error> {
        let mut buf = [0u8; 8];
        buf[0] = OLEDControlBytes::DataFlow as u8;
        let len = 1 + values.len();
        buf[1..len].copy_from_slice(values);
        self.i2c.borrow_mut().write(OLED_ADDRESS, &buf)
    }

    pub fn new(i2c: &'a RefCell<I>) -> Self {
        // Turn on charge pump
        let mut oled = Self { i2c };

        oled.send_command(OLEDCommands::DisplayOff, &[]).unwrap();
        oled.send_command(
            OLEDCommands::ChargePumpSelect,
            &[OLEDValues::ChargePumpOn as u8],
        )
        .unwrap();
        oled.send_command(
            OLEDCommands::AddressMode,
            &[OLEDValues::HorizontalAddressMode as u8],
        )
        .unwrap();
        oled.send_command(OLEDCommands::ColumnRange, &[0, 127])
            .unwrap();
        oled.send_command(OLEDCommands::PageRange, &[0, 7]).unwrap();
        oled.send_command(OLEDCommands::DisplayOn, &[]).unwrap();
        oled
    }
}
