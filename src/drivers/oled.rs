use core::cell::RefCell;

use embedded_graphics::{
    mono_font::{MonoTextStyle, MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal_bus::i2c::RefCellDevice;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

type OLEDDisplay<'a, I> = Ssd1306<
    I2CInterface<RefCellDevice<'a, I>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

pub struct OLED<'a, I> {
    display: OLEDDisplay<'a, I>,
    text_style: MonoTextStyle<'a, BinaryColor>
}

impl<'a, I: embedded_hal::i2c::I2c> OLED<'a, I> {
    pub fn new(i2c: &'a RefCell<I>) -> Self {
        let interface = I2CDisplayInterface::new(RefCellDevice::new(i2c));
        
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        display.flush().unwrap();

        Self { display, text_style }
    }

    pub fn write_text(&mut self, text: &'a str, position: Point)
    {
        Text::with_baseline(text, position, self.text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        self.display.flush().unwrap();
    }
    
}
