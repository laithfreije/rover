use core::cell::RefCell;

use embedded_graphics::{
    mono_font::{MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal_bus::i2c::RefCellDevice;
use ibm437::IBM437_8X8_REGULAR;
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

type OLEDDisplay<'a, I> = Ssd1306<
    I2CInterface<RefCellDevice<'a, I>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

pub struct OLED<'a, I> {
    display: OLEDDisplay<'a, I>,
    text_style: MonoTextStyle<'a, BinaryColor>,
}

impl<'a, I: embedded_hal::i2c::I2c> OLED<'a, I> {
    pub fn new(i2c: &'a RefCell<I>) -> Self {
        let interface = I2CDisplayInterface::new(RefCellDevice::new(i2c));

        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&IBM437_8X8_REGULAR)
            .text_color(BinaryColor::On)
            .build();

        display.flush().unwrap();

        Self {
            display,
            text_style,
        }
    }

    pub fn write_text(&mut self, text: &str, x: i32, y: i32) {
        Text::with_baseline(text, Point::new(x, y), self.text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        self.display.flush().unwrap();
    }

    /// Blank the whole panel. Drawing is additive (the buffered mode never
    /// erases stale pixels on its own), so call this before repainting a
    /// shorter string over a longer one.
    pub fn clear(&mut self) {
        self.display.clear(BinaryColor::Off).unwrap();
        self.display.flush().unwrap();
    }
}
