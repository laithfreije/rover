use core::cell::RefCell;

use embedded_graphics::{
    mono_font::{MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
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

const ROW_SIZE: i32 = 8;

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
        self.write_text_no_flush(text, x, y);
        self.display.flush().unwrap();
    }

    /// Draw text without pushing the framebuffer to the panel. Pair several of
    /// these (and [`Self::clear_row_no_flush`]) with a single [`Self::flush`] to
    /// repaint multiple rows for the cost of one I2C transfer — the per-call
    /// flush in [`Self::write_text`] is expensive enough to stall time-sensitive
    /// work (e.g. the BLE runner) when done many times per second.
    pub fn write_text_no_flush(&mut self, text: &str, x: i32, y: i32) {
        Text::with_baseline(
            text,
            Point::new(x * ROW_SIZE, y * ROW_SIZE),
            self.text_style,
            Baseline::Top,
        )
        .draw(&mut self.display)
        .unwrap();
    }

    /// Push the framebuffer to the panel. Use after a batch of `*_no_flush`
    /// draws.
    pub fn flush(&mut self) {
        self.display.flush().unwrap();
    }

    /// Blank a single text row by painting it black from x=0 across the full
    /// panel width, starting at `y`. Like [`Self::clear`] this works around the
    /// additive buffered mode, but leaves the other rows intact so a fresh
    /// string can replace just one line (e.g. overwrite "Connecting" with the
    /// IP while a status row above it persists).
    pub fn clear_row(&mut self, y: i32) {
        self.clear_row_no_flush(y);
        self.display.flush().unwrap();
    }

    /// Like [`Self::clear_row`] but without flushing; see
    /// [`Self::write_text_no_flush`].
    pub fn clear_row_no_flush(&mut self, y: i32) {
        let height = IBM437_8X8_REGULAR.character_size.height;
        Rectangle::new(
            Point::new(0, y * ROW_SIZE),
            Size::new(self.display.size().width, height),
        )
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(&mut self.display)
        .unwrap();
    }

    /// Blank the whole panel. Drawing is additive (the buffered mode never
    /// erases stale pixels on its own), so call this before repainting a
    /// shorter string over a longer one.
    pub fn clear(&mut self) {
        self.display.clear(BinaryColor::Off).unwrap();
        self.display.flush().unwrap();
    }
}
