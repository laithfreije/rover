//! Bluetooth (cyw43 BLE via trouble-host) — Xbox controller milestone.
//!
//! Brings the cyw43 chip up as a BLE *central* so the MCU can scan for, bond
//! with, and read HID reports from an Xbox Wireless Controller (which presents
//! as a HID-over-GATT peripheral over BLE).
//!
//! This commit only wires in the dependencies and a placeholder entry point
//! with its final signature; the cyw43 BT + trouble-host bring-up lands next.

use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::Peri;
use embassy_time::Timer;

use crate::SharedOled;

/// OLED row (8px units) where Bluetooth status is drawn. Mirrors the row the
/// Wi-Fi build used for its IP/connection line.
const BT_STATUS_ROW: i32 = 2;

/// Bluetooth entry point. Owns the cyw43 pins/peripherals and (once the
/// bring-up lands) drives the BLE central forever. Never returns.
pub async fn run(
    _spawner: Spawner,
    _pwr_pin: Peri<'static, PIN_23>,
    _dio_pin: Peri<'static, PIN_24>,
    _cs_pin: Peri<'static, PIN_25>,
    _clk_pin: Peri<'static, PIN_29>,
    _pio0: Peri<'static, PIO0>,
    _dma0: Peri<'static, DMA_CH0>,
    _dma1: Peri<'static, DMA_CH1>,
    oled: &'static SharedOled,
) -> ! {
    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row(BT_STATUS_ROW);
        o.write_text("BT: deps ok", 0, BT_STATUS_ROW);
    });
    loop {
        Timer::after_secs(1).await;
    }
}
