#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_rp::i2c::{self, I2c};
use embassy_time::Timer;
use panic_halt as _;

use crate::drivers::oled::OLED;

mod drivers;
mod wireless;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // OLED on I2C0: SDA = GPIO16, SCL = GPIO17. The bus lives in a RefCell
    // so the ssd1306 driver can share it via embedded-hal-bus.
    let i2c_bus = I2c::new_blocking(p.I2C0, p.PIN_17, p.PIN_16, i2c::Config::default());
    let refcell_i2c = RefCell::new(i2c_bus);
    let mut oled = OLED::new(&refcell_i2c);

    oled.write_text("Connecting...", 0, 0);

    // Bring up the wireless chip and join the network. Returns the IP
    // address (or a failure reason) ready to display.
    let status = wireless::init(
        spawner, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.PIO0, p.DMA_CH0,
    )
    .await;

    oled.clear();
    oled.write_text(status.as_str(), 0, 0);

    loop {
        Timer::after_secs(1).await;
    }
}
