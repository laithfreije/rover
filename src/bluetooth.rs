//! Bluetooth (cyw43 BLE via trouble-host) — Xbox controller milestone.
//!
//! Brings the cyw43 chip up as a BLE *central* so the MCU can scan for, bond
//! with, and read HID reports from an Xbox Wireless Controller (which presents
//! as a HID-over-GATT peripheral over BLE).
//!
//! This commit performs the bring-up: it powers the cyw43 over PIO-SPI with
//! Bluetooth firmware loaded, hands its HCI controller to trouble-host, builds
//! the BLE host stack, and runs the host runner. Scanning/bonding/HID land in
//! following commits.

use cyw43::{aligned_bytes, Cyw43439};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma, Peri};
use embassy_time::Timer;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::SharedOled;

/// OLED row (8px units) where Bluetooth status is drawn. Mirrors the row the
/// Wi-Fi build used for its IP/connection line.
const BT_STATUS_ROW: i32 = 2;

/// trouble-host resource sizing. Only the controller connects, so one
/// connection slot suffices; the L2CAP channels cover signalling + ATT plus a
/// little headroom.
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// Background task driving the cyw43 SPI runner. Must run for both Wi-Fi and
/// Bluetooth traffic to make progress. With the `bluetooth` feature enabled the
/// runner carries a third type parameter (`Cyw43439`) versus the Wi-Fi-only
/// build.
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>, Cyw43439>,
) -> ! {
    runner.run().await
}

/// Bring up the cyw43 Bluetooth controller and run the trouble-host BLE stack.
/// Never returns.
pub async fn run(
    spawner: Spawner,
    pwr_pin: Peri<'static, PIN_23>,
    dio_pin: Peri<'static, PIN_24>,
    cs_pin: Peri<'static, PIN_25>,
    clk_pin: Peri<'static, PIN_29>,
    pio0: Peri<'static, PIO0>,
    dma0: Peri<'static, DMA_CH0>,
    dma1: Peri<'static, DMA_CH1>,
    oled: &'static SharedOled,
) -> ! {
    let fw = aligned_bytes!("blobs/43439A0.bin");
    let clm = aligned_bytes!("blobs/43439A0_clm.bin");
    let btfw = aligned_bytes!("blobs/43439A0_btfw.bin");
    let nvram = aligned_bytes!("blobs/nvram_rp2040.bin");

    let pwr = Output::new(pwr_pin, Level::Low);
    let cs = Output::new(cs_pin, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    // The Bluetooth-capable cyw43-pio takes two DMA channels (TX/RX) versus the
    // single channel the Wi-Fi-only build used.
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        dio_pin,
        clk_pin,
        dma::Channel::new(dma0, Irqs),
        dma::Channel::new(dma1, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, bt_device, mut control, cyw43_runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await;
    spawner.spawn(cyw43_task(cyw43_runner).unwrap());
    control.init(clm).await;

    // Hand the chip's HCI interface to trouble-host as an external controller.
    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    // Fixed random BLE address for the MCU acting as central. A shipping
    // product would derive this from a unique per-chip value; a constant is
    // fine for bring-up.
    let address = Address::random([0xff, 0x8f, 0x28, 0x05, 0xe4, 0xff]);

    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut runner = stack.runner();

    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row(BT_STATUS_ROW);
        o.write_text("BT: up", 0, BT_STATUS_ROW);
    });

    // Drive the host stack. `runner.run()` performs HCI reset/init and processes
    // controller events; nothing else makes progress without it. It only
    // returns on a fatal error, which we surface before idling.
    let _ = runner.run().await;

    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row(BT_STATUS_ROW);
        o.write_text("BT: runner died", 0, BT_STATUS_ROW);
    });
    loop {
        Timer::after_secs(1).await;
    }
}
