//! CYW43 (Pico W) wireless bring-up.
//!
//! Powers up the cyw43 chip over PIO-SPI, joins the Wi-Fi network whose
//! credentials are baked in at compile time by `build.rs` from
//! `wifi.toml`, brings up the embassy-net stack, and waits for a DHCP
//! lease. [`init`] returns a short string describing the result (the
//! acquired IPv4 address, or a failure reason) that the caller renders
//! on the OLED.

use core::fmt::Write as _;

use cyw43::{Control, JoinOptions, aligned_bytes};
use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_net::{Config, StackResources};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::Timer;
use fixed::FixedU32;
use fixed::types::extra::U8;
use heapless::String;
use static_cell::StaticCell;

use crate::SharedOled;

// `WIFI_NETWORK` / `WIFI_PASSWORD` are generated from `wifi.toml` by `build.rs`.
include!(concat!(env!("OUT_DIR"), "/wifi.rs"));

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

/// Hand-tuned PIO clock divider for the cyw43 GSPI link. The RP2040
/// board this firmware targets is not reliable below divider 6
/// (~10 MHz GSPI), so the cyw43 driver's default is overridden here.
const CLOCK_DIVIDER: FixedU32<U8> = FixedU32::from_bits(0x0600);

/// `smoltcp` per-stack resource pool size. Two sockets cover DHCP + DNS,
/// which is all this firmware needs to obtain and display an address.
const STACK_SOCKET_COUNT: usize = 2;

/// Fixed PRNG seed for the network stack. embassy-net only uses it to
/// randomise TCP initial sequence numbers; this firmware opens no TCP
/// sockets, so a constant seed is fine and avoids pulling in an RNG.
const NET_SEED: u64 = 0x0123_4567_89ab_cdef;

/// OLED row (y, pixels) where the live RSSI reading is drawn. Sits below
/// the reset reason (y=0), VREG status (y=8), and IP/connection (y=16).
const RSSI_ROW: i32 = 3;

/// How often the RSSI reading is refreshed.
const RSSI_POLL_SECS: u64 = 2;

/// Background task that drives the cyw43 SPI runner. Must run for the
/// network stack to make progress.
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

/// Background task that drives the embassy-net stack (IP, DHCP, ...).
#[embassy_executor::task]
async fn network_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    
    runner.run().await
}


/// Background task that polls the cyw43 link RSSI and draws it on the OLED.
/// Owns `control` outright (nothing else needs it once the link is up), so
/// no locking is required to call its `&mut self` methods.
#[embassy_executor::task]
async fn rssi_task(mut control: Control<'static>, oled: &'static SharedOled) -> ! {
    loop {
        // Flip to positive to get magnitude
        let rssi = -control.get_rssi().await;

        // Write actual RSSI power
        let mut line: String<32> = String::new();
        let _ = write!(line, "{} dBm", -rssi);

        // Categorize RSSI        
        if rssi <= 50
        {
            let _ = write!(line, " ({})", "Strong");
        } else if (rssi > 50) && (rssi <= 70) {
            let _ = write!(line, " ({})", "OK");
        } else if  (rssi > 70) && (rssi <= 80)
        {
            let _ = write!(line, " ({})", "Weak");
        }else if  rssi >= 90
        {
            let _ = write!(line, " ({})", "Unstable");
        }
             
        oled.lock(|o| {
            let mut display = o.borrow_mut();
            display.clear_row(RSSI_ROW);
            display.write_text(&line, 0, RSSI_ROW);
        });

        Timer::after_secs(RSSI_POLL_SECS).await;
    }
}

/// Bring up the wireless chip, join the configured network, and wait for
/// a DHCP lease. Returns a short, OLED-friendly status line: the acquired
/// IPv4 address on success, or a failure reason.
pub async fn init(
    spawner: Spawner,
    pwr_pin: Peri<'static, PIN_23>,
    dio_pin: Peri<'static, PIN_24>,
    cs_pin: Peri<'static, PIN_25>,
    clk_pin: Peri<'static, PIN_29>,
    pio_0: Peri<'static, PIO0>,
    dma_0: Peri<'static, DMA_CH0>,
    oled: &'static SharedOled
) -> String<32> {
    let fw = aligned_bytes!("blobs/43439A0.bin");
    let clm = aligned_bytes!("blobs/43439A0_clm.bin");
    let nvram = aligned_bytes!("blobs/nvram_rp2040.bin");

    let pwr = Output::new(pwr_pin, Level::Low);
    let cs = Output::new(cs_pin, Level::High);
    let mut pio = Pio::new(pio_0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        CLOCK_DIVIDER,
        pio.irq0,
        cs,
        dio_pin,
        clk_pin,
        dma::Channel::new(dma_0, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, cyw43_runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    spawner.spawn(cyw43_task(cyw43_runner).unwrap());

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::None)
        .await;

    let config = Config::dhcpv4(Default::default());

    static RESOURCES: StaticCell<StackResources<STACK_SOCKET_COUNT>> = StaticCell::new();
    let (stack, embassy_runner) =
        embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), NET_SEED);

    spawner.spawn(network_task(embassy_runner).unwrap());

    let mut status: String<32> = String::new();
    if control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
        .is_err()
    {
        let _ = status.push_str("Join failed");
        return status;
    }

    stack.wait_link_up().await;
    stack.wait_config_up().await;

    match stack.config_v4() {
        Some(cfg) => {
            let _ = write!(status, "{}", cfg.address.address());
        }
        None => {
            let _ = status.push_str("No IPv4 config");
        }
    }

    spawner.spawn(rssi_task(control, oled).unwrap());
    status
}
