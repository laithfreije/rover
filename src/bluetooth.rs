//! Bluetooth (cyw43 BLE via trouble-host) — Xbox controller milestone.
//!
//! Brings the cyw43 chip up as a BLE *central* so the MCU can scan for, bond
//! with, and read HID reports from an Xbox Wireless Controller (which presents
//! as a HID-over-GATT peripheral over BLE).
//!
//! This commit discovers a controller by *name* (any device advertising
//! "xbox", so any Xbox controller works — no hardcoded address), connects to
//! it, and performs the BLE pairing/bond. An Xbox controller won't deliver HID
//! reports until the link is encrypted, so bonding is a prerequisite for the
//! report-reading step that follows.

use core::cell::Cell;
use core::fmt::Write as _;

use cyw43::{aligned_bytes, Cyw43439};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Timer};
use heapless::String;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::SharedOled;

/// OLED row (8px units) where the Bluetooth connection state is drawn.
const BT_STATUS_ROW: i32 = 2;
/// OLED row showing the discovered controller's MAC.
const XBOX_ROW: i32 = 3;

/// OLED width in 8x8 characters (128px / 8).
const OLED_COLS: usize = 16;

/// trouble-host resource sizing. Only the controller connects, so one
/// connection slot suffices; the L2CAP channels cover signalling + ATT plus a
/// little headroom.
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;

/// How long to hunt for a controller before giving up (it must be in pairing
/// mode and advertising).
const DISCOVER_TIMEOUT_SECS: u64 = 30;

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

/// Bring up the cyw43 Bluetooth controller, discover an Xbox controller by
/// name, connect, and bond. Never returns.
#[allow(clippy::too_many_arguments)]
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

    // Address of a discovered controller, handed from the (synchronous) scan
    // event handler to the async connect logic below.
    let found: Signal<CriticalSectionRawMutex, Address> = Signal::new();
    let finder = XboxFinder::new(oled, &found);

    set_status(oled, "BT: finding");

    // `run_with_handler` drives the host stack (including the one-time security
    // RNG seeding via the controller's LE_Rand) and routes advertising reports
    // to `finder`. It must run for scanning, connecting and pairing to make
    // progress, so everything else happens in the joined async block.
    let _ = join(runner.run_with_handler(&finder), async {
        // --- Discover: active scan until a device named "xbox" shows up. ---
        let central = stack.central();
        let mut scanner = Scanner::new(central);
        let target = {
            let mut config = ScanConfig::default();
            config.active = true;
            config.phys = PhySet::M1;
            config.interval = Duration::from_secs(1);
            config.window = Duration::from_secs(1);
            let _session = match scanner.scan(&config).await {
                Ok(s) => s,
                Err(_) => {
                    set_status(oled, "BT: scan err");
                    return;
                }
            };
            match with_timeout(Duration::from_secs(DISCOVER_TIMEOUT_SECS), found.wait()).await {
                Ok(addr) => addr,
                Err(_) => {
                    set_status(oled, "BT: no xbox");
                    return;
                }
            }
            // `_session` dropped here -> scanning stops, freeing the scanner.
        };

        // --- Connect: hand the scanner's central back and dial the target. ---
        let mut central = scanner.into_inner();
        set_status(oled, "CONNECTING");
        let connect_config = ConnectConfig {
            connect_params: Default::default(),
            scan_config: ScanConfig {
                active: true,
                filter_accept_list: core::slice::from_ref(&target),
                timeout: Duration::from_secs(DISCOVER_TIMEOUT_SECS),
                ..Default::default()
            },
        };
        let conn = match central.connect(&connect_config).await {
            Ok(c) => c,
            Err(_) => {
                set_status(oled, "CONNECT ERR");
                return;
            }
        };
        set_status(oled, "CONNECTED");

        // --- Bond: request an encrypted link. The Xbox controller pairs with
        // Just Works (no passkey); the controller won't serve HID reports until
        // the link is encrypted. Keep pumping events so the connection stays
        // alive after bonding. ---
        let _ = conn.set_bondable(true);
        if conn.request_security().is_err() {
            set_status(oled, "SEC REQ ERR");
            return;
        }
        set_status(oled, "PAIRING...");
        loop {
            match conn.next().await {
                ConnectionEvent::PairingComplete { .. } => set_status(oled, "BONDED"),
                ConnectionEvent::PairingFailed(_) => {
                    set_status(oled, "PAIR FAIL");
                    break;
                }
                ConnectionEvent::Disconnected { .. } => {
                    set_status(oled, "DISCONNECTED");
                    break;
                }
                ConnectionEvent::RequestConnectionParams(req) => {
                    let _ = req.accept(None, &stack).await;
                }
                _ => {}
            }
        }
    })
    .await;

    set_status(oled, "BT: runner died");
    loop {
        Timer::after_secs(1).await;
    }
}

/// Write a short status line on [`BT_STATUS_ROW`], clearing the row first.
fn set_status(oled: &'static SharedOled, msg: &str) {
    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row(BT_STATUS_ROW);
        o.write_text(msg, 0, BT_STATUS_ROW);
    });
}

/// Case-insensitive substring test for "xbox" in an advertised name.
fn is_xbox_name(name: &str) -> bool {
    let needle = b"xbox";
    name.as_bytes()
        .windows(needle.len())
        .any(|w| w.eq_ignore_ascii_case(needle))
}

/// Does this advertising payload carry a local name containing "xbox"?
fn adv_is_xbox(data: &[u8]) -> bool {
    for ad in AdStructure::decode(data) {
        let bytes = match ad {
            Ok(AdStructure::CompleteLocalName(n)) | Ok(AdStructure::ShortenedLocalName(n)) => n,
            _ => continue,
        };
        if let Ok(s) = core::str::from_utf8(bytes) {
            if is_xbox_name(s) {
                return true;
            }
        }
    }
    false
}

/// Scan-result sink that watches for any Xbox controller (matched by advertised
/// name, so any unit works) and hands its address — with the correct address
/// kind — to the connect logic via a [`Signal`].
struct XboxFinder<'a> {
    oled: &'static SharedOled,
    found: &'a Signal<CriticalSectionRawMutex, Address>,
    done: Cell<bool>,
}

impl<'a> XboxFinder<'a> {
    fn new(oled: &'static SharedOled, found: &'a Signal<CriticalSectionRawMutex, Address>) -> Self {
        Self {
            oled,
            found,
            done: Cell::new(false),
        }
    }
}

impl EventHandler for XboxFinder<'_> {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        if self.done.get() {
            return;
        }
        while let Some(Ok(report)) = it.next() {
            if !adv_is_xbox(report.data) {
                continue;
            }
            self.done.set(true);

            let m = report.addr.raw();
            let mut mac: String<OLED_COLS> = String::new();
            let _ = write!(
                mac,
                "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                m[5], m[4], m[3], m[2], m[1], m[0]
            );
            self.oled.lock(|o| {
                let mut o = o.borrow_mut();
                o.clear_row(BT_STATUS_ROW);
                o.write_text("FOUND XBOX", 0, BT_STATUS_ROW);
                o.clear_row(XBOX_ROW);
                o.write_text(&mac, 0, XBOX_ROW);
            });

            // Preserve the advertised address *kind* (public vs random) so the
            // connect filter matches regardless of controller model.
            self.found
                .signal(Address::new(report.addr_kind, report.addr));
            return;
        }
    }
}
