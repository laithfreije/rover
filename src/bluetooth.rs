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

use core::cell::RefCell;
use core::fmt::Write as _;

use cyw43::{aligned_bytes, Cyw43439};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma, Peri};
use embassy_time::{Duration, Timer};
use heapless::{Deque, String};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::SharedOled;

/// OLED row (8px units) where Bluetooth status is drawn. Mirrors the row the
/// Wi-Fi build used for its IP/connection line.
const BT_STATUS_ROW: i32 = 2;

/// OLED row reserved for a found Xbox controller (its MAC).
const XBOX_ROW: i32 = 3;

/// OLED rows used to list other named BLE devices (one per line, wrapping).
const DEVICE_ROW_FIRST: i32 = 4;
const DEVICE_ROW_LAST: i32 = 7;

/// Max distinct devices remembered for de-duplication while scanning.
const SEEN_MAX: usize = 32;

/// OLED width in 8x8 characters (128px / 8).
const OLED_COLS: usize = 16;

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
    let central = stack.central();

    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row(BT_STATUS_ROW);
        o.write_text("BT: scanning", 0, BT_STATUS_ROW);
    });

    // Active scan so peripherals send scan responses (which usually carry the
    // device name). Discovered devices are routed to `DeviceList`, which prints
    // each new one to the OLED. `run_with_handler` drives the host stack and
    // delivers advertising reports to the handler; it must run for scanning to
    // make progress.
    let devices = DeviceList::new(oled);
    let mut scanner = Scanner::new(central);
    let _ = join(runner.run_with_handler(&devices), async {
        let mut config = ScanConfig::default();
        config.active = true;
        config.phys = PhySet::M1;
        config.interval = Duration::from_secs(1);
        config.window = Duration::from_secs(1);
        let _session = scanner.scan(&config).await.unwrap();
        // Keep the scan session alive; the handler does the work.
        loop {
            Timer::after_secs(1).await;
        }
    })
    .await;

    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row(BT_STATUS_ROW);
        o.write_text("BT: runner died", 0, BT_STATUS_ROW);
    });
    loop {
        Timer::after_secs(1).await;
    }
}

/// Scan-result sink. Only devices that advertise a local name are shown (most
/// of the BLE noise around us advertises no name, and showing every MAC floods
/// the panel). A device named like an Xbox controller is called out on its own
/// fixed rows; other named devices cycle through the remaining rows.
///
/// De-duplication keys on "already displayed with a name" rather than "address
/// seen", so a device's scan response (which carries the name) still gets shown
/// even though its earlier, nameless `ADV_IND` was seen first.
struct DeviceList {
    oled: &'static SharedOled,
    inner: RefCell<DeviceListInner>,
}

struct DeviceListInner {
    named: Deque<BdAddr, SEEN_MAX>,
    next_row: i32,
    xbox_shown: bool,
}

impl DeviceList {
    fn new(oled: &'static SharedOled) -> Self {
        Self {
            oled,
            inner: RefCell::new(DeviceListInner {
                named: Deque::new(),
                next_row: DEVICE_ROW_FIRST,
                xbox_shown: false,
            }),
        }
    }
}

/// Case-insensitive substring test for "xbox" in an advertised name.
fn is_xbox_name(name: &str) -> bool {
    let needle = b"xbox";
    name.as_bytes()
        .windows(needle.len())
        .any(|w| w.eq_ignore_ascii_case(needle))
}

impl EventHandler for DeviceList {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut inner = self.inner.borrow_mut();
        while let Some(Ok(report)) = it.next() {
            let addr = report.addr;

            // Extract the advertised local name, if any. Skip nameless reports.
            let mut name: String<OLED_COLS> = String::new();
            let mut has_name = false;
            for ad in AdStructure::decode(report.data) {
                let bytes = match ad {
                    Ok(AdStructure::CompleteLocalName(n)) | Ok(AdStructure::ShortenedLocalName(n)) => n,
                    _ => continue,
                };
                has_name = true;
                if let Ok(s) = core::str::from_utf8(bytes) {
                    for c in s.chars() {
                        if name.push(c).is_err() {
                            break;
                        }
                    }
                }
                break;
            }
            if !has_name {
                continue;
            }

            // Only show each named device once.
            if inner.named.iter().any(|b| b.raw() == addr.raw()) {
                continue;
            }
            if inner.named.is_full() {
                inner.named.pop_front();
            }
            let _ = inner.named.push_back(addr);

            if is_xbox_name(&name) && !inner.xbox_shown {
                inner.xbox_shown = true;
                let m = addr.raw();
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
                continue;
            }

            // Other named device: print on the next cycling row.
            let row = inner.next_row;
            inner.next_row = if row >= DEVICE_ROW_LAST {
                DEVICE_ROW_FIRST
            } else {
                row + 1
            };
            self.oled.lock(|o| {
                let mut o = o.borrow_mut();
                o.clear_row(row);
                o.write_text(&name, 0, row);
            });
        }
    }
}
