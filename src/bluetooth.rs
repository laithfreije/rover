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

use cyw43::{aligned_bytes, Control, Cyw43439};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select3;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, FLASH, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use heapless::String;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::xbox::XboxReport;
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

/// Total RP2040/Pico W flash size.
const FLASH_SIZE: usize = 2 * 1024 * 1024;
/// Bytes reserved at the end of flash for bond storage. Must be a multiple of
/// the 4K erase size; sequential-storage needs at least two pages. Kept in sync
/// with the region carved out of the code area in `memory.x`.
const STORAGE_LEN: u32 = 8 * 1024;
/// Flash offset (from the start of flash) of the bond-storage region.
const STORAGE_OFFSET: u32 = FLASH_SIZE as u32 - STORAGE_LEN;

/// HID-over-GATT UUIDs: the HID service and its Report characteristic.
const HID_SERVICE_UUID: u16 = 0x1812;
const HID_REPORT_UUID: u16 = 0x2a4d;

/// OLED row where raw HID report bytes are drawn.
const HID_BYTES_ROW: i32 = 4;

/// Minimum gap between OLED redraws of the HID report. A gamepad notifies far
/// faster than the (whole-framebuffer-flushing, blocking-I2C) panel can
/// repaint, and a long flush stalls the BLE runner — so we drain every
/// notification promptly but only repaint a few times a second.
const REPORT_DRAW_MS: u64 = 200;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// Background task driving the cyw43 SPI runner. Must run for both Wi-Fi and
/// Bluetooth traffic to make progress. With the `bluetooth` feature enabled the
/// runner carries a third type parameter (`Cyw43439`) versus the Wi-Fi-only
/// build.
/// Max simultaneous subscribers to [`CONTROLLER`].
const CONTROLLER_RECEIVERS: usize = 4;

/// Latest decoded controller state, published by the HID stream at full report
/// rate. Application tasks subscribe with `CONTROLLER.receiver()` and await
/// `changed()` to react to input. Latest-wins: a slow consumer simply sees the
/// most recent state rather than a backlog.
pub static CONTROLLER: Watch<CriticalSectionRawMutex, XboxReport, CONTROLLER_RECEIVERS> = Watch::new();

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>, Cyw43439>,
) -> ! {
    runner.run().await
}

/// Demo consumer of [`CONTROLLER`]: mirror the A button on the Pico W's onboard
/// LED (cyw43 GPIO 0). Shows how application code reacts to controller input
/// from its own task; replace with real logic.
#[embassy_executor::task]
async fn controller_led_task(mut control: Control<'static>) {
    let Some(mut rx) = CONTROLLER.receiver() else {
        return;
    };
    let mut led = false;
    loop {
        let report = rx.changed().await;
        if report.a != led {
            led = report.a;
            control.gpio_set(0, led).await;
        }
    }
}

/// Persisted bond record. `BondInformation` (peer identity + long-term key) is
/// `Serialize`/`Deserialize` under trouble-host's `serde` feature; the newtype
/// wrapper lets us implement sequential-storage's `PostcardValue` (orphan rule).
#[derive(Serialize, Deserialize)]
struct StoredBond(BondInformation);
impl<'a> PostcardValue<'a> for StoredBond {}

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
    flash_peri: Peri<'static, FLASH>,
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

    // Hand the cyw43 control to a demo task that reacts to controller input.
    spawner.spawn(controller_led_task(control).unwrap());

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

    // Open bond storage in the reserved flash region and load any saved bond.
    // Adding it to the stack lets a later reconnect resume encryption without
    // re-pairing (the controller keeps its side of the bond across reboots too).
    let flash = Flash::<_, Blocking, FLASH_SIZE>::new_blocking(flash_peri);
    let mut flash = BlockingAsync::new(flash);
    let mut bonds = MapStorage::<(), _, _>::new(
        &mut flash,
        MapConfig::new(STORAGE_OFFSET..(STORAGE_OFFSET + STORAGE_LEN)),
        NoCache::new(),
    );
    let mut bond_buf = [0u8; 128];
    let mut has_bond = match bonds.fetch_item(&mut bond_buf, &()).await {
        Ok(Some(StoredBond(bond))) => {
            let _ = stack.add_bond_information(bond);
            true
        }
        _ => false,
    };

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

        // Reconnect loop: a controller will drop the link (idle, range, power),
        // so connect/bond/stream is retried indefinitely. Within a session the
        // bond stays in the stack's RAM, so reconnects resume encryption without
        // re-pairing.
        let mut central = scanner.into_inner();
        let connect_config = ConnectConfig {
            connect_params: Default::default(),
            scan_config: ScanConfig {
                active: true,
                filter_accept_list: core::slice::from_ref(&target),
                timeout: Duration::from_secs(DISCOVER_TIMEOUT_SECS),
                ..Default::default()
            },
        };

        loop {
            set_status(oled, "CONNECTING");
            let conn = match central.connect(&connect_config).await {
                Ok(c) => c,
                Err(_) => {
                    set_status(oled, "CONNECT ERR");
                    Timer::after_secs(1).await;
                    continue;
                }
            };
            set_status(oled, "CONNECTED");

            // Establish an encrypted link. With no stored bond this performs the
            // Just Works pairing/bond (the Xbox controller won't serve HID
            // reports until encrypted) and saves it; with a stored bond we don't
            // re-bond and the link just resumes encryption.
            let _ = conn.set_bondable(!has_bond);
            if conn.request_security().is_err() {
                set_status(oled, "SEC REQ ERR");
                Timer::after_secs(1).await;
                continue;
            }
            set_status(oled, "PAIRING...");
            let secured = loop {
                match conn.next().await {
                    ConnectionEvent::PairingComplete { bond, .. } => {
                        // Keep the bond in the stack and persist it to flash so
                        // reconnects (incl. across reboots) skip pairing.
                        if let Some(b) = bond {
                            let _ = stack.add_bond_information(b.clone());
                            if bonds.store_item(&mut bond_buf, &(), &StoredBond(b)).await.is_ok() {
                                has_bond = true;
                            }
                        }
                        break true;
                    }
                    ConnectionEvent::Encrypted { .. } => break true,
                    ConnectionEvent::PairingFailed(_) => {
                        set_status(oled, "PAIR FAIL");
                        // The peer may have forgotten us (e.g. re-paired
                        // elsewhere); allow a fresh bond on the next attempt.
                        has_bond = false;
                        break false;
                    }
                    ConnectionEvent::Disconnected { .. } => break false,
                    ConnectionEvent::RequestConnectionParams(req) => {
                        let _ = req.accept(None, &stack).await;
                    }
                    _ => {}
                }
            };
            if !secured {
                Timer::after_secs(1).await;
                continue;
            }
            set_status(oled, "BONDED");

            // --- HID: discover the HID service, enable notifications on its
            // input report characteristics, and dump raw report bytes. Three
            // tasks run concurrently until the link drops: the GATT client task,
            // the report listener/drawer, and a connection-event pump that
            // accepts the controller's parameter-update requests (not doing so
            // can get the link torn down) and detects disconnect. ---
            let client = match GattClient::<_, DefaultPacketPool, 10>::new(&stack, &conn).await {
                Ok(c) => c,
                Err(_) => {
                    set_status(oled, "GATT ERR");
                    Timer::after_secs(1).await;
                    continue;
                }
            };
            let events = async {
                loop {
                    match conn.next().await {
                        ConnectionEvent::Disconnected { .. } => break,
                        ConnectionEvent::RequestConnectionParams(req) => {
                            let _ = req.accept(None, &stack).await;
                        }
                        _ => {}
                    }
                }
            };
            // Returns as soon as any branch ends — i.e. when the link drops.
            select3(client.task(), hid_dump(&client, oled), events).await;
            // Push a neutral state so consumers (motors!) stop immediately,
            // rather than waiting for their own failsafe timeout.
            CONTROLLER.sender().send(XboxReport::neutral());
            set_status(oled, "RECONNECTING");
            Timer::after_millis(500).await;
        }
    })
    .await;

    set_status(oled, "BT: runner died");
    loop {
        Timer::after_secs(1).await;
    }
}

/// Discover the controller's HID service, enable notifications on each input
/// report characteristic, then decode each report and show the controller
/// state on the OLED. Returns when the GATT link ends (e.g. the controller
/// disconnects).
async fn hid_dump<C: Controller>(
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
    oled: &'static SharedOled,
) {
    let services = match client.services_by_uuid(&Uuid::new_short(HID_SERVICE_UUID)).await {
        Ok(s) => s,
        Err(_) => return set_status(oled, "NO HID SVC"),
    };
    let service = match services.first() {
        Some(s) => s.clone(),
        None => return set_status(oled, "NO HID SVC"),
    };

    let chars = match client.characteristics::<16>(&service).await {
        Ok(c) => c,
        Err(_) => return set_status(oled, "CHAR ERR"),
    };

    // Enable notifications on every notifiable Report characteristic. Writing
    // the CCCD here is also the signal the controller waits for before it
    // treats us as an attached host (its Xbox light goes solid). The listener
    // returned by `subscribe` is dropped; `listen_all` below receives the
    // notifications for all of them.
    let mut subs = 0u8;
    for c in chars.iter() {
        if c.uuid == Uuid::new_short(HID_REPORT_UUID) && c.props.has_cccd() && client.subscribe(c, false).await.is_ok() {
            subs += 1;
        }
    }
    if subs == 0 {
        return set_status(oled, "NO HID RPT");
    }
    set_status(oled, "HID LIVE");

    let mut listener = match client.listen_all() {
        Ok(l) => l,
        Err(_) => return set_status(oled, "LISTEN ERR"),
    };

    let tx = CONTROLLER.sender();
    let mut last = Instant::now();
    loop {
        let n = listener.next().await;
        // Only the gamepad report decodes; ignore other notifications (battery,
        // consumer control, ...).
        let Some(report) = XboxReport::parse(n.as_ref()) else {
            continue;
        };
        // Publish every report so subscribers get the full rate; the OLED
        // (below) is the only throttled consumer.
        tx.send(report);

        let now = Instant::now();
        if now.duration_since(last) < Duration::from_millis(REPORT_DRAW_MS) {
            continue;
        }
        last = now;
        draw_report(oled, &report);
    }
}

/// Append "<tok> " to `s` when `pressed`, ignoring overflow (the line is
/// truncated rather than erroring when many buttons are held).
fn push_btn(s: &mut String<OLED_COLS>, pressed: bool, tok: &str) {
    if pressed {
        let _ = s.push_str(tok);
        let _ = s.push(' ');
    }
}

/// Paint a decoded report across rows 4..7 with a single flush: sticks as
/// signed percentages, triggers as 0..100, the D-pad direction, and the
/// currently-held buttons.
fn draw_report(oled: &'static SharedOled, r: &XboxReport) {
    let mut sticks_l: String<OLED_COLS> = String::new();
    let _ = write!(sticks_l, "L{:+04},{:+04}", r.lx_pct(), r.ly_pct());

    let mut sticks_r: String<OLED_COLS> = String::new();
    let _ = write!(sticks_r, "R{:+04},{:+04}", r.rx_pct(), r.ry_pct());

    let mut trig: String<OLED_COLS> = String::new();
    let _ = write!(trig, "LT{} RT{} {}", r.lt_pct(), r.rt_pct(), r.dpad.label());

    let mut btns: String<OLED_COLS> = String::new();
    push_btn(&mut btns, r.a, "A");
    push_btn(&mut btns, r.b, "B");
    push_btn(&mut btns, r.x, "X");
    push_btn(&mut btns, r.y, "Y");
    push_btn(&mut btns, r.lb, "LB");
    push_btn(&mut btns, r.rb, "RB");
    push_btn(&mut btns, r.menu, "Mn");
    push_btn(&mut btns, r.view, "Vw");
    push_btn(&mut btns, r.xbox, "Xb");
    push_btn(&mut btns, r.ls, "LS");
    push_btn(&mut btns, r.rs, "RS");
    push_btn(&mut btns, r.share, "Sh");

    oled.lock(|o| {
        let mut o = o.borrow_mut();
        o.clear_row_no_flush(HID_BYTES_ROW);
        o.write_text_no_flush(&sticks_l, 0, HID_BYTES_ROW);
        o.clear_row_no_flush(HID_BYTES_ROW + 1);
        o.write_text_no_flush(&sticks_r, 0, HID_BYTES_ROW + 1);
        o.clear_row_no_flush(HID_BYTES_ROW + 2);
        o.write_text_no_flush(&trig, 0, HID_BYTES_ROW + 2);
        o.clear_row_no_flush(HID_BYTES_ROW + 3);
        o.write_text_no_flush(&btns, 0, HID_BYTES_ROW + 3);
        o.flush();
    });
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

/// AD type for "Appearance" (Core Spec supplement). Not modelled by
/// `AdStructure`, so it surfaces as `Unknown`.
const AD_TYPE_APPEARANCE: u8 = 0x19;
/// BLE Appearance value for a HID Gamepad.
const APPEARANCE_GAMEPAD: u16 = 0x03c4;

/// Should we treat this advertiser as a controller to connect to?
///
/// Matches either a local name containing "xbox" *or* an advertised Gamepad
/// appearance. The appearance check matters because once a controller has
/// bonded it often re-advertises for reconnection without its name; keying on
/// the Gamepad appearance still recognises it without matching every BLE HID
/// device (keyboards, mice, ...).
fn adv_is_xbox(data: &[u8]) -> bool {
    for ad in AdStructure::decode(data) {
        match ad {
            Ok(AdStructure::CompleteLocalName(n)) | Ok(AdStructure::ShortenedLocalName(n)) => {
                if let Ok(s) = core::str::from_utf8(n) {
                    if is_xbox_name(s) {
                        return true;
                    }
                }
            }
            Ok(AdStructure::Unknown {
                ty: AD_TYPE_APPEARANCE,
                data,
            }) if data.len() >= 2 => {
                if u16::from_le_bytes([data[0], data[1]]) == APPEARANCE_GAMEPAD {
                    return true;
                }
            }
            _ => {}
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
