//! SG90 micro-servo on GP2, steered by the Xbox right-stick X axis.
//!
//! A hobby servo expects a ~50 Hz pulse train (20 ms period) whose *pulse width*
//! encodes the angle: ~1.5 ms centres it, with ~0.5 ms / ~2.5 ms at the travel
//! limits. GP2 is PWM slice 1 channel A — a slice independent of the motors'
//! slice 0 (GP16/17) — so it runs at 50 Hz without disturbing the ~31 kHz motor
//! PWM.
//!
//! Wiring: servo signal -> GP2, servo V+ -> 5 V (NOT the Pico's 3V3 — the SG90
//! stalls/browns out the regulator otherwise), servo GND -> common ground with
//! the Pico.
//!
//! Fed from the shared controller state ([`crate::bluetooth::CONTROLLER`]) by an
//! embassy task, mirroring [`crate::motor`]: it reacts to each Watch update and
//! recentres on signal loss.

use embassy_rp::{
    peripherals::{PIN_2, PWM_SLICE1},
    pwm::{Config as PwmConfig, Pwm},
    Peri,
};
use embassy_time::{with_timeout, Duration};
use fixed::FixedU16;

use crate::bluetooth::CONTROLLER;
use crate::xbox::XboxReport;

/// PWM clock divider. The RP2040 PWM clock is 125 MHz; dividing by 125 yields a
/// 1 MHz counter, so one count == 1 µs and pulse widths map directly to counts.
const PWM_DIVIDER: u8 = 125;

/// Counter TOP for a 20 ms (50 Hz) period at the 1 MHz tick above. The counter
/// runs 0..=TOP, i.e. TOP + 1 ticks, so 20 000 ticks -> TOP = 19 999.
const PWM_TOP: u16 = 19_999;

/// Pulse widths (µs) for the angular extremes and centre. These give an SG90 its
/// full ~180° (−90°..+90°) travel. If the servo buzzes or strains against its
/// stops at the ends, narrow MIN/MAX toward 1000/2000.
const PULSE_MIN_US: i32 = 500; // −90°
const PULSE_CENTRE_US: i32 = 1500; // 0° (idle)
const PULSE_MAX_US: i32 = 2500; // +90°

/// Largest commanded angle, degrees. The right stick maps linearly to ±this.
const ANGLE_LIMIT_DEG: i32 = 90;

/// Right-stick deadzone (percent of full scale, 0..100) below which the servo is
/// held at centre, filtering resting jitter.
const STICK_DEADZONE_PCT: i32 = 5;

/// Backstop for an *unclean* link stall. A clean disconnect already pushes a
/// neutral report (see `bluetooth.rs`), so this only catches the link dying
/// without an event. It must sit well above the real report gap — the
/// controller notifies on input *change*, so while you hold a steady stick (or
/// pause) reports stop for a while; a tight timeout here would wrongly recentre
/// the servo mid-hold (the earlier 150 ms value did exactly that at the ~200 ms
/// report rate).
const FAILSAFE_MS: u64 = 1500;

/// Owning handle for the servo's PWM channel.
pub struct Servo {
    pwm: Pwm<'static>,
    config: PwmConfig,
}

impl Servo {
    /// Configure PWM slice 1 channel A on GP2 for 50 Hz servo signalling and
    /// park the servo at centre (0°).
    pub fn init(slice: Peri<'static, PWM_SLICE1>, pin: Peri<'static, PIN_2>) -> Self {
        let mut config = PwmConfig::default();
        config.divider = FixedU16::from_num(PWM_DIVIDER);
        config.top = PWM_TOP;
        config.enable = true;
        config.compare_a = PULSE_CENTRE_US as u16;

        let pwm = Pwm::new_output_a(slice, pin, config.clone());
        Self { pwm, config }
    }

    /// Set the servo angle in degrees, clamped to ±[`ANGLE_LIMIT_DEG`]; 0 is
    /// centre. The compare value reloads on the next PWM wrap, so the pulse
    /// width changes glitch-free.
    pub fn set_angle(&mut self, angle_deg: i32) {
        let angle = angle_deg.clamp(-ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
        // Linear map about the centre. The half-spans are taken separately so an
        // asymmetric MIN/MAX calibration still maps 0° to exactly centre.
        let half_span = if angle >= 0 {
            PULSE_MAX_US - PULSE_CENTRE_US
        } else {
            PULSE_CENTRE_US - PULSE_MIN_US
        };
        let pulse = PULSE_CENTRE_US + angle * half_span / ANGLE_LIMIT_DEG;
        self.config.compare_a = pulse as u16;
        self.pwm.set_config(&self.config);
    }

    /// Park the servo at centre (0°, idle).
    pub fn centre(&mut self) {
        self.set_angle(0);
    }
}

/// Map controller state to a servo angle: right-stick X, −100..100% (right is
/// positive), linearly scaled to ±[`ANGLE_LIMIT_DEG`], with a centre deadzone.
/// The sign is inverted so the servo horn follows the stick (pull left → horn
/// left); flip the negation if your linkage is mounted the other way.
fn report_to_angle(r: &XboxReport) -> i32 {
    let x = r.rx_pct();
    if x.abs() <= STICK_DEADZONE_PCT {
        0
    } else {
        -x * ANGLE_LIMIT_DEG / 100
    }
}

/// React to each controller update, recentring the servo if the signal is lost.
#[embassy_executor::task]
pub async fn servo_task(mut servo: Servo) {
    let Some(mut rx) = CONTROLLER.receiver() else {
        return;
    };
    loop {
        match with_timeout(Duration::from_millis(FAILSAFE_MS), rx.changed()).await {
            Ok(report) => servo.set_angle(report_to_angle(&report)),
            Err(_) => servo.centre(),
        }
    }
}
