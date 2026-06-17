//! TB6612FNG dual-motor driver, driven from the Xbox controller.
//!
//! Ported from the rust-microcontroller project's `motor_controller`, adapted
//! to this board's wiring and fed from the shared controller state
//! ([`crate::bluetooth::CONTROLLER`]) by an embassy task — there is no core1
//! superloop or command channel; reacting to the Watch directly is the
//! lowest-latency option (the BLE report interval dominates).
//!
//! Pinout owned by this module:
//! * `PWMA = GP16` (PWM slice 0, channel A) — right motor speed
//! * `PWMB = GP17` (PWM slice 0, channel B) — left motor speed
//! * Right motor direction: `AI1 = GP21`, `AI2 = GP22`
//! * Left motor direction:  `BI1 = GP19`, `BI2 = GP18`
//! * `STBY = GP20` — held high for the duration of the program so the driver
//!   stays enabled.

use embassy_rp::{
    gpio::{Level, Output},
    peripherals::{PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PWM_SLICE0},
    pwm::{Config as PwmConfig, Pwm},
    Peri,
};
use embassy_time::{with_timeout, Duration};

use crate::bluetooth::CONTROLLER;
use crate::xbox::XboxReport;

/// PWM counter wraparound driving both motor channels. Duty = `compare /
/// PWM_TOP`. At the RP2040 default clock this is ~31 kHz, above the motor's
/// audio band.
const PWM_TOP: u16 = 4000;

/// Minimum PWM count below which the motors can't overcome stiction; anything
/// in `(0, PWM_MIN_DUTY_COUNT)` would just buzz, so real commands are remapped
/// above this floor.
const PWM_MIN_DUTY_COUNT: u16 = 1200;

/// Joystick/throttle deadzone (fraction of full scale) below which a side is
/// forced to zero, filtering resting jitter.
const CONTROLLER_DEADZONE: f32 = 0.02;

/// `i8::MAX` as f32, for mapping `-127..127` wire values into `-1.0..1.0`.
const I8_FULL_SCALE_F32: f32 = 127.0;

/// If no controller update arrives within this window (the link delivers a
/// report roughly every ~15 ms), assume the signal is lost and halt — so a
/// disconnect mid-throttle stops the car instead of running away.
const FAILSAFE_MS: u64 = 150;

/// Owning handle for the two motor channels and their direction pins.
pub struct MotorController {
    /// PWM slice 0: channel A = PWMA (right), channel B = PWMB (left).
    pwm: Pwm<'static>,
    pwm_config: PwmConfig,
    ai1: Output<'static>,
    ai2: Output<'static>,
    bi1: Output<'static>,
    bi2: Output<'static>,
    // Held to keep STBY asserted; ownership prevents reassignment. Never toggled.
    #[allow(dead_code)]
    stby: Output<'static>,
}

/// Which physical motor a command targets.
enum MotorSide {
    Left,
    Right,
}

/// Direction state encoded on the AI1/AI2 (and BI1/BI2) pins.
enum MotorDriveDirection {
    Forward,
    Backward,
    Halt,
}

/// A drive command. Both fields are signed `-127..127`.
pub struct MotorDriveInfo {
    /// Forward/backward effort.
    pub throttle: i8,
    /// Steering bias: positive turns right, negative turns left.
    pub steer: i8,
}

/// Result of mixing a [`MotorDriveInfo`] into per-side direction and duty.
struct MotorDriveActuator {
    left_pwm_count: u16,
    right_pwm_count: u16,
    left_direction: MotorDriveDirection,
    right_direction: MotorDriveDirection,
}

impl MotorController {
    /// Construct the PWM slice (both channels) and configure the direction/STBY
    /// pins. STBY is driven high because the TB6612FNG requires it to enable
    /// the outputs.
    #[allow(clippy::too_many_arguments)]
    pub fn init(
        slice0: Peri<'static, PWM_SLICE0>,
        pin_pwma: Peri<'static, PIN_16>,
        pin_pwmb: Peri<'static, PIN_17>,
        pin_ai1: Peri<'static, PIN_21>,
        pin_ai2: Peri<'static, PIN_22>,
        pin_bi1: Peri<'static, PIN_19>,
        pin_bi2: Peri<'static, PIN_18>,
        pin_stby: Peri<'static, PIN_20>,
    ) -> Self {
        let mut pwm_config = PwmConfig::default();
        pwm_config.top = PWM_TOP;
        pwm_config.enable = true;
        pwm_config.compare_a = 0;
        pwm_config.compare_b = 0;

        // GP16 is even -> channel A, GP17 is odd -> channel B, both on slice 0,
        // so one Pwm drives both motors with independent duty.
        let pwm = Pwm::new_output_ab(slice0, pin_pwma, pin_pwmb, pwm_config.clone());

        Self {
            pwm,
            pwm_config,
            ai1: Output::new(pin_ai1, Level::Low),
            ai2: Output::new(pin_ai2, Level::Low),
            bi1: Output::new(pin_bi1, Level::Low),
            bi2: Output::new(pin_bi2, Level::Low),
            stby: Output::new(pin_stby, Level::High),
        }
    }

    /// Drive the direction pins for one motor. The TB6612FNG halts when both
    /// inputs are low.
    fn apply_direction(&mut self, side: MotorSide, direction: MotorDriveDirection) {
        let (pin_1, pin_2): (&mut Output<'static>, &mut Output<'static>) = match side {
            MotorSide::Left => (&mut self.bi1, &mut self.bi2),
            MotorSide::Right => (&mut self.ai1, &mut self.ai2),
        };
        match direction {
            MotorDriveDirection::Forward => {
                pin_1.set_high();
                pin_2.set_low();
            }
            MotorDriveDirection::Backward => {
                pin_1.set_low();
                pin_2.set_high();
            }
            MotorDriveDirection::Halt => {
                pin_1.set_low();
                pin_2.set_low();
            }
        }
    }

    /// Mix a `(throttle, steer)` command and apply it to both motors. The PWM
    /// hardware reloads the compare values on the next wrap, so duty changes are
    /// glitch-free.
    pub fn drive(&mut self, drive_info: MotorDriveInfo) {
        let actuator = motor_drive_info_to_actuator(drive_info);
        self.apply_direction(MotorSide::Left, actuator.left_direction);
        self.apply_direction(MotorSide::Right, actuator.right_direction);
        // Channel A = PWMA = right motor; channel B = PWMB = left motor.
        self.pwm_config.compare_a = actuator.right_pwm_count;
        self.pwm_config.compare_b = actuator.left_pwm_count;
        self.pwm.set_config(&self.pwm_config);
    }

    /// Cut all drive: halt both direction pairs and zero both duties.
    pub fn stop(&mut self) {
        self.apply_direction(MotorSide::Left, MotorDriveDirection::Halt);
        self.apply_direction(MotorSide::Right, MotorDriveDirection::Halt);
        self.pwm_config.compare_a = 0;
        self.pwm_config.compare_b = 0;
        self.pwm.set_config(&self.pwm_config);
    }
}

/// Mix a `(throttle, steer)` command into per-side direction and PWM duty.
/// Negative throttle inverts the steering convention so the vehicle turns the
/// way the operator expects while reversing.
fn motor_drive_info_to_actuator(drive_info: MotorDriveInfo) -> MotorDriveActuator {
    let throttle = drive_info.throttle as f32 / I8_FULL_SCALE_F32;
    let steer = drive_info.steer as f32 / I8_FULL_SCALE_F32;

    // Convention: +ve = forward and right, -ve = backward and left.
    let mut left_mix = throttle + steer;
    let mut right_mix = throttle - steer;

    if throttle < 0.0 {
        left_mix = throttle - steer;
        right_mix = throttle + steer;
    }

    // If either side overflows ±1.0 after mixing, scale both down so the larger
    // sits exactly at ±1.0 (preserves the left/right ratio).
    if left_mix.abs() > 1.0 {
        right_mix /= left_mix.abs();
        left_mix /= left_mix.abs();
    } else if right_mix.abs() > 1.0 {
        left_mix /= right_mix.abs();
        right_mix /= right_mix.abs();
    }

    let left_direction = if left_mix < 0.0 {
        MotorDriveDirection::Backward
    } else {
        MotorDriveDirection::Forward
    };
    let right_direction = if right_mix < 0.0 {
        MotorDriveDirection::Backward
    } else {
        MotorDriveDirection::Forward
    };

    let span = (PWM_TOP - PWM_MIN_DUTY_COUNT) as f32;
    let to_count = |mix: f32| -> u16 {
        if mix.abs() > CONTROLLER_DEADZONE {
            (PWM_MIN_DUTY_COUNT + (mix.abs() * span) as u16).min(PWM_TOP)
        } else {
            0
        }
    };

    MotorDriveActuator {
        left_pwm_count: to_count(left_mix),
        right_pwm_count: to_count(right_mix),
        left_direction,
        right_direction,
    }
}

/// Map controller state to a drive command: throttle = RT − LT (right trigger
/// forward, left trigger reverse), steer = right stick X.
fn report_to_drive(r: &XboxReport) -> MotorDriveInfo {
    let throttle = ((r.rt as i32 - r.lt as i32) * 127 / 1023) as i8;
    let steer = (r.rx_pct() * 127 / 100).clamp(-127, 127) as i8;
    MotorDriveInfo { throttle, steer }
}

/// React to each controller update, halting if the signal is lost.
#[embassy_executor::task]
pub async fn drive_task(mut motors: MotorController) {
    let Some(mut rx) = CONTROLLER.receiver() else {
        return;
    };
    loop {
        match with_timeout(Duration::from_millis(FAILSAFE_MS), rx.changed()).await {
            Ok(report) => motors.drive(report_to_drive(&report)),
            Err(_) => motors.stop(),
        }
    }
}
