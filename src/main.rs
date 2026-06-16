#![no_std]
#![no_main]

//! Quick motor / motor-driver bring-up test.
//!
//! TB6612FNG pinout (same as the reference firmware in
//! `rust-microcontroller/src`):
//! * `PWMA = GPIO15` (PWM slice 7, channel B)  -> right motor speed
//! * `PWMB = GPIO9`  (PWM slice 4, channel B)  -> left  motor speed
//! * Right motor direction: `AI1 = GPIO13`, `AI2 = GPIO14`
//! * Left  motor direction: `BI1 = GPIO11`, `BI2 = GPIO10`
//! * `STBY = GPIO12` — held high to enable the driver.
//!
//! On boot this repeats a pattern forever: drive both motors forward for
//! 5 seconds, then backward for 5 seconds, and so on.
//!
//! The on-board LED (GPIO25) gives visual feedback so you can confirm the
//! firmware is alive even with no motors connected:
//! * 3 fast blinks at startup  -> firmware booted, reached the test
//! * solid ON                  -> motors commanded forward
//! * solid OFF                 -> motors commanded backward

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use hal::pac;
use panic_halt as _;
use rp2040_hal::{self as hal, pwm::Slices, Clock};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

/// PWM counter wraparound. Duty cycle is `compare / PWM_TOP`; matches the
/// reference firmware so behaviour is comparable.
const PWM_TOP: u16 = 4000;

/// How long to run the motors in each direction before reversing.
const RUN_TIME_MS: u32 = 5_000;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // On-board LED for liveness feedback. Triple-blink: we booted and got here.
    let mut led = pins.gpio25.into_push_pull_output();
    for _ in 0..3 {
        led.set_high().unwrap();
        delay.delay_ms(100);
        led.set_low().unwrap();
        delay.delay_ms(100);
    }

    // --- Direction pins -------------------------------------------------------
    // Right motor (A): AI1/AI2.  Left motor (B): BI1/BI2.
    let mut ai1 = pins.gpio7.into_push_pull_output();
    let mut ai2 = pins.gpio6.into_push_pull_output();
    let mut bi1 = pins.gpio26.into_push_pull_output();
    let mut bi2 = pins.gpio27.into_push_pull_output();
    // STBY high enables the TB6612FNG.
    let mut stby = pins.gpio28.into_push_pull_output();

    // --- PWM speed channels ---------------------------------------------------
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    // PWMA = GPIO15 -> slice 7, channel B (right motor).
    let mut pwm_a = pwm_slices.pwm4;
    pwm_a.set_top(PWM_TOP);
    pwm_a.enable();
    pwm_a.channel_a.output_to(pins.gpio8);

    // PWMB = GPIO9 -> slice 4, channel B (left motor).
    let mut pwm_b = pwm_slices.pwm3;
    pwm_b.set_top(PWM_TOP);
    pwm_b.enable();
    pwm_b.channel_a.output_to(pins.gpio22);

    // --- Run the motors -------------------------------------------------------
    stby.set_high().unwrap();

    // Full speed = duty cycle at the PWM top.
    let _ = pwm_a.channel_a.set_duty_cycle(3800);
    let _ = pwm_b.channel_a.set_duty_cycle(3800);

    // Repeat forever: forward for 5 s, then backward for 5 s.
    loop {
        // Forward = IN1 high, IN2 low on both sides. LED solid ON.
        ai1.set_low().unwrap();
        ai2.set_high().unwrap();
        bi1.set_low().unwrap();
        bi2.set_high().unwrap();
        led.set_high().unwrap();
        delay.delay_ms(RUN_TIME_MS);

        // Backward = IN1 low, IN2 high on both sides. LED solid OFF.
        ai1.set_high().unwrap();
        ai2.set_low().unwrap();
        bi1.set_high().unwrap();
        bi2.set_low().unwrap();
        led.set_low().unwrap();
        delay.delay_ms(RUN_TIME_MS);
    }
}
