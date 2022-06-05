//! This module contains flight control code for flying-wing aircraft.

// todo: For wing, consider lowering your main loop frequency to whatever the min servo update frequency is.

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::{OutputCompare, TimChannel, Timer, TimerInterrupt},
};

use crate::{
    dshot,
    flight_ctrls::quad::Motor, // todo: Maybe move this out of quad?
    safety::ArmStatus,
    util,
};

// todo: We're going to assume the servos operate off pulse width, with frequency between 40 and 200hz.
// todo: Unable to find DS for the specific servos used here.

use cfg_if::cfg_if;

use defmt::println;

const ELEVON_MIN: f32 = -1.;
const ELEVON_MAX: f32 = 1.;

// ROLL_COEFF is used to balance pitch and roll input sensitivity, compared to the implied
// pitch coeffecient of 1.
const ROLL_COEFF: f32 = 1.;

// These pulse durations are in seconds, and correspond to nuetral, and full scale deflections.
// todo: Probably should be in a user config.
const ELEVON_PULSE_DUR_NEUTRAL: f32 = 0.0015; // In seconds. With this pulse dir, cervo is centered.
const ELEVON_PULSE_DUR_FULL_DOWN: f32 = 0.001; // In seconds. With this pulse dir, cervo is centered.
const ELEVON_PULSE_DUR_FULL_UP: f32 = 0.002; // In seconds. With this pulse dir, cervo is centered.

cfg_if! {
    if #[cfg(feature = "h7")] {
        pub const PSC: u16 = 0; // todo
        pub const ARR: u32 = 332; // todo
    } else if #[cfg(feature = "g4")] {
        // 170Mhz tim clock. Results in 500Hz.
        // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
        pub const PSC: u16 = 6;
        pub const ARR: u32 = 48_570;
    }
}

// High and low correspond to 2ms, and 1ms pulse width respsectively. (Period of 500Hz is 2ms)
// const DUTY_HIGH: u32 = ARR / 5 * 2;
// const DUTY_LOW: u32 = ARR / 5;
// We don't use full ARR for max high, since that would be full high the whole time.
const SERVO_DUTY_HIGH: f32 = (ARR - 60) as f32;
const SERVO_DUTY_LOW: f32 = (ARR / 2) as f32;

/// Sets the position of an elevon
pub fn set_elevon_posit(elevon: ServoWing, position: f32, servo_timer: &mut Timer<TIM3>) {
    let duty_arr = util::map_linear(
        position,
        (ELEVON_MIN, ELEVON_MAX),
        (SERVO_DUTY_LOW, SERVO_DUTY_HIGH),
    ) as u32;
    servo_timer.set_duty(elevon.tim_channel(), duty_arr);
}

/// See also: `dshot::setup_timers`.
pub fn setup_timers(motor_timer: &mut Timer<TIM2>, servo_timer: &mut Timer<TIM3>) {
    motor_timer.set_prescaler(dshot::DSHOT_PSC_600);
    motor_timer.set_auto_reload(dshot::DSHOT_ARR_600 as u32);
    servo_timer.set_prescaler(PSC);
    servo_timer.set_auto_reload(ARR);

    motor_timer.enable_interrupt(TimerInterrupt::UpdateDma);

    // Arbitrary duty cycle set, since we'll override it with DMA bursts.
    motor_timer.enable_pwm_output(Motor::M1.tim_channel(), OutputCompare::Pwm1, 0.);
    servo_timer.enable_pwm_output(ServoWing::S1.tim_channel(), OutputCompare::Pwm1, 0.);
    servo_timer.enable_pwm_output(ServoWing::S2.tim_channel(), OutputCompare::Pwm1, 0.);
}

/// Equivalent of `Motor` for quadcopters.
#[derive(Clone, Copy)]
pub enum ServoWing {
    S1,
    S2,
}

/// Specify the wing associated with a servo. Equivalent of `RotorPosition` for quadcopters.
/// repr(u8) is for use in Preflight.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ServoWingPosition {
    Left = 0,
    Right = 1,
}

/// Equivalent of `RotorMapping` for quadcopters.
pub struct ServoWingMapping {
    pub s1: ServoWingPosition,
    pub s2: ServoWingPosition,
}

impl Default for ServoWingMapping {
    fn default() -> Self {
        Self {
            s1: ServoWingPosition::Left,
            s2: ServoWingPosition::Right,
        }
    }
}

impl ServoWingMapping {
    pub fn servo_from_position(&self, position: ServoWingPosition) -> ServoWing {
        // todo: This assumes each servo maps to exactly one position. We probably
        // todo should have some constraint to enforce this.
        if self.s1 == position {
            ServoWing::S1
        } else {
            ServoWing::S2
        }
    }
}

/// Represents control settings for the motor, and elevons. Equivalent API to `quad::RotorPower`.
/// Positive elevon value means pointed up relative to its hinge point.
#[derive(Default)]
pub struct ControlSurfaceSettings {
    pub motor: f32,
    pub elevon_left: f32,
    pub elevon_right: f32,
}

impl ControlSurfaceSettings {
    /// Send this command to cause power to be applied to the motor and servos.
    pub fn set(
        &mut self,
        motor_tim: &mut Timer<TIM2>,
        servo_tim: &mut Timer<TIM3>,
        arm_status: ArmStatus,
        dma: &mut Dma<DMA1>,
    ) {
        // M2 isn't used here, but keeps our API similar to Quad.
        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power_a(Motor::M1, Motor::M2, self.motor, 0., motor_tim, dma);

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, servo_tim);
                set_elevon_posit(ServoWing::S2, self.elevon_right, servo_tim);
            }
            ArmStatus::Disarmed => {
                dshot::set_power_a(Motor::M1, Motor::M2, 0., 0., motor_tim, dma);
                servo_tim.set_duty(TimChannel::C1, ARR / 2);
                servo_tim.set_duty(TimChannel::C2, ARR / 2);
            }
        }
    }
}

// todo: Move PWM code out of this module if it makes sense, ie separate servo; flight-control module

/// Apply controls based on pitch, roll, yaw, and throttle. Servo average position controls pitch;
/// servo difference controls roll. We don't have a yaw control.
/// If a servo exceeds min or max power settings, clamp it.
///
/// Positive pitch means nose up. Positive roll means left wing up.
///
/// Input deltas as on an abitrary scale based on PID output; they're not in real units like radians/s.
pub fn apply_controls(
    pitch_delta: f32,
    roll_delta: f32,
    throttle: f32,
    motor_tim: &mut Timer<TIM2>,
    servo_tim: &mut Timer<TIM3>,
    arm_status: ArmStatus,
    dma: &mut Dma<DMA1>,
) {
    let mut elevon_left = 0.;
    let mut elevon_right = 0.;

    elevon_left += pitch_delta;
    elevon_right += pitch_delta;

    elevon_left -= roll_delta * ROLL_COEFF;
    elevon_right += roll_delta * ROLL_COEFF;

    // todo: Clamp both elevons in both directions.

    let mut settings = ControlSurfaceSettings {
        motor: throttle,
        elevon_left,
        elevon_right,
    };

    settings.set(motor_tim, servo_tim, arm_status, dma);
}
