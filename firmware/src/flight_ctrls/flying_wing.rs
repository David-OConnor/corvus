//! This module contains flight control code for flying-wing aircraft.

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::{TimChannel, Timer},
};

use crate::{
    dshot,
    flight_ctrls::quad::Motor, // todo: Maybe move this out of quad?
    safety::ArmStatus,
    util,
};

use cfg_if::cfg_if;

use defmt::println;

const ELEVON_MIN: f32 = -1.;
const ELEVON_MAX: f32 = 1.;

// ROLL_COEFF is used to balance pitch and roll input sensitivity, compared to the implied
// pitch coeffecient of 1.
const ROLL_COEFF: f32 = 1.;

pub const PSC: u16 = 0;

cfg_if! {
    if #[cfg(feature = "h7")] {
        pub const ARR: u32 = 332; // todo
    } else if #[cfg(feature = "g4")] {
        // 170Mhz tim clock. Results in _Hz.
        pub const ARR: u32 = 282; // todo
    }
}

const DUTY_HIGH: f32 = ARR as f32;
const DUTY_LOW: f32 = 0.;

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
                servo_tim.set_duty(
                    TimChannel::C1,
                    util::map_linear(
                        self.elevon_left,
                        (ELEVON_MIN, ELEVON_MAX),
                        (DUTY_LOW, DUTY_HIGH),
                    ) as u32,
                );
                servo_tim.set_duty(
                    TimChannel::C2,
                    util::map_linear(
                        self.elevon_right,
                        (ELEVON_MIN, ELEVON_MAX),
                        (DUTY_LOW, DUTY_HIGH),
                    ) as u32,
                );
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
