//! This module contains flight control code for flying-wing aircraft.
//! We use the motor 1-4 pins for a mix of motors and servos.
//! We use M1 for the power motor, M3 for left elevon, and M4 for right elevon.
//! M2 is currently unused. Possibly future uses include second motor, and rudder.

// todo: For wing, consider lowering your main loop frequency to whatever the min servo update frequency is.

use cfg_if::cfg_if;

use super::common::{CtrlMix, InputMap};
use crate::{
    dshot,
    protocols::servo,
    safety::ArmStatus,
    setup::{MotorTimer, ServoTimer},
    util,
};
// use defmt::println;

const MIN_MOTOR_POWER: f32 = 0.02;

// Max power setting for any individual rotor at idle setting.
pub const MAX_MOTOR_POWER: f32 = 1.;

// Constants that represent min and max position of servos.
const ELEVON_MIN: f32 = -1.;
const ELEVON_MAX: f32 = 1.;

const RUDDER_MIN: f32 = -1.;
const RUDDER_MAX: f32 = 1.;

const ANGULAR_ACCEL_LOG_RATIO: usize = 20;

impl Default for InputMap {
    fn default() -> Self {
        Self {
            pitch_rate: (-6., 6.),
            roll_rate: (-6., 6.),
            yaw_rate: (-6., 6.),
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
        }
    }
}

#[derive(Clone, Copy)]
pub enum YawControl {
    None,
    Rudder,
    DualProps,
    Both,
}

pub struct ControlSurfaceConfig {
    /// If elevator is not present, we assume the ailerons are stabilators.
    pub elevator: bool,
    /// Currently unused. For when both stabilzators and ailerons are present.
    pub stabilators: bool,
    pub yaw_control: YawControl,
}

impl Default for ControlSurfaceConfig {
    /// Dual-elevon config
    fn default() -> Self {
        Self {
            elevator: false,
            stabilators: false,
            yaw_control: YawControl::None,
        }
    }
}
