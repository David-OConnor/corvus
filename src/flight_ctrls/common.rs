//! This module contains flight control code not specific to an aircraft design category.
//! It is mostly types.

use crate::{
    flight_ctrls::pid,
    protocols::dshot,
    safety::ArmStatus,
    setup::{MotorTimer, ServoTimer},
    util::map_linear,
    DT_FLIGHT_CTRLS,
};

use crate::protocols::dshot::Motor;
use lin_alg2::f32::Quaternion;

// Our input ranges for the 4 controls
const PITCH_IN_RNG: (f32, f32) = (-1., 1.);
const ROLL_IN_RNG: (f32, f32) = (-1., 1.);
const YAW_IN_RNG: (f32, f32) = (-1., 1.);
const THROTTLE_IN_RNG: (f32, f32) = (0., 1.);

/// Maps manual control inputs (range 0. to 1. or -1. to 1.) to velocities, rotational velocities etc
/// for various flight modes. The values are for full input range.
/// Note that defaults are defined in the `quad` and `fixed-wing` modules.
pub struct InputMap {
    /// Pitch velocity commanded, (Eg Acro mode). radians/sec
    pub pitch_rate: (f32, f32),
    /// Pitch velocity commanded (Eg Acro mode)
    pub roll_rate: (f32, f32),
    /// Yaw velocity commanded (Eg Acro mode)
    pub yaw_rate: (f32, f32),
    #[cfg(feature = "quad")]
    /// Throttle setting, clamped to leave room for maneuvering near the limits.
    pub throttle_clamped: (f32, f32),
    #[cfg(feature = "quad")]
    /// Pitch angle commanded (Eg Attitude mode) // radians from vertical
    pub pitch_angle: (f32, f32),
    #[cfg(feature = "quad")]
    /// Roll angle commanded (Eg Attitude mode)
    pub roll_angle: (f32, f32),
    /// When a stick (eg throttle) is mapped to a commanded baro altitude.
    /// Offset MSL is MSL, but 0 maps to launch alt
    pub alt_commanded_offset_msl: (f32, f32),
    /// When a stick (eg throttle) is mapped to a commanded AGL altitude.
    pub alt_commanded_agl: (f32, f32),
}

impl InputMap {
    /// Convert from control inputs to radians/s.
    pub fn calc_pitch_rate(&self, input: f32) -> f32 {
        map_linear(input, PITCH_IN_RNG, self.pitch_rate)
    }

    pub fn calc_roll_rate(&self, input: f32) -> f32 {
        map_linear(input, ROLL_IN_RNG, self.roll_rate)
    }

    pub fn calc_yaw_rate(&self, input: f32) -> f32 {
        map_linear(input, YAW_IN_RNG, self.yaw_rate)
    }

    #[cfg(feature = "quad")]
    pub fn calc_manual_throttle(&self, input: f32) -> f32 {
        map_linear(input, THROTTLE_IN_RNG, self.throttle_clamped)
    }

    #[cfg(feature = "quad")]
    pub fn calc_pitch_angle(&self, input: f32) -> f32 {
        map_linear(input, PITCH_IN_RNG, self.pitch_angle)
    }

    #[cfg(feature = "quad")]
    pub fn calc_roll_angle(&self, input: f32) -> f32 {
        map_linear(input, ROLL_IN_RNG, self.roll_angle)
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum AltType {
    /// Above ground level (eg from a TOF sensor)
    Agl,
    /// Mean sea level (eg from GPS or baro)
    Msl,
}

#[derive(Clone, Default)]
/// We use this to store control commands, that map directly to motor power (quads), or
/// elevon (and optionally rudder or differential engine/speedbrake power) on fixed-wing. Similar to `CtrlInputs`,
/// but without the options, and for a different use.
///
/// For quads, pitch, roll, and yaw are in RPM difference between the sum of each pair.
/// todo: For now, throttle is an absolute power setting on quads.
/// For fixed-wing, they're in arbitrary units, due to being unable to measure servo position.
pub struct CtrlMix {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
}

/// Stores inputs to the system. `pitch`, `yaw`, and `roll` are in range -1. to +1.
/// `thrust` is in range 0. to 1. Corresponds to stick positions on a controller, but can
/// also be used as a model for autonomous flight.
/// The interpretation of these depends on the current input mode.
/// These inputs, (if directly from flight control radio inputs), are translated from raw inputs from the radio
/// to -1. to 1. (0. to 1. for thrust)
///
/// A value of `None` indicates no value is commanded.
#[derive(Clone, Default)]
pub struct CtrlInputs {
    /// Acro mode: Change pitch angle
    /// Attitude mode: Command forward and aft motion
    pub pitch: Option<f32>,
    /// Acro mode: Change roll angle
    /// Attitude mode: Command left and right motion
    pub roll: Option<f32>,
    /// Yaw, in either mode
    pub yaw: Option<f32>,
    /// Acro mode: Change overall power (Altitude, or speed depending on orientation)
    /// Attitude mode: Change altitude
    pub throttle: Option<f32>,
}

/// Command a quaternion attitude, part of all of a Euler angle attitude, or neither.
/// If both a quaternion and euler angle are present, favor the quaternion.
#[derive(Default)]
pub struct AttitudeCommanded {
    pub quat: Option<Quaternion>,
    pub pitch: Option<f32>,
    pub roll: Option<f32>,
    pub yaw: Option<f32>,
}

// /// Command one or more angular rates.
// #[derive(Default)]
// pub struct RatesCommanded {
//     pub pitch: Option<f32>,
//     pub roll: Option<f32>,
//     pub yaw: Option<f32>,
// }
