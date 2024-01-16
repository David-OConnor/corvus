//! This module contains flight control code not specific to an aircraft design category.
//! It is mostly types.

use core::f32::consts::TAU;

use defmt::println;
use lin_alg2::f32::Quaternion;

use crate::util::{self, map_linear};

// Our input ranges for the 4 controls. rad/s
const PITCH_IN_RNG: (f32, f32) = (-1., 1.);
const ROLL_IN_RNG: (f32, f32) = (-1., 1.);
const YAW_IN_RNG: (f32, f32) = (-1., 1.);
const THROTTLE_IN_RNG: (f32, f32) = (0., 1.);

// For attitude mode. rad.
const PITCH_IN_RNG_ATT: (f32, f32) = (-TAU / 4., TAU / 4.);
const ROLL_IN_RNG_ATT: (f32, f32) = (-TAU / 4., TAU / 4.);

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
    ///  In m/s.; mapped to throttle settings.
    pub vertical_velocity: (f32, f32),
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
        map_linear(input, PITCH_IN_RNG_ATT, self.pitch_angle)
    }

    #[cfg(feature = "quad")]
    pub fn calc_roll_angle(&self, input: f32) -> f32 {
        map_linear(input, ROLL_IN_RNG_ATT, self.roll_angle)
    }

    #[cfg(feature = "quad")]
    pub fn calc_vv(&self, input: f32, neutral_range: f32) -> f32 {
        // Re-map from 0 to 1, to -1 to 1, to make calculations clearer.
        let input_remapped = map_linear(input, THROTTLE_IN_RNG, (-1., 1.));

        if input_remapped > neutral_range {
            // Climb
            map_linear(
                input_remapped,
                (neutral_range, 1.),
                (0., self.vertical_velocity.1),
            )
        } else if input_remapped < -neutral_range {
            // Descend
            map_linear(
                input_remapped,
                (-1., -neutral_range),
                (self.vertical_velocity.0, 0.),
            )
        } else {
            0.
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)] // for USB ser
pub enum AltType {
    /// Above ground level (eg from a TOF sensor)
    Agl = 0,
    /// Mean sea level (eg from GPS or baro)
    Msl = 1,
}

#[derive(Clone, Default)]
/// We use this to store control commands, that map directly to motor power (quads), or
/// elevon (and optionally rudder or differential engine/speedbrake power) on fixed-wing. Similar to `CtrlInputs`,
/// but without the options, and for a different use.
///
/// For quads, pitch, roll, and yaw are in RPM difference between the sum of each pair.
/// For fixed-wing, they're in arbitrary units, due to being unable to measure servo position.
pub struct CtrlMix {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
}

impl CtrlMix {
    pub fn clamp(&mut self) {
        self.pitch = self.pitch.clamp(PITCH_IN_RNG.0, PITCH_IN_RNG.1);
        self.roll = self.roll.clamp(ROLL_IN_RNG.0, ROLL_IN_RNG.1);
        self.yaw = self.yaw.clamp(YAW_IN_RNG.0, YAW_IN_RNG.1);
        self.throttle = self.throttle.clamp(THROTTLE_IN_RNG.0, THROTTLE_IN_RNG.1);
    }
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

/// Command an attitude and attitude rate.
#[derive(Default)]
pub struct AttitudeCommanded {
    // pub quat: Option<Quaternion>,
    /// Our main attitude commanded
    pub quat: Quaternion,
    /// A change in attitude commanded per second, as an axis, and angular velocity in rad/s.
    /// todo: Should this be determined from current rate controls, or a change in teh quat?
    /// todo probably the latter, since it applies in other control modes.
    // pub quat_dt: Torque,
    /// todo: Switched to pitch, roll, yaw, radians-per-second.
    pub quat_dt: (f32, f32, f32),
    pub throttle: f32, // todo: COnsider after using, if this makes sense here.
                       // /// We use pitch, roll, and yaw if a specific axis is commanded.
                       // /// todo: Are these earth-centered?
                       // /// todo, Jan 2024: Do we use these?
                       // pub pitch: Option<f32>,
                       // pub roll: Option<f32>,
                       // pub yaw: Option<f32>,
}
