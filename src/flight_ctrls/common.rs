//! This module contains flight control code not specific to an aircraft design category.
//! It is mostly types.

use crate::util::map_linear;

use stm32_hal2::{pac, timer::Timer};

use lin_alg2::f32::Quaternion;

use cfg_if::cfg_if;

use defmt::println;

// Our input ranges for the 4 controls
const PITCH_IN_RNG: (f32, f32) = (-1., 1.);
const ROLL_IN_RNG: (f32, f32) = (-1., 1.);
const YAW_IN_RNG: (f32, f32) = (-1., 1.);
const THROTTLE_IN_RNG: (f32, f32) = (0., 1.);

/// Holds all 4 RPMs, by position.
/// todo: Quad-specific, but in `common` due to how we store it in Shared.
#[derive(Default)]
pub struct MotorRpm {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

/// Specify the rotor by its connection to the ESC. Includes methods that get information regarding timer
/// and DMA, per specific board setups, in `setup`.
/// Note that this is more appplicable to quads, but isn't in the `quad` module due to how
/// we've structured DSHOT code.
#[derive(Clone, Copy)]
pub enum Motor {
    M1,
    M2,
    M3,
    M4,
}

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

/// Command one or more angular rates.
#[derive(Default)]
pub struct RatesCommanded {
    pub pitch: Option<f32>,
    pub roll: Option<f32>,
    pub yaw: Option<f32>,
}

/// Aircraft flight parameters, at a given instant. Pitch and roll rates are in the aircraft's
/// frame of reference.
/// todo: This should be elsewhere.
#[derive(Default, Clone)]
pub struct Params {
    /// Latitude in radians. From GPS alone, or blended with accelerometer data.
    pub lat: f32,
    /// Longitude in radians. From GPS alone, or blended with accelerometer data.
    pub lon: f32,
    /// MSL altitude in meters QFE (takeoff location is 0), from a barometer.
    pub baro_alt_msl: f32,
    /// AGL altitude in meters, from the Time of flight sensor.
    pub tof_alt: Option<f32>,

    pub s_pitch: f32,
    pub s_roll: f32,
    /// Ie heading
    pub s_yaw_heading: f32,

    /// Quaternion of the attitude.
    pub attitude_quat: Quaternion,

    // todo: AHRS quaternion field, or leave that as part of the `AHRS` struct?

    // Velocity
    pub v_x: f32,
    pub v_y: f32,
    pub v_z: f32,

    pub v_pitch: f32,
    pub v_roll: f32,
    pub v_yaw: f32,

    // Acceleration
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,

    pub a_pitch: f32,
    pub a_roll: f32,
    pub a_yaw: f32,
}

/// Abstraction over timers, that allows us to feature-gate struct fields based on MCU; this is
/// because we can't use feature gates on function arguments, so we're gating upstream.
/// Note that for fixed-wing on H7, we can configure the 4 channels as any combination of motor and servo.
/// On G4, 1 and 2 are always motors, and 3 adn 4 are always servos.
/// todo: On G4, you can use a third servo on fixed-wing by configuring one of PA0/1 (r12)
/// todo as TIM5, ch 1/2. (instead of TIM2, ch1/2)
pub struct MotorTimers {
    #[cfg(feature = "h7")]
    /// Timer for all 4 rotors.
    pub rotors: Timer<pac::TIM3>,
    #[cfg(feature = "h7")]
    /// Servo timer. Note that we don't always use this, but the cost to store it here is minimal.
    pub servos: Timer<pac::TIM8>,
    #[cfg(feature = "g4")]
    /// Timer for rotors 1 and 2.
    pub r12: Timer<pac::TIM2>,
    #[cfg(all(feature = "g4", feature = "quad"))]
    /// Timer for rotors 3 and 4.
    pub r34: Timer<pac::TIM3>,
    #[cfg(all(feature = "g4", feature = "fixed-wing"))]
    /// Timer for servos. Note that this is the same field value as for G4 `quad` above, but with a
    /// name describing how we use it.
    pub servos: Timer<pac::TIM3>,
}
