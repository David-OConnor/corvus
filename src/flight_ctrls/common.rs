//! This module contains flight control code not specific to an aircraft design category.

// Todo: For rate-based controls on both fixed and quad: Consider using rate with PID while control
// todo inputs are changing. When no controls are present, use attititude to maintain position.
// todo: For the rate sub-loop, consider toning down I term, or possibly skipping the rate loop entirely.
// todo: For that last option, perhaps impl wise to maintain 8kHz etc update rate, make the inner
// todo loop attitude-based, instead of deferring to the mid loop.

use core::f32::consts::TAU;

use crate::{lin_alg::Quaternion, ppks::Location, safety::ArmStatus, util::map_linear};

use stm32_hal2::{pac, timer::Timer};

use super::{
    fixed_wing::ControlPositions,
    quad::{MotorPower, THROTTLE_MAX_MNVR_CLAMP, THROTTLE_MIN_MNVR_CLAMP},
};

// Our input ranges for the 4 controls
const PITCH_IN_RNG: (f32, f32) = (-1., 1.);
const ROLL_IN_RNG: (f32, f32) = (-1., 1.);
const YAW_IN_RNG: (f32, f32) = (-1., 1.);
const THROTTLE_IN_RNG: (f32, f32) = (0., 1.);

use defmt::println;

/// We use this buffer for DMA transfers of IMU readings. Note that reading order is different
/// between different IMUs, due to their reg layout, and consecutive reg reads. In both cases, 6 readings,
/// each with 2 bytes each.
static mut IMU_BUF: [u8; 12] = [0; 12];

// Time in seconds between subsequent data received before we execute lost-link procedures.
pub const LOST_LINK_TIMEOUT: f32 = 1.;

// todo: Quad specific?
/// Maps control inputs (range 0. to 1. or -1. to 1.) to velocities, rotational velocities etc
/// for various flight modes. The values are for full input range.
pub struct InputMap {
    /// Pitch velocity commanded, (Eg Acro mode). radians/sec
    pitch_rate: (f32, f32),
    /// Pitch velocity commanded (Eg Acro mode)
    roll_rate: (f32, f32),
    /// Yaw velocity commanded (Eg Acro mode)
    yaw_rate: (f32, f32),
    /// Throttle setting, clamped to leave room for maneuvering near the limits.
    throttle_clamped: (f32, f32),
    /// Pitch velocity commanded (Eg Attitude mode) // radians from vertical
    pitch_angle: (f32, f32),
    /// Pitch velocity commanded (Eg Attitude mode)
    roll_angle: (f32, f32),
    /// Yaw angle commanded v. Radians from north (?)
    yaw_angle: (f32, f32),
    /// Offset MSL is MSL, but 0 maps to launch alt
    alt_commanded_offset_msl: (f32, f32),
    alt_commanded_agl: (f32, f32),
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

    pub fn calc_manual_throttle(&self, input: f32) -> f32 {
        map_linear(input, THROTTLE_IN_RNG, self.throttle_clamped)
    }

    /// eg for attitude mode.
    pub fn calc_pitch_angle(&self, input: f32) -> f32 {
        map_linear(input, PITCH_IN_RNG, self.pitch_angle)
    }

    pub fn calc_roll_angle(&self, input: f32) -> f32 {
        map_linear(input, ROLL_IN_RNG, self.roll_angle)
    }

    pub fn calc_yaw_angle(&self, input: f32) -> f32 {
        map_linear(input, YAW_IN_RNG, self.yaw_angle)
    }
}

impl Default for InputMap {
    // tood: move deafult impls to their respective moudles (quad, flying wing)?
    fn default() -> Self {
        Self {
            pitch_rate: (-10., 10.),
            roll_rate: (-10., 10.),
            yaw_rate: (-10., 10.),
            throttle_clamped: (THROTTLE_MIN_MNVR_CLAMP, THROTTLE_MAX_MNVR_CLAMP),
            pitch_angle: (-TAU / 4., TAU / 4.),
            roll_angle: (-TAU / 4., TAU / 4.),
            yaw_angle: (0., TAU),
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
        }
    }
}

impl InputMap {
    pub fn default_flying_wing() -> Self {
        Self {
            pitch_rate: (-6., 6.),
            roll_rate: (-6., 6.),
            yaw_rate: (-0., 0.),        // N/A
            throttle_clamped: (0., 0.), // N/A
            pitch_angle: (0., 0.),      // N/A
            roll_angle: (0., 0.),       // N/A
            yaw_angle: (0., 0.),        // N/A
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
        }
    }
}

#[derive(Default)]
pub struct CommandState {
    pub x: f32,
    pub y: f32,
    pub alt: f32, // m MSL
    pub loiter_set: bool,
}

#[derive(Clone, Copy)]
pub enum AltType {
    /// Above ground level (eg from a TOF sensor)
    Agl,
    /// Mean sea level (eg from GPS or baro)
    Msl,
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
    pub thrust: Option<f32>,
}

/// Aircraft flight parameters, at a given instant. Pitch and roll rates are in the aircraft's
/// frame of reference.
#[derive(Default)]
pub struct Params {
    /// Longitude in radians. From GPS alone, or blended with accelerometer data.
    pub lon: f32,
    /// Latitude in radians. From GPS alone, or blended with accelerometer data.
    pub lat: f32,
    /// MSL altitude in meters QFE (takeoff location is 0), from a barometer.
    pub baro_alt_msl: f32,
    /// AGL altitude in meters, from the Time of flight sensor.
    pub tof_alt: f32,

    pub s_pitch: f32,
    pub s_roll: f32,
    /// Ie heading
    pub s_yaw_heading: f32,

    /// Quaternion of the attitude.
    pub quaternion: Quaternion,

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

/// Stores data on how the aircraft has performed in various recent flight conditions.
/// This data is used to estimate control surface positions or rotor power in response
/// to a change in commanded parameters. Rates are in rad/s.
pub struct ResponseDataPt {
    /// Forward airspeed
    pub airspeed: f32,
    // todo: Options for these, or an enum?
    pub motor_power: MotorPower,
    pub control_posits: ControlPositions,

    // todo: If you need more, consider using `Params`.
    pub pitch_rate: f32,
    pub roll_rate: f32,
    pub yaw_rate: f32,
}

// todo: Separate module for control_mixer?

// /// todo: Fill this out as you sort it out
// /// Store this persistently, and use it as a starting point for future updates. Suitable for quad
// /// and fixed wing. This seems very similar to `CtrlInputs`, but that is scaled from -1. to 1. etc,
// /// and this is in terms of rotor half delta.
// /// todo: Currently unused, but passed ina  few places that will show as unused in clippy.
// #[derive(Default)]
// pub struct ControlMix {
//     pub pitch: f32,
//     pub roll: f32,
//     pub yaw: f32,
//     pub throttle: f32,
// }

/// Abstraction over timers, that allows us to feature-gate struct fields based on MCU; this is
/// because we can't use feature gates on function arguments, so we're gating upstream.
/// todo: Consider adding apt methods to this, if it makes sense.
pub struct MotorTimers {
    #[cfg(feature = "h7")]
    /// Timer for all 4 rotors.
    pub r1234: Timer<pac::TIM3>,
    #[cfg(feature = "h7")]
    /// Servo timer. Note that we don't always use this, but the cost to store it here is minimal.
    pub servos: Timer<pac::TIM8>,
    #[cfg(feature = "g4")]
    /// Timer for rotors 1 and 2.
    pub r12: Timer<pac::TIM2>,
    #[cfg(feature = "g4")]
    /// Timer for rotors 3 and 4. Use for servos on fixed wing.
    pub r34_servos: Timer<pac::TIM3>,
}
