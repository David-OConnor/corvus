//! This module contains code related to flight controls. Code specific to the PID
//! loops are in the `pid` module.
//!
//! [Betaflight Signal flow diagram](https://github.com/betaflight/betaflight/wiki/Signal-Flow-Diagram)
//! Note that this is just an example, and isn't necesssarily something to emulate.

// todo: Split up further

use core::{
    f32::consts::TAU,
    ops::{Add, DivAssign, Mul, MulAssign, Sub},
    sync::atomic::{AtomicBool, Ordering},
};

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::Timer,
};

use defmt::println;

use cmsis_dsp_sys as dsp_sys;

use crate::{control_interface::ChannelData, dshot, pid::PidState, safety::ArmStatus, util, util::map_linear, CtrlCoeffGroup, Location, UserCfg, StateVolatile};

// If the throttle signal is below this, set idle power.
const THROTTLE_IDLE_THRESH: f32 = 0.03;

// Power at idle setting.
const THROTTLE_IDLE_POWER: f32 = 0.03; //
                                       // Max power setting. SHould be 1.
const THROTTLE_MAX_POWER: f32 = 1.; //

// Don't execute the calibration procedure from below this altitude, eg for safety.
const MIN_CAL_ALT: f32 = 6.;

// Time in seconds between subsequent data received before we execute lost-link procedures.
pub const LOST_LINK_TIMEOUT: f32 = 1.;

// With maximum commanded pitch, yaw, or roll rate, add and subtract this value from
// opposing rotors. Keep this relatively low, since thi scan add up from maneuvers on multiple
// axes!
const ROTOR_HALF_PAIR_DELTA_MAX: f32 = 0.15;

// Our input ranges for the 4 controls
const PITCH_IN_RNG: (f32, f32) = (-1., 1.);
const ROLL_IN_RNG: (f32, f32) = (-1., 1.);
const YAW_IN_RNG: (f32, f32) = (-1., 1.);
const THRUST_IN_RNG: (f32, f32) = (0., 1.);

// Our output ranges for motor power.
const PITCH_PWR_RNG: (f32, f32) = (-1., 1.);
const ROLL_PWR_RNG: (f32, f32) = (-1., 1.);
const YAW_PWR_RNG: (f32, f32) = (-1., 1.);
// todo: Note that if you support 0-centering throttles, make this -1 to +1 as well.
const THRUST_PWR_RNG: (f32, f32) = (0., 1.);

// Minimium speed before auto-yaw will engate. (if we end up setting up auto-yaw to align flight path
// with heading)
// todo: Maybe this could also be used if we end up setting up auto-yaw as sideway-accel cancellation?
// todo, and this would be the min *fwd* velocity?
pub const YAW_ASSIST_MIN_SPEED: f32 = 0.5; // m/s

// if coeff = 0.5, if accel is 1 m/s^2, yaw correction is 1/2 rad/s
// angular velocity / accel: (radians/s) / (m/s^2) = radiants x s / m
pub const YAW_ASSIST_COEFF: f32 = 0.1;

// We use these LUTs to map thrust commanded to throttle position. Note that the starting values will
// include an idle setting.
// todo: Do we need to use these locally, to offset for idle setting?
// const THRUST_LUT: [f32; 10] = [
//     0.027, 0.075, 0.131, 0.225, 0.345, 0.473, 0.613, 0.751, 0.898, 1.0
// ];

pub const POWER_LUT: [f32; 10] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

/// We use this buffer for DMA transfers of IMU readings. Note that reading order is different
/// between different IMUs, due to their reg layout, and consecutive reg reads. In both cases, 6 readings,
/// each with 2 bytes each.
static mut IMU_BUF: [u8; 12] = [0; 12];

/// Represents a complete quadcopter. Used for setting control parameters.
struct AircraftProperties {
    mass: f32,               // grams
    arm_len: f32,            // meters. COG to rotor center, horizontally.
    drag_coeff: f32,         // unitless
    thrust_coeff: f32,       // N/m^2
    moment_of_intertia: f32, // kg x m^2
    rotor_inertia: f32,      // kg x m^2
}

impl AircraftProperties {
    /// Calculate the power level required, applied to each rotor, to maintain level flight
    /// at a given MSL altitude. (Alt is in meters)
    pub fn level_pwr(&self, alt: f32) -> f32 {
        return 0.1; // todo
    }
}

// /// In the event we lose our control link signal
// /// todo: Consider removing this later once you settle on a procedure.
// pub enum LostLinkProcedure {
//     /// turn off all motors. Aircraft will drop to the ground and crash.
//     Disarm,
//     /// Command level attitude and/or louter, and auto land. TOF optional.
//     LoiterAutoLand,
//     /// Return to base, climbing as required, and land. Requires GPS. TOF optional.
//     RtbLand,
//     /// Climb, to attempt to re-acquire the link
//     Climb,
//     /// Hover
//     Hover
// }

/// We use this to freeze an axis in acro mode, when the control for that axis is neutral.
/// this prevents drift from accumulation of errors. Values are uuler angle locked at, in radians. Only used
/// for pitch and roll, since yaw is coupled to those, and can't be maintained independently.
/// todo: Is there a way to do this using portions of a quaternion?
#[derive(Default)]
pub struct AxisLocks {
    pub pitch_locked: Option<f32>,
    pub roll_locked: Option<f32>,
}

/// Specify the rotor by its connection to the ESC. Includdes methods that get information regarding timer
/// and DMA, per specific board setups, in `setup`.
#[derive(Clone, Copy)]
pub enum Rotor {
    R1,
    R2,
    R3,
    R4,
}

/// Specify the rotor by position. Used in power application code.
#[derive(Clone, Copy)]
pub enum RotorPosition {
    FrontLeft,
    FrontRight,
    AftLeft,
    AftRight,
}

/// Maps control inputs (range 0. to 1. or -1. to 1.) to velocities, rotational velocities etc
/// for various flight modes. The values are for full input range.
pub struct InputMap {
    /// Pitch velocity commanded, (Eg Acro mode). radians/sec
    pitch_rate: (f32, f32),
    /// Pitch velocity commanded (Eg Acro mode)
    roll_rate: (f32, f32),
    /// Yaw velocity commanded (Eg Acro mode)
    yaw_rate: (f32, f32),
    /// Power level
    power_level: (f32, f32),
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

    /// Convert from radians/s to the range used to set motor power.
    pub fn calc_pitch_rate_pwr(&self, input: f32) -> f32 {
        map_linear(input, self.pitch_rate, PITCH_PWR_RNG)
    }

    pub fn calc_roll_rate_pwr(&self, input: f32) -> f32 {
        map_linear(input, self.roll_rate, ROLL_PWR_RNG)
    }

    pub fn calc_yaw_rate_pwr(&self, input: f32) -> f32 {
        map_linear(input, self.yaw_rate, YAW_PWR_RNG)
    }

    pub fn calc_thrust(&self, input: f32) -> f32 {
        map_linear(input, THRUST_IN_RNG, self.power_level)
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
    fn default() -> Self {
        Self {
            pitch_rate: (-10., 10.),
            roll_rate: (-10., 10.),
            yaw_rate: (-10., 10.),
            power_level: (0., 1.),
            pitch_angle: (-TAU / 4., TAU / 4.),
            roll_angle: (-TAU / 4., TAU / 4.),
            yaw_angle: (0., TAU),
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
        }
    }
}

#[derive(Default)]
pub struct CommandState {
    // pub pre_armed: ArmStatus,
    pub arm_status: ArmStatus,
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

/// Categories of control mode, in regards to which parameters are held fixed.
/// Note that some settings are mutually exclusive.
#[derive(Default)]
pub struct AutopilotStatus {
    /// Altitude is fixed. (MSL or AGL)
    pub alt_hold: Option<(AltType, f32)>,
    /// Heading is fixed.
    pub hdg_hold: Option<f32>,
    /// Automatically adjust raw to zero out slip
    pub yaw_assist: bool,
    /// Automatically adjust roll (rate? angle?) to zero out slip, ie based on rudder inputs.
    /// Don't enable both yaw assist and roll assist at the same time.
    pub roll_assist: bool,
    /// Continuously fly towards a path. Note that `pitch` and `yaw` for the
    /// parameters here correspond to the flight path; not attitude.
    pub velocity_vector: Option<(f32, f32)>, // pitch, yaw
    /// Fly direct to a point
    pub direct_to_point: Option<Location>,
    /// The aircraft will fly a fixed profile between sequence points
    pub sequence: bool,
    /// Terrain following mode. Similar to TF radar in a jet. Require a forward-pointing sensor.
    /// todo: Add a forward (or angled) TOF sensor, identical to the downward-facing one?
    pub terrain_following: Option<f32>, // AGL to hold
    /// Take off automatically
    pub takeoff: bool,
    /// Land automatically
    pub land: bool,
    /// Recover to stable, altitude-holding flight. Generally initiated by a "panic button"-style
    /// switch activation
    pub recover: Option<f32>, // value is MSL alt to hold, eg our alt at time of command.
}

// todo: Consider putting these mode switches in `control_interface`.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// For the switch position. We interpret actual mode from this, and other data, like prescense of GPS.
/// val is for passing over USB serial.
pub enum InputModeSwitch {
    /// Acro mode
    Acro = 0,
    /// Command if GPS is present; Attitude if not
    AttitudeCommand = 1,
}

impl Default for InputModeSwitch {
    fn default() -> Self {
        Self::Acro
    }
}

// todo: Consider putting these mode switches in `control_interface`.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// For the switch position. We interpret actual mode from this, and other data, like prescense of GPS.
/// val is for passing over USB serial.
pub enum AltHoldSwitch {
    Disabled = 0,
    EnabledMsl = 1,
    EnabledAgl = 2,
}

impl Default for AltHoldSwitch {
    fn default() -> Self {
        Self::Disabled
    }
}

/// Mode used for control inputs. These are the three "industry-standard" modes.
#[derive(Clone, Copy, PartialEq)]
pub enum InputMode {
    /// Rate, also know as manual, hard or Acro. Attitude and power stay the same after
    /// releasing controls.
    Acro,
    /// Attitude also know as self-level, angle, or Auto-level. Attitude resets to a level
    /// hover after releasing controls.  When moving the
    /// roll/pitch stick to its maximum position, the drone will also reach the maximum angle
    /// it’s allowed to tilt (defined by the user), and it won’t flip over. As you release the
    /// stick back to centre, the aircraft will also return to its level position.
    /// We use attitude mode as a no-GPS fallback.
    Attitude,
    // GPS-hold, also known as Loiter. Maintains a specific position.
    /// In `Command` mode, the device loiters when idle. Otherwise, it flies at specific velocities,
    /// and altitudes commanded by the controller. Allows for precise control, including in confined
    /// spaces.
    Command,
    // /// This mode is easy stable, and designed to make control easy, including in confined spaces.
    // /// Similar to `Command` mode, it loiters when idle. It uses an internal model of
    // /// todo: Same as Command mode? Consolidate?
    // VideoGame,
}

/// Stores the current manual inputs to the system. `pitch`, `yaw`, and `roll` are in range -1. to +1.
/// `thrust` is in range 0. to 1. Corresponds to stick positions on a controller, but can
/// also be used as a model for autonomous flight.
/// The interpretation of these depends on the current input mode.
/// These inputs, (if directly from flight control radio inputs), are translated from raw inputs from the radio
/// to -1. to 1. (0. to 1. for thrust)
#[derive(Clone, Default)]
pub struct CtrlInputs {
    /// Acro mode: Change pitch angle
    /// Attitude mode: Command forward and aft motion
    pub pitch: f32,
    /// Acro mode: Change roll angle
    /// Attitude mode: Command left and right motion
    pub roll: f32,
    /// Yaw, in either mode
    pub yaw: f32,
    /// Acro mode: Change overall power (Altitude, or speed depending on orientation)
    /// Attitude mode: Change altitude
    pub thrust: f32,
}
//
// impl CtrlInputs {
//     /// Get manual inputs from the radio. Map from the radio input values to our standard values of
//     /// 0. to 1. (throttle), and -1. to 1. (pitch, roll, yaw).
//     pub fn get_manual_inputs(cfg: &UserCfg) -> Self {
//         // todo: Get radio input here.!
//         let pitch = 0.;
//         let roll = 0.;
//         let yaw = 0.;
//         let thrust = 0.;
//
//         Self {
//             pitch: map_linear(pitch, cfg.pitch_input_range, PITCH_IN_RNG),
//             roll: map_linear(roll, cfg.roll_input_range, ROLL_IN_RNG),
//             yaw: map_linear(yaw, cfg.pitch_input_range, YAW_IN_RNG),
//             thrust: map_linear(thrust, cfg.throttle_input_range, THRUST_IN_RNG),
//         }
//     }
// }

/// Represents parameters at a fixed instant. Can be position, velocity, or accel.
#[derive(Default)]
pub struct ParamsInst {
    pub x: f32,
    pub y: f32,
    pub z_msl: f32,
    pub z_agl: f32,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

impl Add for ParamsInst {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z_msl: self.z_msl + other.z_msl,
            z_agl: self.z_agl + other.z_agl,
            pitch: self.pitch + other.pitch,
            roll: self.roll + other.roll,
            yaw: self.yaw + other.yaw,
        }
    }
}

impl Sub for ParamsInst {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z_msl: self.z_msl - other.z_msl,
            z_agl: self.z_agl - other.z_agl,
            pitch: self.pitch - other.pitch,
            roll: self.roll - other.roll,
            yaw: self.yaw - other.yaw,
        }
    }
}

// Note: We impl multiplication for f32, as a workaround; that's how you impl it for a scalar.
impl Mul<ParamsInst> for f32 {
    type Output = ParamsInst;

    fn mul(self, other: ParamsInst) -> ParamsInst {
        ParamsInst {
            x: other.x * self,
            y: other.y * self,
            z_msl: other.z_msl * self,
            z_agl: other.z_agl * self,
            pitch: other.pitch * self,
            roll: other.roll * self,
            yaw: other.yaw * self,
        }
    }
}

// todo: Quaternions?

/// Represents a first-order status of the drone. todo: What grid/reference are we using?
#[derive(Default)]
pub struct Params {
    // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
    pub s_x: f32,
    pub s_y: f32,
    // Note that we only need to specify MSL vs AGL for position; velocity and accel should
    // be equiv for them.
    pub s_z_msl: f32,
    pub s_z_agl: f32,

    pub s_pitch: f32,
    pub s_roll: f32,
    pub s_yaw: f32,

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

// impl Params {
//     pub fn get_s(&self) -> ParamsInst {
//         ParamsInst {
//             x: self.s_x, y: self.s_y, z: self.s_z,
//             pitch: self.s_pitch, roll: self.s_roll, yaw: self.s_yaw
//         }
//     }

//     pub fn get_v(&self) -> ParamsInst {
//         ParamsInst {
//             x: self.v_x, y: self.v_y, z: self.v_z,
//             pitch: self.v_pitch, roll: self.v_roll, yaw: self.v_yaw
//         }
//     }

//     pub fn get_a(&self) -> ParamsInst {
//         ParamsInst {
//             x: self.a_x, y: self.a_y, z: self.a_z,
//             pitch: self.a_pitch, roll: self.a_roll, yaw: self.a_yaw
//         }
//     }
// }

/// Map rotor positions to connection numbers to ESC. Positions are associated with flight controls;
/// numbers are associated with motor control.
pub struct RotorMapping {
    // pub front_left: Rotor,
    // pub front_right: Rotor,
    // pub aft_left: Rotor,
    // pub aft_right: Rotor,
    pub r1: RotorPosition,
    pub r2: RotorPosition,
    pub r3: RotorPosition,
    pub r4: RotorPosition,
}

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% power.
/// Per our initial setup: R1 is aft left. R2 is front left. R3 is aft right. R4 is front right.
// todo: Discrete levels perhaps, eg multiples of the integer PWM ARR values.
#[derive(Default)]
pub struct RotorPower {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
    // For info on ratios, see `apply_controls()`.
    // pub pitch_ratio: f32, // scale of -1. to 1..
    // pub roll_ratio: f32,  // scale of -1. to 1..
    // pub yaw_ratio: f32,   // scale of -1. to 1..
    // pub throttle: f32,    // scale of -0. to 1.
    // todo: You may need to add ratios and throttle here.
}

// impl MulAssign<f32> for RotorPower {
//     fn mul_assign(&mut self, rhs: f32) {
//         self.p1 *= rhs;
//         self.p2 *= rhs;
//         self.p3 *= rhs;
//         self.p4 *= rhs;
//     }
// }
//
// impl DivAssign<f32> for RotorPower {
//     fn div_assign(&mut self, rhs: f32) {
//         self.p1 /= rhs;
//         self.p2 /= rhs;
//         self.p3 /= rhs;
//         self.p4 /= rhs;
//     }
// }

impl RotorPower {
    /// Convert rotor position to its associated power setting.
    fn by_rotor_num(&self, mapping: &RotorMapping) -> (f32, f32, f32, f32) {
        // todo: DRY
        let p1 = match mapping.r1 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        let p2 = match mapping.r2 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        let p3 = match mapping.r3 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        let p4 = match mapping.r4 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        (p1, p2, p3, p4)
    }

    /// Calculates total power. Used to normalize individual rotor powers when setting total
    /// power, eg from a thrust setting.
    pub fn total(&self) -> f32 {
        self.front_left + self.front_right + self.aft_left + self.aft_right
    }

    // /// Find the highest motor power level. Used for scaling power, so as
    // /// not to exceed full power on any motor, while preserving power ratios between motors.
    // pub fn highest(&self) -> f32 {
    //     let mut result = self.p1;
    //     if self.p2 > result {
    //         result = self.p2;
    //     }
    //     if self.p3 > result {
    //         result = self.p3;
    //     }
    //     if self.p4 > result {
    //         result = self.p4;
    //     }
    //
    //     result
    // }

    /// Send this power command to the rotors
    pub fn set(
        &mut self,
        mapping: &RotorMapping,
        rotor_timer_a: &mut Timer<TIM2>,
        rotor_timer_b: &mut Timer<TIM3>,
        arm_status: ArmStatus,
        dma: &mut Dma<DMA1>,
    ) {
        let (p1, p2, p3, p4) = self.by_rotor_num(mapping);

        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power_a(Rotor::R1, Rotor::R2, p1, p2, rotor_timer_a, dma);
                dshot::set_power_b(Rotor::R3, Rotor::R4, p3, p4, rotor_timer_b, dma);
            }
            ArmStatus::Disarmed => {
                dshot::stop_all(rotor_timer_a, rotor_timer_b, dma);
            }
        }
    }
}

/// Apply the throttle idle clamp. Generally for modes with manual power setting (?)
pub fn apply_throttle_idle(throttle: f32) -> f32 {
    if throttle < THROTTLE_IDLE_THRESH {
        THROTTLE_IDLE_POWER
    } else {
        throttle
    }
}

// todo: DMA for timer? How?

/// Estimate the (single-axis) rotor tilt angle (relative to a level aircraft) to produce
/// a desired amount of acceleration, with a given current velocity.
/// todo: Assume level flight?
/// // todo: come back to this later.
fn estimate_rotor_angle(a_desired: f32, v_current: f32, ac_properties: &AircraftProperties) -> f32 {
    let drag = ac_properties.drag_coeff * v_current; // todo
    1. / ac_properties.thrust_coeff; // todo
    0. // todo
}

/// Helper function for `apply_controls`. Sets specific rotor power based on commanded rates and throttle.
/// We split thihs out so we can call it a second time, in case of a clamp due to min or max power
/// on a rotor.
///
///  The ratios passed in params are on a scale between -1. and +1.
/// Throttle works differently - it's an overall scaler. If ratios are all 0, and power is 1., power for
/// all motors is 100%. No individual power level is allowed to be above 1, or below our idle power
/// setting.
fn calc_rotor_powers(pitch_rate: f32, roll_rate: f32, yaw_rate: f32, throttle: f32) -> RotorPower {
    // Start by setting all powers equal to the overall throttle setting.
    let mut front_left = throttle;
    let mut front_right = throttle;
    let mut aft_left = throttle;
    let mut aft_right = throttle;

    // todo: This is probably where you need gains etc.

    // todo: Should you adjust how you scale this based on the measured
    // todo rates? Maybe handle in PID.?

    let pitch_delta = ROTOR_HALF_PAIR_DELTA_MAX * pitch_rate;
    let roll_delta = ROTOR_HALF_PAIR_DELTA_MAX * roll_rate;
    let yaw_delta = ROTOR_HALF_PAIR_DELTA_MAX * yaw_rate;

    // Nose up for positive pitch.
    front_left += pitch_delta;
    front_right += pitch_delta;
    aft_left -= pitch_delta;
    aft_right -= pitch_delta;

    // Left side up for positive roll
    front_left += roll_delta;
    front_right -= roll_delta;
    aft_left += roll_delta;
    aft_right -= roll_delta;

    // todo: Check dir on yaw
    // Assumes props rotate inwards towards the front and back ends.
    // Yaw CCW for positive yaw.
    front_left -= yaw_delta;
    front_right += yaw_delta;
    aft_left += yaw_delta;
    aft_right -= yaw_delta;

    RotorPower {
        front_left,
        front_right,
        aft_left,
        aft_right,
    }
}

/// Set rotor speed for all 4 rotors. We model pitch, roll, and yaw based on target angular rates
/// in these areas. We modify power ratio between the appropriate propeller pairs to change these
/// parameters.
///
/// If a rotor exceeds min or max power settings, scale our angular rates uniformly until it's
/// at the threshold, while keeping average power, and rotor symettry intact.
///
/// Rates here are in terms of max/min - they're not in real units like radians/s!
pub fn apply_controls(
    mut pitch_rate: f32,
    mut roll_rate: f32,
    mut yaw_rate: f32,
    throttle: f32,
    mapping: &RotorMapping,
    current_pwr: &mut RotorPower,
    rotor_tim_a: &mut Timer<TIM2>,
    rotor_tim_b: &mut Timer<TIM3>,
    arm_status: ArmStatus,
    dma: &mut Dma<DMA1>,
) {
    let mut pwr = calc_rotor_powers(pitch_rate, roll_rate, yaw_rate, throttle);

    /// todo: Think in terms of torque? Include rotor arm?
    // todo: DRY
    let mut min_rotor = RotorPosition::FrontLeft;
    let mut min_pwr = pwr.front_left;

    if pwr.front_right < min_pwr {
        min_rotor = RotorPosition::FrontRight;
        min_pwr = pwr.front_right;
    }
    if pwr.aft_left < min_pwr {
        min_rotor = RotorPosition::AftLeft;
        min_pwr = pwr.aft_left;
    }
    if pwr.aft_right < min_pwr {
        min_rotor = RotorPosition::AftRight;
        min_pwr = pwr.aft_right;
    }

    // How we derive this scaling calculation, assuming we scale all rates equally:
    // FL: throttle + MAX_DELTA * rate_scaler * (pitch_rate + roll_rate + yaw_rate) = IDLE
    // FR: throttle + MAX_DELTA * rate_scaler * (+pitch_rate - roll_rate - yaw_rate) = IDLE
    // AL: throttle + MAX_DELTA * rate_scaler * (-pitch_rate + roll_rate - yaw_rate) = IDLE
    // AR: throttle + MAX_DELTA * rate_scaler * (-pitch_rate - roll_rate + yaw_rate) = IDLE
    // rate_scaler = (IDLE - throttle) / (MAX_DELTA * (-pitch_rate - roll_rate - yaw_rate))

    // Run the 2 limit checks sequentially, so in the (hopefully unusual case we
    // exceed both limits (eg with very high rates, and/or max delta?), we still
    // don't exceed either limit. Note that in the case of either being exceeded, we
    // attenuate rates, which helps on both ends.

    // Make a var for scaler, so we can use it in our max throttle calc too.
    if min_pwr < THROTTLE_IDLE_POWER {
        println!("Min power exceeded; reducing rates.");
        let rate_val = match min_rotor {
            RotorPosition::FrontLeft => pitch_rate + roll_rate + yaw_rate,
            RotorPosition::FrontRight => pitch_rate - roll_rate - yaw_rate,
            RotorPosition::AftLeft => -pitch_rate + roll_rate - yaw_rate,
            RotorPosition::AftRight => -pitch_rate - roll_rate + yaw_rate,
        };

        // todo: Make sure we're not putting another rotor over the threshold with this approach!

        let rate_scaler = (THROTTLE_IDLE_POWER - throttle) / (ROTOR_HALF_PAIR_DELTA_MAX * rate_val);
        pitch_rate *= rate_scaler;
        roll_rate *= rate_scaler;
        yaw_rate *= rate_scaler;

        pwr = calc_rotor_powers(pitch_rate, roll_rate, yaw_rate, throttle);
    }

    let mut max_rotor = RotorPosition::FrontLeft;
    let mut max_pwr = pwr.front_left;

    if pwr.front_right > max_pwr {
        max_rotor = RotorPosition::FrontRight;
        max_pwr = pwr.front_right;
    }
    if pwr.aft_left > max_pwr {
        max_rotor = RotorPosition::AftLeft;
        max_pwr = pwr.aft_left;
    }
    if pwr.aft_right > max_pwr {
        max_rotor = RotorPosition::AftRight;
        max_pwr = pwr.aft_right;
    }

    if max_pwr > THROTTLE_MAX_POWER {
        println!("Max power exceeded; reducing rates.");
        let rate_val = match max_rotor {
            // todo DRY
            RotorPosition::FrontLeft => pitch_rate + roll_rate + yaw_rate,
            RotorPosition::FrontRight => pitch_rate - roll_rate - yaw_rate,
            RotorPosition::AftLeft => -pitch_rate + roll_rate - yaw_rate,
            RotorPosition::AftRight => -pitch_rate - roll_rate + yaw_rate,
        };

        let rate_scaler = (THROTTLE_MAX_POWER - throttle) / (ROTOR_HALF_PAIR_DELTA_MAX * rate_val);
        // These rates below are already modified from the input args if required by exceeding
        // a min.
        pitch_rate *= rate_scaler;
        roll_rate *= rate_scaler;
        yaw_rate *= rate_scaler;

        pwr = calc_rotor_powers(pitch_rate, roll_rate, yaw_rate, throttle);
    }

    println!(
        "Rotor power: {} {} {} {}",
        pwr.front_left, pwr.front_right, pwr.aft_left, pwr.aft_right
    );

    pwr.set(mapping, rotor_tim_a, rotor_tim_b, arm_status, dma);
    *current_pwr = pwr;
}

/// Calculate the horizontal arget velocity (m/s), for a given distance (m) from a point horizontally.
pub fn enroute_speed_hor(dist: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if dist > 20. {
        max_v
    } else if dist > 10. {
        util::max(2., max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    }
}

/// Calculate the vertical target velocity (m/s), for a given distance (m) from a point vertically.
/// `dist` is postive if the aircraft altitude is below the set point; otherwise negative.
pub fn enroute_speed_ver(dist: f32, max_v: f32, z_agl: f32) -> f32 {
    // todo: fill this out. LUT?

    if z_agl < 7. {
        let mut result = util::max(3., max_v);

        if dist < 0. {
            result *= -1.;
        }
    }

    let dist_abs = util::abs(dist);

    let mut result = if dist_abs > 20. {
        max_v
    } else if dist_abs > 10. {
        util::max(2., max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    };

    if dist < 0. {
        result *= -1.;
    }
    result
}

// todo:
// /// Maybe we do map to acceleration at low speeds or power levels, but then go straight V pid?
// fn enroute_accel(desired_v, current_v) -> Option<f32> {
//}

/// Calculate power level to send to the ESC, from throttle setting. This is set up in a way to map
/// linearly between throttle setting and thrust, with an idle floor for power. Both values are on a scale of 0. to
/// 1., but the map isn't linear.
/// [This article](https://innov8tivedesigns.com/images/specs/Prop-Chart-Instructions-B.pdf) has
/// some plots of relevant info. This fn is based on the "Propeller Thrust vs Throttle Position" chart.
// todo: Fn, or LUT+interp? Maybe with CMSIS
pub fn power_from_throttle(
    throttle: f32,
    interp_inst: &dsp_sys::arm_linear_interp_instance_f32,
) -> f32 {
    // todo: We currently have fixed spacing in our LUT between throttle settings,
    // todo, but we need to reverse that!

    // todo: Is the setting we pass to ESC raw power, or is it RPM???
    // todo if RPM, you need to change this.

    // todo: Can't find this fn? Why??
    // dsp_sys::arm_linear_interp_f32(interp_inst, throttle)

    throttle // todo 1:1 mapping, and ignoring idle power.
}

/// Calculate the landing vertical velocity (m/s), for a given height  (m) above the ground.
pub fn landing_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5;
    }
    util::max(height / 4., max_v)
}

/// Calculate the takeoff vertical velocity (m/s), for a given height (m) above the ground.
pub fn takeoff_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.;
    }
    util::max(height / 4. + 0.01, max_v)
}

pub struct UnsuitableParams {}

/// Execute a profile designed to test PID and motor gain coefficients; update them.
pub fn calibrate_coeffs(params: &Params) -> Result<(), UnsuitableParams> {
    if params.s_z_agl < MIN_CAL_ALT {
        return Err(UnsuitableParams {});
    }

    Ok(())
}

pub fn handle_control_mode(input_mode_control: InputModeSwitch, input_mode: &mut InputMode, state_volatile: &mut StateVolatile) {
    state_volatile.input_mode_switch = input_mode_control; // todo: Do we need or use this field?

    *input_mode = match input_mode_control {
        InputModeSwitch::Acro => {
            InputMode::Acro
        }
        InputModeSwitch::AttitudeCommand => {
            if state_volatile.gps_attached {
                InputMode::Command
            } else {
                InputMode::Attitude
            }
        }
    }
}

