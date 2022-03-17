//! This module contains code related to flight controls.

// todo: Split up further

use core::{
    f32::consts::TAU,
    ops::{Add, Mul, Sub},
};

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::Timer,
};

use cmsis_dsp_sys as sys;

use num_traits::float::FloatCore; // Absolute value.

use crate::{drivers::crsf, dshot, pid::PidState, CtrlCoeffGroup, Location, Rotor, UserCfg};

// Don't execute the calibration procedure from below this altitude, eg for safety.
const MIN_CAL_ALT: f32 = 6.;

/// Our input ranges for the 4 controls.
const PITCH_RNG: (f32, f32) = (-1., 1.);
const ROLL_RNG: (f32, f32) = (-1., 1.);
const YAW_RNG: (f32, f32) = (-1., 1.);
const THRUST_RNG: (f32, f32) = (-0., 1.);

// Minimium speed before auto-yaw will engate. (if we end up setting up auto-yaw to align flight path
// with heading)
// todo: Maybe this could also be used if we end up setting up auto-yaw as sideway-accel cancellation?
// todo, and this would be the min *fwd* velocity?
pub const YAW_ASSIST_MIN_SPEED: f32 = 0.5; // m/s

// if coeff = 0.5, if accel is 1 m/s^2, yaw correction is 1/2 rad/s
// angular velocity / accel: (radians/s) / (m/s^2) = radiants x s / m
pub const YAW_ASSIST_COEFF: f32 = 0.1;

/// We use this buffer for DMA transfers of IMU readings. Note that reading order is different
/// between different IMUs, due to their reg layout, and consecutive reg reads. In both cases, 6 readings,
/// each with 2 bytes each.
static mut IMU_BUF: [u8; 12] = [0; 12];

/// Utility function to linearly map an input value to an output
fn map_linear(val: f32, range_in: (f32, f32), range_out: (f32, f32)) -> f32 {
    // todo: You may be able to optimize calls to this by having the ranges pre-store
    // todo the total range vals.
    let portion = (val - range_in.0) / (range_in.1 - range_in.0);

    portion * (range_out.1 - range_out.0) + range_out.0
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
    pub fn calc_pitch_rate(&self, input: f32) -> f32 {
        map_linear(input, PITCH_RNG, self.pitch_rate)
    }

    pub fn calc_roll_rate(&self, input: f32) -> f32 {
        map_linear(input, ROLL_RNG, self.roll_rate)
    }

    pub fn calc_yaw_rate(&self, input: f32) -> f32 {
        map_linear(input, YAW_RNG, self.yaw_rate)
    }

    pub fn calc_pitch_angle(&self, input: f32) -> f32 {
        map_linear(input, PITCH_RNG, self.pitch_angle)
    }

    pub fn calc_roll_angle(&self, input: f32) -> f32 {
        map_linear(input, ROLL_RNG, self.roll_angle)
    }

    pub fn calc_yaw_angle(&self, input: f32) -> f32 {
        map_linear(input, YAW_RNG, self.yaw_angle)
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

/// Indicates master motor arm status. Used for both pre arm, and arm. If either is
/// set to `Disarmed`, the motors will not spin (or stop spinning immediately).
#[derive(Clone, Copy, PartialEq)]
pub enum ArmStatus {
    /// Motors are [pre]disarmed
    Disarmed,
    /// Motors are [pre]armed
    Armed,
}

impl Default for ArmStatus {
    fn default() -> Self {
        Self::Disarmed
    }
}

#[derive(Default)]
pub struct CommandState {
    pub pre_armed: ArmStatus,
    pub armed: ArmStatus,
    pub x: f32,
    pub y: f32,
    pub alt: f32, // m MSL
    pub loiter_set: bool,
}

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

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
    // todo: Do yaw assist and roll assist make sense for Attitude mode.
    // todo: Do you even want attitude mode as an option??
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
}

/// Mode used for control inputs. These are the three "industry-standard" modes.
#[derive(Clone, Copy)]
pub enum InputMode {
    /// Rate, also know as manual, hard or Acro. Attitude and power stay the same after
    /// releasing controls.
    Acro,
    /// Attitude also know as self-level, angle, or Auto-level. Attitude resets to a level
    /// hover after releasing controls.  When moving the
    /// roll/pitch stick to its maximum position, the drone will also reach the maximum angle
    /// it’s allowed to tilt (defined by the user), and it won’t flip over. As you release the
    /// stick back to centre, the aircraft will also return to its level position.
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

impl CtrlInputs {
    /// Get manual inputs from the radio. Map from the radio input values to our standard values of
    /// 0. to 1. (throttle), and -1. to 1. (pitch, roll, yaw).
    pub fn get_manual_inputs(cfg: &UserCfg) -> Self {
        // todo: Get radio input here.!
        let pitch = 0.;
        let roll = 0.;
        let yaw = 0.;
        let thrust = 0.;

        Self {
            pitch: map_linear(pitch, cfg.pitch_input_range, PITCH_RNG),
            roll: map_linear(roll, cfg.roll_input_range, ROLL_RNG),
            yaw: map_linear(yaw, cfg.pitch_input_range, YAW_RNG),
            thrust: map_linear(thrust, cfg.throttle_input_range, THRUST_RNG),
        }
    }
}

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

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% PWM duty cycle.
// todo: Discrete levels perhaps, eg multiples of the integer PWM ARR values.
#[derive(Default)]
pub struct RotorPower {
    pub p1: f32,
    pub p2: f32,
    pub p3: f32,
    pub p4: f32,
}

impl RotorPower {
    // todo: Remove `total()` if you end up with a current sense from ESC instead.
    pub fn total(&self) -> f32 {
        self.p1 + self.p2 + self.p3 + self.p4
    }

    /// Limit power to a range between 0 and 1.
    fn clamp(&mut self) {
        if self.p1 < 0. {
            self.p1 = 0.;
        } else if self.p1 > 1. {
            self.p1 = 1.;
        }

        if self.p2 < 0. {
            self.p2 = 0.;
        } else if self.p2 > 1. {
            self.p2 = 1.;
        }

        if self.p3 < 0. {
            self.p3 = 0.;
        } else if self.p3 > 1. {
            self.p3 = 1.;
        }

        if self.p4 < 0. {
            self.p4 = 0.;
        } else if self.p4 > 1. {
            self.p4 = 1.;
        }
    }

    /// Send this power command to the rotors
    pub fn set(
        &mut self,
        rotor_timer_a: &mut Timer<TIM3>,
        rotor_timer_b: &mut Timer<TIM5>,
        dma: &mut Dma<DMA1>,
    ) {
        self.clamp();

        dshot::set_power_a(Rotor::R1, self.p1, rotor_timer_a, dma);
        dshot::set_power_a(Rotor::R2, self.p2, rotor_timer_a, dma);
        dshot::set_power_b(Rotor::R3, self.p3, rotor_timer_b, dma);
        dshot::set_power_b(Rotor::R4, self.p4, rotor_timer_b, dma);
    }
}

// todo: DMA for timer? How?

/// Estimate the (single-axis) rotor tilt angle (relative to a level aircraft) to produce
/// a desired amount of acceleration, with a given current velocity.
/// todo: Assume level flight?
/// // todo: come back to this later.
fn estimate_rotor_angle(
    a_desired: f32,
    v_current: f32,
    ac_properties: &crate::AircraftProperties,
) -> f32 {
    let drag = ac_properties.drag_coeff * v_current; // todo
    1. / ac_properties.thrust_coeff; // todo
    0. // todo
}

/// Set rotor speed for all 4 rotors, based on 6-axis control adjustments. Params here are power levels,
/// from 0. to 1. This translates and applies settings to rotor controls. Modifies existing settings
/// with the value specified.
/// todo: This needs conceptual/fundamental work
fn change_attitude(
    pitch: f32,
    roll: f32,
    yaw: f32,
    throttle: f32,
    current_pwr: &mut RotorPower,
    rotor_pwm_a: &mut Timer<TIM3>,
    rotor_pwm_b: &mut Timer<TIM5>,
) {
    current_pwr.p1 += pitch;
    current_pwr.p2 += pitch;
    current_pwr.p3 -= pitch;
    current_pwr.p4 -= pitch;

    current_pwr.p1 += roll;
    current_pwr.p2 -= roll;
    current_pwr.p3 -= roll;
    current_pwr.p4 += roll;

    current_pwr.p1 += yaw;
    current_pwr.p2 -= yaw;
    current_pwr.p3 += yaw;
    current_pwr.p4 -= yaw;

    current_pwr.p1 *= throttle;
    current_pwr.p2 *= throttle;
    current_pwr.p3 *= throttle;
    current_pwr.p4 *= throttle;

    current_pwr.set(rotor_pwm_a, rotor_pwm_b);
}

/// Calculate the horizontal arget velocity (m/s), for a given distance (m) from a point horizontally.
pub fn enroute_speed_hor(dist: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if dist > 20. {
        max_v
    } else if dist > 10. {
        crate::max(2., max_v)
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
        let mut result = crate::max(3., max_v);

        if dist < 0. {
            result *= -1.;
        }
    }

    let dist_abs = crate::abs(dist);

    let mut result = if dist_abs > 20. {
        max_v
    } else if dist_abs > 10. {
        crate::max(2., max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    };

    if dist < 0. {
        result *= -1.;
    }
    result
}

/// Calculate the landing vertical velocity (m/s), for a given height  (m) above the ground.
pub fn landing_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5;
    }
    crate::max(height / 4., max_v)
}

/// Calculate the takeoff vertical velocity (m/s), for a given height (m) above the ground.
pub fn takeoff_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.;
    }
    crate::max(height / 4. + 0.01, max_v)
}

pub struct UnsuitableParams {}

/// Execute a profile designed to test PID and motor gain coefficients; update them.
pub fn calibrate_coeffs(params: &Params, timer: &mut Timer<TIM15>) -> Result<(), UnsuitableParams> {
    if params.s_z_agl < MIN_CAL_ALT {
        return Err(UnsuitableParams {});
    }

    Ok(())
}

/// Adjust controls for a given flight command and PID error.
/// todo: Separate module for code that directly updates the rotors?
pub fn apply_ctrls(
    pid_pitch: &PidState,
    pid_roll: &PidState,
    pid_yaw: &PidState,
    pid_pwr: &PidState,
    coeffs: &CtrlCoeffGroup,
    current_pwr: &mut RotorPower,
    rotor_pwm_a: &mut Timer<TIM3>,
    rotor_pwm_b: &mut Timer<TIM5>,
) {
    // todo: Is this fn superfluous?

    let pitch_adj = pid_pitch.out() * coeffs.gain_pitch;
    let roll_adj = pid_roll.out() * coeffs.gain_roll;
    let yaw_adj = pid_yaw.out() * coeffs.gain_yaw;
    let throttle_adj = pid_pwr.out() * coeffs.gain_thrust;

    change_attitude(
        pitch_adj,
        roll_adj,
        yaw_adj,
        throttle_adj,
        current_pwr, // modified in place, and therefore upstream.
        rotor_pwm_a,
        rotor_pwm_b,
    );
}
