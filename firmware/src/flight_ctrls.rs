//! This module contains code related to flight controls. Code specific to the PID
//! loops are in the `pid` module.

// todo: Split up further

use core::{
    f32::consts::TAU,
    ops::{Add, Mul, Sub, DivAssign, MulAssign},
};

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::Timer,
};

use cmsis_dsp_sys as sys;

use crate::{dshot, pid::PidState, CtrlCoeffGroup, Location, Rotor, UserCfg};

// Don't execute the calibration procedure from below this altitude, eg for safety.
const MIN_CAL_ALT: f32 = 6.;

/// Our input ranges for the 4 controls.
const PITCH_RNG: (f32, f32) = (-1., 1.);
const ROLL_RNG: (f32, f32) = (-1., 1.);
const YAW_RNG: (f32, f32) = (-1., 1.);
const THRUST_RNG: (f32, f32) = (0., 1.);

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

    pub fn calc_thrust(&self, input: f32) -> f32 {
        map_linear(input, THRUST_RNG, self.power_level)
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
    /// Recover to stable, altitude-holding flight. Generally initiated by a "panic button"-style
    /// switch activation
    pub recover: Option<f32>, // value is MSL alt to hold, eg our alt at time of command.
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

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% power.
/// Rotor 1 is aft right. R2 is front right. R3 is aft left. R4 is front left.
// todo: Discrete levels perhaps, eg multiples of the integer PWM ARR values.
#[derive(Default)]
pub struct RotorPower {
    pub p1: f32,
    pub p2: f32,
    pub p3: f32,
    pub p4: f32,

    // For info on ratios, see `apply_controls()`.
    pub pitch_ratio: f32, // scale of -1. to 1..
    pub roll_ratio: f32, // scale of -1. to 1..
    pub yaw_ratio: f32, // scale of -1. to 1..
    pub throttle: f32, // scale of -0. to 1.
    // todo: You may need to add ratios and throttle here.
}

impl MulAssign<f32> for RotorPower {
    fn mul_assign(&mut self, rhs: f32) {
        self.p1 *= rhs;
        self.p2 *= rhs;
        self.p3 *= rhs;
        self.p4 *= rhs;
    }
}

impl DivAssign<f32> for RotorPower {
    fn div_assign(&mut self, rhs: f32) {
        self.p1 /= rhs;
        self.p2 /= rhs;
        self.p3 /= rhs;
        self.p4 /= rhs;
    }
}

impl RotorPower {
    /// Calculates total power. Used to normalize individual rotor powers when setting total
    /// power, eg from a thrust setting.
    pub fn total(&self) -> f32 {
        self.p1 + self.p2 + self.p3 + self.p4
    }
    
    /// Find the highest motor power level. Used for scaling power, so as
    /// not to exceed full power on any motor, while preserving power ratios between motors.
    pub fn highest(&self) -> f32 {
        let mut result = self.p1;
        if self.p2 > highest_v {
            result = self.p2;
        }
        if self.p3 > highest_v {
            result = self.p3;
        }
        if self.p4 > highest_v {
            result = self.p4;
        }

        result
    }

    /// Send this power command to the rotors
    pub fn set(
        &mut self,
        rotor_timer_a: &mut Timer<TIM2>,
        rotor_timer_b: &mut Timer<TIM3>,
        dma: &mut Dma<DMA1>,
    ) {
        dshot::set_power_a(Rotor::R1, Rotor::R2, self.p1, self.p2, rotor_timer_a, dma);
        // dshot::set_power_a(Rotor::R2, self.p2, rotor_timer_a, dma);
        dshot::set_power_b(Rotor::R3, Rotor::R4, self.p3, self.p4, rotor_timer_b, dma);
        // dshot::set_power_b(Rotor::R4, self.p4, rotor_timer_b, dma);
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

/// Set rotor speed for all 4 rotors. We model pitch, roll, and yaw based on target angular rates
/// in these areas, but modifying the power ratio between the appropriate propeller pairs. The ratios
/// passed in params are on a scale between -1. and +1. The inputs are the ratio of the respective rotor sides.
/// Throttle works differently - it's an overall scaler. If ratios are all 0, and power is 1., power for
/// all motors is 100%. No individual power level is allowed to be above 1. Rather than clip, we
/// scale other rotor power levels to leave the one set to exceed 1. at 1.
///
/// Pitch = 0. means equal front/aft power. Pitch = 1.0 means aft/front rotor power is 100%. Pitch = -1.
/// means aft / front power is 0%.... pitch_ratio = 1. means max fwd rotation. roll_ratio = 1. means max right rotation.
/// yaw_ratio 1.0 means max clockwise rotation.
/// Rotor 1 is aft right. R2 is front right. R3 is aft left. R4 is front left.
///
/// Keep in mind: We're trying to map stick position (or more generally, commanded angular rate) to
/// ratios between prop pairs.
pub fn apply_controls(
    pitch_ratio: f32,
    roll_ratio: f32,
    yaw_ratio: f32,
    throttle: f32,
    current_pwr: &mut RotorPower,
    rotor_tim_a: &mut Timer<TIM2>,
    rotor_tim_b: &mut Timer<TIM3>,
    dma: &mut Dma<DMA1>,
) {
    // todo: Store current ratios somehwere probably. Maybe even ditch current power, and/or replace
    // todo it with ratios.
    let mut pwr = RotorPower::default();
    // todo: Do you want a linear mapping between power ratio like this, or is it non-linear?
    // todo: Make some type of trig function?
    let aft = pitch_ratio / 2. + 0.5;
    let front = -pitch_ratio / 2. + 0.5;
    let left = roll_ratio / 2. + 0.5;
    let right = -roll_ratio / 2. + 0.5;
    let ccw = yaw_ratio / 2. + 0.5;
    let cw = -yaw_ratio / 2. + 0.5;

    pwr.p1 += aft;
    pwr.p3 += aft;
    pwr.p2 += front;
    pwr.p4 += front;

    pwr.p3 += left;
    pwr.p4 += left;
    pwr.p1 += right;
    pwr.p2 += right;

    // todo: Check direction on this. Will be obvious if backwards.
    pwr.p2 += ccw;
    pwr.p3 += ccw;
    pwr.p1 += cw;
    pwr.p4 += cw;

    // Normalize using throttle, so the average power is equal to throttle x 4 rotors.
    pwr *= 4. * throttle / pwr.total();

    // If any rotor power is set to exceed 1.0 (Eg due to high power setting in conjunction with
    // one or more high ratios), scale back all powers so the max is at 1. (preserves ratios as a priority
    // over preserving power level)
    let mut highest_v = pwr.highest();
    if highest_v > 1. {
        pwr /= highest_v;
    }

    pwr.set(rotor_tim_a, rotor_tim_b, dma);

    *current_pwr = pwr;
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
pub fn calibrate_coeffs(params: &Params) -> Result<(), UnsuitableParams> {
    if params.s_z_agl < MIN_CAL_ALT {
        return Err(UnsuitableParams {});
    }

    Ok(())
}
