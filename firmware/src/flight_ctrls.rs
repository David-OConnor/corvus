//! This module contains code related to flight controls.

// todo: Split up further

use core::{
    f32::consts::TAU,
    ops::{Add, Mul, Sub},
};

use stm32_hal2::{
    pac::{TIM15, TIM3, TIM5},
    timer::Timer,
};

use cmsis_dsp_sys as sys;

// Don't execute the calibration procedure from below this altitude, eg for safety.
const MIN_CAL_ALT: f32 = 6.;

use crate::{pid::PidState, CtrlCoeffGroup, Location, Rotor, PWM_ARR};

#[derive(Default)]
// todo: Do we use this, or something more general like FlightCmd
pub struct CommandState {
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

/// Categories of control mode, in regards to which parameters are held fixed.
/// Use this in conjunction with `InputMode`, and control inputs.
pub enum AutopilotMode {
    /// There are no specific targets to hold
    None,
    /// Altitude is fixed at a given altimeter (AGL)
    AltHold(f32),
    /// Heading is fixed.
    HdgHold(f32), // hdg
    /// Altidude and heading are fixed
    AltHdgHold(f32, f32), // alt, hdg
    /// Continuously fly towards a path. Note that `pitch` and `yaw` for the
    /// parameters here correspond to the flight path; not attitude.
    VelocityVector(f32, f32), // pitch, yaw
    /// Fly direct to a point
    DirectToPoint(Location),
    /// The aircraft will fly a fixed profile between sequence points
    Sequence,
    /// Terrain following mode. Similar to TF radar in a jet. Require a forward-pointing sensor.
    /// todo: Add a forward (or angled) TOF sensor, identical to the downward-facing one?
    TerrainFollowing(f32), // AGL to hold
    /// Take off automatically
    Takeoff,
    /// Land automatically
    Land,
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
    /// and altitudes commanded by the controller.
    Command,
}

/// Stores the current manual inputs to the system. `pitch`, `yaw`, and `roll` are in range -1. to +1.
/// `throttle` is in range 0. to 1. Corresponds to stick positions on a controller, but can
/// also be used as a model for autonomous flight.
/// The interpretation of these depends on the current input mode.
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
// #[derive(Clone, Copy)]
// pub enum _ParamType {
//     /// Position
//     S,
//     /// Velocity
//     V,
//     /// Acceleration
//     A,
// }

// /// The quadcopter is an under-actuated system. So, some motions are coupled.
// /// Eg to command x or y motion, we just also command pitch and roll respectively.
// /// This enum specifies which we're using.
// #[derive(Clone, Copy)]
// pub enum _CtrlConstraint {
//     Xy,
//     PitchRoll,
// }
//
// /// A set of flight parameters to achieve and/or maintain. Similar values to `Parameters`,
// /// but Options, specifying only the parameters we wish to achieve.
// /// Note:
// #[derive(Clone, Default)]
// pub struct _FlightCmd {
//     pub x_roll: Option<(CtrlConstraint, ParamType, f32)>,
//     pub y_pitch: Option<(CtrlConstraint, ParamType, f32)>,
//     pub yaw: Option<(ParamType, f32)>,
//     pub thrust: Option<(ParamType, f32)>,
//     //
//     // pub x_roll: f32,
//     // pub y_pitch: f32,
//     // pub yaw: f32,
//     // pub thrust: f32,
// }
//
// impl FlightCmd {
//     /// Include manual inputs into the flight command. Ie, attitude velocities.
//     /// Note that this only includes
//     /// rotations: It doesn't affect altitude (or use the throttle input)
//     pub fn _from_inputs(inputs: &CtrlInputs, mode: InputMode) -> Self {
//         // todo: map to input range here, or elsewhere?
//         match mode {
//             InputMode::Acro => Self {
//                 y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::V, inputs.pitch)),
//                 x_roll: Some((CtrlConstraint::PitchRoll, ParamType::V, inputs.roll)),
//                 yaw: Some((ParamType::V, inputs.yaw)),
//                 thrust: Some((ParamType::V, inputs.throttle)),
//             },
//             InputMode::Attitude => Self {
//                 y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, inputs.pitch)),
//                 x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, inputs.roll)),
//                 yaw: Some((ParamType::V, inputs.yaw)),
//                 thrust: Some((ParamType::V, inputs.throttle)),
//             },
//             InputMode::Command => {
//                 // todo: Does this require an inner/outer loop scheme to manage
//                 // todo pitch and roll? Or 3 loop scheme for position, V, pitch/roll?
//                 // todo: EIther way: Don't use this yet!!
//                 // Note that this is for the `mid` flight command. The inner command (where we set
//                 // pitch and roll) is handled upstream, in the update ISR.
//                 Self {
//                     y_pitch: Some((CtrlConstraint::Xy, ParamType::V, inputs.pitch)),
//                     x_roll: Some((CtrlConstraint::Xy, ParamType::V, inputs.roll)),
//                     yaw: Some((ParamType::V, inputs.yaw)),
//                     // throttle could control altitude set point; make sure the throttle input
//                     // in this case is altitude.
//                     // todo: set Z upstream for this, eg you need to store and modify a commanded alt.
//                     // Note: This is misleading. We keep track of set point upstream.
//                     thrust: Some((ParamType::S, inputs.pitch)),
//                 }
//             }
//         }
//     }
//
//     // Command a basic hover. Maintains an altitude and pitch, and attempts to maintain position,
//     // but does revert to a fixed position.
//     // Alt is in AGL.
//     pub fn _hover(alt: f32) -> Self {
//         Self {
//             // Maintaining attitude isn't enough. We need to compensate for wind etc.
//             x_roll: Some((CtrlConstraint::Xy, ParamType::V, 0.)),
//             y_pitch: Some((CtrlConstraint::Xy, ParamType::V, 0.)),
//             thrust: Some((ParamType::V, 0.)),
//
//             // todo: Hover at a fixed position, using more advanced logic. Eg command an acceleration
//             // todo to reach it, then slow down and alt hold while near it?
//             ..Default::default()
//         }
//     }
//
//     /// Keep the device level and zero altitude change, with no other inputs.
//     pub fn _level() -> Self {
//         Self {
//             y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
//             x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
//             yaw: Some((ParamType::S, 0.)),
//             thrust: Some((ParamType::V, 0.)),
//             ..Default::default()
//         }
//     }
//
//     /// Maintains a hover in a specific location. lat and lon are in degrees. alt is in MSL.
//     pub fn _hover_geostationary(lat: f32, lon: f32, alt: f32) -> Self {
//         Default::default()
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
    pub fn set(&mut self, rotor_pwm_a: &mut Timer<TIM3>, rotor_pwm_b: &mut Timer<TIM5>) {
        self.clamp();

        set_power_a(Rotor::R1, self.p1, rotor_pwm_a);
        set_power_a(Rotor::R2, self.p2, rotor_pwm_a);
        set_power_b(Rotor::R3, self.p3, rotor_pwm_b);
        set_power_b(Rotor::R4, self.p4, rotor_pwm_b);
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

/// Set an individual rotor's power. Power ranges from 0. to 1.
fn set_power_a(rotor: Rotor, power: f32, timer: &mut Timer<TIM3>) {
    // todo: Use a LUT or something for performance.
    let arr_portion = power * PWM_ARR as f32;

    timer.set_duty(rotor.tim_channel(), arr_portion as u32);
}

// todo: DRY due to diff TIM types.
fn set_power_b(rotor: Rotor, power: f32, timer: &mut Timer<TIM5>) {
    // todo: Use a LUT or something for performance.
    let arr_portion = power * PWM_ARR as f32;

    timer.set_duty(rotor.tim_channel(), arr_portion as u32);
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
pub fn enroute_speed_ver(dist: f32, max_v: f32, z_agl: f32) -> f32 {
    // todo: fill this out. LUT?

    if z_agl < 7. {
        return crate::max(3., max_v);
    }

    if dist > 20. {
        max_v
    } else if dist > 10. {
        crate::max(2., max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    }
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
