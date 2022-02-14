///! This module contains code related to flight controls.
use core::{
    f32::TAU,
    ops::{Add, Mul, Sub},
};

use stm32_hal2::{pac::{TIM3, TIM5}, timer::Timer};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as sys;

use crate::{Location, Rotor, DT, PWM_ARR};

// todo: What should these be?
const INTEGRATOR_CLAMP_MIN: f32 = -10.;
const INTEGRATOR_CLAMP_MAX: f32 = 10.;

// one stage.
static mut FILTER_STATE_S_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_S_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_S_Z: [f32; 4] = [0.; 4];
static mut FILTER_STATE_S_PITCH: [f32; 4] = [0.; 4];
static mut FILTER_STATE_S_ROLL: [f32; 4] = [0.; 4];
static mut FILTER_STATE_S_YAW: [f32; 4] = [0.; 4];

static mut FILTER_STATE_V_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_V_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_V_Z: [f32; 4] = [0.; 4];
static mut FILTER_STATE_V_PITCH: [f32; 4] = [0.; 4];
static mut FILTER_STATE_V_ROLL: [f32; 4] = [0.; 4];
static mut FILTER_STATE_V_YAW: [f32; 4] = [0.; 4];

/// Cutoff frequency for our PID lowpass frequency,m in Hz
enum LowpassCuttoff {
    // todo: What values should these be?
    H500,
    H1k,
    H10k,
    H20k,
}

/// Coefficients and other configurable parameters for controls.
pub struct CtrlCoeffs {
    // These coefficients map desired change in flight parameters to rotor power change.
    // pitch, roll, and yaw s are in power / radians
    pid_pitch_s: f32,
    pid_roll_s: f32,
    pid_yaw_s: f32,
    pid_z_s: f32,

    k_p_s: f32,
    k_i_s: f32,
    k_d_s: f32,

    k_p_v: f32,
    k_i_v: f32,
    k_d_v: f32,

    pid_deriv_lowpass_cutoff: LowpassCuttoff,
}

impl Default for CtrlCoeffs {
    fn default() -> Self {
        Self {
            pid_pitch_s: 0.1,
            pid_roll_s: 0.1,
            pid_yaw_s: 0.1,
            pid_z_s: 0.1,

            // todo: Diff coeffs for diff cases;
            k_p_s: 0.1,
            k_i_s: 0.00,
            k_d_s: 0.00,
            k_p_v: 0.1,
            k_i_v: 0.00,
            k_d_v: 0.00,
            pid_deriv_lowpass_cutoff: LowpassCutoff::H1k,
        }
    }
}

#[derive(Default)]
// todo: Do we use this, or something more general like FlightCmd
pub struct CommandState {
    x: f32,
    y: f32,
    alt: f32, // m MSL
}

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop.
pub struct PidDerivFilters {
    // pub s_x: IirInstWrapper,
    // pub s_y: IirInstWrapper,
    // pub s_z: IirInstWrapper,
    // 
    // pub s_pitch: IirInstWrapper,
    // pub s_roll: IirInstWrapper,
    // pub s_yaw: IirInstWrapper,
    // 
    // // Velocity
    // pub v_x: IirInstWrapper,
    // pub v_y: IirInstWrapper,
    // pub v_z: IirInstWrapper,
    // 
    // pub v_pitch: IirInstWrapper,
    // pub v_roll: IirInstWrapper,
    // pub v_yaw: IirInstWrapper,
    // 
    // // Acceleration
    // pub a_x: IirInstWrapper,
    // pub a_y: IirInstWrapper,
    // pub a_z: IirInstWrapper,
    // 
    // pub a_pitch: IirInstWrapper,
    // pub a_roll: IirInstWrapper,
    // pub a_yaw: IirInstWrapper,
    // 
    pub mid_x: IirInstWrapper,
    pub mid_y: IirInstWrapper,
    pub mid_yaw: IirInstWrapper,
    pub mid_z: IirInstWrapper,
    
    pub inner_x: IirInstWrapper,
    pub inner_y: IirInstWrapper,
    pub inner_yaw: IirInstWrapper,
    pub inner_z: IirInstWrapper,
}

impl PidDerivFilters {
    pub fn new() -> Self {
        // todo: Instead of initing empty here, maybe we use the proper constructor?
        // todo: Maybe you can use this cleaner approach for Headphones too?

        // todo: Put useful params here.
        // filter_ = signal.iirfilter(1, 60, btype="lowpass", ftype="bessel", output="sos", fs=32_000)
        // coeffs = []
        // for row in filter_:
        //     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

        let coeffs = [
            0.00585605892206321,
            0.00585605892206321,
            0.0,
            0.9882878821558736,
            -0.0,
        ];

        let mut result = Self {
            // s_x: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // s_y: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // s_z: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // 
            // s_pitch: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // s_roll: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // s_yaw: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // 
            // // Velocity
            // v_x: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // v_y: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // v_z: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // 
            // v_pitch: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // v_roll: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // v_yaw: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // 
            // // Acceleration
            // a_x: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // a_y: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // a_z: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // 
            // a_pitch: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // a_roll: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // a_yaw: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // 
            mid_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            mid_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            mid_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            mid_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            
            inner_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            inner_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            inner_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            inner_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

        };

        unsafe {
            dsp_api::biquad_cascade_df1_init_f32(result.s_x, &coeffs, &mut FILTER_STATE_S_X);
            dsp_api::biquad_cascade_df1_init_f32(result.s_y, &coeffs, &mut FILTER_STATE_S_Y);
            dsp_api::biquad_cascade_df1_init_f32(result.s_z, &coeffs, &mut FILTER_STATE_S_Z);
            dsp_api::biquad_cascade_df1_init_f32(
                result.s_pitch,
                &coeffs,
                &mut FILTER_STATE_S_PITCH,
            );
            dsp_api::biquad_cascade_df1_init_f32(result.s_roll, &coeffs, &mut FILTER_STATE_S_ROLL);
            dsp_api::biquad_cascade_df1_init_f32(result.s_yaw, &coeffs, &mut FILTER_STATE_S_YAW);

            dsp_api::biquad_cascade_df1_init_f32(result.v_x, &coeffs, &mut FILTER_STATE_V_X);
            dsp_api::biquad_cascade_df1_init_f32(result.v_y, &coeffs, &mut FILTER_STATE_V_Y);
            dsp_api::biquad_cascade_df1_init_f32(result.v_z, &coeffs, &mut FILTER_STATE_V_Z);
            dsp_api::biquad_cascade_df1_init_f32(
                result.v_pitch,
                &coeffs,
                &mut FILTER_STATE_V_PITCH,
            );
            dsp_api::biquad_cascade_df1_init_f32(result.v_roll, &coeffs, &mut FILTER_STATE_V_ROLL);
            dsp_api::biquad_cascade_df1_init_f32(result.v_yaw, &coeffs, &mut FILTER_STATE_V_YAW);
        }

        result
    }
}

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
//
// /// Proportional, Integral, Derivative error, for flight parameter control updates.
// /// For only a single set (s, v, a). Note that e is the error betweeen commanded
// /// and measured, while the other terms include the PID coefficients (K_P) etc.
// /// So, `p` is always `e` x `K_P`.
// /// todo: Consider using Params, eg this is the error for a whole set of params.
// #[derive(Default)]
// pub struct PidState {
//     /// Measurement: Used for the derivative.
//     pub measurement: ParamsInst,
//     /// Error term. (No coeff multiplication). Used for the integrator
//     pub e: ParamsInst,
//     /// Proportional term
//     pub p: ParamsInst,
//     /// Integral term
//     pub i: ParamsInst,
//     /// Derivative term
//     pub d: ParamsInst,
// }
//
// impl PidState {
//     /// Anti-windup integrator clamp
//     pub fn anti_windup_clamp(&mut self) {
//         // todo: We should proably have separate terms for each param (x, y, z etc; s, v, a).
//         // todo: for now, single term.
//
//         // todo: DRY
//         if self.i.x > INTEGRATOR_CLAMP_MAX {
//             self.i.x = INTEGRATOR_CLAMP_MAX;
//         } else if self.i.x < INTEGRATOR_CLAMP_MIN {
//             self.i.x = INTEGRATOR_CLAMP_MIN;
//         }
//
//         if self.i.y > INTEGRATOR_CLAMP_MAX {
//             self.i.y = INTEGRATOR_CLAMP_MAX;
//         } else if self.i.y < INTEGRATOR_CLAMP_MIN {
//             self.i.y = INTEGRATOR_CLAMP_MIN;
//         }
//
//         if self.i.z > INTEGRATOR_CLAMP_MAX {
//             self.i.z = INTEGRATOR_CLAMP_MAX;
//         } else if self.i.z < INTEGRATOR_CLAMP_MIN {
//             self.i.z = INTEGRATOR_CLAMP_MIN;
//         }
//
//         if self.i.pitch > INTEGRATOR_CLAMP_MAX {
//             self.i.pitch = INTEGRATOR_CLAMP_MAX;
//         } else if self.i.pitch < INTEGRATOR_CLAMP_MIN {
//             self.i.pitch = INTEGRATOR_CLAMP_MIN;
//         }
//
//         if self.i.roll > INTEGRATOR_CLAMP_MAX {
//             self.i.roll = INTEGRATOR_CLAMP_MAX;
//         } else if self.i.roll < INTEGRATOR_CLAMP_MIN {
//             self.i.roll = INTEGRATOR_CLAMP_MIN;
//         }
//
//         if self.i.yaw > INTEGRATOR_CLAMP_MAX {
//             self.i.yaw = INTEGRATOR_CLAMP_MAX;
//         } else if self.i.yaw < INTEGRATOR_CLAMP_MIN {
//             self.i.yaw = INTEGRATOR_CLAMP_MIN;
//         }
//     }
// }

#[derive(Default)]
struct PidGroup {
    pitch: PidState2,
    roll: PidState2,
    yaw: PidState2,
    thrust: PidState2,
}

/// Proportional, Integral, Derivative error, for flight parameter control updates.
/// For only a single set (s, v, a). Note that e is the error betweeen commanded
/// and measured, while the other terms include the PID coefficients (K_P) etc.
/// So, `p` is always `e` x `K_P`.
/// todo: Consider using Params, eg this is the error for a whole set of params.
#[derive(Default)]
pub struct PidState2 {
    /// Measurement: Used for the derivative.
    pub measurement: f32,
    /// Error term. (No coeff multiplication). Used for the integrator
    pub e: f32,
    /// Proportional term
    pub p: f32,
    /// Integral term
    pub i: f32,
    /// Derivative term
    pub d: f32,
}

impl PidState2 {
    /// Anti-windup integrator clamp
    pub fn anti_windup_clamp(&mut self) {
        if self.i > INTEGRATOR_CLAMP_MAX {
            self.i = INTEGRATOR_CLAMP_MAX;
        } else if self.i < INTEGRATOR_CLAMP_MIN {
            self.i = INTEGRATOR_CLAMP_MIN;
        }
    }

    pub fn out(&self) -> f32 {
        // todo: Consider where you want this. This is fine for now; maybe even in general.
        self.p + self.i + self.d
    }
}

/// Mode used for control inputs. These are the three "industry-standard" modes.
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
#[derive(Default)]
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
    pub throttle: f32,
}

pub enum ParamType {
    /// Position
    S,
    /// Velocity
    V,
    /// Acceleration
    A,
}

/// The quadcopter is an under-actuated system. So, some motions are coupled.
/// Eg to command x or y motion, we just also command pitch and roll respectively.
/// This enum specifies which we're using.
pub enum CtrlConstraint {
    Xy,
    PitchRoll,
}

/// A set of flight parameters to achieve and/or maintain. Similar values to `Parameters`,
/// but Options, specifying only the parameters we wish to achieve.
/// Note:
#[derive(Clone, Default)]
pub struct FlightCmd {
    pub x_roll: Option<(CtrlConstraint, ParamType, f32)>,
    pub y_pitch: Option<(CtrlConstraint, ParamType, f32)>,
    pub yaw: Option<(ParamType, f32)>,
    pub thrust: Option<(ParamType, f32)>,
    //
    // pub x_roll: f32,
    // pub y_pitch: f32,
    // pub yaw: f32,
    // pub thrust: f32,
}

impl FlightCmd {
    /// Include manual inputs into the flight command. Ie, attitude velocities.
    /// Note that this only includes
    /// rotations: It doesn't affect altitude (or use the throttle input)
    pub fn from_inputs(inputs: CtrlInputs, mode: InputMode) -> Self {
        // todo: map to input range here, or elsewhere?
        match mode {
            InputMode::Acro => {
                Self {
                    y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::V, inputs.pitch)),
                    x_roll: Some((CtrlConstraint::PitchRoll, ParamType::V, inputs.roll)),
                    yaw: Some((ParamType::V, inputs.yaw)),
                    thrust: Some((ParamType::V, inputs.throttle)),
                }
            }
            InputMode::Attitude => {
                Self {
                    y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, inputs.pitch)),
                    x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, inputs.roll)),
                    yaw: Some((ParamType::V, inputs.yaw)),
                    thrust: Some((ParamType::V, inputs.throttle)),
                }
            }
            InputMode::Command => {
                // todo: Does this require an inner/outer loop scheme to manage
                // todo pitch and roll? Or 3 loop scheme for position, V, pitch/roll?
                // todo: EIther way: Don't use this yet!!
                // Note that this is for the `mid` flight command. The inner command (where we set
                // pitch and roll) is handled upstream, in the update ISR.
                Self {
                    y_pitch: Some((CtrlConstraint::Xy, ParamType::V, inputs.pitch)),
                    x_roll: Some((CtrlConstraint::Xy, ParamType::V, inputs.roll)),
                    yaw: Some((ParamType::V, inputs.yaw)),
                    // throttle could control altitude set point; make sure the throttle input
                    // in this case is altitude.
                    // todo: set Z upstream for this, eg you need to store and modify a commanded alt.
                    // Note: This is misleading. We keep track of set point upstream.
                    thrust: Some((ParamType::S, inputs.pitch)),
                }
            }
        }
    }

    // Command a basic hover. Maintains an altitude and pitch, and attempts to maintain position,
    // but does revert to a fixed position.
    // Alt is in AGL.
    pub fn _hover(alt: f32) -> Self {
        Self {
            // Maintaining attitude isn't enough. We need to compensate for wind etc.
            x_roll: Some((CtrlConstraint::Xy, ParamType::V, 0.)),
            y_pitch: Some((CtrlConstraint::Xy, ParamType::V, 0.)),
            thrust: Some((ParamType::V, 0.)),

            // todo: Hover at a fixed position, using more advanced logic. Eg command an acceleration
            // todo to reach it, then slow down and alt hold while near it?
            ..Default::default()
        }
    }

    /// Keep the device level and zero altitude change, with no other inputs.
    pub fn level() -> Self {
        Self {
            y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
            x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
            yaw: Some((ParamType::S, 0.)),
            thrust: Some((ParamType::V, 0.)),
            ..Default::default()
        }
    }

    /// Maintains a hover in a specific location. lat and lon are in degrees. alt is in MSL.
    pub fn hover_geostationary(lat: f32, lon: f32, alt: f32) -> Self {
        Default::default()
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

impl Mul for ParamsInst {
    type Output = Self;

    fn mul(self, other: f32) -> Self::Output {
        Self {
            x: self.x * other,
            y: self.y * other,
            z_msl: self.z_msl * other,
            z_agl: self.z_agl * other,
            pitch: self.pitch * other,
            roll: self.roll * other,
            yaw: self.yaw * other,
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
    current_pwr.p1 += pitch / PITCH_S_COEFF;
    current_pwr.p2 += pitch / PITCH_S_COEFF;
    current_pwr.p3 -= pitch / PITCH_S_COEFF;
    current_pwr.p4 -= pitch / PITCH_S_COEFF;

    current_pwr.p1 += roll / ROLL_S_COEFF;
    current_pwr.p2 -= roll / ROLL_S_COEFF;
    current_pwr.p3 -= roll / ROLL_S_COEFF;
    current_pwr.p4 += roll / ROLL_S_COEFF;

    current_pwr.p1 += yaw / YAW_S_COEFF;
    current_pwr.p2 -= yaw / YAW_S_COEFF;
    current_pwr.p3 += yaw / YAW_S_COEFF;
    current_pwr.p4 -= yaw / YAW_S_COEFF;

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

/// Calculate the landing vertical velocity (m/s), for a given height above the ground (m).
pub fn landing_speed(height: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5;
    }
    height / 4.
}

/// Calculate the takeoff vertical velocity (m/s), for a given height above the ground (m).
pub fn takeoff_speed(height: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.;
    }
    height / 4. + 0.01
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// Example: https://github.com/pms67/PID/blob/master/PID.c
pub fn calc_pid_error(
    set_pt: f32,
    measurement: f32,
    prev_pid: &PidState2,
    coeffs: CtrlCoeffs,
    filter: IirInstWrapper,
    // This `dt` is dynamic, since we don't necessarily run this function at a fixed interval.
    dt: f32,
) -> PidState2 {
    // Find appropriate control inputs using PID control.

    let error = set_pt - measurement;

    // todo: Determine if you want a deriv component at all; it's apparently not commonly used.

    // todo: Minor optomization: Store the constant terms once, and recall instead of calcing
    // todo them each time (eg the parts with DT, 2., tau.
    // https://www.youtube.com/watch?v=zOByx3Izf5U
    let error_p = coeffs.k_p * error_e_s;
    let error_i = coeffs.k_i * dt / 2. * (error + prev_pid.e) + prev_pid.i;
    // Derivative on measurement vice error, to avoid derivative kick.
    let error_d_prefilt = coeffs.k_d * (measurement - prev_pid.measurement) / dt;

    // todo: Avoid this DRY with a method on `filter`
    // let mut error_d_v_pitch = [0.];
    let mut error_d = [0.];
    unsafe {
        dsp_api::biquad_cascade_df1_f32(filters.inner, &[error_d_prefilt], &mut error_d, 1);
    }

    let mut error = PidState2 {
        measurement,
        e: error_e,
        p: error_p,
        i: error_i,
        d: error_d[0],
    };

    error.anti_windup_clamp();

    error
}

/// Adjust controls for a given flight command and PID error.
/// todo: Separate module for code that directly updates the rotors?
pub fn adjust_ctrls(
    pid_pitch: PidState2,
    pid_roll: PidState2,
    pid_yaw: PidState2,
    pid_pwr: PidState2,
    current_pwr: &mut RotorPower,
    rotor_pwm_a: &mut Timer<TIM3>,
    rotor_pwm_b: &mut Timer<TIM5>,
) {
    // todo: Is this fn superfluous?

    let pitch_adj = pid_pitch.out();
    let roll_adj = pid_roll.out();
    let yaw_adj = pid_yaw.out();
    let pwr_adj = pid_pwr.out();

    change_attitude(
        pitch_adj,
        roll_adj,
        yaw_adj,
        pwr_adj,
        current_pwr, // modified in place, and therefore upstream.
        rotor_pwm_a,
        rotor_pwm_b,
    );
}
