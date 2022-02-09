///! This module contains code related to flight controls.
use core::ops::{Add, Sub, Mul};
use stm32_hal2::{pac::TIM2, timer::Timer};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as sys;

use crate::{Rotor, DT, PWM_ARR};

// These coefficients map desired change in flight parameters to rotor power change.
// pitch, roll, and yaw s are in power / radians
const PITCH_S_COEFF: f32 = 0.1;
const ROLL_S_COEFF: f32 = 0.1;
const YAW_S_COEFF: f32 = 0.1;

const Z_V_COEFF: f32 = 0.1;

// PID "constants
const K_P: f32 = 0.1;
const K_I: f32 = 0.05;
const K_D: f32 = 0.;

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

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop.
pub struct PidDerivFilters {
    pub s_x: IirInstWrapper,
    pub s_y: IirInstWrapper,
    pub s_z: IirInstWrapper,

    pub s_pitch: IirInstWrapper,
    pub s_roll: IirInstWrapper,
    pub s_yaw: IirInstWrapper,

    // Velocity
    pub v_x: IirInstWrapper,
    pub v_y: IirInstWrapper,
    pub v_z: IirInstWrapper,

    pub v_pitch: IirInstWrapper,
    pub v_roll: IirInstWrapper,
    pub v_yaw: IirInstWrapper,

    // Acceleration
    pub a_x: IirInstWrapper,
    pub a_y: IirInstWrapper,
    pub a_z: IirInstWrapper,

    pub a_pitch: IirInstWrapper,
    pub a_roll: IirInstWrapper,
    pub a_yaw: IirInstWrapper,
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

        let coeffs = [0.00585605892206321, 0.00585605892206321, 0.0, 0.9882878821558736, -0.0];

        let mut result = Self {
            s_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            s_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            s_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            s_pitch: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            s_roll: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            s_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            // Velocity
            v_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            v_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            v_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            v_pitch: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            v_roll: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            v_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            // Acceleration
            a_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            a_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            a_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            a_pitch: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            a_roll: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            a_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            dsp_api::biquad_cascade_df1_init_f32(result.s_x, &coeffs, &mut FILTER_STATE_S_X);
            dsp_api::biquad_cascade_df1_init_f32(result.s_y, &coeffs, &mut FILTER_STATE_S_Y);
            dsp_api::biquad_cascade_df1_init_f32(result.s_z, &coeffs, &mut FILTER_STATE_S_Z);
            dsp_api::biquad_cascade_df1_init_f32(result.s_pitch, &coeffs, &mut FILTER_STATE_S_PITCH);
            dsp_api::biquad_cascade_df1_init_f32(result.s_roll, &coeffs, &mut FILTER_STATE_S_ROLL);
            dsp_api::biquad_cascade_df1_init_f32(result.s_yaw, &coeffs, &mut FILTER_STATE_S_YAW);

            dsp_api::biquad_cascade_df1_init_f32(result.v_x, &coeffs, &mut FILTER_STATE_V_X);
            dsp_api::biquad_cascade_df1_init_f32(result.v_y, &coeffs, &mut FILTER_STATE_V_Y);
            dsp_api::biquad_cascade_df1_init_f32(result.v_z, &coeffs, &mut FILTER_STATE_V_Z);
            dsp_api::biquad_cascade_df1_init_f32(result.v_pitch, &coeffs, &mut FILTER_STATE_V_PITCH);
            dsp_api::biquad_cascade_df1_init_f32(result.v_roll, &coeffs, &mut FILTER_STATE_V_ROLL);
            dsp_api::biquad_cascade_df1_init_f32(result.v_yaw, &coeffs, &mut FILTER_STATE_V_YAW);
        }

        result
    }
}

/// Categories of control mode, in regards to which parameters are held fixed.
/// Use this in conjunction with `InputMode`, and control inputs.
enum AutopilotMode {
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
    DirectToPoint(Vector),
    /// The aircraft will fly a fixed profile between sequence points
    Sequence,
    /// Terrain following mode. Similar to TF radar in a jet. Require a forward-pointing sensor.
    TerrainFollowing(f32), // AGL to hold
}

/// Proportional, Integral, Derivative error, for flight parameter control updates.
/// For only a single set (s, v, a). Note that e is the error betweeen commanded
/// and measured, while the other terms include the PID coefficients (K_P) etc.
/// So, `p` is always `e` x `K_P`.
/// todo: Consider using Params, eg this is the error for a whole set of params.
#[derive(Default)]
pub struct PidState {
    /// Measurement: Used for the derivative.
    pub measurement: ParamsInst,
    /// Error term. (No coeff multiplication). Used for the integrator
    pub e: ParamsInst,
    /// Proportional term
    pub p: ParamsInst,
    /// Integral term
    pub i: ParamsInst,
    /// Derivative term
    pub d: ParamsInst,
}

impl PidState {
    /// Anti-windup integrator clamp
    pub fn anti_windup_clamp(&mut self) {
        // todo: We should proably have separate terms for each param (x, y, z etc; s, v, a).
        // todo: for now, single term.

        // todo: DRY
        if self.i.x > INTEGRATOR_CLAMP_MAX {
            self.i.x = INTEGRATOR_CLAMP_MAX;
        } else if self.i.x < INTEGRATOR_CLAMP_MIN {
            self.i.x = INTEGRATOR_CLAMP_MIN;
        }

        if self.i.y > INTEGRATOR_CLAMP_MAX {
            self.i.y = INTEGRATOR_CLAMP_MAX;
        } else if self.i.y < INTEGRATOR_CLAMP_MIN {
            self.i.y = INTEGRATOR_CLAMP_MIN;
        }

        if self.i.z > INTEGRATOR_CLAMP_MAX {
            self.i.z = INTEGRATOR_CLAMP_MAX;
        } else if self.i.z < INTEGRATOR_CLAMP_MIN {
            self.i.z = INTEGRATOR_CLAMP_MIN;
        }

        if self.i.pitch > INTEGRATOR_CLAMP_MAX {
            self.i.pitch = INTEGRATOR_CLAMP_MAX;
        } else if self.i.pitch < INTEGRATOR_CLAMP_MIN {
            self.i.pitch = INTEGRATOR_CLAMP_MIN;
        }

        if self.i.roll > INTEGRATOR_CLAMP_MAX {
            self.i.roll = INTEGRATOR_CLAMP_MAX;
        } else if self.i.roll < INTEGRATOR_CLAMP_MIN {
            self.i.roll = INTEGRATOR_CLAMP_MIN;
        }

        if self.i.yaw > INTEGRATOR_CLAMP_MAX {
            self.i.yaw = INTEGRATOR_CLAMP_MAX;
        } else if self.i.yaw < INTEGRATOR_CLAMP_MIN {
            self.i.yaw = INTEGRATOR_CLAMP_MIN;
        }
    }
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
enum CtrlConstraint {
    Xy,
    PitchRoll,
}

/// A set of flight parameters to achieve and/or maintain. Similar values to `Parameters`,
/// but Options, specifying only the parameters we wish to achieve.
/// Note:
#[derive(Default)]
pub struct FlightCmd {
    pub x_roll: Option<(CtrlConstraint, ParamType, f32)>,
    pub y_pitch: Option<(CtrlConstraint, ParamType, f32)>,
    pub z: Option<(ParamType, f32)>,
    pub yaw: Option<(ParamType, f32)>,
}

impl FlightCmd {
    /// Include manual inputs into the flight command. Ie, attitude velocities.
    /// Note that this only includes
    /// rotations: It doesn't affect altitude (or use the throttle input)
    pub fn add_inputs(&mut self, inputs: CtrlInputs) {
        self.y_pitch = match self.pitch {
            Some(v) => Some((CtrlConstraint::PitchRoll, ParamType::V, v + inputs.pitch)),
            None => Some((CtrlConstraint::PitchRoll, ParamType::V, inputs.pitch)),
        };

        self.x_roll = match self.roll {
            Some(v) => Some((CtrlConstraint::PitchRoll, ParamType::V, v + inputs.roll)),
            None => Some((CtrlConstraint::PitchRoll, ParamType::V, inputs.roll)),
        };

        self.yaw = match self.yaw {
            Some(v) => Some((ParamType::V, v + inputs.yaw)),
            None => Some((ParamType::V, inputs.yaw)),
        };
    }

    // Command a basic hover. Maintains an altitude and pitch, and attempts to maintain position,
    // but does revert to a fixed position.
    // Alt is in AGL.
    pub fn _hover(alt: f32) -> Self {
        Self {
            // Maintaining attitude isn't enough. We need to compensate for wind etc.
            x_roll: Some((CtrlConstraint::Xy, ParamType::V, 0.)),
            y_pitch: Some((CtrlConstraint::Xy, ParamType::V, 0.)),
            z: Some((ParamType::V, 0.)),

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
            z: Some((ParamType::V, 0.)),
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
    pub z: f32,
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
            z: self.z + other.z,
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
            z: self.z - other.z,
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
            z: self.z * other,
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
    /// Altitude, in AGL. We treat MSL as a varying offset from this.
    pub s_z: f32,

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
    pub fn set(&mut self, rotor_timer: &mut Timer<TIM2>) {
        self.clamp();

        set_power(Rotor::R1, self.p1, rotor_timer);
        set_power(Rotor::R2, self.p2, rotor_timer);
        set_power(Rotor::R3, self.p3, rotor_timer);
        set_power(Rotor::R4, self.p4, rotor_timer);
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
    rotor_timer: &mut Timer<TIM2>,
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

    current_pwr.set(rotor_timer);
}

/// Set an individual rotor's power. Power ranges from 0. to 1.
fn set_power(rotor: Rotor, power: f32, timer: &mut Timer<TIM2>) {
    // todo: Use a LUT or something for performance.
    let arr_portion = power * PWM_ARR as f32;

    timer.set_duty(rotor.tim_channel(), arr_portion as u32);
}

/// Calculate the vertical velocity (m/s), for a given height above the ground (m).
fn landing_speed(height: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5
    }
    height / 4.
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// Example: https://github.com/pms67/PID/blob/master/PID.c
/// todo: COnsider consolidating these instead of sep for s, v, a.
pub fn calc_pid_error(
    params: &Params,
    flight_cmd: &FlightCmd,
    prev_pid_s: &PidState,
    prev_pid_v: &PidState,
    filters: PidDerivFilters,
) -> (PidState, PidState) {
    // Find appropriate control inputs using PID control.


    match flight_cmd.z {
        Some(cmd) => ,
        None =>
    }
    let error_e_v = K_P * ParamsInst {
        z: flight_cmd.z.unwrap_or((ParamType::V, 0.)).1 - params.v_z,
        ..Default::default()
    };

    let error_e_s = K_P * ParamsInst {
        pitch: flight_cmd.y_pitch.unwrap_or((CtrlConstraint::PitchRoll, ParamType::S, 0.)).2 - params.s_pitch,
        roll: flight_cmd.x_roll.unwrap_or((CtrlConstraint::PitchRoll, ParamType::S, 0.)).2 - params.s_roll,
        yaw:  flight_cmd.yaw.unwrap_or((ParamType::S, 0.)).1 - params.s_yaw,
        ..Default::default()
    };

    // todo: Apply lowpass to derivative term. (Anywhere in its linear sequence)

    // todo: Determine if you want a deriv component at all; it's apparently not commonly used.

    // todo: Minor optomization: Store the constant terms once, and recall instead of calcing
    // todo them each time (eg the parts with DT, 2., tau.
    // https://www.youtube.com/watch?v=zOByx3Izf5U
    // todo: What is tau??
    let error_p_s = K_P * error_e_s;
    let error_i_s = K_I * DT/2. * (error_e_s + prev_pid_s.e) + prev_pid_s.i;
    // let error_d_s = 2.*K_D / (2.*tau + DT) * (error_e_s - prev_error.s.e) + ((2.*tau - DT) / (2.*tau+DT)) * prev_error.s.d;
    // let error_d_s_prefilt = (error_e_s - prev_error.s.e) / DT;
    // Derivative on measurement vice error, to avoid derivative kick.
    let error_d_s_prefilt = (params.get_s() - prev_pid_s.measurement) / DT;

    let error_p_v = K_P * error_e_v;
    let error_i_v = K_I * DT/2. * (error_e_v + prev_pid_v.e) + prev_error.v.i;
    let error_d_v_prefilt = (params.get_v() - prev_pid_v.measurement) / DT;

    // todo: Avoid this DRY with a method on `filter`
    let mut error_d_v_pitch = [0.];
    unsafe {
        dsp_api::biquad_cascade_df1_f32(
            filters.s_pitch,
            &[error_d_v_prefilt.pitch],
            &mut error_d_v_pitch,
            1,
        );

        let mut error_d_v_roll = [0.];
        dsp_api::biquad_cascade_df1_f32(
            filters.s_roll,
            &[error_d_v_prefilt.roll],
            &mut error_d_v_roll,
            1,
        );
    }

    let mut error_s = PidState {
        e: error_e_s,
        p: error_p_s,
        i: error_i_s,
        d: error_d_s,
    };

    let mut error_v = PidState {
        e: error_e_s,
        p: error_p_v,
        i: error_i_v,
        d: error_d_v,
    };

    error_s.anti_windup_clamp();
    error_v.anti_windup_clamp();

    (error_s, error_v)
}

/// Adjust controls for a given flight command and PID error.
/// todo: Separate module for code that directly updates the rotors?
pub fn adjust_ctrls(
    throttle_adj: f32, // temp for acro?
    pid_s: PidState,
    pid_v: PidState,
    current_pwr: &mut RotorPower,
    rotor_pwm: &mut Timer<TIM2>,
) {

    // todo: Expand and populate this. Currently set up for "acro" controls only.
    let pitch_adj = pid_v.p.pitch + pid_v.i.pitch + pid_v.d.pitch;
    let roll_adj = pid_v.p.roll + pid_v.i.roll + pid_v.d.roll;
    let yaw_adj = pid_v.p.yaw + pid_v.i.yaw + pid_v.d.yaw;

    // alternatively:
    // let throttle_adj = pid_v.p.z + pid_v.i.z + pid_v.d.z;
    // this lets throttle control altitude, albeit coupled with speed if not level.
    // (Assuming PID argument are set up correctly for this)

    change_attitude(
        pitch_adj,
        roll_adj,
        yaw_adj,
        throttle_adj,
        current_pwr, // modified in place, and therefore upstream.
        rotor_pwm,
    );
}
