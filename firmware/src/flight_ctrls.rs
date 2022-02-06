///! This module contains code related to flight controls.
use core::ops::{Add, Sub};
use stm32_hal2::{pac::TIM2, timer::Timer};

use crate::{Rotor, DT, PWM_ARR};

// These coefficients map desired change in flight parameters to rotor power change.
// pitch, roll, and yaw s are in power / radians
const PITCH_S_COEFF: f32 = 0.1;
const ROLL_S_COEFF: f32 = 0.1;
const YAW_S_COEFF: f32 = 0.1;

/// Proportional, Integral, Derivative error, for flight parameter control updates.
/// For only a single set (s, v, a).
/// todo: Consider using Params, eg this is the error for a whole set of params.
#[derive(Default)]
pub struct PidError {
    pub p: ParamsInst,
    pub i: ParamsInst,
    pub d: ParamsInst,
}

/// A set of flight parameters to achieve and/or maintain. Similar values to `Parameters`,
/// but Options, specifying only the parameters we wish to achieve.
/// todo: Instead of hard-set values, consider a range, etc
/// todo: Some of these are mutually exclusive; consider a more nuanced approach.
#[derive(Default)]
pub struct FlightCmd {
    // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
    pub s_x: Option<f32>,
    pub s_y: Option<f32>,
    /// Altitude, in AGL. We treat MSL as a varying offset from this.
    pub s_z: Option<f32>,

    pub s_pitch: Option<f32>,
    pub s_roll: Option<f32>,
    pub s_yaw: Option<f32>,

    // Velocity
    pub v_x: Option<f32>,
    pub v_y: Option<f32>,
    pub v_z: Option<f32>,

    pub v_pitch: Option<f32>,
    pub v_roll: Option<f32>,
    pub v_yaw: Option<f32>,

    // Acceleration
    pub a_x: Option<f32>,
    pub a_y: Option<f32>,
    pub a_z: Option<f32>,

    pub a_pitch: Option<f32>,
    pub a_roll: Option<f32>,
    pub a_yaw: Option<f32>,
}

impl FlightCmd {
    /// Include manual inputs into the flight command.
    pub fn add_inputs(&mut self, inputs: ManualInputs) {
        self.v_pitch = match self.v_pitch {
            Some(v) => Some(v + inputs.pitch),
            None => Some(inputs.pitch),
        };
        self.v_roll = match self.v_roll {
            Some(v) => Some(v + inputs.roll),
            None => Some(inputs.roll),
        };
        self.v_yaw = match self.v_yaw {
            Some(v) => Some(v + inputs.yaw),
            None => Some(inputs.yaw),
        };
        // todo: throttle?
    }

    // Command a basic hover. Maintains an altitude and pitch, and attempts to maintain position,
    // but does revert to a fixed position.
    // Alt is in AGL.
    pub fn hover(alt: f32) -> Self {
        Self {
            // Maintaining attitude isn't enough. We need to compensate for wind etc.
            v_x: Some(0.),
            v_y: Some(0.),
            v_z: Some(0.),
            // todo: Hover at a fixed position, using more advanced logic. Eg command an acceleration
            // todo to reach it, then slow down and alt hold while near it?
            // s_z: Some(alt),
            // Pitch, roll, and yaw probably aren't required here?
            // s_pitch: Some(0.),
            // s_roll: Some(0.),
            // s_yaw: Some(0.),
            ..Default::default()
        }
    }

    /// Keep the device level and zero altitude change, with no other inputs.
    pub fn level() -> Self {
        Self {
            s_pitch: Some(0.),
            s_roll: Some(0.),
            s_yaw: Some(0.),
            v_z: Some(0.),
            ..Default::default()
        }
    }

    /// Maintains a hover in a specific location. lat and lon are in degrees. alt is in MSL.
    pub fn hover_geostationary(lat: f32, lon: f32, alt: f32) {}
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

/// Stores the current manual inputs to the system. `pitch`, `yaw`, and `roll` are in range -1. to +1.
/// `throttle` is in range 0. to 1. Corresponds to stick positions on a controller.
/// The interpretation of these depends on the current input mode.
#[derive(Default)]
pub struct ManualInputs {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
}

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
    // todo: Start with `current_power` instead of zeroing?
    // let mut power = RotorPower::default();
    // let power = current_power;

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
    height / 4.
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// todo: COnsider consolidating these instead of sep for s, v, a.
pub fn calc_pid_error(
    params: &Params,
    flight_cmd: &FlightCmd,
    prev_error_s: &PidError,
    prev_error_v: &PidError,
) -> (PidError, PidError) {
    // Find appropriate control inputs using PID control.
    // todo: Consider how you're using these variou structs with overlapping functionality:
    // todo Params, ParamsInst, FlightCmd. Splitting `pid_error` into 3 vars etc.
    let error_p_v = ParamsInst {
        z: params.v_z - flight_cmd.v_z.unwrap(),
        ..Default::default()
    };

    let error_p_s = ParamsInst {
        pitch: params.s_pitch - flight_cmd.s_pitch.unwrap(), // todo unwrap or 0/default?
        roll: params.s_roll - flight_cmd.s_roll.unwrap(),
        yaw: params.s_yaw - flight_cmd.s_yaw.unwrap(),
        ..Default::default()
    };

    let error_i_s = (prev_error_s.i + error_p_s) * DT;
    let error_d_s = (error_p_s - prev_error_s.d) / DT;

    let error_i_v = (prev_error_v.i + error_p_v) * DT;
    let error_d_v = (error_p_v - prev_error_v.d) / DT;

    (
        PidError {
            p: error_p_s,
            i: error_i_s,
            d: error_d_s,
        },
        PidError {
            p: error_p_v,
            i: error_i_v,
            d: error_d_v,
        },
    )
}

/// Adjust controls for a given flight command and PID error.
/// todo: Separate module for code that directly updates the rotors?
pub fn adjust_ctrls(
    flight_cmd: FlightCmd,
    pid_s: PidError,
    pid_v: PidError,
    current_pwr: &mut RotorPower,
    rotor_timer: &mut Timer<TIM2>,
) {
    // PID "constants
    let k_p = 0.1;
    let k_i = 0.05;
    let k_d = 0.;

    let mut pitch_adj = 0.;
    let mut roll_adj = 0.;
    let mut yaw_adj = 0.;

    if let Some(v) = flight_cmd.s_pitch {
        // todo: Check sign.
        pitch_adj = k_p * -pid_s.p.pitch + k_i * pid_s.i.pitch + k_d * pid_s.d.pitch;
        roll_adj = k_p * -pid_s.p.roll + k_i * pid_s.i.roll + k_d * pid_s.d.roll;
        yaw_adj = k_p * -pid_s.p.yaw + k_i * pid_s.i.yaw + k_d * pid_s.d.yaw;
    }

    let throttle_adj = 0.; // todo. eg start with adjusting for a hover.

    change_attitude(
        pitch_adj,
        roll_adj,
        yaw_adj,
        throttle_adj,
        current_pwr, // modified in place, and therefore upstream.
        rotor_timer,
    );
}
