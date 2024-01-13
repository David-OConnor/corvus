//! This module contains code related to the flight control PID loop. It can be thought of
//! as a sub-module for `flight_ctrls`.
//!
//! See the OneNote document for notes on how we handle the more complicated / cascaded control modes.
//!
//! [Some info on the PID terms, focused on BF](https://gist.github.com/exocode/90339d7f946ad5f83dd1cf29bf5df0dc)
//! https://oscarliang.com/quadcopter-pid-explained-tuning/
//!
//! As of 2023-02-15, we use this only for commanding specific motor RPMs.

use cfg_if::cfg_if;
use cmsis_dsp_api as dsp_api;
use cmsis_dsp_api::iir_new;

use crate::{
    flight_ctrls::filters,
    util::{self, clamp, filter_one, IirInstWrapper},
};

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
    } else {
        // use crate::flight_ctrls::{};
    }
}

// use defmt::println;

// The maximum amount we can change power in a single loop. Power is on a 0 to 1 scale.
const OUTPUT_CLAMP_MAX: f32 = 0.01;
const OUTPUT_CLAMP_MIN: f32 = -OUTPUT_CLAMP_MAX;

const INTEGRATOR_CLAMP_MAX_QUAD: f32 = 0.4;
const INTEGRATOR_CLAMP_MIN_QUAD: f32 = -INTEGRATOR_CLAMP_MAX_QUAD;
const INTEGRATOR_CLAMP_MAX_FIXED_WING: f32 = 0.4;
const INTEGRATOR_CLAMP_MIN_FIXED_WING: f32 = -INTEGRATOR_CLAMP_MAX_FIXED_WING;

static mut FILTER_STATE_FRONT_LEFT: [f32; 4] = [0.; 4];
static mut FILTER_STATE_FRONT_RIGHT: [f32; 4] = [0.; 4];
static mut FILTER_STATE_AFT_LEFT: [f32; 4] = [0.; 4];
static mut FILTER_STATE_AFT_RIGHT: [f32; 4] = [0.; 4];

#[derive(Default)]
pub struct PidState {
    pub p: f32,
    pub i: f32,
}

impl PidState {
    pub fn apply(
        &mut self,
        target: f32,
        current: f32,
        coeffs: &PidCoeffs,
        filter: &mut IirInstWrapper,
        dt: f32,
    ) -> f32 {
        let error_x_prev = self.p;

        self.p = target - current;

        // Note that we take dt into account re the dimensions of the D-term coefficient.
        let d_error = self.p - error_x_prev;

        let d_error = filter_one(filter, d_error);

        self.i += self.p * dt;

        clamp(&mut self.i, (-coeffs.max_i_windup, coeffs.max_i_windup));

        coeffs.p * self.p + coeffs.i * self.i + coeffs.d * d_error
    }
}

#[derive(Default)]
pub struct PidStateRate {
    pub pitch: PidState,
    pub roll: PidState,
    pub yaw: PidState,
}

impl PidStateRate {
    pub fn reset_i(&mut self) {
        self.pitch.i = 0.;
        self.roll.i = 0.;
        self.yaw.i = 0.;
    }
}

/// Cutoff frequency for our PID lowpass frequency, in Hz
#[derive(Clone, Copy)]
pub enum LowpassCutoff {
    // todo: What values should these be?
    H500,
    H1k,
    H10k,
    H20k,
}

// todo: Feature-gate for quad as-required

/// Coefficients and other configurable parameters for our motor PID,
/// where we command RPMs, and use PID to reach and maintain them by adjusting
/// motor power.
pub struct MotorCoeffs {
    pub p_front_left: f32,
    pub p_front_right: f32,
    pub p_aft_left: f32,
    pub p_aft_right: f32,

    pub i_front_left: f32,
    pub i_front_right: f32,
    pub i_aft_left: f32,
    pub i_aft_right: f32,

    pub rpm_cutoff: LowpassCutoff,
}

impl Default for MotorCoeffs {
    #[cfg(feature = "quad")]
    fn default() -> Self {
        // P and I terms are the same for all motors, but we leave this struct with all 4
        // to make it easier to change later.
        // The value of these terms is small since RPMs are 3x the values of power settings,
        // and we use a khz-order update loop.
        // todo: float precision issues working with numbers of very different OOMs? Seems to be
        // todo find from a Rust Playground test.
        let p = 0.00000002;
        let i = 0.00000001;
        Self {
            p_front_left: p,
            p_front_right: p,
            p_aft_left: p,
            p_aft_right: p,

            i_front_left: i,
            i_front_right: i,
            i_aft_left: i,
            i_aft_right: i,

            // i_front_left: 0.,
            // i_front_right: 0.,
            // i_aft_left: 0.,
            // i_aft_right: 0.,
            rpm_cutoff: LowpassCutoff::H1k,
        }
    }
    #[cfg(feature = "fixed-wing")]
    fn default() -> Self {
        Self {
            p_front_left: 1.,
            p_front_right: 1.,
            p_aft_left: 1.,
            p_aft_right: 1.,

            i_front_left: 0.5,
            i_front_right: 0.5,
            i_aft_left: 0.5,
            i_aft_right: 0.5,

            rpm_cutoff: LowpassCutoff::H1k,
        }
    }
}

// #[derive(Default)]
// pub struct MotorCoeffGroup {
//     pub front_left: MotorCoeffs,
//     pub front_right: MotorCoeffs,
//     pub aft_left: MotorCoeffs,
//     pub aft_right: MotorCoeffs,
// }

#[derive(Default)]
pub struct MotorPidGroup {
    pub front_left: PidStateLegacy,
    pub front_right: PidStateLegacy,
    pub aft_left: PidStateLegacy,
    pub aft_right: PidStateLegacy,
}

impl MotorPidGroup {
    /// Reset the interator term on all components.
    pub fn reset_integrator(&mut self) {
        self.front_left.i = 0.;
        self.front_right.i = 0.;
        self.aft_left.i = 0.;
        self.aft_right.i = 0.;
    }
}

pub struct PidCoeffs {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub att_ttc: f32,
    pub max_i_windup: f32,
}

impl Default for PidCoeffs {
    // For rate controls.
    fn default() -> Self {
        Self {
            p: 0.180,
            i: 0.060,
            d: 0.030,
            att_ttc: 0.4,
            max_i_windup: 1.,
        }
    }
}

/// Proportional, Integral, Derivative error, for flight parameter control updates.
/// For only a single set (s, v, a). Note that e is the error betweeen commanded
/// and measured, while the other terms include the PID coefficients (K_P) etc.
/// So, `p` is always `e` x `K_P`.
/// todo: Consider using Params, eg this is the error for a whole set of params.
#[derive(Default)]
pub struct PidStateLegacy {
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

impl PidStateLegacy {
    /// Anti-windup integrator clamp
    fn anti_windup_clamp(&mut self, error_p: f32) {
        //  Dynamic integrator clamping, from https://www.youtube.com/watch?v=zOByx3Izf5U

        // todo: Ac type; use clamp fixed wing

        let lim_max_int = if INTEGRATOR_CLAMP_MAX_QUAD > error_p {
            INTEGRATOR_CLAMP_MAX_QUAD - error_p
        } else {
            0.
        };

        let lim_min_int = if INTEGRATOR_CLAMP_MIN_QUAD < error_p {
            INTEGRATOR_CLAMP_MIN_QUAD - error_p
        } else {
            0.
        };

        util::clamp(&mut self.i, (lim_min_int, lim_max_int));
    }

    pub fn out(&self) -> f32 {
        let mut result = self.p + self.i + self.d;

        util::clamp(&mut result, (OUTPUT_CLAMP_MIN, OUTPUT_CLAMP_MAX));

        result
    }
}

/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop. Note that we don't
/// need this for our horizontal velocity PIDs.
pub struct DerivFilters {
    /// For commanding an RPM, moduling power to meet the target RPM. Motor 1.
    pub rpm_front_left: IirInstWrapper,
    /// Motor 2
    pub rpm_front_right: IirInstWrapper,
    /// Motor 3
    pub rpm_aft_left: IirInstWrapper,
    /// Motor 3
    pub rpm_aft_right: IirInstWrapper,
}

impl Default for DerivFilters {
    fn default() -> Self {
        unsafe {
            Self {
                rpm_front_left: IirInstWrapper {
                    inner: iir_new(&COEFFS_D, &mut FILTER_STATE_FRONT_LEFT),
                },
                rpm_front_right: IirInstWrapper {
                    inner: iir_new(&COEFFS_D, &mut FILTER_STATE_FRONT_RIGHT),
                },
                rpm_aft_left: IirInstWrapper {
                    inner: iir_new(&COEFFS_D, &mut FILTER_STATE_AFT_LEFT),
                },
                rpm_aft_right: IirInstWrapper {
                    inner: iir_new(&COEFFS_D, &mut FILTER_STATE_AFT_RIGHT),
                },
            }
        }
    }
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// Example: https://github.com/pms67/PID/blob/master/PID.c
/// Example 2: https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Pid.c
///
/// The value output from the `out` methd on PidState is how we access results. It is a change in commanded
/// power.
pub fn run(
    set_pt: f32,
    measurement: f32,
    prev_pid: &PidStateLegacy,
    k_p: f32,
    k_i: f32,
    k_d: f32,
    filter: Option<&mut IirInstWrapper>,
    // This `dt` is dynamic, since we don't necessarily run this function at a fixed interval.
    dt: f32,
) -> PidStateLegacy {
    // Find appropriate control inputs using PID control.

    let error = set_pt - measurement;

    // https://www.youtube.com/watch?v=zOByx3Izf5U
    let error_p = k_p * error;

    // For inegral term, use a midpoint formula, and use error, vice measurement.
    let error_i = k_i * (error + prev_pid.e) / 2. * dt + prev_pid.i;

    // Derivative on measurement vice error, to avoid derivative kick. Note that deriv-on-measurment
    // can be considered smoother, while deriv-on-error can be considered more responsive.
    let error_d_prefilt = k_d * (measurement - prev_pid.measurement) / dt;

    let mut error_d = [0.];

    if let Some(f) = filter {
        dsp_api::biquad_cascade_df1_f32(&mut f.inner, &[error_d_prefilt], &mut error_d, 1);
    }

    let mut result = PidStateLegacy {
        measurement,
        e: error,
        p: error_p,
        i: error_i,
        d: error_d[0],
    };

    result.anti_windup_clamp(error_p);

    result
}
