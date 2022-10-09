//! This module contains code related to the flight control PID loop. It can be thought of
//! as a sub-module for `flight_ctrls`.
//!
//! See the OneNote document for notes on how we handle the more complicated / cascaded control modes.
//!
//! [Some info on the PID terms, focused on BF](https://gist.github.com/exocode/90339d7f946ad5f83dd1cf29bf5df0dc)
//! https://oscarliang.com/quadcopter-pid-explained-tuning/

use stm32_hal2::{dma::Dma, pac::DMA1};

use cmsis_dsp_api as dsp_api;

use crate::{
    control_interface::ChannelData,
    state::{SystemStatus, UserCfg},
    util::IirInstWrapper,
    DT_IMU,
};

use super::{
    autopilot::AutopilotStatus,
    common::{AttitudeCommanded, CtrlInputs, InputMap, MotorTimers, Params, RatesCommanded},
    ctrl_logic, ControlMapping,
};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::{ControlPositions};
    } else {
        use crate::flight_ctrls::{InputMode, MotorPower,
         MAX_ROTOR_POWER};
    }
}

use defmt::println;

const INTEGRATOR_CLAMP_MAX_QUAD: f32 = 0.4;
const INTEGRATOR_CLAMP_MIN_QUAD: f32 = -INTEGRATOR_CLAMP_MAX_QUAD;
const INTEGRATOR_CLAMP_MAX_FIXED_WING: f32 = 0.4;
const INTEGRATOR_CLAMP_MIN_FIXED_WING: f32 = -INTEGRATOR_CLAMP_MAX_FIXED_WING;

// filter_ = signal.iirfilter(1, 100, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

// todo: Diff coeffs for diff diff parts, as required.
#[allow(clippy::excessive_precision)]
static COEFFS_D: [f32; 5] = [
    0.037804754170896473,
    0.037804754170896473,
    0.0,
    0.9243904916582071,
    -0.0,
];

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

/// Coefficients and other configurable parameters for our motor PID
pub struct MotorCoeffs {
    pub p_front_left: f32,
    pub p_front_right: f32,
    pub p_aft_left: f32,
    pub p_aft_right: f32,

    pub i_front_left: f32,
    pub i_front_right: f32,
    pub i_aft_left: f32,
    pub i_aft_right: f32,

    // pub d_front_left: f32,
    // pub d_front_right: f32,
    // pub d_aft_left: f32,
    // pub d_aft_right: f32,
    pub rpm_cutoff: LowpassCutoff,
}

impl Default for MotorCoeffs {
    #[cfg(feature = "quad")]
    fn default() -> Self {
        Self {
            p_front_left: 0.,
            p_front_right: 0.,
            p_aft_left: 0.,
            p_aft_right: 0.,

            i_front_left: 0.,
            i_front_right: 0.,
            i_aft_left: 0.,
            i_aft_right: 0.,

            rpm_cutoff: LowpassCutoff::H1k,
        }
    }
}

pub struct MotorCoeffGroup {
    pub front_left: MotorCoeffs,
    pub front_right: MotorCoeffs,
    pub aft_left: MotorCoeffs,
    pub aft_right: MotorCoeffs,
}

impl Default for MotorCoeffGroup {
    fn default() -> Self {
        Self {
            front_left: Default::default(),
            front_right: Default::default(),
            aft_left: Default::default(),
            aft_right: Default::default(),
        }
    }
}

#[derive(Default)]
pub struct MotorPidGroup {
    pub front_left: PidState,
    pub front_right: PidState,
    pub aft_left: PidState,
    pub aft_right: PidState,
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

/// Proportional, Integral, Derivative error, for flight parameter control updates.
/// For only a single set (s, v, a). Note that e is the error betweeen commanded
/// and measured, while the other terms include the PID coefficients (K_P) etc.
/// So, `p` is always `e` x `K_P`.
/// todo: Consider using Params, eg this is the error for a whole set of params.
#[derive(Default)]
pub struct PidState {
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

impl PidState {
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

        if self.i > lim_max_int {
            self.i = lim_max_int;
        } else if self.i < lim_min_int {
            self.i = lim_min_int;
        }
    }

    pub fn out(&self) -> f32 {
        self.p + self.i + self.d
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
        let mut result = Self {
            rpm_front_left: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            rpm_front_right: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            rpm_aft_left: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            rpm_aft_right: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            // todo: Re-initialize fn?
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.roll_attitude.inner,
                &COEFFS_D,
                &mut FILTER_STATE_ROLL_ATTITUDE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.pitch_attitude.inner,
                &COEFFS_D,
                &mut FILTER_STATE_PITCH_ATTITUDE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.yaw_attitude.inner,
                &COEFFS_D,
                &mut FILTER_STATE_YAW_ATTITUDE,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.roll_rate.inner,
                &COEFFS_D,
                &mut FILTER_STATE_ROLL_RATE,
            );
        }

        result
    }
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// Example: https://github.com/pms67/PID/blob/master/PID.c
/// Example 2: https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Pid.c
pub fn run(
    set_pt: f32,
    measurement: f32,
    prev_pid: &PidState,
    k_p: f32,
    k_i: f32,
    k_d: f32,
    filter: Option<&mut IirInstWrapper>,
    // This `dt` is dynamic, since we don't necessarily run this function at a fixed interval.
    dt: f32,
) -> PidState {
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
    match filter {
        Some(f) => {
            dsp_api::biquad_cascade_df1_f32(&mut filter.inner, &[error_d_prefilt], &mut error_d, 1);
        }
        None => error_d
    }

    let mut result = PidState {
        measurement,
        e: error,
        p: error_p,
        i: error_i,
        d: error_d[0],
    };

    result.anti_windup_clamp(error_p);

    // todo: Clamp output?

    result
}
