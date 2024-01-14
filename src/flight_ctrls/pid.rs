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
    util::{self, clamp, iir_apply, IirInstWrapper},
};

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
    } else {
        // use crate::flight_ctrls::{};
    }
}

// use defmt::println;

pub struct PidCoeffs {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub max_i_windup: f32,
    pub att_ttc: f32,
}

impl Default for PidCoeffs {
    // For rate controls.
    fn default() -> Self {
        Self {
            p: 0.180,
            i: 0.060,
            d: 0.030,
            max_i_windup: 1.,
            att_ttc: 0.4,
        }
    }
}

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

        let d_error = iir_apply(filter, d_error);

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

#[derive(Default)]
/// For Motor RPM PID
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