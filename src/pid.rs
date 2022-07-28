//! This module contains code related to the flight control PID loop. It can be thought of
//! as a sub-module for `flight_ctrls`.
//!
//! See the OneNote document for notes on how we handle the more complicated / cascaded control modes.
//!
//! [Some info on the PID terms, focused on BF](https://gist.github.com/exocode/90339d7f946ad5f83dd1cf29bf5df0dc)
//! https://oscarliang.com/quadcopter-pid-explained-tuning/

use core::f32::consts::TAU;

use stm32_hal2::{dma::Dma, pac::DMA1};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as dsp_sys;

use crate::{
    autopilot::AutopilotStatus,
    control_interface::ChannelData,
    flight_ctrls::{
        self,
        common::{AltType, CommandState, CtrlInputs, InputMap, MotorTimers, Params},
        quad::{InputMode, POWER_LUT, YAW_ASSIST_COEFF, YAW_ASSIST_MIN_SPEED},
    },
    util::IirInstWrapper,
    ArmStatus, ControlPositions, RotorMapping, ServoWingMapping, UserCfg, DT_ATTITUDE,
};

use crate::flight_ctrls::quad::MAX_ROTOR_POWER;
use defmt::println;

// todo: You need to take derivative of control inputs into account. For example, when sticks are moved,
// todo: reduce I term, and/or add a "FF"-scaled term direction to PID output in dir of
// todo the change etc

// todo: In rate/acro mode, instead of zeroing unused axes, have them store a value that they return to?'

// Amount each airborne, (from controller) PID adjustment modifies a given PID term.
pub const PID_CONTROL_ADJ_AMT: f32 = 0.001; // in whatever units are PID values are
pub const PID_CONTROL_ADJ_TIMEOUT: f32 = 0.3; // seconds

const INTEGRATOR_CLAMP_MAX_QUAD: f32 = 0.4;
const INTEGRATOR_CLAMP_MIN_QUAD: f32 = -INTEGRATOR_CLAMP_MAX_QUAD;
const INTEGRATOR_CLAMP_MAX_FIXED_WING: f32 = 0.4;
const INTEGRATOR_CLAMP_MIN_FIXED_WING: f32 = -INTEGRATOR_CLAMP_MAX_FIXED_WING;

// "TPA" stands for Throttle PID attenuation - reduction in D term (or more) past a certain
// throttle setting, linearly. This only applies to the rate loop.
// https://github-wiki-see.page/m/betaflight/betaflight/wiki/PID-Tuning-Guide
pub const TPA_MIN_ATTEN: f32 = 0.5; // At full throttle, D term is attenuated to this value.
pub const TPA_BREAKPOINT: f32 = 0.3; // Start engaging TPA at this value.
                                     // `TPA_SLOPE` and `TPA_Y_INT` are cached calculations.
const TPA_SLOPE: f32 = (TPA_MIN_ATTEN - 1.) / (MAX_ROTOR_POWER - TPA_BREAKPOINT);
const TPA_Y_INT: f32 = -(TPA_SLOPE * TPA_BREAKPOINT - 1.);

/// Update the D term with throttle PID attenuation; linear falloff of the D term at a cutoff throttle
/// setting. Multiple the D term by this function's output.
fn tpa_adjustment(throttle: f32) -> f32 {
    TPA_SLOPE * throttle + TPA_Y_INT
}

// These filter states are for the PID D term.
static mut FILTER_STATE_ROLL_ATTITUDE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_PITCH_ATTITUDE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_YAW_ATTITUDE: [f32; 4] = [0.; 4];

static mut FILTER_STATE_ROLL_RATE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_PITCH_RATE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_YAW_RATE: [f32; 4] = [0.; 4];

static mut FILTER_STATE_THRUST: [f32; 4] = [0.; 4];

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

/// Coefficients and other configurable parameters for controls, for pich and roll.
/// Has several variants, due to coupling with horizontal (X and Y) movement.
pub struct CtrlCoeffsPR {
    // These coefficients map desired change in flight parameters to rotor power change.
    // pitch, roll, and yaw s are in power / radians
    pub k_p_rate: f32,
    pub k_i_rate: f32,
    pub k_d_rate: f32,

    pub k_p_attitude: f32,
    pub k_i_attitude: f32,
    pub k_d_attitude: f32,

    pub k_p_velocity: f32,
    pub k_i_velocity: f32,
    // k_d_velocity: f32,
    // Note that we don't use the D component for our velocity PID.
    pub pid_deriv_lowpass_cutoff_rate: LowpassCutoff,
    pub pid_deriv_lowpass_cutoff_attitude: LowpassCutoff,
}

impl Default for CtrlCoeffsPR {
    fn default() -> Self {
        Self {
            k_p_rate: 0.10,
            k_i_rate: 0.50,
            k_d_rate: 0.0030,

            // pid for controlling pitch and roll from commanded horizontal velocity
            k_p_attitude: 47.,
            k_i_attitude: 84.,
            k_d_attitude: 34.,

            // PID for controlling pitch and roll rate directly (eg Acro)
            k_p_velocity: 0.1,
            k_i_velocity: 0.,
            // k_d_velocity: 0.,
            pid_deriv_lowpass_cutoff_rate: LowpassCutoff::H1k,
            pid_deriv_lowpass_cutoff_attitude: LowpassCutoff::H1k,
        }
    }
}

impl CtrlCoeffsPR {
    pub fn default_flying_wing() -> Self {
        Self {
            k_p_rate: 0.06,
            // k_i_rate: 0.60,
            k_i_rate: 0.0,
            // k_d_rate: 0.02,
            k_d_rate: 0.00,

            // Attitude not used.

            // pid for controlling pitch and roll from commanded horizontal velocity
            k_p_attitude: 0.,
            k_i_attitude: 0.,
            k_d_attitude: 0.,

            // PID for controlling pitch and roll rate directly (eg Acro)
            k_p_velocity: 0.1,
            k_i_velocity: 0.,
            // k_d_velocity: 0.,
            pid_deriv_lowpass_cutoff_rate: LowpassCutoff::H1k,
            pid_deriv_lowpass_cutoff_attitude: LowpassCutoff::H1k,
        }
    }
}

/// Coefficients and other configurable parameters for yaw and thrust. This is separate from, and
/// simpler than the variant for pitch and roll, since it's not coupled to X and Y motion.
pub struct CtrlCoeffsYT {
    // PID for controlling yaw or thrust from a velocity directly applied to them. (Eg Acro and attitude)
    pub k_p_rate: f32,
    pub k_i_rate: f32,
    pub k_d_rate: f32,

    // PID for controlling yaw or thrust from an explicitly-commanded heading or altitude.
    pub k_p_attitude: f32,
    pub k_i_attitude: f32,
    pub k_d_attitude: f32,

    pub pid_deriv_lowpass_cutoff: LowpassCutoff,
}

impl Default for CtrlCoeffsYT {
    fn default() -> Self {
        Self {
            // k_p_rate: 0.6 * K_U_YAW,
            // k_i_rate: 1.2 * K_U_YAW / T_U_YAW,
            // k_d_rate: 3. * K_U_YAW * T_U_YAW / 40.,
            k_p_rate: 0.30,
            k_i_rate: 0.01 * 0.,
            k_d_rate: 0.,

            k_p_attitude: 0.1,
            k_i_attitude: 0.0,
            k_d_attitude: 0.0,

            pid_deriv_lowpass_cutoff: LowpassCutoff::H1k,
        }
    }
}

pub struct CtrlCoeffGroup {
    pub pitch: CtrlCoeffsPR,
    pub roll: CtrlCoeffsPR,
    pub yaw: CtrlCoeffsYT,
    pub thrust: CtrlCoeffsYT,
}

impl Default for CtrlCoeffGroup {
    /// These starting values are Betaflight defaults.
    fn default() -> Self {
        Self {
            pitch: Default::default(),
            roll: Default::default(),
            yaw: Default::default(),
            thrust: Default::default(),
        }
    }
}

impl CtrlCoeffGroup {
    pub fn default_flying_wing() -> Self {
        Self {
            pitch: CtrlCoeffsPR::default_flying_wing(),
            roll: CtrlCoeffsPR::default_flying_wing(),
            yaw: Default::default(),
            thrust: Default::default(),
        }
    }
}

#[derive(Default)]
pub struct PidGroup {
    pub pitch: PidState,
    pub roll: PidState,
    pub yaw: PidState,
    pub thrust: PidState,
}

impl PidGroup {
    /// Reset the interator term on all components.
    pub fn reset_integrator(&mut self) {
        self.pitch.i = 0.;
        self.roll.i = 0.;
        self.yaw.i = 0.;
        self.thrust.i = 0.;
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
    pub fn anti_windup_clamp(&mut self, error_p: f32) {
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
pub struct PidDerivFilters {
    pub roll_attitude: IirInstWrapper,
    pub pitch_attitude: IirInstWrapper,
    pub yaw_attitude: IirInstWrapper,

    pub roll_rate: IirInstWrapper,
    pub pitch_rate: IirInstWrapper,
    pub yaw_rate: IirInstWrapper,

    pub thrust: IirInstWrapper, // todo - do we need this?
}

impl Default for PidDerivFilters {
    fn default() -> Self {
        let mut result = Self {
            roll_attitude: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            pitch_attitude: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            yaw_attitude: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            roll_rate: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            pitch_rate: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            yaw_rate: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            thrust: IirInstWrapper {
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
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.pitch_rate.inner,
                &COEFFS_D,
                &mut FILTER_STATE_PITCH_RATE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.yaw_rate.inner,
                &COEFFS_D,
                &mut FILTER_STATE_YAW_RATE,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.thrust.inner,
                &COEFFS_D,
                &mut FILTER_STATE_THRUST,
            );
        }

        result
    }
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// Example: https://github.com/pms67/PID/blob/master/PID.c
/// Example 2: https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Pid.c
pub fn calc_pid_error(
    set_pt: f32,
    measurement: f32,
    prev_pid: &PidState,
    k_p: f32,
    k_i: f32,
    k_d: f32,
    filter: &mut IirInstWrapper,
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
    dsp_api::biquad_cascade_df1_f32(&mut filter.inner, &[error_d_prefilt], &mut error_d, 1);

    // println!(
    //     "SP {} M{} e {} p {} i {} d {}",
    //     set_pt, measurement, error, error_p, error_i, error_d[0]
    // );

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

/// Run the velocity (outer) PID Loop: This is used to determine attitude, eg based on commanded velocity
/// or position.
pub fn run_velocity(
    params: &Params,
    // inputs: &CtrlInputs,
    ch_data: &ChannelData,
    input_map: &InputMap,
    velocities_commanded: &mut CtrlInputs,
    attitude_commanded: &mut CtrlInputs,
    pid: &mut PidGroup,
    filters: &mut PidDerivFilters,
    input_mode: &InputMode,
    autopilot_status: &AutopilotStatus,
    cfg: &UserCfg,
    commands: &mut CommandState,
    coeffs: &CtrlCoeffGroup,
) {
    // todo: GO over this whole function; it's not ready!
    // todo, and you need a fixed wing version.

    match input_mode {
        InputMode::Acro => (),
        InputMode::Attitude => (),
        InputMode::Command => {
            // todo: Impl
            // if autopilot_status.takeoff {
            //     // AutopilotMode::Takeoff => {
            //     *velocities_commanded = CtrlInputs {
            //         pitch: 0.,
            //         roll: 0.,
            //         yaw: 0.,
            //         thrust: flight_ctrls::quad::takeoff_speed(params.tof_alt, cfg.max_speed_ver),
            //     };
            // }
            // else if autopilot_status.land {
            //     *velocities_commanded = CtrlInputs {
            //         pitch: 0.,
            //         roll: 0.,
            //         yaw: 0.,
            //         thrust: flight_ctrls::quad::landing_speed(params.tof_alt, cfg.max_speed_ver),
            //     };
            // }
        }
    }

    let mut param_x = params.v_x;
    let mut param_y = params.v_y;

    let mut k_p_pitch = coeffs.pitch.k_p_attitude;
    let mut k_i_pitch = coeffs.pitch.k_i_attitude;
    let mut k_d_pitch = coeffs.pitch.k_d_attitude;

    let mut k_p_roll = coeffs.roll.k_p_attitude;
    let mut k_i_roll = coeffs.roll.k_i_attitude;
    let mut k_d_roll = coeffs.roll.k_d_attitude;

    let eps1 = 0.01;
    if ch_data.pitch > eps1 || ch_data.roll > eps1 {
        commands.loiter_set = false;
    }

    let eps2 = 0.01;
    // todo: Commanded velocity 0 to trigger loiter logic, or actual velocity?
    // if mid_flight_cmd.y_pitch.unwrap().2 < eps && mid_flight_cmd.x_roll.unwrap().2 < eps {
    if params.lon < eps2 && params.lat < eps2 {
        if !commands.loiter_set {
            commands.x = params.lon;
            commands.y = params.lat;
            commands.loiter_set = true;
        }

        param_x = commands.x;
        param_y = commands.y;

        k_p_pitch = coeffs.pitch.k_p_rate;
        k_i_pitch = coeffs.pitch.k_i_rate;
        k_d_pitch = coeffs.pitch.k_d_rate;

        k_p_roll = coeffs.roll.k_p_rate;
        k_i_roll = coeffs.roll.k_i_rate;
        k_d_roll = coeffs.roll.k_d_rate;
    }

    pid.pitch = calc_pid_error(
        velocities_commanded.pitch.unwrap(),
        param_y,
        &pid.pitch,
        coeffs.pitch.k_p_velocity,
        coeffs.pitch.k_p_velocity,
        0., // first-order + delay system
        &mut filters.pitch_attitude,
        DT_ATTITUDE,
    );

    pid.roll = calc_pid_error(
        velocities_commanded.roll.unwrap(),
        param_x,
        &pid.roll,
        // coeffs,
        coeffs.roll.k_p_velocity,
        coeffs.roll.k_p_velocity,
        0.,
        &mut filters.roll_attitude,
        DT_ATTITUDE,
    );

    // todo: What should this be ??
    pid.yaw = calc_pid_error(
        velocities_commanded.yaw.unwrap(),
        params.s_yaw_heading,
        &pid.yaw,
        0., // todo
        0., // todo
        0.,
        &mut filters.yaw_attitude,
        DT_ATTITUDE,
    );

    // todo: What should this be ??
    pid.thrust = calc_pid_error(
        velocities_commanded.thrust.unwrap(),
        params.baro_alt_msl,
        &pid.thrust,
        0., // todo
        0., // todo
        0.,
        &mut filters.thrust,
        DT_ATTITUDE,
    );

    // Determine commanded pitch and roll positions, and z velocity,
    // based on our middle-layer PID.

    // todo: the actual modification ofn attitude is commented out for now (july 27 2022)
    // todo until we get attitude, rate, and various autopilot modes sorted out.
    *attitude_commanded = CtrlInputs {
        pitch: Some(pid.pitch.out()),
        roll: Some(pid.roll.out()),
        yaw: Some(pid.yaw.out()),
        thrust: Some(pid.thrust.out()),
    };
}

/// To reduce DRY between `run_attitude_quad` and `run_attitude_fixed_wing`. The main purpose of
/// this fn is to modify `rates_commanded`.
fn attitude_apply_common(
    pid_attitude: &mut PidGroup,
    rates_commanded: &mut CtrlInputs,
    params: &Params,
    attitudes_commanded: &CtrlInputs,
    coeffs: &CtrlCoeffGroup,
    filters: &mut PidDerivFilters,
) {
    // If an attitude has been commanded (eg a velocity loop,autopilot mode, or if the aircraft
    // is in attitude mode), apply the PID to it.
    if let Some(pitch_commanded) = attitudes_commanded.pitch {
        pid_attitude.pitch = calc_pid_error(
            pitch_commanded,
            params.s_pitch,
            &pid_attitude.pitch,
            coeffs.pitch.k_p_attitude,
            coeffs.pitch.k_i_attitude,
            coeffs.pitch.k_d_attitude,
            &mut filters.pitch_attitude,
            DT_ATTITUDE,
        );

        rates_commanded.pitch = Some(pid_attitude.pitch.out());
    }

    if let Some(roll_commanded) = attitudes_commanded.roll {
        pid_attitude.roll = calc_pid_error(
            roll_commanded,
            params.s_roll,
            &pid_attitude.roll,
            coeffs.roll.k_p_attitude,
            coeffs.roll.k_i_attitude,
            coeffs.roll.k_d_attitude,
            &mut filters.roll_attitude,
            DT_ATTITUDE,
        );

        rates_commanded.roll = Some(pid_attitude.roll.out());
    }

    if let Some(yaw_commanded) = attitudes_commanded.yaw {
        pid_attitude.yaw = calc_pid_error(
            yaw_commanded,
            params.s_yaw_heading,
            &pid_attitude.yaw,
            coeffs.yaw.k_p_attitude,
            coeffs.yaw.k_i_attitude,
            coeffs.yaw.k_d_attitude,
            &mut filters.yaw_attitude,
            DT_ATTITUDE,
        );
        rates_commanded.yaw = Some(pid_attitude.yaw.out());
    }
}

/// Run the attitude (mid) PID loop: This is used to determine angular velocities, based on commanded
/// attitude. Modifies `rates_commanded`, which is used by the rate PID loop.
pub fn run_attitude_quad(
    params: &Params,
    ch_data: &ChannelData,
    input_map: &InputMap,
    attitudes_commanded: &mut CtrlInputs,
    rates_commanded: &mut CtrlInputs,
    pid_attitude: &mut PidGroup,
    filters: &mut PidDerivFilters,
    input_mode: &InputMode,
    autopilot_status: &AutopilotStatus,
    cfg: &UserCfg,
    coeffs: &CtrlCoeffGroup,
) {
    match input_mode {
        InputMode::Acro => (),

        // If in Attitude control mode, command our initial (pre-autopilot) attitudes based on
        // control positions.
        InputMode::Attitude => {
            *attitudes_commanded = CtrlInputs {
                pitch: Some(input_map.calc_pitch_angle(ch_data.pitch)),
                roll: Some(input_map.calc_roll_angle(ch_data.roll)),
                yaw: Some(input_map.calc_yaw_rate(ch_data.yaw)),
                thrust: None,
            };
        }
        InputMode::Command => {
            // todo: Impl
        }
    }

    autopilot_status.apply_quad(
        params,
        attitudes_commanded,
        rates_commanded,
        pid_attitude,
        filters,
        coeffs,
        input_map,
        cfg.max_speed_ver,
    );

    attitude_apply_common(
        pid_attitude,
        rates_commanded,
        params,
        attitudes_commanded,
        coeffs,
        filters,
    );
}

/// Run the attitude (mid) PID loop: This is used to determine angular velocities, based on commanded
/// attitude. Note that for fixed wing, we have no direct attitude mode, so this is entirely determined
/// by the various autopilot modes, or if we're mapping throttle to airspeed etc.
/// Modifies `rates_commanded`, which is used by the rate PID loop.
pub fn run_attitude_fixed_wing(
    params: &Params,
    // ch_data: &ChannelData,
    // input_map: &InputMap,
    attitudes_commanded: &mut CtrlInputs,
    rates_commanded: &mut CtrlInputs,
    pid_attitude: &mut PidGroup,
    filters: &mut PidDerivFilters,
    // input_mode: &InputMode,
    autopilot_status: &AutopilotStatus,
    // cfg: &UserCfg,
    coeffs: &CtrlCoeffGroup,
) {
    // Note that for fixed wing, we don't have attitude mode.
    autopilot_status.apply_fixed_wing(
        params,
        attitudes_commanded,
        rates_commanded,
        pid_attitude,
        filters,
        coeffs,
    );

    attitude_apply_common(
        pid_attitude,
        rates_commanded,
        params,
        attitudes_commanded,
        coeffs,
        filters,
    );
}

/// To reduce DRY between `run_rate_quad` and `run_rate_fixed_wing`. Modifies `rates_commanded`,
/// and returns PID output as pitch, roll, yaw, thrust.
fn rate_apply_common(
    pid_rate: &mut PidGroup,
    rates_commanded: &mut CtrlInputs,
    params: &Params,
    ch_data: &ChannelData,
    coeffs: &CtrlCoeffGroup,
    filters: &mut PidDerivFilters,
    input_map: &InputMap,
    tpa_scaler: f32,
    dt: f32,
) -> (f32, f32, f32, f32) {
    // If a given rate (or throttle) hasn't been defined by an outer loop or autopilot mode, apply
    // its manual control.
    if rates_commanded.pitch.is_none() {
        pid_rate.pitch = calc_pid_error(
            input_map.calc_pitch_rate(ch_data.pitch),
            params.v_pitch,
            &pid_rate.pitch,
            coeffs.pitch.k_p_rate,
            coeffs.pitch.k_i_rate,
            coeffs.pitch.k_d_rate * tpa_scaler,
            &mut filters.pitch_rate,
            dt,
        );

        rates_commanded.pitch = Some(pid_rate.pitch.out());
    }

    if rates_commanded.roll.is_none() {
        pid_rate.roll = calc_pid_error(
            input_map.calc_roll_rate(ch_data.roll),
            params.v_roll,
            &pid_rate.roll,
            coeffs.roll.k_p_rate,
            coeffs.roll.k_i_rate,
            coeffs.roll.k_d_rate * tpa_scaler,
            &mut filters.roll_rate,
            dt,
        );

        rates_commanded.roll = Some(pid_rate.roll.out());
    }

    if rates_commanded.yaw.is_none() {
        pid_rate.yaw = calc_pid_error(
            input_map.calc_yaw_rate(ch_data.yaw),
            params.v_yaw,
            &pid_rate.yaw,
            coeffs.yaw.k_p_rate,
            coeffs.yaw.k_i_rate,
            coeffs.yaw.k_d_rate,
            &mut filters.yaw_rate,
            dt,
        );

        rates_commanded.yaw = Some(pid_rate.yaw.out());
    }

    if rates_commanded.thrust.is_none() {
        rates_commanded.thrust = Some(input_map.calc_manual_throttle(ch_data.throttle));
    }

    let pitch = rates_commanded.pitch.unwrap();
    let roll = rates_commanded.roll.unwrap();
    let yaw = rates_commanded.yaw.unwrap();
    let throttle = rates_commanded.thrust.unwrap();

    (pitch, roll, yaw, throttle)
}

/// Run the rate (inner) PID loop: This is what directly affects motor output by commanding pitch, roll, and
/// yaw rates. Also affects thrust. These rates are determined either directly by acro inputs, or by the
/// attitude PID loop.
///
/// If acro, we get our inputs each IMU update; ie the inner loop. In other modes,
/// (or with certain autopilot flags enabled?) the inner loop is commanded by the mid loop
/// once per update cycle, eg to set commanded angular rates.
pub fn run_rate_quad(
    params: &Params,
    input_mode: InputMode,
    autopilot_status: &AutopilotStatus,
    ch_data: &ChannelData,
    rates_commanded: &mut CtrlInputs,
    pid: &mut PidGroup,
    filters: &mut PidDerivFilters,
    current_pwr: &mut crate::MotorPower,
    rotor_mapping: &RotorMapping,
    motor_timers: &mut MotorTimers,
    dma: &mut Dma<DMA1>,
    coeffs: &CtrlCoeffGroup,
    max_speed_ver: f32,
    input_map: &InputMap,
    arm_status: ArmStatus,
    dt: f32,
) {
    // see thrust mapping code commented out in rate fixed_wing; it applies here too.

    // todo: This is probably not apt if we're using autopilot, ie not using ch data for throttle.
    let tpa_scaler = if ch_data.throttle > TPA_BREAKPOINT {
        tpa_adjustment(ch_data.throttle)
    } else {
        1.
    };

    // autopilot_status.apply_rate_quad(
    //     params,
    //     rates_commanded,
    //     max_speed_ver,
    //     pid,
    //     filters,
    //     coeffs,
    //     dt,
    // );

    let (pitch, roll, yaw, throttle) = rate_apply_common(
        pid,
        rates_commanded,
        params,
        ch_data,
        coeffs,
        filters,
        input_map,
        tpa_scaler,
        dt,
    );

    flight_ctrls::quad::apply_controls(
        pitch,
        roll,
        yaw,
        throttle,
        current_pwr,
        rotor_mapping,
        motor_timers,
        arm_status,
        dma,
    );
}

pub fn run_rate_fixed_wing(
    params: &Params,
    input_mode: InputMode,
    autopilot_status: &AutopilotStatus,
    ch_data: &ChannelData,
    rates_commanded: &mut CtrlInputs,
    pid: &mut PidGroup,
    filters: &mut PidDerivFilters,
    control_posits: &mut ControlPositions,
    mapping: &ServoWingMapping,
    motor_timers: &mut MotorTimers,
    dma: &mut Dma<DMA1>,
    coeffs: &CtrlCoeffGroup,
    input_map: &InputMap,
    arm_status: ArmStatus,
    dt: f32,
) {
    //         // todo: Power interp not yet implemented.
    //         // let power_interp_inst = dsp_sys::arm_linear_interp_instance_f32 {
    //         //     nValues: 11,
    //         //     x1: 0.,
    //         //     xSpacing: 0.1,
    //         //     pYData: [
    //         //         // Idle power.
    //         //         0.02, // Make sure this matches the above.
    //         //         POWER_LUT[0],
    //         //         POWER_LUT[1],
    //         //         POWER_LUT[2],
    //         //         POWER_LUT[3],
    //         //         POWER_LUT[4],
    //         //         POWER_LUT[5],
    //         //         POWER_LUT[6],
    //         //         POWER_LUT[7],
    //         //         POWER_LUT[8],
    //         //         POWER_LUT[9],
    //         //     ]
    //         //     .as_mut_ptr(),
    //         // };

    // autopilot_status.apply_rate_fixed_wing(params, rates_commanded);

    // For now, we don't have a TPA scaler for fixed, but perhaps adding one or something similar
    // wouldn't be a bad idea.
    let tpa_scaler = 1.;

    let (pitch, roll, _yaw, throttle) = rate_apply_common(
        pid,
        rates_commanded,
        params,
        ch_data,
        coeffs,
        filters,
        input_map,
        tpa_scaler,
        dt,
    );

    flight_ctrls::fixed_wing::apply_controls(
        pitch,
        roll,
        throttle,
        control_posits,
        mapping,
        motor_timers,
        arm_status,
        dma,
    );
}
