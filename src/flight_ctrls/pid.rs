//! This module contains code related to the flight control PID loop. It can be thought of
//! as a sub-module for `flight_ctrls`.
//!
//! See the OneNote document for notes on how we handle the more complicated / cascaded control modes.
//!
//! [Some info on the PID terms, focused on BF](https://gist.github.com/exocode/90339d7f946ad5f83dd1cf29bf5df0dc)
//! https://oscarliang.com/quadcopter-pid-explained-tuning/

use stm32_hal2::{dma::Dma, pac::DMA1};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as dsp_sys;

use crate::{
    control_interface::ChannelData,
    safety::ArmStatus,
    state::{SystemStatus, UserCfg},
    util::IirInstWrapper,
    DT_ATTITUDE,
};

use super::{
    attitude_ctrls,
    autopilot::AutopilotStatus,
    common::{AttitudeCommanded, CtrlInputs, InputMap, MotorTimers, Params, RatesCommanded},
    ControlMapping,
};

use lin_alg2::f32::Quaternion;

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::{ControlPositions};
    } else {
        use crate::flight_ctrls::{InputMode, MAX_ROTOR_POWER, POWER_LUT, YAW_ASSIST_COEFF, YAW_ASSIST_MIN_SPEED};
    }
}

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

cfg_if! {
    if #[cfg(feature = "quad")] {
        // "TPA" stands for Throttle PID attenuation - reduction in D term (or more) past a certain
        // throttle setting, linearly. This only applies to the rate loop.
        // https://github-wiki-see.page/m/betaflight/betaflight/wiki/PID-Tuning-Guide
        pub const TPA_MIN_ATTEN: f32 = 0.5; // At full throttle, D term is attenuated to this value.
        pub const TPA_BREAKPOINT: f32 = 0.3; // Start engaging TPA at this value.
                                             // `TPA_SLOPE` and `TPA_Y_INT` are cached calculations.
        const TPA_SLOPE: f32 = (TPA_MIN_ATTEN - 1.) / (MAX_ROTOR_POWER - TPA_BREAKPOINT);
        const TPA_Y_INT: f32 = -(TPA_SLOPE * TPA_BREAKPOINT - 1.);
    }
}

#[cfg(feature = "quad")]
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
    // Note that we don't use the D component for our velocity PID.
    pub pid_deriv_lowpass_cutoff_rate: LowpassCutoff,
    pub pid_deriv_lowpass_cutoff_attitude: LowpassCutoff,
}

impl Default for CtrlCoeffsPR {
    #[cfg(feature = "quad")]
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
            pid_deriv_lowpass_cutoff_rate: LowpassCutoff::H1k,
            pid_deriv_lowpass_cutoff_attitude: LowpassCutoff::H1k,
        }
    }

    #[cfg(feature = "fixed-wing")]
    fn default() -> Self {
        Self {
            k_p_rate: 0.06,
            // k_i_rate: 0.60,
            k_i_rate: 0.0,
            // k_d_rate: 0.02,
            k_d_rate: 0.00,

            // Attitude not used for now.

            // pid for controlling pitch and roll from commanded horizontal velocity
            k_p_attitude: 0.,
            k_i_attitude: 0.,
            k_d_attitude: 0.,

            k_p_velocity: 0.1,
            k_i_velocity: 0.,
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
    fn default() -> Self {
        Self {
            pitch: Default::default(),
            roll: Default::default(),
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

/// To reduce DRY between `run_attitude_quad` and `run_attitude_fixed_wing`. The main purpose of
/// this fn is to modify `rates_commanded`.
fn attitude_apply_common(
    pid_attitude: &mut PidGroup,
    attitude_commanded: &AttitudeCommanded,
    rates_commanded: &mut RatesCommanded,
    params: &Params,
    autopilot_commands: &CtrlInputs,
    coeffs: &CtrlCoeffGroup,
    filters: &mut PidDerivFilters,
) {
    // If an attitude has been commanded (eg a velocity loop,autopilot mode, or if the aircraft
    // is in attitude mode), apply the PID to it.
    if let Some(pitch_commanded) = autopilot_commands.pitch {
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

    if let Some(roll_commanded) = autopilot_commands.roll {
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

    if let Some(yaw_commanded) = autopilot_commands.yaw {
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

#[cfg(feature = "quad")]
/// Run the attitude (mid) PID loop: This is used to determine angular velocities, based on commanded
/// attitude. Modifies `rates_commanded`, which is used by the rate PID loop.
pub fn run_attitude(
    params: &Params,
    attitude_commanded: &AttitudeCommanded,
    rates_commanded: &mut RatesCommanded,
    autopilot_commands: &mut CtrlInputs,
    ch_data: &ChannelData,
    input_map: &InputMap,
    pid_attitude: &mut PidGroup,
    filters: &mut PidDerivFilters,
    input_mode: InputMode,
    autopilot_status: &AutopilotStatus,
    cfg: &UserCfg,
    coeffs: &CtrlCoeffGroup,
    optional_sensors: &SystemStatus,
) {
    // todo: Try to remove euler angles entirely!

    match input_mode {
        InputMode::Acro => {
            if cfg.attitude_based_rate_mode {
                // *attitudes_commanded = CtrlInputs {
                //     pitch: Some(input_map.calc_pitch_angle(ch_data.pitch)),
                //     roll: Some(input_map.calc_roll_angle(ch_data.roll)),
                //     yaw: Some(input_map.calc_yaw_rate(ch_data.yaw)),
                //     thrust: None,
                // };
            }
        }

        // If in Attitude control mode, command our initial (pre-autopilot) attitudes based on
        // control positions.
        InputMode::Attitude => {
            *attitude_commanded = AttitudeCommanded {
                quat: Some(attitude_ctrls::from_controls(ch_data)),
                ..Default::default()
            }
            // *attitudes_commanded = CtrlInputs {
            //     pitch: Some(input_map.calc_pitch_angle(ch_data.pitch)),
            //     roll: Some(input_map.calc_roll_angle(ch_data.roll)),
            //     yaw: Some(input_map.calc_yaw_rate(ch_data.yaw)),
            //     thrust: None,
            // };
        }
        InputMode::Command => {
            // todo: Impl
        }
    }

    attitude_apply_common(
        pid_attitude,
        attitude_commanded,
        rates_commanded,
        params,
        autopilot_commands,
        coeffs,
        filters,
    );
}

#[cfg(feature = "fixed-wing")]
/// Run the attitude (mid) PID loop: This is used to determine angular velocities, based on commanded
/// attitude. Note that for fixed wing, we have no direct attitude mode, so this is entirely determined
/// by the various autopilot modes, or if we're mapping throttle to airspeed etc.
/// Modifies `rates_commanded`, which is used by the rate PID loop.
pub fn run_attitude(
    params: &Params,
    attitude_commanded: &AttitudeCommanded,
    autopilot_commands: &mut CtrlInputs,
    rates_commanded: &mut CtrlInputs,
    pid_attitude: &mut PidGroup,
    filters: &mut PidDerivFilters,
    autopilot_status: &AutopilotStatus,
    coeffs: &CtrlCoeffGroup,
    system_status: &SystemStatus,
) {
    // Note that for fixed wing, we don't have attitude mode.

    attitude_apply_common(
        pid_attitude,
        rates_commanded,
        params,
        attitude_commanded,
        autopilot_commands,
        coeffs,
        filters,
    );
}

/// To reduce DRY between `run_rate_quad` and `run_rate_fixed_wing`. Modifies `rates_commanded`,
/// and returns PID output as pitch, roll, yaw, thrust.
///
/// If a given rate command is 0 (eg in acro mode with no autopilot modes), define that
/// rate by channel data.
fn rate_apply_common(
    pid_rate: &mut PidGroup,
    rates_commanded: &mut RatesCommanded,
    throttle_commanded: Option<f32>,
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

    let pitch = rates_commanded.pitch.unwrap();
    let roll = rates_commanded.roll.unwrap();
    let yaw = rates_commanded.yaw.unwrap();

    let throttle = match throttle_commanded {
        Some(t) => t,
        None => input_map.calc_manual_throttle(ch_data.throttle),
    };

    (pitch, roll, yaw, throttle)
}

#[cfg(feature = "quad")]
/// Run the rate (inner) PID loop: This is what directly affects motor output by commanding pitch, roll, and
/// yaw rates. Also affects thrust. These rates are determined either directly by acro inputs, or by the
/// attitude PID loop.
///
/// If acro, we get our inputs each IMU update; ie the inner loop. In other modes,
/// (or with certain autopilot flags enabled?) the inner loop is commanded by the mid loop
/// once per update cycle, eg to set commanded angular rates.
pub fn run_rate(
    params: &Params,
    ch_data: &ChannelData,
    rates_commanded: &mut RatesCommanded,
    throttle_commanded: Option<f32>,
    pid: &mut PidGroup,
    filters: &mut PidDerivFilters,
    current_pwr: &mut crate::MotorPower,
    mapping: &ControlMapping,
    motor_timers: &mut MotorTimers,
    dma: &mut Dma<DMA1>,
    coeffs: &CtrlCoeffGroup,
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

    let (pitch, roll, yaw, throttle) = rate_apply_common(
        pid,
        rates_commanded,
        throttle_commanded,
        params,
        ch_data,
        coeffs,
        filters,
        input_map,
        tpa_scaler,
        dt,
    );

    super::apply_controls(
        pitch,
        roll,
        yaw,
        throttle,
        current_pwr,
        mapping,
        motor_timers,
        arm_status,
        dma,
    );
}

#[cfg(feature = "fixed-wing")]
pub fn run_rate(
    params: &Params,
    ch_data: &ChannelData,
    rates_commanded: &mut RatesCommanded,
    throttle_commanded: Option<f32>,
    pid: &mut PidGroup,
    filters: &mut PidDerivFilters,
    control_posits: &mut ControlPositions,
    mapping: &ControlMapping,
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

    // For now, we don't have a TPA scaler for fixed, but perhaps adding one or something similar
    // wouldn't be a bad idea.
    let tpa_scaler = 1.;

    let (pitch, roll, _yaw, throttle) = rate_apply_common(
        pid,
        rates_commanded,
        throttle_commanded,
        params,
        ch_data,
        coeffs,
        filters,
        input_map,
        tpa_scaler,
        dt,
    );

    super::apply_controls(
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
