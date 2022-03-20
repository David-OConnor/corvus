//! This module contains code related to the flight control PID loop. It can be thought of
//! as a sub-module for `flight_ctrls`.

use core::f32::consts::TAU;

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::Timer,
};

use cmsis_dsp_api as dsp_api;

use crate::{
    flight_ctrls::{
        self, AltType, AutopilotStatus, CommandState, CtrlInputs, IirInstWrapper, InputMap,
        InputMode, Params, YAW_ASSIST_COEFF, YAW_ASSIST_MIN_SPEED,
    },
    UserCfg, DT,
};

// todo: What should these be?
const INTEGRATOR_CLAMP_MIN: f32 = -10.;
const INTEGRATOR_CLAMP_MAX: f32 = 10.;

static mut FILTER_STATE_MID_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_MID_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_MID_YAW: [f32; 4] = [0.; 4];
static mut FILTER_STATE_MID_THRUST: [f32; 4] = [0.; 4];

static mut FILTER_STATE_INNER_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_INNER_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_INNER_YAW: [f32; 4] = [0.; 4];
static mut FILTER_STATE_INNER_THRUST: [f32; 4] = [0.; 4];

/// Cutoff frequency for our PID lowpass frequency, in Hz
#[derive(Clone, Copy)]
enum LowpassCutoff {
    // todo: What values should these be?
    H500,
    H1k,
    H10k,
    H20k,
}

/// Coefficients and other configurable parameters for controls, for pich and roll.
/// Has several variants, due to coupling with horizontal (X and Y) movement.
pub struct CtrlCoeffsPR {
    // // These coefficients map desired change in flight parameters to rotor power change.
    // // pitch, roll, and yaw s are in power / radians
    // pid_pitch_s: f32,
    // pid_roll_s: f32,
    // pid_yaw_s: f32,
    // pid_z_s: f32,

    // pid for controlling pitch and roll from commanded horizontal position
    k_p_s_from_s: f32,
    k_i_s_from_s: f32,
    k_d_s_from_s: f32,

    // pid for controlling pitch and roll from commanded horizontal velocity
    k_p_s_from_v: f32,
    k_i_s_from_v: f32,
    k_d_s_from_v: f32,

    // PID for controlling pitch and roll rate directly (eg Acro)
    k_p_v_direct: f32,
    k_i_v_direct: f32,
    k_d_v_direct: f32,

    pid_deriv_lowpass_cutoff: LowpassCutoff,
}

impl Default for CtrlCoeffsPR {
    fn default() -> Self {
        Self {
            // pid for controlling pitch and roll from commanded horizontal position
            k_p_s_from_s: 0.1,
            k_i_s_from_s: 0.,
            k_d_s_from_s: 0.,

            // pid for controlling pitch and roll from commanded horizontal velocity
            k_p_s_from_v: 47.,
            k_i_s_from_v: 84.,
            k_d_s_from_v: 34.,

            // PID for controlling pitch and roll rate directly (eg Acro)
            k_p_v_direct: 0.1,
            k_i_v_direct: 0.,
            k_d_v_direct: 0.,

            pid_deriv_lowpass_cutoff: LowpassCutoff::H1k,
        }
    }
}

/// Coefficients and other configurable parameters for yaw and thrust. This is separate from, and
/// simpler than the variant for pitch and roll, since it's not coupled to X and Y motion.
pub struct CtrlCoeffsYT {
    // PID for controlling yaw or thrust from an explicitly-commanded heading or altitude.
    k_p_s: f32,
    k_i_s: f32,
    k_d_s: f32,

    // PID for controlling yaw or thrust from a velocity directly applied to them. (Eg Acro and attitude)
    k_p_v: f32,
    k_i_v: f32,
    k_d_v: f32,

    pid_deriv_lowpass_cutoff: LowpassCutoff,
}

impl Default for CtrlCoeffsYT {
    fn default() -> Self {
        Self {
            k_p_s: 0.1,
            k_i_s: 0.0,
            k_d_s: 0.0,

            k_p_v: 45.,
            k_i_v: 80.0,
            k_d_v: 0.0,
            pid_deriv_lowpass_cutoff: LowpassCutoff::H1k,
        }
    }
}

pub struct CtrlCoeffGroup {
    pub pitch: CtrlCoeffsPR,
    pub roll: CtrlCoeffsPR,
    pub yaw: CtrlCoeffsYT,
    pub thrust: CtrlCoeffsYT,

    // These coefficients are our rotor gains.
    // todo: Think about how to handle these, and how to map them to the aircraft data struct,
    // todo, and the input range.
    pub gain_pitch: f32,
    pub gain_roll: f32,
    pub gain_yaw: f32,
    pub gain_thrust: f32,
}

impl Default for CtrlCoeffGroup {
    /// These starting values are Betaflight defaults.
    fn default() -> Self {
        Self {
            pitch: Default::default(),
            roll: CtrlCoeffsPR {
                k_p_s_from_v: 45.,
                k_i_s_from_v: 80.,
                k_d_s_from_v: 30.,
                ..Default::default()
            },
            yaw: Default::default(),
            thrust: CtrlCoeffsYT {
                k_p_s: 0.1,
                k_i_s: 0.0,
                k_d_s: 0.0,

                k_p_v: 45.,
                k_i_v: 80.0,
                k_d_v: 0.0,
                pid_deriv_lowpass_cutoff: LowpassCutoff::H1k,
            },

            // These coefficients are our rotor gains.
            // todo: Think about how to handle these, and how to map them to the aircraft data struct,
            // todo, and the input range.
            gain_pitch: 0.,
            gain_roll: 0.,
            gain_yaw: 0.,
            gain_thrust: 0.,
        }
    }
}

#[derive(Default)]
pub struct PidGroup {
    pitch: PidState,
    roll: PidState,
    yaw: PidState,
    thrust: PidState,
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

/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop.
pub struct PidDerivFilters {
    pub mid_x: IirInstWrapper,
    pub mid_y: IirInstWrapper,
    pub mid_yaw: IirInstWrapper,
    pub mid_thrust: IirInstWrapper,

    pub inner_x: IirInstWrapper,
    pub inner_y: IirInstWrapper,
    pub inner_yaw: IirInstWrapper,
    pub inner_thrust: IirInstWrapper,
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
            mid_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            mid_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            mid_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            mid_thrust: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            inner_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            inner_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            inner_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            inner_thrust: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            // todo: Re-initialize fn?
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.mid_x.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_X,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.mid_y.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_Y,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.mid_yaw.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_YAW,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.mid_thrust.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_THRUST,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.inner_x.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_X,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.inner_y.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_Y,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.inner_yaw.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_YAW,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.inner_thrust.inner,
                &coeffs,
                &mut FILTER_STATE_INNER_THRUST,
            );
        }

        result
    }
}

/// Calculate the PID error given flight parameters, and a flight
/// command.
/// Example: https://github.com/pms67/PID/blob/master/PID.c
fn calc_pid_error(
    set_pt: f32,
    measurement: f32,
    prev_pid: &PidState,
    // coeffs: CtrlCoeffsGroup,
    k_p: f32,
    k_i: f32,
    k_d: f32,
    filter: &mut IirInstWrapper,
    // This `dt` is dynamic, since we don't necessarily run this function at a fixed interval.
    dt: f32,
) -> PidState {
    // Find appropriate control inputs using PID control.

    let error = set_pt - measurement;

    // todo: Determine if you want a deriv component at all; it's apparently not commonly used.

    // todo: Minor optomization: Store the constant terms once, and recall instead of calcing
    // todo them each time (eg the parts with DT, 2., tau.
    // https://www.youtube.com/watch?v=zOByx3Izf5U
    let error_p = k_p * error;
    let error_i = k_i * dt / 2. * (error + prev_pid.e) + prev_pid.i;
    // Derivative on measurement vice error, to avoid derivative kick.
    let error_d_prefilt = k_d * (measurement - prev_pid.measurement) / dt;

    // todo: Avoid this DRY with a method on `filter`
    // let mut error_d_v_pitch = [0.];
    let mut error_d = [0.];

    dsp_api::biquad_cascade_df1_f32(&mut filter.inner, &[error_d_prefilt], &mut error_d, 1);

    let mut error = PidState {
        measurement,
        e: error,
        p: error_p,
        i: error_i,
        d: error_d[0],
    };

    error.anti_windup_clamp();

    error
}

/// Run the mid PID loop: This is used to determine pitch and attitude, eg based on commanded speed
/// or position.
pub fn run_pid_mid(
    params: &Params,
    inputs: &CtrlInputs,
    input_map: &InputMap,
    inner_flt_cmd: &mut CtrlInputs,
    pid_mid: &mut PidGroup,
    filters: &mut PidDerivFilters,
    input_mode: &InputMode,
    autopilot_status: &AutopilotStatus,
    cfg: &UserCfg,
    commands: &mut CommandState,
    coeffs: &CtrlCoeffGroup,
) {
    match input_mode {
        InputMode::Acro => {
            // (Acro mode has handled exclusively by the inner loop, for now at least)

            // In rate mode, simply update the inner command; don't do anything
            // in the outer PID loop.

            // todo: If in acro mode, consider moving some of these to the inner loop for faster updates!
            // todo eg control polling.
            //
            // *inner_flt_cmd = CtrlInputs {
            //     pitch: input_map.calc_pitch_rate(inputs.pitch),
            //     roll: input_map.calc_roll_rate(inputs.roll),
            //     yaw: input_map.calc_yaw_rate(inputs.yaw),
            //     thrust: input_map.calc_pitch_rate(inputs.thrust),
            // };
            //
            // // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
            // // either MSL or AGL.
            // // if let AutopilotMode::AltHold(alt_type, alt_commanded) = autopilot_mode {
            // if let Some((alt_type, alt_commanded)) = autopilot_status.alt_hold {
            //     // todo: This naive approach, or command a velocity, like in Cmd mode? Probably latter.
            //     // inner_flt_cmd.thrust = *alt_commanded; // See the inner loop for more on how this is handled.
            //
            //     // Set a vertical velocity for the inner loop to maintain, based on distance
            //     let dist = match alt_type {
            //         AltType::Msl => alt_commanded - params.s_z_msl,
            //         AltType::Agl => alt_commanded - params.s_z_agl,
            //     };
            //     // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
            //     inner_flt_cmd.thrust =
            //         flight_ctrls::enroute_speed_ver(dist, cfg.max_speed_ver, params.s_z_agl);
            // }
            //
            // if autopilot_status.yaw_assist {
            //     // Blend manual inputs with teh autocorrection factor. If there are no manual inputs,
            //     // this autopilot mode should neutralize all sideslip.
            //
            //     // todo: Instead of comparing heading to VVI, maybe you could just neutralize
            //     // X/Y acceleration?
            //
            //     // let net_hor_direction = 0.;
            //     // let current_yaw = params.s_yaw;
            //
            //     // todo: Do we want a function that maps to a commanded yaw velocity, or
            //     // todo, something more simple?
            //
            //     // let yaw_correction_factor = 0.; // todo
            //
            //     // todo: What should this coeff be? Is linear OK?
            //
            //     // if coeff = 0.5, if accel is 1 m/s^2, yaw correction is 1/2 rad/s
            //     // angular velocity / accel: (radians/s) / (m/s^2) = radiants x s / m
            //     let coeff = 0.1;
            //
            //     let yaw_correction_factor = params.a_x * coeff;
            //
            //     // todo: Impl for Attitude mode too? Or is it not appropriate there?
            //     inner_flt_cmd.yaw += yaw_correction_factor;
            // } else if autopilot_roll_assist {
            //     let coeff = 0.1;
            //
            //     let roll_correction_factor = -params.a_x * coeff;
            //
            //     // todo: Impl for Attitude mode too.
            //     inner_flt_cmd.yaw += roll_correction_factor;
            // }
        }

        InputMode::Attitude => {
            // In attitude, our outer loop (eg controls specifying pitch and roll
            // position) determines how to set our inner loop, which sets the pitch
            // and roll. The throttle command sets altitude. Yaw behaves the
            // same as in Rate mode.

            // Note that a naive approach would be to do nothing here, as in
            // Rate mode, and let the inner loop manage pitch instead of pitch rate.
            // Here, we compensate for wind etc, but this will still drift,
            // since we don't attempt to maintain a position.

            *inner_flt_cmd = CtrlInputs {
                pitch: input_map.calc_pitch_angle(inputs.pitch),
                roll: input_map.calc_roll_angle(inputs.roll),
                yaw: input_map.calc_yaw_rate(inputs.yaw),
                thrust: input_map.calc_pitch_rate(inputs.thrust),
            };

            // todo: DRY from above in Acro mode for alt hold autopilot enabled.
            // if let AutopilotMode::AltHold(alt_type, alt_commanded) = autopilot_mode {
            if let Some((alt_type, alt_commanded)) = autopilot_status.alt_hold {
                // todo: This naive approach, or command a velocity, like in Cmd mode? Probably latter.
                // inner_flt_cmd.thrust = *alt_commanded; // See the inner loop for more on how this is handled.

                // Set a vertical velocity for the inner loop to maintain, based on distance
                let dist = match alt_type {
                    AltType::Msl => alt_commanded - params.s_z_msl,
                    AltType::Agl => alt_commanded - params.s_z_agl,
                };
                // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
                inner_flt_cmd.thrust =
                    flight_ctrls::enroute_speed_ver(dist, cfg.max_speed_ver, params.s_z_agl);
            }
        }
        InputMode::Command => {
            // todo: Impl
            // match autopilot_mode {
            if autopilot_status.takeoff {
                // AutopilotMode::Takeoff => {
                *inner_flt_cmd = CtrlInputs {
                    pitch: 0.,
                    roll: 0.,
                    yaw: 0.,
                    thrust: flight_ctrls::takeoff_speed(params.s_z_agl, cfg.max_speed_ver),
                };
            }
            // AutopilotMode::Land => {
            else if autopilot_status.land {
                *inner_flt_cmd = CtrlInputs {
                    pitch: 0.,
                    roll: 0.,
                    yaw: 0.,
                    thrust: flight_ctrls::landing_speed(params.s_z_agl, cfg.max_speed_ver),
                };
            }
            // _ => (),

            let mut param_x = params.v_x;
            let mut param_y = params.v_y;

            let mut k_p_pitch = coeffs.pitch.k_p_s_from_v;
            let mut k_i_pitch = coeffs.pitch.k_i_s_from_v;
            let mut k_d_pitch = coeffs.pitch.k_d_s_from_v;

            let mut k_p_roll = coeffs.roll.k_p_s_from_v;
            let mut k_i_roll = coeffs.roll.k_i_s_from_v;
            let mut k_d_roll = coeffs.roll.k_d_s_from_v;

            let eps1 = 0.01;
            if inputs.pitch > eps1 || inputs.roll > eps1 {
                commands.loiter_set = false;
            }

            let eps2 = 0.01;
            // todo: Commanded velocity 0 to trigger loiter logic, or actual velocity?
            // if mid_flight_cmd.y_pitch.unwrap().2 < eps && mid_flight_cmd.x_roll.unwrap().2 < eps {
            if params.s_x < eps2 && params.s_y < eps2 {
                if !commands.loiter_set {
                    commands.x = params.s_x;
                    commands.y = params.s_y;
                    commands.loiter_set = true;
                }

                param_x = commands.x;
                param_y = commands.y;

                k_p_pitch = coeffs.pitch.k_p_s_from_s;
                k_i_pitch = coeffs.pitch.k_i_s_from_s;
                k_d_pitch = coeffs.pitch.k_d_s_from_s;

                k_p_roll = coeffs.roll.k_p_s_from_s;
                k_i_roll = coeffs.roll.k_i_s_from_s;
                k_d_roll = coeffs.roll.k_d_s_from_s;
            }

            // todo
            // let pitch_cmd = input_map.calc_pitch_rate(inputs.pitch); ??
            // let roll_cmd = 0.;
            // let yaw_cmd = 0.;
            // let thrust_cmd = 0.;

            pid_mid.pitch = calc_pid_error(
                pitch_cmd,
                param_y,
                &pid_mid.pitch,
                k_p_pitch,
                k_i_pitch,
                k_d_pitch,
                &mut filters.mid_y,
                DT,
            );

            pid_mid.roll = calc_pid_error(
                roll_cmd,
                param_x,
                &pid_mid.roll,
                // coeffs,
                k_p_roll,
                k_i_roll,
                k_d_roll,
                &mut filters.mid_x,
                DT,
            );

            pid_mid.yaw = calc_pid_error(
                yaw_cmd,
                params.s_yaw,
                &pid_mid.yaw,
                coeffs.yaw.k_p_s,
                coeffs.yaw.k_i_s,
                coeffs.yaw.k_d_s,
                &mut filters.mid_thrust,
                DT,
            );

            pid_mid.thrust = calc_pid_error(
                thrust_cmd,
                params.s_z_msl,
                &pid_mid.thrust,
                coeffs.thrust.k_p_s,
                coeffs.thrust.k_i_s,
                coeffs.thrust.k_d_s,
                &mut filters.mid_thrust,
                DT,
            );

            // Determine commanded pitch and roll positions, and z velocity,
            // based on our middle-layer PID.

            *inner_flt_cmd = CtrlInputs {
                pitch: pid_mid.pitch.out(),
                roll: pid_mid.roll.out(),
                yaw: pid_mid.yaw.out(),
                thrust: pid_mid.thrust.out(),
            };
        }
    }
}

/// Run the inner PID loop: This is what directly affects motor output by commanding pitch and roll
/// position or rate. Also commands yaw and thrust rates. `inputs` for pitch and roll could be
/// position or velocity, depending on `input_mode`.
pub fn run_pid_inner(
    params: &Params,
    input_mode: InputMode,
    autopilot_status: &AutopilotStatus,
    manual_inputs: &mut CtrlInputs,
    inner_flt_cmd: &mut CtrlInputs,
    pid_inner: &mut PidGroup,
    filters: &mut PidDerivFilters,
    current_pwr: &mut crate::RotorPower,
    rotor_timer_a: &mut Timer<TIM2>,
    rotor_timer_b: &mut Timer<TIM3>,
    dma: &mut Dma<DMA1>,
    coeffs: &CtrlCoeffGroup,
    dt: f32,
) {
    match input_mode {
        InputMode::Acro => {
            // If acro, we get our inputs each IMU update; ie the inner loop.

            *inner_flt_cmd = CtrlInputs {
                pitch: input_map.calc_pitch_rate(manual_inputs.pitch),
                roll: input_map.calc_roll_rate(manual_inputs.roll),
                yaw: input_map.calc_yaw_rate(manual_inputs.yaw),
                thrust: input_map.calc_thrust_rate(manual_inputs.thrust),
            };

            // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
            // either MSL or AGL.
            // if let AutopilotMode::AltHold(alt_type, alt_commanded) = autopilot_mode {
            if let Some((alt_type, alt_commanded)) = autopilot_status.alt_hold {
                // todo: This naive approach, or command a velocity, like in Cmd mode? Probably latter.
                // inner_flt_cmd.thrust = *alt_commanded; // See the inner loop for more on how this is handled.

                // todo: In this mode, consider having the left stick being in the middle ~50% of the range mean
                // todo hold alt, and the upper and lower 25% meaning increase set point and decrease set
                // todo point respectively.

                // Set a vertical velocity for the inner loop to maintain, based on distance
                let dist = match alt_type {
                    AltType::Msl => alt_commanded - params.s_z_msl,
                    AltType::Agl => alt_commanded - params.s_z_agl,
                };
                // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
                inner_flt_cmd.thrust =
                    flight_ctrls::enroute_speed_ver(dist, cfg.max_speed_ver, params.s_z_agl);
            }

            if autopilot_status.yaw_assist {
                // Blend manual inputs with the autocorrection factor. If there are no manual inputs,
                // this autopilot mode should neutralize all sideslip.
                let hor_dir = 0.; // radians
                let hor_speed = 0.; // m/s

                let yaw_correction_factor = ((hor_dir - params.s_yaw) / TAU) * YAW_ASSIST_COEFF;

                // todo: Impl for Attitude mode too? Or is it not appropriate there?
                if hor_speed > YAW_ASSIST_MIN_SPEED {
                    inner_flt_cmd.yaw += yaw_correction_factor;
                }
            } else if autopilot_roll_assist {
                // todo!
                let hor_dir = 0.; // radians
                let hor_speed = 0.; // m/s

                let roll_correction_factor = (-(hor_dir - params.s_yaw) / TAU) * YAW_ASSIST_COEFF;

                // todo: Impl for Attitude mode too?
                if hor_speed > YAW_ASSIST_MIN_SPEED {
                    inner_flt_cmd.yaw += roll_correction_factor;
                }
            }
        }
        _ => {} // todo otherwise, handled in mid loop?
    }

    // In Acro mode, the inner PID loops controls pitch and roll in terms of rotational velocities. In other mode,
    // it controls them for specific attitudes.
    let (param_pitch, param_roll, param_yaw, mut param_thrust) = match input_mode {
        InputMode::Acro => {
            let mut thrust_param = params.v_z;

            // todo: If at more than 90 degrees attitude and below (or near?) commanded altitude,
            // todo: cut motor power entirely until level if an Acro + alt hold mode.

            (params.v_pitch, params.v_roll, params.v_yaw, thrust_param)
        }
        InputMode::Attitude => (params.s_pitch, params.s_roll, params.v_yaw, params.v_z),
        InputMode::Command => (params.s_pitch, params.s_roll, params.s_yaw, params.s_z_msl),
    };

    // todo: Messy / DRY
    let (coeffs_pitch, coeffs_roll, coeffs_yaw, coeffs_thrust) = match input_mode {
        InputMode::Acro => (
            (
                coeffs.pitch.k_p_s_from_v,
                coeffs.pitch.k_i_s_from_v,
                coeffs.pitch.k_d_s_from_v,
            ),
            (
                coeffs.roll.k_p_s_from_v,
                coeffs.roll.k_i_s_from_v,
                coeffs.roll.k_d_s_from_v,
            ),
            (coeffs.yaw.k_p_v, coeffs.yaw.k_i_v, coeffs.yaw.k_d_v),
            (
                coeffs.thrust.k_p_v,
                coeffs.thrust.k_i_v,
                coeffs.thrust.k_d_v,
            ),
        ),
        InputMode::Attitude => (
            (
                coeffs.pitch.k_p_s_from_s,
                coeffs.pitch.k_i_s_from_s,
                coeffs.pitch.k_d_s_from_s,
            ),
            (
                coeffs.roll.k_p_s_from_s,
                coeffs.roll.k_i_s_from_s,
                coeffs.roll.k_d_s_from_s,
            ),
            (coeffs.yaw.k_p_v, coeffs.yaw.k_i_v, coeffs.yaw.k_d_v),
            (
                coeffs.thrust.k_p_v,
                coeffs.thrust.k_i_v,
                coeffs.thrust.k_d_v,
            ),
        ),
        InputMode::Command => (
            (
                coeffs.pitch.k_p_s_from_s,
                coeffs.pitch.k_i_s_from_s,
                coeffs.pitch.k_d_s_from_s,
            ),
            (
                coeffs.roll.k_p_s_from_s,
                coeffs.roll.k_i_s_from_s,
                coeffs.roll.k_d_s_from_s,
            ),
            (coeffs.yaw.k_p_s, coeffs.yaw.k_i_s, coeffs.yaw.k_d_s),
            (
                coeffs.thrust.k_p_s,
                coeffs.thrust.k_i_s,
                coeffs.thrust.k_d_s,
            ),
        ),
    };

    pid_inner.pitch = calc_pid_error(
        inputs.pitch,
        param_pitch,
        &pid_inner.pitch,
        coeffs_pitch.0,
        coeffs_pitch.1,
        coeffs_pitch.2,
        &mut filters.inner_y,
        dt,
    );

    pid_inner.roll = calc_pid_error(
        inputs.roll,
        param_roll,
        &pid_inner.roll,
        coeffs_roll.0,
        coeffs_roll.1,
        coeffs_roll.2,
        &mut filters.inner_x,
        dt,
    );

    pid_inner.yaw = calc_pid_error(
        inputs.yaw,
        param_yaw,
        &pid_inner.yaw,
        coeffs_yaw.0,
        coeffs_yaw.1,
        coeffs_yaw.2,
        &mut filters.inner_yaw,
        dt,
    );

    pid_inner.thrust = calc_pid_error(
        inputs.thrust,
        param_thrust,
        &pid_inner.thrust,
        coeffs_thrust.0,
        coeffs_thrust.1,
        coeffs_thrust.2,
        &mut filters.inner_thrust,
        dt,
    );

    flight_ctrls::apply_ctrls(
        &pid_inner.pitch,
        &pid_inner.roll,
        &pid_inner.yaw,
        &pid_inner.thrust,
        coeffs,
        current_pwr,
        rotor_timer_a,
        rotor_timer_b,
        dma
    );
}
