//! This module contains code related to the flight control PID loop. It can be thought of
//! as a sub-module for `flight_ctrls`.

use stm32_hal2::{
    pac::{TIM3, TIM5},
    timer::Timer,
};

use cmsis_dsp_api as dsp_api;

use crate::{
    flight_ctrls::{
        self, AutopilotMode, CommandState, CtrlConstraint, CtrlInputs, FlightCmd, IirInstWrapper,
        InputMode, ParamType, Params,
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
            k_p_s_from_v: 0.1,
            k_i_s_from_v: 0.,
            k_d_s_from_v: 0.,

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
    pub gain_throttle: f32,
}

#[derive(Default)]
struct PidGroup {
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
    unsafe {
        dsp_api::biquad_cascade_df1_f32(&mut filter.inner, &[error_d_prefilt], &mut error_d, 1);
    }

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
    // mid_flt_cmd: &mut FlightCmd,
    inner_flt_cmd: &mut FlightCmd,
    pid_mid: &mut PidGroup,
    pid_inner: &mut PidGroup,
    filters: &mut PidDerivFilters,
    input_mode: &InputMode,
    autopilot_mode: &AutopilotMode,
    cfg: &UserCfg,
    commands: &mut CommandState,
    coeffs: &CtrlCoeffGroup,
) {
    // todo: DRY. Maybe you just need to use the match to set flight_cmd?
    match autopilot_mode {
        AutopilotMode::None => {
            // Determine inputs to appclassly
            // let mut flight_cmd = FlightCmd::default();
            // let mut flight_cmd = FlightCmd::level();
            // flight_cmd.add_inputs(inputs);

            // todo: Offset this logic into a fn in `flight_ctrls`. mod. Same as in the IMU update.

            // todo: Over contraint, ie matching flight mode here, and in `from_inputs`.

            // todo: Experimenting with ditching the `FlightCmd` struct.
            // let flt_cmd = FlightCmd::from_inputs(inputs, *input_mode);

            match input_mode {
                InputMode::Acro => {
                    // In rate mode, simply update the inner command; don't do anything
                    // in the outer PID loop.
                    *inner_flt_cmd = mid_flt_cmd.clone();
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

                    // todo: For now, keep the naive approach. Test xy velocity-based
                    // todo approaches later, eg after this is working.

                    // Clamp max angle to specified limits.
                    // todo: Clamp total combined angle; not x and y individually?
                    let mut s_pitch = flt_cmd.y_pitch.unwrap().2;
                    let mut s_roll = flt_cmd.x_roll.unwrap().2;

                    if s_pitch > cfg.max_angle {
                        s_pitch = cfg.max_angle;
                    }
                    if s_roll > cfg.max_angle {
                        s_roll = cfg.max_angle;
                    }

                    let y_pitch = Some((CtrlConstraint::PitchRoll, ParamType::S, s_pitch));
                    let x_roll = Some((CtrlConstraint::PitchRoll, ParamType::S, s_roll));

                    *inner_flt_cmd = FlightCmd {
                        y_pitch,
                        x_roll,
                        ..mid_flt_cmd.clone()
                    }
                }
                InputMode::Command => {
                    // todo: This is a cheap version using V. It will drift!
                    // todo: A candidate for attitude mode as well. Here, you might
                    // todo need a third outer loop that handles position??

                    // todo: THis is actually overwriting the x_s etc based approach in
                    // todo the input parser. YOu need to find a way to make this cleaner.
                    // inner_flt_cmd.x_roll = Some((CtrlConstraint::Xy, ParamType::V, inputs.roll));
                    // inner_flt_cmd.y_pitch = Some((CtrlConstraint::Xy, ParamType::V, inputs.pitch));

                    // Stick position commands a horizontal velocity in this mode.

                    // todo: If level, command a loiter, where you drive the inner loop
                    // todo with position errors, not velocity errors.
                    // todo: You could also do this from an outer loop. Perhaps leave small
                    // todo corrections like this to the inner loop, and larger ones to the outer loop.

                    // todo: What happens in this approach if you quickly release the stick?
                    // todo: You could maybe always use the outer loop for this.

                    // todo: This may not be valid if the drone isn't roughly upright??
                    // todo: World to body conversion? (is that just to offset by yaw?)

                    *mid_flt_cmd = flt_cmd.clone();

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

                    match mid_flt_cmd.y_pitch {
                        Some(cmd) => {
                            match cmd.1 {
                                ParamType::S => {

                                }
                                ParamType::V => {

                                }
                                ParamType::A => (), // todo?
                            }
                        }
                        None => {

                        }
                    }

                    pid_mid.pitch = calc_pid_error(
                        mid_flt_cmd.y_pitch.unwrap().2,
                        param_y,
                        &pid_mid.pitch,
                        k_p_pitch,
                        k_i_pitch,
                        k_d_pitch,
                        &mut filters.mid_y,
                        DT,
                    );

                    pid_mid.roll = calc_pid_error(
                        mid_flt_cmd.x_roll.unwrap().2,
                        param_x,
                        &pid_mid.roll,
                        // coeffs,
                        k_p_roll,
                        k_i_roll,
                        k_d_roll,
                        &mut filters.mid_x,
                        DT,
                    );

                    // Attemps to maintain yaw as well as alt?
                    let yaw_commanded = commands.alt + mid_flt_cmd.yaw.unwrap().1;

                    pid_mid.yaw = calc_pid_error(
                        yaw_commanded,
                        params.s_yaw,
                        &pid_mid.yaw,
                        coeffs.yaw.k_p_s,
                        coeffs.yaw.k_i_s,
                        coeffs.yaw.k_d_s,
                        &mut filters.mid_thrust,
                        DT,
                    );

                    /// Add to or subtract from commanded alt.
                    let thrust_commanded = commands.alt + mid_flt_cmd.thrust.unwrap().1;

                    pid_mid.thrust = calc_pid_error(
                        thrust_commanded,
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

                    let command_s_roll = pid_mid.roll.out();
                    let command_s_pitch = pid_mid.pitch.out();
                    let command_s_yaw = pid_mid.yaw.out();
                    let command_s_thrust = pid_mid.thrust.out();

                    inner_flt_cmd.x_roll =
                        Some((CtrlConstraint::PitchRoll, ParamType::S, command_s_roll));

                    inner_flt_cmd.y_pitch =
                        Some((CtrlConstraint::PitchRoll, ParamType::S, command_s_pitch));

                    inner_flt_cmd.yaw = Some((ParamType::S, command_s_yaw));

                    inner_flt_cmd.thrust = Some((ParamType::S, command_s_thrust));
                }
            }
        }
        AutopilotMode::Takeoff => {
            *inner_flt_cmd = FlightCmd {
                y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                yaw: Some((ParamType::V, 0.)),
                thrust: Some((ParamType::V, flight_ctrls::takeoff_speed(params.s_z_agl))),
            };
        }
        AutopilotMode::Land => {
            *inner_flt_cmd = FlightCmd {
                y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                yaw: Some((ParamType::V, 0.)),
                thrust: Some((ParamType::V, -flight_ctrls::landing_speed(params.s_z_agl))),
            };
        }
        _ => (),
    }
}

/// Run the inner PID loop: This is what directly affects motor output by commanding pitch and roll
/// position or rate.
pub fn run_pid_inner(
    params: &Params,
    // inner_flt_cmd: &FlightCmd,
    inner_flt_cmd: &CtrlInputs,
    pid_inner: &mut PidGroup,
    filters: &mut PidDerivFilters,
    current_pwr: &mut crate::RotorPower,
    rotor_timer_a: &mut Timer<TIM3>,
    rotor_timer_b: &mut Timer<TIM5>,
    coeffs: &CtrlCoeffGroup,
    dt: f32,
) {
    match inner_flt_cmd.y_pitch {
        Some(cmd) => {
            let param = match cmd.1 {
                ParamType::S => params.s_pitch,
                ParamType::V => params.v_pitch,
                ParamType::A => params.a_pitch,
            };

            // todo: Use the right coeffs here, eg use the match arm above? Don't just use S for everythign.

            pid_inner.pitch = calc_pid_error(
                cmd.2,
                param,
                &pid_inner.pitch,
                coeffs.pitch.k_p_s_from_s,
                coeffs.pitch.k_i_s_from_s,
                coeffs.pitch.k_d_s_from_s,
                &mut filters.inner_y,
                dt,
            );
        }
        None => {
            pid_inner.pitch = Deafult::default();
            // todo. Consider how to handle this re not screwing up DT and the PID.
            // todo: You don't want to update the controls for this aspect (eg pitch) though.
        }
    }

    match inner_flt_cmd.x_roll {
        Some(cmd) => {
            let param = match cmd.1 {
                ParamType::S => params.s_roll,
                ParamType::V => params.v_roll,
                ParamType::A => params.a_roll,
            };

            pid_inner.roll = calc_pid_error(
                cmd.2,
                param,
                &pid_inner.roll,
                coeffs.roll.k_p_s_from_s,
                coeffs.roll.k_i_s_from_s,
                coeffs.roll.k_d_s_from_s,
                &mut filters.inner_x,
                dt,
            );
        }
        None => {
            pid_inner.roll = Deafult::default();
        }
    }

    match inner_flt_cmd.yaw {
        Some(cmd) => {
            let param = match cmd.0 {
                ParamType::S => params.s_yaw,
                ParamType::V => params.v_yaw,
                ParamType::A => params.a_yaw,
            };

            pid_inner.yaw = calc_pid_error(
                cmd.1,
                param,
                &pid_inner.yaw,
                coeffs.yaw.k_p_s,
                coeffs.yaw.k_i_s,
                coeffs.yaw.k_d_s,
                &mut filters.inner_yaw,
                dt,
            );
        }
        None => {
            pid_inner.yaw = Deafult::default();
        }
    }

    match inner_flt_cmd.thrust {
        Some(cmd) => {
            let param = match cmd.0 {
                ParamType::S => params.s_z_msl,
                ParamType::V => params.v_z,
                ParamType::A => params.a_z,
            };

            pid_inner.thrust = calc_pid_error(
                cmd.1,
                param,
                &pid_inner.thrust,
                coeffs.thrust.k_p_s,
                coeffs.thrust.k_i_s,
                coeffs.thrust.k_d_s,
                &mut filters.inner_thrust,
                dt,
            );
        }
        None => {
            pid_inner.thrust = Deafult::default();
        }
    }

    flight_ctrls::adjust_ctrls(
        &pid_inner.pitch,
        &pid_inner.roll,
        &pid_inner.yaw,
        &pid_inner.thrust,
        coeffs,
        current_pwr,
        rotor_timer_a,
        rotor_timer_b,
    );
}
