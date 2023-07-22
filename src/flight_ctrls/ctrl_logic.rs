//! This module contains code for control logic. It commands constant-jerk corrections to
//! attitude and rate-of-change of attitude.
//!
//! See the comments here, the accompanying Python script in this project, and the associated
//! One note file for details on how we calculate this.

use crate::{
    control_interface::ChannelData,
    util::{self, map_linear},
    ATT_CMD_UPDATE_RATIO, DT_FLIGHT_CTRLS,
};

use super::{
    common::{CtrlMix, InputMap},
    ctrl_effect_est::{AccelMap, AccelMaps},
    filters::FlightCtrlFilters,
    motor_servo::{MotorServoState, RotationDir},
};

use ahrs::{Params, FORWARD, RIGHT, UP};

use lin_alg2::f32::{Quaternion, Vec3};

use num_traits::float::Float; // For sqrt.

use cfg_if::cfg_if;

// todo: YOu probably need filters.

use defmt::println;

cfg_if! {
    if #[cfg(feature = "quad")] {
        use super::motor_servo::MotorRpm;
    } else {
        use super::motor_servo::CtrlSfcPosits;
    }
}

/// Represents a torque; ie a rotation with an associated angular velocity
pub struct _Torque {
    /// A unit vector. Uses the right hand rule.
    pub axis: Vec3,
    /// In radians-per-second.
    pub angular_velocity: f32,
}

impl Default for _Torque {
    fn default() -> Self {
        Self {
            axis: Vec3::new(1., 0., 0.),
            angular_velocity: 0.,
        }
    }
}

impl _Torque {
    // todo: Do you need/want this?
    pub fn from_components(pitch: f32, roll: f32, yaw: f32) -> Self {
        // todo: No idea if this is correct
        let vec = UP * yaw + RIGHT * pitch + FORWARD * roll;
        Self {
            axis: vec.to_normalized(),
            angular_velocity: vec.magnitude(),
        }
    }

    /// Construct torque from 2 quaternions, and the time the rotation takes.
    /// This assumes less than a full rotation between updates.
    pub fn from_attitudes(att_prev: Quaternion, att_this: Quaternion, dt: f32) -> Self {
        let rotation = att_this * att_prev.inverse();

        Self {
            axis: rotation.axis(),
            angular_velocity: rotation.angle() / dt,
        }
    }
}

/// Construct a by-axis representation angular velocities from 2 quaternions, and the time the rotation takes.
/// This assumes less than a full rotation between updates.
pub fn ang_v_from_attitudes(
    att_prev: Quaternion,
    att_this: Quaternion,
    dt: f32,
) -> (f32, f32, f32) {
    let rotation = att_this * att_prev.inverse();

    let (x_comp, y_comp, z_comp) = rotation.to_axes();

    (x_comp * dt, y_comp * dt, z_comp * dt)
}

// The motor RPM of each motor will not go below this. We use this for both quad and fixed-wing motors.
// todo: unimplemented
const IDLE_RPM: f32 = 100.;

#[derive(Default)]
pub struct DragCoeffs {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

/// Control coefficients that affect the toleranaces and restrictions of the flight controls.
pub struct CtrlCoeffs {
    /// todo: For fixed-wing, you should probably have separate roll and pitch values.
    /// If unable to find a linear jerk to cause a correction given the current parameters (or
    /// the resulting TTC is too long), use this TTC, and set an arbitrary thrust to achieve it.
    /// Units: s/rad
    pub ttc_per_dθ: f32,
    /// If the calculated ttc from the continous-accel calculation is over this,
    /// use the discontinous logic. In rad/s
    pub max_ttc_per_dθ: f32,
}

// todo: Maybe a sep `CtrlCoeffs` struct for each axis - especially for fixed-wing!
// todo: Or, have a fixed-wing pitch (or roll) scaler.

impl Default for CtrlCoeffs {
    #[cfg(feature = "quad")]
    fn default() -> Self {
        Self {
            ttc_per_dθ: 0.3,
            max_ttc_per_dθ: 0.5,
        }
    }

    #[cfg(feature = "fixed-wing")]
    fn default() -> Self {
        Self {
            ttc_per_dθ: 0.5,
            max_ttc_per_dθ: 0.7,
        }
    }
}

/// Estimate the linear drag coefficient, given a single data point.
pub fn calc_drag_coeff(ω_meas: f32, α_meas: f32, α_commanded: f32) -> f32 {
    // https://physics.stackexchange.com/questions/304742/angular-drag-on-body
    // This coefficient maps angular velocity to drag acceleration directly,
    // and is measured (and filtered).

    // For "low-speeds", drag is proportionanl to ω. For high speeds, it's
    // prop to ω^2. The distinction is the reynolds number.

    // todo: Low speed for now.
    // drag_accel = -cω
    // α = α_commanded + cω
    // c = (α - α_commanded) / ω
    // α_commanded = α + cω

    (α_meas - α_commanded) / ω_meas
}

/// If we're unable to use our current parameters to determine a linear-jerk trajectory,
/// we specify a time-to-correction based on the params, then using that time, calculate
/// an angular accel and jerk. Returned result is α_0, j
fn α_from_ttc(θ_0: f32, ω_0: f32, θ_tgt: f32, ω_tgt: f32, ttc_per_dθ: f32) -> f32 {
    // Time to correction
    let ttc = ttc_per_dθ * (θ_tgt - θ_0).abs(); // todo: Naive. Take vel and accel into account.

    // static mut i: u32 = 0;
    // unsafe { i += 1 };
    // if unsafe { i } % 3_000 == 0 {
    //     println!("ttc inner: {}", ttc);
    // }

    // Calculate the "initial" target angular acceleration, from the formula we worked out
    // from the kinematic equations for θ(t) and ω(t).
    -2. / ttc.powi(2) * (ttc * (ω_tgt + 2. * ω_0) + 3. * (θ_0 - θ_tgt))
}

// #[cfg(feature = "quad")]
// /// Accel is in m/s^2. Returns an RPM delta between rotor pairs.
// fn accel_to_rpm_delta(α: f32, mapping: asdf) -> f32 {
//
// }
//
// #[cfg(feature = "fixed-wing")]
// /// Accel is in m/s^2. Returns an RPM delta between rotor pairs.
// fn accel_to_rpm_servo_cmds(α: f32, mapping: asdf) -> f32 {
//
// }

// todo: Due to numerical precision issues, should you work in ms?

/// Calculate the time required to perform an angle and angular rate correction, for a given axis.
/// This assumes constant jerk. If this is not possible given these constraints, returns `None`.
/// This function does not check if the time is longer than suitable; we handle that elsewhere.
///
/// We accomplish this by solving the formula
fn find_time_to_correct(
    θ_0: f32, ω_0: f32, ω_dot_0: f32, θ_tgt: f32, ω_tgt: f32
) -> Option<f32> {
    const EPS: f32 = 0.000001;

    if (ω_dot_0).abs() < EPS && (θ_0 - θ_tgt).abs() > EPS && (ω_tgt + 2. * ω_0).abs() > EPS {
        // todo: QC this change that incorporates dω
        Some(3. * (θ_tgt - θ_0) / (ω_tgt + 2. * ω_0))
    } else if (ω_dot_0).abs() < EPS {
        // This happens if there's 0 initial angular acceleration, no change in desired
        // attitude, but we do have a desired change in angular velocity. (todo is this right?)
        None
    } else {
        // If `inner` is negative, there is no solution given our initial conditions, and
        // constant jerk condition. We return `None` here, and conduct an rapid
        // change of acceleration (a discontinuity), at which point we can then find a constant-jerk
        // result after the change has been applied.
        // This would occur if, for example, α_0 and/or θ_0 is high,
        // and/or ω_0 is low.

        let inner = 6. * ω_dot_0 * (θ_tgt - θ_0) + (ω_tgt + 2. * ω_0).powi(2);

        if inner < 0. {
            None
        } else {
            let t_a = -(inner.sqrt() + 2. * ω_0 + ω_tgt) / ω_dot_0;
            let t_b = (inner.sqrt() - 2. * ω_0 - ω_tgt) / ω_dot_0;

            // Choose the lower of these solutions if both are positive; otherwise
            // choose the positive one, should it exist.
            if t_a > 0. && t_b > 0. {
                if t_a < t_b {
                    Some(t_a)
                } else {
                    Some(t_b)
                }
            } else if t_a > 0. {
                Some(t_a)
            } else if t_b > 0. {
                Some(t_b)
            } else {
                None
            }
        }
    }
}

fn find_ctrl_setting(
    θ_0: f32,
    ω_0: f32,
    ω_dot_0: f32,
    // The change in angle desired, along a given axis.
    θ_tgt: f32,
    // The change in angular velocity desired, along a given axis. We can find this by subtracting
    // the current angular velocity from the commanded rate.
    ω_tgt: f32,
    // ctrl_cmd_prev: f32,
    coeffs: &CtrlCoeffs,
    drag_coeff: f32,
    accel_map: &AccelMap,
    // filters: &mut FlightCtrlFilters,
    dt: f32,
) -> f32 {
    // todo: Take time to spin up/down into account?

    // let dθ = θ_tgt - θ_0;

    let time_to_correct = find_time_to_correct(θ_0, ω_0, ω_dot_0, θ_tgt, ω_tgt);

    static mut i: u32 = 0;
    unsafe { i += 1 };

    let mut α_target = match time_to_correct {
        Some(ttc) => {
            // If it would take too longer to perform the correction, calculate a new
            // angular acceleration that fits the criteria.
            if ttc > coeffs.max_ttc_per_dθ * (θ_tgt - θ_0).abs() {
                if unsafe { i } % 4_000 == 0 {
                    println!("unable to correct in time");
                }

                α_from_ttc(θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ)
                // If the time to correction is sufficiently small, apply a constant jerk, which
                // should be roughly the previous jerk if part way through a maneuver.
            } else {
                if unsafe { i } % 4_000 == 0 {
                    println!("Can correct in time");
                }

                // Calculate the (~constant for a given correction) change in angular acceleration.
                // let j = 6. * (2. * θ_0 + ttc * ω_0) / ttc.powi(3);
                let j = 6. / ttc.powi(3) * (2. * θ_0 + ttc * ω_0 - 2. * θ_tgt + ttc * ω_tgt);

                // This is the actual target acceleration, determined by the questions above:
                ω_dot_0 + j * dt
            }
        }
        None => {
            // if unsafe { i } % 7_000 == 0 {
            //     println!(
            //         "theta_0: {}, omega_0: {}, theta_tgt: {}, omega_tgt: {}, coeff: {}",
            //         θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ
            //     );
            // }

            α_from_ttc(θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ)
        }
    };

    // The target acceleration needs to include both the correction, and drag compensation.
    let drag_accel = -drag_coeff * ω_0;
    α_target += drag_accel;

    // Calculate how, most recently, the control command is affecting angular accel.
    // A higher constant means a given command has a higher affect on angular accel.
    // todo: Track and/or lowpass effectiveness over recent history, at diff params.
    // todo: Once you have bidir dshot, use RPM instead of power.
    // todo: DO we sill want this?
    // todo: You should probably track ctrl eff in the main loop; not here.
    // let ctrl_effectiveness = α_meas / ctrl_cmd_prev;

    // Apply a lowpass filter to our effectiveness, to reduce noise and fluctuations.
    // let ctrl_effectiveness = filters.apply(ctrl_effectiveness);

    // This distills to: (dω / time_to_correction) / (α / ctrl_cmd_prev) =
    // (dω / time_to_correction) x (ctrl_cmd_prev / α) =
    // (dω x ctrl_cmd_prev) / (time_to_correction x α) =
    //
    // (dθ * coeffs.p_ω - ω x ctrl_cmd_prev) /
    // ((coeffs.time_to_correction_p_ω * dω.abs() + coeffs.time_to_correction_p_θ * dθ.abs()) x α)

    // Units: rad x cmd / (s * rad/s) = rad x cmd / rad = cmd
    // `cmd` is the unit we use for ctrl inputs. Not sure what (if any?) units it has.
    // α_target /= ctrl_effectiveness;

    // static mut i: u32 = 0;
    // unsafe { i += 1 };
    // if unsafe { i } % 6_000 == 0 {
    //     println!("Tgt accel: {}", α_target);
    // }

    if unsafe { i } % 4_000 == 0 {
        println!("TTC: {}, Pitch acc tgt: {}", time_to_correct, α_target);
    }

    accel_map.interpolate(α_target)
}

#[cfg(feature = "quad")]
/// Calculate target rotor RPM or control-surface positions from current and target attitudes,
/// and current and target angular velocities.
/// This is our entry point
/// for control application:
/// The DT passed is the IMU update rate, since we update params_prev each IMU update.
pub fn ctrl_mix_from_att(
    target_attitude: Quaternion,
    // todo: Which do you want: A vector torque, or a rotation since last with DT?
    // todo: Probably torque.
    // target_torque: &Torque,
    target_ω: &(f32, f32, f32), // (pitch, roll, yaw)
    // target_rotation_since_prev: Quaternion,
    // current_attitude: Quaternion,
    throttle: f32,
    front_left_dir: RotationDir,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    params_prev: &Params,
    coeffs: &CtrlCoeffs,
    drag_coeffs: &DragCoeffs,
    accel_maps: &AccelMaps,
    filters: &mut FlightCtrlFilters,
    dt: f32, // seconds
) -> CtrlMix {
    // todo: Comfirming rotations work as expected. They appear to, although this axis-angle
    // todo thing is all wrong! (Rotation along the RIGHT vec is showing it's a roll, vice pitch...
    // let target_attitude = Quaternion::from_axis_angle(RIGHT, 0.2);

    // let att_axes = params.attitude.to_axes();
    // let target_axes = target_attitude.to_axes();

    // todo: Unused??
    // This is the rotation we need to create to arrive at the target attitude from the current one.
    let rotation_cmd = target_attitude * params.attitude.inverse();
    let rot_cmd_axes = rotation_cmd.to_axes();

    let (pitch, roll, yaw) = {
        // todo: Sloppy
        static mut integral_x: f32 = 0.;
        static mut integral_y: f32 = 0.;
        static mut integral_z: f32 = 0.;

        let p_term = 5_000.;
        let i_term = 400.;

        // todo: DO we want to multiply by dt here?
        // let rot_to_apply = Quaternion::new_identity().slerp(rotation_cmd, p_term * dt);

        // let (rot_x, rot_y, rot_z) = rot_to_apply.to_axes();

        // PID here; get this working, then refine as required, or get fancier
        let error_x = rot_cmd_axes.0;
        let error_y = rot_cmd_axes.1;
        let error_z = rot_cmd_axes.2;

        unsafe {
            integral_x += error_x;
            integral_y += error_y;
            integral_z += error_z;
        }

        let pitch = p_term * error_x + i_term * unsafe { integral_x };
        let roll = p_term * error_y + i_term * unsafe { integral_y };
        let yaw = p_term * error_z + i_term * unsafe { integral_z };

        (pitch, roll, yaw)
    };

    let ttc_target_per_rad = 0.2;

    // I don't think it makes sense to apply a TTC to the quat directly, since the multiple dimensions
    // involved means it's very likely on an intercept course.
    // radians / (rad/s) = s
    let time_to_correct_x = rot_cmd_axes.0 / (params.v_pitch - target_ω.0);
    let time_to_correct_y = rot_cmd_axes.1 / (params.v_roll - target_ω.1);
    let time_to_correct_z = rot_cmd_axes.2 / (params.v_yaw - target_ω.2);

    // todo: QC where you're getting these target angular accels.
    let (target_ω_pitch, target_ω_roll, target_ω_yaw) = target_ω;
    let dω_pitch = target_ω_pitch - params.v_pitch;
    let dω_roll = target_ω_roll - params.v_roll;
    let dω_yaw = target_ω_yaw - params.v_yaw;

    // todo: It's possible thetas should be 0, and theta target should be the rot cmd.

    //

    // todo: Testing alternative, more intuitive approach

    // let amt_dv = 1.;

    // If there is a positive TTC, we should apply a weaker correction.
    // If there is a negative TTC, we apply a stronger one.
    let corr_factor = 0.3;
    // todo: You probably want an additive factor, not multiplicative.
    // let v_correction_x = map_linear(time_to_correct_x, (-5., 5.), (1. + corr_factor, 1. - corr_factor));

    // let d_ttc_x = (time_to_correct_x - ttc_target_per_rad) * ;
    // let d_ttc_y = (time_to_correct_y - ttc_target_per_rad);
    // let d_ttc_z = (time_to_correct_z - ttc_target_per_rad);

    // let = rot_cmd_axes.0

    // Examples, reasoning this out:
    // if there is a high TTC, We need to scale
    // let v_correction_x = p_term *

    // let pitch = find_ctrl_setting(
    //     att_axes.0,
    //     params.v_pitch,
    //     params.a_pitch,
    //     target_axes.0,
    //     *target_ω_pitch,
    //     coeffs,
    //     drag_coeffs.pitch,
    //     &accel_maps.map_pitch,
    //     dt,
    // );

    // let roll = find_ctrl_setting(
    //       att_axes.1,
    //     params.v_roll,
    //     params.a_roll,
    //     target_axes.1,
    //     *target_ω_roll,
    //     coeffs,
    //     drag_coeffs.roll,
    //     &accel_maps.map_roll,
    //     dt,
    // );
    //
    // let yaw = find_ctrl_setting(
    //       att_axes.2,
    //     params.v_yaw,
    //     params.a_yaw,
    //   target_axes.2,
    //     *target_ω_yaw,
    //     coeffs,
    //     drag_coeffs.yaw,
    //     &accel_maps.map_yaw,
    //     dt,
    // );

    let mut result = CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    };

    static mut i: u32 = 0;
    unsafe { i += 1 };
    if unsafe { i } % 4_000 == 0 {
        println!("\n***Attitude***");

        ahrs::print_quat(params.attitude, "Current att");
        ahrs::print_quat(target_attitude, "Target att");
        ahrs::print_quat(rotation_cmd, "Rot cmd");

        println!("\n***Ang Velocity***");
        println!(
            "Current: v_p{} v_r{} v_y{}",
            params.v_pitch, params.v_roll, params.v_yaw
        );
        println!(
            "Target ω p{}, r{}, y{}",
            target_ω_pitch, target_ω_roll, target_ω_yaw
        );

        println!("Required dω: p{}, r{}, y{}", dω_pitch, dω_roll, dω_yaw);

        println!(
            "TTC x{}, y{}, z{}",
            time_to_correct_x, time_to_correct_y, time_to_correct_z,
        );

        // println!(
        //     "d TTC x{}, y{}, z{}",
        //    d_ttc_x, d_ttc_y, d_ttc_z,
        // );

        // println!(
        //     "Target torque: x{}, y{}, z{} v{} ",
        //     target_torque.axis.x,
        //     target_torque.axis.y,
        //     target_torque.axis.z,
        //     target_torque.angular_velocity
        // );

        println!("\n***Command generated:***");

        // ahrs::print_quat(rot_to_apply, "Rot to apply");
        println!(
            "Mix. p{} r{} y{} t{}\n\n\n",
            result.pitch, result.roll, result.yaw, result.throttle
        );
    }

    result.clamp();
    result
}

#[cfg(feature = "fixed-wing")]
/// Similar to the above fn on quads. Note that we do not handle yaw command using this. Yaw
/// is treated as coupled to pitch and roll, with yaw controls used to counter adverse-yaw.
/// Yaw is to maintain coordinated flight, or deviate from it.
pub fn ctrl_mix_from_att(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    throttle: f32,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    params_prev: &Params,
    coeffs: &CtrlCoeffs,
    drag_coeffs: &DragCoeffs,
    accel_maps: &AccelMaps,
    filters: &mut FlightCtrlFilters,
    dt: f32, // seconds
) -> CtrlMix {
    // todo: Modulate based on airspeed.

    let rotation_cmd = target_attitude * current_attitude.inverse();
    let rot_euler = rotation_cmd.to_euler();

    let pitch = find_ctrl_setting(
        rot_euler.pitch,
        params.v_pitch,
        params.a_pitch,
        coeffs,
        drag_coeffs.pitch,
        &accel_maps.map_pitch,
    );
    let roll = find_ctrl_setting(
        rot_euler.roll,
        params.v_roll,
        params.a_roll,
        coeffs,
        drag_coeffs.roll,
        &accel_maps.map_roll,
    );

    let yaw = 0.; // todo?

    CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    }
}

/// Modify our attitude commanded from rate-based user inputs. `ctrl_crates` are in radians/s, and `dt` is in s.
pub fn modify_att_target(
    orientation: Quaternion,
    pitch: f32,
    roll: f32,
    yaw: f32,
    dt: f32,
) -> Quaternion {
    // todo: Error handling on this?

    // Rotate our basis vecs using the orientation, such that control inputs are relative to the
    // aircraft's attitude.
    let right_ac = orientation.rotate_vec(RIGHT);
    let fwd_ac = orientation.rotate_vec(FORWARD);
    let up_ac = orientation.rotate_vec(UP);

    let rotation_pitch = Quaternion::from_axis_angle(right_ac, pitch * dt);
    let rotation_roll = Quaternion::from_axis_angle(fwd_ac, roll * dt);
    let rotation_yaw = Quaternion::from_axis_angle(up_ac, yaw * dt);

    // todo: This EPS is to prevent f32 drift. We probably need a better way.
    const EPS: f32 = 0.00005;
    if (pitch * dt).abs() < EPS && (roll * dt).abs() < EPS && (yaw * dt).abs() < EPS {
        return orientation;
    }

    // todo: Order?
    // rotation_yaw * rotation_roll * rotation_pitch * orientation
    let rotation = rotation_yaw * rotation_roll * rotation_pitch;

    static mut i: u32 = 0;
    unsafe { i += 1 };
    if unsafe { i } % 3_000 == 0 {
        // println!("p: {} r: {} y: {}", pitch * dt, roll * dt, yaw * dt);

        let re = rotation.to_euler();
        // println!("Rot. p: {} r: {} y: {}. norm: {}", re.pitch, re.roll, re.yaw, rotation.magnitude());
        let o = orientation.to_euler();
        // println!("Or. p: {} r: {} y: {}. Fwd vec: ({} {} {})", o.pitch, o.roll, o.yaw, fwd_ac.x, fwd_ac.y, fwd_ac.z);
    }

    // todo: f32 precision issues?

    // Should already be normalized, but do this to avoid drift.
    (rotation * orientation).to_normalized()
    // orientation
}

/// Calculate an attitude based on control input, in `attitude mode`.
pub fn _att_from_ctrls(ch_data: &ChannelData) -> Quaternion {
    // todo: How do you deal with heading? That's a potential disadvantage of using a quaternion:
    // todo we can calculate pitch and roll, but not yaw.
    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, ch_data.pitch);
    let rotation_roll = Quaternion::from_axis_angle(FORWARD, ch_data.roll);

    rotation_roll * rotation_pitch
}

//tpdo: move this

/// Based on control channel data, update attitude commanded, and attitude-rate
/// commanded
pub fn update_att_commanded(
    ch_data: &ChannelData,
    input_map: &InputMap,
    att_commanded_prev: Quaternion,
    has_taken_off: bool,
    takeoff_attitude: Quaternion,
) -> (Quaternion, (f32, f32, f32)) {
    // let rates_commanded = RatesCommanded {
    //     pitch: Some(cfg.input_map.calc_pitch_rate(ch_data.pitch)),
    //     roll: Some(cfg.input_map.calc_roll_rate(ch_data.roll)),
    //     yaw: Some(cfg.input_map.calc_yaw_rate(ch_data.yaw)),
    // };

    // Negative on pitch, since we want pulling down (back) on the stick to raise
    // the nose.
    let pitch_rate_cmd = input_map.calc_pitch_rate(-ch_data.pitch);
    let roll_rate_cmd = input_map.calc_roll_rate(ch_data.roll);
    let yaw_rate_cmd = input_map.calc_yaw_rate(ch_data.yaw);

    // todo: Temp for testing flight control logic!! Take away this hard set.
    // state_volatile.has_taken_off = true;

    // todo: Should attitude commanded regress to current attitude if it hasn't changed??

    // If we haven't taken off, apply the attitude lock.

    // Don't update attitude commanded, or the change in attitude commanded
    // each loop. We don't get control commands that rapidly, and more importantly,
    // doing it every loop leads to numerical precision issues due to how small
    // the changes are.
    if has_taken_off {
        let att_commanded_current = modify_att_target(
            att_commanded_prev,
            pitch_rate_cmd,
            roll_rate_cmd,
            yaw_rate_cmd,
            DT_FLIGHT_CTRLS * ATT_CMD_UPDATE_RATIO as f32,
        );
        // todo: Instead of skipping ones not on the update ratio, you could store
        // todo a buffer of attitudes, and look that far back.
        // if i % ATT_CMD_UPDATE_RATIO == 0 {
        // state_volatile.attitude_commanded.quat_dt = Torque::from_attitudes(
        //     att_cmd_prev,
        //     state_volatile.attitude_commanded.quat,
        //     DT_FLIGHT_CTRLS * ATT_CMD_UPDATE_RATIO as f32,
        // );

        (
            att_commanded_current,
            ang_v_from_attitudes(
                att_commanded_prev,
                att_commanded_current,
                DT_FLIGHT_CTRLS * ATT_CMD_UPDATE_RATIO as f32,
            ),
        )
    } else {
        (takeoff_attitude, (0., 0., 0.))
    }
}
