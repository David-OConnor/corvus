//! This module contains code for control logic. It commands constant-jerk corrections to
//! attitude and rate-of-change of attitude.
//!
//! See the comments here, the accompanying Python script in this project, and the associated
//! One note file for details on how we calculate this.

use crate::{
    control_interface::ChannelData,
    util::{self, map_linear},
};

use super::{
    common::CtrlMix,
    ctrl_effect_est::{AccelMap, AccelMaps},
    filters::FlightCtrlFilters,
    motor_servo::{MotorServoState, RotationDir},
};

use ahrs::Params;

use lin_alg2::f32::{EulerAngle, Quaternion, Vec3};

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

// todo: Make sure this jives with the axis you're using elsewhere;
// todo: Especially y and z! I'm attempting to use something similar to how the ICM42688 defines
// todo axis. Maybe use a standard coord system? Check the signs on UP and FWD in particular.
const RIGHT: Vec3 = Vec3 {
    x: 1.,
    y: 0.,
    z: 0.,
};
const UP: Vec3 = Vec3 {
    x: 0.,
    y: 0.,
    z: 1.,
};
const FWD: Vec3 = Vec3 {
    x: 0.,
    y: 1.,
    z: 0.,
};

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
        let vec = UP * yaw + RIGHT * pitch + FWD * roll;
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

/// Construct 3 euler-representation angular velocities from 2 quaternions, and the time the rotation takes.
/// This assumes less than a full rotation between updates.
pub fn ang_v_from_attitudes(
    att_prev: Quaternion,
    att_this: Quaternion,
    dt: f32,
) -> (f32, f32, f32) {
    let rotation = att_this * att_prev.inverse();

    let euler = rotation.to_euler();

    (euler.pitch * dt, euler.roll * dt, euler.yaw * dt)
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
    // use the discontinous logic. In rad/s
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
                α_from_ttc(θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ)
                // If the time to correction is sufficiently small, apply a constant jerk, which
                // should be roughly the previous jerk if part way through a maneuver.
            } else {
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

    // This is the rotation we need to create to arrive at the target attitude from the current one.
    // Note: This isn't the recipe I imagined, but the one that seems to work due to trial+error.
    // it's possible you have flipped quaternion multiplication in your impl, or something.
    let rotation_cmd = params.attitude_quat * target_attitude.inverse();
    // Split the rotation into 3 euler angles. We do this due to our controls and sensors acting
    // along individual axes.
    let mut rot_euler = rotation_cmd.to_euler();
    // again, strange results, so compensationg
    rot_euler.pitch *= -1.;
    rot_euler.roll *= -1.;
    rot_euler.yaw *= -1.;

    // todo: It looks like our new API uses the target eulers directly...
    let target_euler = target_attitude.to_euler();

    // todo: Is this logic correct?

    // We arrive at a target attitude, and target angular velocity.

    // todo: Why the assymetry; why is target angular velocity represented as a torque,
    // todo but params are as a euler angle? Possibly because params comes directly from gyros,
    // todo but target comes from a change in quaternions.
    // todo: YOu might skip torque and convert the quaternion diff direclty to euler angles.
    // let target_ω_pitch = target_torque.axis.dot(RIGHT) * target_torque.angular_velocity;
    // let target_ω_roll = target_torque.axis.dot(FWD) * target_torque.angular_velocity;
    // let target_ω_yaw = target_torque.axis.dot(UP) * target_torque.angular_velocity;

    let (target_ω_pitch, target_ω_roll, target_ω_yaw) = target_ω;
    let dω_pitch = target_ω_pitch - params.v_pitch;
    let dω_roll = target_ω_roll - params.v_roll;
    let dω_yaw = target_ω_yaw - params.v_yaw;

    // let dω_pitch = (target_torque.axis.x) * target_torque.angular_velocity - params.v_pitch;
    // let dω_roll = (target_torque.axis.z) * target_torque.angular_velocity - params.v_roll;
    // let dω_yaw = (target_torque.axis.y) * target_torque.angular_velocity - params.v_yaw;

    let pitch = find_ctrl_setting(
        params.s_pitch,
        params.v_pitch,
        params.a_pitch,
        target_euler.pitch,
        // rot_euler.pitch,
        // dω_pitch,
        *target_ω_pitch,
        coeffs,
        drag_coeffs.pitch,
        &accel_maps.map_pitch,
        dt,
    );

    let roll = find_ctrl_setting(
        params.s_roll,
        params.v_roll,
        params.a_roll,
        target_euler.roll,
        // rot_euler.roll,
        // dω_roll,
        *target_ω_roll,
        coeffs,
        drag_coeffs.roll,
        &accel_maps.map_roll,
        dt,
    );

    let yaw = find_ctrl_setting(
        params.s_yaw_heading,
        params.v_yaw,
        params.a_yaw,
        // rot_euler.yaw,
        target_euler.yaw,
        *target_ω_yaw,
        coeffs,
        drag_coeffs.yaw,
        &accel_maps.map_yaw,
        dt,
    );

    let mut result = CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    };

    static mut i: u32 = 0;
    unsafe { i += 1 };
    if unsafe { i } % 4_000 == 0 {
        // let c = params.attitude_quat.to_euler();

        println!("\n***Attitude***");
        // println!("Current: p{} r{} y{}", c.pitch, c.roll, c.yaw);
        let euler = params.attitude.to_euler();
        println!("Current p{} r{} y{}", euler.pitch, euler.roll, euler.yaw,);
        println!(
            "Target: p{} r{} y{}",
            target_euler.pitch, target_euler.roll, target_euler.yaw
        );
        println!(
            "Required rot: p{} r{} y{}",
            rot_euler.pitch, rot_euler.roll, rot_euler.yaw
        );

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

        // println!(
        //     "Target torque: x{}, y{}, z{} v{} ",
        //     target_torque.axis.x,
        //     target_torque.axis.y,
        //     target_torque.axis.z,
        //     target_torque.angular_velocity
        // );

        println!("\n***Command generated:***");
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
    let fwd_ac = orientation.rotate_vec(FWD);
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
    let rotation_roll = Quaternion::from_axis_angle(FWD, ch_data.roll);

    rotation_roll * rotation_pitch
}
