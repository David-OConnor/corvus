//! This module contains code for control logic. It commands constant-jerk corrections to
//! attitude and rate-of-change of attitude.
//!
//! See the comments here, the accompanying Python script in this project, and the associated
//! One note file for details on how we calculate this.

use crate::{
    control_interface::ChannelData,
    main_loop::{ATT_CMD_UPDATE_RATIO, DT_FLIGHT_CTRLS},
    util::{self, map_linear},
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
    dt: f32,              // seconds
    pry: (f32, f32, f32), // todo temp to TS
) -> CtrlMix {
    // todo: temp to make the throttle more manageable while flying indoors.
    let throttle = throttle / 2.;

    // This is the rotation we need to create to arrive at the target attitude from the current one.
    let rotation_cmd = target_attitude * params.attitude.inverse();
    let rot_cmd_axes = rotation_cmd.to_axes();

    let (pitch, roll, yaw) = {
        // todo: Sloppy
        static mut integral_x: f32 = 0.;
        static mut integral_y: f32 = 0.;
        static mut integral_z: f32 = 0.;

        let p_term = 0.08;
        let i_term = 0.00001;

        // todo: DO we want to multiply by dt here?
        // let rot_to_apply = Quaternion::new_identity().slerp(rotation_cmd, p_term * dt);

        // let (rot_x, rot_y, rot_z) = rot_to_apply.to_axes();

        // PID here; get this working, then refine as required, or get fancier
        let error_x = rot_cmd_axes.0;
        let error_y = rot_cmd_axes.1;
        let error_z = rot_cmd_axes.2;

        // Attempting rate
        // let error_x = pry.0 - params.v_pitch;
        // let error_y = pry.1 - params.v_roll;
        // let error_z = pry.2 - params.v_yaw;

        unsafe {
            integral_x += error_x;
            integral_y += error_y;
            integral_z += error_z;
        }

        // The I-term builds up if corrections are unable to expeditiously converge.
        // An example of when this can happen is when the aircraft is on the ground.
        // todo: Use `is_airborne` etc, vice idle throttle.
        if throttle < 0.1 {
            unsafe {
                integral_x = 0.;
                integral_y = 0.;
                integral_z = 0.;
            }
        }

        let pitch = p_term * error_x + i_term * unsafe { integral_x };
        let roll = p_term * error_y + i_term * unsafe { integral_y };
        let yaw = p_term * error_z + i_term * unsafe { integral_z };

        (pitch, roll, yaw)
    };

    let mut result = CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    };

    result.clamp();

    static mut i: u32 = 0;
    unsafe { i += 1 };
    // if unsafe { i } % 4_000 == 0 {
    if false {
        // println!("\n***Attitude***");

        ahrs::print_quat(params.attitude, "Current att");
        ahrs::print_quat(target_attitude, "Target att");
        ahrs::print_quat(rotation_cmd, "Rot cmd");

        println!("Throttle: {:?}", throttle);

        // println!("\n***Ang Velocity***");
        // println!(
        //     "Current: v_p{} v_r{} v_y{}",
        //     params.v_pitch, params.v_roll, params.v_yaw
        // );
        // println!(
        //     "Target ω p{}, r{}, y{}",
        //     target_ω_pitch, target_ω_roll, target_ω_yaw
        // );
        //
        // println!("Required dω: p{}, r{}, y{}", dω_pitch, dω_roll, dω_yaw);
        //
        // println!(
        //     "TTC x{}, y{}, z{}",
        //     time_to_correct_x, time_to_correct_y, time_to_correct_z,
        // );

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

        // println!("\n***Command generated:***");

        // ahrs::print_quat(rot_to_apply, "Rot to apply");
        println!(
            "Mix. p{} r{} y{} t{}\n",
            result.pitch, result.roll, result.yaw, result.throttle
        );
    }

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

    let rotation_cmd = target_attitude / current_attitude;

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
    // todo: This DEADZONE is to prevent f32(?) drift. We probably need a better way.
    // todo: This works for now though, at least when the stick is idle.
    const DEADZONE: f32 = 0.001;
    if (pitch * dt).abs() < DEADZONE && (roll * dt).abs() < DEADZONE && (yaw * dt).abs() < DEADZONE
    {
        return orientation;
    }
    // todo: Error handling on this?

    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, -pitch * dt);
    let rotation_roll = Quaternion::from_axis_angle(FORWARD, -roll * dt);
    let rotation_yaw = Quaternion::from_axis_angle(UP, yaw * dt);

    // todo: Order?
    let rotation = (rotation_yaw * rotation_roll * rotation_pitch).to_normalized();

    // todo: f32 precision issues, fixed by the att update ratio?

    // Should already be normalized, but do this to avoid drift.
    (rotation * orientation).to_normalized()
}

/// Calculate an attitude based on control input, in `attitude mode`.
pub fn _att_from_ctrls(ch_data: &ChannelData) -> Quaternion {
    // todo: How do you deal with heading? That's a potential disadvantage of using a quaternion:
    // todo we can calculate pitch and roll, but not yaw.
    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, ch_data.pitch);
    let rotation_roll = Quaternion::from_axis_angle(FORWARD, ch_data.roll);

    rotation_roll * rotation_pitch
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

/// Based on control channel data, update attitude commanded, and attitude-rate
/// commanded
pub fn update_att_commanded(
    ch_data: &ChannelData,
    input_map: &InputMap,
    att_commanded_prev: Quaternion,
    has_taken_off: bool,
    takeoff_attitude: Quaternion,
) -> (Quaternion, (f32, f32, f32)) {
    if !has_taken_off {
        return (takeoff_attitude, (0., 0., 0.));
    }

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

    // todo: Should attitude commanded regress to current attitude if it hasn't changed??

    // If we haven't taken off, apply the attitude lock.

    // Don't update attitude commanded, or the change in attitude commanded
    // each loop. We don't get control commands that rapidly, and more importantly,
    // doing it every loop leads to numerical precision issues due to how small
    // the changes are.

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
}
