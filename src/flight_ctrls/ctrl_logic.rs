//! This module contains code for control logic. It commands constant-jerk corrections to
//! attitude and rate-of-change of attitude.
//!
//! See the comments here, the accompanying Python script in this project, and the associated
//! One note file for details on how we calculate this.

use ahrs::{Params, FORWARD, RIGHT, UP};
use cfg_if::cfg_if;
use defmt::println;
use lin_alg2::f32::Quaternion;
use num_traits::float::Float; // For sqrt.

use super::{
    common::{CtrlMix, InputMap},
    ctrl_effect_est::AccelMaps,
    filters::FlightCtrlFilters,
};
use crate::{
    controller_interface::ChannelData,
    flight_ctrls::{
        motor_servo::RotationDir,
        pid::{PidCoeffs, PidStateRate},
    },
    main_loop::{ATT_CMD_UPDATE_RATIO, DT_FLIGHT_CTRLS, FLIGHT_CTRL_IMU_RATIO},
    util::{self, clamp, map_linear, IirInstWrapper},
};

// This should be on the order of the error term (Roughly radians)

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

/// Calculate an angular velocity command to perform a given attitude correction on a given axis.
/// Attempts to command a constant angular acceleration, using kinematics. Assumes we have an accurate
/// way of commanding angular velocity, as that's downstream of this.
fn att_correction_to_ω(dθ: f32, time_to_correct: f32, ω_target: f32) -> f32 {
    // You can store 2./TTC instead of calculating it each time; or at least
    // instead of calculating it for each axis.
    2. / time_to_correct * dθ - ω_target
}

#[cfg(feature = "quad")]
/// Calculate target rotor RPM or control-surface positions from current and target attitudes,
/// and current and target angular velocities.
/// This is our entry point
/// for control application:
/// The DT passed is the IMU update rate, since we update params_prev each IMU update.
pub fn ctrl_mix_from_att(
    target_attitude: Quaternion,
    target_ω: &(f32, f32, f32), // (pitch, roll, yaw)
    throttle: f32,
    front_left_dir: RotationDir,
    // todo: Params is just for attitude. Maybe just pass those?
    params: &Params,
    params_prev: &Params,
    coeffs: &CtrlCoeffs,
    drag_coeffs: &DragCoeffs,
    accel_maps: &AccelMaps,
    filters: &mut FlightCtrlFilters,
    dt: f32, // seconds
    pid_coeffs: &PidCoeffs,
    pid_state: &mut PidStateRate,
    has_taken_off: bool,
) -> CtrlMix {
    // This is the rotation we need to create to arrive at the target attitude from the current one.
    let rot_cmd_axes = (target_attitude / params.attitude).to_axes();

    // These are in rad
    let error_att_x = rot_cmd_axes.0;
    let error_att_y = rot_cmd_axes.1;
    let error_att_z = rot_cmd_axes.2;

    let mut pitch_rate_cmd = att_correction_to_ω(error_att_x, pid_coeffs.att_ttc, target_ω.0);
    let mut roll_rate_cmd = att_correction_to_ω(error_att_y, pid_coeffs.att_ttc, target_ω.1);
    let mut yaw_rate_cmd = att_correction_to_ω(error_att_z, pid_coeffs.att_ttc, target_ω.2);

    // This cap mainly applies to non-continuous attitude commands.
    const MAX_ATT_CORRECTION_ω: f32 = 12.;

    clamp(
        &mut pitch_rate_cmd,
        (-MAX_ATT_CORRECTION_ω, MAX_ATT_CORRECTION_ω),
    );
    clamp(
        &mut roll_rate_cmd,
        (-MAX_ATT_CORRECTION_ω, MAX_ATT_CORRECTION_ω),
    );
    clamp(
        &mut yaw_rate_cmd,
        (-MAX_ATT_CORRECTION_ω, MAX_ATT_CORRECTION_ω),
    );

    // The I-term builds up if corrections are unable to expeditiously converge.
    // An example of when this can happen is when the aircraft is on the ground.
    // todo: Use `is_airborne` etc, vice idle throttle?
    if has_taken_off {
        pid_state.reset_i();
    }

    let pitch = pid_state.pitch.apply(
        pitch_rate_cmd,
        params.v_pitch,
        pid_coeffs,
        &mut filters.d_term_x,
        dt,
    );
    let roll = pid_state.pitch.apply(
        roll_rate_cmd,
        params.v_roll,
        pid_coeffs,
        &mut filters.d_term_y,
        dt,
    );
    let yaw = pid_state.pitch.apply(
        yaw_rate_cmd,
        params.v_yaw,
        pid_coeffs,
        &mut filters.d_term_z,
        dt,
    );

    let mut result = CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    };

    result.clamp();

    static mut i: u32 = 0;
    unsafe { i += 1 };
    if unsafe { i } % 2_000 == 0 {
        println!(
            "rate cmds P{} R{} Y{}",
            pitch_rate_cmd, roll_rate_cmd, yaw_rate_cmd
        );
        // println!("Err rate P{} R{} Y{}", error_att_rate_x, error_att_rate_y, error_att_rate_y);
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

    // let rotation_cmd = target_attitude / current_attitude;

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

/// Modify our attitude commanded from rate-based user inputs. ctrl_crates are in radians/s, and `dt` is in s.
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

    // f32 precision issues, fixed by the att update ratio?

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
    let rotation = att_this / att_prev;

    let (x_comp, y_comp, z_comp) = rotation.to_axes();

    (x_comp * dt, y_comp * dt, z_comp * dt)
}

/// Used in Acro mode. Based on control channel data, update attitude commanded, and attitude-rate
/// commanded. Controls map to commanded angular velocity.
pub fn update_att_commanded_acro(
    ch_data: &ChannelData,
    input_map: &InputMap,
    att_commanded_prev: Quaternion,
    current_att: Quaternion,
    has_taken_off: bool,
    takeoff_attitude: Quaternion,
) -> (Quaternion, (f32, f32, f32)) {
    // If we haven't taken off, apply the attitude lock.
    if !has_taken_off {
        // Prevent the aircraft from snapping north on takeoff. `takeoff_att` is only relevant for
        // pitch and roll. This syncs the commadned attiude's yaw to current attitude's.
        let aircraft_hdg = current_att.to_axes().2;

        let heading_rotation = Quaternion::from_axis_angle(UP, -aircraft_hdg);

        let att = heading_rotation * takeoff_attitude;
        return (att, (0., 0., 0.));
    }

    let pitch_cmd = -ch_data.pitch;
    let roll_cmd = ch_data.roll;
    let yaw_cmd = ch_data.yaw;

    // Negative on pitch, since we want pulling down (back) on the stick to raise
    // the nose.
    let pitch_rate_cmd = input_map.calc_pitch_rate(pitch_cmd);
    let roll_rate_cmd = input_map.calc_roll_rate(roll_cmd);
    let yaw_rate_cmd = input_map.calc_yaw_rate(yaw_cmd);

    // Don't update attitude commanded, or the change in attitude commanded
    // each loop. We don't get control commands that rapidly, and more importantly,
    // doing it every loop leads to numerical precision issues due to how small
    // the changes are.

    let dt = DT_FLIGHT_CTRLS * ATT_CMD_UPDATE_RATIO as f32;

    let att_commanded_current = modify_att_target(
        att_commanded_prev,
        pitch_rate_cmd,
        roll_rate_cmd,
        yaw_rate_cmd,
        dt,
    );

    (
        att_commanded_current,
        ang_v_from_attitudes(att_commanded_prev, att_commanded_current, dt),
    )
}

/// Used in Attitude mode. Based on control channel data, update attitude commanded, and attitude-rate
/// commanded. Controls map to attitude directly.
pub fn update_att_commanded_att_mode(
    ch_data: &ChannelData,
    input_map: &InputMap,
    att_commanded_prev: Quaternion,
    current_att: Quaternion,
    has_taken_off: bool,
    takeoff_attitude: Quaternion,
) -> (Quaternion, (f32, f32, f32)) {
    // todo: DRY with acro fn.
    // If we haven't taken off, apply the attitude lock.
    if !has_taken_off {
        // Prevent the aircraft from snapping north on takeoff. `takeoff_att` is only relevant for
        // pitch and roll. This syncs the commadned attiude's yaw to current attitude's.
        let aircraft_hdg = current_att.to_axes().2;

        let heading_rotation = Quaternion::from_axis_angle(UP, -aircraft_hdg);

        let att = heading_rotation * takeoff_attitude;
        return (att, (0., 0., 0.));
    }

    let pitch_cmd = -ch_data.pitch;
    let roll_cmd = ch_data.roll;
    let yaw_cmd = ch_data.yaw;

    // Negative on pitch, since we want pulling down (back) on the stick to raise
    // the nose.
    let pitch_att_cmd = input_map.calc_pitch_angle(pitch_cmd);
    let roll_att_cmd = input_map.calc_roll_angle(roll_cmd);
    let yaw_rate_cmd = input_map.calc_yaw_rate(yaw_cmd);

    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, -pitch_att_cmd);
    let rotation_roll = Quaternion::from_axis_angle(FORWARD, -roll_att_cmd);

    let dt = DT_FLIGHT_CTRLS * ATT_CMD_UPDATE_RATIO as f32;
    let rotation_yaw = Quaternion::from_axis_angle(UP, yaw_rate_cmd * dt);

    // todo: Axis order, A/R. And, DRY from above.
    let att_commanded_current = (rotation_yaw * rotation_roll * rotation_pitch).to_normalized();

    (
        att_commanded_current,
        ang_v_from_attitudes(att_commanded_prev, att_commanded_current, dt),
    )
}

/// Used in Attitude and Loiter modes. Based on control channel data, update baro alt commanded, and
/// vv commanded..
pub fn update_alt_baro_commanded(
    ch_data_throttle: f32, // todo: Move A/R; this is only for manual controls. Keep for now.
    input_map: &InputMap,
    alt_commanded_prev: f32,
) -> (f32, f32) {
    let neutral_range = 0.2; // todo: Adjustable config setting.

    let vv_cmd = input_map.calc_vv(ch_data_throttle, neutral_range);

    let alt_commanded_current = alt_commanded_prev + vv_cmd * DT_FLIGHT_CTRLS;

    // todo: This thresh adds a bit of a pad. Consider how you want to handle this.
    if alt_commanded_current < -5. {
        // todo: Has taken off check?
        return (0., 0.);
    }

    (
        alt_commanded_current,
        (alt_commanded_current - alt_commanded_prev) / DT_FLIGHT_CTRLS,
    )
}
