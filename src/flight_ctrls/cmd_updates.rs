//! This module contains code that updates commanded attitude and altitude from
//! control inputs.

use ahrs::{Params, FORWARD, RIGHT, UP};
use lin_alg2::f32::Quaternion;

use super::common::{CtrlMix, InputMap};
use crate::main_loop::{ATT_CMD_UPDATE_RATIO, DT_FLIGHT_CTRLS, FLIGHT_CTRL_IMU_RATIO};

// todo: This DEADZONE is to prevent f32(?) drift. We probably need a better way.
// todo: This works for now though, at least when the stick is idle.
const ACRO_DEADZONE: f32 = 0.001;

/// Modify our attitude commanded from rate-based user inputs. ctrl_crates are in radians/s, and `dt` is in s.
fn modify_att_target(
    orientation: Quaternion,
    pitch: f32,
    roll: f32,
    yaw: f32,
    dt: f32,
) -> Quaternion {
    if (pitch * dt).abs() < ACRO_DEADZONE
        && (roll * dt).abs() < ACRO_DEADZONE
        && (yaw * dt).abs() < ACRO_DEADZONE
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

/// Construct a by-axis representation angular velocities from 2 quaternions, and the time the rotation takes.
/// This assumes less than a full rotation between updates.
fn ang_v_from_attitudes(att_prev: Quaternion, att_this: Quaternion, dt: f32) -> (f32, f32, f32) {
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
