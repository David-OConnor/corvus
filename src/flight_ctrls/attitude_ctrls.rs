//! This module contains code for attitude-based controls. This includes sticks mapping
//! to attitude, and an internal attitude model with rate-like controls, where attitude is the target.

use core::cmp;

use crate::control_interface::ChannelData;

use super::common::{Params, RatesCommanded};

use lin_alg2::f32::{Quaternion, Vec3};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "quad")] {
        use super::{MotorPower, RotationDir};
    } else {
        use super::ControlPositions;
    }

}

// Used as a crude PID, while we are experimenting.
const P_COEFF: f32 = 1.;

const RIGHT: Vec3 = Vec3 {
    x: 1.,
    y: 0.,
    z: 0.,
};
const UP: Vec3 = Vec3 {
    x: 0.,
    y: 1.,
    z: 0.,
};
const FWD: Vec3 = Vec3 {
    x: 0.,
    y: 0.,
    z: 1.,
};

#[cfg(feature = "quad")]
pub fn motor_power_from_atts(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    throttle: f32,
    front_left_dir: RotationDir,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    current_power: &MotorPower,
) -> MotorPower {
    // todo: This fn and approach is a WIP!!

    // This is the rotation we need to cause to arrive at the target attitude.
    let rotation_cmd = target_attitude * current_attitude.inverse();

    // todo: Common fn for shared code between here and quad.

    // todo: These params should be in `UserCfg`.
    let p_pitch = 1.;
    let p_roll = 1.;
    let p_yaw = 1.;

    let max_rate_pitch = 20.; // radians/s
    let max_rate_roll = 20.; // radians/s
    let max_rate_yaw = 20.; // radians/s

    // We target the `max_rate`s defined above when the distance to travel on a given axes is
    // greater than or equal to these distances.
    let max_rate_dist_pitch = 1.; // radians
    let max_rate_dist_roll = 1.; // radians
    let max_rate_dist_yaw = 1.; // radians

    // todo: Fn to reduce dry between P, R, Y?

    // Split the rotation into 3 euler angles. We do this due to our controls acting primarily
    // along individual axes.
    let (rot_pitch, rot_yaw, rot_roll) = rotation_cmd.to_euler();

    // Compare the current (measured) angular velocities to what we need to apply this rotation.
    let mut target_rate_pitch = if rot_pitch > max_rate_dist_pitch {
        max_rate_dist_pitch * max_rate_pitch
    } else {
        // Clamp
        max_rate_pitch
    };

    let mut target_rate_roll = if rot_roll > max_rate_dist_roll {
        max_rate_dist_roll * max_rate_roll
    } else {
        // Clamp
        max_rate_roll
    };

    let mut target_rate_yaw = if rot_yaw > max_rate_dist_yaw {
        max_rate_dist_yaw * max_rate_yaw
    } else {
        // Clamp
        max_rate_yaw
    };

    let target_rate_pitch_change = params.v_pitch - target_rate_pitch;
    let target_rate_roll_change = params.v_roll - target_rate_roll;
    let target_rate_yaw_change = params.v_yaw - target_rate_yaw;

    let pitch_cmd = target_rate_pitch_change * p_pitch;
    let roll_cmd = target_rate_roll_change * p_roll;
    let yaw_cmd = target_rate_yaw_change * p_yaw;

    // Examine if our current control settings are appropriately effecting the change we want.
    MotorPower::from_cmds(pitch_cmd, roll_cmd, yaw_cmd, throttle, front_left_dir)
}

#[cfg(feature = "fixed-wing")]
pub fn control_posits_from_atts(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    throttle: f32,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    current_posits: &ControlPositions,
) -> ControlPositions {
    // todo: Modulate based on airspeed.

    let rotation_cmd = target_attitude * current_attitude.inverse();

    let (rot_pitch, rot_yaw, rot_roll) = rotation_cmd.to_euler();

    ControlPositions::from_cmds(pitch_cmd, roll_cmd, yaw_cmd, throttle)
}

/// Modify our attitude commanded from rate-based user inputs. `ctrl_crates` are in radians/s, and `dt` is in s.
pub fn modify_att_target(orientation: Quaternion, rates: &RatesCommanded, dt: f32) -> Quaternion {
    // todo: Error handling on this?
    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, rates.pitch.unwrap() * dt);
    let rotation_roll = Quaternion::from_axis_angle(FWD, rates.roll.unwrap() * dt);
    let rotation_yaw = Quaternion::from_axis_angle(UP, rates.yaw.unwrap() * dt);

    // todo: Order?
    rotation_yaw * rotation_roll * rotation_pitch * orientation
}

/// Calculate an attitude based on control input, in `attitude mode`.
pub fn from_controls(ch_data: &ChannelData) -> Quaternion {
    // todo: How do you deal with heading? That's a potential disadvantage of using a quaternion:
    // todo we can calculate pitch and roll, but not yaw.
    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, ch_data.pitch);
    let rotation_roll = Quaternion::from_axis_angle(FWD, ch_data.roll);

    rotation_roll * rotation_pitch
}
