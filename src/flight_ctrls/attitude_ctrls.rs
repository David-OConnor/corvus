//! This module contains code for attitude-based controls. This includes sticks mapping
//! to attitude, and an internal attitude model with rate-like controls.

use crate::{
    control_interface::ChannelData,
    lin_alg::{Quaternion, Vec3},
};

use super::common::CtrlInputs;

#[cfg(feature = "quad")]
use super::MotorPower;

pub const RIGHT: Vec3 = Vec3 {
    x: 1.,
    y: 0.,
    z: 0.,
};
pub const UP: Vec3 = Vec3 {
    x: 0.,
    y: 1.,
    z: 0.,
};
pub const FWD: Vec3 = Vec3 {
    x: 0.,
    y: 0.,
    z: 1.,
};

#[cfg(feature = "quad")]
pub fn motor_power_from_rotation(rotation: Quaternion) -> MotorPower {
    MotorPower {
        front_left: 0.,
        front_right: 0.,
        aft_left: 0.,
        aft_right: 0.,
    }
}

/// Modify our attitude commanded from user inputs. `ctrl_crates` are in radians/s, and `dt` is in s.
pub fn modify_commanded(orientation: Quaternion, ctrl_rates: &CtrlInputs, dt: f32) -> Quaternion {
    // todo: Error handling on this?
    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, ctrl_rates.pitch.unwrap() * dt);
    let rotation_roll = Quaternion::from_axis_angle(FWD, ctrl_rates.roll.unwrap() * dt);
    let rotation_yaw = Quaternion::from_axis_angle(UP, ctrl_rates.yaw.unwrap() * dt);

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
