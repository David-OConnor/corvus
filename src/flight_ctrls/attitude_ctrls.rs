//! This module contains code for attitude-based controls. This includes sticks mapping
//! to attitude, and an internal attitude model with rate-like controls, where attitude is the target.

use crate::control_interface::ChannelData;

use super::common::{RatesCommanded};

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
    power: f32,
    front_left_dir: RotationDir,
) -> MotorPower {
    let rotation_cmd = target_attitude * current_attitude.inverse();

    let (pitch_half_delta, mut yaw_half_delta, roll_half_delta) = rotation_cmd.to_euler();

    let mut front_left = 0.;
    let mut front_right = 0.;
    let mut aft_left = 0.;
    let mut aft_right = 0.;

    // Nose down for positive pitch.
    front_left -= pitch_half_delta;
    front_right -= pitch_half_delta;
    aft_left += pitch_half_delta;
    aft_right += pitch_half_delta;

    // Left side up for positive roll
    front_left += roll_half_delta;
    front_right -= roll_half_delta;
    aft_left += roll_half_delta;
    aft_right -= roll_half_delta;

    // Assumes positive yaw from the IMU means clockwise. // todo: Confirm this.
    // If props rotate in, front-left/aft-right rotors induce a CCW torque on the aircraft.
    // If props rotate out, these same rotors induce a CW torque.
    // This code assumes props rotate inwards towards the front and back ends.
    if front_left_dir == RotationDir::Clockwise {
        yaw_half_delta *= -1.;
    }

    front_left += yaw_half_delta;
    front_right -= yaw_half_delta;
    aft_left -= yaw_half_delta;
    aft_right += yaw_half_delta;

    let mut pwr = MotorPower {
        front_left,
        front_right,
        aft_left,
        aft_right,
    };

    pwr.clamp_individual_rotors();

    pwr
}

#[cfg(feature = "fixed-wing")]
pub fn control_posits_from_atts(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    power: f32,
) -> ControlPositions {
    // todo: Modulate based on airspeed.

    let rotation_cmd = target_attitude * current_attitude.inverse();

    let (pitch_delta, yaw_delta, roll_delta) = rotation_cmd.to_euler();

    let mut elevon_left = 0.;
    let mut elevon_right = 0.;
    let mut rudder = 0.;

    elevon_left += pitch_delta;
    elevon_right += pitch_delta;

    elevon_left += roll_delta * ROLL_COEFF;
    elevon_right -= roll_delta * ROLL_COEFF;

    rudder += pitch_delta * YAW_COEFF;

    ControlPositions {
        motor: power,
        elevon_left,
        elevon_right,
        rudder,
    }
}

/// Modify our attitude commanded from rate-based user inputs. `ctrl_crates` are in radians/s, and `dt` is in s.
pub fn modify_commanded(orientation: Quaternion, rates: &RatesCommanded, dt: f32) -> Quaternion {
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
