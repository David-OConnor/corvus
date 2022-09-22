//! This module contains code for attitude-based controls. This includes sticks mapping
//! to attitude, and an internal attitude model with rate-like controls, where attitude is the target.

use crate::control_interface::ChannelData;

use super::common::{Params, RatesCommanded};

use lin_alg2::f32::{Quaternion, Vec3};

use num_traits::float::Float; // For sqrt.

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

// todo: COde shortner 3x.

// #[cfg(feature = "quad")]
// fn motor_power_chan(rot: f32, max_rate_dist: f32, max_rate: f32, ) -> f32 {
//     // Compare the current (measured) angular velocities to what we need to apply this rotation.
//     let mut target_rate = if rot > max_rate_dist {
//         max_rate_dist * max_rate
//     } else {
//         // Clamp
//         max_rate
//     };
//
//     // todo Look at how this current power setting is changing rates over time (derivative?)
//     let d_param = params.v - params_prev.v;
//
//     // todo: d rate targets as well?
//
//     let target_rate_change = target_rate - params.v;
//
//     // For each channel, compare the previous control positions to the [rate? change in rate?)
//     // Then adjust the motor powers A/R.
//
//     target_rate_change * p;
// }

#[cfg(feature = "quad")]
pub fn motor_power_from_atts(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    throttle: f32,
    front_left_dir: RotationDir,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    params_prev: &Params,
    prev_power: &MotorPower,
) -> MotorPower {
    // todo: This fn and approach is a WIP!!

    // This is the rotation we need to cause to arrive at the target attitude.
    let rotation_cmd = target_attitude * current_attitude.inverse();

    // todo: Common fn for shared code between here and quad.

    // todo: See if you can briefen this

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

    let ang_accel_pitch = params.v_pitch - params_prev.v_pitch;
    let ang_accel_roll = params.v_roll - params_prev.v_roll;
    let ang_accel_yaw = params.v_yaw - params_prev.v_yaw;

    // If the rate is within this value of the target rate, don't change motor power. This is
    // an experimental way to save power and maybe reduce oscillations(?)
    let rate_thresh = 0.1;

    // todo: Should these go in the delegation fn?
    let tgt_rate_chg_pitch = target_rate_pitch - params.v_pitch;
    let tgt_rate_chg_roll = target_rate_roll - params.v_roll;
    let tgt_rate_chg_yaw = target_rate_yaw - params.v_yaw;

    // Split the rotation into 3 euler angles. We do this due to our controls acting primarily
    // along individual axes.
    let (rot_pitch, rot_roll, rot_yaw) = rotation_cmd.to_euler();

    // todo Start code that should be delegated to by-axis fns

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

    // Estimate the time it will take to arrive at our target attitude,
    // given the current angular velocity, and angular acceleration.
    // We numerically integrate, then solve for time.

    // todo: Take into account reduction in accel as velocity increases due to drag?

    // Integrate using angular rate, and angular accel:
    // θ(t) = θ_0 + ω_0 * t + ω_dot * t^2

    // wolfram alpha: `theta = h + v * t + a * Power[t,2] solve for t`
    // 1/(2*ω_dot) * (sqrt(-4 * 0. + 4ω_dot θ(t) + ω_0^2) +/- ω_0)

    // todo: Should you apply filtering to any of these terms?

    const EPS_1: f32 = 0.0001;

    // todo: From tests, the second variant (b2) appears to work for test examples.
    // todo: Figure out when to use each
    // todo: Figure out what to do when it will never convege; ie it it's accelerating in the wrong
    // todo direction.
    let time_to_tgt_att_pitch = if ang_accel_pitch.abs() < EPS_1 {
        Some(rot_pitch / params.v_pitch)
    } else {
        let a = 2. * ang_accel_pitch;

        let inner = 4. * ang_accel_pitch * rot_pitch + params.v_pitch.powi(2);
        if inner < 0. {
            // Will never reach target (without wrapping around).
            // todo: Use modulus TAU and allow going the wrong way?
            None
        } else {
            let b1 = inner.sqrt() + params.v_pitch;
            let b2 = inner.sqrt() - params.v_pitch;

            let v1 = b2 / a;
            // todo: Is this the approach you want? Pick the positive one?
            // todo: Maybe choose the one with smaller abs? I think that indicates going
            // todo back in time, so probably not.
            if v1 > 0. {
                Some(v1)
            } else {
                Some(-(b1 / a))
            }
        }
    };

    let target_time_to_tgt_att_pitch = rot_pitch * asdf;

    // For each channel, compare the previous control positions to the [rate? change in rate?)
    // Then adjust the motor powers A/R.

    // todo OK, say it is like a pid...
    let d_term_pitch = d_pitch_param * p_pitch;
    let p_term_pitch = tgt_rate_chg_pitch * p_pitch;

    // Calculate how long it will take to reach the target pitch rate at the current rate
    // of change.
    let time_to_tgt_rate_pitch = tgt_rate_chg_pitch / d_term_pitch;

    // eg 5 m/s delta
    //

    // if tgt_rate_chg_pitch <= rate_thresh {
    //     // power_chg_pitch = 0.
    // }

    let power_chg_pitch = if tgt_rate_chg_pitch > rate_thresh {
        // todo: YOu need to take rate changes into effect
        tgt_rate_chg_pitch * p_pitch
    } else {
        0.
    };

    let pitch_cmd = prev_power.pitch + power_chg_pitch;
    let roll_cmd = prev_power.roll + power_chg_roll;
    let yaw_cmd = prev_power.yaw + power_chg_yaw;

    let pitch_cmd = tgt_rate_chg_pitch * p_pitch;
    let roll_cmd = tgt_rate_chg_roll * p_roll;
    let yaw_cmd = tgt_rate_chg_yaw * p_yaw;

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
    params_prev: &Params,
    prev_ctrls: &ControlPositions,
) -> ControlPositions {
    // todo: Modulate based on airspeed.

    let rotation_cmd = target_attitude * current_attitude.inverse();

    let (rot_pitch, rot_roll, rot_yaw) = rotation_cmd.to_euler();

    ControlPositions::from_cmds(pitch_cmd, roll_cmd, yaw_cmd, throttle)
}

/// Modify our attitude commanded from rate-based user inputs. `ctrl_crates` are in radians/s, and `dt` is in s.
pub fn modify_att_target(orientation: Quaternion, rates: &RatesCommanded, dt: f32) -> Quaternion {
    // todo: Error handling on this?

    // Rotate our basis vecs using the orientation, such that control inputs are relative to the
    // current attitude.
    let right = orientation.rotate_vec(RIGHT);
    let fwd = orientation.rotate_vec(FWD);
    let up = orientation.rotate_vec(UP);

    let rotation_pitch = Quaternion::from_axis_angle(right, rates.pitch.unwrap() * dt);
    let rotation_roll = Quaternion::from_axis_angle(fwd, rates.roll.unwrap() * dt);
    let rotation_yaw = Quaternion::from_axis_angle(up, rates.yaw.unwrap() * dt);

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
