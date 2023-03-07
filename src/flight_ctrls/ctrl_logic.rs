//! This module contains code for attitude-based controls. This includes sticks mapping
//! to attitude, and an internal attitude model with rate-like controls, where attitude is the target.

use crate::{
    control_interface::ChannelData,
    params::Params,
    util::{self, map_linear},
};

use super::{
    common::CtrlMix,
    ctrl_effect_est::{AccelMap, AccelMaps},
    filters::FlightCtrlFilters,
    motor_servo::{MotorServoState, RotationDir},
};

use lin_alg2::f32::{Quaternion, Vec3};

use num_traits::float::Float; // For sqrt.

use cfg_if::cfg_if;

// todo: YOu probably need filters.

cfg_if! {
    if #[cfg(feature = "quad")] {
        use super::motor_servo::MotorRpm;
    } else {
        use super::motor_servo::CtrlSfcPosits;
    }
}

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
fn α_from_ttc(dθ: f32, ω: f32, ttc_per_dθ: f32) -> f32 {
    // Time to correction
    let ttc = ttc_per_dθ * dθ.abs(); // todo: Naive. Take vel and accel into account.

    // Calculate the "initial" target angular acceleration.
    let α_0 = -(6. * dθ + 4. * ttc * ω) / ttc.powi(2);

    // It appears we don't actually need to calculate j
    // for our controller; although this value calculated here should be
    // roughly maintained given how α evolves over time given
    // the same control logic is applied repeatedly.

    // Calculate the (~constant for a given correction) change in angular acceleration.
    // (Commented-out, since we don't directly need this)
    // let j = 6. * (2. * dθ + ttc * ω_0) / ttc.powi(3);

    // α_0, j
    α_0
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

fn find_ctrl_setting(
    dθ: f32,
    ω_0: f32,
    α_meas: f32,
    // ctrl_cmd_prev: f32,
    coeffs: &CtrlCoeffs,
    drag_coeff: f32,
    accel_map: &AccelMap,
    // filters: &mut FlightCtrlFilters,
) -> f32 {
    // todo: Take time to spin up/down into account

    const EPS: f32 = 0.000001;

    // `t` here is the total time to complete this correction, using the analytic
    // formula.
    let t = if α_meas.abs() < EPS {
        Some((3. * dθ) / (2. * ω_0))
    } else {
        // If `inner` is negative, there is no solution for the desired α_0;
        // we must change it.
        // It would be negative if, for example, α_0 and/or θ_0 is high,
        // and/or ω_0 is low.
        // This would manifest in an imaginary time.
        // We resolve this by specifying a time-to-correction based on
        // current parameters, and applying a discontinuity in angular accel;
        // this discontinuity allows us to still find a constant-jerk
        // result.
        let inner = 4. * ω_0.powi(2) - 6. * α_meas * dθ;
        if inner < 0. {
            None
        } else {
            let t_a = -(inner.sqrt() + 2. * ω_0) / α_meas;
            let t_b = (inner.sqrt() - 2. * ω_0) / α_meas;

            // todo: QC this.
            if t_a < 0. {
                Some(t_b)
            } else {
                Some(t_a)
            }
        }
    };

    let mut α_target = match t {
        Some(ttc) => {
            // If it would take too longer to perform the correction, calculate a new
            // angular acceleration that fits the criteria.
            if ttc > coeffs.max_ttc_per_dθ * dθ {
                α_from_ttc(dθ, ω_0, coeffs.ttc_per_dθ)
                // If the time to correction is sufficiently small, apply a constant jerk, which
                // should be roughly the previous jerk if part way through a maneuver.
            } else {
                // Calculate the (~constant for a given correction) change in angular acceleration.
                let j = 6. * (2. * dθ + ttc * ω_0) / ttc.powi(3);

                // This is the actual target acceleration, determined by the questions above:
                α_meas + j
            }
        }
        None => α_from_ttc(dθ, ω_0, coeffs.ttc_per_dθ),
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

    accel_map.interpolate(α_target)
}

#[cfg(feature = "quad")]
/// Calculate target rotor RPM from current and target attitudes. This is our entry point
/// for control application: It generates motor powers (in 2 formats) based on the
/// parameters, and commands.
/// The DT passed is the IMU rate, since we update params_prev each IMU update.
pub fn ctrl_mix_from_att(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
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
    // This is the rotation we need to create to arrive at the target attitude.
    let rotation_cmd = target_attitude * current_attitude.inverse();
    // Split the rotation into 3 euler angles. We do this due to our controls acting primarily
    // along individual axes.
    let rot_euler = rotation_cmd.to_euler();

    let pitch = find_ctrl_setting(
        rot_euler.pitch,
        params.v_pitch,
        params.a_pitch,
        coeffs,
        drag_coeffs.pitch,
        &accel_maps.rpm_to_accel_pitch,
    );
    let roll = find_ctrl_setting(
        rot_euler.roll,
        params.v_roll,
        params.a_roll,
        coeffs,
        drag_coeffs.roll,
        &accel_maps.rpm_to_accel_roll,
    );
    let yaw = find_ctrl_setting(
        rot_euler.yaw,
        params.v_yaw,
        params.a_yaw,
        coeffs,
        drag_coeffs.yaw,
        &accel_maps.rpm_to_accel_yaw,
    );

    CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    }
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
        &accel_maps.rpm_to_accel_pitch,
    );
    let roll = find_ctrl_setting(
        rot_euler.roll,
        params.v_roll,
        params.a_roll,
        coeffs,
        drag_coeffs.roll,
        &accel_maps.rpm_to_accel_roll,
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

    // todo: Order?
    rotation_yaw * rotation_roll * rotation_pitch * orientation
}

/// Calculate an attitude based on control input, in `attitude mode`.
pub fn _att_from_ctrls(ch_data: &ChannelData) -> Quaternion {
    // todo: How do you deal with heading? That's a potential disadvantage of using a quaternion:
    // todo we can calculate pitch and roll, but not yaw.
    let rotation_pitch = Quaternion::from_axis_angle(RIGHT, ch_data.pitch);
    let rotation_roll = Quaternion::from_axis_angle(FWD, ch_data.roll);

    rotation_roll * rotation_pitch
}
