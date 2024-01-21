//! This module contains code for control logic. (todo: expand)

use ahrs::Params;
use cfg_if::cfg_if;
use defmt::println;
use lin_alg2::f32::Quaternion;

use super::{common::CtrlMix, ctrl_effect_est::AccelMaps, filters::FlightCtrlFilters};
use crate::flight_ctrls::{
    motor_servo::RotationDir,
    pid::{PidCoeffs, PidStateRate},
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

    pitch_rate_cmd = pitch_rate_cmd.clamp(-MAX_ATT_CORRECTION_ω, MAX_ATT_CORRECTION_ω);
    roll_rate_cmd = roll_rate_cmd.clamp(-MAX_ATT_CORRECTION_ω, MAX_ATT_CORRECTION_ω);
    yaw_rate_cmd = yaw_rate_cmd.clamp(-MAX_ATT_CORRECTION_ω, MAX_ATT_CORRECTION_ω);

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
