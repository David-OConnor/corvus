//! This module contains code for attitude-based controls. This includes sticks mapping
//! to attitude, and an internal attitude model with rate-like controls, where attitude is the target.

use crate::{control_interface::ChannelData, util::map_linear};

use super::{
    common::{CtrlMix, Params, RatesCommanded},
    filters::FlightCtrlFilters,
};

use lin_alg2::f32::{Quaternion, Vec3};

use num_traits::float::Float; // For sqrt.

use cfg_if::cfg_if;

// todo: YOu probably need filters.

cfg_if! {
    if #[cfg(feature = "quad")] {
        use super::{MotorPower, RotationDir};
    } else {
        use super::ControlPositions;
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

/// Map RPM to angular acceleration (thrust proxy). Average over time, and over all props.
/// Note that this relationship may be exponential, or something similar, with RPM increases
/// at higher ranges providing a bigger change in thrust.
/// /// For fixed wing, we use servo position instead of RPM.
#[cfg(feature = "quad")]
struct RpmToAccel {
    // Value are in RPM.

    // todo: Use the same approach as power to RPM, where you log intermediate values.

    // todo: What is the max expected RPM? Adjust this A/R.
    // todo: An internet search implies 4-6k is normal.

    // Lower power should probably be from idle, not 0. So inclide p_0 here.
    r_0: f32, // Likely 0.
    r_1k: f32,
    r_2k: f32,
    r_3k: f32,
    r_4k: f32,
    r_5k: f32,
    r_6k: f32,
    r_7k: f32,
    r_8k: f32,
    r_9k: f32,
    r_10k: f32,
}

#[cfg(feature = "quad")]
impl RpmToAccel {
    // todo: DRY with pwr to rpm MAP
    /// Interpolate, to get power from this LUT.
    pub fn rpm_to_angular_accel(&self, rpm: f32) -> f32 {
        let end_slope = (self.r_10k - self.r_9k) / 1_000.;

        match rpm {
            (0.0..=1_000.) => map_linear(rpm, (0.0, 1_000.), (self.r_0, self.r_1k)),
            (1_000.0..=2_000.) => map_linear(rpm, (1_000., 2_000.), (self.r_1k, self.r_2k)),
            (2_000.0..=3_000.) => map_linear(rpm, (2_000., 3_000.), (self.r_2k, self.r_3k)),
            (3_000.0..=4_000.) => map_linear(rpm, (3_000., 4_000.), (self.r_3k, self.r_4k)),
            (4_000.0..=5_000.) => map_linear(rpm, (4_000., 5_000.), (self.r_4k, self.r_5k)),
            (5_000.0..=6_000.) => map_linear(rpm, (5_000., 6_000.), (self.r_5k, self.r_6k)),
            (6_000.0..=7_000.) => map_linear(rpm, (6_000., 7_000.), (self.r_6k, self.r_7k)),
            (7_000.0..=8_000.) => map_linear(rpm, (7_000., 8_000.), (self.r_7k, self.r_8k)),
            (8_000.0..=9_000.) => map_linear(rpm, (8_000., 9_000.), (self.r_8k, self.r_9k)),
            (9_000.0..=10_000.) => map_linear(rpm, (9_000., 10_000.), (self.r_9k, self.r_10k)),
            // If above 10k, extrapolate from the prev range.
            _ => rpm * end_slope,
        }
    }

    /// Log a power, and rpm.
    pub fn log_val(&mut self, rpm: f32, accel: f32) {
        // todo: Allow for spin-up time.

        // todo: filtering! But how, given the pwr these are logged at changes?
        // todo: Maybe filter a an interpolation to the actual values, and store those?

        if rpm < 0.1 {
            self.p_0 = (rpm, accel);
        } else if rpm < 0.3 {
            self.p_20 = (rpm, accel);
        } else if rpm < 0.5 {
            self.p_40 = (rpm, accel);
        } else if rpm < 0.7 {
            self.p_60 = (rpm, accel);
        } else if rpm < 0.9 {
            self.p_80 = (rpm, accel);
        } else {
            self.p_100 = (rpm, accel);
        }

        self.p_x = (rpm, accel);
    }
}

/// This struct contains maps of 0-1 power level to RPM and angular accel.
/// For fixed wing, substitude servo position setting for RPM.
#[derive(Default)]
pub struct PowerMaps {
    // pub pwr_to_rpm_pitch: PwrToRpmMap,
    // pub pwr_to_rpm_roll: PwrToRpmMap,
    // pub pwr_to_rpm_yaw: PwrToRpmMap,
    pub rpm_to_accel_pitch: RpmToAccel,
    pub rpm_to_accel_roll: RpmToAccel,
    pub rpm_to_accel_yaw: RpmToAccel,
}

/// Control coefficients that affect the toleranaces and restrictions of the flight controls.
pub struct CtrlCoeffs {
    /// This field is used for a simple target angular velocity for a given angular distance
    /// from the target angle (eg pitch, roll, yaw). Units are (rad/s) / rad = 1/s
    /// Eg with value = 1, at tau/2 rad (max angle), we get target a rate of tau/2 rad/s. At 0 angle,
    /// we command 0 rad/s regardless of thi value. Higher values for this constant
    /// raise the rate for a given angular distance. This value is the time, at this
    /// rate, it takes to travel a whole revolution.
    /// todo: More complicated, non-linear model? Eg specified by a LUT.
    /// todo: Another possible approach is with a clipped value past a certain angle.
    /// todo: Consider different values for pitch, roll, and yaw.
    p_ω: f32,
    /// Time to correction is a coefficient that determines how quickly the angular
    /// velocity will be corrected to the target.
    /// Lower values mean more aggressive corrections.
    // time_to_correction_p_ω: f32,
    // time_to_correction_p_θ: f32,
    time_to_correction: f32,
    // dist_to_correction_p_ω: f32,
    // dist_to_correction_p_θ: f32,
    // /// In rad/s^2. A higher value will allow for more aggressive corrections.
    // max_ω_dot: f32,
}

// todo: Maybe a sep `CtrlCoeffs` struct for each axis.

impl Default for CtrlCoeffs {
    #[cfg(feature = "quad")]
    fn default() -> Self {
        Self {
            p_ω: 10.,
            // time_to_correction_p_ω: 0.1, // todo?
            // time_to_correction_p_θ: 0.5, // todo?
            time_to_correction: 0.1,
            // max_ω_dot: 10.,              // todo: What should this be?
        }
    }

    #[cfg(feature = "fixed-wing")]
    fn default() -> Self {
        Self {
            p_ω: 10.,
            // time_to_correction_p_ω: 0.1,
            // time_to_correction_p_θ: 0.5,
            // dist_to_correction_p_ω: 0.1,
            // dist_to_correction_p_θ: 0.1,
            time_to_correction: 0.1,
            // max_ω_dot: 10.,
        }
    }
}

/// Calculate the commanded acceleration required to meet a desired acceleration
/// by taking drag into account
fn calc_drag_coeff(ω_meas: f32, ω_dot_meas: f32, ω_dot_commanded: f32) -> f32 {
    // https://physics.stackexchange.com/questions/304742/angular-drag-on-body
    // This coefficient maps angular velocity to drag acceleration directly,
    // and is measured (and filtered).

    // todo: For "low-speeds", drag is proportionanl to ω. For high speeds, it's
    // todo prop to ω^2. The distinction is the reynolds number.

    // todo: Low speed for now.
    // drag_accel = -cω
    // ω_dot = ω_dot_commanded - cω
    // c = -1/ω * (ω_dot - ω_dot_commanded)
    // ω_dot_commanded = ω_dot + cω

    -1. / ω_meas * (ω_dot - ω_dot_commanded)
}

/// Calculate the time in seconds allocated to perform an attitude change, based
/// on the rotation angle to perform, and current angular velocity.
/// ω positive means going towards the target; negative means away.
fn calc_time_to_correction(dθ: f32, ω: f32, coeff: f32) -> f32 {
    // Attempting an approach based on calculating an impulse.
    // This is in units of force * units of time. Taking out mass,
    // we use acceleration * time.
    // J = F_average x (t2 - t1)

    // dθ = ω * dt

    // Let's say θ = 1. ω = 0. Let's say time = 1
    // It takes 1 rad/s dωto arrive

    // if θ = 1. ω = 1, it takes 0 rad/s to arrive in 1s
    // if θ = 1. ω = 0, it takes 1 rad/s to arrive
    // if θ = 1. ω = -1, it takes 2 rad/s to arrive
    // if θ = 1. ω = -2, it takes 3 rad/s to arrive
    // if θ = 2. ω = -2, it takes 4 rad/s to arrive in 1s

    // if θ = 1. ω = 0, it takes 1/2 rad/s to arrive in 2s
    // if θ = 1. ω = 1, it takes -1/2 rad/s to arrive in 2s
    // if θ = 1. ω = -2, it takes -1/2 rad/s to arrive in 2s

    // The rule: dω/dt  =  θ/t.      dω =      ω - t?
    // θ/t - ω

    //If
    //

    // todo: Probably not right?
    // t

    // rad/s - rad/s = 0

    θ / t - ω
}

/// Find the desired control setting on a single axis; loosely corresponds to a
/// commanded angular acceleration. We assume, physical limits (eg motor power available)
/// aside, a constant change in angular acceleration (jerk) for a given correction.
fn find_ctrl_setting(
    dθ: f32,
    ω_0: f32,
    ω_dot: f32,
    ctrl_cmd_prev: f32,
    coeffs: &CtrlCoeffs,
    filters: &mut FlightCtrlFilters,
) -> f32 {
    // todo: Take RPM and/or time-to-spin up/down into account.

    // todo: More work here.
    let ttc = calc_time_to_correction(dθ, ω_0, coeffs.time_to_correction);

    // Calculate the "initial" target angular acceleration.
    let ω_dot_0 = -(6. * dθ + 4. * ttc * ω_0) / ttc.powi(2);

    // Calculate the (~constant for a given correction) change in angular acceleration.
    let ω_dot_dot = 6. * (2. * dθ + ttc * ω_0) / ttc.powi(3);

    // The target acceleration needs to include both the correction, and drag compensation.
    // todo: QC sign etc on this.
    ω_dot_target -= drag_accel;

    // Calculate how, most recently, the control command is affecting angular accel.
    // A higher constant means a given command has a higher affect on angular accel.
    // todo: Track and/or lowpass effectiveness over recent history, at diff params.
    // todo: Once you have bidir dshot, use RPM instead of power.

    let ctrl_effectiveness = ω_dot / ctrl_cmd_prev;

    // Apply a lowpass filter to our effectiveness, to reduce noise and fluctuations.
    let ctrl_effectiveness = filters.apply(ctrl_effectiveness);

    // This distills to: (dω / time_to_correction) / (ω_dot / ctrl_cmd_prev) =
    // (dω / time_to_correction) x (ctrl_cmd_prev / ω_dot) =
    // (dω x ctrl_cmd_prev) / (time_to_correction x ω_dot) =
    //
    // (dθ * coeffs.p_ω - ω x ctrl_cmd_prev) /
    // ((coeffs.time_to_correction_p_ω * dω.abs() + coeffs.time_to_correction_p_θ * dθ.abs()) x ω_dot)

    // Units: rad x cmd / (s * rad/s) = rad x cmd / rad = cmd
    // `cmd` is the unit we use for ctrl inputs. Not sure what (if any?) units it has.
    ω_dot_target / ctrl_effectiveness
}

#[cfg(feature = "quad")]
pub fn motor_power_from_atts(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    throttle: f32,
    front_left_dir: RotationDir,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    params_prev: &Params,
    mix_prev: &CtrlMix,
    coeffs: &CtrlCoeffs,
    filters: &mut FlightCtrlFilters,
    dt: f32, // seconds
) -> (CtrlMix, MotorPower) {
    // todo: This fn and approach is a WIP!!

    // This is the rotation we need to cause to arrive at the target attitude.
    let rotation_cmd = target_attitude * current_attitude.inverse();
    // Split the rotation into 3 euler angles. We do this due to our controls acting primarily
    // along individual axes.
    let (rot_pitch, rot_roll, rot_yaw) = rotation_cmd.to_euler();

    let ang_accel_pitch = (params.v_pitch - params_prev.v_pitch) * dt;
    let ang_accel_roll = (params.v_roll - params_prev.v_roll) * dt;
    let ang_accel_yaw = (params.v_yaw - params_prev.v_yaw) * dt;

    let pitch = find_ctrl_setting(
        rot_pitch,
        params.v_pitch,
        ang_accel_pitch,
        mix_prev.pitch,
        // dt,
        coeffs,
        filters,
    );
    let roll = find_ctrl_setting(
        rot_roll,
        params.v_roll,
        ang_accel_roll,
        mix_prev.roll,
        // dt,
        coeffs,
        filters,
    );
    let yaw = find_ctrl_setting(
        rot_yaw,
        params.v_yaw,
        ang_accel_yaw,
        mix_prev.yaw,
        // dt,
        coeffs,
        filters,
    );

    let mix_new = CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    };

    let power = MotorPower::from_cmds(&mix_new, front_left_dir);

    // Examine if our current control settings are appropriately effecting the change we want.
    (mix_new, power)
}

#[cfg(feature = "fixed-wing")]
/// Similar to the above fn on quads. Note that we do not handle yaw command using this. Yaw
/// is treated as coupled to pitch and roll, with yaw controls used to counter adverse-yaw.
/// Yaw is to maintain coordinated flight, or deviate from it.
pub fn control_posits_from_atts(
    target_attitude: Quaternion,
    current_attitude: Quaternion,
    throttle: f32,
    // todo: Params is just for current angular rates. Maybe just pass those?
    params: &Params,
    params_prev: &Params,
    mix_prev: &CtrlMix,
    coeffs: &CtrlCoeffs,
    filters: &mut FlightCtrlFilters,
    dt: f32, // seconds
) -> (CtrlMix, ControlPositions) {
    // todo: Modulate based on airspeed.

    let rotation_cmd = target_attitude * current_attitude.inverse();
    let (rot_pitch, rot_roll, _rot_yaw) = rotation_cmd.to_euler();

    let ang_accel_pitch = (params.v_pitch - params_prev.v_pitch) * dt;
    let ang_accel_roll = (params.v_roll - params_prev.v_roll) * dt;

    let pitch = find_ctrl_setting(
        rot_pitch,
        params.v_pitch,
        ang_accel_pitch,
        mix_prev.pitch,
        // dt,
        coeffs,
        filters,
    );
    let roll = find_ctrl_setting(
        rot_roll,
        params.v_roll,
        ang_accel_roll,
        mix_prev.roll,
        // dt,
        coeffs,
        filters,
    );

    let yaw = 0.; // todo?

    let mix_new = CtrlMix {
        pitch,
        roll,
        yaw,
        throttle,
    };

    let posits = ControlPositions::from_cmds(&mix_new);

    (mix_new, posits)
}

/// Modify our attitude commanded from rate-based user inputs. `ctrl_crates` are in radians/s, and `dt` is in s.
pub fn modify_att_target(orientation: Quaternion, rates: &RatesCommanded, dt: f32) -> Quaternion {
    // todo: Error handling on this?

    // Rotate our basis vecs using the orientation, such that control inputs are relative to the
    // aircraft's attitude.
    let right_ac = orientation.rotate_vec(RIGHT);
    let fwd_ac = orientation.rotate_vec(FWD);
    let up_ac = orientation.rotate_vec(UP);

    let rotation_pitch = Quaternion::from_axis_angle(right_ac, rates.pitch.unwrap() * dt);
    let rotation_roll = Quaternion::from_axis_angle(fwd_ac, rates.roll.unwrap() * dt);
    let rotation_yaw = Quaternion::from_axis_angle(up_ac, rates.yaw.unwrap() * dt);

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
