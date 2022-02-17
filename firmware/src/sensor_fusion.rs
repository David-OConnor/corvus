//! This module contains code related to sensor fusion, eg using an extended kalman filter
//! to combine inputs from multiple sensors. It also includes code for interpreting, integrating,
//! and taking the derivatives of sensor readings. Code here is device-agnostic.
//!
//! Some IMUs can integrate with a magnetometer and do some sensor fusion; we use
//! software since it's more general, flexible, and device-agnostic.
//!
//! From HyperShield: "Unscented will not offer any improvement over EKF. The reason for this is
//! that your quadrotor will mainly stay near hover (because of the assumption that gravity
//! will be the dominant acceleration) so the system will appear quasi-linear. If you want to go
//! agile flight then UKF might offer improvements, but then you can't rely on the gravity assumption.
//! There are attitude estimators like the hua filter that circumvents this (it estimates the
//! acceleration in the inertial frame and uses that for prediction instead assuming it's equal
//! to the gravity vector)."
//!

// todo: Try this : https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py

use cmsis_dsp_sys::{arm_cos_f32 as cos, arm_sin_f32 as sin};

use crate::flight_ctrls::Params;

// C file with impl of EKF for quaternion rotation:
// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/EKF.h
// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/updateEKFQuatAtt.m

// Rotors: Quaternion generalization?
// https://marctenbosch.com/quaternions/

// Re quaternions, euler angles, and error state;
// Euler angles are mostly used to synthesize controllers such as PIDs.
// When it comes to state/attitude estimation, quaternions are more common.
// However you can't use a quaternion directly as a state in your EKF (technically you can with
// some care) since the the unit constraint will be violated. You can instead use an
// error-state EKF
// http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf

const G: f32 = 9.8; // m/s

fn tan(val: f32) -> f32 {
    unsafe { sin(val) / cos(val) }
}

/// Represents sensor readings from a 6-axis accelerometer + gyro. Similar to
/// `ParamsInst`.
#[derive(Default)]
pub struct ImuReadings {
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,
    pub v_pitch: f32,
    pub v_roll: f32,
    pub v_yaw: f32,
}

/// Estimate attitude, based on IMU data of accelerations and roll rates.
pub fn estimate_attitude(readings: &ImuReadings) -> Params {
    // Euler angle conventions: θ = pitch. phi = roll.

    // todo: Put useful params here.
    // filter_ = signal.iirfilter(1, 60, btype="lowpass", ftype="bessel", output="sos", fs=32_000)
    // coeffs = []
    // for row in filter_:
    //     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])
    // let coeffs = [0.00585605892206321, 0.00585605892206321, 0.0, 0.9882878821558736, -0.0];

    // Values at rest:
    // a_x = g * sin(θ)
    // a_y = -g * cos(θ) * sin(ϕ)
    // a_z = -g * cos(θ) * cos(ϕ)

    // todo: Apply your lowpass, or confirm hardware lowpass is set up.

    // todo: time-varying bias?

    // Pitch and roll from accel readings alone, using gravity.
    // let ϕ_est = (ay / az).atan();
    // let θ_est = (ax / G).asin();

    // hat = est

    // https://www.youtube.com/watch?v=RZd6XDx5VXo

    // Pitch and roll angles from gyroscopes alone.
    let mut θ_est = 0.;
    let mut ϕ_est = 0.;

    // Transform body rates to Euler angles. (Matrix multiplication)
    // todo: rename from dot etc
    // v_ϕ = readings.p + tan(θ_est) * sin(ϕ_est) * readings.q + cos(ϕ_est) * readings.r;
    // v_θ = cos(ϕ_est) * readings.q - sin(ϕ_est) * readings.r;

    // Integrate Euler rates to get estimate of roll and pitch angles.
    // ϕ_est = ϕ_hat + DT * v_ϕ;
    // θ_est = θ_hat + DT * v_θ;

    Params {
        ..Default::default()
    }
}
