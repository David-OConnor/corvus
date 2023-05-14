//! This module contains code related to using sensor fusion to create an attitude platform. It also includes code for interpreting, integrating,
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
//! Python guide: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
//!
//! [Youtube video: Phil's Lab](https://www.youtube.com/watch?v=hQUkiC5o0JI)
//!
//! Important: We define X to be left/right (pitch), Y to be forward/back (roll, and
//! Z to be up/down (yaw)

// todo: Try this : https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py

// use cmsis_dsp_sys::{arm_cos_f32 as cos, arm_sin_f32 as sin};

use crate::ahrs_fusion::Ahrs;

use crate::params::Params;
use lin_alg2::f32::Vec3;

// use defmt::println;

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

/// Update the attitude from the AHRS.
pub fn update_attitude(ahrs: &mut Ahrs, params: &mut Params, mag_data: Option<Vec3>, dt: f32) {
    // Gyro measurements - not really a vector.
    // In our IMU interpretation, we use direction references that make sense for our aircraft.
    // See `imu_shared::ImuReadings` field descriptions for this. Here, we undo it: The AHRS
    // fusion algorithm expects raw readings. (See the - signs there and here; they correspond)
    let accel_data = Vec3 {
        x: -params.a_y,
        y: -params.a_x,
        z: params.a_z,
    };

    let gyro_data = Vec3 {
        x: params.v_roll,
        y: params.v_pitch,
        z: params.v_yaw,
    };

    // Apply calibration
    // todo: Come back to this.
    // gyro_data = madgwick::apply_cal_inertial(
    //     gyro_data,
    //     ahrs.calibration.gyro_misalignment.clone(),
    //     ahrs.calibration.gyro_sensitivity,
    //     ahrs.calibration.gyro_offset,
    // );
    // accel_data = madgwick::apply_cal_inertial(
    //     accel_data,
    //     ahrs.calibration.accel_misalignment.clone(),
    //     ahrs.calibration.accel_sensitivity,
    //     ahrs.calibration.accel_offset,
    // );

    // let magnetometer = madgwick::apply_cal_magnetic(magnetometer, softIronMatrix, hardIronOffset);

    // todo: Consider putting this offset bit back later
    // Update gyroscope offset correction algorithm
    // let gyro_data_with_offset = ahrs.offset.update(gyro_data);
    // let gyro_data_with_offset = gyro_data;

    match mag_data {
        Some(m) => {
            let mag_data = Vec3 {
                x: m.y,
                y: m.x,
                z: m.z,
            };
            ahrs.update(gyro_data, accel_data, mag_data, dt);
        }
        None => {
            ahrs.update_no_magnetometer(gyro_data, accel_data, dt);
        }
    }

    let att_euler = ahrs.quaternion.to_euler();

    params.s_pitch = att_euler.pitch; // around x axis
    params.s_yaw_heading = att_euler.yaw; // around y axis
    params.s_roll = att_euler.roll; // around z axis
    params.attitude_quat = ahrs.quaternion;
}