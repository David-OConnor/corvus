//! This module contains code that stores and updates flight parameters: Attitude, angular
//! rates, altitude etc.

// todo: Maybe a more-specific name, like 'flight_params' etc?

use crate::{imu_shared::ImuReadings, DT_IMU};

use lin_alg2::f32::Quaternion;

/// Aircraft flight parameters, at a given instant. Pitch and roll rates are in the aircraft's
/// frame of reference.
#[derive(Default, Clone)]
pub struct Params {
    /// Latitude in radians. From GPS alone, or blended with accelerometer data.
    pub lat: f32,
    /// Longitude in radians. From GPS alone, or blended with accelerometer data.
    pub lon: f32,
    /// MSL altitude in meters QFE (takeoff location is 0), from a barometer.
    pub baro_alt_msl: f32,
    /// AGL altitude in meters, from the Time of flight sensor.
    pub tof_alt: Option<f32>,

    pub s_pitch: f32,
    pub s_roll: f32,
    /// Ie heading
    pub s_yaw_heading: f32,

    /// Quaternion of the attitude.
    pub attitude_quat: Quaternion,
    // todo: AHRS quaternion field, or leave that as part of the `AHRS` struct?

    // Velocity
    pub v_x: f32,
    pub v_y: f32,
    pub v_z: f32,

    pub v_pitch: f32,
    pub v_roll: f32,
    pub v_yaw: f32,

    // Acceleration
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,

    pub a_pitch: f32,
    pub a_roll: f32,
    pub a_yaw: f32,
}

impl Params {
    pub fn update_from_imu_readings(&mut self, mut imu_data: ImuReadings) {
        // todo: This is a good place to apply IMU calibration.

        // Calculate angular acceleration. Do this before updating velocities, since we use
        // the prev ones here.
        self.a_pitch = (imu_data.v_pitch - self.v_pitch) / DT_IMU;
        self.a_roll = (imu_data.v_roll - self.v_roll) / DT_IMU;
        self.a_yaw = (imu_data.v_yaw - self.v_yaw) / DT_IMU;

        // Apply filtered gyro and accel readings directly to self.
        self.v_pitch = imu_data.v_pitch;
        self.v_roll = imu_data.v_roll;
        self.v_yaw = imu_data.v_yaw;

        self.a_x = imu_data.a_x;
        self.a_y = imu_data.a_y;
        self.a_z = imu_data.a_z;
    }
}
