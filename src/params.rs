//! This module contains code that stores and updates flight parameters: Attitude, angular
//! rates, altitude etc.

// todo: Maybe a more-specific name, like 'flight_params' etc?

use crate::DT_FLIGHT_CTRLS;

use ahrs::ImuReadings;
use lin_alg2::f32::Quaternion;

/// Aircraft flight parameters, at a given instant. Pitch and roll rates are in the aircraft's
/// frame of reference.
#[derive(Default, Clone)]
pub struct Params {
    /// Latitude in degrees; fused.
    pub lat_e8: i64,
    /// Longitude in degrees; fused.
    pub lon_e8: i64,
    /// Altitude fused from GNSS, IMU, and maybe baro. In meters.
    pub alt_msl_fused: f32,
    /// MSL altitude in meters QFE (takeoff location is 0), from a barometer.
    pub alt_msl_baro: f32,
    /// AGL altitude in meters, from the Time of flight sensor.
    pub alt_tof: Option<f32>,
    pub dtd_alt_agl: Option<f32>,

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
    /// Update params with IMU readings, and attitude. If filtering IMU readings, to so before
    ///running this.
    pub fn update_from_imu_readings(&mut self, imu_data: &ImuReadings, attitude: Quaternion) {
        // todo: This is a good place to apply IMU calibration.

        // Calculate angular acceleration. Do this before updating velocities, since we use
        // the prev ones here.
        self.a_pitch = (imu_data.v_pitch - self.v_pitch) / DT_FLIGHT_CTRLS;
        self.a_roll = (imu_data.v_roll - self.v_roll) / DT_FLIGHT_CTRLS;
        self.a_yaw = (imu_data.v_yaw - self.v_yaw) / DT_FLIGHT_CTRLS;

        // Apply filtered gyro and accel readings directly to self.
        self.v_pitch = imu_data.v_pitch;
        self.v_roll = imu_data.v_roll;
        self.v_yaw = imu_data.v_yaw;

        self.a_x = imu_data.a_x;
        self.a_y = imu_data.a_y;
        self.a_z = imu_data.a_z;

        self.attitude_quat = attitude;

        let euler = attitude.to_euler();
        self.s_pitch = euler.pitch;
        self.s_roll = euler.roll;
        self.s_yaw_heading = euler.yaw;
    }

    /// Update lat, lon, and MSL altitude values from our fused position
    pub fn update_positions_from_fused(&mut self, fused: &PositionFused) {
        self.lat_e8 = fused.lat_e8;
        self.lon_e8 = fused.lon_e8;
        self.alt_msl_fused = fused.elevation_msl;
        self.v_x = fused.ned_velocity[0];
        self.v_y = fused.ned_velocity[1];
        self.v_z = fused.ned_velocity[2];
    }
}
