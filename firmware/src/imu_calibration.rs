//! This calibration functionality is from [AHRS](https://github.com/xioTechnologies/Fusion)

use crate::lin_alg::{Mat3, Vec3};

pub struct ImuCalibration {
    pub gyro_misalignment: Mat3,
    pub gyro_sensitivity: Vec3,
    pub gyro_offset: Vec3,
    pub accel_misalignment: Mat3,
    pub accel_sensitivity: Vec3,
    pub accel_offset: Vec3,
    pub soft_iron_matrix: Mat3,
    pub hard_iron_offset: Vec3,
}

impl Default for ImuCalibration {
    #[rustfmt::skip]
    fn default() -> Self {
        Self {
            gyro_misalignment: Mat3 {
                data: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
            },
            gyro_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            gyro_offset: Vec3::new(0.0, 0.0, 0.0),
            accel_misalignment: Mat3 {
                data: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
            },
            accel_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            accel_offset: Vec3::new(0.0, 0.0, 0.0),
            soft_iron_matrix: Mat3 {
                data: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
            },
            hard_iron_offset: Vec3::new(0.0, 0.0, 0.0),
        }
    }
}

/// Gyroscope and accelerometer calibration model. Returns calibrated measurement.
pub fn apply_cal_inertial(
    uncalibrated: Vec3,
    misalignment: Mat3,
    sensitivity: Vec3,
    offset: Vec3,
) -> Vec3 {
    misalignment * (uncalibrated - offset).hadamard_product(sensitivity)
}

/// Magnetometer calibration model. Returns calibrated measurement.
pub fn apply_cal_magnetic(
    uncalibrated: Vec3,
    soft_iron_matrix: Mat3,
    hard_iron_offset: Vec3,
) -> Vec3 {
    soft_iron_matrix * uncalibrated - hard_iron_offset
}

/// Calibrate the IMU, by taking a series of series while on a level surface.
pub fn calibrate() -> ImuCalibration {
    // todo: average? lowpass?
    Default::default()
}
