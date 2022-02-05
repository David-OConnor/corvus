//! This module contains code for the _ inertial measuring unit.

/// Represents sensor readings from a 6-axis accelerometer + gyro. Similar to
/// `ParamsInst`.
#[derive(Default)]
pub struct ImuReadings {
    a_x: f32,
    a_y: f32,
    a_z: f32,
    s_pitch: f32,
    s_roll: f32,
    s_yaw: f32,
}
