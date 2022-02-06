//! This module contains code for the ICM42605 inertial measuring unit.

use stm32_hal2::{pac::SPI1, spi::Spi};

/// Represents sensor readings from a 6-axis accelerometer + gyro. Similar to
/// `ParamsInst`.
#[derive(Default)]
pub struct ImuReadings {
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,
    pub s_pitch: f32,
    pub s_roll: f32,
    pub s_yaw: f32,
}

/// Read all data
pub fn read_all(spi: &mut Spi<SPI1>) -> ImuReadings {
    Default::default() // todo
}
