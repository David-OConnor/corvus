//! This module contains code for the ICM42605 inertial measuring unit.

use stm32_hal2::{
    spi::Spi,
    pac::{SPI1},
}

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

/// Read all data
pub fn read_all(spi: &mut Spi<SPI1>) -> ImuReadings {
    Default::default()  // todo
}
