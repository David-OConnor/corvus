//! This module contains code for the ICM42605 inertial measuring unit.
//! This IMU has a 8kHz maximum update rate.

use stm32_hal2::{pac::SPI1, spi::Spi};

use crate::sensor_fusion::ImuReadings;

// todo: Read via DMA at a very high rate, then apply a lowpass filter?

// https://github.com/pms67/Attitude-Estimation

/// Read all data
pub fn read_all(spi: &mut Spi<SPI1>) -> ImuReadings {
    Default::default() // todo
}
