//! This module contains code for the ICM42605 inertial measuring unit.
//! This IMU has a 8kHz maximum update rate.

use stm32_hal2::{
    i2c::I2c,
    pac::{I2C1},
};

use crate::Location;

pub struct GpsFixError {}

pub fn get_fix() -> Result<Location, GpsFixError> {
    Err(GpsFixError {})
}