//! Driver for the ST LIS3MDL 3-axis magnetometer. This is an I2C or SPI device;
//! This module only includes functionality for I2C mode.

use stm32_hal2::{i2c::I2c, pac::I2C1};

use cortex_m::delay::Delay;

pub fn setup(i2c: I2c<I2C1>) {

}