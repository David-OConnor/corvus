//! Driver for the ST LIS3MDL 3-axis magnetometer. This is an I2C or SPI device;
//! This module only includes functionality for I2C mode.

use stm32_hal2::{i2c::{self, I2c}, pac::I2C1};

use cortex_m::delay::Delay;

// todo: What is the addr?
const ADDR: u8 = 0x52;

pub struct MagNotConnectedError {}

impl From<i2c::Error> for MagNotConnectedError {
    fn from(e: i2c::Error) -> Self {
        Self {}
    }
}

pub fn setup(i2c: &mut I2c<I2C1>) -> Result<(), MagNotConnectedError> {

    Ok(())
}
