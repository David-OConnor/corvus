//! This module contains code for the Infineon DPS310 barometer.

use stm32_hal2::{
    i2c::I2c,
    pac::I2C1,
};

/// Used to calibrate. Can use a single point, in conjunction with a standard
/// atmosphere model.
struct BaroCalPt {
    pressure: f32, // Pa
    altitude: f32, // MSL, via GPS, in meters
    temp: f32,  // C
}

pub struct Barometer {
    calibration: BaroCalPt
}

impl Barometer {
    pub fn calibrate(&mut self, altitude: f32, temp: f32) {
        self.calibration = BaroCalPt {
            pressure: self.read(),
            altitude,
            temp
        }
    }

    /// Read atmospheric pressure, in kPa
    pub fn read(i2c: &mut I2c<I2C1>) -> f32 {
        0.
    }
}