//! This module contains code for the Infineon DPS310 barometer.
//! This device supports both I2C and SPI interfaces.
//! For now, this driver assumes I2C, since that's what's used on the
//! Matek H7 slim board we're testing with.
//!
//! Note that both this and the ICM-42605 IMU read temperature.

use stm32_hal2::{
    i2c::I2c,
    pac::{self, I2C2},
};

// The sensor's address is 0x77 (if SDO pin is left floating or pulled-up to VDDIO) or 0x76 (if the SDO pin is
// pulled-down to GND).
const ADDR: u8 = 0x77;

// See Datasheet, secction 7: Register Map
const PSR_B2: u8 = 0x00;
const PSR_B1: u8 = 0x01;
const PSR_B0: u8 = 0x02;
const TMP_B0: u8 = 0x03;
const TMP_B2: u8 = 0x04;
const TMP_B1: u8 = 0x05;
const PRS_CFG: u8 = 0x06;
const TMP_CFG: u8 = 0x07;
const MEAS_CFG: u8 = 0x08;
const CFG_REG: u8 = 0x09;
const INT_STS: u8 = 0x0A;
const FIFO_STS: u8 = 0x0B;
const RESET: u8 = 0x0C;

const COEF_1: u8 = 0x10;
const COEF_12: u8 = 0x11;
// ...

const COEF_SRCE: u8 = 0x28;

// todo: Should we use normal measurements, or FIFO? Probably FIFO via DMA?

/// Fix the sign on signed 24 bit integers, represented as `i32`.
fn fix_i24_sign(val: &mut i32) {
    *val = (*val << 8) >> 8;
}

/// Used to calibrate. Can use a single point, in conjunction with a standard
/// atmosphere model.
struct BaroCalPt {
    pressure: f32, // Pa
    altitude: f32, // MSL, via GPS, in meters
    temp: f32,     // C
}

impl Default for BaroCalPt {
    /// Standard temperature and pressure.
    fn default() -> Self {
        Self {
            pressure: 101_325.,
            altitude: 0.,
            temp: 15.,
        }
    }
}

pub struct Barometer {
    calibration: BaroCalPt,
}

impl Barometer {
    /// Configure settings, including pressure mreasurement rate, and return an instance.
    pub fn new(i2c: &mut I2c<I2C2>) -> Self {
        // Set 16 x oversampling, and 128 measurements per second
        // todo: Balance oversampling between time and precision
        i2c.write(ADDR, &[PRS_CFG, 0b0111_0100]);

        // todo: Do more config!

        let mut result = Self {
            calibration: Default::default(),
        };

        result
    }

    pub fn calibrate(&mut self, altitude: f32, temp: f32, i2c: &mut I2c<I2C2>) {
        self.calibration = BaroCalPt {
            pressure: self.read(i2c),
            altitude,
            temp,
        }
    }

    /// Read atmospheric pressure, in kPa.
    pub fn read(&mut self, i2c: &mut I2c<I2C2>) -> f32 {
        // todo: FIFO instead?

        // let reading = i2c.write_read()

        // The Pressure Data registers contains the 24 bit (3 bytes) 2's complement pressure measurement value.
        // If the FIFO is enabled, the register will contain the FIFO pressure and/or temperature results. Otherwise, the
        // register contains the pressure measurement results and will not be cleared after read.

        // todo: Do we need to do 3 separate reads here?
        let mut buf2 = [0];
        let mut buf1 = [0];
        let mut buf0 = [0];
        i2c.write_read(ADDR, &[PSR_B2], &mut buf2);
        i2c.write_read(ADDR, &[PSR_B1], &mut buf1);
        i2c.write_read(ADDR, &[PSR_B0], &mut buf0);

        let mut result = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_i24_sign(&mut result);

        result as f32 // todo: Is this right?
    }
}

// todo: temp read fn, similar to pressure read? Or use the IMU?

/// Interpret pressure as altitude. Pressure is in kPa.
pub fn pressure_to_alt(pressure: f32) -> f32 {
    // todo: If you use more than one baro driver, move this outside this module; it's a general fn.
    // P = 101.29 * (((T + 273.1))/288.08)^5.256   (in kPa)
    // T = 150.4 - .00649h

    0.
}
