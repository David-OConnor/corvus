//! This module contains code for the Infineon DPS310 barometer.
//! This device supports both I2C and SPI interfaces.
//! For now, this driver assumes I2C, since that's what's used on the
//! Matek H7 slim board we're testing with.
//!
//! Note that both this and the ICM-42605 IMU read temperature.

use stm32_hal2::{i2c::I2c, pac::I2C2};

use defmt::println;

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

// Some awkward boundaries due to being packed. See datasheet section 8.11
const COEF_0: u8 = 0x10;
const COEF_0_1: u8 = 0x11;
const COEF_1: u8 = 0x12;
const COEF_00_A: u8 = 0x13;
const COEF_00_B: u8 = 0x14;
const COEF_00_10: u8 = 0x15;
const COEF_10_A: u8 = 0x16;
const COEF_10_B: u8 = 0x17;
const COEF_01_A: u8 = 0x18;
const COEF_01_B: u8 = 0x19;
const COEF_11_A: u8 = 0x1A;
const COEF_11_B: u8 = 0x1B;
const COEF_20_A: u8 = 0x1C;
const COEF_20_B: u8 = 0x1D;
const COEF_21_A: u8 = 0x1E;
const COEF_21_B: u8 = 0x1F;
const COEF_30_A: u8 = 0x20;
const COEF_30_B: u8 = 0x21;
// ...

// See datasheet, table 9. Used in pressure and temperature calculation. This is set up for 64-times
// oversampling rate.
const K_P: i32 = 1_040_384; // 64x oversample
const K_T: i32 = 1_040_384; // 64x oversample
                            // const SCALE_FACTOR: i32 = 2_088_960; // 128x oversample
                            // todo: 128 times for even more precision?
                            // todo: Result shift (bit 2 and 3 address 0x09, for use with this setting)

/// Fix the sign on signed 24 bit integers, represented as `i32`.
fn fix_i24_sign(val: &mut i32) {
    *val = (*val << 8) >> 8;
}

/// Used to map pressure to altitude. Can use a single point, in conjunction with a standard
/// atmosphere model.
pub struct AltitudeCalPt {
    pressure: f32, // Pa
    altitude: f32, // MSL, via GPS, in meters
    temp: f32,     // C
}

impl Default for AltitudeCalPt {
    /// Standard temperature and pressure.
    fn default() -> Self {
        Self {
            pressure: 101_325.,
            altitude: 0.,
            temp: 15.,
        }
    }
}

/// Calibration coefficients, read from factory-assigned registers.
/// 2's complement numbers.
struct PressureCal {
    // todo: These values have varying numbers of bits, but we use them with 24-bit readings.
    c0: i32,
    c1: i32,
    c00: i32,
    c10: i32,
    c20: i32,
    c30: i32,
    c01: i32,
    c11: i32,
    c21: i32,
}

pub struct Altimeter {
    altitude_cal: AltitudeCalPt,
    pressure_cal: PressureCal,
}

impl Altimeter {
    /// Configure settings, including pressure mreasurement rate, and return an instance.
    /// And load calibration data.
    pub fn new(i2c: &mut I2c<I2C2>, altitude_cal: AltitudeCalPt) -> Self {
        // Set 16 x oversampling, and 128 measurements per second
        // todo: Balance oversampling between time and precision
        i2c.write(ADDR, &[PRS_CFG, 0b0111_0100]).ok();

        // todo temp debug
        let mut prs_buf = [0; 10];
        i2c.write_read(ADDR, &[PRS_CFG], &mut prs_buf);
        println!("PRS CFG REG: {:?}", prs_buf);

        // Load calibration data, factory-coded.
        let mut buf_a = [0];
        let mut buf_b = [0];
        let mut buf_c = [0];

        // todo DRY. Split up into sub fns, at least for the last few that are uniform.

        i2c.write_read(ADDR, &[COEF_0], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_0_1], &mut buf_b).ok();

        let mut c0 = i16::from_be_bytes([buf_a[0], buf_b[0] >> 4]) as i32;

        i2c.write_read(ADDR, &[COEF_1], &mut buf_a).ok();

        let mut c1 = i16::from_be_bytes([buf_b[0] & 0xFFFF, buf_a[0]]) as i32;

        i2c.write_read(ADDR, &[COEF_00_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_00_B], &mut buf_b).ok();
        i2c.write_read(ADDR, &[COEF_00_10], &mut buf_c).ok();
        let mut c00 = i32::from_be_bytes([0, buf_a[0], buf_b[0], buf_c[0] >> 4]);

        i2c.write_read(ADDR, &[COEF_10_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_10_B], &mut buf_b).ok();
        let mut c10 = i32::from_be_bytes([0, buf_c[0] & 0xFFFF, buf_a[0], buf_b[0]]);

        i2c.write_read(ADDR, &[COEF_01_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_01_B], &mut buf_b).ok();
        let mut c01 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[COEF_11_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_11_B], &mut buf_b).ok();
        let mut c11 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[COEF_20_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_20_B], &mut buf_b).ok();
        let mut c20 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[COEF_21_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_21_B], &mut buf_b).ok();
        let mut c21 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[COEF_30_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_30_B], &mut buf_b).ok();
        let mut c30 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        fix_i24_sign(&mut c0);
        fix_i24_sign(&mut c1);
        fix_i24_sign(&mut c00);
        fix_i24_sign(&mut c10);
        fix_i24_sign(&mut c20);
        fix_i24_sign(&mut c30);
        fix_i24_sign(&mut c01);
        fix_i24_sign(&mut c11);
        fix_i24_sign(&mut c21);

        Self {
            altitude_cal,
            pressure_cal: PressureCal {
                c0,
                c1,
                c00,
                c10,
                c20,
                c30,
                c01,
                c11,
                c21,
            },
        }
    }

    pub fn setup(i2c: &mut I2c<I2C2>) {

        // todo: Do more config!
    }

    pub fn calibrate(&self, altitude: f32, i2c: &mut I2c<I2C2>) -> AltitudeCalPt {
        AltitudeCalPt {
            pressure: self.read_pressure(i2c),
            altitude,
            temp: self.read_temp(i2c),
        }
    }

    /// Apply compensation values from calibration coefficients to the pressure reading.
    /// Output is in Pa (todo is it?). Datasheet, section 4.9.1. We use naming conventions
    /// to match the DS.
    fn pressure_from_reading(&self, p_raw: i32, t_raw: i32) -> f32 {
        let t_raw_sc = t_raw / K_T;
        let p_raw_sc = p_raw / K_P;

        let cal = self.pressure_cal; // code shortener

        (cal.c00
            + p_raw_sc * (cal.c10 + p_raw_sc * (cal.c20 + p_raw_sc * cal.c30))
            + t_raw_sc * c01
            + t_raw_sc * p_raw_sc * (cal.c11 + p_raw_sc * cal.c21)) as f32
    }

    /// Datasheet, section 4.9.1
    fn temp_from_reading(&self, t_raw: i32) -> f32 {
        let t_raw_sc = t_raw / K_T;
        (self.cal.c0 * 0.5 * self.cal.c1 * t_raw_sc) as f32
    }

    /// Read atmospheric pressure, in kPa.
    pub fn read_pressure(&self, i2c: &mut I2c<I2C2>) -> f32 {
        // let reading = i2c.write_read()

        // The Pressure Data registers contains the 24 bit (3 bytes) 2's complement pressure measurement value.
        // If the FIFO is enabled, the register will contain the FIFO pressure and/or temperature results. Otherwise, the
        // register contains the pressure measurement results and will not be cleared after read.

        // todo: Do we need to do 3 separate reads here?
        let mut buf2 = [0];
        let mut buf1 = [0];
        let mut buf0 = [0];
        i2c.write_read(ADDR, &[PSR_B2], &mut buf2).ok();
        i2c.write_read(ADDR, &[PSR_B1], &mut buf1).ok();
        i2c.write_read(ADDR, &[PSR_B0], &mut buf0).ok();

        let mut reading = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_i24_sign(&mut reading);

        pressure_from_reading(reading)
    }

    // todo: temp read fn, similar to pressure read.
}

/// Interpret pressure as altitude. Pressure is in kPa.
pub fn pressure_to_alt(pressure: f32) -> f32 {
    // todo: If you use more than one baro driver, move this outside this module; it's a general fn.
    // P = 101.29 * (((T + 273.1))/288.08)^5.256   (in kPa)
    // T = 150.4 - .00649h

    0.
}
