//! This module contains code for the Infineon DPS310 barometer.
//! This device supports both I2C and SPI interfaces.
//! For now, this driver assumes I2C.
//!
//! Measurement rate: 128Hz.
//!
//! Note that both this and the ICM-42605 IMU read temperature.

#![allow(dead_code)]

use crate::{
    atmos_model::{AltitudeCalPt, POINT_0, POINT_1},
    util,
};

use stm32_hal2::{
    i2c::{self, I2c},
    pac::I2C2,
};

use defmt::println;

// The sensor's address is 0x77 (if SDO pin is left floating or pulled-up to VDDIO) or 0x76 (if the SDO pin is
// pulled-down to GND).
const ADDR: u8 = 0x77;

pub struct BaroNotConnectedError {}

impl From<i2c::Error> for BaroNotConnectedError {
    fn from(e: i2c::Error) -> Self {
        Self {}
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    PsrB2 = 0x00,
    PsrB1 = 0x01,
    PsrB0 = 0x02,
    TmpB0 = 0x03,
    TmpB2 = 0x04,
    TmpB1 = 0x05,
    PrsCfg = 0x06,
    TmpCfg = 0x07,
    MeasCfg = 0x08,
    CfgReg = 0x09,
    InitSts = 0x0A,
    FifoSts = 0x0B,
    Reset = 0x0C,

    // Some awkward boundaries due to being packed. See datasheet section 8.11
    Coef0 = 0x10,
    Coef01 = 0x11,
    Coef1 = 0x12,
    Coef00A = 0x13,
    Coef00B = 0x14,
    Coef0010 = 0x15,
    Coef10A = 0x16,
    Coef10B = 0x17,
    Coef01A = 0x18,
    Coef01B = 0x19,
    Coef11A = 0x1A,
    Coef11B = 0x1B,
    Coef20A = 0x1C,
    Coef20B = 0x1D,
    Coef21A = 0x1E,
    Coef21B = 0x1F,
    Coef30A = 0x20,
    Coef30B = 0x21,
}

// See Datasheet, secction 7: Register Map

// ...

// See datasheet, table 9. Used in pressure and temperature calculation. This is set up for 64-times
// oversampling rate.
const K_P: i32 = 1_040_384; // 64x oversample
const K_T: i32 = 1_040_384; // 64x oversample
                            // const SCALE_FACTOR: i32 = 2_088_960; // 128x oversample
                            // todo: 128 times for even more precision?
                            // todo: Result shift (bit 2 and 3 address 0x09, for use with this setting)

/// Fix the sign on signed 24 bit integers, represented as `i32`. (Here, we use this for pressure
/// and temp readings)
fn fix_int_sign(val: &mut i32, num_bits: u8) {
    let diff = 32 - num_bits as i32;
    *val = (*val << diff) >> diff;
}

/// Calibration coefficients, read from factory-assigned registers.
/// 2's complement numbers.
#[derive(Default)]
struct HardwareCoeffCal {
    // todo: These values have varying numbers of bits, but we use them with 24-bit readings.
    c0: i32,
    c1: i32,
    c00: i32,
    c10: i32,
    c01: i32,
    c11: i32,
    c20: i32,
    c30: i32,
    c21: i32,
}

/// Represents an Altimeter, storing several types of calibration values. Of note, this is mostly hardware-agnostic,
/// but is in this hardware-specific module due to the factory-stored calibration coeffs.
/// The default impl is used if the baro isn't connected, or there's a communciations error.
#[derive(Default)]
pub struct Altimeter {
    /// Calibration point taken during initialization; used to interpret pressure readings
    /// as pseudo-AGL. (QFE) This point's altitude field must be 0.
    ground_cal: AltitudeCalPt,
    /// Calibration point taken during initialization; Stores measured pressure and temperature
    /// at the GPS's reported MSL alt.
    /// todo: Currently GPS cal is unused.
    gps_cal_init: Option<AltitudeCalPt>,
    gps_cal_air: Option<AltitudeCalPt>,
    hardware_coeff_cal: HardwareCoeffCal,
}

// todo: Consider adding a second and/or 3rd gps cal point based on GPS reporting.

impl Altimeter {
    /// Configure settings, including pressure mreasurement rate, and return an instance.
    /// And load calibration data.
    pub fn new(i2c: &mut I2c<I2C2>) -> Result<Self, BaroNotConnectedError> {
        // todo temp
        // return Ok(Self {
        //     ground_cal: Default::default(),
        //     gps_cal_init: None,
        //     gps_cal_air: None,
        //     hardware_coeff_cal: Default::default(),
        // });

        // Set 64x oversampling, and 128 measurements per second, for both temp and pres.
        i2c.write(ADDR, &[Reg::PrsCfg as u8, 0b0111_0110])?;
        i2c.write(ADDR, &[Reg::TmpCfg as u8, 0b0111_0110])?;

        // Continuous pressure and temp measurement in background mode.
        i2c.write(ADDR, &[Reg::MeasCfg as u8, 0b0000_0111])?;

        // Enable pressure and temp bit-shift due to our high sampling rate.
        // No interrupts enabled.
        i2c.write(ADDR, &[Reg::CfgReg as u8, 0b0000_1100])?;

        // todo temp debug
        let mut prs_buf = [0];
        i2c.write_read(ADDR, &[Reg::PrsCfg as u8], &mut prs_buf)?;
        println!("PRS CFG REG: {:b}", prs_buf[0]);

        i2c.write_read(ADDR, &[Reg::MeasCfg as u8], &mut prs_buf)?;
        println!("Meas REG: {:b}", prs_buf[0]);

        // Load calibration data, factory-coded.
        // Wait until the coefficients are ready to read. Also, wait until readings are ready
        // here as well.
        loop {
            let mut buf = [0];
            i2c.write_read(ADDR, &[Reg::MeasCfg as u8], &mut buf)?;
            if (buf[0] & 0b1100_0000) == 0b1100_0000 {
                break;
            }
        }

        let mut buf_a = [0];
        let mut buf_b = [0];
        let mut buf_c = [0];

        // todo DRY. Split up into sub fns, at least for the last few that are uniform.

        i2c.write_read(ADDR, &[Reg::Coef0 as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef01 as u8], &mut buf_b)?;

        let mut c0 = i16::from_be_bytes([buf_a[0], buf_b[0] >> 4]) as i32;

        i2c.write_read(ADDR, &[Reg::Coef1 as u8], &mut buf_a)?;

        let mut c1 = i16::from_be_bytes([buf_b[0] & 0xF, buf_a[0]]) as i32;

        i2c.write_read(ADDR, &[Reg::Coef00A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef00B as u8], &mut buf_b)?;
        i2c.write_read(ADDR, &[Reg::Coef0010 as u8], &mut buf_c)?;
        let mut c00 = i32::from_be_bytes([0, buf_a[0], buf_b[0], buf_c[0] >> 4]);

        i2c.write_read(ADDR, &[Reg::Coef10A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef10B as u8], &mut buf_b)?;
        let mut c10 = i32::from_be_bytes([0, buf_c[0] & 0xF, buf_a[0], buf_b[0]]);

        i2c.write_read(ADDR, &[Reg::Coef01A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef01B as u8], &mut buf_b)?;
        let mut c01 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[Reg::Coef11A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef11B as u8], &mut buf_b)?;
        let mut c11 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[Reg::Coef20A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef20B as u8], &mut buf_b)?;
        let mut c20 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[Reg::Coef21A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef21B as u8], &mut buf_b)?;
        let mut c21 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        i2c.write_read(ADDR, &[Reg::Coef30A as u8], &mut buf_a)?;
        i2c.write_read(ADDR, &[Reg::Coef30B as u8], &mut buf_b)?;
        let mut c30 = i16::from_be_bytes([buf_a[0], buf_b[0]]) as i32;

        // c0 and c1 are 12 bits. c00 and c10 are 20 bits. The rest are 16.
        // All are 2's complement.
        fix_int_sign(&mut c0, 12);
        fix_int_sign(&mut c1, 12);
        fix_int_sign(&mut c00, 20);
        fix_int_sign(&mut c10, 20);

        // println!("C0 {} C1 {} C10 {} C20 {} C30 {} C01 {} C11 {} C21 {} C30 {}", c0, c1, c10, c20, c30, c01, c11, c21, c30);

        let mut result = Self {
            ground_cal: Default::default(),
            gps_cal_init: None,
            gps_cal_air: None,
            hardware_coeff_cal: HardwareCoeffCal {
                c0,
                c1,
                c00,
                c10,
                c01,
                c11,
                c20,
                c21,
                c30,
            },
        };

        result.ground_cal = AltitudeCalPt {
            pressure: result.read_pressure(i2c).unwrap_or(0.),
            altitude: 0.,
            temp: result.read_temp(i2c).unwrap_or(0.),
        };

        Ok(result)
    }

    pub fn calibrate_from_gps(&mut self, gps_alt: Option<f32>, i2c: &mut I2c<I2C2>) {
        let pressure = self.read_pressure(i2c).unwrap_or(0.);
        let temp = self.read_temp(i2c).unwrap_or(0.);

        self.ground_cal = AltitudeCalPt {
            pressure,
            altitude: 0.,
            temp,
        };

        self.gps_cal_air = match gps_alt {
            Some(alt_msl) => Some(AltitudeCalPt {
                pressure,
                altitude: alt_msl,
                temp, // todo: Convert to K if required!
            }),
            None => None,
        };
    }

    /// Apply compensation values from calibration coefficients to the pressure reading.
    /// Output is in Pa (todo is it?). Datasheet, section 4.9.1. We use naming conventions
    /// to match the DS.
    fn pressure_from_reading(&self, p_raw: i32, t_raw: i32) -> f32 {
        let t_raw_sc = t_raw / K_T;
        let p_raw_sc = p_raw / K_P;

        let cal = &self.hardware_coeff_cal; // code shortener

        (cal.c00
            + p_raw_sc * (cal.c10 + p_raw_sc * (cal.c20 + p_raw_sc * cal.c30))
            + t_raw_sc * cal.c01
            + t_raw_sc * p_raw_sc * (cal.c11 + p_raw_sc * cal.c21)) as f32
    }

    /// Datasheet, section 4.9.1
    fn temp_from_reading(&self, t_raw: i32) -> f32 {
        let t_raw_sc = t_raw / K_T;
        // (self.pressure_cal.c0 * 0.5 * self.pressure_call.c1 * t_raw_sc) as f32
        // todo: Should we be doing these operations (Here and above in pressure-from_reading
        // todo as floats, and storing the c vals as floats?

        // todo: Is this C or K?

        (self.hardware_coeff_cal.c0 / 2 * self.hardware_coeff_cal.c1 * t_raw_sc) as f32
    }

    // todo: Given you use temp readings to feed into pressure, combine somehow to reduce reading
    // todo and computation.

    /// Read atmospheric pressure, in kPa.
    pub fn read_pressure(&self, i2c: &mut I2c<I2C2>) -> Result<f32, BaroNotConnectedError> {
        // The Pressure Data registers contains the 24 bit (3 bytes) 2's complement pressure measurement value.
        // If the FIFO is enabled, the register will contain the FIFO pressure and/or temperature results. Otherwise, the
        // register contains the pressure measurement results and will not be cleared after read.

        let mut buf2 = [0];
        let mut buf1 = [0];
        let mut buf0 = [0];
        i2c.write_read(ADDR, &[Reg::PsrB2 as u8], &mut buf2)?;
        i2c.write_read(ADDR, &[Reg::PsrB1 as u8], &mut buf1)?;
        i2c.write_read(ADDR, &[Reg::PsrB0 as u8], &mut buf0)?;

        let mut p_raw = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_int_sign(&mut p_raw, 24);

        // todo: DRY with above and temp readings below, also does this twice if getting temp at same time.
        i2c.write_read(ADDR, &[Reg::TmpB2 as u8], &mut buf2)?;
        i2c.write_read(ADDR, &[Reg::TmpB1 as u8], &mut buf1)?;
        i2c.write_read(ADDR, &[Reg::TmpB0 as u8], &mut buf0)?;

        let mut t_raw = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_int_sign(&mut t_raw, 24);

        Ok(self.pressure_from_reading(p_raw, t_raw))
    }

    /// Read temperature, in °C.
    pub fn read_temp(&self, i2c: &mut I2c<I2C2>) -> Result<f32, BaroNotConnectedError> {
        // The Temperature Data registers contain the 24 bit (3 bytes) 2's complement temperature measurement value
        // ( unless the FIFO is enabled, please see FIFO operation ) and will not be cleared after the read.

        // todo: DRY with pressure read

        let mut buf2 = [0];
        let mut buf1 = [0];
        let mut buf0 = [0];
        i2c.write_read(ADDR, &[Reg::TmpB2 as u8], &mut buf2)?;
        i2c.write_read(ADDR, &[Reg::TmpB1 as u8], &mut buf1)?;
        i2c.write_read(ADDR, &[Reg::TmpB0 as u8], &mut buf0)?;

        let mut reading = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_int_sign(&mut reading, 24);

        Ok(self.temp_from_reading(reading))
    }
}
