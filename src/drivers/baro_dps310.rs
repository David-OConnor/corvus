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
    pac::{I2C1, I2C2},
};

use defmt::println;

// The sensor's address is 0x77 (if SDO pin is left floating or pulled-up to VDDIO) or 0x76 (if the SDO pin is
// pulled-down to GND).
pub const ADDR: u8 = 0x76; // todo: Switch back to 0x77 A/R
pub const PRODUCT_ID: u8 = 0x10;

type I2C = I2c<I2C1>; // todo temp 1

pub struct BaroNotConnectedError {}

impl From<i2c::Error> for BaroNotConnectedError {
    fn from(_: i2c::Error) -> Self {
        Self {}
    }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    PsrB2 = 0x00,
    PsrB1 = 0x01,
    PsrB0 = 0x02,
    TmpB2 = 0x03,
    TmpB1 = 0x04,
    TmpB0 = 0x05,
    PrsCfg = 0x06,
    TmpCfg = 0x07,
    MeasCfg = 0x08,
    CfgReg = 0x09,
    InitSts = 0x0A,
    FifoSts = 0x0B,
    Reset = 0x0C,
    ProductId = 0x0d,

    // Calibration coefficients
    // Some awkward boundaries due to being packed. See datasheet section 8.11
    c0 = 0x10,
    c0_c1 = 0x11,
    c1 = 0x12,
    c00_a = 0x13,
    c00_b = 0x14,
    c00_c10 = 0x15,
    c10_a = 0x16,
    c10_b = 0x17,
    c01_a = 0x18,
    c01_b = 0x19,
    c11_a = 0x1A,
    c11_b = 0x1B,
    c20_a = 0x1C,
    c20_b = 0x1D,
    c21_a = 0x1E,
    c21_b = 0x1F,
    c30_a = 0x20,
    c30_b = 0x21,
}

// See Datasheet, secction 7: Register Map

// ...

// See datasheet, table 9. Used in pressure and temperature calculation. This is set up for 64-times
// oversampling rate.
const K_P: f32 = 1_040_384.; // 64x oversample
const K_T: f32 = 1_040_384.; // 64x oversample
// const SCALE_FACTOR: i32 = 2_088_960; // 128x oversample
// todo: 128 times for even more precision?
// todo: Result shift (bit 2 and 3 address 0x09, for use with this setting)

/// Fix the sign on signed 24 bit integers, represented as `i32`. (Here, we use this for pressure
/// and temp readings)
fn fix_int_sign(val: &mut i32, num_bits: u8) {
    let diff = 32 - num_bits as i32;
    *val = (*val << diff) >> diff;
}

/// Utility function to read a single byte.
fn read_one(reg: Reg, i2c: &mut I2C) -> Result<u8, BaroNotConnectedError> {
    let mut buf = [0];
    i2c.write_read(ADDR, &[reg as u8], &mut buf)?;
    Ok(buf[0])
}

/// Calibration coefficients, read from factory-assigned registers.
/// 2's complement numbers.
#[derive(Default)]
struct HardwareCoeffCal {
    // These values have varying numbers of bits, but we use them with 24-bit readings.
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

impl HardwareCoeffCal {
    pub fn new(i2c: &mut I2C) -> Result<Self, BaroNotConnectedError> {
        // Read each register value.
        let c0 = read_one(Reg::c0, i2c)?;
        let c0_c1 = read_one(Reg::c0_c1, i2c)?;
        let c1 = read_one(Reg::c1, i2c)?;
        let c00_a = read_one(Reg::c00_a, i2c)?;
        let c00_b = read_one(Reg::c00_b, i2c)?;
        let c00_c10 = read_one(Reg::c00_c10, i2c)?;
        let c10_a = read_one(Reg::c10_a, i2c)?;
        let c10_b = read_one(Reg::c10_b, i2c)?;
        let c01_a = read_one(Reg::c01_a, i2c)?;
        let c01_b = read_one(Reg::c01_b, i2c)?;
        let c11_a = read_one(Reg::c11_a, i2c)?;
        let c11_b = read_one(Reg::c11_b, i2c)?;
        let c20_a = read_one(Reg::c20_a, i2c)?;
        let c20_b = read_one(Reg::c20_b, i2c)?;
        let c21_a = read_one(Reg::c21_a, i2c)?;
        let c21_b = read_one(Reg::c21_b, i2c)?;
        let c30_a = read_one(Reg::c30_a, i2c)?;
        let c30_b = read_one(Reg::c30_b, i2c)?;

        // Unpack into coefficients.
        let mut c0 = i16::from_be_bytes([c0, c0_c1 >> 4]) as i32;
        let mut c1 = i16::from_be_bytes([c1, c0_c1 & 0xf]) as i32;
        let mut c00 = i32::from_be_bytes([0, c00_a, c00_b, c00_c10 >> 4]);
        let mut c10 = i32::from_be_bytes([0, c00_c10 & 0xf, c10_a, c10_b]);
        let c01 = i16::from_be_bytes([c01_a, c01_b]) as i32;
        let c11 = i16::from_be_bytes([c11_a, c11_b]) as i32;
        let c20 = i16::from_be_bytes([c20_a, c20_b]) as i32;
        let c21 = i16::from_be_bytes([c21_a, c21_b]) as i32;
        let c30 = i16::from_be_bytes([c30_a, c30_b]) as i32;

        // c0 and c1 are 12 bits. c00 and c10 are 20 bits. The rest are 16.
        // All are 2's complement.
        fix_int_sign(&mut c0, 12);
        fix_int_sign(&mut c1, 12);
        fix_int_sign(&mut c00, 20);
        fix_int_sign(&mut c10, 20);

        // println!("C0 {} C1 {} C10 {} C20 {} C30 {} C01 {} C11 {} C21 {} C30 {}", c0, c1, c10, c20, c30, c01, c11, c21, c30);

        Ok(Self {
            c0,
            c1,
            c00,
            c10,
            c01,
            c11,
            c20,
            c21,
            c30,
        })
    }
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

impl Altimeter {
    /// Configure settings, including pressure mreasurement rate, and return an instance.
    /// And load calibration data.
    pub fn new(i2c: &mut I2C) -> Result<Self, BaroNotConnectedError> { // todo 1 temp
        let mut read_buf = [0];
        i2c.write_read(ADDR, &[Reg::ProductId as u8], &mut read_buf);

        if read_buf[0] != PRODUCT_ID {
            return Err(BaroNotConnectedError {});
        }

        // Set 64x oversampling, and 32 measurements per second, for both temp and pres.
        i2c.write(ADDR, &[Reg::PrsCfg as u8, 0b0101_0110])?;
        i2c.write(ADDR, &[Reg::TmpCfg as u8, 0b0101_0110])?;

        // Continuous pressure and temp measurement in background mode.
        i2c.write(ADDR, &[Reg::MeasCfg as u8, 0b0000_0111])?;

        // Enable pressure and temp bit-shift due to our high sampling rate.
        // No interrupts enabled.
        i2c.write(ADDR, &[Reg::CfgReg as u8, 0b0000_1100])?;

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

        let mut result = Self {
            ground_cal: Default::default(),
            gps_cal_init: None,
            gps_cal_air: None,
            hardware_coeff_cal: HardwareCoeffCal::new(i2c)?,
        };

        result.ground_cal = AltitudeCalPt {
            pressure: result.read_pressure(i2c).unwrap_or(0.),
            altitude: 0.,
            temp: result.read_temp(i2c).unwrap_or(0.),
        };

        Ok(result)
    }

    pub fn calibrate_from_gps(&mut self, gps_alt: Option<f32>, i2c: &mut I2C) {
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
    fn pressure_from_raw(&self, p_raw: i32, t_raw: i32) -> f32 {
        // let p_raw_sc = p_raw / K_P;
        // let t_raw_sc = t_raw / K_T;
        // 
        // let cal = &self.hardware_coeff_cal; // code shortener
        // 
        // println!("CAL: {:?} {} {} {} {}", cal.c00, cal.c20, cal.c30, cal.c01, cal.c10);
        // 
        // println!("P raw: {}, T RAW: {}", p_raw, t_raw);
        // println!("P raw sc: {}, T RAW sc: {}", p_raw_sc, t_raw_sc);
        // 
        // println!("A: {} B: {} C: {}", p_raw_sc * (cal.c10 + p_raw_sc * (cal.c20 + p_raw_sc * cal.c30)), t_raw_sc * cal.c01, t_raw_sc * p_raw_sc * (cal.c11 + p_raw_sc * cal.c21));
        // 
        // (cal.c00
        //     + p_raw_sc * (cal.c10 + p_raw_sc * (cal.c20 + p_raw_sc * cal.c30))
        //     + t_raw_sc * cal.c01
        //     + t_raw_sc * p_raw_sc * (cal.c11 + p_raw_sc * cal.c21)) as f32
        
        // todo: With floats
        let p_raw_sc = p_raw as f32 / K_P;
        let t_raw_sc = t_raw as f32 / K_T;

        let cal = &self.hardware_coeff_cal; // code shortener

        // println!("CAL: {:?} {} {} {} {}", cal.c00, cal.c20, cal.c30, cal.c01, cal.c10);
        // 
        // println!("P raw: {}, T RAW: {}", p_raw, t_raw);
        // println!("P raw sc: {}, T RAW sc: {}", p_raw_sc, t_raw_sc);
        // 
        // println!("A: {} B: {} C: {}", p_raw_sc * (cal.c10 + p_raw_sc * (cal.c20 + p_raw_sc * cal.c30)), t_raw_sc * cal.c01, t_raw_sc * p_raw_sc * (cal.c11 + p_raw_sc * cal.c21));

        cal.c00 as f32
            + p_raw_sc * (cal.c10 as f32 + p_raw_sc * (cal.c20 as f32 + p_raw_sc * cal.c30 as f32))
            + t_raw_sc * cal.c01 as f32
            + t_raw_sc * p_raw_sc * (cal.c11 as f32 + p_raw_sc * cal.c21 as f32)

    }

    /// Datasheet, section 4.9.1
    fn temp_from_raw(&self, t_raw: i32) -> f32 {
        let t_raw_sc = t_raw as f32 / K_T;
        // (self.pressure_cal.c0 * 0.5 * self.pressure_call.c1 * t_raw_sc) as f32
        // todo: Should we be doing these operations (Here and above in pressure-from_reading
        // todo as floats, and storing the c vals as floats?

        // todo: Is this C or K?

        (self.hardware_coeff_cal.c0 as f32 * 0.5 * self.hardware_coeff_cal.c1 as f32 * t_raw_sc) as f32
    }

    // todo: Given you use temp readings to feed into pressure, combine somehow to reduce reading
    // todo and computation.

    /// Given readings taken from registers directly, calcualte pressure.
    // pub fn pressure_from_readings(&self, p2: u8, p1: u8, p0: u8, t2: u8, t1: u8, t0: u8) -> f32 {
    pub fn pressure_from_readings(&self, buf: &[u8]) -> f32 {
        let p2 = buf[0];
        let p1 = buf[1];
        let p0 = buf[2];
        let t2 = buf[3];
        let t1 = buf[4];
        let t0 = buf[5];

        let mut p_raw = i32::from_be_bytes([0, p2, p1, p0]);
        fix_int_sign(&mut p_raw, 24);

        let mut t_raw = i32::from_be_bytes([0, t2, t1, t0]);
        // println!("t RAW pre: {:b}", t_raw);
        fix_int_sign(&mut t_raw, 24);
        // println!("t RAW post: {:b}", t_raw);

        self.pressure_from_raw(p_raw, t_raw)
    }

    /// Read atmospheric pressure, in kPa.
    pub fn read_pressure(&self, i2c: &mut I2C) -> Result<f32, BaroNotConnectedError> {
        // The Pressure Data registers contains the 24 bit (3 bytes) 2's complement pressure measurement value.
        // If the FIFO is enabled, the register will contain the FIFO pressure and/or temperature results. Otherwise, the
        // register contains the pressure measurement results and will not be cleared after read.

        // 3 pressure readings; 3 temp readings. The sensor auto-increments the register; these
        // are sequentially P2, P1, P0, T2, T1, T0.
        let mut buf = [0; 6];
        i2c.write_read(ADDR, &[Reg::PsrB2 as u8], &mut buf)?;

        Ok(self.pressure_from_readings(&buf))
    }

    /// Read temperature, in °C.
    pub fn read_temp(&self, i2c: &mut I2C) -> Result<f32, BaroNotConnectedError> {
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

        Ok(self.temp_from_raw(reading))
    }
}
