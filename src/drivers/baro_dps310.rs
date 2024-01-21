//! This module contains code for the Infineon DPS310 barometer.
//! This device supports both I2C and SPI interfaces.
//! For now, this driver assumes I2C.
//!
//! Measurement rate: 128Hz.
//!
//! Note that both this and the ICM-42605 IMU read temperature.

#![allow(dead_code)]

use hal::{delay_ms, i2c};

use crate::{atmos_model::AltitudeCalPt, board_config::AHB_FREQ, setup::I2cBaro};

// The sensor's address is 0x77 (if SDO pin is left floating or pulled-up to VDDIO) or 0x76 (if the SDO pin is
// pulled-down to GND).
pub const ADDR: u8 = 0x77;
pub const PRODUCT_ID: u8 = 0x10;

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

// See datasheet, table 9. Used in pressure and temperature calculation. This is set up for 64-times
// oversampling rate.
const K_P: f32 = 1_040_384.; // 64x oversample
const K_T: f32 = 1_040_384.; // 64x oversample

/// Fix the sign on signed 24-bit and 12-bit integers, represented as `i32`.
/// See section 8.11 in the datasheet for the algorithm.
/// (Here, we use this for pressure and temp readings)
fn fix_int_sign(val: &mut i32, num_bits: u8) {
    if *val > (1 << (num_bits - 1)) - 1 {
        *val -= 1 << (num_bits as i32);
    }
}

/// Utility function to read a single byte.
fn read_one(reg: Reg, i2c: &mut I2cBaro) -> Result<u8, BaroNotConnectedError> {
    let mut buf = [0];
    i2c.write_read(ADDR, &[reg as u8], &mut buf)?;
    // println!("Baro Buf: {:?}", buf);
    Ok(buf[0])
}

/// Calibration coefficients, read from factory-assigned registers.
/// 2's complement numbers.
/// We store these as floats, since that's how they're used in operations.
#[derive(Default)]
struct HardwareCoeffCal {
    // c0: f32,  // 12 bits
    // c1: f32,  // 12 bits
    // c00: f32, // 20 bits
    // c10: f32, // 20 bits
    // c01: f32, // 16 bits for the rest.
    // c11: f32,
    // c20: f32,
    // c30: f32,
    // c21: f32,
    c0: i16,  // 12 bits
    c1: i16,  // 12 bits
    c00: i32, // 20 bits
    c10: i32, // 20 bits
    c01: i16, // 16 bits for the rest.
    c11: i16,
    c20: i16,
    c30: i16,
    c21: i16,
}

impl HardwareCoeffCal {
    /// See table 8.11
    pub fn new(i2c: &mut I2cBaro) -> Result<Self, BaroNotConnectedError> {
        let mut buf = [0; 18];

        i2c.write_read(ADDR, &[Reg::c0 as u8], &mut buf)?;

        // Unpack into coefficients. See datasheet Table 18.
        let mut c0 = ((buf[0] as i32) << 4) | (buf[1] as i32 >> 4);
        let mut c1 = (((buf[1] as i32) & 0xf) << 8) | (buf[2] as i32);

        let mut c00 = ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);
        let mut c10 = i32::from_be_bytes([0, buf[5] & 0xf, buf[6], buf[7]]);

        let c01 = i16::from_be_bytes([buf[8], buf[9]]);
        let c11 = i16::from_be_bytes([buf[10], buf[11]]);
        let c20 = i16::from_be_bytes([buf[12], buf[13]]);
        let c21 = i16::from_be_bytes([buf[14], buf[15]]);
        let c30 = i16::from_be_bytes([buf[16], buf[17]]);

        // c0 and c1 are 12 bits. c00 and c10 are 20 bits. The rest are 16.
        // All are 2's complement.
        fix_int_sign(&mut c0, 12);
        fix_int_sign(&mut c1, 12);

        fix_int_sign(&mut c00, 20);
        fix_int_sign(&mut c10, 20);

        Ok(Self {
            c0: c0 as i16,
            c1: c1 as i16,
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
    pub ground_cal: AltitudeCalPt,
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
    pub fn new(i2c: &mut I2cBaro) -> Result<Self, BaroNotConnectedError> {
        if read_one(Reg::ProductId, i2c)? != PRODUCT_ID {
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
        // here as well prior to our base point initialization.
        // Datasheet: It takes a max of 40ms for coefficients to be ready. From a fortuitious bug in
        // Betaflight, we verify 20ms isn't long enough.
        loop {
            if (read_one(Reg::MeasCfg, i2c)? & 0b1100_0000) == 0b1100_0000 {
                break;
            }
        }

        let mut result = Self {
            ground_cal: Default::default(), // initialized elsewhere.
            gps_cal_init: None,
            gps_cal_air: None,
            hardware_coeff_cal: HardwareCoeffCal::new(i2c)?,
        };

        // Ground initialization.

        // It appears we need a sizable delay between setup and the first reading to set up the base
        // pressure. 300ms seems to work reliably. 100ms doesn't.
        delay_ms(300, AHB_FREQ);

        let (mut pressure, mut temp) = result.read_pressure_temp(i2c)?;

        result.ground_cal = AltitudeCalPt {
            pressure,
            altitude: 0., // QFE
            temp,
        };

        Ok(result)
    }

    pub fn calibrate_from_gps(
        &mut self,
        gps_alt: Option<f32>,
        i2c: &mut I2cBaro,
    ) -> Result<(), BaroNotConnectedError> {
        let (pressure, temp) = self.read_pressure_temp(i2c)?;

        self.ground_cal = AltitudeCalPt {
            pressure,
            altitude: 0.,
            temp,
        };

        self.gps_cal_air = gps_alt.map(|alt_msl| AltitudeCalPt {
            pressure,
            altitude: alt_msl,
            temp, // todo: Convert to K if required!
        });

        Ok(())
    }

    /// Apply compensation values from calibration coefficients to the pressure reading.
    /// Output is in Pascals. Datasheet, section 4.9.1. We use naming conventions
    /// to match the DS.
    fn pressure_from_raw(&self, p_raw_sc: f32, t_raw_sc: f32) -> f32 {
        let cal = &self.hardware_coeff_cal; // code shortener

        cal.c00 as f32
            + p_raw_sc * (cal.c10 as f32 + p_raw_sc * (cal.c20 as f32 + p_raw_sc * cal.c30 as f32))
            + t_raw_sc * cal.c01 as f32
            + t_raw_sc * p_raw_sc * (cal.c11 as f32 + p_raw_sc * cal.c21 as f32)
    }

    /// Datasheet, section 4.9.2. Returns temperature in K.
    /// Assumes we've calculated `t_raw_sc` already as part of our pressure reading.
    fn temp_from_raw_sc(&self, t_raw_sc: f32) -> f32 {
        (self.hardware_coeff_cal.c0 as f32 * 0.5 + self.hardware_coeff_cal.c1 as f32 * t_raw_sc)
            + 273.15
        // (self.hardware_coeff_cal.c0 as f32 * 0.5 + self.hardware_coeff_cal.c1 as f32 * t_raw_sc) * 100.
    }

    /// Given readings taken from registers directly, calcualte pressure.
    /// We split this from the other functions for use with DMA.
    pub fn pressure_temp_from_readings(&self, buf: &[u8; 6]) -> (f32, f32) {
        let [p2, p1, p0, t2, t1, t0] = *buf;

        let mut p_raw = i32::from_be_bytes([0, p2, p1, p0]);
        fix_int_sign(&mut p_raw, 24);

        let mut t_raw = i32::from_be_bytes([0, t2, t1, t0]);
        fix_int_sign(&mut t_raw, 24);

        let p_raw_sc = p_raw as f32 / K_P;
        let t_raw_sc = t_raw as f32 / K_T;

        (
            self.pressure_from_raw(p_raw_sc, t_raw_sc),
            self.temp_from_raw_sc(t_raw_sc),
        )
    }

    // todo: Given you use temp readings to feed into pressure, combine somehow to reduce reading
    // todo and computation.

    /// Read atmospheric pressure, in Pascals., and temperature, in Kelvin.
    /// This is our main non-DMA API. Note that we read them together, since we need both
    /// to estimate altitude.
    ///
    /// Note: We don't use this function in practice after init; we use DMA instead to populate the
    /// buffer.
    pub fn read_pressure_temp(
        &self,
        i2c: &mut I2cBaro,
    ) -> Result<(f32, f32), BaroNotConnectedError> {
        // The Pressure Data registers contains the 24 bit (3 bytes) 2's complement pressure measurement value.
        // If the FIFO is enabled, the register will contain the FIFO pressure and/or temperature results. Otherwise, the
        // register contains the pressure measurement results and will not be cleared after read.

        // 3 pressure readings; 3 temp readings. The sensor auto-increments the register; these
        // are sequentially P2, P1, P0, T2, T1, T0.
        let mut buf = [0; 6];
        i2c.write_read(ADDR, &[Reg::PsrB2 as u8], &mut buf)?;

        Ok(self.pressure_temp_from_readings(&buf))
    }
}
