//! This module contains code for the Infineon DPS310 barometer.
//! This device supports both I2C and SPI interfaces.
//! For now, this driver assumes I2C, since that's what's used on the
//! Matek H7 slim board we're testing with.
//!
//! Note that both this and the ICM-42605 IMU read temperature.

use crate::{
    atmos_model::{AltitudeCalPt, POINT_0, POINT_1},
    util,
};

use stm32_hal2::{i2c::I2c, pac::I2C2};

use defmt::println;

use num_traits::Float; // power.

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
pub struct Altimeter {
    /// Calibration point taken during initialization; used to interpret pressure readings
    /// as pseudo-AGL. This point's altitude field must be 0.
    ground_cal: AltitudeCalPt,
    /// Calibration point taken during initialization; Stores measured pressure and temperature
    /// at the GPS's reported MSL alt.
    gps_cal_init: Option<AltitudeCalPt>,
    gps_cal_air: Option<AltitudeCalPt>,
    hardware_coeff_cal: HardwareCoeffCal,
}

// todo: Consider adding a second and/or 3rd gps cal point based on GPS reporting.

impl Altimeter {
    /// Configure settings, including pressure mreasurement rate, and return an instance.
    /// And load calibration data.
    pub fn new(i2c: &mut I2c<I2C2>) -> Self {
        // todo temp
        return Self {
            ground_cal: Default::default(),
            gps_cal_init: None,
            gps_cal_air: None,
            hardware_coeff_cal: Default::default(),
        };

        // Set 64x oversampling, and 128 measurements per second, for both temp and pres.
        i2c.write(ADDR, &[PRS_CFG, 0b0111_0110]).ok();
        i2c.write(ADDR, &[TMP_CFG, 0b0111_0110]).ok();

        // Continuous pressure and temp measurement in background mode.
        i2c.write(ADDR, &[MEAS_CFG, 0b0000_0111]).ok();

        // Enable pressure and temp bit-shift due to our high sampling rate.
        // No interrupts enabled.
        i2c.write(ADDR, &[CFG_REG, 0b0000_1100]).ok();

        // todo temp debug
        let mut prs_buf = [0];
        i2c.write_read(ADDR, &[PRS_CFG], &mut prs_buf).ok();
        println!("PRS CFG REG: {:b}", prs_buf[0]);

        i2c.write_read(ADDR, &[MEAS_CFG], &mut prs_buf).ok();
        println!("Meas REG: {:b}", prs_buf[0]);

        // Load calibration data, factory-coded.
        // Wait until the coefficients are ready to read. Also, wait until readings are ready
        // here as well.
        loop {
            let mut buf = [0];
            i2c.write_read(ADDR, &[MEAS_CFG], &mut buf).ok();
            if (buf[0] & 0b1100_0000) == 0b1100_0000 {
                break;
            }
        }

        let mut buf_a = [0];
        let mut buf_b = [0];
        let mut buf_c = [0];

        // todo DRY. Split up into sub fns, at least for the last few that are uniform.

        i2c.write_read(ADDR, &[COEF_0], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_0_1], &mut buf_b).ok();

        let mut c0 = i16::from_be_bytes([buf_a[0], buf_b[0] >> 4]) as i32;

        i2c.write_read(ADDR, &[COEF_1], &mut buf_a).ok();

        let mut c1 = i16::from_be_bytes([buf_b[0] & 0xF, buf_a[0]]) as i32;

        i2c.write_read(ADDR, &[COEF_00_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_00_B], &mut buf_b).ok();
        i2c.write_read(ADDR, &[COEF_00_10], &mut buf_c).ok();
        let mut c00 = i32::from_be_bytes([0, buf_a[0], buf_b[0], buf_c[0] >> 4]);

        i2c.write_read(ADDR, &[COEF_10_A], &mut buf_a).ok();
        i2c.write_read(ADDR, &[COEF_10_B], &mut buf_b).ok();
        let mut c10 = i32::from_be_bytes([0, buf_c[0] & 0xF, buf_a[0], buf_b[0]]);

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

        // c0 and c1 are 12 bits. c00 and c10 are 20 bits. The rest are 16.
        // All are 2's complement.
        fix_int_sign(&mut c0, 12);
        fix_int_sign(&mut c1, 12);
        fix_int_sign(&mut c00, 20);
        fix_int_sign(&mut c10, 20);

        // println!("C0 {} C1 {} C10 {} C20 {} C30 {} C01 {} C11 {} C21 {} C30 {}", c0, c1, c10, c20, c30, c01, c11, c21, c30);

        Self {
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
        }
    }

    pub fn calibrate(&mut self, gps_alt: Option<f32>, i2c: &mut I2c<I2C2>) {
        let pressure = self.read_pressure(i2c);
        let temp = self.read_temp(i2c);

        self.ground_cal = AltitudeCalPt {
            pressure,
            altitude: 0.,
            temp,
        };

        self.gps_cal_air = match gps_alt {
            Some(alt_msl) => Some(AltitudeCalPt {
                pressure,
                altitude: alt_msl,
                temp,
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

        (self.hardware_coeff_cal.c0 / 2 * self.hardware_coeff_cal.c1 * t_raw_sc) as f32
    }

    // todo: Given you use temp readings to feed into pressure, combine somehow to reduce reading
    // todo and computation.

    /// Read atmospheric pressure, in kPa.
    pub fn read_pressure(&self, i2c: &mut I2c<I2C2>) -> f32 {
        // The Pressure Data registers contains the 24 bit (3 bytes) 2's complement pressure measurement value.
        // If the FIFO is enabled, the register will contain the FIFO pressure and/or temperature results. Otherwise, the
        // register contains the pressure measurement results and will not be cleared after read.

        let mut buf2 = [0];
        let mut buf1 = [0];
        let mut buf0 = [0];
        i2c.write_read(ADDR, &[PSR_B2], &mut buf2).ok();
        i2c.write_read(ADDR, &[PSR_B1], &mut buf1).ok();
        i2c.write_read(ADDR, &[PSR_B0], &mut buf0).ok();

        let mut p_raw = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_int_sign(&mut p_raw, 24);

        // todo: DRY with above and temp readings below, also does this twice if getting temp at same time.
        i2c.write_read(ADDR, &[TMP_B2], &mut buf2).ok();
        i2c.write_read(ADDR, &[TMP_B1], &mut buf1).ok();
        i2c.write_read(ADDR, &[TMP_B0], &mut buf0).ok();

        let mut t_raw = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_int_sign(&mut t_raw, 24);

        self.pressure_from_reading(p_raw, t_raw)
    }

    /// Read temperature, in °C.
    pub fn read_temp(&self, i2c: &mut I2c<I2C2>) -> f32 {
        // The Temperature Data registers contain the 24 bit (3 bytes) 2's complement temperature measurement value
        // ( unless the FIFO is enabled, please see FIFO operation ) and will not be cleared after the read.

        // todo: DRY with pressure read

        let mut buf2 = [0];
        let mut buf1 = [0];
        let mut buf0 = [0];
        i2c.write_read(ADDR, &[TMP_B2], &mut buf2).ok();
        i2c.write_read(ADDR, &[TMP_B1], &mut buf1).ok();
        i2c.write_read(ADDR, &[TMP_B0], &mut buf0).ok();

        let mut reading = i32::from_be_bytes([0, buf2[0], buf1[0], buf0[0]]);
        fix_int_sign(&mut reading, 24);

        self.temp_from_reading(reading)
    }

    /// Estimate altitude MSL from pressure and temperature. Pressure is in Pa; temp is in K.
    /// Uses a linear map between 2 pointers: Either from the standard atmosphere model, or from
    /// GPS points, if available.
    /// https://en.wikipedia.org/wiki/Barometric_formula
    /// https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
    pub fn estimate_altitude_msl(&self, pressure: f32, temp: f32) -> f32 {
        // P = 101.29 * ((temp)/288.08)^5.256   (in kPa)
        // T = 150.4 - .00649h

        let (point_0, point_1) = if self.gps_cal_init.is_some() && self.gps_cal_air.is_some() {
            (
                self.gps_cal_init.as_ref().unwrap(),
                self.gps_cal_air.as_ref().unwrap(),
            )
        } else {
            (&POINT_0, &POINT_1)
        };

        (((POINT_0.pressure / pressure).powf(1. / 5.257) - 1.) * temp) / 0.00649

        // log_lapse_rate(P/POINT_0.pressure) = (POINT_0.temp + (alt - POINT_0.altitude) * )

        // todo: Temp compensate!
        // todo: You probably want a non-linear, eg exponential model.
        // util::map_linear(pressure, (point_0.pressure, POINT_1.pressure), (point_0.altitude, point_1.altitude))
    }
}
