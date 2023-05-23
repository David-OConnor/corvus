#![allow(dead_code)]

//! Driver for the ST LIS3MDL 3-axis magnetometer. This is an I2C or SPI device;
//! This module only includes functionality for I2C mode.

use stm32_hal2::i2c;

use cortex_m::delay::Delay;

use defmt::println;

use crate::setup::I2cMag;

use lin_alg2::f32::Vec3;

// DS: "The Slave Address (SAD) associated to the LIS3MDL is 00111x0b, whereas the x bit is
// modified by the SDO/SA1 pin in order to modify the device address. If the SDO/SA1 pin is
// connected to the voltage supply, the address is 0011110b, otherwise, if the SDO/SA1 pin is
// connected to ground, the address is 0011100b."
// Our board has it wired to ground.
pub const ADDR: u8 = 0b11100; // ie 0x1C (Can alternatively be wired for 0b11110)

const WHOAMI: u8 = 0x3d;

const FULL_SCALE_DEFLECTION: FullScaleDeflection = FullScaleDeflection::G4;

pub struct MagNotConnectedError {}

#[derive(Clone, Copy)]
#[repr(u8)]
enum FullScaleDeflection {
    /// +/- 4 Gauss, etc
    G4 = 0b00,
    G8 = 0b01,
    G12 = 0b10,
    G16 = 0b11,
}

impl FullScaleDeflection {
    /// Value, in Gauss.
    pub fn value(&self) -> f32 {
        match self {
            Self::G4 => 4.,
            Self::G8 => 8.,
            Self::G12 => 12.,
            Self::G16 => 16.,
        }
    }
}

impl From<i2c::Error> for MagNotConnectedError {
    fn from(_e: i2c::Error) -> Self {
        Self {}
    }
}
//
// // todo: Use Vec3 instead?
// /// Represents sensor readings from a 3-axis magnetometer.
// /// Accelerometer readings are in m/2^2. Gyroscope readings are in radians/s.
// #[derive(Default)]
// pub struct MagReadings {
//     /// Positive X: Accel towards right wing
//     pub x: f32,
//     /// Positive Y: Accel forwards
//     pub y: f32,
//     /// Positive X: Accel up
//     pub z: f32,
// }

/// DS, Table 16. Register address map.
#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    WhoAmI = 0x0f,
    Ctrl1 = 0x20,
    Ctrl2 = 0x21,
    Ctrl3 = 0x22,
    Ctrl4 = 0x23,
    Ctrl5 = 0x24,
    Status = 0x27,
    OutXL = 0x28,
    OutXH = 0x29,
    OutYL = 0x2a,
    OutYH = 0x2b,
    OutZL = 0x2c,
    OutZH = 0x2d,
    TempOutL = 0x2e,
    TempOutH = 0x2f,
    IntCfg = 0x30,
    IntSrc = 0x31,
    IntThsL = 0x32,
    INtThsH = 0x33,
}

pub fn setup(i2c: &mut I2cMag) -> Result<(), MagNotConnectedError> {
    let mut read_buf = [0];
    i2c.write_read(ADDR, &[Reg::WhoAmI as u8], &mut read_buf)?;

    if (read_buf[0]) != WHOAMI {
        return Err(MagNotConnectedError {});
    }

    // Disable temp sensor. Set fast Output Data Register, in ultra-high-performance mode.
    // This leads to a 155Hz refresh rate on the ODR.
    // todo: Do you want perhaps HP or MP mode at a higher rate?
    i2c.write(ADDR, &[Reg::Ctrl1 as u8, 0b0110_0010])?;

    // Set fullscale range.
    let ctrl2_val = (FULL_SCALE_DEFLECTION as u8) << 5;
    i2c.write(ADDR, &[Reg::Ctrl2 as u8, ctrl2_val])?;

    // Set to continuous-conversion mode.
    i2c.write(ADDR, &[Reg::Ctrl3 as u8, 0b0000_0000])?;

    // Set Z-axis to ultra-high performance mode. Set little endian data.
    i2c.write(ADDR, &[Reg::Ctrl4 as u8, 0b0000_1100])?;

    // Disable fast read. Disable block data update. Perhaps this
    // will ensure the latest data is always what we read.
    // "Block data update for magnetic data. Default value: 0
    // (0: continuous update;
    // 1: output registers not updated until MSb and LSb have been read)"
    i2c.write(ADDR, &[Reg::Ctrl5 as u8, 0b0000_0000])?;

    Ok(())
}

fn interpret_mag(val: i16) -> f32 {
    (val as f32 / i16::MAX as f32) * FULL_SCALE_DEFLECTION.value()
}

/// Given readings taken from registers directly, calcualte magnetic induction (flux density),
/// in Gauss.
/// We split this from the other functions for use with DMA.
pub fn mag_induction_from_readings(buf: &[u8; 6]) -> Vec3 {
    let [xl, xh, yl, yh, zl, zh] = *buf;

    let x = interpret_mag(i16::from_le_bytes([xl, xh]));
    let y = interpret_mag(i16::from_le_bytes([yl, yh]));
    let z = interpret_mag(i16::from_le_bytes([zl, zh]));

    Vec3::new(x, y, z)
}

/// Reads magnetic induction (flux density), in Gauss.
pub fn read(i2c: &mut I2cMag) -> Result<Vec3, MagNotConnectedError> {
    // The Pressure Data registers contains the 16 bit 2's complement magnetic inductino measurement value.
    let mut buf = [0; 6];
    i2c.write_read(ADDR, &[Reg::OutXL as u8], &mut buf)?;

    Ok(mag_induction_from_readings(&buf))
}
