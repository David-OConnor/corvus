//! Driver for the ST LIS3MDL 3-axis magnetometer. This is an I2C or SPI device;
//! This module only includes functionality for I2C mode.

use stm32_hal2::{
    i2c::{self, I2c},
    pac::I2C1,
};

use cortex_m::delay::Delay;

use defmt::println;

// DS: "The Slave Address (SAD) associated to the LIS3MDL is 00111x0b, whereas the x bit is
// modified by the SDO/SA1 pin in order to modify the device address. If the SDO/SA1 pin is
// connected to the voltage supply, the address is 0011110b, otherwise, if the SDO/SA1 pin is
// connected to ground, the address is 0011100b."
// Our board has it wired to ground.
pub const ADDR: u8 = 0b11100; // ie 0x1C (Can alternatively be wired for 0b11110)

const WHOAMI: u8 = 0x3d;

pub struct MagNotConnectedError {}

impl From<i2c::Error> for MagNotConnectedError {
    fn from(_e: i2c::Error) -> Self {
        Self {}
    }
}

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

pub fn setup(i2c: &mut I2c<I2C1>) -> Result<(), MagNotConnectedError> {
    // todo 2 to TS
    // todo: Could we take advantaeg of temp here for a diff purpose, away
    // from the FC board?

    println!("MAG Pre setup");

    let mut read_buf = [0];
    i2c.write_read(ADDR, &[Reg::WhoAmI as u8], &mut read_buf)?;

    println!("READ BUF Mag: {:?}", read_buf);
    if (read_buf[0] & 0xf) != WHOAMI {
        // return Err(BaroNotConnectedError {}); // todo: PUt back once baro is workign
    }

    // Disable temp sensor. Set fast ODR, in ultra-high-performance mode.
    // Todo: This sets a relatively low refresh rate of 155Hz.
    // todo: But lower performance modes have up to 1kHz??
    i2c.write(ADDR, &[Reg::Ctrl1 as u8, 0b0110_0010])?;
    println!("MAG Post setup");

    // Set fullscale range to +-4 gauss.
    i2c.write(ADDR, &[Reg::Ctrl2 as u8, 0b0000_0000])?;

    // Set to continuous-conversion mode.
    i2c.write(ADDR, &[Reg::Ctrl3 as u8, 0b0000_0011])?;

    // Set Z-axis to ultra-high performance mode. Set little endian data.
    i2c.write(ADDR, &[Reg::Ctrl4 as u8, 0b0000_1100])?;

    // Disable fast read. Disable block data update. Perhaps this
    // will ensure the latest data is always what we read.
    // todo: Do we want block data update?
    // "Block data update for magnetic data. Default value: 0
    // (0: continuous update;
    // 1: output registers not updated until MSb and LSb have been read)"
    i2c.write(ADDR, &[Reg::Ctrl5 as u8, 0b0000_0000])?;

    Ok(())
}

// i2c.write_read(ADDR, &[Reg::MeasCfg as u8], &mut prs_buf)?;
