//! This module contains code for the U-BLOX SAM-M8Q GNSS module.
//! We use the UBX protocol, although others are available. It uses I2C,
//! although the hardware also supports UART.

use stm32_hal2::{i2c::I2c, pac::I2C1};

use crate::ppks::Location;

pub struct GpsNotConnectedError {}
pub struct GpsFixError {}

const ADDR: u8 = 0x69; // todo

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Reg {
    NavStatus,
    PositLatLon,
    SatInfo,
}

impl Reg {
    /// Return the 2 register values
    pub fn vals(&self) -> (u8, u8) {
        match self {
            Self::NavStatus => (0x01, 0x03),
            Self::PositLatLon => (0x01, 0x02),
            Self::SatInfo => (0x01, 0x35),
        }
    }
}

/// Configure the GPS; run this at init.
pub fn setup(i2c: &mut I2c<I2C1>) -> Result<(), GpsNotConnectedError> {
    let mut buf = [0x01, 0x02, 0, 0, 0, 0, 0, 0];
    i2c.read(ADDR, &mut buf).unwrap(); // todo: Map to error type.

    Ok(())
}

pub fn get_fix(i2c: &mut I2c<I2C1>) -> Result<Location, GpsFixError> {
    Err(GpsFixError {})
}
