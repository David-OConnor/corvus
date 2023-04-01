//! This module contains code for U-BLOX M10 GNSS modules.
//! It usse the UBX protocol, although others are available. It uses USART, although
//! I2C is also available.

use num_enum::TryFromPrimitive;

use stm32_hal2::usart;

use crate::{ppks::Location, setup::UartGnss};

// UBX messages always start with these 2 preamble characters.
const PREAMBLE_1: u8 = 0xb5;
const PREAMBLE_2: u8 = 0x62;

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
enum MsgClass {}

pub enum GnssError {
    Bus,
    Fix,
    Crc,
}

pub struct GnssNotConnectedError {}
pub struct GpsFixError {}

impl From<usart::Error> for GnssError {
    fn from(_e: usart::Error) -> Self {
        Self::Bus
    }
}

struct Payload<'a> {
    /// A 1-byte message class field follows. A class is a group of messages that are related to each
    /// other.
    pub class: MsgClass,
    /// A 1-byte message ID field defines the message that is to follow.
    pub id: u8,
    /// A 2-byte length field follows. The length is defined as being that of the payload only. It does not
    /// include the preamble, message class, message ID, length, or UBX checksum fields. The number
    /// format of the length field is an unsigned little-endian 16-bit integer (a "U2" in UBX data types).
    pub payload_len: u16,
    pub payload: &'a [u8],
}

impl Payload {
    fn calc_checksum(&self) -> u16 {}

    pub fn to_buf(&self, buf: &mut [u8]) {
        let payload_end = 6 + self.payload_len as usize;

        buf[0] = PREAMBLE_1;
        buf[1] = PREAMBLE_2;
        buf[2] = self.class as u8;
        buf[3] = self.id;
        buf[4..6].clone_from_slice(&self.payload_len.to_le_bytes().unwrap());
        buf[6..payload_end].clone_from_slice(self.payload);
        buf[payload_end..payload_end + 2].clone_from_slice(&self.calc_checksum().to_le_bytes());
    }

    pub fn from_buf(buf: &[u8]) -> Result<Self, GnssError> {
        let payload_len = buf[4..6].to_le_bytes();
        let payload_end = 6 + payload_len as usize;

        let mut result = Self {
            class: buf[2].try_into().unwrap(),
            id: buf[3],
            payload_len,
            payload: &buf[6..payload_end],
        };

        let crc_received = payload[payload_end..payload_end + 2].to_le_bytes();

        if crc_received != result.calc_checksum() {
            return Err(GnssError::Crc);
        }

        Ok((Result))
    }
}

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
pub fn setup(i2c: &mut I2c<I2C1>) -> Result<(), GnssNotConnectedError> {
    let mut buf = [0x01, 0x02, 0, 0, 0, 0, 0, 0];
    i2c.read(ADDR, &mut buf)?;

    Ok(())
}

pub fn get_fix(i2c: &mut I2c<I2C1>) -> Result<Location, GpsFixError> {
    Err(GpsFixError {})
}
