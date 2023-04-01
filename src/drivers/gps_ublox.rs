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
enum MsgClass {
    Test = 0,
}

pub enum GnssError {
    Bus,
    Fix,
    Crc,
}

pub struct GnssNotConnectedError {}
pub struct GnssFixError {}

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

/// See interface manual, section 3.4: UBX checksum
/// "The checksum is calculated over the message, starting and including the class field up until, but
/// excluding, the checksum fields (see the figure UBX frame structure).
/// The checksum algorithm used is the 8-bit Fletcher algorithm, which is used in the TCP standard
/// RFC 1145)."
///
/// This is a standalone fn due to needing a ref to the buffer in question.
fn calc_checksum(buffer: &[u8]) -> (u8, u8) {
    // This code is taken directly from the interface manual.
    let mut ck_a = 0;
    let mut ck_b = 0;

    for i in 0..buffer.len() {
        ck_a += buffer[i];
        ck_b += ck_a;
    }

    (ck_a, ck_b)
}

impl <'a> Payload<'a> {
    pub fn to_buf(&self, buf: &mut [u8]) {
        let payload_end = 6 + self.payload_len as usize;

        buf[0] = PREAMBLE_1;
        buf[1] = PREAMBLE_2;
        buf[2] = self.class as u8;
        buf[3] = self.id;
        buf[4..6].clone_from_slice(&self.payload_len.to_le_bytes());
        buf[6..payload_end].clone_from_slice(self.payload);

        // "The checksum is calculated over the message, starting and including the class field up until, but
        // excluding, the checksum fields"
        let (checksum_a, checksum_b) = calc_checksum(&buf[2..payload_end]);

        buf[payload_end] = checksum_a;
        buf[payload_end + 1] = checksum_b;
    }

    pub fn from_buf(buf: &'a [u8]) -> Result<Self, GnssError> {
        let payload_len = u16::from_le_bytes(buf[4..6].try_into().unwrap());
        let payload_end = 6 + payload_len as usize;

        let mut result = Self {
            class: buf[2].try_into().unwrap(),
            id: buf[3],
            payload_len,
            payload: &buf[6..payload_end],
        };

        let crc_received = (buf[payload_end], buf[payload_end + 1]);

        if crc_received != calc_checksum(&buf[2..payload_end]) {
            return Err(GnssError::Crc);
        }

        Ok(result)
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
pub fn setup(uart: &mut UartGnss) -> Result<(), GnssNotConnectedError> {
    // let mut buf = [0x01, 0x02, 0, 0, 0, 0, 0, 0];
    // i2c.read(ADDR, &mut buf)?;

    Ok(())
}

pub fn get_fix(uart: &mut UartGnss) -> Result<Location, GnssFixError> {
    Err(GnssFixError {})
}
