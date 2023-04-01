//! This module contains code for U-BLOX M10 GNSS modules.
//! It used the UBX protocol, although others are available. It uses USART, although
//! I2C is also available.
//!
//! See the Ublox M10 interface manual for how this is set up.

use num_enum::TryFromPrimitive;

use stm32_hal2::usart;

use crate::{ppks::Location, setup::UartGnss};

// UBX messages always start with these 2 preamble characters.
const PREAMBLE_1: u8 = 0xb5;
const PREAMBLE_2: u8 = 0x62;

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
/// See Interface manual, section 3.8: UBX messages overview
///
/// todo: Class enum, and id-per-class enum?
enum MsgClassId {
    /// Message acknowledged (Output)
    AckAck,
    /// Message not acknowledged (Output)
    AckNak,
    /// Clear, save and load configurations (Command)
    CfgCfg,
    /// Reset receiver / Clear backup data structures (Command)
    CfgRst,
    /// Delete configuration item values (Set)
    /// Delete configuration item values (with transaction) (Set)
    CfgValDel,
    CfgValGet,
    CfgValSet,
    InfDebug,
    InfError,
    InfNotice,
    InfTest,
    InfWarning,
    LogBatch,
    LogRetrieveBath,
    MgaAck,
    MgaAno,
    MgaBds,
    MgaDbd,
    MgaGal,
    MgaGlo,
    MgaGps,
    MgaIni,
    MgaQzss,
    MonBatch,
    MonComms,
    MonGnss,
    MonHw3,
    MonPatch,
    MonRf,
    MonRxr,
    MonSpan,
    MonVer,
    /// AssistNow Autonomous status (Periodic/polled)
    NavAopstatus,
    /// Clock solution (Periodic/polled)
    NavClock,
    NavCov,
    NavDop,
    NavEoe,
    NavOdo,
    NavOrb,
    NavPl,
    NavPosecef,
    NavPosllh,
    NavPvt,
    NavResetOdo,
    /// Satellite information (Periodic/polled)
    NavSat,
    /// SBAS status data (Periodic/polled)
    NavSbas,
    /// Signal information (Periodic/polled)
    NavSig,
    NavSlas,
    NavStatus,
    NavTimeBds,
    NavTimeGal,
    NavTimeGlo,
    NavTimeGps,
    NavTimeLs,
    NavTimeQzss,
    /// UTC time solution (Periodic/polled)
    NavTimeUtc,
    /// Velocity solution in ECEF (Periodic/polled)
    NavVelecef,
    /// Velocity solution in NED frame (Periodic/polled)
    NavVelned,
    RxmMeas20,
    RxmMeas50,
    RxmMeasc12,
    RxmMeasd12,
    RxmMeasx,
    RxmPmreq,
    RxmRlm,
    RxmSfrbx,
    SecUniqid,
    TimTm2,
    TimTp,
    TimVrfy,
    UpdSos,
}

impl MsgClassId {
    /// Return a (class, id) tuple.
    pub fn to_vals(&self) -> (u8, u8) {
        match self {
            Self::AckAck => (0x05, 0x01),
            Self::AckNak => (0x05, 0x00),
            Self::CfgCfg => (0x06, 0x09),
            Self::CfgRst => (0x06, 0x04),
            Self::CfgValDel => (0x06, 0x8c),
            Self::CfgValGet => (0x06, 0x8b),
            Self::CfgValSet => (0x06, 0x8a),
            Self::InfDebug => (0x04, 0x04),
            Self::InfError => (0x04, 0x00),
            Self::InfNotice => (0x04, 0x02),
            Self::InfTest => (0x04, 0x03),
            Self::InfWarning => (0x04, 0x01),
            Self::LogBatch => (0x21, 0x11),
            Self::LogRetrieveBath => (0x21, 0x10),
            Self::MgaAck => (0x13, 0x60),
            Self::MgaAno => (0x13, 0x20),
            Self::MgaBds => (0x13, 0x03),
            Self::MgaDbd => (0x13, 0x80),
            Self::MgaGal => (0x13, 0x02),
            Self::MgaGlo => (0x13, 0x06),
            Self::MgaGps => (0x13, 0x00),
            Self::MgaIni => (0x13, 0x40),
            Self::MgaQzss => (0x13, 0x05),
            Self::MonBatch => (0x0a, 0x32),
            Self::MonComms => (0x0a, 0x36),
            Self::MonGnss => (0x0a, 0x28),
            Self::MonHw3 => (0x0a, 0x37),
            Self::MonPatch => (0x0a, 0x27),
            Self::MonRf => (0x0a, 0x38),
            Self::MonRxr => (0x0a, 0x21),
            Self::MonSpan => (0x0a, 0x31),
            Self::MonVer => (0x0a, 0x04),
            Self::NavAopstatus => (0x01, 0x60),
            Self::NavClock => (0x01, 0x22),
            Self::NavCov => (0x01, 0x36),
            Self::NavDop => (0x01, 0x04),
            Self::NavEoe => (0x01, 0x61),
            Self::NavOdo => (0x01, 0x09),
            Self::NavOrb => (0x01, 0x34),
            Self::NavPl => (0x01, 0x62),
            Self::NavPosecef => (0x01, 0x01),
            Self::NavPosllh => (0x01, 0x02),
            Self::NavPvt => (0x01, 0x07),
            Self::NavResetOdo => (0x01, 0x10),
            Self::NavSat => (0x01, 0x35),
            Self::NavSbas => (0x01, 0x32),
            Self::NavSig => (0x01, 0x43),
            Self::NavSlas => (0x01, 0x42),
            Self::NavStatus => (0x01, 0x03),
            Self::NavTimeBds => (0x01, 0x24),
            Self::NavTimeGal => (0x01, 0x25),
            Self::NavTimeGlo => (0x01, 0x23),
            Self::NavTimeGps => (0x01, 0x20),
            Self::NavTimeLs => (0x01, 0x26),
            Self::NavTimeQzss => (0x01, 0x27),
            Self::NavTimeUtc => (0x01, 0x21),
            Self::NavVelecef => (0x01, 0x11),
            Self::NavVelned => (0x01, 0x12),
            Self::RxmMeas20 => (0x02, 0x84),
            Self::RxmMeas50 => (0x02, 0x86),
            Self::RxmMeasc12 => (0x02, 0x82),
            Self::RxmMeasd12 => (0x02, 0x80),
            Self::RxmMeasx => (0x02, 0x14),
            Self::RxmPmreq => (0x02, 0x41),
            Self::RxmRlm => (0x02, 0x59),
            Self::RxmSfrbx => (0x02, 0x13),
            Self::SecUniqid => (0x27, 0x03),
            Self::TimTm2 => (0x0d, 0x03),
            Self::TimTp => (0x0d, 0x001),
            Self::TimVrfy => (0xd0, 0x06),
            Self::UpdSos => (0x09, 0x14),
        }
    }

    pub fn from_vals(vals: (u8, u8)) -> Self {}
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
    /// A 1-byte message ID field defines the message that is to follow.
    pub class_id: MsgClassId,
    // pub id: u8,
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

impl<'a> Payload<'a> {
    pub fn to_buf(&self, buf: &mut [u8]) {
        let payload_end = 6 + self.payload_len as usize;
        let (class, id) = self.class_id.to_vals();

        buf[0] = PREAMBLE_1;
        buf[1] = PREAMBLE_2;
        buf[2] = class;
        buf[3] = id;
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

        let class = buf[2];
        let id = buf[3];
        let class_id = MsgClassId.from_vals(class, id);

        let mut result = Self {
            class_id,
            // class: .try_into().unwrap(),
            // id: buf[3],
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
