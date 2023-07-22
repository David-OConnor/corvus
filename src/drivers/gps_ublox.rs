//! This module contains code for U-BLOX M10 GNSS modules.
//! It used the UBX protocol, although others are available. It uses USART, although
//! I2C is also available.
//!
//! See the Ublox M10 interface manual for how this is set up.

use core::sync::atomic::AtomicBool;

use num_enum::TryFromPrimitiveError;

use stm32_hal2::{
    clocks::Clocks,
    usart::{self, UsartInterrupt},
};

use ahrs::{Fix, FixType};

use chrono::NaiveDate;

use defmt::println;

use crate::setup::UartGnss;

// UBX messages always start with these 2 preamble characters.
const PREAMBLE_1: u8 = 0xb5;
const PREAMBLE_2: u8 = 0x62;

// Max Baud, per DS, is 921,600
// The peripheral is initialized at 9.6kbps. Once a higher update rate is configured on the GNSS,
// we update the UART peripheral. We try each at startup, since we don't know if the GNSS has had it's
// power interrupted, eg during debug runs. In operations, we expect the MCU and GNSS to power
// reset at the same time.
pub const BAUD_AT_RESET: u32 = 9_600;
pub const BAUD: u32 = 691_200;

// Maximum of 18Hz with a single constellation. Lower rates with fused data. For example,
// GPS + GAL is 10Hz max.
pub const MEASUREMENT_RATE: f32 = 10.; // Measurement rate in Hz.

// Includes start bytes, class, id, payload length, and CRC.
const MSG_SIZE_WITHOUT_PAYLOAD: usize = 8;

// Position, velocity, time data payload side: UBX-NAV-PVT.
const PAYLOAD_LEN_PVT: usize = 92;
const PAYLOAD_LEN_DOP: usize = 18;
const PAYLOAD_LEN_COVARIANCE: usize = 64;
// Payload length for an acknowledgement message.
const PAYLOAD_LEN_ACK_NAK: usize = 2;

// The first few messages of config are reserved or used to set RAM vs FLASH
const CFG_PAYLOAD_START_I: usize = 4;

// We use this max length for our DMA read buffer.
// We pad this due to the possibility of shifted data.
pub const MAX_BUF_LEN: usize = PAYLOAD_LEN_PVT + MSG_SIZE_WITHOUT_PAYLOAD + 4;

pub static mut RX_BUFFER: [u8; MAX_BUF_LEN] = [0; MAX_BUF_LEN];

pub static TRANSFER_IN_PROG: AtomicBool = AtomicBool::new(false);

// todo: Dedicated lib for these helpers:
/// Helper function; keeps syntax terser on repeated calls.
fn u16_from_le(buf: &[u8]) -> u16 {
    u16::from_le_bytes(buf.try_into().unwrap())
}

/// Helper function; keeps syntax terser on repeated calls.
fn i32_from_le(buf: &[u8]) -> i32 {
    i32::from_le_bytes(buf.try_into().unwrap())
}

/// Helper function; keeps syntax terser on repeated calls.
fn f32_from_le(buf: &[u8]) -> f32 {
    f32::from_le_bytes(buf.try_into().unwrap())
}

/// Get position, velocity, and time data, assuming it has already been transsferred into the reception
/// buffer. Run this after a packet has been completedly received, eg as indicated by the UART idle
/// interrupt.
///
/// The input buffer includes the entire packet, including CRC, message length, Class and ID etc.
///
/// Timestamp is seconds since program start.
pub fn fix_from_payload(payload: &[u8], timestamp: f32) -> Result<Fix, GnssError> {
    if payload.len() < PAYLOAD_LEN_PVT {
        println!("Incorrect PVT payload.");
        return Err(GnssError::MessageData);
    }

    let flags = payload[21];
    let fix_ok = (flags & 1) != 0;
    let heading_valid = (flags & 0b10_0000) != 0;

    let date = NaiveDate::from_ymd_opt(
        u16_from_le(&payload[4..6]) as i32,
        payload[6] as u32,
        payload[7] as u32,
    );
    if date.is_none() {
        return Err(GnssError::MessageData);
    }
    let date = date.unwrap(); // eg invalid values.

    let ns = i32_from_le(&payload[16..20]);
    let datetime = date.and_hms_nano_opt(
        payload[8] as u32,
        payload[9] as u32,
        payload[10] as u32,
        ns as u32,
    );
    if datetime.is_none() {
        return Err(GnssError::MessageData);
    }
    let datetime = datetime.unwrap();

    let lat_e7 = i32_from_le(&payload[28..32]);
    let lon_e7 = i32_from_le(&payload[24..28]);

    let heading = if heading_valid {
        Some(i32_from_le(&payload[84..88]) as f32)
    } else {
        None
    };

    let type_ = if fix_ok {
        payload[20].try_into()?
    } else {
        FixType::NoFix
    };

    Ok(Fix {
        timestamp_s: timestamp,
        datetime,
        type_,
        lat_e7,
        lon_e7,
        elevation_hae: i32_from_le(&payload[32..36]),
        elevation_msl: i32_from_le(&payload[36..40]),
        ground_speed: i32_from_le(&payload[60..64]),
        ned_velocity: [
            i32_from_le(&payload[48..52]),
            i32_from_le(&payload[52..56]),
            i32_from_le(&payload[56..60]),
        ],
        heading,
        sats_used: payload[23],
        pdop: u16_from_le(&payload[76..78]),
    })
}
// pub fn print(&self) {
//     println!(
//         "Fix data: Timestamp: {}, {}:{}:{}, type: {}, \nlat: {}, lon: {}, \n\
//         HAE: {}, MSL: {}, sats used: {}",
//         self.datetime.day(),
//         self.datetime.hour(),
//         self.datetime.minute(),
//         self.datetime.second(),
//         self.type_ as u8,
//         self.lat as f32 / 10_000_000.,
//         self.lon as f32 / 10_000_000.,
//         self.elevation_hae as f32 / 1_000.,
//         self.elevation_msl as f32 / 1_000.,
//         self.sats_used,
//     );
// }

#[derive(Default)]
/// Dilution of precision (DOP) Eg parsed from UBX-NAV-DOP.
/// DOP values are dimensionless.
/// All DOP values are scaled by a factor of 100. If the unit transmits a value of e.g. 156,
// the DOP value is 1.56.
pub struct DilutionOfPrecision {
    /// GPS time of week of the navigation epoch; reported by the GNSS.
    // pub timestamp: u32,
    // pub datetime: NaiveDateTime,
    pub geometric: u16,
    pub position: u16,
    pub time: u16,
    pub vertical: u16,
    pub horizontal: u16,
    pub northing: u16,
    pub easting: u16,
}

impl DilutionOfPrecision {
    pub fn from_payload(payload: &[u8]) -> Result<Self, GnssError> {
        if payload.len() < PAYLOAD_LEN_DOP {
            println!("Incorrect DOP payload.");
            return Err(GnssError::MessageData);
        }

        Ok(Self {
            geometric: u16_from_le(&payload[4..6]),
            position: u16_from_le(&payload[6..8]),
            time: u16_from_le(&payload[8..10]),
            vertical: u16_from_le(&payload[10..12]),
            horizontal: u16_from_le(&payload[12..14]),
            northing: u16_from_le(&payload[14..16]),
            easting: u16_from_le(&payload[16..18]),
        })
    }
}

/// UBX-NAV-COV
/// "This message outputs the covariance matrices for the position and velocity solutions in the topocentric
/// coordinate system defined as the local-level North (N), East (E), Down (D) frame. As the covariance matrices
/// are symmetric, only the upper triangular part is output."
#[derive(Default)]
pub struct Covariance {
    pub posit_valid: bool,
    pub velocity_valid: bool,
    pub pos_nn: f32,
    pub pos_ne: f32,
    pub pos_nd: f32,
    pub pos_ee: f32,
    pub pos_ed: f32,
    pub pos_dd: f32,
    pub vel_nn: f32,
    pub vel_ne: f32,
    pub vel_nd: f32,
    pub vel_ee: f32,
    pub vel_ed: f32,
    pub vel_dd: f32,
}

impl Covariance {
    pub fn from_payload(payload: &[u8]) -> Result<Self, GnssError> {
        if payload.len() < PAYLOAD_LEN_COVARIANCE {
            println!("Incorrect DOP payload.");
            return Err(GnssError::MessageData);
        }

        Ok(Self {
            posit_valid: payload[5] != 0,
            velocity_valid: payload[6] != 0,
            pos_nn: f32_from_le(&payload[16..20]),
            pos_ne: f32_from_le(&payload[20..24]),
            pos_nd: f32_from_le(&payload[24..28]),
            pos_ee: f32_from_le(&payload[28..32]),
            pos_ed: f32_from_le(&payload[32..36]),
            pos_dd: f32_from_le(&payload[36..40]),
            vel_nn: f32_from_le(&payload[40..44]),
            vel_ne: f32_from_le(&payload[44..48]),
            vel_nd: f32_from_le(&payload[48..52]),
            vel_ee: f32_from_le(&payload[52..56]),
            vel_ed: f32_from_le(&payload[56..60]),
            vel_dd: f32_from_le(&payload[60..64]),
        })
    }
}

#[derive(Clone, Copy, PartialEq)]
/// See Interface manual, section 3.8: UBX messages overview
///
/// todo: Class enum, and id-per-class enum?
pub enum MsgClassId {
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

    /// Construct a message given (class, id) numerical values.
    pub fn from_vals(vals: (u8, u8)) -> Result<Self, GnssError> {
        Ok(match vals {
            (0x05, 0x01) => Self::AckAck,
            (0x05, 0x00) => Self::AckNak,
            (0x06, 0x09) => Self::CfgCfg,
            (0x06, 0x04) => Self::CfgRst,
            (0x06, 0x8c) => Self::CfgValDel,
            (0x06, 0x8b) => Self::CfgValGet,
            (0x06, 0x8a) => Self::CfgValSet,
            (0x04, 0x04) => Self::InfDebug,
            (0x04, 0x00) => Self::InfError,
            (0x04, 0x02) => Self::InfNotice,
            (0x04, 0x03) => Self::InfTest,
            (0x04, 0x01) => Self::InfWarning,
            (0x21, 0x11) => Self::LogBatch,
            (0x21, 0x10) => Self::LogRetrieveBath,
            (0x13, 0x60) => Self::MgaAck,
            (0x13, 0x20) => Self::MgaAno,
            (0x13, 0x03) => Self::MgaBds,
            (0x13, 0x80) => Self::MgaDbd,
            (0x13, 0x02) => Self::MgaGal,
            (0x13, 0x06) => Self::MgaGlo,
            (0x13, 0x00) => Self::MgaGps,
            (0x13, 0x40) => Self::MgaIni,
            (0x13, 0x05) => Self::MgaQzss,
            (0x0a, 0x32) => Self::MonBatch,
            (0x0a, 0x36) => Self::MonComms,
            (0x0a, 0x28) => Self::MonGnss,
            (0x0a, 0x37) => Self::MonHw3,
            (0x0a, 0x27) => Self::MonPatch,
            (0x0a, 0x38) => Self::MonRf,
            (0x0a, 0x21) => Self::MonRxr,
            (0x0a, 0x31) => Self::MonSpan,
            (0x0a, 0x04) => Self::MonVer,
            (0x01, 0x60) => Self::NavAopstatus,
            (0x01, 0x22) => Self::NavClock,
            (0x01, 0x36) => Self::NavCov,
            (0x01, 0x04) => Self::NavDop,
            (0x01, 0x61) => Self::NavEoe,
            (0x01, 0x09) => Self::NavOdo,
            (0x01, 0x34) => Self::NavOrb,
            (0x01, 0x62) => Self::NavPl,
            (0x01, 0x01) => Self::NavPosecef,
            (0x01, 0x02) => Self::NavPosllh,
            (0x01, 0x07) => Self::NavPvt,
            (0x01, 0x10) => Self::NavResetOdo,
            (0x01, 0x35) => Self::NavSat,
            (0x01, 0x32) => Self::NavSbas,
            (0x01, 0x43) => Self::NavSig,
            (0x01, 0x42) => Self::NavSlas,
            (0x01, 0x03) => Self::NavStatus,
            (0x01, 0x24) => Self::NavTimeBds,
            (0x01, 0x25) => Self::NavTimeGal,
            (0x01, 0x23) => Self::NavTimeGlo,
            (0x01, 0x20) => Self::NavTimeGps,
            (0x01, 0x26) => Self::NavTimeLs,
            (0x01, 0x27) => Self::NavTimeQzss,
            (0x01, 0x21) => Self::NavTimeUtc,
            (0x01, 0x11) => Self::NavVelecef,
            (0x01, 0x12) => Self::NavVelned,
            (0x02, 0x84) => Self::RxmMeas20,
            (0x02, 0x86) => Self::RxmMeas50,
            (0x02, 0x82) => Self::RxmMeasc12,
            (0x02, 0x80) => Self::RxmMeasd12,
            (0x02, 0x14) => Self::RxmMeasx,
            (0x02, 0x41) => Self::RxmPmreq,
            (0x02, 0x59) => Self::RxmRlm,
            (0x02, 0x13) => Self::RxmSfrbx,
            (0x27, 0x03) => Self::SecUniqid,
            (0x0d, 0x03) => Self::TimTm2,
            (0x0d, 0x01) => Self::TimTp,
            (0xd0, 0x06) => Self::TimVrfy,
            (0x09, 0x14) => Self::UpdSos,
            _ => {
                return Err(GnssError::MsgType);
            }
        })
    }
}

pub enum GnssError {
    Bus,
    Fix,
    /// CRC validation failed for a recieved message.
    Crc,
    MsgType,
    /// Received Nak, or did not receive Ack.
    NoAck,
    MessageData,
}

impl From<usart::Error> for GnssError {
    fn from(_e: usart::Error) -> Self {
        Self::Bus
    }
}

impl From<TryFromPrimitiveError<FixType>> for GnssError {
    fn from(_e: TryFromPrimitiveError<FixType>) -> Self {
        Self::Fix
    }
}

pub struct Message<'a> {
    /// A 1-byte message class field follows. A class is a group of messages that are related to each
    /// other.
    /// A 1-byte message ID field defines the message that is to follow.
    pub class_id: MsgClassId,
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

    for val in buffer {
        ck_a += val;
        ck_b += ck_a;
    }

    (ck_a, ck_b)
}

impl<'a> Message<'a> {
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
        let mut shift: isize = 0;
        // Check if the message has been shifted left due to jitter;
        // if so, shift right accordingly.
        if buf[0] == PREAMBLE_2 {
            shift = -1;
            // println!("GNSS L shift")
        } else if buf[0] != PREAMBLE_1 {
            return Err(GnssError::MessageData);
            // We can't prove this, but it's possible
            // shift = 1;
            // println!("Trying a right shift");
            // println!("BUf first 5: {:?}", buf[0 ..5]);
        }

        let class = buf[(2 + shift) as usize];
        let id = buf[(3 + shift) as usize];

        let class_id = MsgClassId::from_vals((class, id))?;

        let payload_len = u16_from_le(&buf[(4 + shift) as usize..(6 + shift) as usize]);
        let payload_end = (6 + shift) as usize + payload_len as usize;

        if payload_end > buf.len() {
            // This can come up while reading the acknowledgement if attempting to set up while already
            // set up, at a high data rate, eg if the read happens during PVT reception.
            return Err(GnssError::MessageData);
        }

        let result = Self {
            class_id,
            payload_len,
            payload: &buf[(6 + shift) as usize..payload_end],
        };

        let crc_received = (buf[payload_end], buf[payload_end + 1]);

        if crc_received != calc_checksum(&buf[(2 + shift as usize)..payload_end]) {
            return Err(GnssError::Crc);
        }

        Ok(result)
    }
}

/// Configure the UART interrupts, and GNSS configuration settings.
/// Configure the Char match and idle interrupts, which will allow the initial UART ISR to run
/// upon receiving data. Run this once, on initial firmware setup.
/// We alternate between char matching the flight controller destination address, and
/// line idle, to indicate we're received, or stopped receiving a message respectively.

/// Additionally, configure several settings on the GNSS module itself.
/// After this is run, the module will periodically transmit Position, Velocity, and Time (PVT)
/// packets.
pub fn setup(uart: &mut UartGnss, clock_cfg: &Clocks) -> Result<(), GnssError> {
    // todo: You should enable sensor fusion mode, eg to get heading?
    // todo: Enable dead-recoking.

    uart.enable_interrupt(UsartInterrupt::CharDetect(Some(PREAMBLE_1)));
    uart.enable_interrupt(UsartInterrupt::Idle);

    // Note: Fix mode defaults to auto, which allows fixes from multiple constellations.

    // CFG-UART1_BAUDRATE
    let key_id_baud: u32 = 0x4052_0001;
    let val_baud: u32 = BAUD;

    // Output rate of the UBX-NAV-PVT message on
    // port UART1. By default, no fix messages are output. It appears this is a divisor of the
    // measurement rate. So, a value of 1 means 1 PVT output on UART1 per measurement.

    // CFG-MSGOUT-UBX_NAV_PVT_UART1
    let key_id_pvt_rate: u32 = 0x2091_0007;
    let val_pvt_rate: u8 = 1;

    // CFG-RATE-MEAS
    let key_id_rate_meas: u32 = 0x3021_0001;
    let val_rate_meas = (1_000. / MEASUREMENT_RATE) as u16;

    // CFG-MSGOUT-UBX_NAV_DOP_UART1
    let key_id_dop_rate: u32 = 0x2091_0039;
    let val_dop_rate: u8 = 1;

    // CFG-MSGOUT-UBX_NAV_COV_UART1
    let key_id_cov_rate: u32 = 0x2091_0084;
    let val_cov_rate: u8 = 1;

    // "Configuration data is the binary representation of a list of Key ID and Value pairs. It is formed by
    // concatenating keys (U4 values) and values (variable type) without any padding. This format is used
    // in the UBX-CFG-VALSET and UBX-CFG-VALGET messages."

    const PAYLOAD_LEN_CFG: u16 = 33; // Adjust this based on which items you configure.
    let mut cfg_payload = [0; PAYLOAD_LEN_CFG as usize];

    // Bytes 0 and 1 are CFG metadata, prior to the key and value pairs. We use this to set the layer
    // as RAM. Bytes 2-3 are reserved.
    cfg_payload[1] = 0b001;

    cfg_payload[CFG_PAYLOAD_START_I..CFG_PAYLOAD_START_I + 4]
        .clone_from_slice(&key_id_baud.to_le_bytes());
    cfg_payload[8..12].clone_from_slice(&val_baud.to_le_bytes());

    cfg_payload[12..16].clone_from_slice(&key_id_pvt_rate.to_le_bytes());
    cfg_payload[16] = val_pvt_rate;

    cfg_payload[17..21].clone_from_slice(&key_id_rate_meas.to_le_bytes());
    cfg_payload[21..23].clone_from_slice(&val_rate_meas.to_le_bytes());

    cfg_payload[23..27].clone_from_slice(&key_id_dop_rate.to_le_bytes());
    cfg_payload[27] = val_dop_rate;

    cfg_payload[28..32].clone_from_slice(&key_id_cov_rate.to_le_bytes());
    cfg_payload[32] = val_cov_rate;

    let cfg_msg = Message {
        class_id: MsgClassId::CfgValSet,
        payload_len: PAYLOAD_LEN_CFG,
        payload: &cfg_payload,
    };

    const CFG_BUF_SIZE: usize = MSG_SIZE_WITHOUT_PAYLOAD + PAYLOAD_LEN_CFG as usize;

    let mut cfg_write_buf = [0; CFG_BUF_SIZE];
    cfg_msg.to_buf(&mut cfg_write_buf);

    uart.write(&cfg_write_buf)?;

    // The GNSS sends its ack at the new baud, so set it before reading.
    uart.set_baud(BAUD, clock_cfg)?;

    // Due to the delay on possible responses, don't check here.
    Ok(())
}
