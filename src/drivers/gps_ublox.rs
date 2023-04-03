//! This module contains code for U-BLOX M10 GNSS modules.
//! It used the UBX protocol, although others are available. It uses USART, although
//! I2C is also available.
//!
//! See the Ublox M10 interface manual for how this is set up.

use core::sync::atomic::AtomicBool;

use num_enum::TryFromPrimitive;

use stm32_hal2::usart::{self, UsartInterrupt};

use crate::{
    ppks::{Location, LocationType},
    setup::{UartGnss, GNSS_DMA_PERIPH, GNSS_RX_CH, GNSS_TX_CH},
};

// UBX messages always start with these 2 preamble characters.
const PREAMBLE_1: u8 = 0xb5;
const PREAMBLE_2: u8 = 0x62;

// Max Baud, per DS, is 921,600
// pub const BAUD: u16 = 800_000; // todo: Set once you implement customizable baud. For now,
// todo we'll use the default of 9600.
pub const BAUD: u32 = 9_600;

// Includes start bytes, class, id, payload length, and CRC.
const MSG_SIZE_WITHOUT_PAYLOAD: usize = 8;

// Position, velocity, time data payload side: UBX-NAV-PVT.
const PAYLOAD_LEN_PVT: usize = 92;
// Payload length for an acknowledgement message.
const PAYLOAD_LEN_ACK_NAK: usize = 2;

// We use this max length for our DMA read buffer.
const MAX_BUF_LEN: usize = PAYLOAD_LEN_PVT + MSG_SIZE_WITHOUT_PAYLOAD + 10;

pub static mut RX_BUFFER: [u8; MAX_BUF_LEN] = [0; MAX_BUF_LEN];
// static mut TX_BUFFER: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];

pub static TRANSFER_IN_PROG: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum FixType {
    // These values are from the UBLOX protocol.
    NoFix = 0,
    DeadReckoning = 1,
    _2d = 2,
    _3d = 3,
    Combined = 4,
    TimeOnly = 5,
}

pub struct Fix {
    pub type_: FixType,
    pub lat: f64,
    pub lon: f64,
    pub elevation_hae: f32,
    pub elevation_msl: f32,
    pub ground_speed: f32,
    pub heading: Option<f32>, // only when valid.
}

#[derive(Clone, Copy, PartialEq)]
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
}

impl From<usart::Error> for GnssError {
    fn from(_e: usart::Error) -> Self {
        Self::Bus
    }
}

struct Message<'a> {
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

    for i in 0..buffer.len() {
        ck_a += buffer[i];
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
        let payload_len = u16::from_le_bytes(buf[4..6].try_into().unwrap());
        let payload_end = 6 + payload_len as usize;

        let class = buf[2];
        let id = buf[3];
        let class_id = MsgClassId::from_vals((class, id))?;

        let mut result = Self {
            class_id,
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

fn _setup_cfg_payload(payload: &mut [u8], item_id: u16, group_id: u8, storage_size: u8) {
    let key_id =
        (item_id & 0xfff) as u32 | (group_id as u32) << 16 | ((storage_size & 0b111) as u32) << 28;
}

/// Configure the UART interrupts, and GNSS configuration settings.
/// Configure the Char match and idle interrupts, which will allow the initial UART ISR to run
/// upon receiving data. Run this once, on initial firmware setup.
/// We alternate between char matching the flight controller destination address, and
/// line idle, to indicate we're received, or stopped receiving a message respectively.

/// Additionally, configure several settings on the GNSS module itself.
/// After this is run, the module will periodically transmit Position, Velocity, and Time (PVT)
/// packets.
pub fn setup(uart: &mut UartGnss) -> Result<(), GnssError> {
    // todo: You should enable sensor fusion mode, eg to get heading?
    // todo: Enable dead-recoking.

    uart.enable_interrupt(UsartInterrupt::CharDetect(Some(PREAMBLE_1)));
    uart.enable_interrupt(UsartInterrupt::Idle);

    // Note: Fix mode defaults to auto, which allows fixes from multiple constellations.

    let key_id_baud: u32 = 0x4052_0001;
    // let val_baud: u32 = BAUD;
    let val_baud: u32 = 9_600;

    // Output rate of the UBX-NAV-PVT message on
    // port UART1. By default, no fix messages are automatically output.
    let key_id_pvt_rate: u32 = 0x2091_0007;
    let val_pvt_rate: u8 = 1;

    // let key_id_fix_mode: u32 = 0x20110011;

    // "Configuration data is the binary representation of a list of Key ID and Value pairs. It is formed by
    // concatenating keys (U4 values) and values (variable type) without any padding. This format is used
    // in the UBX-CFG-VALSET and UBX-CFG-VALGET messages."

    const PAYLOAD_LEN_CFG: u16 = 12; // todo: Adjust this based on which settings you configure.

    let mut cfg_payload = [0; PAYLOAD_LEN_CFG as usize];
    cfg_payload[0..4].clone_from_slice(&key_id_baud.to_le_bytes());
    cfg_payload[4..8].clone_from_slice(&val_baud.to_le_bytes());

    cfg_payload[8..12].clone_from_slice(&key_id_pvt_rate.to_le_bytes());
    cfg_payload[12] = val_pvt_rate;

    let cfg_msg = Message {
        class_id: MsgClassId::CfgValSet,
        payload_len: PAYLOAD_LEN_CFG,
        payload: &cfg_payload,
    };

    const CFG_BUF_SIZE: usize = MSG_SIZE_WITHOUT_PAYLOAD + PAYLOAD_LEN_CFG as usize;

    let mut cfg_write_buf = [0; CFG_BUF_SIZE];
    cfg_msg.to_buf(&mut cfg_write_buf);

    uart.write(&cfg_write_buf);

    // We've written the configuration: Now check for an acknolwedgement from the device.
    let mut read_buf = [0; MSG_SIZE_WITHOUT_PAYLOAD + PAYLOAD_LEN_ACK_NAK];

    // todo: loop/timeout?
    // "Output upon processing of an input message. A UBX-ACK-ACK is sent as soon as possible but at
    // least within one second."
    uart.read(&mut read_buf);

    let msg = Message::from_buf(&read_buf)?;

    let (cfg_class, cfg_id) = MsgClassId::CfgValSet.to_vals();

    if msg.class_id != MsgClassId::AckAck || msg.payload[0] != cfg_class || msg.payload[1] != cfg_id
    {
        return Err(GnssError::NoAck);
    }

    Ok(())
}

// /// Get position, velocity, and time data.
// pub fn get_fix(uart: &mut UartGnss) -> Result<Option<Fix>, GnssError> {
//     // let write_payload = [];
//     //
//     // // todo: Ack.
//     //
//     // let write_msg = Message {
//     //     // Get position, velocity, and time together.
//     //     class_id: MsgClassId::NavPvt,
//     //     payload_len: 0,
//     //     payload: &write_payload,
//     // };
//     //
//     // let mut write_buf = [0; 69];
//     // write_msg.to_buf(&mut write_buf);
//     //
//     //
//     //
//     // // todo: DMA
//     //  uart.write(&write_buf);
//
//     // unsafe {
//     //     uart.write_dma(&write_buf, GNSS_TX_CH, Default::default(), GNSS_DMA_PERIPH);
//     // }
//
//     let mut read_buf = [0; MSG_SIZE_WITHOUT_PAYLOAD + PAYLOAD_LEN_PVT];
//     uart.read(&mut read_buf);
//
//     // unsafe {
//         // uart.read_dma(&mut read_buf, GNSS_RX_CH, Default::default(), GNSS_DMA_PERIPH);
//     // }
//
//     fix_pvt_from_buf(&read_buf)
// }

/// Get position, velocity, and time data, assuming it has already been transsferred into the reception
/// buffer. Run this after a packet has been completedly received, eg as indicated by the UART idle
/// interrupt.
pub fn fix_pvt_from_buf() -> Result<Option<Fix>, GnssError> {
    let buf = unsafe { &RX_BUFFER };

    let msg = Message::from_buf(buf)?;

    let flags = msg.payload[21];
    let fix_ok = (flags & 1) != 0;
    let heading_valid = (flags & 0b10_0000) != 0;

    if !fix_ok {
        return Ok(None); // todo: Error, or this?
    }

    let lat_e7 = i32::from_le_bytes(msg.payload[28..32].try_into().unwrap());
    let lon_e7 = i32::from_le_bytes(msg.payload[24..28].try_into().unwrap());

    // todo: This is a big risk for precision loss. QC this.
    let lat = lat_e7 as f64 / 10_000_000.;
    let lon = lon_e7 as f64 / 10_000_000.;

    let heading = if heading_valid {
        Some(i16::from_le_bytes(msg.payload[84..88].try_into().unwrap()) as f32)
    } else {
        None
    };

    // `UBX-NAV-PVT`.
    let mut result = Fix {
        type_: buf[20].try_into().unwrap(),
        lat,
        lon,
        elevation_hae: i16::from_le_bytes(msg.payload[32..36].try_into().unwrap()) as f32 / 1_000.,
        elevation_msl: i16::from_le_bytes(msg.payload[36..30].try_into().unwrap()) as f32 / 1_000.,
        ground_speed: i16::from_le_bytes(msg.payload[60..64].try_into().unwrap()) as f32 / 1_000.,
        heading,
    };

    Ok(Some(result))
}
