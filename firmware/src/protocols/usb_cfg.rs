//! This module describes the custom profile we use to exchange data between the flight
//! controller, and a PC running configuration software, over USB.
//!
//! Format - Byte 0: message type. Byte 1: payload len. Byte -1: CRC. Rest: payload
//!

// const START_BYTE: u8 =

// todo: Should we use this module and/or a similar structure for data exchange over RF,
// todo: beyond the normal control info used by ELRS? (Eg sending a route, autopilot data etc)

use crate::{flight_ctrls::Params, util};

// Note: LUT is here, since it depends on the poly.
static mut CRC_LUT: [u8; 256] = [0; 256];
const CRC_POLY: u8 = 0xab;

// #[derive(Clone, Copy)]
// /// Repr is payload size.
// pub enum RxMsgType {
//     Params,
//     SetMotorDir
// }
//
// #[derive(Clone, Copy)]
// /// Repr is payload size.
// pub enum TxMsgType {
//     ReqParams,
// }

// todo: init LUT.
// util::crc_init(unsafe { &mut CRC_LUT }, CRC_POLY);

struct DecodeError {}
// struct CrcError {} todo?

#[derive(Clone, Copy)]
/// Repr is payload size.
pub enum MsgType {
    // Transmit from FC
    Params,
    SetMotorDirs,
    // Receive to FC
    ReqParams,
    /// Acknowledgement, eg in response to setting something.
    Ack,
}

impl MsgType {
    pub fn payload_size(&self) -> usize {
        match self {
            Self::Params => 10,
            Self::SetMotorDirs => 1, // Packed bits: motors 1-4, R-L. True = CW.
            Self::ReqParams => 0,
            Self::Ack => 0,
        }
    }
}

pub struct Packet {
    message_type: MsgType,
    payload: [u8; 69], // todo?
    crc: u8,
}

// todo
// impl From<[u8; 69]> for Packet {
//     let expected_crc = util::calc_crc(
//         unsafe { &CRC_LUT },
//         &payload[2..payload.len() - 1],
//         payload.len() as u8 - 3,
//     );
//
//     fn from(v: [u8; 69]) -> Self {
//
//     }
// }
//
// impl From<Packet> for [u8; 69] {
//     fn from(v: Packet) -> Self {
//
//     }
// }
//
// impl From<[u8; 69]> for Params {
//     fn from(v: [u8; 69]) -> Self {
//
//     }
// }
//
// impl From<Params> for [u8; 69] {
//     fn from(v: Params) -> Self {
//
//     }
// }
