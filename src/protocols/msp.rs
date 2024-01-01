//! Contains tools to construct and send MSP packets: Multiwii Serial Protocol. Supports
//! both V1 and V2.
//! We use this to send data to OSD via MSP Displayport, but this module supports MSP broadly.
//!
//! [Reference](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)
//!  Some general MSP example code:
//! https://github.com/chris1seto/PX4-Autopilot/tree/turbotimber/src/modules/msp_osd
//!
//! List of message types: https://github.com/betaflight/betaflight/blob/5d29a0be0656303641b7004edcf8f22c770c5d22/src/main/msp/msp_protocol.h#L98

#![allow(dead_code)] // todo: So we can comment-out the V2 or V1 code as required.

use defmt::println;

use crate::{
    setup::{UartOsd, OSD_DMA_PERIPH, OSD_TX_CH},
    util,
};

// const CRC_POLY: u8 = 0xd;
// The poly and LUT are for V2.
const CRC_POLY: u8 = 0x0; // todo: WHich one, this or the above?
const CRC_LUT: [u8; 256] = util::crc_init(CRC_POLY);

pub const PREAMBLE_0: u8 = 0x24; // aka $
const PREAMBLE_1_V1: u8 = 0x4d; // aka M
const PREAMBLE_1_V2: u8 = 0x58; // aka X

const FRAME_START_I_V1: usize = 5;
const FRAME_START_I_V2: usize = 8;

const CRC_SIZE_V1: usize = 1;
const CRC_SIZE_V2: usize = 2;

// The size of the packet not including the payload. Used for initializing buffers for
// individual messages.
pub const METADATA_SIZE_V1: usize = FRAME_START_I_V1 + CRC_SIZE_V1;
pub const METADATA_SIZE_V2: usize = FRAME_START_I_V2 + CRC_SIZE_V2;

pub const MSG_ID_DP: u8 = 182;
pub const MSG_ID_STATUS: u8 = 101;
pub const MSG_ID_FC_TYPE: u8 = 3;

#[derive(Copy, Clone)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Direction {
    /// Sent to the flight controller, by the transmitter
    VtxToFc = 0x3c, // aka <
    /// For all messages we send to the video transmitter
    FcToVtx = 0x3e, // aka >
    /// Sent or processed by either. Response to receipt of data that cannot be processed
    /// (corrupt checksum, unknown function, message type that cannot be processed)
    Error = 0x21, // aka !
}

/// A MSP packet (V1 or V2)
pub struct Packet<'a> {
    /// Request, response, or error
    pub direction: Direction,
    /// (little endian). 0 - 255 is the same function as V1 for backwards compatibility
    pub function: u16,
    /// (little endian) Payload size in bytes. (8-bits for V1)
    pub payload_size: u16,
    /// The payload.
    pub payload: &'a [u8],
}

impl<'a> Packet<'a> {
    pub fn new(
        message_type: Direction,
        function: u16,
        payload_size: usize,
        payload: &'a [u8],
    ) -> Self {
        Self {
            direction: message_type,
            function,
            payload_size: payload_size as u16,
            payload,
        }
    }

    /// Convert this payload to a buffer in MSP V1 format. See `to_buf` for
    /// a description and comments.
    pub fn to_buf_v1(&self, buf: &mut [u8]) {
        // The first two bytes are a hard-set preamble.
        buf[0] = PREAMBLE_0;
        buf[1] = PREAMBLE_1_V1;
        buf[2] = self.direction as u8;
        buf[3] = self.payload_size as u8;
        // `function` is called `message_id` in v1.
        buf[4] = self.function as u8;

        buf[FRAME_START_I_V1..FRAME_START_I_V1 + self.payload_size as usize]
            .copy_from_slice(&self.payload);

        let mut crc = self.payload_size as u8 ^ (self.function as u8);
        for i in 0..self.payload_size as usize {
            crc ^= self.payload[i];
        }

        buf[FRAME_START_I_V1 + self.payload_size as usize] = crc;
    }

    /// Convert this payload to a buffer in MSP V2 format, modifying the argument
    /// `buf` in place. Payload is passed here instead of as a struct field
    /// due to its variable size.
    pub fn _to_buf_v2(&self, buf: &mut [u8]) {
        // The first two bytes are a hard-set preamble.
        buf[0] = PREAMBLE_0;
        buf[1] = PREAMBLE_1_V2;
        buf[2] = self.direction as u8;
        buf[3] = 0; // `Flag` is currently unimplemented in the protocol
        buf[4..5].clone_from_slice(&self.function.to_le_bytes());
        buf[5..6].clone_from_slice(&self.payload_size.to_le_bytes());

        buf[FRAME_START_I_V2..FRAME_START_I_V2 + self.payload_size as usize]
            .copy_from_slice(&self.payload);

        // The CRC includes the payload size, frame ID, and payload.
        let crc_payload = &buf[3..self.payload_size as usize + 3];
        let crc = util::calc_crc(&CRC_LUT, &crc_payload, self.payload_size as u8 + 2);

        buf[FRAME_START_I_V2 + self.payload_size as usize] = crc;
    }

    pub fn _send_v1(&self, buf: &mut [u8], uart: &mut UartOsd) {
        self.to_buf_v1(buf);
        unsafe { uart.write_dma(&buf, OSD_TX_CH, Default::default(), OSD_DMA_PERIPH) };
    }

    // todo: DRY
    pub fn _send_v2(&self, buf: &mut [u8], uart: &mut UartOsd) {
        self._to_buf_v2(buf);
        unsafe { uart.write_dma(&buf, OSD_TX_CH, Default::default(), OSD_DMA_PERIPH) };
    }
}
