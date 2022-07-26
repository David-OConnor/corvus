//! Contains tools to construct and send MSP packets: Multiwii Serial Protocol. Supports
//! both V1 and V2.
//! We use this to send data to the DJI OSD, but this module supports MSP broadly.
//!
//! For specific MSP 'function' types, and their serialization, see the `msp_defines` module.
//!
//! [Reference](https://github.com/iNavFlight/inav/wiki/MSP-V2)

use stm32_hal2::{
    dma::{Dma, DmaChannel},
    pac::{DMA1, USART2},
    usart::Usart,
};

use crate::{
    protocols::msp_defines::{Function, OSD_CONFIG_SIZE},
    util,
};

static mut CRC_LUT: [u8; 256] = [0; 256];
// const CRC_POLY: u8 = 0xd;
const CRC_POLY: u8 = 0x0; // todo: WHich one, this or the above?

const PREAMBLE_0: u8 = 0x24;
const PREAMBLE_1_V2: u8 = 0x58;
const PREAMBLE_1_V1: u8 = 0x4d;

// The size of the packet not including the payload. Used for initializing buffers for
// individual messages.
pub const METADATA_SIZE: usize = 9;
pub const METADATA_SIZE_V1: usize = 6;

const CRC_SIZE: usize = 1;

#[derive(Copy, Clone)]
#[repr(u8)]
#[allow(dead_code)]
pub enum MsgType {
    /// Sent by master, processed by slave
    Request = 0x3c, // aka <
    /// Sent by slave, processed by master. Only sent in response to a request
    Response = 0x3e, // aka >
    /// Sent or processed by either. Response to receipt of data that cannot be processed
    /// (corrupt checksum, unknown function, message type that cannot be processed)
    Error = 0x21, // aka !
}

/// A MSP v2 packet
pub struct Packet {
    /// Request, response, or error
    pub message_type: MsgType,
    /// (little endian). 0 - 255 is the same function as V1 for backwards compatibility
    pub function: Function,
    /// (little endian) Payload size in bytes. (8-bits for V1)
    pub payload_size: u16,
    // We don't store payload here, since it varies in size. (up to 65535 bytes)
}

impl Packet {
    pub fn new(message_type: MsgType, function: Function, payload_size: usize) -> Self {
        Self {
            message_type,
            function,
            payload_size: payload_size as u16,
        }
    }

    /// Convert this payload to a buffer in MSP V2 format, modifying the argument
    /// `buf` in place. Payload is passed here instead of as a struct field
    /// due to its variable size.
    pub fn to_buf(&self, payload: &[u8], buf: &mut [u8]) {
        // The first two bytes are a hard-set preamble.
        buf[0] = PREAMBLE_0;
        buf[1] = PREAMBLE_1_V2;
        buf[2] = self.message_type as u8;
        buf[3] = 0; // `Flag` is currently unimplemented in the protocol
        buf[4..5].clone_from_slice(&(self.function as u16).to_le_bytes());
        buf[5..6].clone_from_slice(&self.payload_size.to_le_bytes());

        for i in 0..self.payload_size as usize {
            buf[i + METADATA_SIZE - 1] = payload[i];
        }

        // The CRC includes the payload size, frame ID, and payload.
        let crc_payload = &buf[3..self.payload_size as usize + 3];
        let crc = util::calc_crc(
            unsafe { &CRC_LUT },
            &crc_payload,
            self.payload_size as u8 + 2,
        );

        buf[METADATA_SIZE - 1 + self.payload_size as usize] = crc;
    }

    /// Convert this payload to a buffer in MSP V1 format. See `to_buf` for
    /// a description and comments.
    pub fn to_buf_v1(&self, payload: &[u8], buf: &mut [u8]) {
        buf[0] = PREAMBLE_0;
        buf[1] = PREAMBLE_1_V1;
        buf[2] = self.message_type as u8;
        buf[3] = self.payload_size as u8;
        // `function` is called `message_id` in v1.
        buf[4] = self.function as u16 as u8;

        for i in 0..self.payload_size as usize {
            buf[i + METADATA_SIZE_V1 - 1] = payload[i];
        }

        let mut crc = self.payload_size as u8 ^ (self.function as u16 as u8);
        for i in 0..self.payload_size as usize {
            crc ^= buf[METADATA_SIZE_V1 - 1 + i];
        }

        buf[METADATA_SIZE_V1 - 1 + self.payload_size as usize] = crc;
    }
}

/// Send a packet on the UART line. Note that we don't use this in practice, since we send
/// a single buf in the `osd` module for all the OSD MSP commands together.
pub fn _send_packet(
    uart: &mut Usart<USART2>,
    dma_chan: DmaChannel,
    dma: &mut Dma<DMA1>,
    packet: &Packet,
    payload: &[u8],
    buf: &mut [u8],
) {
    packet.to_buf(payload, buf);
    unsafe { uart.write_dma(&buf, dma_chan, Default::default(), dma) };
}

/// Send a packet on the UART line. Unusd; see doc comment on `_send_packet`.
pub fn _send_packet_v1(
    uart: &mut Usart<USART2>,
    dma_chan: DmaChannel,
    dma: &mut Dma<DMA1>,
    packet: &Packet,
    payload: &[u8],
    buf: &mut [u8],
) {
    // todo: DRY with `send_packet`.
    packet.to_buf_v1(payload, buf);
    unsafe { uart.write_dma(&buf, dma_chan, Default::default(), dma) };
}

/// Set up MSP.
pub fn setup_crc() {
    util::crc_init(unsafe { &mut CRC_LUT }, CRC_POLY);
}
