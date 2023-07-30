//! Contains tools to construct and send MSP packets: Multiwii Serial Protocol. Supports
//! both V1 and V2.
//! We use this to send data to OSD via MSP Displayport, but this module supports MSP broadly.
//!
//! [Reference](https://github.com/iNavFlight/inav/wiki/MSP-V2)
//!  Some general MSP example code:
//! https://github.com/chris1seto/PX4-Autopilot/tree/turbotimber/src/modules/msp_osd

#![allow(dead_code)] // todo: So we can comment-out the V2 or V1 code as required.

use stm32_hal2::{dma::DmaChannel, pac::USART2, usart::Usart};

use crate::{
    setup::{UartOsd, OSD_DMA_PERIPH},
    util,
};

// const CRC_POLY: u8 = 0xd;
const CRC_POLY: u8 = 0x0; // todo: WHich one, this or the above?
const CRC_LUT: [u8; 256] = util::crc_init(CRC_POLY);

const PREAMBLE_0: u8 = 0x24; // aka $
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

// const CRC_SIZE: usize = 1;

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

/// A MSP packet (V1 or V2)
pub struct Packet<'a> {
    /// Request, response, or error
    pub message_type: MsgType,
    /// (little endian). 0 - 255 is the same function as V1 for backwards compatibility
    pub function: u16,
    /// (little endian) Payload size in bytes. (8-bits for V1)
    pub payload_size: u16,
    /// The payload.
    pub payload: &'a [u8],
}

impl<'a> Packet<'a> {
    pub fn new(
        message_type: MsgType,
        function: u16,
        payload_size: usize,
        payload: &'a [u8],
    ) -> Self {
        Self {
            message_type,
            function,
            payload_size: payload_size as u16,
            payload,
        }
    }

    /// Convert this payload to a buffer in MSP V1 format. See `to_buf` for
    /// a description and comments.
    pub fn to_buf_v1(&self, buf: &mut [u8]) {
        buf[0] = PREAMBLE_0;
        buf[1] = PREAMBLE_1_V1;
        buf[2] = self.message_type as u8;
        buf[3] = self.payload_size as u8;
        // `function` is called `message_id` in v1.
        buf[4] = self.function as u8;

        buf[FRAME_START_I_V1..FRAME_START_I_V1 + self.payload_size as usize]
            .copy_from_slice(&self.payload);

        let mut crc = self.payload_size as u8 ^ (self.function as u8);
        for i in 0..self.payload_size as usize {
            crc ^= buf[FRAME_START_I_V1 + i];
        }

        buf[FRAME_START_I_V1 + self.payload_size as usize] = crc;
    }

    /// Convert this payload to a buffer in MSP V2 format, modifying the argument
    /// `buf` in place. Payload is passed here instead of as a struct field
    /// due to its variable size.
    pub fn to_buf_v2(&self, buf: &mut [u8]) {
        // The first two bytes are a hard-set preamble.
        buf[0] = PREAMBLE_0;
        buf[1] = PREAMBLE_1_V2;
        buf[2] = self.message_type as u8;
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

    pub fn send_v1(&self, buf: &mut [u8], uart: &mut UartOsd, dma_chan: DmaChannel) {
        self.to_buf_v1(buf);
        // todo: Switch back to DMA.
        // unsafe { uart.write_dma(&buf, dma_chan, Default::default(), setup::OSD_DMA_PERIPH) };
        uart.write(buf);
    }

    // todo: DRY
    pub fn send_v2(&self, buf: &mut [u8], uart: &mut UartOsd, dma_chan: DmaChannel) {
        self.to_buf_v2(buf);
        // todo: Switch back to DMA.
        // unsafe { uart.write_dma(&buf, dma_chan, Default::default(), setup::OSD_DMA_PERIPH) };
        uart.write(buf);
    }
}
