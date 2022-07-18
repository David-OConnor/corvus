//! Contains tools to construct and send MSP packets: Multiwii Serial Protocol Version 2.
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
const CRC_POLY: u8 = 0xd;

const PREAMBLE_0: u8 = 0x24;
const PREAMBLE_1: u8 = 0x58;

const MAX_BUF_LEN: usize = OSD_CONFIG_SIZE;

const FRAME_START_SIZE: usize = 5;

const CRC_SIZE: usize = 1;

#[derive(Copy, Clone)]
#[repr(u8)]
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
    // /// uint8, flag, usage to be defined (set to zero)
    // pub flag: u8,
    /// (little endian). 0 - 255 is the same function as V1 for backwards compatibility
    pub function: Function,
    /// (little endian) payload size in bytes
    pub payload_size: u16,
    // We don't store payload here, since it varies in size.
    // n (up to 65535 bytes) payload
    // pub payload: []
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
        buf[1] = PREAMBLE_1;
        buf[2] = self.message_type as u8;
        buf[3] = 0; // Flag currently unimplemented in protocol
                    // todo: QC direction on this!
        buf[4] = ((self.function as u16) >> 8) as u8;
        buf[5] = (self.function as u16) as u8;
        buf[6] = (self.payload_size >> 8) as u8;
        buf[7] = self.payload_size as u8;


        // todo: A diff impl shows this conflicting format:
        // 	packet[0] = '$';
        // 	packet[1] = 'M';
        // 	packet[2] = '<';
        // 	packet[3] = payload_size;
        // 	packet[4] = message_id;

        for i in 0..self.payload_size as usize {
            buf[i + 8] = payload[i];
        }

        // todo: CRC init?? With what poly?
        // let crc = util::calc_crc(unsafe { &CRC_LUT }, &payload, self.payload_size as u8);
        // todo: What is the first argument to this CRC algo??
        // let crc = util::crc8_dvb_s2(payload[0], self.payload_size as u8);

        let message_id = 0; // todo: What should this be?
        let mut crc = self.payload_size as u8 ^ message_id;

        // QC this. Likely not correct?
        for i in 0..self.payload_size as usize {
            crc ^= buf[FRAME_START_SIZE + i];
        }

        buf[8 + self.payload_size as usize] = crc;
    }
}

/// Send a packet on the UART line
/// todo: Try to get this working with DMA.
pub fn send_packet(
    uart: &mut Usart<USART2>,
    dma_chan: DmaChannel,
    dma: &mut Dma<DMA1>,
    packet: &Packet,
    payload: &[u8],
) {
    // todo: Don't always send this max buffer size. Figure out the best way to send packets
    // todo only of the required len.
    let mut buf = [0; MAX_BUF_LEN];
    packet.to_buf(payload, &mut buf);

    unsafe { uart.write_dma(&buf, dma_chan, Default::default(), dma) };
}
