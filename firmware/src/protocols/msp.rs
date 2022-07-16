//! Multiwii Serial Protocol Version 2. We use this to send data to the DJI OSD.
//! https://github.com/iNavFlight/inav/wiki/MSP-V2
//!
//! todo: Relation between this file and `osd_alt`. todo: Maybe keep this separate, since
//! todo it may be also used for onboard ELRS?

// todo: Trim this down to what you need, eg for OSD.

use stm32_hal2::{pac::USART2, usart::Usart};

use crate::util;

static mut CRC_LUT: [u8; 256] = [0; 256];
const CRC_POLY: u8 = 0xd;

const PREAMBLE_0: u8 = 0x24;
const PREAMBLE_1: u8 = 0x58;

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum MessageType {
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
    pub message_type: MessageType,
    // /// uint8, flag, usage to be defined (set to zero)
    // pub flag: u8,
    /// (little endian). 0 - 255 is the same function as V1 for backwards compatibility
    pub function: u16,
    /// (little endian) payload size in bytes
    pub payload_size: u16,
    // We don't store payload here, since it varies in size.
    // n (up to 65535 bytes) payload
    // pub payload: []
}

impl Packet {
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
        buf[4] = (self.function >> 8) as u8;
        buf[5] = self.function as u8;
        buf[6] = (self.payload_size >> 8) as u8;
        buf[7] = self.payload_size as u8;

        for i in 0..self.payload_size as usize {
            buf[i + 8] = payload[i];
        }

        // todo: Is len whole packet len, or just payload?
        // todo: CRC init?? With what poly?
        // let crc = util::calc_crc(unsafe { &CRC_LUT }, &payload, self.payload_size as u8);
        // todo: What is the first argument to this CRC algo??
        let crc = util::crc8_dvb_s2(payload[0], self.payload_size as u8);

        buf[8 + self.payload_size as usize] = crc;
    }
}

/// Send a packet on the UART line
/// todo: Try to get this working with DMA.
pub fn send_packet(uart: &mut Usart<USART2>, packet: &Packet, payload: &[u8]) {
    let mut buf = [0; 69]; // todo: Size.
    packet.to_buf(payload, &mut buf);

    uart.write(&buf);
    // uart.write_dma(
    //     &buf,
    //     write_chan,
    //     Default::default(),
    //     dma,
    // );
}
