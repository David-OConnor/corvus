#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/MSP/msptypes.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/MSP/msp.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/MSP/msp.cpp

use defmt::println;

// todo: These are defines; unkonwn type.
const ELRS_FUNC: u16 =       0x4578; // ['E','x']

const SET_RX_CONFIG: u16 =    45;
const VTX_CONFIG : u16 =      88 ;  //out message         Get vtx settings - betaflight
const SET_VTX_CONFIG : u16 =  89 ;  //in message          Set vtx settings - betaflight
const EEPROM_WRITE : u16 =    250 ; //in message          no param

// ELRS specific opcodes
const ELRS_RF_MODE    : u16 =                 0x06;
const ELRS_TX_PWR     : u16 =                 0x07;
const ELRS_TLM_RATE    : u16 =                0x08;
const ELRS_BIND       : u16 =                 0x09;
const ELRS_MODEL_ID     : u16 =               0x0A;
const ELRS_REQU_VTX_PKT    : u16 =            0x0B;
const ELRS_SET_TX_BACKPACK_WIFI_MODE: u16 =   0x0C;
const ELRS_SET_VRX_BACKPACK_WIFI_MODE: u16 =  0x0D;
const ELRS_SET_RX_WIFI_MODE    : u16 =        0x0E;

const ELRS_POWER_CALI_GET    : u16 =          0x20;
const ELRS_POWER_CALI_SET    : u16 =          0x21;

// CRSF encapsulated msp defines
const ENCAPSULATED_HEADER_CRC_LEN  : u16 =    4;
const ENCAPSULATED_MAX_PAYLOAD_SIZE : u16 =   4;
const ENCAPSULATED_MAX_FRAME_LEN   : u16 =    ENCAPSULATED_HEADER_CRC_LEN + ENCAPSULATED_MAX_PAYLOAD_SIZE;


// TODO: PORT_INBUF_SIZE should be changed to
// dynamically allocate array length based on the payload size
// Hardcoding payload size to 8 bytes for now, since MSP is
// limited to a 4 byte payload on the BF side
const PORT_INBUF_SIZE: usize =  8; // todo: define type?

// #define CHECK_PACKET_PARSING() \
//   if (packet->readError) {\
//     return;\
//   }

#[derive(Clone, Copy, PartialEq)]
enum State {
    IDLE,
    HEADER_START,
    HEADER_X,

    HEADER_V2_NATIVE,
    PAYLOAD_V2_NATIVE,
    CHECKSUM_V2_NATIVE,

    COMMAND_RECEIVED
}

#[derive(Clone, Copy, PartialEq)]
enum PacketType {
    UNKNOWN,
    COMMAND,
    RESPONSE
}

// typedef struct __attribute__((packed)) { // todo: Anything special to pack?
pub struct HeaderV2 {
    flags: u8,
    function: u16,
    payloadSize: u16,
}

pub struct Packet {
    type_: PacketType,
    flags: u8,
    function: u16,
    payloadSize: usize,
    payload: [u8; PORT_INBUF_SIZE],
    payloadReadIterator: usize,
    readError: bool,
}

impl Packet {
    fn reset(&mut self)    {
        self.type_ = PacketType::UNKNOWN;
        self.flags = 0;
        self.function = 0;
        self.payloadSize = 0;
        self.payloadReadIterator = 0;
        self.readError = false;
    }

    fn addByte(&mut self, b: u8) {
        self.payloadSize += 1;
        self.payload[payloadSize] = b;
    }

    fn makeResponse(&mut self)    {
        self.type_ = PacketType::RESPONSE;
    }

    fn makeCommand(&mut self)    {
        self.type_ = PacketType::COMMAND;
    }

    fn readByte(&mut self) -> u8    {
        if self.payloadReadIterator >= self.payloadSize {
            // We are trying to read beyond the length of the payload
            self.readError = true;
            return 0;
        }

        self.payloadReadIterator += 1;
        payload[payloadReadIterator]
    }
}

/////////////////////////////////////////////////

struct MSP {
    inputState: State,
    offset: u16,
    inputBuffer: [u8; PORT_INBUF_SIZE],
    crc: u8,
}


/* ==========================================
MSP V2 Message Structure:
Offset: Usage:         In CRC:  Comment:
======= ======         =======  ========
0       $                       Framing magic start char
1       X                       'X' in place of v1 'M'
2       type                    '<' / '>' / '!' Message Type (TODO find out what ! type is)
3       flag           +        uint8, flag, usage to be defined (set to zero)
4       function       +        uint16 (little endian). 0 - 255 is the same function as V1 for backwards compatibility
6       payload size   +        uint16 (little endian) payload size in bytes
8       payload        +        n (up to 65535 bytes) payload
n+8     checksum                uint8, (n= payload size), crc8_dvb_s2 checksum
========================================== */

// CRC helper function. External to MSP class
// TODO: Move all our CRC functions to a CRC lib
fn crc8_dvb_s2(crc: u8, a: char) -> u8 {
    // crc ^= a;
    let mut crc = crc ^ a;
    for i in 0..8 {
        if crc & 0x80 != 0 {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    crc
}

impl MSP {


    fn processReceivedByte(&mut self, c: u8) -> bool {
        match self.inputState {
            State::IDLE => {
                // Wait for framing char
                if c == '$' {
                    self.inputState = State::HEADER_START;
                }
            }
            State::HEADER_START => {
                // Waiting for 'X' (MSPv2 native)
                match c {
                    'X' => {
                        self.inputState = State::HEADER_X;
                    }
                    _ => {
                        self.inputState = State::IDLE;
                    }
                }
            }
            State::HEADER_X => {
                // Wait for the packet type (cmd or req)
                self.inputState = State::HEADER_V2_NATIVE;

                // Start of a new packet
                // reset the packet, offset iterator, and CRC
                self.packet.reset();
                self.offset = 0;
                self.crc = 0;

                match c {
                    '<' => {
                        self.packet.type_ = PacketType::COMMAND;
                    }
                    '>' => {
                        self.packet.type_ = PacketType::RESPONSE;
                    }
                    _ => {
                        self.packet.type_ = PacketType::UNKNOWN;
                        self.inputState = State::IDLE;
                    }
                }
            }

            State::HEADER_V2_NATIVE => {
                // Read bytes until we have a full header
                self.offset += 1;
                self.inputBuffer[self.offset] = c;
                self.crc = crc8_dvb_s2(self.crc, c);

                // If we've received the correct amount of bytes for a full header
                if self.offset == sizeof(mspHeaderV2_t) {
                    // Copy header values into packet
                    let mut header: HeaderV2 = m_inputBuffer[0];
                    self.packet.payloadSize = header.payloadSize;
                    self.packet.function = header.function;
                    self.packet.flags = header.flags;
                    // reset the offset iterator for re-use in payload below
                    self.offset = 0;
                    self.inputState = State::PAYLOAD_V2_NATIVE;
                }
            }
            State::PAYLOAD_V2_NATIVE => {
                // Read bytes until we reach payloadSize
                self.offset += 1;
                self.offset += 1;
                self.packet.payload[self.offset] = c;
                self.crc = crc8_dvb_s2(m_crc, c);

                // If we've received the correct amount of bytes for payload
                if m_offset == self.packet.payloadSize {
                    // Then we're up to the CRC
                    self.inputState = State::CHECKSUM_V2_NATIVE;
                }
            }
            State::CHECKSUM_V2_NATIVE => {
                // Assert that the checksums match
                if self.crc == c {
                    self.inputState = State::COMMAND_RECEIVED;
                } else {
                    println!("CRC failure on MSP packet - Got %d expected %d", c, m_crc);
                    self.inputState = State::IDLE;
                }
            }

            _ => {
                self.inputState = State::IDLE;
            }
        }

        // If we've successfully parsed a complete packet
        // return true so the calling function knows that
        // a new packet is ready.
        if self.inputState == State::COMMAND_RECEIVED {
            return true;
        }
        return false;
    }

    fn getReceivedPacket(&self) -> Packet {
        self.packet
    }

    fn markPacketReceived(&mut self)    {
    // Set input state to idle, ready to receive the next packet
    // The current packet data will be discarded internally
    self.inputState = State::IDLE;
    }

    // todo: This involves a serial write, I think. Probably not what we want?
    // fn sendPacket(packet: Packet, Stream* port) -> bool {
    //     // Sanity check the packet before sending
    //     if packet.type_ != PacketType::COMMAND && packet.type_ != PacketType::RESPONSE {
    //         // Unsupported packet type (note: ignoring '!' until we know what it is)
    //         return false;
    //     }
    //
    //     if packet.type_ == PacketType::RESPONSE && packet.payloadSize == 0 {
    //         // Response packet with no payload
    //         return false;
    //     }
    //
    //     // Write out the framing chars
    //     port.write('$');
    //     port.write('X');
    //
    //     // Write out the packet type
    //     if packet.type_ == PacketType::COMMAND {
    //         port.write('<');
    //     }
    //     else if packet.type_ == PacketType::RESPONSE {
    //         port.write('>');
    //     }
    //
    //     // Subsequent bytes are contained in the crc
    //     let mut crc: u8 = 0;
    //
    //     // Pack header struct into buffer
    //     uint8_t headerBuffer[5];
    //     mspHeaderV2_t* header = (mspHeaderV2_t*)&headerBuffer[0];
    //     header.flags = packet->flags;
    //     header.function = packet->function;
    //     header.payloadSize = packet->payloadSize;
    //
    //     // Write out the header buffer, adding each byte to the crc
    //     for (uint8_t i = 0; i < sizeof(mspHeaderV2_t); ++i) {
    //         port.write(headerBuffer[i]);
    //         crc = crc8_dvb_s2(crc, headerBuffer[i]);
    //     }
    //
    //     // Write out the payload, adding each byte to the crc
    //     for (uint16_t i = 0; i < packet->payloadSize; ++i) {
    //         port.write(packet->payload[i]);
    //         crc = crc8_dvb_s2(crc, packet->payload[i]);
    //     }
    //
    //     // Write out the crc
    //     port.write(crc);
    //
    //     return true;
    // }
    //

}