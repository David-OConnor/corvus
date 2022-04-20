//!  CRSF support, for receiving radio control signals from ELRS receivers. Only handles communication
//! over serial; the modules handle over-the-air procedures. See also the `elrs` module, for use
//! with the LoRa chip directly, over SPI.
//!
//! [Detailed protocol info](https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol)
//! [WIP clean driver in C](https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c)
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.h
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Serial3.c#L160
//!
//! [Addtional standaone ref](https://github.com/CapnBry/CRServoF/tree/master/lib/CrsfSerial)
//! From BF:
//!         // CRSF protocol uses a single wire half duplex uart connection.
//!         //  * The master sends one frame every 4ms and the slave replies between two frames from the master.
//!         //  *
//!         //  * 420000 baud
//!         //  * not inverted
//!         //  * 8 Bit
//!         //  * 1 Stop bit
//!         //  * Big endian
//!         //  * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
//!         //  * Max frame size is 64 bytes
//!         //  * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
//!
//! Note that there doesn't appear to be a published spec, so we piece together what we can from
//! code and wisdom from those who've done this before.

use num_enum::TryFromPrimitive;
use core::convert::TryFrom;

use stm32_hal2::{
    dma::{Dma, DmaChannel, ChannelCfg, Circular},
    usart::{Usart, UsartInterrupt},
    pac::{USART3, DMA1},
};

use defmt::println;

const CRC_POLY: u8 = 0xd;
const MAX_RX_PAYLOAD_SIZE: usize = 64; // todo
const MAX_TX_PAYLOAD_SIZE: usize = 22; // todo

static mut RX_BUFFER: [u8; MAX_RX_PAYLOAD_SIZE] = [0; MAX_RX_PAYLOAD_SIZE];

// todo: Consider storing static (non-mut) TX packets, depending on variety of responses.
static mut TX_BUFFER: [u8; MAX_TX_PAYLOAD_SIZE] = [0; MAX_TX_PAYLOAD_SIZE];

// todo: Maybe put in a struct etc? It's constant, but we use a function call to populate it.
static mut CRC_LUT: [u8; 256] = [0; 256];

// "All packets are in the CRSF format [dest] [len] [type] [payload] [crc8]"

/// Invalid packet, etc.
struct DecodeError {}

#[derive(Clone, Copy, TryFromPrimitive)]
#[repr(u8)]
/// Destination address, or "sync" byte
enum DestAddr {
    Broadcast = 0x00,
    Usb = 0x10,
    TbsCorePnpPro = 0x80,
    Reserved1 = 0x8a,
    CurrentSensor = 0xc0,
    Gps = 0xc2,
    TbsBlackbox = 0xc4,
    /// Going to the flight controller
    FlightController = 0xc8,
    Reserved2 = 0xca,
    RaceTag = 0xcc,
    /// Going to the handset
    RadioTransmitter = 0xea,
    /// Going to the receiver
    CrsfReceiver = 0xec,
    /// Going to the transmitter module
    CrsfTransmitter = 0xee,
}

// impl TryFrom<u8> for DestAddr {
//     type Error = DecodeError;
//
//     fn try_from(v: u8) -> Result<Self, Self::Error> {
//         match v {
//             0xee => Ok(Self::CrsfTransmitter),
//             0xea => Ok(Self::RadioTransmitter),
//             0xc8 => Ok(Self::FlightController),
//             0xec => Ok(Self::CrsfReceiver),
//             _ => Err(DecodeError {}),
//         }
//     }
// }

#[derive(Clone, Copy, TryFromPrimitive)]
#[repr(u8)]
/// Frame type (packet type?)
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c#L29
enum FrameType {
    // todo: Description of each of these?
    Gps = 0x02,
    BatterySensor = 0x08,
    LinkStatistics = 0x14,
    OpentxSync = 0x10,
    RadioId = 0x3a,
    RcChannelsPacked = 0x16,
    Attitude = 0x1e,
    FlightMode = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    DevicePing = 0x28,
    DeviceInfo = 0x29,
    ParameterSettingsEntry = 0x2b,
    ParameterRead = 0x2c,
    ParameterWrite = 0x2d,
    Command = 0x32,
    // MSP commands
    MspReq = 0x7a,
    MspResp = 0x7b,
    MspWrite = 0x7c,
}

// impl TryFrom<u8> for FrameType {
//     type Error = DecodeError;
//
//     fn try_from(v: u8) -> Result<Self, Self::Error> {
//         match v {
//             0x16 => Ok(Self::RcChannelsPacked),
//             0x28 => Ok(Self::DevicePing),
//             0x29 => Ok(Self::DeviceInfo),
//             0x2b => Ok(Self::ParameterSettingsEntry),
//             0x2c => Ok(Self::ParameterRead),
//             0x2d => Ok(Self::ParameterWrite),
//             _ => Err(DecodeError {}),
//         }
//     }
// }

struct LinkStats {
    timestamp: TickType,
    uplink_rssi_1: u8,
    uplink_rssi_2: u8,
    uplink_link_quality: u8,
    uplink_snr: i8,
    active_antenna: u8,
    rf_mode: u8,
    uplink_tx_power: u8,
    downlink_rssi: u8,
    downlink_link_quality: u8,
    downlink_snr: i8,
}

/// Configure the Idle interrupt, and start the circular DMA transfer. Run this once, on initial
/// firmware setup.
pub fn setup(uart: &mut Usart<USART3>, channel: DmaChannel, dma: &mut Dma<DMA1>) {
    // Idle interrupt, in conjunction with circular DMA, to indicate we're received a message.
    uart.enable_interrupt(UsartInterrupt::Idle),

    unsafe {
        uart.read_dma(
            &mut RX_BUFFER,
            channel,
            ChannelCfg {
                // circular: Circular,
                ..Default::default()
            },
            dma
        );
    }

    crc_init(unsafe { &mut CRC_LUT }, CRC_POLY);
}

struct Packet {
    pub dest_addr: DestAddr,
    // Len, starting with `type`, to the end. Payload len + 2 normally, or + 4 with extended packet.
    pub len: usize,
    pub frame_type: FrameType,
    pub extended_dest: Option<DestAddr>,
    pub extended_src: Option<DestAddr>,
    pub payload: [u8; MAX_RX_PAYLOAD_SIZE], // todo: Could be TX or RX.
    pub crc: u8,
}

impl Packet {
    /// Decode a packet, from the buffer.
    pub fn from_buf(buf: &[u8], start_i: usize) -> Result<Self, DecodeError> {
        let dest: DestAddr = buf[start_i].try_into()?;

        // Byte 1 is the length of bytes to follow, ie type (1 byte), payload (determined
        // by this), and crc (1 byte)
        // Overall packet length is PayloadLength + 4 (dest, len, type, crc),
        let len = buf[start_i + 1] as usize;
        let payload_len = len - 2; // todo: Take extended into acct!

        let frame_type: FrameType = buf[start_i + 2].try_into()?;

        let mut payload = [0; MAX_RX_PAYLOAD_SIZE];

        // todo: Handle wrapping!

        // todo: Extended src/dest

        payload[0..payload_len] = buf[start_i + 3..start_i + 3 + payload_len];

        let crc = buf[start_i + 3 + payload_len];

        Ok(Packet { dest_addr, len, frame_type, extended_dest: None, extended_src: None, payload, crc })
    }
    
    pub fn to_buf(&self, buf: &mut u8) {
        buf[0] = self.dest_addr as u8;
        buf[1] = self.len as u8;
        buf[2] = self.frame_type as u8;

        let mut i = 3;
        if let Some(dest) = self.extended_dest {
            buf[i] = dest as u8;
            i += 1;
        }
        if let Some(src) = self.extended_src {
            buf[i] = src as u8;
            i += 1;
        }

        buf[i..i + payload_len] = self.payload;
        buf[3 + payload_len = self.crc];
    }
}

/// Handle an incomming packet. Triggered whenever the line goes idle.
pub fn handle_packet(uart: &mut Usart<USART3>, dma: &mut Dma<DMA1>, rx_chan: DmaChannel, tx_chan: DmaChannel) {
    uart.clear_interrupt(UsartInterrupt::Idle);
    dma.stop(rx_chan);

    // Note: As long as we're stopping and starting the transfer each packet, `start_i` will
    // always be 0.
    let start_i = 0;

    // todo: How do you guarantee you don't have a write in progress?

    let packet = Packet::from_buf(unsafe { &RX_BUFFER }, start_i);

    match packet.dest {
        DestAddr::FlightController => (),
        // Improper destination address from the sender.
        _ => return
    }

        match packet.frame_type {
            FrameType::DevicePing => {
                // Send a reply.
                // todo: Consider hard coding this as a dedicated buffer instead of calculating each time.
                let mut payload = [0; MAX_RX_PAYLOAD_SIZE];
                #[rustfmt::skip]
                payload[0..22] = [
                    // Display name - "Anyleaf", null-terminated
                    0x41, 0x6e, 0x79, 0x6c, 0x65, 0x61, 0x66, 00,
                    0x45, 0x4c, 0x52, 0x53, // Serial number - "ELRS"
                    0, 0, 1, 0,  // Hardware version
                    0, 0, 1, 0, // Software version,
                    0x13, // Number of config params. todo
                    0, // parameter protocol version. todo.
                ];

                let response = Packet {
                    dest_addr: DestAddr::RadioTransmitter,
                    len: 4,
                    frame_type: FrameType::DeviceInfo,
                    extended_dest: Some(DestAddr::RadioTransmitter),
                    extended_src: Some(DestAddr::CrsfTransmitter),
                    payload,
                    crc: calc_crc(unsafe { &CRC_LUT }),
                };

                unsafe {
                    response.to_buf(&mut TX_BUFFER);

                    uart.write_dma(
                        &TX_BUFFER,
                        tx_chan,
                        Default::default(),
                        dma,
                    );
                }
            }
            FrameType::ParameterRead => {

            }
            FrameType::ParameterSettingsEntry => {

            }
            FrameType::ParameterWrite => {

            }
            _ => (),
        }

    // Start the next transfer.
    unsafe {
        uart.read_dma(
            &mut RX_BUFFER,
            rx_chan,
            Default::default(),
            dma
        );
    }
}

/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
fn crc_init(lut: &mut [u8; 256], poly: u8) {
    for i in 0..256 {
        let mut crc = idx as u8;
        for _ in 0..8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
        }
        lut[i] = crc & 0xff;
    }
}

/// CRC8 using poly 0xD5, includes all bytes from type (buffer[2]) to end of payload.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
fn calc_crc(lut: &[u8; 256], data: &[u8], mut size: u8) -> u8 {
    let mut crc = 0;
    let mut i = 0;

    while size > 0 {
        size -= 1;
        i += 1;
        crc = lut[crc ^ data[i]];
    }
    crc
}