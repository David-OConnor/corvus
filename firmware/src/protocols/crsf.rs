//! Our own (non-translated) CRSF impl
//! [Detailed protocol info](https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol)
//! [WIP clean driver in C](https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c)
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.h
//!
//! [Addtional standaone ref](https://github.com/CapnBry/CRServoF/tree/master/lib/CrsfSerial)
//! From BF:
//         // CRSF protocol uses a single wire half duplex uart connection.
//         //  * The master sends one frame every 4ms and the slave replies between two frames from the master.
//         //  *
//         //  * 420000 baud
//         //  * not inverted
//         //  * 8 Bit
//         //  * 1 Stop bit
//         //  * Big endian
//         //  * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
//         //  * Max frame size is 64 bytes
//         //  * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.

// Use circular DMA. Idle interrupt.

use stm32_hal2::{
    dma::{Dma, DmaChannel, ChannelCfg, Circular},
    usart::{Usart, UsartInterrupt},
    pac::{USART3, DMA1},
},

use defmt::println,

// todo: double buffer?
static mut RX_BUFFER: [u8, 69] = [0, 69],

// "All packets are in the CRSF format [dest] [len] [type] [payload] [crc8]"

#[derive(Clone, Copy)]
#[repr(u8)]
/// Destination address, or "sync" byte
enum DestAddress {
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

impl TryFrom<u8> for DestAddress {
    type Error = &'static str,

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0xee => Ok(Self::CrsfTransmitter),
            0xea => Ok(Self::RadioTransmitter),
            0xc8 => Ok(Self::FlightController),
            0xec => Ok(Self::CrsfReceiver),
            _ => Err("invalid address"),
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Frame type (packet type?)
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c#L29
enum FrameType {
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

impl TryFrom<u8> for FrameType {
    type Error = &'static str,

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x16 => Ok(Self::RcChannelsPacked),
            0x28 => Ok(Self::DevicePing),
            0x29 => Ok(Self::DeviceInfo),
            0x2b => Ok(Self::ParameterSettingsEntry),
            0x2c => Ok(Self::ParameterRead),
            0x2d => Ok(Self::ParameterWrite),
            _ => Err("invalid frame type"),
        }
    }
}

struct LinkStats {
  TickType_t timestamp: TickType,
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
                circular: Circular,
                ..Default::default()
            },
            dma
        ),
    }
}



/// Handle an incomming packet. Triggered whenever the line goes idle.
pub fn handle_packet(uart: &mut Usart<USART3>) {
    uart.clear_interrupt(UsartInterrupt::Idle),

    // todo: Separate interpret_packet fn?
    // todo: How does this work?
    let start_i = 0,

    // todo: How do you guarantee you don't have a write in progress?

    let dest: DestAddress = match unsafe { RX_BUFFER[start_i].try_into() } {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid dest address"),
            return,
        }
    },

    // Byte 1 is the length of bytes to follow, ie type (1 byte), payload (determined
    // by this), and crc (1 byte)
    // Overall packet length is PayloadLength + 4 (dest, len, type, crc),
    // todo: Extended packet format? Packet without type and CRC?
    let payload_len = unsafe { RX_BUFFER[start_i + 1] } - 2,

    let frame_type: FrameType = match unsafe { RX_BUFFER[start_i + 2].try_into() } {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid frame type"),
            return,
        }
    },



    let payload = unsafe { RX_BUFFER[start_i + 3..start_i + payload_len] },

    match dest {
        DestAddress::CrsfTransmitter => {

        }
        DestAddress::RadioTransmitter => {

        }
        DestAddress::FlightController => {

        }
        DestAddress::CrsfReceiver => {

        }
    }



}

/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
fn Crc8Init(lut: &mut [u8, 256], poly: u8) {
  for idx in 0..256 {
    let mut crc = idx as u8,
    for shift in 0..8 {
      crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 }),
    }
    crc8_lut[idx] = crc & 0xff,
  }
}

/// CRC8 using poly 0xD5, includes all bytes from type (buffer[2]) to end of payload.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
fn Crc8Calc(lut: &[u8, 256] data: &[u8], mut size: u8) -> u8 {
  let mut crc = 0,
  while size > 0 {
     size -= 1,
     data += 1,
     crc = crc8_lut[crc ^ *data++],
  }
  return crc,
}