//!  CRSF support, for receiving radio control signals from ELRS receivers. Only handles communication
//! over serial; the modules handle over-the-air procedures. See also the `elrs` module, for use
//! with the LoRa chip directly, over SPI.
//!
//! [Detailed protocol info](https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol)
//! [WIP clean driver in C](https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c)
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.h
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Serial3.c#L160
//! https://github.com/chris1seto/PX4-Autopilot/tree/pr-rc_crsf_standalone_driver/src/drivers/rc/crsf_rc
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

use core::convert::TryFrom;
use num_enum::TryFromPrimitive; // Enum from integer

use defmt::println;

use stm32_hal2::{
    dma::{ChannelCfg, Dma, DmaChannel},
    pac::{DMA1, USART3},
    usart::{Usart, UsartInterrupt},
};

use crate::{control_interface::ElrsChannelData, util};

const CRC_POLY: u8 = 0xd;
const HEADER: u8 = 0xc8;

// todo: I'm tracking diff min and max ELRS values.
// todo: u16 or u32
const CHANNEL_VAL_MIN: u32 = 172;
const CHANNEL_VAL_MAX: u32 = 1811;
const CHANNEL_VAL_SPAN: u32 = CHANNEL_VAL_MAX - CHANNEL_VAL_MIN;

const CONTROL_VAL_MIN: f32 = -1.;
const CONTROL_VAL_MAX: f32 = 1.;

// Used both both TX and RX buffers. Includes payload, and other data words.
const MAX_BUF_SIZE: usize = 64;

const PAYLOAD_SIZE_GPS: usize = 15;
const PAYLOAD_SIZE_BATTERY: usize = 8;
const PAYLOAD_SIZE_LINK_STATS: usize = 10;
const PAYLOAD_SIZE_RC_CHANNELS: usize = 22;
const PAYLOAD_SIZE_ATTITUDE: usize = 6;

const CRSF_CHANNEL_COUNT: usize = 16;

static mut RX_BUFFER: [u8; MAX_BUF_SIZE] = [0; MAX_BUF_SIZE];

// todo: Consider storing static (non-mut) TX packets, depending on variety of responses.
static mut TX_BUFFER: [u8; MAX_BUF_SIZE] = [0; MAX_BUF_SIZE];

// todo: Maybe put in a struct etc? It's constant, but we use a function call to populate it.
static mut CRC_LUT: [u8; 256] = [0; 256];

// "All packets are in the CRSF format [dest] [len] [type] [payload] [crc8]"

/// Invalid packet, etc.
struct DecodeError {}

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
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

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
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

#[derive(Default)]
/// https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support
struct LinkStats {
    /// Timestamp these stats were recorded. (TBD format; processed locally; not part of packet from tx).
    timestamp: u32,
    /// Uplink - received signal strength antenna 1 (RSSI). RSSI dBm as reported by the RX. Values
    /// vary depending on mode, antenna quality, output power and distance. Ranges from -128 to 0.
    uplink_rssi_1: u8,
    /// Uplink - received signal strength antenna 2 (RSSI).  	Second antenna RSSI, used in diversity mode
    /// (Same range as rssi_1)
    uplink_rssi_2: u8,
    /// Uplink - link quality (valid packets). The number of successful packets out of the last
    /// 100 from TX → RX
    uplink_link_quality: u8,
    /// Uplink - signal-to-noise ratio. SNR reported by the RX. Value varies mostly by radio chip
    /// and gets lower with distance (once the agc hits its limit)
    uplink_snr: i8,
    /// Active antenna for diversity RX (0 - 1)
    active_antenna: u8,
    rf_mode: u8,
    /// Uplink - transmitting power. (mW?) 50mW reported as 0, as CRSF/OpenTX do not have this option
    uplink_tx_power: u8,
    /// Downlink - received signal strength (RSSI). RSSI dBm of telemetry packets received by TX.
    downlink_rssi: u8,
    /// Downlink - link quality (valid packets). An LQ indicator of telemetry packets received RX → TX
    /// (0 - 100)
    downlink_link_quality: u8,
    /// Downlink - signal-to-noise ratio. 	SNR reported by the TX for telemetry packets
    downlink_snr: i8,
}

/// Configure the Idle interrupt, and start the circular DMA transfer. Run this once, on initial
/// firmware setup.
pub fn setup(uart: &mut Usart<USART3>, channel: DmaChannel, dma: &mut Dma<DMA1>) {
    // Idle interrupt, in conjunction with circular DMA, to indicate we're received a message.
    uart.enable_interrupt(UsartInterrupt::Idle);

    unsafe {
        uart.read_dma(
            &mut RX_BUFFER,
            channel,
            ChannelCfg {
                // circular: Circular,
                ..Default::default()
            },
            dma,
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
    pub payload: [u8; MAX_BUF_SIZE], // todo: Could be TX or RX.
    pub crc: u8,
}

impl Packet {
    /// Decode a packet, from the rx buffer.
    pub fn from_buf(buf: &[u8], start_i: usize) -> Result<Self, DecodeError> {
        let dest_addr: DestAddr = match buf[start_i].try_into() {
            Ok(d) => d,
            Err(_) => return Err(DecodeError {}),
        };

        // Byte 1 (`len` var below) is the length of bytes to follow, ie type (1 byte),
        // payload (determined by this), and crc (1 byte)
        // Overall packet length is PayloadLength + 4 (dest, len, type, crc),
        let len = buf[start_i + 1] as usize;
        let payload_len = len - 2; // todo: Take extended into acct!

        let frame_type: FrameType = match buf[start_i + 2].try_into() {
            Ok(f) => f,
            Err(_) => return Err(DecodeError {}),
        };

        let mut payload = [0; MAX_BUF_SIZE];

        // todo: Extended src/dest

        for i in start_i..start_i + payload_len {
            payload[i] = buf[i + 3]
        }

        let crc = buf[start_i + 3 + payload_len];

        // todo: QC CRC here?

        Ok(Packet {
            dest_addr,
            len,
            frame_type,
            extended_dest: None,
            extended_src: None,
            payload,
            crc,
        })
    }

    /// Fill a buffer from this payload. Used to respond to ping requests from the transmitter.
    pub fn to_buf(&self, buf: &mut [u8; MAX_BUF_SIZE]) {
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

        let payload_len = self.len - 2;

        buf[i..i + payload_len].clone_from_slice(&self.payload);
        buf[3 + payload_len] = self.crc;
    }

    /// Interpret a CRSF packet as ELRS channel data.
    /// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c#L148
    pub fn to_channel_data(&self) -> Result<ElrsChannelData, DecodeError> {
        let mut result = ElrsChannelData::default();

        let mut data = [0; MAX_BUF_SIZE];
        for i in 0..MAX_BUF_SIZE {
            data[i] = self.payload[i] as u32;
        }

        let mut raw_channels = [0_u32; CRSF_CHANNEL_COUNT];

        // Decode channel data
        raw_channels[0] = (data[0] | data[1] << 8) & 0x07FF;
        raw_channels[1] = (data[1] >> 3 | data[2] << 5) & 0x07FF;
        raw_channels[2] = (data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF;
        raw_channels[3] = (data[4] >> 1 | data[5] << 7) & 0x07FF;
        raw_channels[4] = (data[5] >> 4 | data[6] << 4) & 0x07FF;
        raw_channels[5] = (data[6] >> 7 | data[7] << 1 | data[8] << 9) & 0x07FF;
        raw_channels[6] = (data[8] >> 2 | data[9] << 6) & 0x07FF;
        raw_channels[7] = (data[9] >> 5 | data[10] << 3) & 0x07FF;
        raw_channels[8] = (data[11] | data[12] << 8) & 0x07FF;
        raw_channels[9] = (data[12] >> 3 | data[13] << 5) & 0x07FF;
        raw_channels[10] = (data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF;
        raw_channels[11] = (data[15] >> 1 | data[16] << 7) & 0x07FF;
        raw_channels[12] = (data[16] >> 4 | data[17] << 4) & 0x07FF;
        raw_channels[13] = (data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF;
        raw_channels[14] = (data[19] >> 2 | data[20] << 6) & 0x07FF;
        raw_channels[15] = (data[20] >> 5 | data[21] << 3) & 0x07FF;

        // crsf_status.channel_data.timestamp = Ticks_Now();

        // Clamp, and map CRSF data to a scale between -1. and 1.
        // todo: ELRS uses 10-bit channel 1-4 data, 2-pos arm (aux1), and 64-bit or 128-bit aux2-8.
        // todo: How do we map the above to that? And ultimately, we want channel 1-4 on a 0. to 1.,
        // or -1. to +1. scale.
        for i in 0..CRSF_CHANNEL_COUNT {
            if raw_channels[i] < CHANNEL_VAL_MIN {
                raw_channels[i] = CHANNEL_VAL_MIN
            } else if raw_channels[i] > CHANNEL_VAL_MAX {
                raw_channels[i] = CHANNEL_VAL_MAX
            }

            // todo: Figure out if you're using floats or integers here.
            // raw_channels[i] = util::map_linear(
            //     raw_channels[i] as f32, (CHANNEL_VAL_MIN as f32, CHANNEL_VAL_MAX as f32), (CONTROL_VAL_MIN, CONTROL_VAL_MAX)
            // );
        }

        Ok(result)
    }

    /// Interpret a CRSF packet as link statistics
    /// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c#L179
    pub fn to_link_stats(&self) -> Result<LinkStats, DecodeError> {
        let mut result = LinkStats::default();

        let data = self.payload;

        // result.timestamp = Ticks_Now(); // todo
        result.uplink_rssi_1 = data[0];
        result.uplink_rssi_2 = data[1];
        result.uplink_link_quality = data[2];
        result.uplink_snr = data[3] as i8;
        result.active_antenna = data[4];
        result.rf_mode = data[5];
        result.uplink_tx_power = data[6];
        result.downlink_rssi = data[7];
        result.downlink_link_quality = data[8];
        result.downlink_snr = data[9] as i8;

        Ok(result)
    }
}

/// Handle an incomming packet. Triggered whenever the line goes idle.
pub fn handle_packet(
    uart: &mut Usart<USART3>,
    dma: &mut Dma<DMA1>,
    rx_chan: DmaChannel,
    tx_chan: DmaChannel,
) {
    uart.clear_interrupt(UsartInterrupt::Idle);
    dma.stop(rx_chan);

    // Note: As long as we're stopping and starting the transfer each packet, `start_i` will
    // always be 0.
    let start_i = 0;

    let packet = match Packet::from_buf(unsafe { &RX_BUFFER }, start_i) {
        Ok(p) => p,
        Err(_) => {
            println!("Error decoding packet address or frame type; skipping");
            return;
        }
    };

    // Only handle packets addressed to a flight controller.
    match packet.dest_addr {
        DestAddr::FlightController => (),
        // Improper destination address from the sender.
        _ => return,
    }

    // Processing channel data, and link statistics packets. Respond to ping packets.
    match packet.frame_type {
        FrameType::DevicePing => {
            // Send a reply.
            // todo: Consider hard coding this as a dedicated buffer instead of calculating each time.
            let mut payload = [0; MAX_BUF_SIZE];
            payload[0..22].clone_from_slice(&[
                // Display name - "Anyleaf", null-terminated
                0x41, 0x6e, 0x79, 0x6c, 0x65, 0x61, 0x66, 00, 0x45, 0x4c, 0x52,
                0x53, // Serial number - "ELRS"
                0, 0, 1, 0, // Hardware version
                0, 0, 1, 0,    // Software version,
                0x13, // Number of config params. todo
                0,    // parameter protocol version. todo.
            ]);

            let response = Packet {
                dest_addr: DestAddr::RadioTransmitter,
                len: 4,
                frame_type: FrameType::DeviceInfo,
                extended_dest: Some(DestAddr::RadioTransmitter),
                extended_src: Some(DestAddr::CrsfTransmitter),
                payload,
                crc: calc_crc(unsafe { &CRC_LUT }, &payload, 2),
            };

            unsafe {
                response.to_buf(&mut TX_BUFFER);

                uart.write_dma(&TX_BUFFER, tx_chan, Default::default(), dma);
            }
        }
        FrameType::RcChannelsPacked => {
            let channel_data = packet.to_channel_data();
        }
        FrameType::LinkStatistics => {
            let link_stats = packet.to_link_stats();
        }
        _ => (),
    }

    // Start the next transfer.
    unsafe {
        uart.read_dma(&mut RX_BUFFER, rx_chan, Default::default(), dma);
    }
}

/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
fn crc_init(lut: &mut [u8; 256], poly: u8) {
    for i in 0..256 {
        let mut crc = i as u8;
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
        crc = lut[(crc ^ data[i]) as usize];
        i += 1;
    }
    crc
}
