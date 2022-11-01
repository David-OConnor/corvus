//! CRSF support, for receiving radio control signals from ELRS receivers. Only handles communication
//! over serial; the modules handle over-the-air procedures. See also the `elrs` module, for use
//! with the LoRa chip directly, over SPI.
//!
//! [Detailed protocol info](https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol)
//! [WIP clean driver in C](https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c)
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.h
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Serial3.c#L160
//! https://github.com/chris1seto/PX4-Autopilot/tree/pr-rc_crsf_standalone_driver/src/drivers/rc/crsf_rc
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CrsfProtocol/crsf_protocol.h
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

use num_enum::TryFromPrimitive; // Enum from integer

use defmt::println;

use stm32_hal2::{
    dma::{ChannelCfg, Circular, Dma, DmaChannel},
    pac::DMA1,
    usart::{Usart, UsartInterrupt},
};

use crate::{
    control_interface::{
        AltHoldSwitch, AutopilotSwitchA, AutopilotSwitchB, ChannelData, InputModeSwitch, LinkStats,
        PidTuneActuation, PidTuneMode, SteerpointCycleActuation,
    },
    safety::ArmStatus,
    util, UART_ELRS,
};

use cfg_if::cfg_if;

const CRC_POLY: u8 = 0xd5;
const CRC_LUT: [u8; 256] = util::crc_init(CRC_POLY);

const CHANNEL_VAL_MIN: u16 = 172;
const CHANNEL_VAL_MAX: u16 = 1_811;
const CHANNEL_VAL_MIN_F32: f32 = 172.;
const CHANNEL_VAL_MAX_F32: f32 = 1_811.;
// const CHANNEL_VAL_SPAN: u16 = CHANNEL_VAL_MAX - CHANNEL_VAL_MIN;

const CONTROL_VAL_MIN: f32 = -1.;
// todo: Note that if you support 0-centering throttles, make this -1 as well.
const CONTROL_VAL_MIN_THROTTLE: f32 = 0.;
const CONTROL_VAL_MAX: f32 = 1.;

// Used both both TX and RX buffers. Includes payload, and other data words.
// Note that for receiving channel data, we use 26 bytes total (22 of which are channel data).

const PAYLOAD_SIZE_LINK_STATS: usize = 10;
const PAYLOAD_SIZE_RC_CHANNELS: usize = 22;

// Note: 64 bytes is allowed per the protocol. Lower this to reduce latency and mem use. (Minor concern)
// - We only expect 26-byte packets for channel data, and 14-byte packets for link stats.
const MAX_PAYLOAD_SIZE: usize = PAYLOAD_SIZE_RC_CHANNELS;
const MAX_PACKET_SIZE: usize = MAX_PAYLOAD_SIZE + 4; // Extra 4: dest, size, frametype, CRC.

// A pad allows lags in reading to not overwrite the packet start with a new message.
// todo: do we need to erase messages as we read them from the buf?
const RX_BUF_SIZE: usize = MAX_PACKET_SIZE + 20;

pub static mut RX_BUFFER: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];

static mut TX_BUFFER: [u8; MAX_PACKET_SIZE] = [0; MAX_PACKET_SIZE];

// "All packets are in the CRSF format [dest] [len] [type] [payload] [crc8]"

/// Invalid packet, etc.
struct DecodeError {}
// struct CrcError {} todo?

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
    ElrsLua = 0xef,
}

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
/// Frame type (packet type?)
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c#L29
enum FrameType {
    Gps = 0x02,
    BatterySensor = 0x08,
    /// Link data and telemtry, eg RSSI.
    LinkStatistics = 0x14,
    OpentxSync = 0x10,
    RadioId = 0x3a,
    /// Control channel data, for each of 16 channels.
    RcChannelsPacked = 0x16,
    Attitude = 0x1e,
    FlightMode = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    /// A request for device metadata
    DevicePing = 0x28,
    /// Device metadata, in response to a ping
    DeviceInfo = 0x29,
    ParameterSettingsEntry = 0x2b,
    ParameterRead = 0x2c,
    ParameterWrite = 0x2d,
    Command = 0x32,
    // MSP commands
    MspReq = 0x7a,
    MspResp = 0x7b,
    MspWrite = 0x7c,
    ArdupilotResp = 0x80,
}

/// Used to pass packet data to the main program, returned by the handler triggered in the UART-idle
/// interrupt.
pub enum PacketData {
    ChannelData(ChannelData),
    LinkStats(LinkStats),
}

/// Configure the Idle interrupt, and start the circular DMA transfer. Run this once, on initial
/// firmware setup.
pub fn setup(uart: &mut Usart<UART_ELRS>, channel: DmaChannel, dma: &mut Dma<DMA1>) {
    // We alternate between char matching the flight controller destination address, and
    // line idle, to indicate we're received, or stopped receiving a message respectively.
    // uart.enable_interrupt(UsartInterrupt::CharDetect(DestAddr::FlightController as u8));
    // uart.enable_interrupt(UsartInterrupt::ReadNotEmpty);
    uart.enable_interrupt(UsartInterrupt::Idle);

    unsafe {
        uart.read_dma(
            &mut RX_BUFFER,
            channel,
            ChannelCfg {
                circular: Circular::Enabled,
                ..Default::default()
            },
            dma,
        );
    }
}

struct Packet {
    pub dest_addr: DestAddr,
    // Len, starting with `type`, to the end. Payload len + 2 normally, or + 4 with extended packet.
    pub len: usize,
    pub frame_type: FrameType,
    pub extended_dest: Option<DestAddr>,
    pub extended_src: Option<DestAddr>,
    pub payload: [u8; MAX_PAYLOAD_SIZE],
    pub crc: u8,
}

/// Map a raw CRSF channel value to a useful value.
fn channel_to_val(mut chan_val: u16, is_throttle: bool) -> f32 {
    if chan_val < CHANNEL_VAL_MIN {
        chan_val = CHANNEL_VAL_MIN
    } else if chan_val > CHANNEL_VAL_MAX {
        chan_val = CHANNEL_VAL_MAX
    }

    let control_val_min = if is_throttle {
        CONTROL_VAL_MIN_THROTTLE
    } else {
        CONTROL_VAL_MIN
    };
    util::map_linear(
        chan_val as f32,
        (CHANNEL_VAL_MIN_F32, CHANNEL_VAL_MAX_F32),
        (control_val_min, CONTROL_VAL_MAX),
    )
}

impl Packet {
    /// Decode a packet, from the rx buffer.
    pub fn from_buf(buf: &[u8]) -> Result<Self, DecodeError> {
        let dest_addr: DestAddr = match buf[0].try_into() {
            Ok(d) => d,
            Err(_) => return Err(DecodeError {}),
        };

        // Byte 1 (`len` var below) is the length of bytes to follow, ie type (1 byte),
        // payload (determined by this), and crc (1 byte)
        // Overall packet length is PayloadLength + 4 (dest, len, type, crc) for non-extended
        // packets. Channel data doesn't use extended.
        let len = buf[1] as usize;
        // Note: Take extended into acct, if you end up supporting that.
        let payload_len = len - 2;

        let frame_type: FrameType = match buf[2].try_into() {
            Ok(f) => f,
            Err(_) => return Err(DecodeError {}),
        };

        let mut payload = [0; MAX_PAYLOAD_SIZE];

        // Note Extended src/dest is not included, but we don't need that for channel data
        // or link statistics, which is all this module currently supports.

        if payload_len > MAX_PAYLOAD_SIZE {
            // If we don't catch this here, code will crash at the line below.
            println!("Payload len is too large; skipping.");
            return Err(DecodeError {});
        }

        payload[..payload_len].copy_from_slice(&buf[3..(payload_len + 3)]);

        let received_crc = buf[payload_len + 3];

        let expected_crc = util::calc_crc(
            &CRC_LUT,
            // len + 2 gets us to the end. -1 to ommit CRC itself, which isn't part of the calculation.
            &buf[2..len + 1],
            len as u8 - 1,
        );

        if expected_crc != received_crc {
            println!(
                "CRC failed on recieved packet. Expected: {}. Received: {}",
                expected_crc, received_crc
            );
            return Err(DecodeError {});
        };

        Ok(Packet {
            dest_addr,
            len,
            frame_type,
            extended_dest: None,
            extended_src: None,
            payload,
            crc: received_crc,
        })
    }

    /// Fill a buffer from this payload. Used to respond to ping requests from the transmitter.
    pub fn to_buf(&self, buf: &mut [u8; MAX_PACKET_SIZE]) {
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
    pub fn to_channel_data(&self) -> ChannelData {
        let mut data = [0; MAX_PAYLOAD_SIZE];
        for i in 0..MAX_PAYLOAD_SIZE {
            data[i] = self.payload[i] as u16;
        }

        #[cfg(feature = "quad")]
        let motors_armed = ArmStatus::Armed;
        #[cfg(feature = "fixed-wing")]
        let motors_armed = ArmStatus::MotorsControlsArmed;

        // As you change the number of channels used, increase the `raw_channels` size,
        // and comment or uncomment the unpacking lines below, up to 16.
        let mut raw_channels = [0_u16; 14];

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

        cfg_if! {
            if #[cfg(feature = "fixed-wing")] {
                // A normal cfg block doesn't work with index assignment.
                raw_channels[13] = (data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF;
            }
        }

        // raw_channels[14] = (data[19] >> 2 | data[20] << 6) & 0x07FF;
        // raw_channels[15] = (data[20] >> 5 | data[21] << 3) & 0x07FF;

        // https://www.expresslrs.org/2.0/software/switch-config/:
        // "WARNING: Put your arm switch on AUX1, and set it as ~1000 is disarmed, ~2000 is armed."
        // todo: On fixed wing, you want this to be a 3-pos switch, but this may not be
        // todo possible with ELRS, with this channel hard-coded as a 2-pos arm sw?
        let motors_armed = match raw_channels[4] {
            0..=1_500 => false,
            // 0..=1_500 => ArmStatus::Disarmed,
            _ => true,
            // _ => motors_armed,
        };
        let input_mode = match raw_channels[5] {
            0..=1_000 => InputModeSwitch::Acro,
            _ => InputModeSwitch::AttitudeCommand,
        };
        let alt_hold = match raw_channels[6] {
            0..=667 => AltHoldSwitch::Disabled,
            668..=1_333 => AltHoldSwitch::EnabledMsl,
            _ => AltHoldSwitch::EnabledAgl,
        };

        let autopilot_a = match raw_channels[7] {
            0..=667 => AutopilotSwitchA::Disabled,
            668..=1_333 => AutopilotSwitchA::LoiterOrbit,
            _ => AutopilotSwitchA::DirectToPoint,
        };

        let autopilot_b = match raw_channels[8] {
            0..=667 => AutopilotSwitchB::Disabled,
            668..=1_333 => AutopilotSwitchB::HdgHold,
            _ => AutopilotSwitchB::Land,
        };

        let steerpoint_cycle = match raw_channels[9] {
            0..=667 => SteerpointCycleActuation::Decrease,
            668..=1_333 => SteerpointCycleActuation::Neutral,
            _ => SteerpointCycleActuation::Increase,
        };

        let pid_tune_mode = match raw_channels[10] {
            0..=511 => PidTuneMode::Disabled,
            512..=1_023 => PidTuneMode::P,
            1_024..=1533 => PidTuneMode::I,
            _ => PidTuneMode::D,
        };

        let pid_tune_actuation = match raw_channels[11] {
            0..=667 => PidTuneActuation::Decrease,
            668..=1_333 => PidTuneActuation::Neutral,
            _ => PidTuneActuation::Increase,
        };

        let level_attitude_commanded = match raw_channels[12] {
            0..=1_000 => false,
            _ => true,
        };

        // todo: Ideally, this would be on the same channel as motor arm in a 3-pos
        // todo switch, but ELRS hard codes is
        #[cfg(feature = "fixed-wing")]
        let controls_armed = match raw_channels[13] {
            0..=1_000 => false,
            _ => true,
        };

        cfg_if! {
            if #[cfg(feature = "quad")] {
                let arm_status = if motors_armed { ArmStatus::Armed } else {ArmStatus::Disarmed };
            } else {
                let arm_status = if motors_armed {
                    // Implicitly here, if the motor switch armed and control isn't, arm
                    // controls.
                    ArmStatus::MotorsControlsArmed
                } else if controls_armed {
                    ArmStatus::ControlsArmed
                } else {
                    ArmStatus::Disarmed
                };
            }
        }

        // Note that we could map to CRSF channels (Or to their ELRS-mapped origins), but this is
        // currently set up to map directly to how we use the controls.
        ChannelData {
            // Clamp, and map CRSF data to a scale between -1. and 1.  or 0. to +1.
            roll: channel_to_val(raw_channels[0], false),
            pitch: channel_to_val(raw_channels[1], false),
            throttle: channel_to_val(raw_channels[2], true),
            yaw: channel_to_val(raw_channels[3], false),
            arm_status,
            input_mode,
            alt_hold,
            autopilot_a,
            autopilot_b,
            steerpoint_cycle,
            pid_tune_mode,
            pid_tune_actuation,
            level_attitude_commanded,
        }
    }

    /// Interpret a CRSF packet as link statistics
    /// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c#L179
    pub fn to_link_stats(&self) -> LinkStats {
        let data = self.payload;

        LinkStats {
            timestamp: 0, // todo
            uplink_rssi_1: data[0],
            uplink_rssi_2: data[1],
            uplink_link_quality: data[2],
            uplink_snr: data[3] as i8,
            active_antenna: data[4],
            rf_mode: data[5],
            uplink_tx_power: data[6].try_into().unwrap_or_default(),
            downlink_rssi: data[7],
            downlink_link_quality: data[8],
            downlink_snr: data[9] as i8,
        }
    }
}

/// Handle an incomming packet. Triggered whenever the line goes idle.
pub fn handle_packet(
    uart: &mut Usart<UART_ELRS>,
    // dma: &mut Dma<DMA1>,
    rx_chan: DmaChannel,
    // tx_chan: DmaChannel,
) -> Option<PacketData> {
    // Find the position in the buffer where our data starts. It's a circular buffer, so this could
    // be anywhere, at time of line going idle. Then pass in a buffer, rearranged start-to-end.
    // todo: Is there a cheaper way to do this than scanning for a matching pattern?

    let mut start_i = 0;
    let mut start_i_found = false;

    for i in 0..RX_BUF_SIZE {
        unsafe {
            if RX_BUFFER[i] != DestAddr::FlightController as u8 {
                continue;
            }
            let next_byte = RX_BUFFER[(i + 1) % RX_BUF_SIZE];
            let two_bytes_ahead = RX_BUFFER[(i + 2) % RX_BUF_SIZE];

            if (next_byte == PAYLOAD_SIZE_RC_CHANNELS as u8 + 2
                && two_bytes_ahead == FrameType::RcChannelsPacked as u8)
                || (next_byte == PAYLOAD_SIZE_LINK_STATS as u8 + 2
                    && two_bytes_ahead == FrameType::LinkStatistics as u8)
            {
                start_i = i;
                start_i_found = true;
                break;
            }
        }
    }
    if !start_i_found {
        // todo: BIT flag.
        println!("Can't find starting position in Rx payload");
        println!("RX buf: {:?}", unsafe { RX_BUFFER });
        return None;
    }

    // todo: buf shifted not required.
    let mut buf_shifted = [0; MAX_PACKET_SIZE];
    for i in 0..MAX_PACKET_SIZE {
        let msg_start_i = (start_i + i) % RX_BUF_SIZE;
        buf_shifted[i] = unsafe { RX_BUFFER }[msg_start_i];
        // todo: do we need to erase messages as we read them from the buf?
        // unsafe { RX_BUFFER[msg_start_i] = 3 }; // todo: Set to 0
    }

    // todo: do we need to erase messages as we read them from the buf?

    let packet = match Packet::from_buf(&buf_shifted) {
        Ok(p) => p,
        Err(_) => {
            println!("Error decoding packet address or frame type; skipping");
            return None;
        }
    };

    // Only handle packets addressed to a flight controller.
    match packet.dest_addr {
        DestAddr::FlightController => (),
        _ => {
            // Improper destination address from the sender.
            println!("Rx data: Improper destination address from the sender.");
            return None;
        }
    }

    let mut result = None;

    // Processing channel data, and link statistics packets. Respond to ping packets.
    // Note: We currently have this disabled; we have the write, and `dma` arg to this fn
    // commented out.
    match packet.frame_type {
        FrameType::DevicePing => {
            // Send a reply.
            // todo: Consider hard coding this as a dedicated buffer instead of calculating each time.
            let mut payload = [0; MAX_PAYLOAD_SIZE];
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
                crc: util::calc_crc(
                    &CRC_LUT,
                    &payload[2..payload.len() - 1],
                    payload.len() as u8 - 3,
                ),
            };

            unsafe {
                // response.to_buf(&mut TX_BUFFER);
                // uart.write_dma(&TX_BUFFER, tx_chan, Default::default(), dma);
            }
        }
        FrameType::RcChannelsPacked => {
            // We expect a 22-byte payload of channel data, and no extended source or dest.
            let channel_data = packet.to_channel_data();
            result = Some(PacketData::ChannelData(channel_data));
        }
        FrameType::LinkStatistics => {
            let link_stats = packet.to_link_stats();
            result = Some(PacketData::LinkStats(link_stats));
        }
        _ => {
            println!("Unexpected Rx frame type.")
        }
    }

    result
}
