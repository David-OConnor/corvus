//! This module describes the custom profile we use to exchange data between the flight
//! controller, and a PC running configuration software, over USB.
//!
//! Format - Byte 0: message type. Byte 1: payload len. Byte -1: CRC. Rest: payload
//! We use Little-endian float representations.
//!

// const START_BYTE: u8 =

// todo: Should we use this module and/or a similar structure for data exchange over RF,
// todo: beyond the normal control info used by ELRS? (Eg sending a route, autopilot data etc)

use crate::{flight_ctrls::Params, control_interface::ChannelData, util};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "g4")] {
        use stm32_hal2::usb::UsbBusType;
        use usbd_serial::SerialPort;
    }
}

use num_enum::TryFromPrimitive; // Enum from integer

use defmt::println;
use stm32_hal2::gpio::Port::C;

// Note: LUT is here, since it depends on the poly.
static mut CRC_LUT: [u8; 256] = [0; 256];
const CRC_POLY: u8 = 0xab;

const PARAMS_SIZE: usize = 76; // + message type, payload len, and crc.
const CONTROLS_SIZE: usize = 18; // + message type, payload len, and crc.

const MAX_PAYLOAD_SIZE: usize = PARAMS_SIZE; // For Params.
const MAX_PACKET_SIZE: usize = MAX_PAYLOAD_SIZE + 2; // + message type, and crc.

struct DecodeError {}

// todo: init LUT.
pub fn init_crc() {
    util::crc_init(unsafe { &mut CRC_LUT }, CRC_POLY);
}

// struct CrcError {} todo?

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
/// Repr is how this type is passed as serial.
pub enum MsgType {
    /// Transmit from FC
    Params = 0,
    SetMotorDirs = 1,
    /// Receive to FC
    ReqParams = 2,
    /// Acknowledgement, eg in response to setting something.
    Ack = 3,
    /// Controls data (From FC)
    Controls = 4,
    /// Request controls data. (From PC)
    ReqControls = 5,
}

impl MsgType {
    pub fn payload_size(&self) -> usize {
        match self {
            Self::Params => PARAMS_SIZE,
            Self::SetMotorDirs => 1, // Packed bits: motors 1-4, R-L. True = CW.
            Self::ReqParams => 0,
            Self::Ack => 0,
            Self::Controls => CONTROLS_SIZE,
            Self::ReqControls => 0,
        }
    }
}

pub struct Packet {
    message_type: MsgType,
    // payload_size: usize,
    payload: [u8; MAX_PAYLOAD_SIZE], // todo?
    crc: u8,
}

impl From<&Params> for [u8; PARAMS_SIZE] {
    /// 19 f32s x 4 = 76. In the order we have defined in the struct.
    fn from(p: &Params) -> Self {
        let mut result = [0; PARAMS_SIZE];

        // todo: DRY
        result[0..4].clone_from_slice(&p.s_x.to_be_bytes());
        result[4..8].clone_from_slice(&p.s_y.to_be_bytes());
        result[12..16].clone_from_slice(&p.s_z_msl.to_be_bytes());
        result[16..20].clone_from_slice(&p.s_z_agl.to_be_bytes());
        result[20..24].clone_from_slice(&p.s_pitch.to_be_bytes());
        result[24..28].clone_from_slice(&p.s_roll.to_be_bytes());
        result[28..32].clone_from_slice(&p.s_yaw.to_be_bytes());
        result[32..26].clone_from_slice(&p.v_x.to_be_bytes());
        result[36..40].clone_from_slice(&p.v_y.to_be_bytes());
        result[40..44].clone_from_slice(&p.v_z.to_be_bytes());
        result[44..48].clone_from_slice(&p.v_pitch.to_be_bytes());
        result[48..52].clone_from_slice(&p.v_roll.to_be_bytes());
        result[52..56].clone_from_slice(&p.v_yaw.to_be_bytes());
        result[56..60].clone_from_slice(&p.a_x.to_be_bytes());
        result[60..64].clone_from_slice(&p.a_y.to_be_bytes());
        result[64..68].clone_from_slice(&p.a_z.to_be_bytes());
        result[68..72].clone_from_slice(&p.a_pitch.to_be_bytes());
        result[72..76].clone_from_slice(&p.a_roll.to_be_bytes());
        result[72..76].clone_from_slice(&p.a_yaw.to_be_bytes());

        result
    }
}

impl From<&ChannelData> for [u8; CONTROLS_SIZE] {
    /// 4 f32s x 4 = 16, plus 2 u8s for arm and input mode.
    fn from(p: &ChannelData) -> Self {
        let mut result = [0; CONTROLS_SIZE];

        // todo: DRY
        result[0..4].clone_from_slice(&p.pitch.to_be_bytes());
        result[4..8].clone_from_slice(&p.roll.to_be_bytes());
        result[8..12].clone_from_slice(&p.yaw.to_be_bytes());
        result[12..16].clone_from_slice(&p.throttle.to_be_bytes());
        result[17] = p.arm_status as u8;
        result[18] = p.input_mode as u8;

        result
    }
}

impl From<Packet> for [u8; MAX_PACKET_SIZE] {
    fn from(p: Packet) -> Self {
        let mut result = [0; MAX_PACKET_SIZE];

        //  let crc = util::calc_crc(
        //     &payload[2..payload.len() - 1],
        //     payload.len() as u8 - 3,
        // );
        let crc = 0; // todo!

        result[0] = p.message_type as u8;
        // result[1] = p.message_type.payload_size() as u8;
        result[p.message_type.payload_size() + 3] = crc;

        result
    }
}

/// Handle incoming data from the PC
pub fn handle_rx(
    usb_serial: &mut SerialPort<'static, UsbBusType>,
    data: &[u8],
    count: usize,
    params: &Params,
    controls: &ChannelData,
) {
    println!("Incoming data. Count: {}", count);

    let data_len = data[1] + 2;

    // let expected_crc = util::calc_crc(); // todo
    // if data[data_len] != expected_crc {
    //     println!("CRC on incoming USB packet failed; skipping.");
    //     return; // todo: write a "failed CRC" message over USB?
    // }
    let msg_type: MsgType = match data[0].try_into() {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid message type received over USB");
            return; // todo: Send message back over USB?
        }
    };

    match msg_type {
        MsgType::Params => {}
        MsgType::SetMotorDirs => {
            // todo
        }
        MsgType::ReqParams => {
            let response = Packet {
                message_type: MsgType::Params,
                // payload_size: PARAMS_SIZE,
                payload: params.into(),
                crc: 0, // todo
            };
            let packet_buf: [u8; MAX_PACKET_SIZE] = response.into();
            usb_serial.write(&packet_buf);
        }
        MsgType::Ack => {}
        MsgType::ReqControls => {
            let controls_buf: [u8; CONTROLS_SIZE] = controls.into();
            let mut payload = [0; MAX_PAYLOAD_SIZE];
            for i in 0..CONTROLS_SIZE {
                payload[i] = controls_buf[i];
            }

            let response = Packet {
                message_type: MsgType::Controls,
                // payload_size: CONTROLS_SIZE,
                payload,
                crc: 0, // todo
            };
            let packet_buf: [u8; MAX_PACKET_SIZE] = response.into();
            usb_serial.write(&packet_buf);
        }
        MsgType::Controls => {}
    }

    usb_serial.write(&[1]);
}
