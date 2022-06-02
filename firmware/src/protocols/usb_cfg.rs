//! This module describes the custom profile we use to exchange data between the flight
//! controller, and a PC running configuration software, over USB.
//!
//! Format - Byte 0: message type. Byte -1: CRC. Rest: payload
//! We use Little-endian float representations.
//!

// const START_BYTE: u8 =

// todo: Should we use this module and/or a similar structure for data exchange over RF,
// todo: beyond the normal control info used by ELRS? (Eg sending a route, autopilot data etc)

use crate::{
    control_interface::ChannelData,
    dshot,
    flight_ctrls::quad::{Motor, RotorMapping, RotorPosition},
    lin_alg::Quaternion,
    util, ArmStatus, OperationMode,
};

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::Timer,
};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "g4")] {
        use stm32_hal2::usb::UsbBusType;
        use usbd_serial::SerialPort;
    }
}

use num_enum::TryFromPrimitive; // Enum from integer

use defmt::println;

const F32_BYTES: usize = 4;

// Note: LUT is here, since it depends on the poly.
static mut CRC_LUT: [u8; 256] = [0; 256];
const CRC_POLY: u8 = 0xab;

// const PARAMS_SIZE: usize = 76;
const QUATERNION_SIZE: usize = F32_BYTES * 4; // Quaternion (4x4 + altimeter)
const PARAMS_SIZE: usize = QUATERNION_SIZE + 4; //
const CONTROLS_SIZE: usize = 18;

// Packet sizes are payload size + 2. Additional data are message type, and CRC.
const ATTITUDE_PACKET_SIZE: usize = 18;
const CONTROLS_PACKET_SIZE: usize = 20;

struct _DecodeError {}

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
    /// Arm all motors, for testing
    ArmMotors = 6,
    DisarmMotors = 7,
    /// Start a specific motor
    StartMotor = 8,
    /// Stop a specific motor
    StopMotor = 9,
}

impl MsgType {
    pub fn payload_size(&self) -> usize {
        match self {
            // Self::Params => PARAMS_SIZE,
            Self::Params => PARAMS_SIZE,
            Self::SetMotorDirs => 1, // Packed bits: motors 1-4, R-L. True = CW.
            Self::ReqParams => 0,
            Self::Ack => 0,
            Self::Controls => CONTROLS_SIZE,
            Self::ReqControls => 0,
            Self::ArmMotors => 0,
            Self::DisarmMotors => 0,
            Self::StartMotor => 1,
            Self::StopMotor => 1,
        }
    }
}

// impl From<&Params> for [u8; PARAMS_SIZE] {
//     /// 19 f32s x 4 = 76. In the order we have defined in the struct.
//     fn from(p: &Params) -> Self {
//         let mut result = [0; PARAMS_SIZE];
//
//         // todo: DRY
//         result[0..4].clone_from_slice(&p.s_x.to_be_bytes());
//         result[4..8].clone_from_slice(&p.s_y.to_be_bytes());
//         result[8..12].clone_from_slice(&p.s_z_msl.to_be_bytes());
//         result[12..16].clone_from_slice(&p.s_z_agl.to_be_bytes());
//         result[16..20].clone_from_slice(&p.s_pitch.to_be_bytes());
//         result[20..24].clone_from_slice(&p.s_roll.to_be_bytes());
//         result[24..28].clone_from_slice(&p.s_yaw.to_be_bytes());
//         result[28..32].clone_from_slice(&p.quaternion.to_be_bytes());
//
//         // todo: If you transmit all params again, set this back up for the offset squishing in quaternion
//         // todo implies.
//         result[28..32].clone_from_slice(&p.v_x.to_be_bytes());
//         result[32..36].clone_from_slice(&p.v_y.to_be_bytes());
//         result[36..40].clone_from_slice(&p.v_z.to_be_bytes());
//         result[40..44].clone_from_slice(&p.v_pitch.to_be_bytes());
//         result[44..48].clone_from_slice(&p.v_roll.to_be_bytes());
//         result[48..52].clone_from_slice(&p.v_yaw.to_be_bytes());
//         result[52..56].clone_from_slice(&p.a_x.to_be_bytes());
//         result[56..60].clone_from_slice(&p.a_y.to_be_bytes());
//         result[60..64].clone_from_slice(&p.a_z.to_be_bytes());
//         result[64..68].clone_from_slice(&p.a_pitch.to_be_bytes());
//         result[68..72].clone_from_slice(&p.a_roll.to_be_bytes());
//         result[72..76].clone_from_slice(&p.a_yaw.to_be_bytes());
//
//         result
//     }
// }

impl From<Quaternion> for [u8; QUATERNION_SIZE] {
    /// 4 f32s = 76. In the order we have defined in the struct.
    fn from(p: Quaternion) -> Self {
        let mut result = [0; QUATERNION_SIZE];

        result[0..4].clone_from_slice(&p.w.to_be_bytes());
        result[4..8].clone_from_slice(&p.x.to_be_bytes());
        result[8..12].clone_from_slice(&p.y.to_be_bytes());
        result[12..16].clone_from_slice(&p.z.to_be_bytes());
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
        result[16] = p.arm_status as u8;
        result[17] = p.input_mode as u8;

        result
    }
}

/// Handle incoming data from the PC
pub fn handle_rx(
    usb_serial: &mut SerialPort<'static, UsbBusType>,
    data: &[u8],
    count: usize,
    // params: &Params,
    attitude: Quaternion,
    altimeter: f32,
    controls: &ChannelData,
    arm_status: &mut ArmStatus,
    rotor_mapping: &mut RotorMapping,
    op_mode: &mut OperationMode,
    rotor_timer_a: &mut Timer<TIM2>,
    rotor_timer_b: &mut Timer<TIM3>,
    dma: &mut Dma<DMA1>,
) {
    let rx_msg_type: MsgType = match data[0].try_into() {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid message type received over USB");
            return; // todo: Send message back over USB?
        }
    };

    let expected_crc_rx = util::calc_crc(
        unsafe { &CRC_LUT },
        &data[..rx_msg_type.payload_size() + 1],
        rx_msg_type.payload_size() as u8 + 1,
    );

    if data[rx_msg_type.payload_size() + 1] != expected_crc_rx {
        println!("Incorrect inbound CRC");
        // todo: return here.
    }

    match rx_msg_type {
        MsgType::Params => {}
        MsgType::SetMotorDirs => {
            // todo
        }
        MsgType::ReqParams => {
            println!("Params requested...");

            // todo: current behavior is to set preflight at first params request, and never set
            // todo it back. This could potentially be dangerous.
            *op_mode = OperationMode::Preflight;

            let mut tx_buf = [0; ATTITUDE_PACKET_SIZE];
            tx_buf[0] = MsgType::Params as u8;

            let quat_buf: [u8; QUATERNION_SIZE] = attitude.into();

            tx_buf[1..(QUATERNION_SIZE + 1)].copy_from_slice(&quat_buf);

            let altimeter_bytes = altimeter.to_be_bytes();
            for i in 0..4 {
                tx_buf[i + 1 + QUATERNION_SIZE] = altimeter_bytes[i];
            }

            let payload_size = MsgType::Params.payload_size();
            tx_buf[payload_size + 1] = util::calc_crc(
                unsafe { &CRC_LUT },
                &tx_buf[..payload_size + 1],
                payload_size as u8 + 1,
            );

            usb_serial.write(&tx_buf).ok();
        }
        MsgType::Ack => {}
        MsgType::ReqControls => {
            println!("Controls requested...");

            // todo: DRY with above.
            let mut tx_buf = [0; CONTROLS_PACKET_SIZE];
            tx_buf[0] = MsgType::Controls as u8;

            let payload: [u8; CONTROLS_SIZE] = controls.into();

            tx_buf[1..(CONTROLS_SIZE + 1)].copy_from_slice(&payload);

            let payload_size = MsgType::Controls.payload_size();
            tx_buf[payload_size + 1] = util::calc_crc(
                unsafe { &CRC_LUT },
                &tx_buf[..payload_size + 1],
                payload_size as u8 + 1,
            );

            usb_serial.write(&tx_buf).ok();
        }
        MsgType::Controls => {}
        MsgType::ArmMotors => {
            // We use the same `ArmStatus` flag for testing motors in preflight as we do
            // for flight.
            *arm_status = ArmStatus::Armed;
        }
        MsgType::DisarmMotors => {
            // We use the same `ArmStatus` flag for testing motors in preflight as we do
            // for flight.
            *arm_status = ArmStatus::Disarmed;
        }
        MsgType::StartMotor => {
            let rotor_position = match data[1] {
                // todo: This more robust approach won't work.
                // RotorPosition::FrontLeft as u8 => RotorPosition::FrontLeft,
                // RotorPosition::FrontRight as u8 => RotorPosition::FrontRight,
                // RotorPosition::AftLeft as u8 => RotorPosition::AftLeft,
                // RotorPosition::AftRight as u8 => RotorPosition::AftRight,
                0 => RotorPosition::FrontLeft,
                1 => RotorPosition::FrontRight,
                2 => RotorPosition::AftLeft,
                3 => RotorPosition::AftRight,
                _ => panic!(),
            };

            // todo: Don't hard-code rotor power. Let it be user-selected.
            let power = 0.05;

            match rotor_mapping.motor_from_position(rotor_position) {
                Motor::M1 => {
                    dshot::set_power_single_a(Motor::M1, power, rotor_timer_a, dma);
                }
                Motor::M2 => {
                    dshot::set_power_single_a(Motor::M2, power, rotor_timer_a, dma);
                }
                Motor::M3 => {
                    dshot::set_power_single_b(Motor::M3, power, rotor_timer_b, dma);
                }
                Motor::M4 => {
                    dshot::set_power_single_b(Motor::M4, power, rotor_timer_b, dma);
                }
            }
        }
        MsgType::StopMotor => {
            let rotor_position = match data[1] {
                0 => RotorPosition::FrontLeft,
                1 => RotorPosition::FrontRight,
                2 => RotorPosition::AftLeft,
                3 => RotorPosition::AftRight,
                _ => panic!(),
            };

            match rotor_mapping.motor_from_position(rotor_position) {
                Motor::M1 => {
                    dshot::set_power_single_a(Motor::M1, 0., rotor_timer_a, dma);
                }
                Motor::M2 => {
                    dshot::set_power_single_a(Motor::M2, 0., rotor_timer_a, dma);
                }
                Motor::M3 => {
                    dshot::set_power_single_b(Motor::M3, 0., rotor_timer_b, dma);
                }
                Motor::M4 => {
                    dshot::set_power_single_b(Motor::M4, 0., rotor_timer_b, dma);
                }
            }
        }
    }
}
