//! This module describes the custom profile we use to exchange data between the flight
//! controller, and a PC running configuration software, over USB. Usd with the `Preflight` PC
//! software.
//!
//! Format - Byte 0: message type. Byte -1: CRC. Rest: payload
//! We use Little-endian float representations.

// const START_BYTE: u8 =

// todo: Should we use this module and/or a similar structure for data exchange over RF,
// todo: beyond the normal control info used by ELRS? (Eg sending a route, autopilot data etc)

use crate::{
    control_interface::ChannelData,
    dshot,
    flight_ctrls::{self, common::MotorTimers, ControlMapping},
    ppks::{Location, WAYPOINT_MAX_NAME_LEN},
    safety::ArmStatus,
    state::{OperationMode, MAX_WAYPOINTS},
    util, LinkStats,
};

use lin_alg2::f32::Quaternion;

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::{ServoWing, ServoWingPosition};
    } else {
        use crate::flight_ctrls::{RotorPosition};
    }
}

use stm32_hal2::{
    adc::Adc,
    dma::Dma,
    pac::{self, DMA1},
};

// use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "h7")] {
        use stm32_hal2::usb_otg::Usb1BusType as UsbBusType;
        type ADC = pac::ADC1;
    } else {
        use stm32_hal2::usb::UsbBusType;
        type ADC = pac::ADC2;
    }
}

use usbd_serial::SerialPort;

use num_enum::TryFromPrimitive; // Enum from integer

use defmt::println;

const CRC_POLY: u8 = 0xab;
const CRC_LUT: [u8; 256] = util::crc_init(CRC_POLY);

// These sizes are in bytes.
const F32_SIZE: usize = 4;

const QUATERNION_SIZE: usize = F32_SIZE * 4;

// Quaternion (4x4 + altimeter + altimeter_agl +
// voltage reading + current reading + option byte for altimeter.)
const PARAMS_SIZE: usize = QUATERNION_SIZE + F32_SIZE * 4 + 1; //
const CONTROLS_SIZE: usize = 18;
// const LINK_STATS_SIZE: usize = F32_BYTES * 4; // Only the first 4 fields.
const LINK_STATS_SIZE: usize = 5; // Only 5 fields.

// 3 coords + option to indicate if used. (Some/None)
pub const WAYPOINT_SIZE: usize = F32_SIZE * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = crate::state::MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_SIZE; // Servo num, value

// Packet sizes are payload size + 2. Additional data are message type, and CRC.
const PARAMS_PACKET_SIZE: usize = PARAMS_SIZE + 2;
const CONTROLS_PACKET_SIZE: usize = CONTROLS_SIZE + 2;
const LINK_STATS_PACKET_SIZE: usize = LINK_STATS_SIZE + 2;
pub const WAYPOINTS_PACKET_SIZE: usize = WAYPOINTS_SIZE + 2;
pub const SET_SERVO_POSIT_PACKET_SIZE: usize = WAYPOINTS_SIZE + 2;

struct _DecodeError {}

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
    /// Link quality data (From FC)
    LinkStats = 6,
    /// Request Link quality data (From PC)
    ReqLinkStats = 7,
    /// Arm all motors, for testing
    ArmMotors = 8,
    DisarmMotors = 9,
    /// Start a specific motor
    StartMotor = 10,
    /// Stop a specific motor
    StopMotor = 11,
    ReqWaypoints = 12,
    Updatewaypoints = 13,
    Waypoints = 14,
    #[cfg(feature = "fixed-wing")]
    /// Set all servo positions
    SetServoPosit = 15,
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
            Self::LinkStats => LINK_STATS_SIZE,
            Self::ReqLinkStats => 0,
            Self::ArmMotors => 0,
            Self::DisarmMotors => 0,
            Self::StartMotor => 1,
            Self::StopMotor => 1,
            Self::ReqWaypoints => 0,
            Self::Updatewaypoints => 10, // todo?
            Self::Waypoints => WAYPOINTS_SIZE,
            #[cfg(feature = "fixed-wing")]
            Self::SetServoPosit => SET_SERVO_POSIT_SIZE,
        }
    }
}

fn quat_to_bytes(p: Quaternion) -> [u8; QUATERNION_SIZE] {
    let mut result = [0; QUATERNION_SIZE];

    result[0..4].clone_from_slice(&p.w.to_be_bytes());
    result[4..8].clone_from_slice(&p.x.to_be_bytes());
    result[8..12].clone_from_slice(&p.y.to_be_bytes());
    result[12..16].clone_from_slice(&p.z.to_be_bytes());
    result
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

impl From<&LinkStats> for [u8; LINK_STATS_SIZE] {
    fn from(p: &LinkStats) -> Self {
        [
            p.uplink_rssi_1,
            p.uplink_rssi_2,
            p.uplink_link_quality,
            p.uplink_snr as u8,
            p.uplink_tx_power as u8,
        ]
    }
}

// impl From<[Option<Location>; MAX_WAYPOINTS]> for [u8; WAYPOINTS_SIZE] {
/// Standalone fn instead of impl due to a Rust restriction.
fn waypoints_to_buf(w: &[Option<Location>; MAX_WAYPOINTS]) -> [u8; WAYPOINTS_SIZE] {
    let mut result = [0; WAYPOINTS_SIZE];

    for i in 0..MAX_WAYPOINTS {
        let wp_start_i = i * WAYPOINT_SIZE;

        match &w[i] {
            Some(wp) => {
                result[wp_start_i] = 1;

                result[wp_start_i + 1..wp_start_i + 1 + WAYPOINT_MAX_NAME_LEN]
                    .clone_from_slice(&wp.name);

                let coords_start_i = wp_start_i + 1 + WAYPOINT_MAX_NAME_LEN;

                result[coords_start_i..coords_start_i + 4].clone_from_slice(&wp.lon.to_be_bytes());
                result[coords_start_i + 4..coords_start_i + 8]
                    .clone_from_slice(&wp.lat.to_be_bytes());
                result[coords_start_i + 8..coords_start_i + 12]
                    .clone_from_slice(&wp.alt_msl.to_be_bytes());
            }
            None => {
                result[wp_start_i] = 0;
            }
        };
    }

    result
}

// Readings for preflight
struct PreflightData {}

/// Handle incoming data from the PC
pub fn handle_rx(
    usb_serial: &mut SerialPort<'static, UsbBusType>,
    rx_buf: &[u8],
    attitude: Quaternion,
    altimeter: f32,
    altimeter_agl: Option<f32>,
    controls: &ChannelData,
    link_stats: &LinkStats,
    waypoints: &[Option<Location>; MAX_WAYPOINTS],
    arm_status: &mut ArmStatus,
    control_mapping: &mut ControlMapping,
    op_mode: &mut OperationMode,
    motor_timers: &mut MotorTimers,
    adc: &Adc<ADC>,
    dma: &mut Dma<DMA1>,
) {
    let rx_msg_type: MsgType = match rx_buf[0].try_into() {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid message type received over USB");
            return; // todo: Send message back over USB?
        }
    };

    let expected_crc_rx = util::calc_crc(
        &CRC_LUT,
        &rx_buf[..rx_msg_type.payload_size() + 1],
        rx_msg_type.payload_size() as u8 + 1,
    );

    if rx_buf[rx_msg_type.payload_size() + 1] != expected_crc_rx {
        println!("Incorrect inbound CRC");
        // todo: return here.
    }

    #[cfg(feature = "quad")]
    let motors_armed = ArmStatus::Armed;
    #[cfg(feature = "fixed-wing")]
    let motors_armed = ArmStatus::MotorsControlsArmed;

    match rx_msg_type {
        MsgType::Params => {}
        MsgType::SetMotorDirs => {
            // todo
        }
        MsgType::ReqParams => {
            println!("Req params");
            // todo: current behavior is to set preflight at first params request, and never set
            // todo it back. This could potentially be dangerous.
            *op_mode = OperationMode::Preflight;

            let mut i = 0;

            let mut tx_buf = [0; PARAMS_PACKET_SIZE];

            tx_buf[i] = MsgType::Params as u8;
            i += 1;

            let attitude_buf = quat_to_bytes(attitude);
            tx_buf[i..QUATERNION_SIZE + i].copy_from_slice(&attitude_buf);
            i += QUATERNION_SIZE;

            let altimeter_bytes = altimeter.to_be_bytes();
            tx_buf[i..F32_SIZE + i].copy_from_slice(&altimeter_bytes);
            i += F32_SIZE;

            tx_buf[i] = if altimeter_agl.is_some() { 1 } else { 0 };
            i += 1;

            match altimeter_agl {
                Some(alt) => {
                    let altimeter_agl_bytes = alt.to_be_bytes();
                    tx_buf[i..F32_SIZE + i].copy_from_slice(&altimeter_agl_bytes)
                }
                None => (),
            }
            i += F32_SIZE;

            let batt_v = adc
                .reading_to_voltage(unsafe { crate::sensors_shared::V_A_ADC_READ_BUF }[0])
                * crate::ADC_BATT_DIVISION;
            let curr = adc
                .reading_to_voltage(unsafe { crate::sensors_shared::V_A_ADC_READ_BUF }[1])
                * crate::ADC_CURR_DIVISION;

            let batt_bytes = batt_v.to_be_bytes();
            tx_buf[i..F32_SIZE + i].copy_from_slice(&batt_bytes);
            i += F32_SIZE;

            let curr_bytes = curr.to_be_bytes();
            tx_buf[i..F32_SIZE + i].copy_from_slice(&curr_bytes);
            i += F32_SIZE;

            let payload_size = MsgType::Params.payload_size();
            tx_buf[payload_size + 1] = util::calc_crc(
                &CRC_LUT,
                &tx_buf[..payload_size + 1],
                payload_size as u8 + 1,
            );

            usb_serial.write(&tx_buf).ok();
        }
        MsgType::Ack => {}
        MsgType::ReqControls => {
            // todo: DRY with above.
            let mut tx_buf = [0; CONTROLS_PACKET_SIZE];
            tx_buf[0] = MsgType::Controls as u8;

            let payload: [u8; CONTROLS_SIZE] = controls.into();

            tx_buf[1..(CONTROLS_SIZE + 1)].copy_from_slice(&payload);

            let payload_size = MsgType::Controls.payload_size();
            tx_buf[payload_size + 1] = util::calc_crc(
                &CRC_LUT,
                &tx_buf[..payload_size + 1],
                payload_size as u8 + 1,
            );

            usb_serial.write(&tx_buf).ok();
        }
        MsgType::Controls => {}
        MsgType::ReqLinkStats => {
            // todo: DRY with above.
            let mut tx_buf = [0; LINK_STATS_PACKET_SIZE];
            tx_buf[0] = MsgType::Controls as u8;

            let payload: [u8; LINK_STATS_SIZE] = link_stats.into();

            tx_buf[1..(LINK_STATS_SIZE + 1)].copy_from_slice(&payload);

            let payload_size = MsgType::LinkStats.payload_size();
            tx_buf[payload_size + 1] = util::calc_crc(
                &CRC_LUT,
                &tx_buf[..payload_size + 1],
                payload_size as u8 + 1,
            );

            usb_serial.write(&tx_buf).ok();
        }
        MsgType::LinkStats => {}
        MsgType::ArmMotors => {
            // We use the same `ArmStatus` flag for testing motors in preflight as we do
            // for flight.
            *arm_status = motors_armed;
        }
        MsgType::DisarmMotors => {
            // We use the same `ArmStatus` flag for testing motors in preflight as we do
            // for flight.
            *arm_status = ArmStatus::Disarmed;
        }
        // todo: Message type for set arm to arm controls.
        MsgType::StartMotor => {
            cfg_if! {
                if #[cfg(feature = "fixed-wing")]{
                    dshot::set_power(0.05, 0., 0., 0., motor_timers, dma);
                } else {

                    let rotor_position = match rx_buf[1] {
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

                    dshot::set_power_single(
                        control_mapping.motor_from_position(rotor_position),
                        power,
                        motor_timers,
                        dma,
                    );
                }
            }
        }
        MsgType::StopMotor => {
            cfg_if! {
                if #[cfg(feature = "fixed-wing")]{
                    dshot::set_power(0., 0., 0., 0., motor_timers, dma);
                } else {
                    let rotor_position = match rx_buf[1] {
                        0 => RotorPosition::FrontLeft,
                        1 => RotorPosition::FrontRight,
                        2 => RotorPosition::AftLeft,
                        3 => RotorPosition::AftRight,
                        _ => panic!(),
                    };

                    dshot::set_power_single(
                        control_mapping.motor_from_position(rotor_position),
                        0.,
                        motor_timers,
                        dma,
                    );
                }
            }
        }
        MsgType::ReqWaypoints => {
            println!("Req wps");
            // todo: DRY with above.
            let mut tx_buf = [0; WAYPOINTS_PACKET_SIZE];
            tx_buf[0] = MsgType::Waypoints as u8;

            let payload: [u8; WAYPOINTS_SIZE] = waypoints_to_buf(waypoints);

            tx_buf[1..(WAYPOINTS_SIZE + 1)].copy_from_slice(&payload);

            let payload_size = MsgType::Waypoints.payload_size();
            tx_buf[payload_size + 1] = util::calc_crc(
                &CRC_LUT,
                &tx_buf[..payload_size + 1],
                payload_size as u8 + 1,
            );

            usb_serial.write(&tx_buf).ok();
        }
        MsgType::Waypoints => {}
        MsgType::Updatewaypoints => {}
        #[cfg(feature = "fixed-wing")]
        MsgType::SetServoPosit => {
            let servo = rx_buf[1];
            let value = f32::from_be_bytes(rx_buf[2..6].try_into().unwrap());

            let l = ServoWingPosition::Left as u8;
            let r = ServoWingPosition::Right as u8;

            let servo_wing = match servo {
                l => ServoWingPosition::Left,
                r => ServoWingPosition::Right,
                _ => {
                    println!("Invalid servo requested");
                    return;
                }
            };

            flight_ctrls::set_elevon_posit(
                control_mapping.servo_from_position(servo_wing),
                value,
                control_mapping,
                motor_timers,
            );
        }
    }
}
