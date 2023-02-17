//! This module describes the custom profile we use to exchange data between the flight
//! controller, and a PC running configuration software, over USB. Usd with the `Preflight` PC
//! software.
//!
//! Format - Byte 0: message type. Byte -1: CRC. Rest: payload
//! We use Little-endian float representations.

// todo: Should we use this module and/or a similar structure for data exchange over RF,
// todo: beyond the normal control info used by ELRS? (Eg sending a route, autopilot data etc)

// todo: Start char for all messages?

use core::sync::atomic::Ordering;

use crate::{
    control_interface::ChannelData,
    dshot,
    flight_ctrls::{
        common::{AttitudeCommanded, RpmReadings},
        ControlMapping,
    },
    ppks::{Location, WAYPOINT_MAX_NAME_LEN},
    safety::ArmStatus,
    setup,
    state::{OperationMode, MAX_WAYPOINTS},
    system_status::{self, SystemStatus},
    util, LinkStats,
};

use defmt::println;

#[cfg(feature = "fixed-wing")]
use crate::flight_ctrls;

use lin_alg2::f32::Quaternion;

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::ServoWingPosition;
    } else {
        use crate::flight_ctrls::{RotorPosition};
    }
}

// use stm32_hal2::{adc::Adc, pac};

cfg_if! {
    if #[cfg(feature = "h7")] {
        // todo: USB2 on H743; USB1 on H723.
        // use stm32_hal2::usb_otg::Usb1BusType as UsbBusType;
        use stm32_hal2::usb_otg::Usb2BusType as UsbBusType;
        // type ADC = pac::ADC1;
    } else {
        use stm32_hal2::usb::UsbBusType;
        // type ADC = pac::ADC2;
    }
}

use usbd_serial::SerialPort;

use num_enum::TryFromPrimitive; // Enum from integer

const CRC_POLY: u8 = 0xab;
const CRC_LUT: [u8; 256] = util::crc_init(CRC_POLY);

// These sizes are in bytes.
const F32_SIZE: usize = 4;

const QUATERNION_SIZE: usize = F32_SIZE * 4;

// Quaternion attitude + quaternion target + altimeter_baro + altimeter_agl +
// + option byte for altimeter + voltage reading + current reading.

const PARAMS_SIZE: usize = 2 * QUATERNION_SIZE + 6 * F32_SIZE + 1 + 4 * 3 + 1; //
const CONTROLS_SIZE: usize = 19; // Includes first byte as an Option byte.
                                 // const LINK_STATS_SIZE: usize = F32_BYTES * 4; // Only the first 4 fields.
const LINK_STATS_SIZE: usize = 5; // Only 5 fields.

// 3 coords + option to indicate if used. (Some/None)
pub const WAYPOINT_SIZE: usize = F32_SIZE * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = crate::state::MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_SIZE; // Servo num, value
pub const SYS_STATUS_SIZE: usize = 11; // Sensor status (u8) * 11
pub const AP_STATUS_SIZE: usize = 0; // todo
pub const SYS_AP_STATUS_SIZE: usize = SYS_STATUS_SIZE + AP_STATUS_SIZE;
#[cfg(feature = "quad")]
pub const CONTROL_MAPPING_SIZE: usize = 2; // Packed tightly!
                                           // todo: May need to change to add `servo_high` etc.
#[cfg(feature = "fixed-wing")]
pub const CONTROL_MAPPING_SIZE: usize = 1; // Packed tightly!

// const START_BYTE: u8 =

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
    /// Systems status, and autopilot data combined
    ReqSysApStatus = 16,
    SysApStatus = 17,
    ReqControlMapping = 18,
    ControlMapping = 19,
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
            Self::ReqSysApStatus => 0,
            Self::SysApStatus => SYS_AP_STATUS_SIZE,
            Self::ReqControlMapping => 0,
            Self::ControlMapping => CONTROL_MAPPING_SIZE,
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

fn params_to_bytes(
    attitude: Quaternion,
    attitude_commanded: Quaternion,
    alt_baro: f32,
    pressure_static: f32,
    temp_baro: f32,
    alt_agl: Option<f32>,
    voltage: f32,
    current: f32,
    rpm_status: &RpmReadings,
    aircraft_type: u8,
) -> [u8; PARAMS_SIZE] {
    let mut result = [0; PARAMS_SIZE];

    let (agl, agl_present) = match alt_agl {
        Some(a) => (a, 1),
        None => (0., 0),
    };

    result[0..QUATERNION_SIZE].clone_from_slice(&quat_to_bytes(attitude));
    result[QUATERNION_SIZE..2 * QUATERNION_SIZE]
        .clone_from_slice(&quat_to_bytes(attitude_commanded));
    result[32..36].clone_from_slice(&alt_baro.to_be_bytes());
    result[36..40].clone_from_slice(&agl.to_be_bytes());
    result[40] = agl_present;
    result[41..45].clone_from_slice(&voltage.to_be_bytes());
    result[45..49].clone_from_slice(&current.to_be_bytes());
    result[49..53].clone_from_slice(&pressure_static.to_be_bytes());
    result[53..57].clone_from_slice(&temp_baro.to_be_bytes());

    let mut i = 57;
    for r in &[
        rpm_status.front_left,
        rpm_status.aft_left,
        rpm_status.front_right,
        rpm_status.aft_right,
    ] {
        if let Some(rpm) = r {
            result[i] = 1;
            result[i + 1..i + 3].clone_from_slice(&(*rpm as u16).to_be_bytes());
            // otherwise None
        }
        i += 3;
    }
    i -= 3; // undo last +3.
    i += 1;

    result[i] = aircraft_type;

    result
}

/// 4 f32s x 4 = 16, plus 2 u8s for arm and input mode.
fn channel_data_to_bytes(p: &Option<ChannelData>) -> [u8; CONTROLS_SIZE] {
    let mut result = [0; CONTROLS_SIZE];

    // todo: DRY
    match p {
        Some(c) => {
            result[0] = 1; // `Some`.
            result[1..5].clone_from_slice(&c.pitch.to_be_bytes());
            result[5..9].clone_from_slice(&c.roll.to_be_bytes());
            result[9..13].clone_from_slice(&c.yaw.to_be_bytes());
            result[13..17].clone_from_slice(&c.throttle.to_be_bytes());
            result[17] = c.arm_status as u8;
            result[18] = c.input_mode as u8;
        }
        None => (), // Leave first bit as 0, and rest empty. (also 0)
    }

    result
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

impl From<&SystemStatus> for [u8; SYS_STATUS_SIZE] {
    fn from(p: &SystemStatus) -> Self {
        // todo: You could achieve more efficient packing.
        [
            p.imu as u8,
            p.baro as u8,
            p.gps as u8,
            p.tof as u8,
            p.magnetometer as u8,
            p.esc_telemetry as u8,
            p.esc_rpm as u8,
            p.rf_control_link as u8,
            p.flash_spi as u8,
            system_status::RX_FAULT.load(Ordering::Acquire) as u8,
            system_status::RPM_FAULT.load(Ordering::Acquire) as u8,
        ]
    }
}

#[cfg(feature = "quad")]
impl From<&mut ControlMapping> for [u8; CONTROL_MAPPING_SIZE] {
    fn from(p: &mut ControlMapping) -> Self {
        [
            // 2 bits each
            (p.m1 as u8) | ((p.m2 as u8) << 2) | ((p.m3 as u8) << 4) | ((p.m4 as u8) << 6),
            // 1 bit each
            p.m1_reversed as u8
                | ((p.m2_reversed as u8) << 1)
                | ((p.m3_reversed as u8) << 2)
                | ((p.m4_reversed as u8) << 3)
                | ((p.frontleft_aftright_dir as u8) << 4),
        ]
    }
}

#[cfg(feature = "fixed-wing")]
impl From<&mut ControlMapping> for [u8; CONTROL_MAPPING_SIZE] {
    fn from(p: &mut ControlMapping) -> Self {
        [
            // 1 bit each
            (p.s1 as u8)
                | ((p.s2 as u8) << 2)
                | ((p.s1_reversed as u8) << 4)
                | ((p.s2_reversed as u8) << 6),
            // todo: `servo_high` etc!
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
    attitude_commanded: &AttitudeCommanded,
    altitude_baro: f32,
    pressure_static: f32,
    temp_baro: f32,
    altitude_agl: Option<f32>,
    batt_v: f32,
    esc_current: f32,
    controls: &Option<ChannelData>,
    link_stats: &LinkStats,
    waypoints: &[Option<Location>; MAX_WAYPOINTS],
    sys_status: &SystemStatus,
    arm_status: &mut ArmStatus,
    control_mapping: &mut ControlMapping,
    op_mode: &mut OperationMode,
    motor_timer: &mut setup::MotorTimer,
    servo_timer: &mut setup::ServoTimer,
    rpm_status: &RpmReadings,
) {
    let rx_msg_type: MsgType = match rx_buf[0].try_into() {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid message type received over USB");
            return; // todo: Send message back over USB?
        }
    };

    let payload_size_rx = rx_msg_type.payload_size();
    let crc_read = rx_buf[payload_size_rx + 1];

    // Calculate the CRC starting at the beginning of the packet, and ending at the end of the payload.
    // (This is everything except the CRC byte itself.)
    let crx_expected_rx = util::calc_crc(
        &CRC_LUT,
        &rx_buf[..payload_size_rx + 1],
        payload_size_rx as u8 + 1,
    );

    if crc_read != crx_expected_rx {
        println!("Incorrect inbound CRC on {} message", rx_buf[0]);
        // todo: return here.
    }

    cfg_if! {
        if #[cfg(feature = "quad")] {
            let motors_armed = ArmStatus::Armed;
            let aircraft_type = 0;
        } else {
            let motors_armed = ArmStatus::MotorsControlsArmed;
            let aircraft_type = 1;
        }
    }

    match rx_msg_type {
        MsgType::Params => {}
        MsgType::SetMotorDirs => {
            // todo
        }
        MsgType::ReqParams => {
            // todo: current behavior is to set preflight at first params request, and never set
            // todo it back. This could potentially be dangerous.
            *op_mode = OperationMode::Preflight;

            let payload = params_to_bytes(
                attitude,
                // todo: Pass more than just the quat, here and in prelfight.
                attitude_commanded
                    .quat
                    .unwrap_or(Quaternion::new_identity()),
                altitude_baro,
                pressure_static,
                temp_baro,
                altitude_agl,
                batt_v,
                esc_current,
                rpm_status,
                aircraft_type,
            );

            send_payload::<{ PARAMS_SIZE + 2 }>(MsgType::Params, &payload, usb_serial);
        }
        MsgType::Ack => {}
        MsgType::ReqControls => {
            let payload: [u8; CONTROLS_SIZE] = channel_data_to_bytes(controls);
            send_payload::<{ CONTROLS_SIZE + 2 }>(MsgType::Controls, &payload, usb_serial);
        }
        MsgType::Controls => {}
        MsgType::ReqLinkStats => {
            let payload: [u8; LINK_STATS_SIZE] = link_stats.into();
            send_payload::<{ LINK_STATS_SIZE + 2 }>(MsgType::LinkStats, &payload, usb_serial);
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
                    dshot::set_power(0.05, 0., 0., 0., motor_timer);
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
                        motor_timer,
                    );
                }
            }
        }
        MsgType::StopMotor => {
            cfg_if! {
                if #[cfg(feature = "fixed-wing")]{
                    dshot::set_power(0., 0., 0., 0., motor_timer);
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
                        motor_timer,
                    );
                }
            }
        }
        MsgType::ReqWaypoints => {
            let payload: [u8; WAYPOINTS_SIZE] = waypoints_to_buf(waypoints);
            send_payload::<{ WAYPOINTS_SIZE + 2 }>(MsgType::Waypoints, &payload, usb_serial);
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
                servo_timer,
            );
        }
        MsgType::ReqSysApStatus => {
            // todo: Just sys status for now; do AP too.
            let payload: [u8; SYS_AP_STATUS_SIZE] = sys_status.into();
            send_payload::<{ SYS_AP_STATUS_SIZE + 2 }>(MsgType::SysApStatus, &payload, usb_serial);
        }
        MsgType::SysApStatus => {}
        MsgType::ReqControlMapping => {
            let payload: [u8; CONTROL_MAPPING_SIZE] = control_mapping.into();
            send_payload::<{ CONTROL_MAPPING_SIZE + 2 }>(
                MsgType::ControlMapping,
                &payload,
                usb_serial,
            );
        }
        MsgType::ControlMapping => {}
    }
}

fn send_payload<const N: usize>(
    msg_type: MsgType,
    payload: &[u8],
    usb_serial: &mut SerialPort<'static, UsbBusType>,
) {
    // N is the packet size.
    let payload_size = msg_type.payload_size();

    let mut tx_buf = [0; N];

    tx_buf[0] = msg_type as u8;
    tx_buf[1..(payload_size + 1)].copy_from_slice(&payload);

    tx_buf[payload_size + 1] = util::calc_crc(
        &CRC_LUT,
        &tx_buf[..payload_size + 1],
        payload_size as u8 + 1,
    );

    usb_serial.write(&tx_buf).ok();
}
