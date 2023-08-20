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

use stm32_hal2::flash::Flash;

use anyleaf_usb::{self, MessageType, CRC_LEN, DEVICE_CODE_CORVUS, MSG_START, PAYLOAD_START_I};

use ahrs::ppks::PositVelEarthUnits;

use defmt::println;

use lin_alg2::f32::Quaternion;

use cfg_if::cfg_if;

use crate::{
    control_interface::ChannelData,
    dshot::{self, Motor},
    flight_ctrls::{
        common::AttitudeCommanded,
        motor_servo::{MotorPower, MotorRpm, MotorServoState, RotationDir},
    },
    safety::ArmStatus,
    setup,
    state::{OperationMode, UserConfig, MAX_WAYPOINTS},
    system_status::{self, SystemStatus},
    util,
};

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        // use crate::flight_ctrls::ServoWingPosition;
        use crate::flight_ctrls;
    } else {
        // use crate::flight_ctrls::{RotorPosition};
    }
}

use usbd_serial::SerialPort;

use crate::flight_ctrls::autopilot::AutopilotStatus;
use crate::protocols::crsf::LinkStats;
use num_enum::TryFromPrimitive; // Enum from integer

const CRC_POLY: u8 = 0xab;
const CRC_LUT: [u8; 256] = util::crc_init(CRC_POLY);

// These sizes are in bytes.
const F32_SIZE: usize = 4;

const QUATERNION_SIZE: usize = F32_SIZE * 4;

// Quaternion attitude + quaternion target + altimeter_baro + altimeter_agl +
// + option byte for altimeter + voltage reading + current reading.

const PARAMS_SIZE: usize = 2 * QUATERNION_SIZE + 6 * F32_SIZE + 1 + 4 * 3 + 1 + F32_SIZE * 4; //
const CONTROLS_SIZE: usize = 19; // Includes first byte as an Option byte.
                                 // const LINK_STATS_SIZE: usize = F32_BYTES * 4; // Only the first 4 fields.
const LINK_STATS_SIZE: usize = 5; // Only 5 fields.

// 3 coords + option to indicate if used. (Some/None)
const WAYPOINT_MAX_NAME_LEN: usize = 12; // todo
pub const WAYPOINT_SIZE: usize = F32_SIZE * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = crate::state::MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_SIZE; // Servo num, value
pub const SYS_STATUS_SIZE: usize = 12; // Sensor status (u8) * 12
pub const AP_STATUS_SIZE: usize = 12; //
pub const SYS_AP_STATUS_SIZE: usize = SYS_STATUS_SIZE + AP_STATUS_SIZE;
#[cfg(feature = "quad")]
pub const CONTROL_MAPPING_SIZE: usize = 2; // Packed tightly!
                                           // todo: May need to change to add `servo_high` etc.
#[cfg(feature = "fixed-wing")]
pub const CONTROL_MAPPING_SIZE: usize = 2; // Packed tightly! todo?
pub const SET_MOTOR_POWER_SIZE: usize = F32_SIZE * 4;

pub const CONFIG_SIZE: usize = F32_SIZE * 3; // todo: Currently PID only.

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
    /// Start all motors (Assuming appropriate arm and power settings are set)
    StartMotors = 10,
    /// Stop all motors.
    StopMotors = 11,
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
    /// All 4
    SetMotorPowers = 20,
    /// All 4
    SetMotorRpms = 21,
    Config = 22,
    ReqConfig = 23,
    SaveConfig = 24,
}

impl MessageType for MsgType {
    fn val(&self) -> u8 {
        *self as u8
    }

    fn payload_size(&self) -> usize {
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
            Self::StartMotors => 0,
            Self::StopMotors => 0,
            Self::ReqWaypoints => 0,
            Self::Updatewaypoints => 10, // todo?
            Self::Waypoints => WAYPOINTS_SIZE,
            #[cfg(feature = "fixed-wing")]
            Self::SetServoPosit => SET_SERVO_POSIT_SIZE,
            Self::ReqSysApStatus => 0,
            Self::SysApStatus => SYS_AP_STATUS_SIZE,
            Self::ReqControlMapping => 0,
            Self::ControlMapping => CONTROL_MAPPING_SIZE,
            Self::SetMotorPowers => SET_MOTOR_POWER_SIZE,
            Self::SetMotorRpms => SET_MOTOR_POWER_SIZE,
            Self::Config => CONFIG_SIZE,
            Self::ReqConfig => 0,
            Self::SaveConfig => CONFIG_SIZE,
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
    // rpm_status: &RpmReadings,
    motor_servo_state: &MotorServoState,
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

    #[cfg(feature = "quad")]
    for r in &[
        motor_servo_state.rotor_front_left.rpm_reading,
        motor_servo_state.rotor_aft_left.rpm_reading,
        motor_servo_state.rotor_front_right.rpm_reading,
        motor_servo_state.rotor_aft_right.rpm_reading,
    ] {
        if let Some(rpm) = r {
            result[i] = 1;
            result[i + 1..i + 3].clone_from_slice(&(*rpm as u16).to_be_bytes());
            // otherwise None
        }
        i += 3;
    }

    #[cfg(feature = "fixed-wing")]
    for r in &[
        motor_servo_state.motor_thrust1.rpm,
        motor_servo_state.motor_thrust1.rpm,
    ] {
        if let Some(rpm) = r {
            result[i] = 1;
            result[i + 1..i + 3].clone_from_slice(&(*rpm as u16).to_be_bytes());
            // otherwise None
        }
        i += 3;
    }

    cfg_if! {
        if #[cfg(feature = "fixed-wing")] {
            i += 6; // Since we're only sending 2 RPMs, but keeping data packing intact.
        }
    }

    result[i] = aircraft_type;
    i += 1;

    // todo: Update for new system
    // result[i..i + 4].clone_from_slice(&current_pwr.front_left.to_be_bytes());
    // i += 4;
    // result[i..i + 4].clone_from_slice(&current_pwr.aft_left.to_be_bytes());
    // i += 4;
    // result[i..i + 4].clone_from_slice(&current_pwr.front_right.to_be_bytes());
    // i += 4;
    // result[i..i + 4].clone_from_slice(&current_pwr.aft_right.to_be_bytes());
    // i += 4;

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

impl SystemStatus {
    pub fn to_bytes(&self) -> [u8; SYS_STATUS_SIZE] {
        [
            self.imu as u8,
            self.baro as u8,
            self.gnss as u8,
            self.tof as u8,
            self.magnetometer as u8,
            self.esc_telemetry as u8,
            self.esc_rpm as u8,
            self.rf_control_link as u8,
            self.flash_spi as u8,
            self.osd as u8,
            system_status::RX_FAULT.load(Ordering::Acquire) as u8,
            system_status::RPM_FAULT.load(Ordering::Acquire) as u8,
        ]
    }
}

impl AutopilotStatus {
    pub fn to_bytes(&self) -> [u8; AP_STATUS_SIZE] {
        let mut result = [0; AP_STATUS_SIZE];

        match self.alt_hold {
            Some((hold_type, hold_value)) => {
                // println!("Alt hold enabled");
                result[0] = 1; // Indicates AP hold is on.
                result[1] = hold_type as u8;
                result[2..6].clone_from_slice(&hold_value.to_be_bytes());
            }
            None => (),
        }

        match self.hdg_hold {
            Some(hold_value) => {
                result[6] = 1; // Indicates AP hold is on.
                result[7..11].clone_from_slice(&hold_value.to_be_bytes());
            }
            None => (),
        }

        result[11] = self.yaw_assist as u8;

        result
    }
}

// #[cfg(feature = "quad")]
// impl From<&mut ControlMapping> for [u8; CONTROL_MAPPING_SIZE] {
//     fn from(p: &mut ControlMapping) -> Self {
//         [
//             // 2 bits each
//             (p.m1 as u8) | ((p.m2 as u8) << 2) | ((p.m3 as u8) << 4) | ((p.m4 as u8) << 6),
//             // 1 bit each
//             p.m1_reversed as u8
//                 | ((p.m2_reversed as u8) << 1)
//                 | ((p.m3_reversed as u8) << 2)
//                 | ((p.m4_reversed as u8) << 3)
//                 | ((p.frontleft_aftright_dir as u8) << 4),
//         ]
//     }
// }

// #[cfg(feature = "fixed-wing")]
// impl From<&mut ControlMapping> for [u8; CONTROL_MAPPING_SIZE] {
//     fn from(p: &mut ControlMapping) -> Self {
//         [
//             // 1 bit each
//             (p.s1 as u8)
//                 | ((p.s2 as u8) << 2)
//                 | ((p.s1_reversed as u8) << 4)
//                 | ((p.s2_reversed as u8) << 6),
//             // todo: `servo_high` etc!
//         ]
//     }
// }

// impl From<[Option<Location>; MAX_WAYPOINTS]> for [u8; WAYPOINTS_SIZE] {
/// Standalone fn instead of impl due to a Rust restriction.
fn waypoints_to_buf(w: &[Option<PositVelEarthUnits>; MAX_WAYPOINTS]) -> [u8; WAYPOINTS_SIZE] {
    let mut result = [0; WAYPOINTS_SIZE];

    for i in 0..MAX_WAYPOINTS {
        let wp_start_i = i * WAYPOINT_SIZE;

        match &w[i] {
            Some(wp) => {
                result[wp_start_i] = 1;

                // todo: Put back etc.
                // result[wp_start_i + 1..wp_start_i + 1 + WAYPOINT_MAX_NAME_LEN]
                //     .clone_from_slice(&wp.name);

                let coords_start_i = wp_start_i + 1 + WAYPOINT_MAX_NAME_LEN;

                // todo: Update with your 64-bit lat/lons.
                // result[coords_start_i..coords_start_i + 4].clone_from_slice(&wp.lon_e8.to_be_bytes());
                // result[coords_start_i + 4..coords_start_i + 8]
                //     .clone_from_slice(&wp.lat.to_be_bytes());

                result[coords_start_i + 8..coords_start_i + 12]
                    .clone_from_slice(&wp.elevation_msl.to_be_bytes());
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
    usb_serial: &mut SerialPort<'static, setup::UsbBusType>,
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
    config: &mut UserConfig,
    sys_status: &SystemStatus,
    autopilot_status: &AutopilotStatus,
    arm_status: &mut ArmStatus,
    op_mode: &mut OperationMode,
    motor_timer: &mut setup::MotorTimer,
    servo_timer: &mut setup::ServoTimer,
    // rpm_status: &RpmReadings,
    motor_servo_state: &mut MotorServoState,
    preflight_motors_running: &mut bool,
    flash: &mut Flash,
) {
    if rx_buf[0] != MSG_START {
        println!("Invalid start byte rec");
        return;
    }

    let rx_msg_type: MsgType = match rx_buf[2].try_into() {
        Ok(d) => d,
        Err(_) => {
            println!("Invalid message type received over USB");
            return; // todo: Send message back over USB?
        }
    };

    if !anyleaf_usb::check_crc(&rx_buf, rx_msg_type.payload_size() + PAYLOAD_START_I) {
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
                attitude_commanded.quat,
                altitude_baro,
                pressure_static,
                temp_baro,
                altitude_agl,
                batt_v,
                esc_current,
                // rpm_status,
                motor_servo_state,
                aircraft_type,
            );

            send_payload::<{ PARAMS_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::Params,
                &payload,
                usb_serial,
            );
        }
        MsgType::Ack => {}
        MsgType::ReqControls => {
            let payload: [u8; CONTROLS_SIZE] = channel_data_to_bytes(controls);
            send_payload::<{ CONTROLS_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::Controls,
                &payload,
                usb_serial,
            );
        }
        MsgType::Controls => {}
        MsgType::ReqLinkStats => {
            let payload: [u8; LINK_STATS_SIZE] = link_stats.into();
            send_payload::<{ LINK_STATS_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::LinkStats,
                &payload,
                usb_serial,
            );
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
        MsgType::StartMotors => {
            *preflight_motors_running = true;
            println!("Preflight motors started");
            // cfg_if! {
            // if #[cfg(feature = "fixed-wing")]{
            //     dshot::set_power(0.05, 0., 0., 0., motor_timer);
            // } else {
            //
            //     let motor = match rx_buf[1] {
            //         // todo: This more robust approach won't work.
            //         // RotorPosition::FrontLeft as u8 => RotorPosition::FrontLeft,
            //         // RotorPosition::FrontRight as u8 => RotorPosition::FrontRight,
            //         // RotorPosition::AftLeft as u8 => RotorPosition::AftLeft,
            //         // RotorPosition::AftRight as u8 => RotorPosition::AftRight,
            //         0 => Motor::M1,
            //         1 => Motor::M2,
            //         2 => Motor::M3,
            //         3 => Motor::M4,
            //         _ => panic!(),
            //     };
            //
            //     // todo: Don't hard-code rotor power. Let it be user-selected.
            //     let power = 0.05;
            //
            //     dshot::set_power_single(
            //         motor,
            //         power,
            //         motor_timer,
            //     );
            // }
            // }
        }
        MsgType::StopMotors => {
            *preflight_motors_running = false;
            println!("Preflight motors stopped");
            // cfg_if! {
            //     if #[cfg(feature = "fixed-wing")]{
            //         dshot::set_power(0., 0., 0., 0., motor_timer);
            //     } else {
            //         let motor = match rx_buf[1] {
            //             0 => Motor::M1,
            //             1 => Motor::M2,
            //             2 => Motor::M3,
            //             3 => Motor::M4,
            //             _ => panic!(),
            //         };
            //
            //         dshot::set_power_single(
            //             motor,
            //             0.,
            //             motor_timer,
            //         );
            //     }
            // }
        }
        MsgType::ReqWaypoints => {
            let payload: [u8; WAYPOINTS_SIZE] = waypoints_to_buf(&config.waypoints);
            send_payload::<{ WAYPOINTS_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::Waypoints,
                &payload,
                usb_serial,
            );
        }
        MsgType::Waypoints => {}
        MsgType::Updatewaypoints => {}
        #[cfg(feature = "fixed-wing")]
        MsgType::SetServoPosit => {
            let servo = rx_buf[1];
            let value = f32::from_be_bytes(rx_buf[2..6].try_into().unwrap());

            // todo: COme back to this.
            // let l = ServoWingPosition::Left as u8;
            // let r = ServoWingPosition::Right as u8;
            //
            // let servo_wing = match servo {
            //     l => ServoWingPosition::Left,
            //     r => ServoWingPosition::Right,
            //     _ => {
            //         println!("Invalid servo requested");
            //         return;
            //     }
            // };

            // flight_ctrls::set_elevon_posit(
            //     control_mapping.servo_from_position(servo_wing),
            //     value,
            //     control_mapping,
            //     servo_timer,
            // );
        }
        MsgType::ReqSysApStatus => {
            let mut payload: [u8; SYS_AP_STATUS_SIZE] = [0; SYS_AP_STATUS_SIZE];
            payload[..SYS_STATUS_SIZE].clone_from_slice(&sys_status.to_bytes());
            payload[SYS_STATUS_SIZE..SYS_AP_STATUS_SIZE]
                .clone_from_slice(&autopilot_status.to_bytes());

            send_payload::<{ SYS_AP_STATUS_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::SysApStatus,
                &payload,
                usb_serial,
            );
        }
        MsgType::SysApStatus => {}
        MsgType::ReqControlMapping => {
            // todo: This message type needs to be replaced by
            // todo MotorServoState. perhaps
            // let payload = [0_u8; 2]; // todo placeholder while we re-set up motorss etc

            // todo: Temp ahrdcoded mappings:

            //             rotor_front_left_hardware: MotorServoHardware::Pin4,
            //             rotor_front_right_hardware: MotorServoHardware::Pin2,
            //             rotor_aft_left_hardware: MotorServoHardware::Pin3,
            //             rotor_aft_right_hardware: MotorServoHardware::Pin1,

            let m = &motor_servo_state; // code shortener
            #[cfg(feature = "quad")]
            let payload = [
                // 2 bits each
                (3) | ((1) << 2) | ((2) << 4) | ((0) << 6),
                // 1 bit each
                false as u8
                    | ((false as u8) << 1)
                    | ((false as u8) << 2)
                    | ((false as u8) << 3)
                    | ((m.frontleft_aftright_dir as u8) << 4),
            ];

            send_payload::<{ CONTROL_MAPPING_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::ControlMapping,
                &payload,
                usb_serial,
            );
        }
        MsgType::ControlMapping => {}
        MsgType::SetMotorPowers => {
            let power = MotorPower {
                front_left: f32::from_be_bytes(rx_buf[0..4].try_into().unwrap()),
                front_right: f32::from_be_bytes(rx_buf[4..8].try_into().unwrap()),
                aft_left: f32::from_be_bytes(rx_buf[8..12].try_into().unwrap()),
                aft_right: f32::from_be_bytes(rx_buf[12..16].try_into().unwrap()),
            };

            println!("Preflight motor power FL: {}", power.front_left);
            motor_servo_state.set_cmds_from_power(&power);
        }
        MsgType::SetMotorRpms => {
            // todo: YOu need a safety rail on this.
            let rpms = MotorRpm {
                front_left: f32::from_be_bytes(rx_buf[0..4].try_into().unwrap()),
                front_right: f32::from_be_bytes(rx_buf[4..8].try_into().unwrap()),
                aft_left: f32::from_be_bytes(rx_buf[8..12].try_into().unwrap()),
                aft_right: f32::from_be_bytes(rx_buf[12..16].try_into().unwrap()),
            };

            // todo.
            // motor_servo_state.set_cmds_from_rpms(
            //     &rpms,
            //     rpm_readings,
            //     motor_pid_group,
            //     motor_pid_coeffs,
            // );
        }
        MsgType::Config => (),
        MsgType::ReqConfig => {
            let payload = config.to_bytes();

            send_payload::<{ CONFIG_SIZE + PAYLOAD_START_I + CRC_LEN }>(
                MsgType::Config,
                &payload,
                usb_serial,
            );
        }
        MsgType::SaveConfig => {
            println!("Save config received");
            *config =
                UserConfig::from_bytes(&rx_buf[PAYLOAD_START_I..PAYLOAD_START_I + CONFIG_SIZE]);
            config.save(flash);
        }
    }
}

fn send_payload<const N: usize>(
    msg_type: MsgType,
    payload: &[u8],
    usb_serial: &mut SerialPort<'static, setup::UsbBusType>,
) {
    // N is the packet size.
    let payload_size = msg_type.payload_size();

    let mut tx_buf = [0; N];

    tx_buf[0] = MSG_START;
    tx_buf[1] = DEVICE_CODE_CORVUS;
    tx_buf[2] = msg_type as u8;

    tx_buf[PAYLOAD_START_I..(payload_size + PAYLOAD_START_I)]
        .copy_from_slice(&payload[..payload_size]);

    tx_buf[payload_size + PAYLOAD_START_I] = anyleaf_usb::calc_crc(
        &anyleaf_usb::CRC_LUT,
        &tx_buf[..payload_size + PAYLOAD_START_I],
        (payload_size + PAYLOAD_START_I) as u8,
    );

    usb_serial.write(&tx_buf).ok();
}
