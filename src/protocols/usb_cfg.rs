//! This module describes the custom profile we use to exchange data between the flight
//! controller, and a PC running configuration software, over USB. Usd with the `Preflight` PC
//! software.
//!
//! Format - Byte 0: message type. Byte -1: CRC. Rest: payload
//! We use Little-endian float representations.

// todo: Should we use this module and/or a similar structure for data exchange over RF,
// todo: beyond the normal control info used by ELRS? (Eg sending a route, autopilot data etc)

use crate::{
    control_interface::ChannelData,
    dshot,
    flight_ctrls::{
        self,
        common::{AttitudeCommanded, MotorTimers},
        ControlMapping,
    },
    ppks::{Location, WAYPOINT_MAX_NAME_LEN},
    safety::ArmStatus,
    state::{OperationMode, SystemStatus, MAX_WAYPOINTS},
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

// Quaternion attitude + quaternion target + altimeter_baro + altimeter_agl +
// + option byte for altimeter + voltage reading + current reading.

const PARAMS_SIZE: usize = 2 * QUATERNION_SIZE + 4 * F32_SIZE + 1; //
const CONTROLS_SIZE: usize = 18;
// const LINK_STATS_SIZE: usize = F32_BYTES * 4; // Only the first 4 fields.
const LINK_STATS_SIZE: usize = 5; // Only 5 fields.

// 3 coords + option to indicate if used. (Some/None)
pub const WAYPOINT_SIZE: usize = F32_SIZE * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = crate::state::MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_SIZE; // Servo num, value
pub const SYS_STATUS_SIZE: usize = 7; // Sensor status (u8) * 7
pub const AP_STATUS_SIZE: usize = 0; // todo
pub const SYS_AP_STATUS_SIZE: usize = SYS_STATUS_SIZE + AP_STATUS_SIZE;

// Packet sizes are payload size + 2. Additional data are message type, and CRC.
const PARAMS_PACKET_SIZE: usize = PARAMS_SIZE + 2;
const CONTROLS_PACKET_SIZE: usize = CONTROLS_SIZE + 2;
const LINK_STATS_PACKET_SIZE: usize = LINK_STATS_SIZE + 2;
pub const WAYPOINTS_PACKET_SIZE: usize = WAYPOINTS_SIZE + 2;
pub const SET_SERVO_POSIT_PACKET_SIZE: usize = WAYPOINTS_SIZE + 2;
pub const SYS_AP_STATUS_PACKET_SIZE: usize = SYS_AP_STATUS_SIZE + 2;

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
    alt_agl: Option<f32>,
    voltage: f32,
    current: f32,
) -> [u8; PARAMS_SIZE] {
    let mut result = [0; PARAMS_SIZE];

    let (agl, agl_present) = match alt_agl {
        Some(a) => (a, 1),
        None => (0., 0),
    };

    result[0..QUATERNION_SIZE].clone_from_slice(quat_to_bytes(attitude));
    result[QUATERNION_SIZE..2 * QUATERNION_SIZE].clone_from_slice(quat_to_bytes(attitude_commanded));
    result[32..36].clone_from_slice(&alt_baro.to_be_bytes());
    result[36..40].clone_from_slice(&agl.to_be_bytes());
    result[40] = agl_present;
    result[41..45].clone_from_slice(&voltage.to_be_bytes());
    result[45..PARAMS_SIZE].clone_from_slice(&current.to_be_bytes());

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

impl From<&SystemStatus> for [u8; SYS_STATUS_SIZE] {
    fn from(p: &SystemStatus) -> Self {
        [
            p.imu as u8,
            p.baro as u8,
            p.gps as u8,
            p.tof as u8,
            p.magnetometer as u8,
            p.esc_telemetry as u8,
            p.esc_rpm as u8,
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
    attitude_commanded: AttitudeCommanded,
    altitude_baro: f32,
    altitude_agl: Option<f32>,
    controls: &ChannelData,
    link_stats: &LinkStats,
    waypoints: &[Option<Location>; MAX_WAYPOINTS],
    sys_status: &SystemStatus,
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

            println!("Req params");

            // todo: Delegate this v/current calc
            let batt_voltage = adc
                .reading_to_voltage(unsafe { crate::sensors_shared::V_A_ADC_READ_BUF }[0])
                * crate::ADC_BATT_DIVISION;
            let esc_current = adc
                .reading_to_voltage(unsafe { crate::sensors_shared::V_A_ADC_READ_BUF }[1])
                * crate::ADC_CURR_DIVISION;

            let payload = params_to_bytes(
                attitude,
                // todo: Pass more than just the quat, here and in prelfight.
                attitude_commanded
                    .quat
                    .unwrap_or(Quaternion::new_identity()),
                altitude_baro,
                altitude_agl,
                batt_voltage,
                esc_current,
            );

            send_payload(MsgType::Params, &payload, usb_serial);
        }
        MsgType::Ack => {}
        MsgType::ReqControls => {
            println!("Req controls");
            let payload: [u8; CONTROLS_SIZE] = controls.into();
            send_payload(MsgType::Controls, &payload, usb_serial);
        }
        MsgType::Controls => {}
        MsgType::ReqLinkStats => {
            println!("Req link stats");
            let payload: [u8; LINK_STATS_SIZE] = link_stats.into();
            send_payload(MsgType::LinkStats, &payload, usb_serial);
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
            println!("Req waypoints");
            let payload: [u8; WAYPOINTS_SIZE] = waypoints_to_buf(waypoints);
            send_payload(MsgType::Waypoints, &payload, usb_serial);
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
        MsgType::ReqSysApStatus => {
            // todo: Just sys status for now; do AP too.
            println!("Req Sys status and AP status");
            let payload: [u8; SYS_AP_STATUS_SIZE] = sys_status.into();
            send_payload(MsgType::SysApStatus, &payload, usb_serial);
            // send_payload<MsgType::SysApStatus.message_size()>(MsgType::SysApStatus, &payload, usb_serial);
        }
        MsgType::SysApStatus => {}
    }
}

fn send_payload<const N: usize>(
    msg_type: MsgType,
    payload: &[u8],
    usb_serial: &mut SerialPort<'static, UsbBusType>,
) {
    // where M = N + 2{
    // where N: Sized {
    //     const M: usize = N + 2;

    let payload_size = msg_type.payload_size();


    let mut tx_buf = [0; N + 2] where [(); {N + 2}]:;

    tx_buf[0] = MsgType::SysApStatus as u8;
    tx_buf[1..(payload_size + 1)].copy_from_slice(&payload);

    tx_buf[payload_size + 1] = util::calc_crc(
        &CRC_LUT,
        &tx_buf[..payload_size + 1],
        payload_size as u8 + 1,
    );

    usb_serial.write(&tx_buf).ok();
}
