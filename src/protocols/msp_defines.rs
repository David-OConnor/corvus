//! This module contains structs and associated enums that represent MSP functions.
//! We use it for sending OSD data, eg for DJI goggles.
//! For code to construct and send MSP packets, see the `msp` module.
//!
//! Some items have been updated with rust naming conventions, while others haven't. Some have
//! code for creating a buffer to send, while others haven't. Ideally, all should, but we've only
//! implmented ones currently used.
//!
//! https://github.com/chris1seto/PX4-Autopilot/blob/turbotimber/src/modules/msp_osd/msp_defines.h
//! (From that: Found on https://github.com/d3ngit/djihdfpv_mavlink_to_msp_V2/blob/master/Arduino_libraries/MSP/MSP.h)
//!
//! Note that we have many commented out data structures: These are currently unused, but we keep
//! for reference, if needed later.

#![allow(dead_code)]

const FLIGHT_CONTROLLER_IDENTIFIER_LENGTH: usize = 4;

#[derive(Clone, Copy, PartialEq)]
#[repr(u16)]
/// Message type for requests and replies
pub enum Function {
    ApiVersion = 1,
    FcVariant = 2,
    FcVersion = 3,
    BoardInfo = 4,
    BuildInfo = 5,
    Name = 10,
    CalibrationData = 14,
    Feature = 36,
    BoardAlignment = 38,
    CurrentMeterConfig = 40,
    RxConfig = 44,
    SonarAltitude = 58,
    ArmingConfig = 61,
    RxMap = 64,
    // get channel map (also returns number of channels total)
    LoopTime = 73,
    // FC cycle time i.e looptime parameter
    OsdConfig = 84,
    Status = 101,
    RawImu = 102,
    SERVO = 103,
    MOTOR = 104,
    RC = 105,
    RawGps = 106,
    CompGps = 107,
    // distance home, direction home
    Attitude = 108,
    Altitude = 109,
    Analog = 110,
    RcTuning = 111,
    // rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
    PID = 112,
    // P I D coeff
    MISC = 114,
    ServoConfigurations = 120,
    NavStatus = 121,
    // navigation status
    SensorAlignment = 126,
    // orientation of acc,gyro,mag
    BatteryState = 130,
    EscSensorData = 134,
    MotorTelemetry = 139,
    StatusEx = 150,
    SensorStatus = 151,
    Boxids = 119,
    Uid = 160,
    // Unique device ID
    GpsSvInfo = 164,
    // get Signal Strength (only U-Blox)
    GpsStatistics = 166,
    // get GPS debugging data
    SetPid = 202, // set P I D coeff
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Commands
enum Command {
    Head = 211,
    // define a new heading hold direction
    RawRc = 200,
    // 8 rc chan
    RawGps = 201,
    // fix, numsat, lat, lon, alt, speed
    Wp = 209, // sets a given WP (WP#, lat, lon, alt, flags)
}

// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// /// bits of getActiveModes() return value
// enum Mode {
//     ARM = 0,
//     ANGLE = 1,
//     HORIZON = 2,
//     NAVALTHOLD = 3, /* cleanflight BARO */
//     MAG = 4,
//     HEADFREE = 5,
//     HEADADJ = 6,
//     CAMSTAB = 7,
//     NAVRTH = 8,     /* cleanflight GPSHOME */
//     NAVPOSHOLD = 9, /* cleanflight GPSHOLD */
//     PASSTHRU = 10,
//     BEEPERON = 11,
//     LEDLOW = 12,
//     LLIGHTS = 13,
//     OSD = 14,
//     TELEMETRY = 15,
//     GTUNE = 16,
//     SONAR = 17,
//     BLACKBOX = 18,
//     FAILSAFE = 19,
//     NAVWP = 20,     /* cleanflight AIRMODE */
//     AIRMODE = 21,   /* cleanflight DISABLE3DSWITCH */
//     HOMERESET = 22, /* cleanflight FPVANGLEMIX */
//     GCSNAV = 23,    /* cleanflight BLACKBOXERASE */
//     HEADINGLOCK = 24,
//     SURFACE = 25,
//     FLAPERON = 26,
//     TURNASSIST = 27,
//     NAVLAUNCH = 28,
//     AUTOTRIM = 29,
// }

pub struct EscSensorData {
    pub motor_count: u8,
    pub temperature: u8,
    pub rpm: u16,
}

pub const EC_SENSOR_DATA_SIZE: usize = 4;

impl EscSensorData {
    pub fn to_buf(&self) -> [u8; EC_SENSOR_DATA_SIZE] {
        let mut result = [0; EC_SENSOR_DATA_SIZE];

        result[0] = self.motor_count;
        result[1] = self.temperature;
        result[2..4].clone_from_slice(&self.rpm.to_le_bytes());

        result
    }
}

#[derive(Default)]
pub struct EscSensorDataDji {
    temperature: u8,
    rpm: u16,
}
//
// struct MotorTelemetry {
//     motor_count: u8,
//     rpm: u32,
//     invalid_percent: u16,
//     temperature: u8,
//     voltage: u16,
//     current: u16,
//     consumption: u16,
// }

// /// MSP_API_VERSION reply
// struct ApiVersion {
//     protocolVersion: u8,
//     APIMajor: u8,
//     APIMinor: u8,
// }
//
// /// MSP_FC_VARIANT reply
// #[derive(Default)]
// pub struct FcVariant {
//     flightControlIdentifier: [u8; 4],
// }
//
// /// MSP_FC_VERSION reply
// struct FcVersion {
//     versionMajor: u8,
//     versionMinor: u8,
//     versionPatchLevel: u8,
// }
//
// /// MSP_BOARD_INFO reply
// struct BoardInfo {
//     oardIdentifier: [u8; 4],
//     hardwareRevision: u16,
// }
//
// /// MSP_BUILD_INFO reply
// struct BuildInfo {
//     buildDate: [u8; 11],
//     buildTime: [u8; 8],
//     shortGitRevision: [u8; 7],
// }

// /// MSP_RAW_IMU reply
// struct RawImu {
//     acc: [i16; 3],  // x, y, z
//     gyro: [i16; 3], // x, y, z
//     mag: [i16; 3],  // x, y, z
// }
//
// // flags for msp_status_ex_t.sensor and msp_status_t.sensor
// const MSP_STATUS_SENSOR_ACC: u16 = 1;
// const MSP_STATUS_SENSOR_BARO: u16 = 2;
// const MSP_STATUS_SENSOR_MAG: u16 = 4;
// const MSP_STATUS_SENSOR_GPS: u16 = 8;
// const MSP_STATUS_SENSOR_SONAR: u16 = 16;

// /// MSP_STATUS_EX reply
// /// HHH I B HH BB I B
// struct StatusEx {
//     cycleTime: u16,
//     i2cErrorCounter: u16,
//     sensor: u16,          // MSP_STATUS_SENSOR_...
//     flightModeFlags: u32, // see getActiveModes()
//     nop_1: u8,
//     system_load: u16, // 0...100
//     gyro_time: u16,
//     nop_2: u8,
//     nop_3: u32,
//     extra: u8,
// }

// /// MSP_STATUS
// struct Status {
//     cycleTime: u16,
//     i2cErrorCounter: u16,
//     sensor: u16,          // MSP_STATUS_SENSOR_...
//     flightModeFlags: u32, // see getActiveModes()
//     configProfileIndex: u8,
//     gyroCycleTime: u16,
// }

// /// MSP_SENSOR_STATUS reply
// struct SensorStatus {
//     isHardwareHealthy: u8, // 0...1
//     hwGyroStatus: u8,
//     hwAccelerometerStatus: u8,
//     hwCompassStatus: u8,
//     hwBarometerStatus: u8,
//     hwGPSStatus: u8,
//     hwRangefinderStatus: u8,
//     hwPitotmeterStatus: u8,
//     hwOpticalFlowStatus: u8,
// }
//
// const MSP_MAX_SUPPORTED_SERVOS: usize = 8;
//
// /// MSP_SERVO reply
// struct Servo {
//     servo: [u16; MSP_MAX_SUPPORTED_SERVOS],
// }

// /// MSP_SERVO_CONFIGURATIONS reply
// struct ServoConfigurations {
//     min: u16,
//     max: u16,
//     middle: u16,
//     rate: u8,
//     angleAtMin: u8,
//     angleAtMax: u8,
//     forwardFromChannel: u8,
//     reversedSources: u32,
//     // conf[MSP_MAX_SUPPORTED_SERVOS]; // todo
// }

/*#define MSP_MAX_SERVO_RULES (2 * MSP_MAX_SUPPORTED_SERVOS)

// MSP_SERVO_MIX_RULES reply
struct msp_servo_mix_rules_t {
  __attribute__ ((packed)) struct {
    uint8_t targetChannel;
    uint8_t inputSource;
    uint8_t rate;
    uint8_t speed;
    uint8_t min;
    uint8_t max;
  } mixRule[MSP_MAX_SERVO_RULES];
}*/
//
// const MSP_MAX_SUPPORTED_MOTORS: usize = 8;
//
// /// MSP_MOTOR reply
// struct Motor {
//     motor: [u16; MSP_MAX_SUPPORTED_MOTORS],
// }

// const MSP_MAX_SUPPORTED_CHANNELS: usize = 16;
//
// /// MSP_RC reply
// struct Rc {
//     channelValue: [u16; MSP_MAX_SUPPORTED_CHANNELS],
// }

/// Attitude, as Euler angles. In degrees, multiplied by 10.
pub struct Attitude {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
}

pub const ATTITUDE_SIZE: usize = 6;

impl Attitude {
    pub fn to_buf(&self) -> [u8; ATTITUDE_SIZE] {
        let mut result = [0; ATTITUDE_SIZE];

        result[0..2].clone_from_slice(&self.roll.to_le_bytes());
        result[2..4].clone_from_slice(&self.pitch.to_le_bytes());
        result[4..6].clone_from_slice(&self.yaw.to_le_bytes());

        result
    }
}

/// MSP_ALTITUDE reply
#[derive(Default)]
pub struct Altitude {
    pub estimated_actual_position: i32, // cm
    pub estimated_actual_velocity: i16, // cm/s
    pub baro_latest_altitude: i32,
}

pub const ALTITUDE_SIZE: usize = 10;

impl Altitude {
    pub fn to_buf(&self) -> [u8; ALTITUDE_SIZE] {
        let mut result = [0; ALTITUDE_SIZE];

        result[0..4].clone_from_slice(&self.estimated_actual_position.to_le_bytes());
        result[4..6].clone_from_slice(&self.estimated_actual_velocity.to_le_bytes());
        result[6..10].clone_from_slice(&self.baro_latest_altitude.to_le_bytes());

        result
    }
}

// /// MSP_SONAR_ALTITUDE reply
// struct SonarAltitude {
//     altitude: i32,
// }

/// MSP_ANALOG reply
#[derive(Default)]
pub struct Analog {
    pub vbat: u8,       // 0...255
    pub mah_drawn: u16, // milliamp hours drawn from battery
    pub rssi: u16,      // 0..1023
    pub amperage: i16,  // send amperage in 0.01 A steps, range is -320A to 320A
}

/// MSP_ARMING_CONFIG reply
struct ArmingConfig {
    auto_disarm_delay: u8,
    disarm_kill_switch: u8,
}

// /// MSP_LOOP_TIME reply
// struct LoopTime {
//     looptime: u16,
// }

// /// MSP_RC_TUNING reply
// struct RcTuning {
//     rcRate8: u8, // no longer used
//     rcExpo8: u8,
//     rates: [u8; 3], // R,P,Y
//     dynThrPID: u8,
//     thrMid8: u8,
//     thrExpo8: u8,
//     tpa_breakpoint: u16,
//     rcYawExpo8: u8,
// }
//
// /// MSP_PID reply
// struct Pid {
//     roll: [u8; 3],    // 0=P, 1=I, 2=D
//     pitch: [u8; 3],   // 0=P, 1=I, 2=D
//     yaw: [u8; 3],     // 0=P, 1=I, 2=D
//     pos_z: [u8; 3],   // 0=P, 1=I, 2=D
//     pos_xy: [u8; 3],  // 0=P, 1=I, 2=D
//     vel_xy: [u8; 3],  // 0=P, 1=I, 2=D
//     surface: [u8; 3], // 0=P, 1=I, 2=D
//     level: [u8; 3],   // 0=P, 1=I, 2=D
//     heading: [u8; 3], // 0=P, 1=I, 2=D
//     vel_z: [u8; 3],   // 0=P, 1=I, 2=D
// }

// /// MSP_MISC reply
// struct Misc {
//     midrc: u16,
//     minthrottle: u16,
//     maxthrottle: u16,
//     mincommand: u16,
//     failsafe_throttle: u16,
//     gps_provider: u8,
//     gps_baudrate: u8,
//     gps_ubx_sbas: u8,
//     multiwii_current_meter_output: u8,
//     rssi_channel: u8,
//     dummy: u8,
//     mag_declination: u16,
//     vbatscale: u8,
//     vbatmincellvoltage: u8,
//     vbatmaxcellvoltage: u8,
//     vbatwarningcellvoltage: u8,
// }

/// values for msp_raw_gps_t.fixType
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum GpsFixType {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
}

impl Default for GpsFixType {
    fn default() -> Self {
        Self::NoFix
    }
}

/// Raw GPS data, ie directly from a GPS unit without further processing.
#[derive(Default)]
pub struct RawGps {
    pub fix_type: GpsFixType, // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
    pub num_sat: u8,
    pub lat: i32,           // 1 / 10000000 deg
    pub lon: i32,           // 1 / 10000000 deg
    pub alt: i16,           // meters
    pub ground_speed: i16,  // cm/s
    pub ground_course: i16, // unit: degree x 10
    pub hdop: u16,
}

pub const RAW_GPS_SIZE: usize = 18;

impl RawGps {
    pub fn to_buf(&self) -> [u8; RAW_GPS_SIZE] {
        let mut result = [0; RAW_GPS_SIZE];

        result[0] = self.fix_type as u8;
        result[1] = self.num_sat;
        result[2..6].clone_from_slice(&self.lat.to_le_bytes());
        result[6..10].clone_from_slice(&self.lon.to_le_bytes());
        result[10..12].clone_from_slice(&self.alt.to_le_bytes());
        result[12..14].clone_from_slice(&self.ground_speed.to_le_bytes());
        result[14..16].clone_from_slice(&self.ground_course.to_le_bytes());
        result[16..18].clone_from_slice(&self.hdop.to_le_bytes());

        result
    }
}

/// MSP_COMP_GPS reply
#[derive(Default)]
pub struct CompGps {
    pub distance_to_home: i16,  // distance to home in meters
    pub direction_to_home: i16, // direction to home in degrees
    pub heartbeat: u8,          // toggles 0 and 1 for each change
}

// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// /// values for msp_nav_status_t.mode
// pub enum NavStatusMode {
//     NONE = 0,
//     HOLD = 1,
//     RTH = 2,
//     NAV = 3,
//     EMERG = 15,
// }

// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// /// values for msp_nav_status_t.state
// enum NavStatusState {
//     NONE = 0,                // None
//     RTH_START = 1,           // RTH Start
//     RTH_ENROUTE = 2,         // RTH Enroute
//     HOLD_INFINIT = 3,        // PosHold infinit
//     HOLD_TIMED = 4,          // PosHold timed
//     WP_ENROUTE = 5,          // WP Enroute
//     PROCESS_NEXT = 6,        // Process next
//     DO_JUMP = 7,             // Jump
//     LAND_START = 8,          // Start Land
//     LAND_IN_PROGRESS = 9,    // Land in Progress
//     LANDED = 10,             // Landed
//     LAND_SETTLE = 11,        // Settling before land
//     LAND_START_DESCENT = 12, // Start descent
// }

// // values for msp_nav_status_t.activeWpAction, msp_set_wp_t.action
// const MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT: u8 = 0x01;
// const MSP_NAV_STATUS_WAYPOINT_ACTION_RTH: u8 = 0x04;
//
// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// /// values for msp_nav_status_t.error
// pub enum NavStatusError {
//     None = 0,          // All systems clear
//     TooFar = 1,        // Next waypoint distance is more than safety distance
//     SpoiledGps = 2,    // GPS reception is compromised - Nav paused - copter is adrift !
//     WpCrc = 3,         // CRC error reading WP data from EEPROM - Nav stopped
//     Finish = 4,        // End flag detected, navigation finished
//     TimeWait = 5,      // Waiting for poshold timer
//     InvalidJump = 6,   // Invalid jump target detected, aborting
//     InvalidData = 7,   // Invalid mission step action code, aborting, copter is adrift
//     WaitForRthAlt = 8, // Waiting to reach RTH Altitude
//     GpsFixLost = 9,    // Gps fix lost, aborting mission
//     Disarmed = 10,     // NAV engine disabled due disarm
//     Landing = 11,      // Landing
// }

// /// MSP_NAV_STATUS reply
// struct NavStatus {
//     mode: u8,             // one of MSP_NAV_STATUS_MODE_XXX
//     state: u8,            // one of MSP_NAV_STATUS_STATE_XXX
//     active_wp_action: u8, // combination of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
//     active_wp_number: u8,
//     error: u8, // one of MSP_NAV_STATUS_ERROR_XXX
//     mag_hold_heading: i16,
// }

// /// MSP_GPSSVINFO reply
// struct GpsSvInfo {
//     dummy1: u8,
//     dummy2: u8,
//     dummy3: u8,
//     dummy4: u8,
//     hdop: u8,
// }

// /// MSP_GPSSTATISTICS reply
// struct GpsStatistics {
//     last_message_dt: u16,
//     errors: u32,
//     timeouts: u32,
//     packet_count: u32,
//     hdop: u16,
//     eph: u16,
//     epv: u16,
// }

// /// MSP_UID reply
// struct Uid {
//     uid0: u32,
//     uid1: u32,
//     uid2: u32,
// }

#[derive(Clone, Copy, PartialEq)]
#[repr(u32)]
/// MSP_FEATURE mask
pub enum Feature {
    RxPpm = 1 << 0,
    Vbat = 1 << 1,
    Unused1 = 1 << 2,
    RxSerial = 1 << 3,
    MotorStop = 1 << 4,
    ServoTilt = 1 << 5,
    SoftSerial = 1 << 6,
    Gps = 1 << 7,
    Unused3 = 1 << 8,
    // was FEATURE_FAILSAFE
    Unused4 = 1 << 9,
    // was FEATURE_SONAR
    Telemetry = 1 << 10,
    CurrentMeter = 1 << 11,
    _3D = 1 << 12,
    RxParallelPwm = 1 << 13,
    RxMsp = 1 << 14,
    RssiAdc = 1 << 15,
    LedStrip = 1 << 16,
    Dashboard = 1 << 17,
    Unused2 = 1 << 18,
    Blackbox = 1 << 19,
    ChannelForwarding = 1 << 20,
    Transponder = 1 << 21,
    Airmode = 1 << 22,
    SuperexpoRates = 1 << 23,
    Vtx = 1 << 24,
    RxSpi = 1 << 25,
    SoftSpi = 1 << 26,
    PwmServoDriver = 1 << 27,
    PwmOutputEnable = 1 << 28,
    Osd = 1 << 29,
}

// /// MSP_FEATURE reply
// struct FeatureS {
//     featureMask: u32, // combination of MSP_FEATURE_XXX
// }

// /// MSP_BOARD_ALIGNMENT reply
// struct BoardAlignment {
//     rollDeciDegrees: i16,
//     pitchDeciDegrees: i16,
//     yawDeciDegrees: i16,
// }

// /// values for `CurrentMeterConfig`, current_meter_type
// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// pub enum CurrentMeterType {
//     SensorNone = 0,
//     SensorAdc = 1,
//     /// Virtual, or max
//     SensorVirtualMax = 2,
// }
//
// /// MSP_CURRENT_METER_CONFIG reply
// struct CurrentMeterConfig {
//     current_meter_scale: i16,
//     current_meter_offset: i16,
//     current_meter_type: CurrentMeterType,
//     battery_capacity: u16,
// }

// // msp_rx_config_t.serialrx_provider
// const MSP_SERIALRX_SPEKTRUM1024: u16 = 0;
// const MSP_SERIALRX_SPEKTRUM2048: u16 = 1;
// const MSP_SERIALRX_SBUS: u16 = 2;
// const MSP_SERIALRX_SUMD: u16 = 3;
// const MSP_SERIALRX_SUMH: u16 = 4;
// const MSP_SERIALRX_XBUS_MODE_B: u16 = 5;
// const MSP_SERIALRX_XBUS_MODE_B_RJ01: u16 = 6;
// const MSP_SERIALRX_IBUS: u16 = 7;
// const MSP_SERIALRX_JETIEXBUS: u16 = 8;
// const MSP_SERIALRX_CRSF: u16 = 9;
//
// // msp_rx_config_t.rx_spi_protocol values
// const MSP_SPI_PROT_NRF24RX_V202_250K: u16 = 0;
// const MSP_SPI_PROT_NRF24RX_V202_1M: u16 = 1;
// const MSP_SPI_PROT_NRF24RX_SYMA_X: u16 = 2;
// const MSP_SPI_PROT_NRF24RX_SYMA_X5C: u16 = 3;
// const MSP_SPI_PROT_NRF24RX_CX10: u16 = 4;
// const MSP_SPI_PROT_NRF24RX_CX10A: u16 = 5;
// const MSP_SPI_PROT_NRF24RX_H8_3D: u16 = 6;
// const MSP_SPI_PROT_NRF24RX_INAV: u16 = 7;

// /// MSP_RX_CONFIG reply
// struct RxConfig {
//     serialrx_provider: u8, // one of MSP_SERIALRX_XXX values
//     maxcheck: u16,
//     midrc: u16,
//     mincheck: u16,
//     spektrum_sat_bind: u8,
//     rx_min_usec: u16,
//     rx_max_usec: u16,
//     dummy1: u8,
//     dummy2: u8,
//     dummy3: u16,
//     rx_spi_protocol: u8, // one of MSP_SPI_PROT_XXX values
//     rx_spi_id: u32,
//     rx_spi_rf_channel_count: u8,
// }
//
// const MSP_MAX_MAPPABLE_RX_INPUTS: usize = 8;
//
// /// MSP_RX_MAP reply
// struct RxMap {
//     rxmap: [u8; MSP_MAX_MAPPABLE_RX_INPUTS], // [0]=roll channel, [1]=pitch channel, [2]=yaw channel, [3]=throttle channel, [3+n]=aux n channel, etc...
// }

// // values for msp_sensor_alignment_t.gyro_align, acc_align, mag_align
// const MSP_SENSOR_ALIGN_CW0_DEG: u16 = 1;
// const MSP_SENSOR_ALIGN_CW90_DEG: u16 = 2;
// const MSP_SENSOR_ALIGN_CW180_DEG: u16 = 3;
// const MSP_SENSOR_ALIGN_CW270_DEG: u16 = 4;
// const MSP_SENSOR_ALIGN_CW0_DEG_FLIP: u16 = 5;
// const MSP_SENSOR_ALIGN_CW90_DEG_FLIP: u16 = 6;
// const MSP_SENSOR_ALIGN_CW180_DEG_FLIP: u16 = 7;
// const MSP_SENSOR_ALIGN_CW270_DEG_FLIP: u16 = 8;

// /// MSP_SENSOR_ALIGNMENT reply
// struct SensorAlignment {
//     gyro_align: u8, // one of MSP_SENSOR_ALIGN_XXX
//     acc_align: u8,  // one of MSP_SENSOR_ALIGN_XXX
//     mag_align: u8,  // one of MSP_SENSOR_ALIGN_XXX
// }

// /// MSP_CALIBRATION_DATA reply
// struct CalibrationData {
//     accZeroX: i16,
//     accZeroY: i16,
//     accZeroZ: i16,
//     accGainX: i16,
//     accGainY: i16,
//     accGainZ: i16,
//     magZeroX: i16,
//     magZeroY: i16,
//     magZeroZ: i16,
// }

// /// MSP_SET_HEAD command
// struct SetHead {
//     magHoldHeading: i16, // degrees
// }

// /// MSP_SET_RAW_RC command
// struct SetRawRc {
//     channel: [u16; MSP_MAX_SUPPORTED_CHANNELS],
// }

// MSP_SET_PID command
// type msp_pid_t msp_set_pid_t;

// /// MSP_SET_RAW_GPS command
// struct SetRawGps {
//     fix_type: u8, // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
//     num_sat: u8,
//     lat: i32,          // 1 / 10000000 deg
//     lon: i32,          // 1 / 10000000 deg
//     alt: i16,          // meters
//     ground_speed: i16, // cm/s
// }

// /// MSP_SET_WP command
// /// Special waypoints are 0 and 255. 0 is the RTH position, 255 is the POSHOLD position (lat, lon, alt).
// struct SetWp {
//     waypoint_number: u8,
//     action: u8, // one of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
//     lat: i32,   // decimal degrees latitude * 10000000
//     lon: i32,   // decimal degrees longitude * 10000000
//     alt: i32,   // altitude (cm)
//     p1: i16, // speed (cm/s) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT, or "land" (value 1) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_RTH
//     p2: i16, // not used
//     p3: i16, // not used
//     flag: i8, // 0xa5 = last, otherwise set to 0
// }

/// todo: Where is this protocol defined, including its serialization?
/// todo: Is it specific to DJI? Vista/Air unit only, or also O3?
#[derive(Default)]
pub struct OsdConfig {
    pub osdflags: u8,
    pub video_system: u8,
    pub units: u8,
    pub rssi_alarm: u8,
    pub cap_alarm: u16,
    pub old_timer_alarm: u8,
    pub item_count: u8, //56
    pub alt_alarm: u16,
    pub rssi_value_pos: u16,
    pub main_batt_voltage_pos: u16,
    pub crosshairs_pos: u16,
    pub artificial_horizon_pos: u16,
    pub horizon_sidebars_pos: u16,
    pub item_timer_1_pos: u16,
    pub item_timer_2_pos: u16,
    pub flymode_pos: u16,
    pub craft_name_pos: u16,
    pub throttle_pos_pos: u16,
    pub vtx_channel_pos: u16,
    pub current_draw_pos: u16,
    pub mah_drawn_pos: u16,
    pub gps_speed_pos: u16,
    pub gps_sats_pos: u16,
    pub altitude_pos: u16,
    pub roll_pids_pos: u16,
    pub pitch_pids_pos: u16,
    pub yaw_pids_pos: u16,
    pub power_pos: u16,
    pub pidrate_profile_pos: u16,
    pub warnings_pos: u16,
    pub avg_cell_voltage_pos: u16,
    pub gps_lon_pos: u16,
    pub gps_lat_pos: u16,
    pub debug_pos: u16,
    pub pitch_angle_pos: u16,
    pub roll_angle_pos: u16,
    pub main_batt_usage_pos: u16,
    pub disarmed_pos: u16,
    pub home_dir_pos: u16,
    pub home_dist_pos: u16,
    pub numerical_heading_pos: u16,
    pub numerical_vario_pos: u16,
    pub compass_bar_pos: u16,
    pub esc_tmp_pos: u16,
    pub esc_rpm_pos: u16,
    pub remaining_time_estimate_pos: u16,
    pub rtc_datetime_pos: u16,
    pub adjustment_range_pos: u16,
    pub core_temperature_pos: u16,
    pub anti_gravity_pos: u16,
    pub g_force_pos: u16,
    pub motor_diag_pos: u16,
    pub log_status_pos: u16,
    pub flip_arrow_pos: u16,
    pub link_quality_pos: u16,
    pub flight_dist_pos: u16,
    pub stick_overlay_left_pos: u16,
    pub stick_overlay_right_pos: u16,
    pub display_name_pos: u16,
    pub esc_rpm_freq_pos: u16,
    pub rate_profile_name_pos: u16,
    pub pid_profile_name_pos: u16,
    pub profile_name_pos: u16,
    pub rssi_dbm_value_pos: u16,
    pub rc_channels_pos: u16,
    pub stat_count: u8, //24
    pub stat_rtc_date_time: u8,
    pub stat_timer_1: u8,
    pub stat_timer_2: u8,
    pub stat_max_speed: u8,
    pub stat_max_distance: u8,
    pub stat_min_battery: u8,
    pub stat_end_battery: u8,
    pub stat_battery: u8,
    pub stat_min_rssi: u8,
    pub stat_max_current: u8,
    pub stat_used_mah: u8,
    pub stat_max_altitude: u8,
    pub stat_blackbox: u8,
    pub stat_blackbox_number: u8,
    pub stat_max_g_force: u8,
    pub stat_max_esc_temp: u8,
    pub stat_max_esc_rpm: u8,
    pub stat_min_link_quality: u8,
    pub stat_flight_distance: u8,
    pub stat_max_fft: u8,
    pub stat_total_flights: u8,
    pub stat_total_time: u8,
    pub stat_total_dist: u8,
    pub stat_min_rssi_dbm: u8,
    pub timer_count: u16,
    pub timer_1: u16,
    pub timer_2: u16,
    pub enabledwarnings: u16,
    pub warning_count: u8, // 16
    pub enabledwarnings_1_41_plus: u32,
    pub profile_count: u8,      // 1
    pub osdprofileindex: u8,    // 1
    pub overlay_radio_mode: u8, //  0
}

pub const OSD_CONFIG_SIZE: usize = 51; // todo?

impl OsdConfig {
    pub fn to_buf(&self) -> [u8; OSD_CONFIG_SIZE] {
        let mut result = [0; OSD_CONFIG_SIZE];

        // todo: You must fill this in.

        // todo: We only include values we use.
        // todo: See note about: Where is this protocol defined, and is it DJI-specific?
        result[0] = self.osdflags;
        result[1] = self.video_system;
        result[2] = self.units;
        result[3] = self.rssi_alarm;
        result[4..6].clone_from_slice(&self.cap_alarm.to_le_bytes());
        result[6] = self.old_timer_alarm;
        result[7] = self.item_count;
        result[8..10].clone_from_slice(&self.alt_alarm.to_le_bytes());
        result[10..12].clone_from_slice(&self.rssi_value_pos.to_le_bytes());
        result[12..14].clone_from_slice(&self.main_batt_voltage_pos.to_le_bytes());
        result[14..16].clone_from_slice(&self.crosshairs_pos.to_le_bytes());
        result[16..18].clone_from_slice(&self.artificial_horizon_pos.to_le_bytes());
        result[18..20].clone_from_slice(&self.horizon_sidebars_pos.to_le_bytes());
        result[20..22].clone_from_slice(&self.item_timer_1_pos.to_le_bytes());
        result[22..24].clone_from_slice(&self.item_timer_2_pos.to_le_bytes());
        result[24..26].clone_from_slice(&self.craft_name_pos.to_le_bytes());
        result[26..28].clone_from_slice(&self.throttle_pos_pos.to_le_bytes());
        result[28..30].clone_from_slice(&self.vtx_channel_pos.to_le_bytes());
        result[30..32].clone_from_slice(&self.current_draw_pos.to_le_bytes());
        result[32..34].clone_from_slice(&self.mah_drawn_pos.to_le_bytes());
        result[34..36].clone_from_slice(&self.gps_speed_pos.to_le_bytes());
        result[36..38].clone_from_slice(&self.gps_sats_pos.to_le_bytes());
        result[38..40].clone_from_slice(&self.altitude_pos.to_le_bytes());
        result[40..42].clone_from_slice(&self.roll_pids_pos.to_le_bytes());
        result[42..44].clone_from_slice(&self.pitch_pids_pos.to_le_bytes());
        result[44..46].clone_from_slice(&self.yaw_pids_pos.to_le_bytes());
        result[46..48].clone_from_slice(&self.power_pos.to_le_bytes());
        result[48..50].clone_from_slice(&self.pidrate_profile_pos.to_le_bytes());

        // todo: Finish this.

        // pub power_pos: u16,
        // pub pidrate_profile_pos: u16,
        // pub warnings_pos: u16,
        // pub avg_cell_voltage_pos: u16,
        // pub gps_lon_pos: u16,
        // pub gps_lat_pos: u16,
        // pub debug_pos: u16,
        // pub pitch_angle_pos: u16,
        // pub roll_angle_pos: u16,
        // pub main_batt_usage_pos: u16,
        // pub disarmed_pos: u16,
        // pub home_dir_pos: u16,
        // pub home_dist_pos: u16,
        // pub numerical_heading_pos: u16,
        // pub numerical_vario_pos: u16,
        // pub compass_bar_pos: u16,
        // pub esc_tmp_pos: u16,
        // pub esc_rpm_pos: u16,
        // pub remaining_time_estimate_pos: u16,
        // pub rtc_datetime_pos: u16,
        // pub adjustment_range_pos: u16,
        // pub core_temperature_pos: u16,
        // pub anti_gravity_pos: u16,
        // pub g_force_pos: u16,
        // pub motor_diag_pos: u16,
        // pub log_status_pos: u16,
        // pub flip_arrow_pos: u16,
        // pub link_quality_pos: u16,
        // pub flight_dist_pos: u16,
        // pub stick_overlay_left_pos: u16,
        // pub stick_overlay_right_pos: u16,
        // pub display_name_pos: u16,
        // pub esc_rpm_freq_pos: u16,
        // pub rate_profile_name_pos: u16,
        // pub pid_profile_name_pos: u16,
        // pub profile_name_pos: u16,
        // pub rssi_dbm_value_pos: u16,
        // pub rc_channels_pos: u16,
        // pub stat_count: u8, //24
        // pub stat_rtc_date_time: u8,
        // pub stat_timer_1: u8,
        // pub stat_timer_2: u8,
        // pub stat_max_speed: u8,
        // pub stat_max_distance: u8,
        // pub stat_min_battery: u8,
        // pub stat_end_battery: u8,
        // pub stat_battery: u8,
        // pub stat_min_rssi: u8,
        // pub stat_max_current: u8,
        // pub stat_used_mah: u8,
        // pub stat_max_altitude: u8,
        // pub stat_blackbox: u8,
        // pub stat_blackbox_number: u8,
        // pub stat_max_g_force: u8,
        // pub stat_max_esc_temp: u8,
        // pub stat_max_esc_rpm: u8,
        // pub stat_min_link_quality: u8,
        // pub stat_flight_distance: u8,
        // pub stat_max_fft: u8,
        // pub stat_total_flights: u8,
        // pub stat_total_time: u8,
        // pub stat_total_dist: u8,
        // pub stat_min_rssi_dbm: u8,
        // pub timer_count: u16,
        // pub timer_1: u16,
        // pub timer_2: u16,
        // pub enabledwarnings: u16,
        // pub warning_count: u8, // 16
        // pub enabledwarnings_1_41_plus: u32,
        // pub profile_count: u8,      // 1
        // pub osdprofileindex: u8,    // 1
        // pub overlay_radio_mode: u8, //  0

        result
    }
}

#[derive(Default)]
pub struct Name {
    /// Craft name - up to 15 characters.
    pub craft_name: [u8; NAME_SIZE],
}

pub const NAME_SIZE: usize = 15;

#[derive(Default)]
pub struct BatteryState {
    pub battery_cell_count: u8,
    pub battery_capacity: u16,
    pub legacy_battery_voltage: u8,
    pub mah_drawn: u16,
    pub amperage: u16,
    pub battery_state: u8,
    pub battery_voltage: u16,
}

pub const BATTERY_STATE_SIZE: usize = 11;

impl BatteryState {
    pub fn to_buf(&self) -> [u8; BATTERY_STATE_SIZE] {
        let mut result = [0; BATTERY_STATE_SIZE];

        result[0] = self.battery_cell_count;
        result[1..3].clone_from_slice(&self.battery_capacity.to_le_bytes());
        result[3] = self.legacy_battery_voltage;
        result[4..6].clone_from_slice(&self.mah_drawn.to_le_bytes());
        result[6..8].clone_from_slice(&self.amperage.to_le_bytes());
        result[8] = self.battery_state;
        result[9..11].clone_from_slice(&self.battery_voltage.to_le_bytes());

        result
    }
}

/// MSP_STATUS reply customized for BF/DJI
#[derive(Default)]
pub struct StatusBf {
    pub task_delta_time: u16,
    pub i2c_error_count: u16,
    pub sensor_status: u16,
    pub flight_mode_flags: u32,
    pub pid_profile: u8,
    pub system_load: u16,
    pub gyro_cycle_time: u16,
    pub box_mode_flags: u8,
    pub arming_disable_flags_count: u8,
    pub arming_disable_flags: u32,
    pub extra_flags: u8,
}

pub const STATUS_BF_SIZE: usize = 22;

impl StatusBf {
    pub fn to_buf(&self) -> [u8; STATUS_BF_SIZE] {
        let mut result = [0; STATUS_BF_SIZE];

        result[0..2].clone_from_slice(&self.task_delta_time.to_le_bytes());
        result[2..4].clone_from_slice(&self.i2c_error_count.to_le_bytes());
        result[4..6].clone_from_slice(&self.sensor_status.to_le_bytes());
        result[6..10].clone_from_slice(&self.flight_mode_flags.to_le_bytes());
        result[10] = self.pid_profile;
        result[11..13].clone_from_slice(&self.system_load.to_le_bytes());
        result[13..15].clone_from_slice(&self.gyro_cycle_time.to_le_bytes());
        result[15] = self.box_mode_flags;
        result[16] = self.arming_disable_flags_count;
        result[17..21].clone_from_slice(&self.arming_disable_flags.to_le_bytes());
        result[21] = self.extra_flags;

        result
    }
}

// /// ArduPlane
// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// enum ArduPlaneModes {
//     Manual = 0,
//     Circle = 1,
//     Stabilize = 2,
//     Training = 3,
//     Acro = 4,
//     FLY_BY_WIRE_A = 5,
//     FLY_BY_WIRE_B = 6,
//     CRUISE = 7,
//     AUTOTUNE = 8,
//     AUTO = 10,
//     RTL = 11,
//     LOITER = 12,
//     TAKEOFF = 13,
//     AVOID_ADSB = 14,
//     GUIDED = 15,
//     INITIALISING = 16,
//     QSTABILIZE = 17,
//     QHOVER = 18,
//     QLOITER = 19,
//     QLAND = 20,
//     QRTL = 21,
//     QAUTOTUNE = 22,
//     QACRO = 23,
// }

//DJI supported flightModeFlags
// 0b00000001 acro/arm
// 0b00000010 stab
// 0b00000100 hor
// 0b00001000 head
// 0b00010000 !fs!
// 0b00100000 resc
// 0b01000000 acro
// 0b10000000 acro
