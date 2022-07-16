// https://github.com/chris1seto/PX4-Autopilot/blob/turbotimber/src/modules/msp_osd/msp_defines.h
// (From that: Found on https://github.com/d3ngit/djihdfpv_mavlink_to_msp_V2/blob/master/Arduino_libraries/MSP/MSP.h)


const FLIGHT_CONTROLLER_IDENTIFIER_LENGTH: usize =   4;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// requests & replies
enum ReqsResps {
    API_VERSION = 1,
    FC_VARIANT = 2,
    FC_VERSION = 3,
    BOARD_INFO = 4,
    BUILD_INFO = 5,
    CALIBRATION_DATA = 14,
    FEATURE = 36,
    BOARD_ALIGNMENT = 38,
    CURRENT_METER_CONFIG = 40,
    RX_CONFIG = 44,
    SONAR_ALTITUDE = 58,
    ARMING_CONFIG = 61,
    RX_MAP = 64,
    // get channel map (also returns number of channels total)
    LOOP_TIME = 73,
    // FC cycle time i.e looptime parameter
    STATUS = 101,
    RAW_IMU = 102,
    SERVO = 103,
    MOTOR = 104,
    RC = 105,
    RAW_GPS = 106,
    COMP_GPS = 107,
    // distance home, direction home
    ATTITUDE = 108,
    ALTITUDE = 109,
    ANALOG = 110,
    RC_TUNING = 111,
    // rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
    PID = 112,
    // P I D coeff
    MISC = 114,
    SERVO_CONFIGURATIONS = 120,
    NAV_STATUS = 121,
    // navigation status
    SENSOR_ALIGNMENT = 126,
    // orientation of acc,gyro,mag
    ESC_SENSOR_DATA = 134,
    MOTOR_TELEMETRY = 139,
    STATUS_EX = 150,
    SENSOR_STATUS = 151,
    BOXIDS = 119,
    UID = 160,
    // Unique device ID
    GPSSVINFO = 164,
    // get Signal Strength (only U-Blox)
    GPSSTATISTICS = 166,
    // get GPS debugging data
    SET_PID = 202, // set P I D coeff
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Commands
enum Command {
    HEAD = 211,
    // define a new heading hold direction
    RAW_RC = 200,
    // 8 rc chan
    RAW_GPS = 201,
    // fix, numsat, lat, lon, alt, speed
    WP = 209, // sets a given WP (WP#, lat, lon, alt, flags)
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// bits of getActiveModes() return value
enum Mode {
    ARM = 0,
    ANGLE = 1,
    HORIZON = 2,
    NAVALTHOLD = 3, /* cleanflight BARO */
    MAG = 4,
    HEADFREE = 5,
    HEADADJ = 6,
    CAMSTAB = 7,
    NAVRTH = 8, /* cleanflight GPSHOME */
    NAVPOSHOLD = 9, /* cleanflight GPSHOLD */
    PASSTHRU = 10,
    BEEPERON = 11,
    LEDLOW = 12,
    LLIGHTS = 13,
    OSD = 14,
    TELEMETRY = 15,
    GTUNE = 16,
    SONAR = 17,
    BLACKBOX = 18,
    FAILSAFE = 19,
    NAVWP = 20, /* cleanflight AIRMODE */
    AIRMODE = 21, /* cleanflight DISABLE3DSWITCH */
    HOMERESET = 22, /* cleanflight FPVANGLEMIX */
    GCSNAV = 23, /* cleanflight BLACKBOXERASE */
    HEADINGLOCK = 24,
    SURFACE = 25,
    FLAPERON = 26,
    TURNASSIST = 27,
    NAVLAUNCH = 28,
    AUTOTRIM = 29,
}

struct EscSensorData {
    motor_count: u8,
    temperature: u8,
    rpm: u16,
}

struct EscSensorDataDji {
    temperature: u8,
    rpm: u16,
}

struct MotorTelemetry {
    motor_count: u8,
    rpm: u32,
    invalid_percent: u16,
    temperature: u8,
    voltage: u16,
    current: u16,
    consumption: u16,
}

/// MSP_API_VERSION reply
struct ApiVersion {
    protocolVersion: u8,
    APIMajor: u8,
    APIMinor: u8,
}


/// MSP_FC_VARIANT reply
struct FcVariant {
    flightControlIdentifier: [u8; 4],
}


/// MSP_FC_VERSION reply
struct FcVersion {
    versionMajor: u8,
    versionMinor: u8,
    versionPatchLevel: u8,
}


/// MSP_BOARD_INFO reply
struct BoardInfo {
    oardIdentifier: [u8; 4],
    hardwareRevision: u16,
}


/// MSP_BUILD_INFO reply
struct BuildInfo {
    buildDate: [u8; 11],
    buildTime: [u8; 8],
    shortGitRevision: [u8; 7],
}


/// MSP_RAW_IMU reply
struct RawImu {
    acc: [i16; 3],  // x, y, z
    gyro: [i16; 3], // x, y, z
    mag: [i16; 3],  // x, y, z
}


// flags for msp_status_ex_t.sensor and msp_status_t.sensor
const MSP_STATUS_SENSOR_ACC: u16 =    1;
const MSP_STATUS_SENSOR_BARO: u16 =    2;
const MSP_STATUS_SENSOR_MAG: u16 =     4;
const MSP_STATUS_SENSOR_GPS: u16 =     8;
const MSP_STATUS_SENSOR_SONAR: u16 =  16;


/// MSP_STATUS_EX reply
/// HHH I B HH BB I B
struct StatusEx {
    cycleTime: u16,
    i2cErrorCounter: u16,
    sensor: u16,                    // MSP_STATUS_SENSOR_...
    flightModeFlags: u32,           // see getActiveModes()
    nop_1: u8,
    system_load: u16,  // 0...100
    gyro_time: u16,
    nop_2: u8,
    nop_3: u32,
    extra: u8,
}


/// MSP_STATUS
struct Status {
    cycleTime: u16,
    i2cErrorCounter: u16,
    sensor: u16,             // MSP_STATUS_SENSOR_...
    flightModeFlags: u32,        // see getActiveModes()
    configProfileIndex: u8,
    gyroCycleTime: u16,
}


/// MSP_SENSOR_STATUS reply
struct SensorStatus {
    isHardwareHealthy: u8,  // 0...1
    hwGyroStatus: u8,
    hwAccelerometerStatus: u8,
    hwCompassStatus: u8,
    hwBarometerStatus: u8,
    hwGPSStatus: u8,
    hwRangefinderStatus: u8,
    hwPitotmeterStatus: u8,
    hwOpticalFlowStatus: u8,
}


const MSP_MAX_SUPPORTED_SERVOS: usize = 8;

/// MSP_SERVO reply
struct Servo {
    servo: [u16; MSP_MAX_SUPPORTED_SERVOS],
}


/// MSP_SERVO_CONFIGURATIONS reply
struct ServoConfigurations {
    min: u16,
    max: u16,
    middle: u16,
    rate: u8,
    angleAtMin: u8,
    angleAtMax: u8,
    forwardFromChannel: u8,
    reversedSources: u32,
    // conf[MSP_MAX_SUPPORTED_SERVOS]; // todo
}


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


const MSP_MAX_SUPPORTED_MOTORS: usize = 8;

/// MSP_MOTOR reply
struct msp_motor_t {
    uint16_t motor[MSP_MAX_SUPPORTED_MOTORS];
}


const MSP_MAX_SUPPORTED_CHANNELS: usize = 16;

/// MSP_RC reply
struct Rc{
    channelValue: [u16; MSP_MAX_SUPPORTED_CHANNELS],
}


/// MSP_ATTITUDE reply
struct Attitude {
    roll: i16,
    pitch: i16,
    yaw: i16,
}


/// MSP_ALTITUDE reply
struct Altitude {
    estimatedActualPosition: i32,  // cm
    estimatedActualVelocity: i16,  // cm/s
    baroLatestAltitude: i32,
}


/// MSP_SONAR_ALTITUDE reply
struct SonarAltitude {
    altitude: i32,
}


/// MSP_ANALOG reply
struct Analog {
    vbat: u8,    // 0...255
    mAhDrawn: u16, // milliamp hours drawn from battery
    rssi: u16,     // 0..1023
    amperage: i16, // send amperage in 0.01 A steps, range is -320A to 320A
}


/// MSP_ARMING_CONFIG reply
struct ArmingConfig {
    auto_disarm_delay: u8,
    disarm_kill_switch: u8,
}


// MSP_LOOP_TIME reply
struct msp_loop_time_t {
    uint16_t looptime;
}


// MSP_RC_TUNING reply
struct msp_rc_tuning_t {
    uint8_t  rcRate8;  // no longer used
    uint8_t  rcExpo8;
    uint8_t  rates[3]; // R,P,Y
    uint8_t  dynThrPID;
    uint8_t  thrMid8;
    uint8_t  thrExpo8;
    uint16_t tpa_breakpoint;
    uint8_t  rcYawExpo8;
}


// MSP_PID reply
struct msp_pid_t {
    uint8_t roll[3];     // 0=P, 1=I, 2=D
    uint8_t pitch[3];    // 0=P, 1=I, 2=D
    uint8_t yaw[3];      // 0=P, 1=I, 2=D
    uint8_t pos_z[3];    // 0=P, 1=I, 2=D
    uint8_t pos_xy[3];   // 0=P, 1=I, 2=D
    uint8_t vel_xy[3];   // 0=P, 1=I, 2=D
    uint8_t surface[3];  // 0=P, 1=I, 2=D
    uint8_t level[3];    // 0=P, 1=I, 2=D
    uint8_t heading[3];  // 0=P, 1=I, 2=D
    uint8_t vel_z[3];    // 0=P, 1=I, 2=D
}


// MSP_MISC reply
struct msp_misc_t {
    uint16_t midrc;
    uint16_t minthrottle;
    uint16_t maxthrottle;
    uint16_t mincommand;
    uint16_t failsafe_throttle;
    uint8_t  gps_provider;
    uint8_t  gps_baudrate;
    uint8_t  gps_ubx_sbas;
    uint8_t  multiwiiCurrentMeterOutput;
    uint8_t  rssi_channel;
    uint8_t  dummy;
    uint16_t mag_declination;
    uint8_t  vbatscale;
    uint8_t  vbatmincellvoltage;
    uint8_t  vbatmaxcellvoltage;
    uint8_t  vbatwarningcellvoltage;
}


// values for msp_raw_gps_t.fixType
#define MSP_GPS_NO_FIX 0
#define MSP_GPS_FIX_2D 1
#define MSP_GPS_FIX_3D 2


// MSP_RAW_GPS reply
struct msp_raw_gps_t {
    uint8_t  fixType;       // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
    uint8_t  numSat;
    int32_t  lat;           // 1 / 10000000 deg
    int32_t  lon;           // 1 / 10000000 deg
    int16_t  alt;           // meters
    int16_t  groundSpeed;   // cm/s
    int16_t  groundCourse;  // unit: degree x 10
    uint16_t hdop;
}


// MSP_COMP_GPS reply
struct msp_comp_gps_t {
    int16_t  distanceToHome;  // distance to home in meters
    int16_t  directionToHome; // direction to home in degrees
    uint8_t  heartbeat;       // toggles 0 and 1 for each change
}


#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// values for msp_nav_status_t.mode
enum NavStatusMode {
    NONE = 0,
    HOLD =  1,
    RTH   = 2,
    NAV  =  3,
    EMERG = 15,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// values for msp_nav_status_t.state
enum NavStatusState {
    NONE       =         0 , // None
    RTH_START   =        1 , // RTH Start
    RTH_ENROUTE   =      2 , // RTH Enroute
    HOLD_INFINIT  =      3 , // PosHold infinit
    HOLD_TIMED    =      4 , // PosHold timed
    WP_ENROUTE    =      5 , // WP Enroute
    PROCESS_NEXT    =    6 , // Process next
    DO_JUMP     =        7 , // Jump
    LAND_START  =        8 , // Start Land
    LAND_IN_PROGRESS  =  9,  // Land in Progress
    LANDED       =      10 , // Landed
    LAND_SETTLE   =     11 , // Settling before land
    LAND_START_DESCENT =12 , // Start descent
}

// values for msp_nav_status_t.activeWpAction, msp_set_wp_t.action
const MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT 0x01;
const MSP_NAV_STATUS_WAYPOINT_ACTION_RTH      0x04;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// values for msp_nav_status_t.error
enum NavStatusError {
    NONE          =     0; // All systems clear
    TOOFAR      =       1; // Next waypoint distance is more than safety distance
    SPOILED_GPS   =     2; // GPS reception is compromised - Nav paused - copter is adrift !
    WP_CRC     =        3; // CRC error reading WP data from EEPROM - Nav stopped
    FINISH      =       4; // End flag detected, navigation finished
    TIMEWAIT     =      5; // Waiting for poshold timer
    INVALID_JUMP  =     6; // Invalid jump target detected, aborting
    INVALID_DATA  =     7; // Invalid mission step action code, aborting, copter is adrift
    WAIT_FOR_RTH_ALT =  8; // Waiting to reach RTH Altitude
    GPS_FIX_LOST =      9; // Gps fix lost, aborting mission
    DISARMED     =     10; // NAV engine disabled due disarm
    LANDING    =       11;  // Landing
}


/// MSP_NAV_STATUS reply
struct NavStatus {
    mode: u8,         // one of MSP_NAV_STATUS_MODE_XXX
    state: u8,          // one of MSP_NAV_STATUS_STATE_XXX
    activeWpAction: u8,  // combination of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
    activeWpNumber: u8,
    error: u8,           // one of MSP_NAV_STATUS_ERROR_XXX
    magHoldHeading: i16,
}


/// MSP_GPSSVINFO reply
struct GpsSvInfo {
    dummy1: u8,
    dummy2: u8,
    dummy3: u8,
    dummy4: u8,
    HDOP: u8,
}


/// MSP_GPSSTATISTICS reply
struct msp_gpsstatistics_t {
    lastMessageDt: u16,
    errors: u32,
    timeouts: u32,
    packetCount: u32,
    hdop: u16,
    eph: u16,
    epv: u16,
}


/// MSP_UID reply
struct Uid {
    uid0: u32,
    uid1: u32,
    uid2: u32,
}



#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// MSP_FEATURE mask
enum Feature {
    RX_PPM: u16 = (1 << 0),
    VBAT: u16 = (1 << 1),
    UNUSED_1: u16 = (1 << 2),
    RX_SERIAL: u16 = (1 << 3),
    MOTOR_STOP: u16 = (1 << 4),
    SERVO_TILT: u16 = (1 << 5),
    SOFTSERIAL: u16 = (1 << 6),
    GPS: u16 = (1 << 7),
    UNUSED_3: u16 = (1 << 8),
    // was FEATURE_FAILSAFE
    UNUSED_4: u16 = (1 << 9),
    // was FEATURE_SONAR
    TELEMETRY: u16 = (1 << 10),
    CURRENT_METER: u16 = (1 << 11),
    3D: u16 = (1 << 12),
    RX_PARALLEL_PWM: u16 = (1 << 13),
    RX_MSP: u16 = (1 << 14),
    RSSI_ADC: u16 = (1 << 15),
    LED_STRIP: u16 = (1 << 16),
    DASHBOARD: u16 = (1 << 17),
    UNUSED_2: u16 = (1 << 18),
    BLACKBOX: u16 = (1 << 19),
    CHANNEL_FORWARDING: u16 = (1 << 20),
    TRANSPONDER: u16 = (1 << 21),
    AIRMODE: u16 = (1 << 22),
    SUPEREXPO_RATES: u16 = (1 << 23),
    VTX: u16 = (1 << 24),
    RX_SPI: u16 = (1 << 25),
    SOFTSPI: u16 = (1 << 26),
    PWM_SERVO_DRIVER: u16 = (1 << 27),
    PWM_OUTPUT_ENABLE: u16 = (1 << 28),
    OSD: u16 = (1 << 29),
}


/// MSP_FEATURE reply
struct Feature {
    featureMask: u32, // combination of MSP_FEATURE_XXX
}


/// MSP_BOARD_ALIGNMENT reply
struct BoardAlignment {
    rollDeciDegrees: i16,
    pitchDeciDegrees: i16,
    yawDeciDegrees: i16,
}


// values for msp_current_meter_config_t.currentMeterType
const MSP_CURRENT_SENSOR_NONE: u16 =    0;
const MSP_CURRENT_SENSOR_ADC: u16 =     1;
const MSP_CURRENT_SENSOR_VIRTUAL: u16 = 2;
const MSP_CURRENT_SENSOR_MAX: u16 =     CURRENT_SENSOR_VIRTUAL;


/// MSP_CURRENT_METER_CONFIG reply
struct CurrentMeterConfig {
    currentMeterScale: i16,
    currentMeterOffset: i16,
    currentMeterType: u8, // MSP_CURRENT_SENSOR_XXX
    batteryCapacity: u16,
}


// msp_rx_config_t.serialrx_provider
const MSP_SERIALRX_SPEKTRUM1024: u16 =      0;
const MSP_SERIALRX_SPEKTRUM2048   : u16 =     1;
const MSP_SERIALRX_SBUS     : u16 =           2;
const MSP_SERIALRX_SUMD    : u16 =            3;
const MSP_SERIALRX_SUMH    : u16 =            4;
const MSP_SERIALRX_XBUS_MODE_B   : u16 =      5;
const MSP_SERIALRX_XBUS_MODE_B_RJ01 : u16 =   6;
const MSP_SERIALRX_IBUS       : u16 =         7;
const MSP_SERIALRX_JETIEXBUS   : u16 =        8;
const MSP_SERIALRX_CRSF   : u16 =             9;


// msp_rx_config_t.rx_spi_protocol values
const MSP_SPI_PROT_NRF24RX_V202_250K : u16 =  0;
const MSP_SPI_PROT_NRF24RX_V202_1M : u16 =    1;
const MSP_SPI_PROT_NRF24RX_SYMA_X : u16 =     2;
const MSP_SPI_PROT_NRF24RX_SYMA_X5C : u16 =   3;
const MSP_SPI_PROT_NRF24RX_CX10 : u16 =       4;
const MSP_SPI_PROT_NRF24RX_CX10A : u16 =      5;
const MSP_SPI_PROT_NRF24RX_H8_3D : u16 =      6;
const MSP_SPI_PROT_NRF24RX_INAV  : u16 =      7;


/// MSP_RX_CONFIG reply
struct RxConfig {
    serialrx_provider: u8,  // one of MSP_SERIALRX_XXX values
    maxcheck: u16,
    midrc: u16,
    mincheck: u16,
    spektrum_sat_bind: u8,
    rx_min_usec: u16,
    rx_max_usec: u16,
    dummy1: u8,
    dummy2: u8,
    dummy3: u16,
    rx_spi_protocol: u8,  // one of MSP_SPI_PROT_XXX values
    rx_spi_id: u32,
    rx_spi_rf_channel_count: u8,
}


const MSP_MAX_MAPPABLE_RX_INPUTS 8;

/// MSP_RX_MAP reply
struct RxMap {
    rxmap: [u8; MSP_MAX_MAPPABLE_RX_INPUTS],  // [0]=roll channel, [1]=pitch channel, [2]=yaw channel, [3]=throttle channel, [3+n]=aux n channel, etc...
}

// values for msp_sensor_alignment_t.gyro_align, acc_align, mag_align
const MSP_SENSOR_ALIGN_CW0_DEG: u16 =        1;
const MSP_SENSOR_ALIGN_CW90_DEG : u16 =       2;
const MSP_SENSOR_ALIGN_CW180_DEG   : u16 =    3;
const MSP_SENSOR_ALIGN_CW270_DEG  : u16 =     4;
const MSP_SENSOR_ALIGN_CW0_DEG_FLIP : u16 =   5;
const MSP_SENSOR_ALIGN_CW90_DEG_FLIP : u16 =  6;
const MSP_SENSOR_ALIGN_CW180_DEG_FLIP: u16 =  7;
const MSP_SENSOR_ALIGN_CW270_DEG_FLIP : u16 = 8;

/// MSP_SENSOR_ALIGNMENT reply
struct SensorAlignment {
    gyro_align: u8,   // one of MSP_SENSOR_ALIGN_XXX
    acc_align: u8,    // one of MSP_SENSOR_ALIGN_XXX
    mag_align: u8,   // one of MSP_SENSOR_ALIGN_XXX
}


/// MSP_CALIBRATION_DATA reply
struct CalibrationData {
    accZeroX: i16,
    accZeroY: i16,
    accZeroZ: i16,
    accGainX: i16,
    accGainY: i16,
    accGainZ: i16,
    magZeroX: i16,
    magZeroY: i16,
    magZeroZ: i16,
}


/// MSP_SET_HEAD command
struct SetHead {
    magHoldHeading: i16, // degrees
}


/// MSP_SET_RAW_RC command
struct SetRawRc {
    channel: [u16; MSP_MAX_SUPPORTED_CHANNELS],
}


// MSP_SET_PID command
// type msp_pid_t msp_set_pid_t;


/// MSP_SET_RAW_GPS command
struct SetRawGps {
    fixType: u8,       // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
    numSat: u8,
    lat: i32,          // 1 / 10000000 deg
    lon: i32,         // 1 / 10000000 deg
    alt: i16,          // meters
    groundSpeed: i16,   // cm/s
}


/// MSP_SET_WP command
/// Special waypoints are 0 and 255. 0 is the RTH position, 255 is the POSHOLD position (lat, lon, alt).
struct SetWp {
    waypointNumber: u8,
    action: u8,  // one of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
    lat: i32,      // decimal degrees latitude * 10000000
    lon: i32,        // decimal degrees longitude * 10000000
    alt: i32,        // altitude (cm)
    p1: i16,         // speed (cm/s) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT, or "land" (value 1) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_RTH
    p2: i16,       // not used
    p3: i16,      // not used
    flag: i18,     // 0xa5 = last, otherwise set to 0
}

const MSP_OSD_CONFIG: u16 =            84;        //out message         Get osd settings - betaflight
const MSP_NAME    : u16 =               10;
const MSP_BATTERY_STATE  : u16 =        130;       //out message         Connected/Disconnected, Voltage, Current Used

struct OsdConfig {
    osdflags: u8,
    video_system: u8,
    units: u8,
    rssi_alarm: u8,
    cap_alarm: u16,
    old_timer_alarm: u8,
    osd_item_count: u8,                     //56
    alt_alarm: u16,
    osd_rssi_value_pos: u16,
    osd_main_batt_voltage_pos: u16,
    osd_crosshairs_pos: u16,
    osd_artificial_horizon_pos: u16,
    osd_horizon_sidebars_pos: u16,
    osd_item_timer_1_pos: u16,
    osd_item_timer_2_pos: u16,
    osd_flymode_pos: u16,
    osd_craft_name_pos: u16,
    osd_throttle_pos_pos: u16,
    osd_vtx_channel_pos: u16,
    osd_current_draw_pos: u16,
    osd_mah_drawn_pos: u16,
    osd_gps_speed_pos: u16,
    osd_gps_sats_pos: u16,
    osd_altitude_pos: u16,
    osd_roll_pids_pos: u16,
    osd_pitch_pids_pos: u16,
    osd_yaw_pids_pos: u16,
    osd_power_pos: u16,
    osd_pidrate_profile_pos: u16,
    osd_warnings_pos: u16,
    osd_avg_cell_voltage_pos: u16,
    osd_gps_lon_pos: u16,
    osd_gps_lat_pos: u16,
    osd_debug_pos: u16,
    osd_pitch_angle_pos: u16,
    osd_roll_angle_pos: u16,
    osd_main_batt_usage_pos: u16,
    osd_disarmed_pos: u16,
    osd_home_dir_pos: u16,
    osd_home_dist_pos: u16,
    osd_numerical_heading_pos: u16,
    osd_numerical_vario_pos: u16,
    osd_compass_bar_pos: u16,
    osd_esc_tmp_pos: u16,
    osd_esc_rpm_pos: u16,
    osd_remaining_time_estimate_pos: u16,
    osd_rtc_datetime_pos: u16,
    osd_adjustment_range_pos: u16,
    osd_core_temperature_pos: u16,
    osd_anti_gravity_pos: u16,
    osd_g_force_pos: u16,
    osd_motor_diag_pos: u16,
    osd_log_status_pos: u16,
    osd_flip_arrow_pos: u16,
    osd_link_quality_pos: u16,
    osd_flight_dist_pos: u16,
    osd_stick_overlay_left_pos: u16,
    osd_stick_overlay_right_pos: u16,
    osd_display_name_pos: u16,
    osd_esc_rpm_freq_pos: u16,
    osd_rate_profile_name_pos: u16,
    osd_pid_profile_name_pos: u16,
    osd_profile_name_pos: u16,
    osd_rssi_dbm_value_pos: u16,
    osd_rc_channels_pos: u16,
    osd_stat_count: u8,                    //24
    osd_stat_rtc_date_time: u8: u8,
    osd_stat_timer_1: u8,
    osd_stat_timer_2: u8,
    osd_stat_max_speed: u8,
    osd_stat_max_distance: u8,
    osd_stat_min_battery: u8,
    osd_stat_end_battery: u8,
    osd_stat_battery: u8,
    osd_stat_min_rssi: u8,
    osd_stat_max_current: u8,
    osd_stat_used_mah: u8,
    osd_stat_max_altitude: u8,
    osd_stat_blackbox: u8,
    osd_stat_blackbox_number: u8,
    osd_stat_max_g_force: u8,
    osd_stat_max_esc_temp: u8,
    osd_stat_max_esc_rpm: u8,
    osd_stat_min_link_quality: u8,
    osd_stat_flight_distance: u8,
    osd_stat_max_fft: u8,
    osd_stat_total_flights: u8,
    osd_stat_total_time: u8,
    osd_stat_total_dist: u8,
    osd_stat_min_rssi_dbm: u8,
    osd_timer_count: u16,
    osd_timer_1: u16,
    osd_timer_2: u16,
    enabledwarnings: u16,
    osd_warning_count: u8,              // 16
    enabledwarnings_1_41_plus: u32,
    osd_profile_count: u8,             // 1
    osdprofileindex: u8,               // 1
    overlay_radio_mode: u8,            //  0
}

struct Name {
    craft_name: [u8; 15];                    //15 characters max possible displayed in the goggles
}

struct BatteryState {
    batteryCellCount: u8,
    batteryCapacity: u16,
    legacyBatteryVoltage: u8,
    mAhDrawn: u16,
    amperage: u16,
    batteryState: u8,
    batteryVoltage: u16,
}

// MSP_STATUS reply customized for BF/DJI
struct StatusBf {
    task_delta_time: u16,
    i2c_error_count: u16,
    sensor_status: u16,
    flight_mode_flags: u32,
    pid_profile: u8,
    system_load: u16,
    gyro_cycle_time: u16,
    box_mode_flags: u8,
    arming_disable_flags_count: u8,
    arming_disable_flags: u32,
    extra_flags: u8,
}

/// ArduPlane
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum ArduPlaneModes {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    ACRO          = 4,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    CRUISE        = 7,
    AUTOTUNE      = 8,
    AUTO          = 10,
    RTL           = 11,
    LOITER        = 12,
    TAKEOFF       = 13,
    AVOID_ADSB    = 14,
    GUIDED        = 15,
    INITIALISING  = 16,
    QSTABILIZE    = 17,
    QHOVER        = 18,
    QLOITER       = 19,
    QLAND         = 20,
    QRTL          = 21,
    QAUTOTUNE     = 22,
    QACRO         = 23,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum BetaflightDjiModesMask {
    ARM_ACRO_BF = (1 << 0),
    STAB_BF     = (1 << 1),
    HOR_BF      = (1 << 2),
    HEAD_BF     = (1 << 3),
    FS_BF       = (1 << 4),
    RESC_BF     = (1 << 5)
}

//DJI supported flightModeFlags
// 0b00000001 acro/arm
// 0b00000010 stab
// 0b00000100 hor
// 0b00001000 head
// 0b00010000 !fs!
// 0b00100000 resc
// 0b01000000 acro
// 0b10000000 acro

