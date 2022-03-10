#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/common.cpp
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/common.h


static mut BindingUID: [u8; 6] = [0; 6];
static mut UID: [u8; 6] = [0; 6];
static mut MasterUID: [u8; 6] = [0; 6];
static mut CRCInitializer: u16 = 0;

#[repr(u8)]
enum TlmRatio
{
    NO_TLM = 0,
    _1_128 = 1,
    _1_64 = 2,
    _1_32 = 3,
    _1_16 = 4,
    _1_8 = 5,
    _1_4 = 6,
    _1_2 = 7

}

impl TlmRatio {
    pub fn value(&self) -> u8 {
        match self {
            Self::_NO_TLM => 1,
            Self::_1_2 =>2,
            Self::_1_4 => 4,
            Self::_1_8 => 8,
            Self::_1_16 =>1,
            Self::_1_32 =>32,
            Self::_1_64 => 64,
            Self::_1_128 => 128,
            _ => 0,
        }
    }
}

    switch (enumval)
    {
    TLM_RATIO_NO_TLM => {
        1;
    TLM_RATIO_1_2 => {
        2;
    TLM_RATIO_1_4 => {
        4;
    TLM_RATIO_1_8 => {
        8;
    TLM_RATIO_1_16 => {
        16;
    TLM_RATIO_1_32 => {
        32;
    TLM_RATIO_1_64 => 64,
    TLM_RATIO_1_128 => 128,
    _ => 0,


enum ConnectionState
{
    connected,
    tentative,
    disconnected,
    disconnectPending, // used on modelmatch change to drop the connection
    MODE_STATES,
    // States below here are special mode states
    noCrossfire,
    wifiUpdate,
    bleJoystick,
    // Failure states go below here to display immediately
    FAILURE_STATES,
    radioFailed
}

/**
 * On the TX, tracks what to do when the Tock timer fires
 **/
enum TxTlmRcvPhase
{
    ttrpTransmitting,     // Transmitting RC channels as normal
    ttrpPreReceiveGap,    // Has switched to Receive mode for telemetry, but in the gap between TX done and Tock
    ttrpExpectingTelem    // Still in Receive mode, Tock has fired, receiving telem as far as we know
}

#[repr(u8)]
enum RXtimerState
{
    tim_disconnected = 0,
    tim_tentative = 1,
    tim_locked = 2
}

extern connectionState_e connectionState;
extern bool connectionHasModelMatch;

#[repr(u8)]
enum expresslrs_tlm_header
{
    RF_DOWNLINK_INFO = 0,
    RF_UPLINK_INFO = 1,
    RF_AIRMODE_PARAMETERS = 2
}

#[repr(u8)]
enum RfRate
{
    LORA_4HZ = 0,
    LORA_25HZ,
    LORA_50HZ,
    LORA_100HZ,
    LORA_150HZ,
    LORA_200HZ,
    LORA_250HZ,
    LORA_500HZ,
    FLRC_500HZ,
    FLRC_1000HZ,
} // Max value of 16 since only 4 bits have been assigned in the sync package.

impl RfRate {
    pub fn hz(&self) -> u16 {
        match self {
            Self::FLRC_1000HZ => 1000,
            Self::FLRC_500HZ => 500,
            Self::LORA_500HZ => 500,
            Self::LORA_250HZ => 250,
            Self::LORA_200HZ => 200,
            Self::LORA_150HZ => 150,
            Self::LORA_100HZ => 100,
            Self::LORA_50HZ => 50,
            Self::LORA_25HZ => 25,
            Self::LORA_4HZ => 4,
        }
    }
}


enum RadioType {
    SX127x_LORA,
    SX128x_LORA,
    SX128x_FLRC,
}

struct PrefParams
{
    index: u8,
    enum_rate: RfRate,                    // Max value of 4 since only 2 bits have been assigned in the sync package.
    RXsensitivity: i32,                // expected RF sensitivity based on
    TOA: u32,                         // time on air in microseconds
    DisconnectTimeoutMs: u32,         // Time without a packet before receiver goes to disconnected (ms)
    RxLockTimeoutMs: u32,             // Max time to go from tentative -> connected state on receiver (ms)
    SyncPktIntervalDisconnected: u32, // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
    SyncPktIntervalConnected: u32,    // how often to send the SYNC_PACKET packet (ms) when there we have a connection
}

impl PrefParams {
    pub fn new(index: u8, enum_rate: RfRate, RXsensitivity: i32, TOA: u32, DisconnectTimeoutMs: u32, RxLockTimeoutMs: u32,
    SyncPktIntervalDisconnect: u32, SyncPktIntervalConnected: u32) -> Self {
        Self {
            index, enum_rate, RXsensitivity, TOA, DisconnectTimeoutMs, RxLockTimeoutMs, SyncPktIntervalDisconnected, SyncPktIntervalConnected
        }
    }
}

struct ModSettings
{
    index: u8,
    radio_type: u8,
    enum_rate: RfRate,          // Max value of 4 since only 2 bits have been assigned in the sync package.
    bw: u8,
    sf: u8,
    cr: u8,
    interval: u32,         // interval in us seconds that corresponds to that frequency
    TLMinterval: u8,        // every X packets is a response TLM packet, should be a power of 2
    FHSShopInterval: u8,    // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    PreambleLen: u8,
    PayloadLength: u8,     // Number of OTA bytes to be sent.
}

// #ifndef UNIT_TEST
const RATE_MAX: u8 = 6;      // 2xFLRC + 4xLoRa
const RATE_DEFAULT: u8 = 0;  // Default to FLRC 1000Hz
const RATE_BINDING: u8 = 5;  // 50Hz bind mode

const SYNC_PACKET_SWITCH_OFFSET: u8 =    0;   // Switch encoding mode
const SYNC_PACKET_TLM_OFFSET : u8 =      2;   // Telemetry ratio
const SYNC_PACKET_RATE_OFFSET : u8 =     5;   // Rate index
const SYNC_PACKET_SWITCH_MASK : u8 =    0b11;
const SYNC_PACKET_TLM_MASK   : u8 =      0b111;
const SYNC_PACKET_RATE_MASK : u8 =       0b111;
// #endif // UNIT_TEST


const AUX1: u8 = 4;
const AUX2: u8 =  5;
const AUX3: u8 =  6;
const AUX4: u8 =  7;
const AUX5: u8 =  8;
const AUX6: u8 =  9;
const AUX7: u8 =  10;
const AUX8: u8 =  11;
const AUX9: u8 =  12;
const AUX10: u8 =  13;
const AUX11: u8 =  14;
const AUX12: u8 =  15;

//ELRS SPECIFIC OTA CRC
//Koopman formatting https://users.ece.cmu.edu/~koopman/crc/
const ELRS_CRC_POLY: u8 = 0x07; // 0x83
const ELRS_CRC14_POLY: u8 =  0x2E57; // 0x372B
// Sx1280[1] only
const AirRateConfig: [ModSettings; RATE_MAX as usize] = [
  ModSettings::new(0, RadioType::SX128x_FLRC, RATE_FLRC_1000HZ, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2,     1000, TLM_RATIO_1_128,  8, 32, 8),
    ModSettings::new(1, RadioType::SX128x_FLRC, RfRate::FLRC_500HZ,  SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2,     2000, TLM_RATIO_1_128,  4, 32, 8),
    ModSettings::new(2, RadioType::SX128x_LORA, RfRate::LORA_500HZ,  SX1280_LORA_BW_0800,         SX1280_LORA_SF5,  SX1280_LORA_CR_LI_4_6,  2000, TLM_RATIO_1_128,  4, 12, 8),
    ModSettings::new(3, RadioType::SX128x_LORA, RfRate::LORA_250HZ,  SX1280_LORA_BW_0800,         SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_7,  4000, TLM_RATIO_1_64,   4, 14, 8),
    ModSettings::new(4, RadioType::SX128x_LORA, RfRate::LORA_150HZ,  SX1280_LORA_BW_0800,         SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_7,  6666, TLM_RATIO_1_32,   4, 12, 8),
    ModSettings::new(5, RadioType::SX128x_LORA, RfRate::LORA_50HZ,   SX1280_LORA_BW_0800,         SX1280_LORA_SF9,  SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 2, 12, 8)

];

const AirRateRFperf: [PrefParams; RATE_MAX as usize] = [
    PrefParams::new(0, RfRate::FLRC_1000HZ, -104,   389, 2500, 2500,  3, 5000),
    PrefParams::new(1, RfRate::FLRC_500HZ,  -104,   389, 2500, 2500,  3, 5000),
    PrefParams::new(2, RfRate::LORA_500HZ,  -105,  1665, 2500, 2500,  3, 5000),
    PrefParams::new(3, RfRate::LORA_250HZ,  -108,  3300, 3000, 2500,  6, 5000),
    PrefParams::new(4, RfRate::LORA_150HZ,  -112,  5871, 3500, 2500, 10, 5000),
    PrefParams::new(5, RfRate::LORA_50HZ,   -117, 18443, 4000, 2500,  0, 5000)
];


fn get_elrs_airRateConfig(index: usize) -> ModSettings
{
    if RATE_MAX <= index
    {
        // Set to last usable entry in the array
        index = RATE_MAX - 1;
    }
    ExpressLRS_AirRateConfig[index]
}

fn get_elrs_RFperfParams(index: usize) -> RfPrefParams {
{
    if RATE_MAX <= index
    {
        // Set to last usable entry in the array
        index = RATE_MAX - 1;
    }
    AirRateRFperf[index]
}

fn enumRatetoIndex(rate: u8) -> u8
{ // convert enum_rate to index
    expresslrs_mod_settings_s const * ModParams;
        for i in 0..RATE_MAX {
        ModParams = get_elrs_airRateConfig(i);
        if ModParams.enum_rate == rate
        {
            return i;
        }
    }
    // If 25Hz selected and not available, return the slowest rate available
    // else return the fastest rate available (500Hz selected but not available)
    if rate == RATE_25HZ { RATE_MAX - 1 } else { 0 }
}

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;

connectionState_e connectionState = disconnected;
bool connectionHasModelMatch;

const BindingUID: [u8; 6] = [0, 1, 2, 3, 4, 5]; // Special binding UID values
// #if defined(MY_UID)
//     uint8_t UID[6] = {MY_UID};
// #else
//     #ifdef PLATFORM_ESP32
//         uint8_t UID[6];
//         esp_err_t WiFiErr = esp_read_mac(UID, ESP_MAC_WIFI_STA);
//     #elif PLATFORM_STM32
//         uint8_t UID[6] = {
//             (uint8_t)HAL_GetUIDw0(), (uint8_t)(HAL_GetUIDw0() >> 8),
//             (uint8_t)HAL_GetUIDw1(), (uint8_t)(HAL_GetUIDw1() >> 8),
//             (uint8_t)HAL_GetUIDw2(), (uint8_t)(HAL_GetUIDw2() >> 8)};
//     #else
//         uint8_t UID[6] = {0};
//     #endif
// #endif
const MasterUID: [u8; 6] = [UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]]; // Special binding UID values

const CRCInitializer: u16 = (UID[4] << 8) | UID[5];

fn uidMacSeedGet() -> u32
{
((UID[2] as u32) << 24) + ((UID[3] as u32) << 16) + ((UID[4] as u32) << 8) + UID[5]
}

// `options.cpp`

#define QUOTE(arg) #arg
#define STR(macro) QUOTE(macro)
const unsigned char target_name[] = "\xBE\xEF\xCA\xFE" STR(TARGET_NAME);
const target_name_size: u8 = sizeof(target_name);
const char device_name[] = DEVICE_NAME;
const device_name_size: u8 = sizeof(device_name);
const commit: [char; 1] [LATEST_COMMIT, 0];
const version: [char; 1] = [LATEST_VERSION, 0];

// #if defined(TARGET_TX)
const wifi_hostname: [char; 10] = "elrs_tx";
const wifi_ap_ssid: [char; 10] = "ExpressLRS TX";
// #else
const wifi_hostname: [char; 6] = "elrs_rx";
const wifi_ap_ssid: [char; 12] = "ExpressLRS RX";
// #endif
const wifi_ap_password: [char; 10] = "expresslrs";
const wifi_ap_address: [char; 10] = "10.0.0.1";

const home_wifi_ssid: [char; 0] = ""
//#ifdef HOME_WIFI_SSID
STR(HOME_WIFI_SSID)
//#endif
;
const home_wifi_password: [char; 0] = ""
//#ifdef HOME_WIFI_PASSWORD
STR(HOME_WIFI_PASSWORD)
//#endif
;

const char PROGMEM compile_options[] = {
#ifdef MY_BINDING_PHRASE
    "-DMY_BINDING_PHRASE=\"" STR(MY_BINDING_PHRASE) "\" "
#endif
//
// #ifdef TARGET_TX
//     #ifdef UNLOCK_HIGHER_POWER
//         "-DUNLOCK_HIGHER_POWER "
//     #endif
//     #ifdef NO_SYNC_ON_ARM
//         "-DNO_SYNC_ON_ARM "
//     #endif
//     #ifdef UART_INVERTED
//         "-DUART_INVERTED "
//     #endif
//     #ifdef DISABLE_ALL_BEEPS
//         "-DDISABLE_ALL_BEEPS "
//     #endif
//     #ifdef JUST_BEEP_ONCE
//         "-DJUST_BEEP_ONCE "
//     #endif
//     #ifdef DISABLE_STARTUP_BEEP
//         "-DDISABLE_STARTUP_BEEP "
//     #endif
//     #ifdef MY_STARTUP_MELODY
//         "-DMY_STARTUP_MELODY=\"" STR(MY_STARTUP_MELODY) "\" "
//     #endif
//     #ifdef WS2812_IS_GRB
//         "-DWS2812_IS_GRB "
//     #endif
//     #ifdef TLM_REPORT_INTERVAL_MS
//         "-DTLM_REPORT_INTERVAL_MS=" STR(TLM_REPORT_INTERVAL_MS) " "
//     #endif
//     #ifdef USE_TX_BACKPACK
//         "-DUSE_TX_BACKPACK "
//     #endif
//     #ifdef USE_BLE_JOYSTICK
//         "-DUSE_BLE_JOYSTICK "
//     #endif
// #endif
//
// #ifdef TARGET_RX
//     #ifdef LOCK_ON_FIRST_CONNECTION
//         "-DLOCK_ON_FIRST_CONNECTION "
//     #endif
//     #ifdef USE_R9MM_R9MINI_SBUS
//         "-DUSE_R9MM_R9MINI_SBUS "
//     #endif
//     #ifdef AUTO_WIFI_ON_INTERVAL
//         "-DAUTO_WIFI_ON_INTERVAL=" STR(AUTO_WIFI_ON_INTERVAL) " "
//     #endif
//     #ifdef USE_DIVERSITY
//         "-DUSE_DIVERSITY "
//     #endif
//     #ifdef RCVR_UART_BAUD
//         "-DRCVR_UART_BAUD=" STR(RCVR_UART_BAUD) " "
//     #endif
//     #ifdef RCVR_INVERT_TX
//         "-DRCVR_INVERT_TX "
//     #endif
//     #ifdef USE_R9MM_R9MINI_SBUS
//         "-DUSE_R9MM_R9MINI_SBUS "
//     #endif
// #endif
};

// `rx_main.cpp`