#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/common.cpp
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/include/common.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/options.cpp
//!
//! Reviewed against source files: 2022-03-19

use super::sx1280_regs::{
    FlrcBandwidths, FlrcCodingRates, FlrcGaussianFilter, LoRaBandwidths, LoRaCodingRates,
    LoraSpreadingFactors,
};

#[derive(Copy, Clone)]
#[repr(u8)]
enum TlmRatio {
    NO_TLM = 0,
    _1_128 = 1,
    _1_64 = 2,
    _1_32 = 3,
    _1_16 = 4,
    _1_8 = 5,
    _1_4 = 6,
    _1_2 = 7,
}

impl TlmRatio {
    pub fn value(&self) -> u8 {
        match self {
            Self::NO_TLM => 1,
            Self::_1_2 => 2,
            Self::_1_4 => 4,
            Self::_1_8 => 8,
            Self::_1_16 => 1,
            Self::_1_32 => 32,
            Self::_1_64 => 64,
            Self::_1_128 => 128,
            _ => 0,
        }
    }
}

#[derive(Copy, Clone)]
enum ConnectionState {
    connected,
    tentative,
    disconnected,
    disconnectPending, // used on modelmatch change to drop the connection
    MODE_STATES,
    /// States below here are special mode states
    noCrossfire,
    wifiUpdate,
    bleJoystick,
    /// Failure states go below here to display immediately
    FAILURE_STATES,
    radioFailed,
}

///On the TX, tracks what to do when the Tock timer fires
#[derive(Copy, Clone)]
enum TxTlmRcvPhase {
    ttrpTransmitting,   // Transmitting RC channels as normal
    ttrpPreReceiveGap, // Has switched to Receive mode for telemetry, but in the gap between TX done and Tock
    ttrpExpectingTelem, // Still in Receive mode, Tock has fired, receiving telem as far as we know
}

#[derive(Copy, Clone)]
#[repr(u8)]
enum RXtimerState {
    tim_disconnected = 0,
    tim_tentative = 1,
    tim_locked = 2,
}

#[derive(Copy, Clone)]
#[repr(u8)]
enum TlmHeader {
    RF_DOWNLINK_INFO = 0,
    RF_UPLINK_INFO = 1,
    RF_AIRMODE_PARAMETERS = 2,
}

#[derive(Copy, Clone)]
#[repr(u8)]
enum RfRates {
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

#[derive(Copy, Clone)]
enum RadioType {
    SX127x_LORA,
    SX128x_LORA,
    SX128x_FLRC,
}

struct PrefParams {
    index: u8,
    enum_rate: RfRates, // Max value of 4 since only 2 bits have been assigned in the sync package.
    RXsensitivity: i32, // expected RF sensitivity based on
    TOA: u32,           // time on air in microseconds
    DisconnectTimeoutMs: u32, // Time without a packet before receiver goes to disconnected (ms)
    RxLockTimeoutMs: u32, // Max time to go from tentative -> connected state on receiver (ms)
    SyncPktIntervalDisconnected: u32, // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
    SyncPktIntervalConnected: u32, // how often to send the SYNC_PACKET packet (ms) when there we have a connection
}

impl PrefParams {
    pub fn new(
        index: u8,
        enum_rate: RfRate,
        RXsensitivity: i32,
        TOA: u32,
        DisconnectTimeoutMs: u32,
        RxLockTimeoutMs: u32,
        SyncPktIntervalDisconnected: u32,
        SyncPktIntervalConnected: u32,
    ) -> Self {
        Self {
            index,
            enum_rate,
            RXsensitivity,
            TOA,
            DisconnectTimeoutMs,
            RxLockTimeoutMs,
            SyncPktIntervalDisconnected,
            SyncPktIntervalConnected,
        }
    }
}

/// Note: Some of these draw from one of two enums (FLRC, LoRa), so we store as their u8-reprs.
struct ModSettings {
    index: u8,
    radio_type: RadioType,
    rf_rate: RfRates, // Max value of 4 since only 2 bits have been assigned in the sync package.
    bw: u8,
    sf: u8,
    cr: u8,
    interval: u32,         // interval in us seconds that corresponds to that frequency
    TLMinterval: TlmRatio, // every X packets is a response TLM packet, should be a power of 2
    FHSShopInterval: u8, // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    PreambleLen: u8,
    PayloadLength: u8, // Number of OTA bytes to be sent.
}

impl ModSettings {
    pub fn new(
        index: u8,
        radio_type: RadioType,
        rf_rate: RfRates,
        bw: u8,
        sf: u8,
        cr: u8,
        interval: u32,
        TLMinterval: TlmRatio,
        FHSShopInterval: u8,
        PreambleLen: u8,
        PayloadLength: u8,
    ) -> Self {
        Self {
            index,
            radio_type,
            rf_rate,
            bw,
            sf,
            cr,
            interval,
            TLMinterval,
            FHSShopInterval,
            PreambleLen,
            PayloadLength,
        }
    }
}

// #ifndef UNIT_TEST
const RATE_MAX: u8 = 6; // 2xFLRC + 4xLoRa
const RATE_DEFAULT: u8 = 0; // Default to FLRC 1000Hz
const RATE_BINDING: u8 = 5; // 50Hz bind mode

const SYNC_PACKET_SWITCH_OFFSET: u8 = 0; // Switch encoding mode
const SYNC_PACKET_TLM_OFFSET: u8 = 2; // Telemetry ratio
const SYNC_PACKET_RATE_OFFSET: u8 = 5; // Rate index
const SYNC_PACKET_SWITCH_MASK: u8 = 0b11;
const SYNC_PACKET_TLM_MASK: u8 = 0b111;
const SYNC_PACKET_RATE_MASK: u8 = 0b111;
// #endif // UNIT_TEST

const AUX1: u8 = 4;
const AUX2: u8 = 5;
const AUX3: u8 = 6;
const AUX4: u8 = 7;
const AUX5: u8 = 8;
const AUX6: u8 = 9;
const AUX7: u8 = 10;
const AUX8: u8 = 11;
const AUX9: u8 = 12;
const AUX10: u8 = 13;
const AUX11: u8 = 14;
const AUX12: u8 = 15;

// ELRS SPECIFIC OTA CRC
// Koopman formatting https://users.ece.cmu.edu/~koopman/crc/
const ELRS_CRC_POLY: u8 = 0x07; // 0x83
const ELRS_CRC14_POLY: u8 = 0x2E57; // 0x372B

// Sx1280[1] only
const AirRateConfig: [ModSettings; RATE_MAX as usize] = [
    ModSettings::new(
        0,
        RadioType::SX128x_FLRC,
        RfRates::FLRC_1000HZ,
        FlrcBandwidths::BR_0_650_BW_0_6 as u8,
        FlrcGaussianFilter::BT_1 as u8,
        FlrcCodingRates::CR_1_2 as u8,
        1000,
        TlmRatio::_1_128,
        8,
        32,
        8,
    ),
    ModSettings::new(
        1,
        RadioType::SX128x_FLRC,
        RfRates::FLRC_500HZ,
        FlrcBandwidths::BR_0_650_BW_0_6 as u8,
        LoRaBandwidths::BW_0800 as u8,
        FlrcCodingRates::CR_1_2 as u8,
        2000,
        TlmRatio::_1_128,
        4,
        32,
        8,
    ),
    ModSettings::new(
        2,
        RadioType::SX128x_LORA,
        RfRates::LORA_500HZ,
        LoRaBandwidths::BW_0800 as u8,
        LoraSpreadingFactors::SF5 as u8,
        LoRaCodingRates::CR_4_6 as u8,
        2000,
        TlmRatio::_1_128,
        4,
        12,
        8,
    ),
    ModSettings::new(
        3,
        RadioType::SX128x_LORA,
        RfRates::LORA_250HZ,
        LoRaBandwidths::BW_0800 as u8,
        LoraSpreadingFactors::SF6 as u8,
        LoRaCodingRates::CR_4_7 as u8,
        4000,
        TlmRatio::_1_64,
        4,
        14,
        8,
    ),
    ModSettings::new(
        4,
        RadioType::SX128x_LORA,
        RfRates::LORA_150HZ,
        LoRaBandwidths::BW_0800 as u8,
        LoraSpreadingFactors::SF7 as u8,
        LoRaCodingRates::CR_4_7 as u8,
        6666,
        TlmRatio::_1_32,
        4,
        12,
        8,
    ),
    ModSettings::new(
        5,
        RadioType::SX128x_LORA,
        RfRates::LORA_50HZ,
        LoRaBandwidths::BW_0800 as u8,
        LoraSpreadingFactors::SF9 as u8,
        LoRaCodingRates::CR_4_6 as u8,
        20000,
        TlmRatio::NO_TLM,
        2,
        12,
        8,
    ),
];

const AirRateRFperf: [PrefParams; RATE_MAX as usize] = [
    PrefParams::new(0, RfRates::FLRC_1000HZ, -104, 389, 2500, 2500, 3, 5000),
    PrefParams::new(1, RfRates::FLRC_500HZ, -104, 389, 2500, 2500, 3, 5000),
    PrefParams::new(2, RfRates::LORA_500HZ, -105, 1665, 2500, 2500, 3, 5000),
    PrefParams::new(3, RfRates::LORA_250HZ, -108, 3300, 3000, 2500, 6, 5000),
    PrefParams::new(4, RfRates::LORA_150HZ, -112, 5871, 3500, 2500, 10, 5000),
    PrefParams::new(5, RfRates::LORA_50HZ, -117, 18443, 4000, 2500, 0, 5000),
];

fn get_elrs_airRateConfig(index: usize) -> ModSettings {
    let mut i = index as u8;
    if RATE_MAX <= i {
        // Set to last usable entry in the array
        i = RATE_MAX - 1;
    }
    AirRateConfig[i as usize]
}

fn get_elrs_RFperfParams(index: usize) -> PrefParams {
    let mut i = index as u8;
    if RATE_MAX <= i {
        // Set to last usable entry in the array
        i = RATE_MAX - 1;
    }
    AirRateRFperf[i as usize]
}

/// Convert enum_rate to index
fn enumRatetoIndex(rate: u8) -> u8 {
    for i in 0..RATE_MAX {
        let ModParams = get_elrs_airRateConfig(i as usize);
        if ModParams.rf_rate as u8 == rate {
            return i;
        }
    }
    // If 25Hz selected and not available, return the slowest rate available
    // else return the fastest rate available (500Hz selected but not available)
    if rate == RATE_25HZ {
        RATE_MAX - 1
    } else {
        0
    }
}

// expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
// expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;

static mut connectionState: ConnectionState = ConnectionState::disconnected;
static mut connectionHasModelMatch: bool = false;

// todo: Fix this binding ID.
static mut BindingUID: [u8; 6] = [0, 1, 2, 3, 4, 5]; // Special binding UID values
                                                     // if MY_UID {
                                                     //     = {MY_UID};
                                                     // }
                                                     // #else {
                                                     //          = [
                                                     //             (uint8_t)HAL_GetUIDw0(), (uint8_t)(HAL_GetUIDw0() >> 8),
                                                     //             (uint8_t)HAL_GetUIDw1(), (uint8_t)(HAL_GetUIDw1() >> 8),
                                                     //             (uint8_t)HAL_GetUIDw2(), (uint8_t)(HAL_GetUIDw2() >> 8)};
                                                     // }

static mut MasterUID: [u8; 6] = unsafe { [UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]] }; // Special binding UID values

static mut CRCInitializer: u16 = unsafe { ((UID[4] as u16) << 8) | (UID[5] as u16) };

unsafe fn uidMacSeedGet() -> u32 {
    ((UID[2] as u32) << 24) + ((UID[3] as u32) << 16) + ((UID[4] as u32) << 8) + UID[5]
}

// `options.cpp`

/// Our function to deal with the `target_name`.
pub fn encode_hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        write!(&mut s, "{:02x}", b).unwrap();
    }
    s
}

// #define QUOTE(arg) #arg
// #define STR(macro) QUOTE(macro)
// const target_name: &str = "\xBE\xEF\xCA\xFE";
const target_name: &str = &encode_hex(&[0xbE, 0xEF, 0xCA, 0xFE]);
const target_name_size: u8 = sizeof(target_name);
const device_name: &str = "Anyleaf Mercury G4";
const device_name_size: u8 = sizeof(device_name);
// const commit: [char; 1] [LATEST_COMMIT, 0];
// const version: [char; 1] = [LATEST_VERSION, 0];

const wifi_hostname: &str = "elrs_rx";
const wifi_ap_ssid: &str = "ExpressLRS RX";
const wifi_ap_password: &str = "expresslrs";
const wifi_ap_address: &str = "10.0.0.1";
const home_wifi_ssid: &str = "";
const home_wifi_password: &str = "";

// const char PROGMEM compile_options[] = {
// #ifdef MY_BINDING_PHRASE
//     "-DMY_BINDING_PHRASE=\"" STR(MY_BINDING_PHRASE) "\" "
// #endif

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
// };

// `rx_main.cpp`
