//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/tree/master/src/src


// `common.cpp`


const AirRateConfig: [ModSettings; RATE_MAX] = [
    [0, RADIO_TYPE_SX128x_LORA, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6,  2000, TLM_RATIO_1_128,  4, 12, 8],
    [1, RADIO_TYPE_SX128x_LORA, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7,  4000, TLM_RATIO_1_64,   4, 14, 8],
    [2, RADIO_TYPE_SX128x_LORA, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7,  6666, TLM_RATIO_1_32,   4, 12, 8],
    [3, RADIO_TYPE_SX128x_LORA, RATE_50HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 2, 12, 8]
];

const AirRateRFperf: [expresslrs_rf_pref_params; RATE_MAX] = [
    [0, RATE_500HZ, -105, 1665, 2500, 2500, 3, 5000],
    [1, RATE_250HZ, -108, 3300, 3000, 2500, 6, 5000],
    [2, RATE_150HZ, -112, 5871, 3500, 2500, 10, 5000],
    [3, RATE_50HZ, -117, 18443, 4000, 2500, 0, 5000]
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
    if rate == RATE_25HZ { RATE_MAX - 1 } else { 0 };
}

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;

connectionState_e connectionState = disconnected;
bool connectionHasModelMatch;

uint8_t BindingUID[6] = {0, 1, 2, 3, 4, 5}; // Special binding UID values
#if defined(MY_UID)
    uint8_t UID[6] = {MY_UID};
#else
    #ifdef PLATFORM_ESP32
        uint8_t UID[6];
        esp_err_t WiFiErr = esp_read_mac(UID, ESP_MAC_WIFI_STA);
    #elif PLATFORM_STM32
        uint8_t UID[6] = {
            (uint8_t)HAL_GetUIDw0(), (uint8_t)(HAL_GetUIDw0() >> 8),
            (uint8_t)HAL_GetUIDw1(), (uint8_t)(HAL_GetUIDw1() >> 8),
            (uint8_t)HAL_GetUIDw2(), (uint8_t)(HAL_GetUIDw2() >> 8)};
    #else
        uint8_t UID[6] = {0};
    #endif
#endif
uint8_t MasterUID[6] = {UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]}; // Special binding UID values

uint16_t CRCInitializer = (UID[4] << 8) | UID[5];

uint8_t ICACHE_RAM_ATTR TLMratioEnumToValue(uint8_t const enumval)
{
    switch (enumval)
    {
    case TLM_RATIO_NO_TLM:
        return 1;
    case TLM_RATIO_1_2:
        return 2;
    case TLM_RATIO_1_4:
        return 4;
    case TLM_RATIO_1_8:
        return 8;
    case TLM_RATIO_1_16:
        return 16;
    case TLM_RATIO_1_32:
        return 32;
    case TLM_RATIO_1_64:
        return 64;
    case TLM_RATIO_1_128:
        return 128;
    default:
        return 0;
    }
}

fn RateEnumToHz(eRate: u8) -> u16
{
    match eRate {
     RATE_1000HZ => 1000,
     RATE_500HZ => 500,
     RATE_250HZ => 250,
     RATE_200HZ => 200,
     RATE_150HZ => 150,
     RATE_100HZ => 100,
     RATE_50HZ => 50,
     RATE_25HZ => 25,
     RATE_4HZ => 4,
    _ => 1,
    }
}

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