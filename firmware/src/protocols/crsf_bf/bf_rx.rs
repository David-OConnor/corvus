#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adaptation from Betaflight's general RF code, for use with CRSF>
//! https://github.com/betaflight/betaflight/blob/master/src/main/rx/rx.h
//! https://github.com/betaflight/betaflight/blob/master/src/main/rx/rx.c


//  * This file is part of Cleanflight and Betaflight.
//  *
//  * Cleanflight and Betaflight are free software. You can redistribute
//  * this software and/or modify this software under the terms of the
//  * GNU General Public License as published by the Free Software
//  * Foundation, either version 3 of the License, or (at your option)
//  * any later version.
//  *
//  * Cleanflight and Betaflight are distributed in the hope that they
//  * will be useful, but WITHOUT ANY WARRANTY; without even the implied
//  * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//  * See the GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with this software.
//  *
//  * If not, see <http://www.gnu.org/licenses/>.
//  */

use core::{
    // ffi::c_char,
    mem,
};

use super::rc_modes;

type TimeUs = u32;
type TimeDelta = u32;

// todo: These are defines; QC types.
const STICK_CHANNEL_COUNT: u8 = 4;

const PWM_RANGE_MIN: u16 = 1_000;
const PWM_RANGE_MAX: u16 = 2_000;
const PWM_RANGE: u16 = PWM_RANGE_MAX - PWM_RANGE_MIN;
const PWM_RANGE_MIDDLE: u16 = PWM_RANGE_MIN + (PWM_RANGE / 2);

const PWM_PULSE_MIN: u16 = 750; // minimum PWM pulse width which is considered valid
const PWM_PULSE_MAX: u16 = 2250; // maximum PWM pulse width which is considered valid

fn RXFAIL_STEP_TO_CHANNEL_VALUE(step: u16) -> u16 {
    (PWM_PULSE_MIN + 25 * step)
}
fn CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue: u16) -> u16 {
    // todo: Type on the param and output?
    (constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25
}
const MAX_RXFAIL_RANGE_STEP: u16 = ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25);

const DEFAULT_SERVO_MIN: u16 = 1000;
const DEFAULT_SERVO_MIDDLE: u16 = 1500;
const DEFAULT_SERVO_MAX: u16 = 2000;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum RxFrameState {
    PENDING = 0,
    COMPLETE = (1 << 0),
    FAILSAFE = (1 << 1),
    PROCESSING_REQUIRED = (1 << 2),
    DROPPED = (1 << 3),
}

// Todo: We can probably remove this, since we only use Crsf
pub enum SerialRxType {
    // SERIALRX_SPEKTRUM1024 = 0,
    // SERIALRX_SPEKTRUM2048 = 1,
    // SERIALRX_SBUS = 2,
    // SERIALRX_SUMD = 3,
    // SERIALRX_SUMH = 4,
    // SERIALRX_XBUS_MODE_B = 5,
    // SERIALRX_XBUS_MODE_B_RJ01 = 6,
    // SERIALRX_IBUS = 7,
    // SERIALRX_JETIEXBUS = 8,
    SERIALRX_CRSF = 9,
    // SERIALRX_SRXL = 10,
    // SERIALRX_TARGET_CUSTOM = 11,
    // SERIALRX_FPORT = 12,
    // SERIALRX_SRXL2 = 13,
    // SERIALRX_GHST = 14
}

// todo: Defines; not sure of type
const MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT: u8 = 12;
const MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT: u8 = 8;
const MAX_SUPPORTED_RC_CHANNEL_COUNT: usize = 18;

const NON_AUX_CHANNEL_COUNT: u8 = 4;
const MAX_AUX_CHANNEL_COUNT: usize = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

// #if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT
// const MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT: u8 = MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT;
// #else
const MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT: u8 = MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT;
// #endif

// extern const char rcChannelLetters[];

const rcData: [f32; MAX_SUPPORTED_RC_CHANNEL_COUNT] = [0.; MAX_SUPPORTED_RC_CHANNEL_COUNT]; // interval [1000;2000]

const RSSI_SCALE_MIN: u8 = 1;
const RSSI_SCALE_MAX: u8 = 255;

const RSSI_SCALE_DEFAULT: u8 = 100;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum RxFailsafeChannelMode {
    AUTO = 0,
    HOLD,
    SET,
    INVALID,
}

const RX_FAILSAFE_MODE_COUNT: u8 = 3; // #define

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum RxFailsafeChannelType {
    FLIGHT = 0,
    AUX,
}

const RX_FAILSAFE_TYPE_COUNT: u8 = 2;

struct RxFailsafeChannelConfig {
    pub mode: u8, // See rxFailsafeChannelMode_e
    pub step: u8,
}

// PG_DECLARE_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs);

struct RxChannelRangeConfig {
    min: u16,
    max: u16,
}

// PG_DECLARE_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs);

// struct rxRuntimeState_s;
// typedef float (*rcReadRawDataFnPtr)(const struct rxRuntimeState_s *rxRuntimeState, uint8_t chan); // used by receiver driver to return channel data
// typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeState_s *rxRuntimeState);
// typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeState_s *rxRuntimeState);
// typedef timeUs_t rcGetFrameTimeUsFn(void);  // used to retrieve the timestamp in microseconds for the last channel data frame

// todo: Probably not required.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum RxProvider {
    NONE = 0,
    PARALLEL_PWM,
    PPM,
    SERIAL,
    MSP,
    SPI,
}

struct RxRuntimeState {
    pub rxProvider: RxProvider,
    pub serialrxProvider: SerialRxType,
    pub channelCount: u8, // number of RC channels as reported by current input driver
    pub rxRefreshRate: u16,
    pub rcReadRawFn: ReadRawDataFn,
    pub rcFrameStatusFn: FrameStatusFn,
    pub rcProcessFrameFn: ProcessFrameFn,
    pub rcFrameTimeUsFn: GetFrameTimeUsFn,
    pub channelData: [u16; 69],
    pub frameData: [u8; 69], // todo void??
    pub lastRcFrameTimeUs: timeUs,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum RssiSource {
    NONE = 0,
    ADC,
    RX_CHANNEL,
    RX_PROTOCOL,
    MSP,
    FRAME_ERRORS,
    RX_PROTOCOL_CRSF,
}

// extern rssiSource_e rssiSource;

// todo: Consider removing this enum?
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum LinkQualitySource {
    NONE = 0,
    RX_PROTOCOL_CRSF,
    // RX_PROTOCOL_GHST,
}

// extern linkQualitySource_e linkQualitySource;

// extern rxRuntimeState_t rxRuntimeState; //!!TODO remove this extern, only needed once for channelCount

// struct rxConfig_s;

const RSSI_MAX_VALUE: u16 = 1023; // define

// void setRssiDirect(uint16_t newRssi, rssiSource_e source);
// void setRssi(uint16_t rssiValue, rssiSource_e source);
// void setRssiMsp(uint8_t newMspRssi);
// void updateRSSI(timeUs_t currentTimeUs);
// uint16_t getRssi(void);
// uint8_t getRssiPercent(void);
// bool isRssiConfigured(void);

const LINK_QUALITY_MAX_VALUE: u16 = 1023;

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


// const rcChannelLetters: [u8; 20] = "AERT12345678abcdefgh";
const rcChannelLetters: [u8; 20] = "AERT12345678abcdefgh".into();

static mut rssi: u16 = 0;                  // range: [0;1023]
static mut rssiDbm: i16 = CRSF_RSSI_MIN;    // range: [-130,20]
static mut lastMspRssiUpdateUs: TimeUs = 0;

static mut frameErrFilter: Pt1Filter = unsafe { mem::zeroed() };

// #ifdef USE_RX_LINK_QUALITY_INFO
static mut linkQuality: u16 = 0;
static mut rfMode: u8 = 0;
// #endif

// #ifdef USE_RX_LINK_UPLINK_POWER
// static uint16_t uplinkTxPwrMw = 0;  //Uplink Tx power in mW
// #endif

const RSSI_ADC_DIVISOR: u16 = (4096 / 1024); // todo define type
const RSSI_OFFSET_SCALING: f32 = (1024. / 100.0); // todo define type

static mut rssiSource: RssiSource = unsafe { mem::zeroed() };
static mut linkQualitySource: linkQualitySource = unsafe { mem::zeroed() };

static mut rxDataProcessingRequired: bool = false;
static mut auxiliaryProcessingRequired: bool = false;

static mut rxSignalReceived: bool = false;
static mut rxFlightChannelsValid: bool = false;
static mut rxChannelCount: u8 = 0;

static needRxSignalBefore: TimeUs = 0;
static suspendRxSignalUntil: TimeUs = 0;
static mut  skipRxSamples: u8 = 0;

static mut rcRaw: [f32; MAX_SUPPORTED_RC_CHANNEL_COUNT] = [0.; MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
static mut rcData: [f32; MAX_SUPPORTED_RC_CHANNEL_COUNT] = [0; MAX_SUPPORTED_RC_CHANNEL_COUNT];           // interval [1000;2000]
static mut validRxSignalTimeout: [u32; MAX_SUPPORTED_RC_CHANNEL_COUNT] = [-; MAX_SUPPORTED_RC_CHANNEL_COUNT];

// todo defines
const MAX_INVALID_PULSE_TIME_MS: u32 = 300;                   // hold time in milliseconds after bad channel or Rx link loss
// will not be actioned until the nearest multiple of 100ms
const PPM_AND_PWM_SAMPLE_COUNT: usize = 3;

const DELAY_20_MS: u32 = (20 * 1000);                         // 20ms in us
const DELAY_100_MS: u32 = (100 * 1000);                       // 100ms in us
const DELAY_1500_MS: u32 = (1500 * 1000);                     // 1.5 seconds in us
const SKIP_RC_SAMPLES_ON_RESUME: u32 = 2;                    // flush 2 samples to drop wrong measurements (timing independent)

// static mut rxRuntimeState = RxRuntimeState {};
static mut rxRuntimeState: RxRuntimeState = unsafe { mem::zeroed() };
static mut rcSampleIndex: u8 = 0;

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
// set default calibration to full range and 1:1 mapping
for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
}
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
fn pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfigs: &RxFailsafeChannelConfig) {
    for i in 0..MAX_SUPPORTED_RC_CHANNEL_COUNT {
        rxFailsafeChannelConfigs[i].mode = if i < NON_AUX_CHANNEL_COUNT { RxFailsafeChannelMode::AUTO } else { RxFailsafeChannelMode::HOLD };
        rxFailsafeChannelConfigs[i].step = if i == THROTTLE {
            CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
        } else {
            CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC)
        };
    }
}

fn resetAllRxChannelRangeConfigurations(rxChannelRangeConfig: &mut RxChannelRangeConfig) {
    // set default calibration to full range and 1:1 mapping
    for i in 0..NON_AUX_CHANNEL_COUNT {
        rxChannelRangeConfig.min = PWM_RANGE_MIN;
        rxChannelRangeConfig.max = PWM_RANGE_MAX;
        rxChannelRangeConfig += 1;
    }
}

fn nullReadRawRC(rxRuntimeState: &RxRuntimeState, channel: u8) -> f32 {
    // UNUSED(rxRuntimeState);
    // UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

fn nullFrameStatus(rxRuntimeState: &RxRuntimeState) -> u8 {
    // UNUSED(rxRuntimeState);

    return RX_FRAME_PENDING;
}

fn nullProcessFrame(rxRuntimeState: &RxRuntimeState) -> bool {
    // UNUSED(rxRuntimeState);

    return true;
}

fn isPulseValid(pulseDuration: u16) -> bool {
    return  pulseDuration >= rxConfig().rx_min_usec &&
        pulseDuration <= rxConfig().rx_max_usec;
}

// // #ifdef USE_SERIAL_RX
// static bool serialRxInit(rxConfig: &RxConfig, rxRuntimeState: &RxRuntimeState)
// {
// bool enabled = false;
// switch (rxRuntimeState->serialrxProvider) {
// #ifdef USE_SERIALRX_SRXL2
// case SERIALRX_SRXL2:
// enabled = srxl2RxInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_SPEKTRUM
// case SERIALRX_SRXL:
// case SERIALRX_SPEKTRUM1024:
// case SERIALRX_SPEKTRUM2048:
// enabled = spektrumInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_SBUS
// case SERIALRX_SBUS:
// enabled = sbusInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_SUMD
// case SERIALRX_SUMD:
// enabled = sumdInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_SUMH
// case SERIALRX_SUMH:
// enabled = sumhInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_XBUS
// case SERIALRX_XBUS_MODE_B:
// case SERIALRX_XBUS_MODE_B_RJ01:
// enabled = xBusInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_IBUS
// case SERIALRX_IBUS:
// enabled = ibusInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_JETIEXBUS
// case SERIALRX_JETIEXBUS:
// enabled = jetiExBusInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_CRSF
// case SERIALRX_CRSF:
// enabled = crsfRxInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_GHST
// case SERIALRX_GHST:
// enabled = ghstRxInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_TARGET_CUSTOM
// case SERIALRX_TARGET_CUSTOM:
// enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeState);
// break;
// #endif
// #ifdef USE_SERIALRX_FPORT
// case SERIALRX_FPORT:
// enabled = fportRxInit(rxConfig, rxRuntimeState);
// break;
// #endif
// default:
// enabled = false;
// break;
// }
// return enabled;
// }
// #endif

unsafe fn rxInit() {
    // if featureIsEnabled(FEATURE_RX_PARALLEL_PWM) {
    //     rxRuntimeState.rxProvider = RxProvider::PARALLEL_PWM;
    // } else if featureIsEnabled(FEATURE_RX_PPM) {
    //     rxRuntimeState.rxProvider = RxProvider::PPM;
    // } else if featureIsEnabled(FEATURE_RX_SERIAL) {
    rxRuntimeState.rxProvider = RxProvider::SERIAL;
    // } else if featureIsEnabled(FEATURE_RX_MSP) {
    //     rxRuntimeState.rxProvider = RxProvider::MSP;
    // } else if featureIsEnabled(FEATURE_RX_SPI) {
    //     rxRuntimeState.rxProvider = RxProvider::SPI;
    // } else {
    //     rxRuntimeState.rxProvider = RxProvider::NONE;
    // }

    rxRuntimeState.serialrxProvider = rxConfig().serialrx_provider;
    rxRuntimeState.rcReadRawFn = nullReadRawRC;
    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeState.rcProcessFrameFn = nullProcessFrame;
    rxRuntimeState.lastRcFrameTimeUs = 0;
    rcSampleIndex = 0;

    for i in 0..MAX_SUPPORTED_RC_CHANNEL_COUNT {
        rcData[i] = rxConfig().midrc;
        validRxSignalTimeout[i] = millis() + MAX_INVALID_PULSE_TIME_MS;
    }

    rcData[THROTTLE] = if featureIsEnabled(FEATURE_3D) {  rxConfig().midrc } else { rxConfig().rx_min_usec };

    // Initialize ARM switch to OFF position when arming via switch is defined
    // TODO - move to rc_mode.c
    for i in 0..MAX_MODE_ACTIVATION_CONDITION_COUNT {
        let modeActivationCondition = modeActivationConditions(i);
        if modeActivationCondition.modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition.range) {
            // ARM switch is defined, determine an OFF value
            let mut value = 0_u16;
            if modeActivationCondition.range.startStep > 0 {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition.range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition.range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationCondition.auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

    match rxRuntimeState.rxProvider {
        _ => (),
// #ifdef USE_SERIAL_RX
        RxProvider::SERIAL => {
            let enabled: bool = serialRxInit(rxConfig(), &rxRuntimeState);
            if !enabled {
                rxRuntimeState.rcReadRawFn = nullReadRawRC;
                rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            }
        }
    }
// #endif
//
// #if defined(USE_PWM) || defined(USE_PPM)
//     case RX_PROVIDER_PPM:
//     case RX_PROVIDER_PARALLEL_PWM:
//         rxPwmInit(rxConfig(), &rxRuntimeState);
//
//         break;
// #endif
//
// #if defined(USE_ADC)
//     if (featureIsEnabled(FEATURE_RSSI_ADC)) {
//         rssiSource = RssiSource::ADC;
//     } else
// #endif
    if rxConfig().rssi_channel > 0 {
        rssiSource = RssiSource::RX_CHANNEL;
    }

    // Setup source frame RSSI filtering to take averaged values every FRAME_ERR_RESAMPLE_US
    pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig().rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US/1000000.0));

    rxChannelCount = (rxConfig().max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeState.channelCount).min();
}

fn rxIsReceivingSignal() -> bool {
    return rxSignalReceived;
}

fn rxAreFlightChannelsValid() -> bool {
    return rxFlightChannelsValid;
}

fn suspendRxSignal() {
// #if defined(USE_PWM) || defined(USE_PPM)
//     if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
//         suspendRxSignalUntil = micros() + DELAY_1500_MS;  // 1.5s
//         skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
//     }
// #endif
    failsafeOnRxSuspend(DELAY_1500_MS);  // 1.5s
}

fn resumeRxSignal() {
// #if defined(USE_PWM) || defined(USE_PPM)
//     if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
//         suspendRxSignalUntil = micros();
//         skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
//     }
// #endif
    failsafeOnRxResume();
}

// #ifdef USE_RX_LINK_QUALITY_INFO
const LINK_QUALITY_SAMPLE_COUNT: usize = 16; // todo: Define type

static mut samples_ulqs: [u16; LINK_QUALITY_SAMPLE_COUNT] = 0; LINK_QUALITY_SAMPLE_COUNT;
static mut sampleIndex_ulqs: u8 = 0;
static mut sum_ulqs: u16 = 0;

unsafe fn updateLinkQualitySamples(value: u16) -> u16 {


    sum_ulqs += value - samples_ulqs[sampleIndex_ulqs];
    samples_ulqs[sampleIndex_ulqs] = value;
    sampleIndex_ulqs = (sampleIndex_ulqs + 1) % LINK_QUALITY_SAMPLE_COUNT;
    return sum_ulqs / LINK_QUALITY_SAMPLE_COUNT;
}

fn rxSetRfMode(rfModeValue: u8) {
    unsafe { rfMode = rfModeValue };
}
// #endif

static mut rssiSum: u16 = 0;
static mut rssiCount: u16 = 0;
static mut resampleTimeUs: TimeDelta = 0;

unsafe fn setLinkQuality(validFrame: bool, currentDeltaTimeUs: TimeDelta) {

// #ifdef USE_RX_LINK_QUALITY_INFO
    if linkQualitySource = LinkQualitySource::NONE {
        // calculate new sample mean
        linkQuality = updateLinkQualitySamples(if validFrame { LINK_QUALITY_MAX_VALUE } else { 0 });
    }
// #endif

    if rssiSource == RssiSource::FRAME_ERRORS {
        resampleTimeUs += currentDeltaTimeUs;
        rssiSum += if validFrame { RSSI_MAX_VALUE } else { 0 };
        rssiCount += 1;

        if resampleTimeUs >= FRAME_ERR_RESAMPLE_US {
            setRssi(rssiSum / rssiCount, rssiSource);
            rssiSum = 0;
            rssiCount = 0;
            resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
        }
    }
}

fn setLinkQualityDirect(linkqualityValue: u16) {
// #ifdef USE_RX_LINK_QUALITY_INFO
    unsafe { linkQuality = linkqualityValue; }
// #else
//     UNUSED(linkqualityValue);
// #endif
}

// #ifdef USE_RX_LINK_UPLINK_POWER
// void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue)
// {
//     uplinkTxPwrMw = uplinkTxPwrMwValue;
// }
// #endif

fn rxUpdateCheck(currentTimeUs: TimeUs, currentDeltaTimeUs: TimeDelta) -> bool {
    // UNUSED(currentTimeUs);
    // UNUSED(currentDeltaTimeUs);

    return taskUpdateRxMainInProgress() || unsafe { rxDataProcessingRequired || auxiliaryProcessingRequired };
}

unsafe fn rxFrameCheck(currentTimeUs: TimeUs, currentDeltaTimeUs: TimeDelta) {
    let mut signalReceived = false;
    let mut useDataDrivenProcessing = true;
    let mut needRxSignalMaxDelayUs: TimeDelta = DELAY_100_MS;

    // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 2, MIN(2000, currentDeltaTimeUs / 100));

    if taskUpdateRxMainInProgress() {
        //  no need to check for new data as a packet is being processed already
        return;
    }

    match rxRuntimeState.rxProvider {
        _ => (),
// #if defined(USE_PWM) || defined(USE_PPM)
//     case RX_PROVIDER_PPM:
//         if (isPPMDataBeingReceived()) {
//             signalReceived = true;
//             resetPPMDataReceivedState();
//         }
// 
//         break;
//     case RX_PROVIDER_PARALLEL_PWM:
//         if (isPWMDataBeingReceived()) {
//             signalReceived = true;
//             useDataDrivenProcessing = false;
//         }
// 
//         break;
// #endif
        RxProvider::SERIAL | RxProvider::MSP | RxProvider::SPI => {
            let frameStatus: u8 = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
            // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 1, (frameStatus & RX_FRAME_FAILSAFE));
            signalReceived = (frameStatus & RX_FRAME_COMPLETE) && !((frameStatus & RX_FRAME_FAILSAFE) || (frameStatus & RX_FRAME_DROPPED));
            setLinkQuality(signalReceived, currentDeltaTimeUs);
            auxiliaryProcessingRequired |= (frameStatus & RX_FRAME_PROCESSING_REQUIRED);
        }
    }

    if signalReceived {
        //  true only when a new packet arrives
        needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        rxSignalReceived = true; // immediately process packet data
    } else {
        //  watch for next packet
        if cmpTimeUs(currentTimeUs, needRxSignalBefore) > 0 {
            //  initial time to signalReceived failure is 100ms, then we check every 100ms
            rxSignalReceived = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            //  review rcData values every 100ms in failsafe changed them
            rxDataProcessingRequired = true;
        }
    }

    // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 0, rxSignalReceived);
    //  process the new Rx packet when it arrives
    rxDataProcessingRequired |= (signalReceived && useDataDrivenProcessing);

}

// #if defined(USE_PWM) || defined(USE_PPM)
// static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample)
// {
//     static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
//     static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
//     static bool rxSamplesCollected = false;
//
//     const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;
//
//     // update the recent samples and compute the average of them
//     rcSamples[chan][currentSampleIndex] = sample;
//
//     // avoid returning an incorrect average which would otherwise occur before enough samples
//     if (!rxSamplesCollected) {
//         if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
//             return sample;
//         }
//         rxSamplesCollected = true;
//     }
//
//     rcDataMean[chan] = 0;
//     for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
//         rcDataMean[chan] += rcSamples[chan][sampleIndex];
//     }
//     return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
// }
// #endif

fn getRxfailValue(channel: u8) -> u16 {
    const channelFailsafeConfig: RxFailsafeChannelConfig = rxFailsafeChannelConfigs(channel);

    match channelFailsafeConfig.mode {
        RX_FAILSAFE_MODE_AUTO => {
            match channel {
                ROLL | PITCH | YAW => {
                    return rxConfig().midrc;
                }
                THROTTLE => {
                    if featureIsEnabled(FEATURE_3D) && !IS_RC_MODE_ACTIVE(BOX3D) && !flight3DConfig().switched_mode3d {
                        return rxConfig().midrc;
                    } else {
                        return rxConfig().rx_min_usec;
                    }
                }
                _ => (),
            }
        }

        RX_FAILSAFE_MODE_INVALID |  RX_FAILSAFE_MODE_HOLD => {
            return rcData[channel];
        }

        RX_FAILSAFE_MODE_SET => {
            return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig.step as u16);
        }
    }
}

fn applyRxChannelRangeConfiguraton(sample: f32, range: &RxChannelRangeConfig) -> f32 {
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if sample == PPM_RCVR_TIMEOUT {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRangef(sample, range.min, range.max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    sample = constrainf(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

    return sample;
}

unsafe fn readRxChannelsApplyRanges() {
    for channel in 0..rxChannelCount {
        let rawChannel: u8 = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig().rcmap[channel] : channel;

        // sample the channel
        let mut sample = 0.;
// #if defined(USE_RX_MSP_OVERRIDE)
//         if (rxConfig().msp_override_channels_mask) {
//             sample = rxMspOverrideReadRawRc(&rxRuntimeState, rxConfig(), rawChannel);
//         } else
// #endif
        {
            sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);
        }

        // apply the rx calibration
        if channel < NON_AUX_CHANNEL_COUNT {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

unsafe fn detectAndApplySignalLossBehaviour() {
    let currentTimeMs: u32 = millis();
    let failsafeAuxSwitch: bool = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    let mut allAuxChannelsAreGood = true;
    // used to record if any non-aux channel is out of range for the timeout period, assume they are good
    rxFlightChannelsValid = rxSignalReceived && !failsafeAuxSwitch;
    //  set rxFlightChannelsValid false when a packet is bad or we use a failsafe switch

    for channel in 0..rxChannelCount {
        let mut sample: f32 = rcRaw[channel];
        let thisChannelValid: bool = rxFlightChannelsValid && isPulseValid(sample);
        //  if the packet is bad, we don't allow any channels to be good

        if thisChannelValid {
        //  reset the invalid pulse period timer for every good channel
        validRxSignalTimeout[channel] = currentTimeMs + MAX_INVALID_PULSE_TIME_MS;
        }

        if ARMING_FLAG(ARMED) && failsafeIsActive() {
            //  apply failsafe values, until failsafe ends, or disarmed, unless in GPS Return (where stick values should remain)
            if channel < NON_AUX_CHANNEL_COUNT {
            if !FLIGHT_MODE(GPS_RESCUE_MODE) {
                if channel == THROTTLE {
                        sample = failsafeConfig().failsafe_throttle;
                    } else {
                        sample = rxConfig().midrc;
                    }
                }
            } else if !failsafeAuxSwitch {
            // aux channels as Set in Configurator, unless failsafe initiated by switch
            sample = getRxfailValue(channel);
            }
        } else {
        if failsafeAuxSwitch {
            if channel < NON_AUX_CHANNEL_COUNT {
            sample = getRxfailValue(channel);
            //  set RPYT values to Stage 1 values immediately if initiated by switch
            }
        } else if !thisChannelValid {
        if cmp32(currentTimeMs, validRxSignalTimeout[channel]) < 0 {
        //  HOLD bad channel/s for MAX_INVALID_PULSE_TIME_MS (300ms) after Rx loss
        } else {
        //  then use STAGE 1 failsafe values
        if (channel < NON_AUX_CHANNEL_COUNT) {
        allAuxChannelsAreGood = false;
        //  declare signal lost after 300ms of at least one bad flight channel
        }
    sample = getRxfailValue(channel);
    //  set all channels to Stage 1 values
        }
        }
    }

    // #if defined(USE_PWM) || defined(USE_PPM)
    // if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM {
    // //  smooth output for PWM and PPM using moving average
    // rcData[channel] = calculateChannelMovingAverage(channel, sample);
    // } else
    // #endif

    {
    //  set rcData to either clean raw incoming values, or failsafe-modified values
    rcData[channel] = sample;
        }
        }

    if rxFlightChannelsValid && allAuxChannelsAreGood {
        failsafeOnValidDataReceived();
        //  --> start the timer to exit stage 2 failsafe
    } else {
    failsafeOnValidDataFailed();
    //  -> start timer to enter stage2 failsafe
    }

// DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 3, rcData[THROTTLE]);
}

unsafe fn calculateRxChannelsAndUpdateFailsafe(currentTimeUs: TimeUs) -> bool {
    if auxiliaryProcessingRequired {
        auxiliaryProcessingRequired = !rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
    }

    if !rxDataProcessingRequired {
        return false;
    }

    rxDataProcessingRequired = false;

    // only proceed when no more samples to skip and suspend period is over
    if skipRxSamples || currentTimeUs <= suspendRxSignalUntil {
        if currentTimeUs > suspendRxSignalUntil {
            skipRxSamples -= 1;
        }

        return true;
    }

    readRxChannelsApplyRanges();            // returns rcRaw
    detectAndApplySignalLossBehaviour();    // returns rcData

    rcSampleIndex += 1;

    return true;
}

fn parseRcChannels(input: u8, rxConfig: &RxConfig) {
    for c in input..c
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

fn setRssiDirect(newRssi: u16, source: RssiSource) {
    if unsafe { source != rssiSource } {
        return;
    }

    unsafe { rssi = newRssi; }
}

const RSSI_SAMPLE_COUNT: usize = 16; // todo: #define - type?

static mut samples: [u16; RSSI_SAMPLE_COUNT] = [0; RSSI_SAMPLE_COUNT];
static mut sampleIndex: u8 = 0;
static mut sum: usize = 16;

unsafe fn updateRssiSamples(value: u16) -> u16 {
    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % RSSI_SAMPLE_COUNT;
    return (sum / RSSI_SAMPLE_COUNT) as u16;
}

unsafe fn setRssi(rssiValue: u16, source: RssiSource) {
    if source != rssiSource {
        return;
    }

    // Filter RSSI value
    if source == RssiSource::FRAME_ERRORS {
        rssi = pt1FilterApply(&frameErrFilter, rssiValue);
    } else {
        // calculate new sample mean
        rssi = updateRssiSamples(rssiValue);
    }
}

fn setRssiMsp(newMspRssi: u8) {
    unsafe {
        if rssiSource == RssiSource::NONE {
            rssiSource = RssiSource::MSP;
        }

        if rssiSource == RssiSource::MSP {
            rssi = (newMspRssi as u16) << 2;
            lastMspRssiUpdateUs = micros();
        }
    }
}

fn updateRSSIPWM() {
    // Read value of AUX channel as rssi
    let pwmRssi: i16 = rcData[rxConfig().rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RssiSource::RX_CHANNEL);
}

static mut rssiUpdateAt: u32 = 0;

fn updateRSSIADC(currentTimeUs: TimeUs) {
// #ifndef USE_ADC
//     UNUSED(currentTimeUs);
// #else
//     static uint32_t rssiUpdateAt = 0;

    unsafe {
        if (int32_t)(currentTimeUs - rssiUpdateAt) < 0 {
            return;
        }
        rssiUpdateAt = currentTimeUs + DELAY_20_MS;
    }

    let adcRssiSample: u16 = adcGetChannel(ADC_RSSI);
    let rssiValue: u16 = adcRssiSample / RSSI_ADC_DIVISOR;

    setRssi(rssiValue, RssiSource::ADC);
    // #endif
}

fn updateRSSI(currentTimeUs: TimeUs, source: RssiSource) {
    match source {
        RssiSource::RX_CHANNEL => {
            updateRSSIPWM();
        }
        RssiSource::ADC => {
            updateRSSIADC(currentTimeUs);
        }
        RssiSource::MSP => unsafe {
            if cmpTimeUs(micros(), lastMspRssiUpdateUs) > DELAY_1500_MS {  // 1.5s
                rssi = 0;
            }
        }
        _ => (),
    }
}

fn getRssi() -> u16 {
    let mut rssiValue: u16 = unsafe { rssi };

    // RSSI_Invert option
    if rxConfig().rssi_invert {
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

    return rxConfig().rssi_scale / 100.0 * rssiValue + rxConfig().rssi_offset * RSSI_OFFSET_SCALING;
}

fn getRssiPercent() -> u8 {
    return scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 100);
}

fn getRssiDbm() -> i16 {
    return unsafe { rssiDbm };
}

const RSSI_SAMPLE_COUNT_DBM: i16 = 16; // todo - type?

static mut samplesdbm: [u16; RSSI_SAMPLE_COUNT_DBM as usize] = [0; RSSI_SAMPLE_COUNT_DBM];
static mut sampledbmIndex: u8 = 0;
static mut sumdbm: i16 = 0; // todo: Type?

unsafe fn updateRssiDbmSamples(value: i16) -> i16 {


    sumdbm += value - samplesdbm[sampledbmIndex];
    samplesdbm[sampledbmIndex] = value;
    sampledbmIndex = (sampledbmIndex + 1) % RSSI_SAMPLE_COUNT_DBM;
    return sumdbm / RSSI_SAMPLE_COUNT_DBM;
}

fn setRssiDbm(rssiDbmValue: i16, source: RssiSource) {
    if source != unsafe { rssiSource } {
        return;
    }

    unsafe { rssiDbm = updateRssiDbmSamples(rssiDbmValue); }
}

fn setRssiDbmDirect(newRssiDbm: i16,source: RssiSource) {
    if source != unsafe { rssiSource } {
    return;
    }

    unsafe { rssiDbm = newRssiDbm; }
}

// #ifdef USE_RX_LINK_QUALITY_INFO
fn rxGetLinkQuality() -> u16 {
return unsafe { linkQuality };
}

fn rxGetRfMode() -> u8 {
return unsafe { rfMode };
}

fn rxGetLinkQualityPercent() -> u16 {
    unsafe {
        if linkQualitySource == LinkQualitySource::NONE {
            scaleRange(linkQuality, 0, LINK_QUALITY_MAX_VALUE, 0, 100)
        } else {
            linkQuality
        }
    }
}
// #endif

#ifdef USE_RX_LINK_UPLINK_POWER
uint16_t rxGetUplinkTxPwrMw(void)
{
return uplinkTxPwrMw;
}
#endif

fn rxGetRefreshRate() -> u16 {
    unsafe { rxRuntimeState.rxRefreshRate }
}

fn isRssiConfigured() -> bool {
    unsafe { rssiSource != RssiSource::NONE }
}

static mut previousFrameTimeUs: TimeUs = 0;
static mut frameTimeDeltaUs: TimeDelta = 0;

unsafe fn rxGetFrameDelta(frameAgeUs: TimeDelta) -> TimeDelta {
    if rxRuntimeState.rcFrameTimeUsFn {
        let frameTimeUs: TimeUs = rxRuntimeState.rcFrameTimeUsFn();

        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);

        let deltaUs: TimeDelta = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if deltaUs {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }

    frameTimeDeltaUs
}

fn rxFrameTimeUs() -> TimeUs {
return unsafe { rxRuntimeState.lastRcFrameTimeUs };
}