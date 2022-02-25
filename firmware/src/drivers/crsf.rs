//! This module interacts with a TBS Crossfire (CRSF) protocol radio receiver, eg ExpressLRS (ELRS). This is what
//!allows the aircraft to receive commands from a handheld radio receiver, etc.
//!
//! Translated from [Betaflight's implementation](https://github.com/betaflight/betaflight/blob/b8c58abf0e4a11a8bee54b9abbaed8683c65d5d2/src/main/rx/crsf.c)
//! .h file here: https://github.com/betaflight/betaflight/blob/b8c58abf0e4a11a8bee54b9abbaed8683c65d5d2/src/main/rx/crsf.h

use core::cmp::min;

use stm32_hal2::{pac::USART1, usart::Usart};

/// Possible ELS frequencies
#[derive(Copy, Clone)]
enum RadioFreq {
    /// 900 Hz. Requires larger antenna. Longer range
    F900,
    /// 2.4kHz. Can use a smaller antenna
    F2_400,
}

/// From `common/crc.c`
fn calc_crc(mut crc: u8, a: u8, poly: u8) -> u8 {
    crc ^= a;
    for i in 0..8 {
        if crc & 0x80 {
            crc = (crc << 1) ^ poly;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// `rx.h` here:

const STICK_CHANNEL_COUNT: usize = 4;

const PWM_RANGE_MIN: u16 = 1000;
const PWM_RANGE_MAX: u16 = 2000;
const PWM_RANGE: u16 = (PWM_RANGE_MAX - PWM_RANGE_MIN);
const PWM_RANGE_MIDDLE: u16 = (PWM_RANGE_MIN + (PWM_RANGE / 2));

const PWM_PULSE_MIN: u16 = 750; // minimum PWM pulse width which is considered valid
const PWM_PULSE_MAX: u16 = 2250; // maximum PWM pulse width which is considered valid

// const RXFAIL_STEP_TO_CHANNEL_VALUE(step): u16 =  (PWM_PULSE_MIN + 25 * step);
// const CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue) ((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25);
const MAX_RXFAIL_RANGE_STEP: u16 = ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25);

const DEFAULT_SERVO_MIN: u16 = 1000;
const DEFAULT_SERVO_MIDDLE: u16 = 1500;
const DEFAULT_SERVO_MAX: u16 = 2000;

#[repr(u8)]
enum RxFrameState {
    Pending = 0,
    Complete = (1 << 0),
    Failsafe = (1 << 1),
    ProcessingRequired = (1 << 2),
    Dropped = (1 << 3),
}

#[repr(u8)]
enum SerialRxType {
    SPEKTRUM1024 = 0,
    SPEKTRUM2048 = 1,
    SBUS = 2,
    SUMD = 3,
    SUMH = 4,
    XBUS_MODE_B = 5,
    XBUS_MODE_B_RJ01 = 6,
    IBUS = 7,
    JETIEXBUS = 8,
    CRSF = 9,
    SRXL = 10,
    TARGET_CUSTOM = 11,
    FPORT = 12,
    SRXL2 = 13,
    GHST = 14,
}

const MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT: usize = 12;
const MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT: usize = 8;
const MAX_SUPPORTED_RC_CHANNEL_COUNT: usize = 18;

const NON_AUX_CHANNEL_COUNT: usize = 4;
const MAX_AUX_CHANNEL_COUNT: usize = (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT);

// if MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT > MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT {
//     const  MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT = MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT;
// } else {
//     const MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT = MAX_SUPPORTED_RC_PPM_CHANNEL_COUNT;
// }

const RSSI_SCALE_MIN: u8 = 1;
const RSSI_SCALE_MAX: u8 = 255;

const RSSI_SCALE_DEFAULT: u8 = 100;

enum RxFailsafeChannelMode {
    Auto,
    Hold,
    Set,
    Invalid,
}

const RX_FAILSAFE_MODE_COUNT: usize = 3;

enum RxFailsafeChannelType {
    Flight,
    Aux,
}

const RX_FAILSAFE_TYPE_COUNT: usize = 2;

struct RxFailsafeChannelConfig {
    mode: u8, // See rxFailsafeChannelMode_e
    ustep: u8,
}

struct RxChannelRangeConfig {
    min: u16,
    max: u16,
}

// typedef float (*rcReadRawDataFnPtr)(const struct rxRuntimeState_s *rxRuntimeState, uint8_t chan); // used by receiver driver to return channel data
// typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeState_s *rxRuntimeState);
// typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeState_s *rxRuntimeState);
// typedef timeUs_t rcGetFrameTimeUsFn(void);  // used to retrieve the timestamp in microseconds for the last channel data frame

enum RxProvider {
    RX_PROVIDER_NONE,
    RX_PROVIDER_PARALLEL_PWM,
    RX_PROVIDER_PPM,
    RX_PROVIDER_SERIAL,
    RX_PROVIDER_MSP,
    RX_PROVIDER_SPI,
}

struct RxRuntimeState {
    rxProvider: RxProvider,
    serialrxProvider: SerialRxType,
    channelCount: u8, // number of RC channels as reported by current input driver
    rxRefreshRate: u16,
    rcReadRawFn: rcReadRawDataFnPtr,
    rcFrameStatusFnPtr: rcFrameStatusFnPtr,
    rcProcessFrameFnPtr: rcProcessFrameFnPtr,
    rcGetFrameTimeUsFn: rcGetFrameTimeUsFn, //*
    channelData: u16,                       //*
    frameData: Void,                        //*
    lastRcFrameTimeUs: TimeUs,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum RssiSource {
    NONE = 0,
    ADC = 1,
    RX_CHANNEL = 2,
    RX_PROTOCOL = 3,
    MSP = 4,
    FRAME_ERRORS = 5,
    RX_PROTOCOL_CRSF = 6,
}

enum LinkQualitySource {
    NONE,
    RX_PROTOCOL_CRSF,
    RX_PROTOCOL_GHST,
}

const RSSI_MAX_VALUE: u16 = 1023;

const LINK_QUALITY_MAX_VALUE: u16 = 1023;

// `rx.c` here:



const char rcChannelLetters[] = "AERT12345678abcdefgh";

static mut RSSI: u16 = 0;                  // range: [0;1023]
static mut rssiDbm: i16 = CRSF_RSSI_MIN;    // range: [-130,20]
static mut lastMspRssiUpdateUs: TimeUs = 0;

static pt1Filter_t frameErrFilter;

// If  USE_RX_LINK_QUALITY_INFO
static mut linkQuality: u16 = 0;
static mut rfMode: u8 = 0;

// If USE_RX_LINK_UPLINK_POWER
static mut uplinkTxPwrMw: u16 = 0;  //Uplink Tx power in mW

const MSP_RSSI_TIMEOUT_US: u32 = 1500000;   // 1.5 sec

const RSSI_ADC_DIVISOR: u16 = (4096 / 1024);
const RSSI_OFFSET_SCALING: f32 = (1024 / 100.0); // todo type?


static mut rxDataProcessingRequired: bool = false;
static mut auxiliaryProcessingRequired: bool = false;

static mut rxSignalReceived: bool = false;
static mut rxFlightChannelsValid: bool = false;
static mut rxIsInFailsafeMode: bool = true;
static mut rxChannelCount: u8 = 0;

static mut rxNextUpdateAtUs: TimeUs = 0;
static mut needRxSignalBefore: u32 = 0;
static mut needRxSignalMaxDelayUs: u32 = 0;
static mut suspendRxSignalUntil: u32 = 0;
static mut skipRxSamples: u8 = 0;

static float rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];           // interval [1000;2000]
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];

const MAX_INVALID_PULS_TIME: u32 =     300;
const PPM_AND_PWM_SAMPLE_COUNT: usize =  3;

const DELAY_50_HZ: u32 = (1000000 / 50);
const DELAY_33_HZ: u32 =  (1000000 / 33);
const DELAY_15_HZ: u32 =  (1000000 / 15);
const DELAY_10_HZ: u32 =  (1000000 / 10);
const DELAY_5_HZ: u32 =  (1000000 / 5);
const SKIP_RC_ON_SUSPEND_PERIOD: u32 =  1500000 ;          // 1.5 second period in usec (call frequency independent)
const SKIP_RC_SAMPLES_ON_RESUME: u32 =   2   ;             // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeState_t rxRuntimeState;
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
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig) {
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfig->min = PWM_RANGE_MIN;
        rxChannelRangeConfig->max = PWM_RANGE_MAX;
        rxChannelRangeConfig++;
    }
}

static float nullReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return RX_FRAME_PENDING;
}

static bool nullProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return true;
}

STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIAL_RX
static bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    bool enabled = false;
    switch (rxRuntimeState->serialrxProvider) {
#ifdef USE_SERIALRX_SRXL2
    case SERIALRX_SRXL2:
        enabled = srxl2RxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SRXL:
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_GHST
    case SERIALRX_GHST:
        enabled = ghstRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_TARGET_CUSTOM
    case SERIALRX_TARGET_CUSTOM:
        enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeState);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_PARALLEL_PWM;
    } else if (featureIsEnabled(FEATURE_RX_PPM)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_PPM;
    } else if (featureIsEnabled(FEATURE_RX_SERIAL)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;
    } else if (featureIsEnabled(FEATURE_RX_MSP)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_MSP;
    } else if (featureIsEnabled(FEATURE_RX_SPI)) {
        rxRuntimeState.rxProvider = RX_PROVIDER_SPI;
    } else {
        rxRuntimeState.rxProvider = RX_PROVIDER_NONE;
    }
    rxRuntimeState.serialrxProvider = rxConfig()->serialrx_provider;
    rxRuntimeState.rcReadRawFn = nullReadRawRC;
    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeState.rcProcessFrameFn = nullProcessFrame;
    rxRuntimeState.lastRcFrameTimeUs = 0;
    rcSampleIndex = 0;
    needRxSignalMaxDelayUs = DELAY_10_HZ;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig()->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (featureIsEnabled(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    // TODO - move to rc_mode.c
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(i);
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationCondition->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

    switch (rxRuntimeState.rxProvider) {
    default:

        break;
#ifdef USE_SERIAL_RX
    case RX_PROVIDER_SERIAL:
        {
            const bool enabled = serialRxInit(rxConfig(), &rxRuntimeState);
            if (!enabled) {
                rxRuntimeState.rcReadRawFn = nullReadRawRC;
                rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif

#ifdef USE_RX_MSP
    case RX_PROVIDER_MSP:
        rxMspInit(rxConfig(), &rxRuntimeState);
        needRxSignalMaxDelayUs = DELAY_5_HZ;

        break;
#endif

#ifdef USE_RX_SPI
    case RX_PROVIDER_SPI:
        {
            const bool enabled = rxSpiInit(rxSpiConfig(), &rxRuntimeState);
            if (!enabled) {
                rxRuntimeState.rcReadRawFn = nullReadRawRC;
                rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif

#if defined(USE_PWM) || defined(USE_PPM)
    case RX_PROVIDER_PPM:
    case RX_PROVIDER_PARALLEL_PWM:
        rxPwmInit(rxConfig(), &rxRuntimeState);

        break;
#endif
    }

#if defined(USE_ADC)
    if (featureIsEnabled(FEATURE_RSSI_ADC)) {
        rssiSource = RSSI_SOURCE_ADC;
    } else
#endif
    if (rxConfig()->rssi_channel > 0) {
        rssiSource = RSSI_SOURCE_RX_CHANNEL;
    }

    // Setup source frame RSSI filtering to take averaged values every FRAME_ERR_RESAMPLE_US
    pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US/1000000.0));

    rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeState.channelCount);
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

void suspendRxPwmPpmSignal(void)
{
#if defined(USE_PWM) || defined(USE_PPM)
    if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros() + SKIP_RC_ON_SUSPEND_PERIOD;
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
        failsafeOnRxSuspend(SKIP_RC_ON_SUSPEND_PERIOD);
    }
#endif
}

void resumeRxPwmPpmSignal(void)
{
#if defined(USE_PWM) || defined(USE_PPM)
    if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros();
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
        failsafeOnRxResume();
    }
#endif
}

#ifdef USE_RX_LINK_QUALITY_INFO
#define LINK_QUALITY_SAMPLE_COUNT 16

STATIC_UNIT_TESTED uint16_t updateLinkQualitySamples(uint16_t value)
{
    static uint16_t samples[LINK_QUALITY_SAMPLE_COUNT];
    static uint8_t sampleIndex = 0;
    static uint16_t sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % LINK_QUALITY_SAMPLE_COUNT;
    return sum / LINK_QUALITY_SAMPLE_COUNT;
}

void rxSetRfMode(uint8_t rfModeValue)
{
    rfMode = rfModeValue;
}
#endif

static void setLinkQuality(bool validFrame, timeDelta_t currentDeltaTimeUs)
{
    static uint16_t rssiSum = 0;
    static uint16_t rssiCount = 0;
    static timeDelta_t resampleTimeUs = 0;

    if USE_RX_LINK_QUALITY_INFO {
        if (linkQualitySource == LQ_SOURCE_NONE) {
            // calculate new sample mean
            linkQuality = updateLinkQualitySamples(validFrame ? LINK_QUALITY_MAX_VALUE : 0);
        }
    }

    if (rssiSource == RSSI_SOURCE_FRAME_ERRORS) {
        resampleTimeUs += currentDeltaTimeUs;
        rssiSum += validFrame ? RSSI_MAX_VALUE : 0;
        rssiCount++;

        if (resampleTimeUs >= FRAME_ERR_RESAMPLE_US) {
            setRssi(rssiSum / rssiCount, rssiSource);
            rssiSum = 0;
            rssiCount = 0;
            resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
        }
    }
}

void setLinkQualityDirect(uint16_t linkqualityValue)
{
#ifdef USE_RX_LINK_QUALITY_INFO
    linkQuality = linkqualityValue;
#else
    UNUSED(linkqualityValue);
#endif
}

#ifdef USE_RX_LINK_UPLINK_POWER
void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue)
{
    uplinkTxPwrMw = uplinkTxPwrMwValue;
}
#endif

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    bool signalReceived = false;
    bool useDataDrivenProcessing = true;

    if (taskUpdateRxMainInProgress()) {
        // There are more states to process
        return true;
    }

    switch (rxRuntimeState.rxProvider) {
    default:

        break;
#if defined(USE_PWM) || defined(USE_PPM)
    case RX_PROVIDER_PPM:
        if (isPPMDataBeingReceived()) {
            signalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            resetPPMDataReceivedState();
        }

        break;
    case RX_PROVIDER_PARALLEL_PWM:
        if (isPWMDataBeingReceived()) {
            signalReceived = true;
            rxIsInFailsafeMode = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            useDataDrivenProcessing = false;
        }

        break;
#endif
    case RX_PROVIDER_SERIAL:
    case RX_PROVIDER_MSP:
    case RX_PROVIDER_SPI:
        {
            const uint8_t frameStatus = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
            if (frameStatus & RX_FRAME_COMPLETE) {
                rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
                bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;
                signalReceived = !(rxIsInFailsafeMode || rxFrameDropped);
                if (signalReceived) {
                    needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
                }

                setLinkQuality(signalReceived, currentDeltaTimeUs);
            }

            if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
                auxiliaryProcessingRequired = true;
            }
        }

        break;
    }

    if (signalReceived) {
        rxSignalReceived = true;
    } else if (currentTimeUs >= needRxSignalBefore) {
        rxSignalReceived = false;
    }

    if ((signalReceived && useDataDrivenProcessing) || cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0) {
        rxDataProcessingRequired = true;
    }

    return rxDataProcessingRequired || auxiliaryProcessingRequired; // data driven or 50Hz
}

#if defined(USE_PWM) || defined(USE_PPM)
static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
    static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
    static bool rxSamplesCollected = false;

    const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

    // update the recent samples and compute the average of them
    rcSamples[chan][currentSampleIndex] = sample;

    // avoid returning an incorrect average which would otherwise occur before enough samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    rcDataMean[chan] = 0;
    for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
        rcDataMean[chan] += rcSamples[chan][sampleIndex];
    }
    return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}
#endif

fn getRxfailValue(channel: u8) -> u16
{
    const rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigs(channel);

    match channelFailsafeConfig.mode {
    RX_FAILSAFE_MODE_AUTO => {
        match channel {
            ROLL | PITCH | YAW => rxConfig().midrc,
            cTHROTTLE => {
                if (featureIsEnabled(FEATURE_3D) && !IS_RC_MODE_ACTIVE(BOX3D) && !flight3DConfig().switched_mode3d) {
                    return rxConfig().midrc;
                } else {
                    return rxConfig().rx_min_usec;
                }
            }
        }

    FALLTHROUGH;
    default:
    RX_FAILSAFE_MODE_INVALID | RX_FAILSAFE_MODE_HOLD => rcData[channel],
    RX_FAILSAFE_MODE_SET => RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig.step),
    }
}

STATIC_UNIT_TESTED float applyRxChannelRangeConfiguraton(float sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRangef(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    sample = constrainf(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxChannelCount; channel++) {

        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig()->rcmap[channel] : channel;

        // sample the channel
        float sample;
#if defined(USE_RX_MSP_OVERRIDE)
        if (rxConfig()->msp_override_channels_mask) {
            sample = rxMspOverrideReadRawRc(&rxRuntimeState, rxConfig(), rawChannel);
        } else
#endif
        {
            sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);
        }

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

static void detectAndApplySignalLossBehaviour(void)
{
    const uint32_t currentTimeMs = millis();

    const bool useValueFromRx = rxSignalReceived && !rxIsInFailsafeMode;

    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 0, rxSignalReceived);
    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 1, rxIsInFailsafeMode);

    rxFlightChannelsValid = true;
    for (int channel = 0; channel < rxChannelCount; channel++) {
        float sample = rcRaw[channel];

        const bool validPulse = useValueFromRx && isPulseValid(sample);

        if (validPulse) {
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        } else {
            if (cmp32(currentTimeMs, rcInvalidPulsPeriod[channel]) < 0) {
                continue;           // skip to next channel to hold channel value MAX_INVALID_PULS_TIME
            } else {
                sample = getRxfailValue(channel);   // after that apply rxfail value
                if (channel < NON_AUX_CHANNEL_COUNT) {
                    rxFlightChannelsValid = false;
                }
            }
        }
#if defined(USE_PWM) || defined(USE_PPM)
        if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
            // smooth output for PWM and PPM
            rcData[channel] = calculateChannelMovingAverage(channel, sample);
        } else
#endif
        {
            rcData[channel] = sample;
        }
    }

    if (rxFlightChannelsValid && !IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
        failsafeOnValidDataReceived();
    } else {
        rxIsInFailsafeMode = true;
        failsafeOnValidDataFailed();
        for (int channel = 0; channel < rxChannelCount; channel++) {
            rcData[channel] = getRxfailValue(channel);
        }
    }
    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 3, rcData[THROTTLE]);
}

bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    if (auxiliaryProcessingRequired) {
        auxiliaryProcessingRequired = !rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
    }

    if (!rxDataProcessingRequired) {
        return false;
    }

    rxDataProcessingRequired = false;
    rxNextUpdateAtUs = currentTimeUs + DELAY_15_HZ;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples || currentTimeUs <= suspendRxSignalUntil) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }

        return true;
    }

    readRxChannelsApplyRanges();
    detectAndApplySignalLossBehaviour();

    rcSampleIndex++;

    return true;
}

fn parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

fn setRssiDirect(newRssi: u16, source: RssiSource)
{
    if source != rssiSource {
        return;
    }

    unsafe {
        RSSI = newRssi;
    }
}

const RSSI_SAMPLE_COUNT: usize = 16;

fn updateRssiSamples(value: u16) -> u16
{
    let mut samples = [0; RSSI_SAMPLE_COUNT];
    let mut sampleIndex = 0;
    let mut sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % RSSI_SAMPLE_COUNT;

    sum / RSSI_SAMPLE_COUNT
}

fn setRssi(rssiValueL: u16, source: RssiSource)
{
    if source != rssiSource {
        return;
    }

    // Filter RSSI value
    if source == RssiSource::FRAME_ERRORS as u8 {
        unsafe {
            RSSI = pt1FilterApply(&frameErrFilter, rssiValue);
        }
    } else {
        // calculate new sample mean
        unsafe {
            RSSI = updateRssiSamples(rssiValue);
        }
    }
}

fn setRssiMsp(new_msp_rssi: u8)
{
    if rssiSource == RssiSource::NONE as u8 {
        rssiSource = RSSI_SOURCE_MSP;
    }

    if rssiSource == RSSI_SOURCE_MSP {
        unsafe {
        RSSI = (new_msp_rssi as u16) << 2;
        lastMspRssiUpdateUs = micros();
    }
    }
}

fn updateRSSIPWM()
{
    // Read value of AUX channel as rssi
    let pwmRssi: i16 = rcData[rxConfig()->rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}

fn updateRSSIADC(currentTimeUs: TimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint32_t rssiUpdateAt = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAt) < 0) {
        return;
    }
    rssiUpdateAt = currentTimeUs + DELAY_50_HZ;

    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint16_t rssiValue = adcRssiSample / RSSI_ADC_DIVISOR;

    setRssi(rssiValue, RSSI_SOURCE_ADC);
#endif
}

fn updateRSSI(currentTimeUs: TimeUs)
{
    match rssiSource {
        RssiSource::RX_CHANNEL => {
            updateRSSIPWM();
        }
        RssiSource::ADC => {
            updateRSSIADC(currentTimeUs);
        }
        RssiSource::MSP => {
            if (cmpTimeUs(micros(), lastMspRssiUpdateUs) > MSP_RSSI_TIMEOUT_US) {
                RSSI = 0;
            }
        }
        _ => ()
    }
}

fn getRssi() -> u16
{
   let mut rssiValue: u16 = RSSI;

    // RSSI_Invert option
    if rxConfig().rssi_invert {
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

    rxConfig().rssi_scale / 100.0 * rssiValue + rxConfig().rssi_offset * RSSI_OFFSET_SCALING
}

fn getRssiPercent() -> u8
{
    scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 100)
}

fn getRssiDbm() -> u16
{
    rssiDbm
}

const RSSI_SAMPLE_COUNT_DBM: u16 = 16;

fn updateRssiDbmSamples(value: i16) -> i16
{
    static int16_t samplesdbm[RSSI_SAMPLE_COUNT_DBM];
    static uint8_t sampledbmIndex = 0;
    static int sumdbm = 0;

    sumdbm += value - samplesdbm[sampledbmIndex];
    samplesdbm[sampledbmIndex] = value;
    sampledbmIndex = (sampledbmIndex + 1) % RSSI_SAMPLE_COUNT_DBM;
    return sumdbm / RSSI_SAMPLE_COUNT_DBM;
}

fn setRssiDbm(rssiDbmValue: i16, source: RssiSource)
{
    if source != rssiSource {
        return;
    }

    rssiDbm = updateRssiDbmSamples(rssiDbmValue);
}

fn setRssiDbmDirect(newRssiDbm: i16, source: RssiSource)
{
    if source != rssiSource {
        return;
    }

    rssiDbm = newRssiDbm;
}

/// If USE_RX_LINK_QUALITY_INFO
fn rxGetLinkQuality() -> u16
{
    return linkQuality;
}

uint8_t rxGetRfMode(void)
{
    return rfMode;
}

uint16_t rxGetLinkQualityPercent(void)
{
    return (linkQualitySource == LQ_SOURCE_NONE) ? scaleRange(linkQuality, 0, LINK_QUALITY_MAX_VALUE, 0, 100) : linkQuality;
}
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
uint16_t rxGetUplinkTxPwrMw(void)
{
    return uplinkTxPwrMw;
}
#endif

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeState.rxRefreshRate;
}

bool isRssiConfigured(void)
{
    return rssiSource != RSSI_SOURCE_NONE;
}

timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs)
{
    static timeUs_t previousFrameTimeUs = 0;
    static timeDelta_t frameTimeDeltaUs = 0;

    if (rxRuntimeState.rcFrameTimeUsFn) {
        const timeUs_t frameTimeUs = rxRuntimeState.rcFrameTimeUsFn();

        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);

        const timeDelta_t deltaUs = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if (deltaUs) {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }

    return frameTimeDeltaUs;
}

timeUs_t rxFrameTimeUs(void)
{
    return rxRuntimeState.lastRcFrameTimeUs;
}



// `crsf_protocol.h` here:

const BAUDRATE: u16 = 420000;

const SYNC_BYTE: u8 = 0xC8;

const FRAME_SIZE_MAX: usize = 64; // 62 bytes frame plus 2 bytes frame header(<length><type>)
const PAYLOAD_SIZE_MAX: usize = FRAME_SIZE_MAX - 6;

#[repr(u8)]
enum FrameType {
    Gps = 0x02,
    BatterySensor = 0x08,
    LinkStatistics = 0x14,
    RcChannelsPacked = 0x16,
    SubsetRcChannelsPacked = 0x17,
    LinkStatisticsRx = 0x1C,
    LinkStatisticsTx = 0x1D,
    ATTITUDE = 0x1E,
    FlightMode = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    DevicePing = 0x28,
    DeviceInfo = 0x29,
    ParameterSettingsEntry = 0x2B,
    ParameterRead = 0x2C,
    ParameterWrite = 0x2D,
    COMMAND = 0x32,
    // MSP commands
    MspReq = 0x7A,         // response request using msp sequence as command
    MspResp = 0x7B,        // reply with 58 byte chunked binary
    MspWrite = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    DisplayportCmd = 0x7D, // displayport control command
}

const COMMAND_SUBCMD_GENERAL: u8 = 0x0A; // general command

const COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL: u8 = 0x70; // proposed new CRSF port speed
const COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE: u8 = 0x71; // response to the proposed CRSF port speed

const DISPLAYPORT_SUBCMD_UPDATE: u8 = 0x01; // transmit displayport buffer to remote
const DISPLAYPORT_SUBCMD_CLEAR: u8 = 0x02; // clear client screen
const DISPLAYPORT_SUBCMD_OPEN: u8 = 0x03; // client request to open cms menu
const DISPLAYPORT_SUBCMD_CLOSE: u8 = 0x04; // client request to close cms menu
const DISPLAYPORT_SUBCMD_POLL: u8 = 0x05; // client request to poll/refresh cms menu

const DISPLAYPORT_OPEN_ROWS_OFFSET: u8 = 1;
const DISPLAYPORT_OPEN_COLS_OFFSET: u8 = 2;

const FRAME_GPS_PAYLOAD_SIZE: u8 = 15;
const FRAME_BATTERY_SENSOR_PAYLOAD_SIZE: u8 = 8;
const FRAME_LINK_STATISTICS_PAYLOAD_SIZE: u8 = 10;
const FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE: u8 = 6;
const FRAME_RC_CHANNELS_PAYLOAD_SIZE: u8 = 22; // 11 bits per channel * 16 channels = 22 bytes.
const FRAME_ATTITUDE_PAYLOAD_SIZE: u8 = 6;

const FRAME_LENGTH_ADDRESS: u8 = 1; // length of ADDRESS field
const FRAME_LENGTH_FRAMELENGTH: u8 = 1; // length of FRAMELENGTH field
const FRAME_LENGTH_TYPE: u8 = 1; // length of TYPE field
const FRAME_LENGTH_CRC: u8 = 1; // length of CRC field
const FRAME_LENGTH_TYPE_CRC: u8 = 2; // length of TYPE and CRC fields combined
const FRAME_LENGTH_EXT_TYPE_CRC: u8 = 4; // length of Extended Dest/Origin; TYPE and CRC fields combined
const FRAME_LENGTH_NON_PAYLOAD: u8 = 4; // combined length of all fields except payload

const FRAME_TX_MSP_FRAME_SIZE: u8 = 58;
const FRAME_RX_MSP_FRAME_SIZE: u8 = 8;
const FRAME_ORIGIN_DEST_SIZE: u8 = 2;

#[repr(u8)]
enum Address {
    Broadcast = 0x00,
    Usb = 0x10,
    TbsCorePnpPro = 0x80,
    Reserved1 = 0x8A,
    CurrentSensor = 0xC0,
    Gps = 0xC2,
    TbsBlackbox = 0xC4,
    FlightController = 0xC8,
    RESERVED2 = 0xCA,
    RaceTag = 0xCC,
    RadioTransmitter = 0xEA,
    Receiver = 0xEC,
    Transmitter = 0xEE,
}

// `crsf.h` here:'

const USE_CRSF_V3: bool = false; // todo: True eventually; newer version.

// todo: Check teh types of these

const MAX_CHANNEL: usize = 16;
const CRSFV3_MAX_CHANNEL: u8 = 24;

const SUBSET_RC_STARTING_CHANNEL_BITS: u8 = 5;
const SUBSET_RC_STARTING_CHANNEL_MASK: u8 = 0x1F;
const SUBSET_RC_RES_CONFIGURATION_BITS: u8 = 2;
const SUBSET_RC_RES_CONFIGURATION_MASK: u8 = 0x03;
const SUBSET_RC_RESERVED_CONFIGURATION_BITS: u8 = 1;

const RC_CHANNEL_SCALE_LEGACY: f32 = 0.62477120195241;
const SUBSET_RC_RES_CONF_10B: u8 = 0;
const SUBSET_RC_RES_BITS_10B: u8 = 10;
const SUBSET_RC_RES_MASK_10B: u16 = 0x03FF;
const SUBSET_RC_CHANNEL_SCALE_10B: f32 = 1.0;
const SUBSET_RC_RES_CONF_11B: u8 = 1;
const SUBSET_RC_RES_BITS_11B: u8 = 11;
const SUBSET_RC_RES_MASK_11B: u16 = 0x07FF;
const SUBSET_RC_CHANNEL_SCALE_11B: f32 = 0.5;
const SUBSET_RC_RES_CONF_12B: u8 = 2;
const SUBSET_RC_RES_BITS_12B: u8 = 12;
const SUBSET_RC_RES_MASK_12B: u16 = 0x0FFF;
const SUBSET_RC_CHANNEL_SCALE_12B: f32 = 0.25;
const SUBSET_RC_RES_CONF_13B: u8 = 3;
const SUBSET_RC_RES_BITS_13B: u8 = 13;
const SUBSET_RC_RES_MASK_13B: u16 = 0x1FFF;
const SUBSET_RC_CHANNEL_SCALE_13B: f32 = 0.125;

const RSSI_MIN: u8 = -130;
const RSSI_MAX: u8 = 0;
const SNR_MIN: u8 = -30;
const SNR_MAX: u8 = 20;

type TimeUs = u32; // todo: Is this what we want?

/* For documentation purposes
typedef enum {
    RF_MODE_4_FPS = 0,
    RF_MODE_50_FPS,
    RF_MODE_150_FPS,
} crsfRfMode_e;
*/

struct FrameDef {
    device_address: Address,
    frame_length: u8,
    type_: FrameType,
    payload: [u8; PAYLOAD_SIZE_MAX + 1], // +1 for CRC at end of payload
}

struct Frame {
    bytes: [u8; FRAME_SIZE_MAX],
    frame: FrameDef,
}

struct RxConfig {}
struct RxRuntimeStats {}

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

// `crsf.c` here:

// todo: Data types of these?
const TIME_NEEDED_PER_FRAME_US: u16 = 1750; // a maximally sized 64byte payload will take ~1550us, round up to 1750.
const TIME_BETWEEN_FRAMES_US: u16 = 6667; // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

const DIGITAL_CHANNEL_MIN: u16 = 172;
const DIGITAL_CHANNEL_MAX: u16 = 1811;

// const  PAYLOAD_OFFSET: u16 =  offsetof(crsfFrameDef, type);
const PAYLOAD_OFFSET: usize = 0; // todo?

const LINK_STATUS_UPDATE_TIMEOUT_US: u16 = 250000; // 250ms, 4 Hz mode 1 telemetry

const FRAME_ERROR_COUNT_THRESHOLD: u8 = 100;

static mut FRAME_DONE: bool = false;

static mut FRAME: Frame = Frame {
    bytes: [0; FRAME_SIZE_MAX],
    frame: FrameDef {
        device_address: Address::Broadcast,
        frame_length: 0,
        type_: FrameType::RcChannelsPacked,
        payload: [0; PAYLOAD_SIZE_MAX + 1], // +1 for CRC at end of payload
    },
};

static mut CHANNEL_DATA_FRAME: Frame = Frame {
    bytes: [0; FRAME_SIZE_MAX],
    frame: FrameDef {
        device_address: Address::Broadcast,
        frame_length: 0,
        type_: FrameType::RcChannelsPacked,
        payload: [0; PAYLOAD_SIZE_MAX + 1], // +1 for CRC at end of payload
    },
};

static mut CHANNEL_DATA: [u32; MAX_CHANNEL] = [0; MAX_CHANNEL];

static mut FRAME_START_AT_US: TimeUs = 0;
static mut TELEMETRY_BUF: [u8; FRAME_SIZE_MAX] = [0; FRAME_SIZE_MAX];
static mut TELEMETRY_BUF_LEN: usize = 0;
static mut CHANNEL_SCALE: f32 = RC_CHANNEL_SCALE_LEGACY;

const UPLINK_POWER_LEVEL_MW_ITEMS_COUNT: usize = 9;
// Uplink power levels by uplinkTXPower expressed in mW (250 mW is from ver >=4.00, 50 mW in a future version and for ExpressLRS)
/// If USE_RX_LINK_UPLINK_POWER
const UPLINK_TX_POWER_STATES_MW: [u16; UPLINK_POWER_LEVEL_MW_ITEMS_COUNT] =
    [0, 10, 25, 100, 500, 1000, 2000, 250, 50];

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

struct PayloadChannelsPacked {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    chan0: u8,
    chan1: u8,
    chan2: u8,
    chan3: u8,
    chan4: u8,
    chan5: u8,
    chan6: u8,
    chan7: u8,
    chan8: u8,
    chan9: u8,
    chan10: u8,
    chanu11: u8,
    chan12: u8,
    chan13: u8,
    chan14: u8,
    chan15: u8,
}

/*
* SUBSET RC FRAME 0x17
*
* The structure of 0x17 frame consists of 8-bit configuration data & variable length packed channel data.
*
* definition of the configuration byte
* bits 0-4: number of first channel packed
* bits 5-6: resolution configuration of the channel data (00 -> 10 bits, 01 -> 11 bits, 10 -> 12 bits, 11 -> 13 bits)
* bit 7:    reserved

* data structure of the channel data
*  - first channel packed with specified resolution
*  - second channel packed with specified resolution
*  - third channel packed with specified resolution
*                       ...
*  - last channel packed with specified resolution
*/

// #if defined(USE_CRSF_LINK_STATISTICS)
/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

/*
 * 0x1C Link statistics RX
 * Payload:
 *
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink RSSI ( % )
 * uint8_t Downlink Package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * uint8_t Uplink RF Power ( db )
 */

/*
 * 0x1D Link statistics TX
 * Payload:
 *
 * uint8_t Uplink RSSI ( dBm * -1 )
 * uint8_t Uplink RSSI ( % )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Downlink RF Power ( db )
 * uint8_t Uplink FPS ( FPS / 10 )
 */

struct PayloadLinkStatistics {
    uplink_rssi_1: u8,
    uplink_rssi_2: u8,
    uplink_link_quality: u8,
    uplink_snr: i8,
    active_antenna: u8,
    rf_mode: u8,
    uplink_tx_power: u8,
    downlink_rssi: u8,
    downlink_link_quality: u8,
    downlink_snr: i8,
}

// If USE_CRSF_V3
struct PayloadLinkStatisticsRx {
    downlink_rssi_1: u8,
    downlink_rssi_1_percentage: u8,
    downlink_link_quality: u8,
    downlink_snr: i8,
    uplink_power: u8,
} // this struct is currently not used

struct PayloadLinkStatisticsTx {
    uplink_rssi: u8,
    uplink_rssi_percentage: u8,
    uplink_link_quality: u8,
    uplink_snr: i8,
    downlink_power: u8,
    // currently not used
    uplink_fps: u8, // currently not used
}

static mut LAST_LINK_STATISTICS_FRAME_US: TimeUs = 0;

fn handle_link_statistics_frame(stats_ptr: &crsfLinkStatistics, current_time_us: TimeUs) {
    let stats = *stats_ptr;
    unsafe { LAST_LINK_STATISTICS_FRAME_US = current_time_us };
    let mut rssi_dbm = -1
        * (if stats.active_antenna {
            stats.uplink_RSSI_2
        } else {
            stats.uplink_RSSI_1
        });
    if rssiSource == RssiSource::RX_PROTOCOL_CRSF {
        if rxConfig().crsf_use_rx_snr {
            // -10dB of SNR mapped to 0 RSSI (fail safe is likely to happen at this measure)
            //   0dB of SNR mapped to 20 RSSI (default alarm)
            //  41dB of SNR mapped to 99 RSSI (SNR can climb to around 60, but showing that is not very meaningful)
            let rsnr_percent_scaled: u16 =
                constrain((stats.uplink_SNR + 10) * 20, 0, RSSI_MAX_VALUE);
            setRssi(rsnr_percent_scaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);

            if USE_RX_RSSI_DBM {
                rssi_dbm = stats.uplink_SNR;
            }
        } else {
            let rssi_percent_scaled = scaleRange(rssi_dbm, RSSI_MIN, 0, 0, RSSI_MAX_VALUE);
            setRssi(rssi_percent_scaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);
        }
    }
    if USE_RX_RSSI_DBM {
        setRssiDbm(rssi_dbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    }

    if USE_RX_LINK_QUALITY_INFO {
        if linkQualitySource == LinkQualitySource::RX_PROTOCOL_CRSF {
            setLinkQualityDirect(stats.uplink_Link_quality);
            rxSetRfMode(stats.rf_Mode);
        }
    }

    if USE_RX_LINK_UPLINK_POWER {
        let uplink_power_states_item_index: u8 =
            if stats.uplink_TX_Power < UPLINK_POWER_LEVEL_MW_ITEMS_COUNT {
                stats.uplink_TX_Power
            } else {
                0
            };
        rxSetUplinkTxPwrMw(UPLINK_TX_POWER_STATES_MW[uplink_power_states_item_index]);
    }

    // todo?
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI_1);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_RSSI_2);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.rf_Mode);
    //
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 0, stats.active_antenna);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 1, stats.uplink_SNR);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 2, stats.uplink_TX_Power);
    //
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 0, stats.downlink_RSSI);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 1, stats.downlink_Link_quality);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 2, stats.downlink_SNR);
}

/// For V3
fn handle_link_statistics_tx_frame(stats_ptr: &crsfLinkStatisticsTx, current_time_us: TimeUs) {
    let stats: LinkStatisticsTx = *stats_ptr;
    unsafe { LAST_LINK_STATISTICS_FRAME_US = current_time_us };
    if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF {
        let rssi_percent_scaled: u16 =
            scaleRange(stats.uplink_RSSI_percentage, 0, 100, 0, RSSI_MAX_VALUE);
        setRssi(rssi_percent_scaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    }
    if USE_RX_RSSI_DBM {
        let mut rssi_dbm: i16 = -1 * stats.uplink_RSSI;
        if rxConfig().crsf_use_rx_snr {
            rssi_dbm = stats.uplink_SNR;
        }
        setRssiDbm(rssi_dbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    }

    if USE_RX_LINK_QUALITY_INFO {
        if linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF {
            setLinkQualityDirect(stats.uplink_Link_quality);
        }
    }

    // todo?
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_SNR);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
    // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.uplink_RSSI_percentage);
}

/// If USE_CRSF_LINK_STATISTICS
fn check_rssi(current_time_us: u32) {
    if cmpTimeUs(current_time_us, unsafe { LAST_LINK_STATISTICS_FRAME_US })
        > LINK_STATUS_UPDATE_TIMEOUT_US
    {
        if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF {
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL_CRSF);

            if USE_RX_RSSI_DBM {
                if rxConfig().crsf_use_rx_snr {
                    setRssiDbmDirect(SNR_MIN, RSSI_SOURCE_RX_PROTOCOL_CRSF);
                } else {
                    setRssiDbmDirect(RSSI_MIN, RSSI_SOURCE_RX_PROTOCOL_CRSF);
                }
            }
        }
        if USE_RX_LINK_QUALITY_INFO {
            if linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF {
                setLinkQualityDirect(0);
            }
        }
    }
}

fn frame_crc() -> u8 {
    // CRC includes type and payload
    let mut crc: u8 = calc_crc(0, unsafe { FRAME.frame.type_ as u8 }, 0xd5);

    unsafe {
        for ii in 0..CRSF_FRAME.frame.frame_length - FRAME_LENGTH_TYPE_CRC {
            crc = calc_crc(crc, FRAME.frame.payload[ii], 0xd5);
        }
    }
    crc
}

fn frame_cmd_crc() -> u8 {
    // CRC includes type and payload
    let mut crc: u8 = calc_crc(0, unsafe { FRAME.frame.type_ }, 0xba);

    unsafe {
        for ii in 0..CRSF_FRAME.frame.frame_length - FRAME_LENGTH_TYPE_CRC - 1 {
            crc = calc_crc(crc, FRAME.frame.payload[ii], 0xba);
        }
    }
    crc
}

// Receive ISR callback, called back from serial port
fn data_receive(c: u16) {
    let rx_runtime_state: rxRuntimeState = data as &[rxRuntimeState];

    // let mut crsfFramePosition: u8 = 0; // todo: As u8 in original, but usize makes more sens?
    let mut frame_position = 0;

    let mut frame_error_cnt: u8 = 0; // For V3

    let current_time_us: TimeUs = 0; // todo: This calls a timer or RTC etc.

    if cmpTimeUs(current_time_us, unsafe { FRAME_START_AT_US }) > TIME_NEEDED_PER_FRAME_US {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        frame_position = 0;
    }

    if frame_position == 0 {
        unsafe { FRAME_START_AT_US = current_time_us };
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    // sometimes we can receive some garbage data. So, we need to check max size for preventing buffer overrun.
    let full_frame_length: usize = if frame_position < 3 {
        5
    } else {
        min(
            unsafe { FRAME.frame.frame_length as usize }
                + FRAME_LENGTH_ADDRESS
                + FRAME_LENGTH_FRAMELENGTH,
            FRAME_SIZE_MAX,
        )
    };

    unsafe {
        if frame_position < full_frame_length {
            frame_position += 1;
            FRAME.bytes[crsf_frame_position] = c as u8;

            if frame_position >= full_frame_length {
                frame_position = 0;
                let crc = frame_crc();
                if crc == FRAME.bytes[full_frame_length - 1] {
                    if USE_CRSF_V3 {
                        frame_error_cnt = 0;
                    }

                    match FRAME.frame.type_ {
                        FrameType::RcChannelsPacked => (),
                        FrameType::SubsetRcChannelsPacked => {
                            if FRAME.frame.device_address == Address::FlightController {
                                rx_runtime_state.lastRcFrameTimeUs = current_time_us;
                                FRAME_DONE = true;
                                memcpy(&CHANNEL_DATA_FRAME, &CRSF_FRAME, sizeof(CRSF_FRAME));
                            }
                        }

                        // if USE_TELEMETRY_CRSF && USE_MSP_OVER_TELEMETRY {
                        FrameType::MspReq => (),
                        FrameType::MspWrite => {
                            FRAME.frame.payload as u8 + FRAME_ORIGIN_DEST_SIZE;
                            if bufferCrsfMspFrame(frameStart, FRAME.frame.frame_length - 4) {
                                crsfScheduleMspResponse(CRSF_FRAME.frame.payload[1]);
                            }
                            // }
                        }
                        // if USE_CRSF_CMS_TELEMETRY {
                        CCrsfFrameType::DEVICE_PING => {
                            crsfScheduleDeviceInfoResponse();
                        }
                        FrameType::DisplayportCmd => {
                            let frame_start: u8 =
                                &CRSF_FRAME.frame.payload as u8 + FRAME_ORIGIN_DEST_SIZE;
                            crsfProcessDisplayPortCmd(frame_start);
                        }
                        // }
                        // #if defined(USE_CRSF_LINK_STATISTICS)
                        FrameType::LinkStatistics => {
                            // if to FC and 10 bytes + FRAME_ORIGIN_DEST_SIZE
                            if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF
                                && CRSF_FRAME.frame.device_address == ADDRESS_FLIGHT_CONTROLLER
                                && CRSF_FRAME.frame.frame_length
                                    == FRAME_ORIGIN_DEST_SIZE + FRAME_LINK_STATISTICS_PAYLOAD_SIZE
                            {
                                let stats_frame = CRSF_FRAME.frame.payload as crsfLinkStatistics;
                                handle_link_statistics_frame(stats_frame, current_time_us);
                            }
                        }

                        // if USE_CRSF_V3 {
                        FrameType::LinkStatisticsRx => (),
                        FrameType::LinkStatisticsTx => {
                            if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF
                                && CRSF_FRAME.frame.device_address == ADDRESS_FLIGHT_CONTROLLER
                                && CRSF_FRAME.frame.frame_length
                                    == FRAME_ORIGIN_DEST_SIZE
                                        + FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE
                            {
                                let statsFrame = CRSF_FRAME.frame.payload as crsfLinkStatisticsTx;
                                handle_link_statistics_tx_frame(statsFrame, current_time_us);
                            }
                        }
                        // if USE_CRSF_V3 {
                        FrameType::COMMAND => {
                            if CRSF_FRAME.bytes[full_frame_length - 2] == frame_cmd_crc()
                                && CRSF_FRAME.bytes[3] == ADDRESS_FLIGHT_CONTROLLER
                            {
                                crsfProcessCommand(
                                    CRSF_FRAME.frame.payload + FRAME_ORIGIN_DEST_SIZE,
                                );
                            }
                        }
                        // }
                        _ => (),
                    }
                } else {
                    if USE_CRSF_V3 {
                        if frame_error_cnt < FRAME_ERROR_COUNT_THRESHOLD {
                            frame_error_cnt += 1;
                        }
                    }
                }
            } else {
                if USE_CRSF_V3 {
                    if frame_error_cnt < FRAME_ERROR_COUNT_THRESHOLD {
                        frame_error_cnt += 1;
                    }
                }
            }
            if USE_CRSF_V3 {
                if frame_error_cnt >= FRAME_ERROR_COUNT_THRESHOLD {
                    // fall back to default speed if speed mismatch detected
                    setCrsfDefaultSpeed();
                    frame_error_cnt = 0;
                }
            }
        }
    }
}

fn frame_status() -> RxFrameState {
    if USE_CRSF_LINK_STATISTICS {
        check_rssi(micros());
    }

    unsafe {
        if FRAME_DONE {
            FRAME_DONE = false;

            // unpack the RC channels
            if CHANNEL_DATA_FRAME.frame.type_ == FrameType::RcChannelsPacked {
                // use ordinary RC frame structure (0x16)

                let rc_channels = CHANNEL_DATA_FRAME.frame.payload as PayloadChannelsPacked; // todo: Not right
                CHANNEL_SCALE = RC_CHANNEL_SCALE_LEGACY;

                CHANNEL_DATA[0] = rc_channels.chan0 as u32;
                CHANNEL_DATA[1] = rc_channels.chan1 as u32;
                CHANNEL_DATA[2] = rc_channels.chan2 as u32;
                CHANNEL_DATA[3] = rc_channels.chan3 as u32;
                CHANNEL_DATA[4] = rc_channels.chan4 as u32;
                CHANNEL_DATA[5] = rc_channels.chan5 as u32;
                CHANNEL_DATA[6] = rc_channels.chan6 as u32;
                CHANNEL_DATA[7] = rc_channels.chan7 as u32;
                CHANNEL_DATA[8] = rc_channels.chan8 as u32;
                CHANNEL_DATA[9] = rc_channels.chan9 as u32;
                CHANNEL_DATA[10] = rc_channels.chan10 as u32;
                CHANNEL_DATA[11] = rc_channels.chan11 as u32;
                CHANNEL_DATA[12] = rc_channels.chan12 as u32;
                CHANNEL_DATA[13] = rc_channels.chan13 as u32;
                CHANNEL_DATA[14] = rc_channels.chan14 as u32;
                CHANNEL_DATA[15] = rc_channels.chan15 as u32;
            } else {
                // use subset RC frame structure (0x17)
                let mut read_byte_index: u8 = 0;
                letpayload: [u8; 59] = unsafe { CHANNEL_DATA_FRAME.frame.payload };

                // get the configuration byte
                read_byte_index += 1;
                let mut config_byte: u8 = payload[read_byte_index];

                // get the channel number of start channel
                let start_channel: u8 = config_byte & SUBSET_RC_STARTING_CHANNEL_MASK;
                config_byte >>= SUBSET_RC_STARTING_CHANNEL_BITS;

                // get the channel resolution settings
                let mut channel_bits: u8 = 0;
                let mut channel_mask: u16 = 0;
                let mut channel_res: u8 = config_byte & SUBSET_RC_RES_CONFIGURATION_MASK;
                config_byte >>= SUBSET_RC_RES_CONFIGURATION_BITS;

                unsafe {
                    match channel_res {
                        SUBSET_RC_RES_CONF_10B => {
                            channel_bits = SUBSET_RC_RES_BITS_10B;
                            channel_mask = SUBSET_RC_RES_MASK_10B;
                            CHANNEL_SCALE = SUBSET_RC_CHANNEL_SCALE_10B;
                        }
                        SUBSET_RC_RES_CONF_11B => {
                            channel_bits = SUBSET_RC_RES_BITS_11B;
                            channel_mask = SUBSET_RC_RES_MASK_11B;
                            CHANNEL_SCALE = SUBSET_RC_CHANNEL_SCALE_11B;
                        }
                        SUBSET_RC_RES_CONF_12B => {
                            channel_bits = SUBSET_RC_RES_BITS_12B;
                            channel_mask = SUBSET_RC_RES_MASK_12B;
                            CHANNEL_SCALE = SUBSET_RC_CHANNEL_SCALE_12B;
                        }
                        SUBSET_RC_RES_CONF_13B => {
                            channel_bits = SUBSET_RC_RES_BITS_13B;
                            channel_mask = SUBSET_RC_RES_MASK_13B;
                            CHANNEL_SCALE = SUBSET_RC_CHANNEL_SCALE_13B;
                        }
                        _ => (),
                    }
                }

                // do nothing for the reserved configuration bit
                config_byte >>= SUBSET_RC_RESERVED_CONFIGURATION_BITS;

                // calculate the number of channels packed
                let num_of_channels: u8 =
                    ((CHANNEL_DATA_FRAME.frame.frame_length - FRAME_LENGTH_TYPE_CRC - 1) * 8)
                        / channel_bits;

                // unpack the channel data
                let mut bits_merged: u8 = 0;
                let mut read_value: u32 = 0;
                for n in 0..num_of_channels {
                    while bits_merged < channel_bits {
                        let read_byte: u8 = payload[read_byte_index += 1];
                        read_value |= (read_byte as u32) << bits_merged;
                        bits_merged += 8;
                    }
                    CHANNEL_DATA[start_channel + n] = read_value & channel_mask;
                    read_value >>= channel_bits;
                    bits_merged -= channel_bits;
                }
            }
            return RxFrameState::Complete;
        }
        return RxFrameState::Pending;
    }
}

fn read_raw_rc(chan: u8) -> f32 {
    unsafe {
        if CHANNEL_SCALE == RC_CHANNEL_SCALE_LEGACY {
            /* conversion from RC value to PWM
             * for 0x16 RC frame
             *       RC     PWM
             * min  172 ->  988us
             * mid  992 -> 1500us
             * max 1811 -> 2012us
             * scale factor = (2012-988) / (1811-172) = 0.62477120195241
             * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
             */
            return (CHANNEL_SCALE * CHANNEL_DATA[chan] as f32) + 881.;
        } else {
            /* conversion from RC value to PWM
             * for 0x17 Subset RC frame
             */
            return (CHANNEL_SCALE * CHANNEL_DATA[chan] as f32) + 988.;
        }
    }
}

fn rx_write_telemetry_data(void: &data, len: &mut usize) {
    unsafe {
        *len = min(*len, sizeof(TELEMETRY_BUF) as usize);
        memcpy(TELEMETRY_BUF, data, len);
        TELEMETRY_BUF_LEN = *len;
    }
}

fn rx_send_telemetry_data() {
    // if there is telemetry data to write

    unsafe {
        if TELEMETRY_BUF_LEN > 0 {
            serialWriteBuf(serialPort, TELEMETRY_BUF, TELEMETRY_BUF_LEN);
            TELEMETRY_BUF_LEN = 0; // reset telemetry buffer
        }
    }
}

fn rx_is_telemetry_buf_empty() -> bool {
    return unsafe { TELEMETRY_BUF_LEN } == 0;
}

fn rx_init(rx_config: &RxConfig, rx_runtime_state: &mut rxRuntimeState) -> bool {
    for i in 0..CRSF_MAX_CHANNEL {
        unsafe {
            CHANNEL_DATA[i] = (16 * rx_config.midrc) / 10 - 1408;
        }
    }

    rx_runtime_state.channelCount = MAX_CHANNEL;
    rx_runtime_state.rxRefreshRate = TIME_BETWEEN_FRAMES_US; // TODO this needs checking

    rx_runtime_state.rcReadRawFn = read_raw_rc;
    rx_runtime_state.rcFrameStatusFn = frame_status;
    rx_runtime_state.rcFrameTimeUsFn = rxFrameTimeUs;

    let port_config = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if !port_config {
        return false;
    }

    unsafe {
        serialPort = openSerialPort(
            port_config.identifier,
            FUNCTION_RX_SERIAL,
            data_receive,
            rx_runtime_state,
            BAUDRATE,
            PORT_MODE,
            PORT_OPTIONS
                | (if rx_config.serialrx_inverted {
                    SERIAL_INVERTED
                } else {
                    0
                }),
        );
    }

    if rssiSource == RSSI_SOURCE_NONE {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL_CRSF;
    }
    if USE_RX_LINK_QUALITY_INFO {
        if linkQualitySource == LQ_SOURCE_NONE {
            linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
        }
    }

    return serialPort != NULL;
}
