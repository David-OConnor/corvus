//! Express LRS. Adapted from Betaflight here:
//! https://github.com/betaflight/betaflight/tree/master/src/main/rx.
//! See `expresslrs_` files

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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */



// From `expresslrs_common.h`:

const ELRS_CRC_LEN: u8 = 256;
const ELRS_CRC14_POLY: u8 = 0x2E57;

const ELRS_NR_SEQUENCE_ENTRIES: u8 = 256;

const ELRS_RX_TX_BUFF_SIZE: u8 = 8;

const ELRS_TELEMETRY_TYPE_LINK: u8 = 0x01;
const ELRS_TELEMETRY_TYPE_DATA: u8 = 0x02;
const ELRS_MSP_BIND: u8 = 0x09;
const ELRS_MSP_MODEL_ID: u8 = 0x0A;
const ELRS_MSP_SET_RX_CONFIG: u8 = 45;

const ELRS_MODELMATCH_MASK: u8 = 0x3F;

// todo
// const FREQ_HZ_TO_REG_VAL_900(freq) ((u32)(freq / SX127x_FREQ_STEP))
// const FREQ_HZ_TO_REG_VAL_24(freq) ((u32)(freq / SX1280_FREQ_STEP))

const ELRS_RATE_MAX: u8 = 4;
const ELRS_BINDING_RATE_24: u8 = 3;
const ELRS_BINDING_RATE_900: u8 = 2;

const ELRS_MAX_CHANNELS: u8 = 16;
const ELRS_RSSI_CHANNEL: u8 = 15;
const ELRS_LQ_CHANNEL: u8 = 14;

const ELRS_CONFIG_CHECK_MS: u8 = 200;
const ELRS_LINK_STATS_CHECK_MS: u8 = 100;
const ELRS_CONSIDER_CONNECTION_GOOD_MS: u8 = 1000;

const ELRS_MODE_CYCLE_MULTIPLIER_SLOW: u8 = 10;

enum FreqDomain {
    ISM2400,
}

#[repr(u8)]
enum SwitchMode {
    SM_HYBRID = 0,
    SM_HYBRID_WIDE = 1
}

#[repr(u8)]
enum TlmRatio {
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7,
}

#[repr(u8)]
enum RfRate {
    RATE_500HZ = 0,
    RATE_250HZ = 1,
    RATE_200HZ = 2,
    RATE_150HZ = 3,
    RATE_100HZ = 4,
    RATE_50HZ = 5,
    RATE_25HZ = 6,
    RATE_4HZ = 7,
}

struct ModSettings {
    index: u8,
    elrsRfRate_e enumRate: RfRate,            // Max value of 16 since only 4 bits have been assigned in the sync package.
    bw: u8,
    sf: u8,
    cr: u8,
    interval: u32,                  // interval in us seconds that corresponds to that frequency
    tlmInterval: TlmRatio,       // every X packets is a response TLM packet, should be a power of 2
    fhssHopInterval: u8,            // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    preambleLen: u8,
}

struct RfPerfParams {
    index: i8,
    enumRate: RfRate,        // Max value of 16 since only 4 bits have been assigned in the sync package.
    i32 : i32,            // expected RF sensitivity based on
    toa: u32,                   // time on air in microseconds
    disconnectTimeoutMs: u32,   // Time without a packet before receiver goes to disconnected (ms)
    rxLockTimeoutMs: u32,       // Max time to go from tentative -> connected state on receiver (ms)
    syncPktIntervalDisconnected: u32, // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
    syncPktIntervalConnected: u32,    // how often to send the SYNC_PACKET packet (ms) when there we have a connection
}

type bool (*elrsRxInitFnPtr)(IO_t resetPin, IO_t busyPin);
type void (*elrsRxConfigFnPtr)(const u8 bw, const u8 sf, const u8 cr, const u32 freq, const u8 preambleLen, const bool iqInverted);
type void (*elrsRxStartReceivingFnPtr)(void);
type u8 (*elrsRxISRFnPtr)(timeUs_t *timeStamp);
type void (*elrsRxTransmitDataFnPtr)(const u8 *data, const u8 length);
type void (*elrsRxReceiveDataFnPtr)(u8 *data, const u8 length);
type void (*elrsRxGetRFlinkInfoFnPtr)(i8 *rssi, i8 *snr);
type void (*elrsRxSetFrequencyFnPtr)(const u32 freq);
type void (*elrsRxHandleFreqCorrectionFnPtr)(i32 offset, const u32 freq);

extern elrsModSettings_t airRateConfig[][ELRS_RATE_MAX];
extern elrsRfPerfParams_t rfPerfConfig[][ELRS_RATE_MAX];


// from `expresslrs_impl.h`:

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
#[repr(u8)]
enum PacketType {
    ELRS_RC_DATA_PACKET = 0x00,
    ELRS_MSP_DATA_PACKET = 0x01,
    ELRS_SYNC_PACKET = 0x02,
    ELRS_TLM_PACKET = 0x03,
}

#[repr(u8)]
enum DioReason {
    ELRS_DIO_UNKNOWN = 0,
    ELRS_DIO_RX_DONE = 1,
    ELRS_DIO_TX_DONE = 2,
    ELRS_DIO_RX_AND_TX_DONE = 3,
}

enum ConnectionState {
    ELRS_CONNECTED,
    ELRS_TENTATIVE,
    ELRS_DISCONNECTED,
    ELRS_DISCONNECT_PENDING // used on modelmatch change to drop the connection
}

#[repr(u8)]
enum TimerState {
    ELRS_TIM_DISCONNECTED = 0,
    ELRS_TIM_TENTATIVE = 1,
    ELRS_TIM_LOCKED = 2
}

struct Receiver {
    resetPin: Io,
    busyPin: Io,

    freqOffset: i32,
    currentFreq: u32,

    nonceRX: u8, // nonce that we THINK we are up to.

    elrsModSettings_t *modParams,
    elrsRfPerfParams_t *rfPerfParams,

    const u8 *UID,

    rssi: i8,
    snr: i8,
    rssiFiltered: i8,

    uplinkLQ: u8,

    alreadyFHSS: bool,
    alreadyTLMresp: bool,
    lockRFmode: bool,
    started: bool,

    timerState_e timerState,
    connectionState_e connectionState,

    u8 rfModeCycleMultiplier,
    cycleIntervalMs: u16,
    rfModeCycledAtMs: u32,
    u8 rateIndex,
    u8 nextRateIndex,

    gotConnectionMs: u32,
    lastSyncPacketMs: u32,
    lastValidPacketMs: u32,

    configCheckedAtMs,
    bool configChanged,

    bool inBindingMode,
    volatile bool fhssRequired,

    statsUpdatedAtMs: u32,

    elrsRxInitFnPtr init,
    elrsRxConfigFnPtr config,
    elrsRxStartReceivingFnPtr startReceiving,
    elrsRxISRFnPtr rxISR,
    elrsRxTransmitDataFnPtr transmitData,
    elrsRxReceiveDataFnPtr receiveData,
    elrsRxGetRFlinkInfoFnPtr getRFlinkInfo,
    elrsRxSetFrequencyFnPtr setFrequency,
    elrsRxHandleFreqCorrectionFnPtr handleFreqCorrection,

    timerOvrHandlerRec_t timerUpdateCb,
}




// From `expresslrs_telemetry.h`:

const ELRS_TELEMETRY_SHIFT: u8 =  2;
const ELRS_TELEMETRY_BYTES_PER_CALL: u8 =  5;
const ELRS_TELEMETRY_MAX_PACKAGES: u8 = (255 >> ELRS_TELEMETRY_SHIFT);
const ELRS_TELEMETRY_MAX_MISSED_PACKETS: u8 =  20;

const ELRS_MSP_BYTES_PER_CALL: u8 = 5;
const ELRS_MSP_BUFFER_SIZE: u8 =  65;
const ELRS_MSP_MAX_PACKAGES: u8 =  ((ELRS_MSP_BUFFER_SIZE / ELRS_MSP_BYTES_PER_CALL) + 1);
const ELRS_MSP_PACKET_OFFSET: u8 =  5;
const ELRS_MSP_COMMAND_INDEX: u8 =  7;

#[repr(u8)]
enum StubbornSensorState {
    ELRS_SENDER_IDLE = 0,
    ELRS_SENDING = 1,
    ELRS_WAIT_UNTIL_NEXT_CONFIRM = 2,
    ELRS_RESYNC = 3,
    ELRS_RESYNC_THEN_SEND = 4, // perform a RESYNC then go to SENDING
}

// From `expresslrs.c`:

