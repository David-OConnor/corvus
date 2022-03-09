//! Express LRS. Adapted from Betaflight here:
//! https://github.com/betaflight/betaflight/tree/master/src/main/rx.
//! See `expresslrs_` files

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(unused)]

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

use stm32_hal2::gpio::Pin;
use stm32_hal2::timer::Timer;

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

const ELRS_CONFIG_CHECK_MS: u32 = 200;
const ELRS_LINK_STATS_CHECK_MS: u32 = 100;
const ELRS_CONSIDER_CONNECTION_GOOD_MS: u32 = 1000;

const ELRS_MODE_CYCLE_MULTIPLIER_SLOW: u8 = 10;


static mut nextTelemetryType: u8 = ELRS_TELEMETRY_TYPE_LINK;

static mut telemetryBurstCount: u8 = 1;
static mut telemetryBurstMax: u8 = 1;
static mut telemBurstValid: bool = false;

// Maximum ms between LINK_STATISTICS packets for determining burst max
const TELEM_MIN_LINK_INTERVAL: u32 = 512;

// #ifdef USE_MSP_OVER_TELEMETRY
static mut mspBuffer: [u8; ELRS_MSP_BUFFER_SIZE] = [0; ELRS_MSP_BUFFER_SIZE];
// #endif

//
// Stick unpacking
//


#[derive(PartialEq, Clone, Copy)]
enum FreqDomain {
    ISM2400,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
enum SwitchMode {
    HYBRID = 0,
    HYBRID_WIDE = 1
}

#[derive(Clone, Copy)]
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
            TlmRatio::NO_TLM=> 1,
            TlmRatio::_1_2=>  2,
            TlmRatio::_1_4=> 4,
            TlmRatio::_1_8=> 8,
            TlmRatio::_1_16=>   16,
            TlmRatio::_1_32=>32,
            TlmRatio::_1_64=> 64,
            TlmRatio::_1_128=>128,
        }
    }
}

#[derive(Clone, Copy)]
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

impl RfRate {
    pub fn value(&self) -> u8 {
        match self {
            Self::RATE_500HZ => 500,
            Self::RATE_250HZ => 250,
            Self::RATE_200HZ => 200,
            Self::RATE_150HZ => 150,
            Self::RATE_100HZ => 100,
            Self::RATE_50HZ => 50,
            Self::RATE_25HZ => 25,
            Self::RATE_4HZ => 4,
            _ => 1
        }
    }
}

struct ModSettings {
    index: u8,
    enumRate: RfRate,            // Max value of 16 since only 4 bits have been assigned in the sync package.
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
    sensitivity : i32,            // expected RF sensitivity based on
    toa: u32,                   // time on air in microseconds
    disconnectTimeoutMs: u32,   // Time without a packet before receiver goes to disconnected (ms)
    rxLockTimeoutMs: u32,       // Max time to go from tentative -> connected state on receiver (ms)
    syncPktIntervalDisconnected: u32, // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
    syncPktIntervalConnected: u32,    // how often to send the SYNC_PACKET packet (ms) when there we have a connection
}

// type bool (*elrsRxInitFnPtr)(IO_t resetPin, IO_t busyPin);
// type void (*elrsRxConfigFnPtr)(const u8 bw, const u8 sf, const u8 cr, const u32 freq, const u8 preambleLen, const bool iqInverted);
// type void (*elrsRxStartReceivingFnPtr)(void);
// type u8 (*elrsRxISRFnPtr)(timeUs_t *timeStamp);
// type void (*elrsRxTransmitDataFnPtr)(const u8 *data, const u8 length);
// type void (*elrsRxReceiveDataFnPtr)(u8 *data, const u8 length);
// type void (*elrsRxGetRFlinkInfoFnPtr)(i8 *rssi, i8 *snr);
// type void (*elrsRxSetFrequencyFnPtr)(const u32 freq);
// type void (*elrsRxHandleFreqCorrectionFnPtr)(i32 offset, const u32 freq);

// extern elrsModSettings_t airRateConfig[][ELRS_RATE_MAX];
// extern elrsRfPerfParams_t rfPerfConfig[][ELRS_RATE_MAX];


// from `expresslrs_impl.h`:

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
#[repr(u8)]
enum PacketType {
    RC_DATA_PACKET = 0x00,
    MSP_DATA_PACKET = 0x01,
    SYNC_PACKET = 0x02,
    TLM_PACKET = 0x03,
}

impl From<u8> for PacketType {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::RC_DATA_PACKET,
            1 => Self::MSP_DATA_PACKET,
            2 => Self::RSYNC_PACKET,
            3 => Self::TLM_PACKET,
            _ => panic!(),
        }
    }
}

#[repr(u8)]
enum DioReason {
    UNKNOWN = 0,
    RX_DONE = 1,
    TX_DONE = 2,
    RX_AND_TX_DONE = 3,
}

#[derive(Copy, Clone, PartialEq)]
enum ConnectionState {
    CONNECTED,
    TENTATIVE,
    DISCONNECTED,
    DISCONNECT_PENDING // used on modelmatch change to drop the connection
}

#[repr(u8)]
enum TimerState {
    DISCONNECTED = 0,
    TENTATIVE = 1,
    LOCKED = 2
}

/// Represents the ELRS Receiver. In the translation to Rust, we've made most (previously free-standing)
/// C functions methods, since they make heavy use of modifying a Reciever.
struct Receiver {
    resetPin: Pin,
    busyPin: Pin,

    freqOffset: i32,
    currentFreq: u32,

    nonceRX: u8, // nonce that we THINK we are up to.

    modParams: ModSettings,
    rfPerfParams: RfPerfParams,

    UID: [u8; 6], // todo: 6 is a guess/placeholder

    rssi: i8,
    snr: i8,
    rssiFiltered: i8,

    uplinkLQ: u8,

    alreadyFHSS: bool,
    alreadyTLMresp: bool,
    lockRFmode: bool,
    started: bool,

    timerState: TimerState,
    connectionState: ConnectionState,

    rfModeCycleMultiplier: u8,
    cycleIntervalMs: u16,
    rfModeCycledAtMs: u32,
    rateIndex: u8,
    nextRateIndex: u8,

    gotConnectionMs: u32,
    lastSyncPacketMs: u32,
    lastValidPacketMs: u32,

    configCheckedAtMs: u32,
    configChanged: bool,

    inBindingMode: bool,
    fhssRequired: bool,

    statsUpdatedAtMs: u32,

    // init: RxInitFnPtr,
    // config: RxConfigFnPtr,
    // startReceiving: RxStartReceivingFnPtr,
    // rxISR: RxISRFnPtr,
    // transmitData: RxTransmitterDataFnPtr,
    // receiveData: RxRecieveDataFnPtr,
    // getRFlinkInfo: RxGetRFlinkInfoFnPtr,
    // setFrequency: RxSetFrequencyFnPtr,
    // handleFreqCorrection: RxHandleFreqCorrectionFnPtr,

    timerUpdateCb: timerOverHandlerRec,
}



impl Receiver {
    /// Set up the radio
    pub fn new(resetPin: Pin, busyPin: Pin) -> Self {
        let timeStampMs: u32 = millis();

        let mut result = Self {
            resetPin,
            busyPin,

            freqOffset: 0,
            currentFreq: 0,
            nonceRX: 0,

            modParams: ModSettings {

            },

            rssi: 0,
            rssiFiltered: 0,
            snr: 0,
            uplinkLQ: 0,

            started: false,
            alreadyFHSS: false,
            alreadyTLMresp: false,
            lockRFmode: false,
            timerState: TimerState::DISCONNECTED,
            connectionState: ConnectionState::DISCONNECTED,

            rfModeCycleMultiplier: 0,
            cycleIntervalMs: 0,
            rfModeCycledAtMs: 0,
            rateIndex: 0,
            nextRateIndex: 0,

            gotConnectionMs: 0,
            lastSyncPacketMs: 0,
            lastValidPacketMs: 0,

            configCheckedAtMs: 0,
            configChanged: false,

            inBindingMode: false,
            fhssRequired: false,

            statsUpdatedAtMs: 02,

        };

        FHSSrandomiseFHSSsequence(result.UID, rxExpressLrsSpiConfig().domain);
        lqReset();


        unsafe {
            result.rateIndex = if result.inBindingMode {
                bindingRateIndex
            } else { rxExpressLrsSpiConfig().rateIndex };
        }
        setRFLinkRate(result.rateIndex);

        result
    }

    fn setRFLinkRate(&mut self, index: u8)
    {
// #if defined(USE_RX_SX1280) && defined(USE_RX_SX127X)
        self.modParams = (rxExpressLrsSpiConfig().domain == FreqDomain::ISM2400)? & airRateConfig[1][index]: &airRateConfig[0][index];
        self.rfPerfParams = (rxExpressLrsSpiConfig().domain == FreqDomain::ISM2400)? & rfPerfConfig[1][index]: &rfPerfConfig[0][index];
// #else
//     self.modParams = &airRateConfig[0][index];
//     self.rfPerfParams = &rfPerfConfig[0][index];
// #endif
        self.currentFreq = getInitialFreq(self.freqOffset);
        // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
        self.cycleIntervalMs = (11 * getFHSSNumEntries() * self.modParams.fhssHopInterval * self.modParams.interval) / (10 * 1000);

        self.config(self.modParams.bw, self.modParams.sf, self.modParams.cr, self.currentFreq, self.modParams.preambleLen, self.UID[5] & 0x01);

        expressLrsUpdateTimerInterval(self.modParams.interval);

        rssiFilterReset();
        self.nextRateIndex = index; // presumably we just handled this
        unsafe { telemBurstValid = false };

        if USE_RX_LINK_QUALITY_INFO {
            rxSetRfMode(RfRate::RATE_4HZ as u8 - self.modParams.enumRate as u8);
        }
    }

    fn handleFHSS(&mut self) -> bool
    {
        let modresultFHSS: u8 = (self.nonceRX + 1) % self.modParams.fhssHopInterval;

        if (self.modParams.fhssHopInterval == 0) || self.alreadyFHSS == true || self.inBindingMode || (modresultFHSS != 0) || (self.connectionState == ELRS_DISCONNECTED) {
            return false;
        }

        self.alreadyFHSS = true;
        self.currentFreq = FHSSgetNextFreq(self.freqOffset);
        self.setFrequency(self.currentFreq);

        let modresultTLM: u8 = (self.nonceRX + 1) % self.modParams.tlmInterval.value();

        if modresultTLM != 0 || self.modParams.tlmInterval == TlmRatio::NO_TLM { // if we are about to send a tlm response don't bother going back to rx
            startReceiving();
        }
        return true;
    }

    fn shouldSendTelemetryResponse(&mut self) -> bool
    {
        let modresult: u8 = (self.nonceRX + 1) % self.modParams.tlmInterval.value();
        if (self.connectionState == ConnectionState::DISCONNECTED) || (self.modParams.tlmInterval == TlmRatio::TLM_RATIO_NO_TLM) ||
            (self.alreadyTLMresp == true) || (modresult != 0) {
            return false; // don't bother sending tlm if disconnected or TLM is off
        } else {
            return true;
        }
    }

    fn handleSendTelemetryResponse(&mut self)
    {
        let mut packet = [8_u8];

        let mut data = 0;
        let mut maxLength = 0_u8;
        let mut packageIndex = 0_u8;

        self.alreadyTLMresp = true;
        packet[0] = ELRS_TLM_PACKET;

        unsafe {
            if nextTelemetryType == ELRS_TELEMETRY_TYPE_LINK || !isTelemetrySenderActive() {
                packet[1] = ELRS_TELEMETRY_TYPE_LINK;
                packet[2] = if self.rssiFiltered > 0 {
                    0
                } else { -self.rssiFiltered }; //diversity not supported
                packet[3] = connectionHasModelMatch << 7;
                packet[4] = self.snr as u8;
                packet[5] = self.uplinkLQ;
                // #ifdef USE_MSP_OVER_TELEMETRY
                packet[6] = if getCurrentMspConfirm() {
                    1
                } else { 0 };
                // #else
                packet[6] = 0;
                // #endif
                nextTelemetryType = ELRS_TELEMETRY_TYPE_DATA;
                // Start the count at 1 because the next will be DATA and doing +1 before checking
                // against Max below is for some reason 10 bytes more code
                telemetryBurstCount = 1;
            } else {
                if telemetryBurstCount < telemetryBurstMax {
                    telemetryBurstCoun += 1;
                } else {
                    nextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
                }

                getCurrentTelemetryPayload(&packageIndex, &maxLength, &data);
                packet[1] = (packageIndex << ELRS_TELEMETRY_SHIFT) + ELRS_TELEMETRY_TYPE_DATA;
                packet[2] = if maxLength > 0 { data } else { 0 };
                packet[3] = if maxLength >= 1 { (data + 1) } else { 0 };
                packet[4] = if maxLength >= 2 { (data + 2) } else { 0 };
                packet[5] = if maxLength >= 3 { (data + 3) } else { 0 };
                packet[6] = if maxLength >= 4 { (data + 4) } else { 0 };
            }
        }

        let mut crc: u16 = calcCrc14(packet, 7, unsafe { crcInitializer });
        packet[0] |= (crc >> 6) & 0xFC;
        packet[7] = (crc & 0xFF) as u8;

        dbgPinHi(1);
        self.transmitData(packet, ELRS_RX_TX_BUFF_SIZE);
    }

    fn updatePhaseLock(&mut self)
    {
        if self.connectionState != ELRS_DISCONNECTED && expressLrsEPRHaveBothEvents() {
            let maxOffset: i32 = self.modParams.interval as i32 / 4;
            pl.rawOffsetUs = constrain(expressLrsEPRGetResult(), -maxOffset, maxOffset);

            pl.offsetUs = simpleLPFilterUpdate(&pl.offsetFilter, pl.rawOffsetUs);
            pl.offsetDeltaUs = simpleLPFilterUpdate(&pl.offsetDxFilter, pl.rawOffsetUs - pl.previousRawOffsetUs);

            if self.timerState == ELRS_TIM_LOCKED && lqPeriodIsSet() {
                if self.nonceRX % 8 == 0 { //limit rate of freq offset adjustment slightly
                    if pl.offsetUs > 0 {
                        expressLrsTimerIncreaseFrequencyOffset();
                    } else if pl.offsetUs < 0 {
                        expressLrsTimerDecreaseFrequencyOffset();
                    }
                }
            }

            if self.connectionState != ELRS_CONNECTED {
                expressLrsUpdatePhaseShift(pl.rawOffsetUs >> 1);
            } else {
                expressLrsUpdatePhaseShift(pl.offsetUs >> 2);
            }

            pl.previousOffsetUs = pl.offsetUs;
            pl.previousRawOffsetUs = pl.rawOffsetUs;

            expressLrsTimerDebug();

            DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 0, pl.rawOffsetUs);
            DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 1, pl.offsetUs);
        }

        expressLrsEPRReset();
    }

    //hwTimerCallbackTick
    fn expressLrsOnTimerTickISR(&mut self) // this is 180 out of phase with the other callback, occurs mid-packet reception
    {
        updatePhaseLock();
        self.nonceRX += 1;

        // Save the LQ value before the inc() reduces it by 1
        self.uplinkLQ = lqGet();
        // Only advance the LQI period counter if we didn't send Telemetry this period
        if !self.alreadyTLMresp {
            lqNewPeriod();
        }

        self.alreadyTLMresp = false;
        self.alreadyFHSS = false;
    }

    //hwTimerCallbackTock
    fn expressLrsOnTimerTockISR(&mut self)
    {
        let currentTimeUs: u32 = micros();

        expressLrsEPRRecordEvent(EPR_INTERNAL, currentTimeUs);

        self.fhssRequired = true; //Rest of the code is moved to expressLrsDataReceived to avoid race condition
    }


    fn lostConnection(&mut self)
    {
        unsafe { lostConnectionCounter += 1 };

        self.rfModeCycleMultiplier = 1;
        self.connectionState = ELRS_DISCONNECTED; //set lost connection
        self.timerState = ELRS_TIM_DISCONNECTED;
        expressLrsTimerResetFrequencyOffset();
        self.freqOffset = 0;
        pl.offsetUs = 0;
        pl.offsetDeltaUs = 0;
        pl.rawOffsetUs = 0;
        pl.previousRawOffsetUs = 0;
        self.gotConnectionMs = 0;
        self.uplinkLQ = 0;
        lqReset();
        expressLrsPhaseLockReset();
        self.alreadyTLMresp = false;
        self.alreadyFHSS = false;

        if !self.inBindingMode {
            //while (micros() - expressLrsEPRGetResult() > 250); // time it just after the tock() TODO this currently breaks and is blocking, not a fan of this.
            expressLrsTimerStop();
            setRFLinkRate(self.nextRateIndex); // also sets to initialFreq
            startReceiving();
        }
    }

    fn tentativeConnection(&mut self, timeStampMs: u32)
    {
        self.connectionState = ELRS_TENTATIVE;
        unsafe { connectionHasModelMatch = false };
        self.timerState = ELRS_TIM_DISCONNECTED;
        self.freqOffset = 0;
        pl.offsetUs = 0;
        pl.previousRawOffsetUs = 0;
        expressLrsPhaseLockReset(); //also resets PFD
        self.rfModeCycledAtMs = timeStampMs; // give another 3 sec for lock to occur

        // The caller MUST call hwTimer.resume(). It is not done here because
        // the timer ISR will fire immediately and preempt any other code
    }

    fn gotConnection(&mut self, timeStampMs: u32)
    {
        if self.connectionState == ELRS_CONNECTED {
            return; // Already connected
        }

        self.lockRFmode = true; // currently works as if LOCK_ON_FIRST_CONNECTION was enabled

        self.connectionState = ELRS_CONNECTED; //we got a packet, therefore no lost connection
        self.timerState = ELRS_TIM_TENTATIVE;
        self.gotConnectionMs = timeStampMs;

        if rxExpressLrsSpiConfig().rateIndex != self.rateIndex {
            rxExpressLrsSpiConfigMutable().rateIndex = self.rateIndex;
            self.configChanged = true;
        }
    }

    fn unpackBindPacket(&mut self, packet: &[u8])
    {
        rxExpressLrsSpiConfigMutable().UID[2] = packet[3];
        rxExpressLrsSpiConfigMutable().UID[3] = packet[4];
        rxExpressLrsSpiConfigMutable().UID[4] = packet[5];
        rxExpressLrsSpiConfigMutable().UID[5] = packet[6];

        self.UID = rxExpressLrsSpiConfigMutable().UID;
        unsafe { crcInitializer = ((self.UID[4] as u16) << 8) | (self.UID[5] as u16) };
        self.inBindingMode = false;

        initializeReceiver();

        self.configChanged = true; //after initialize as it sets it to false
    }

    /**
     * Process the assembled MSP packet in mspBuffer[]
     **/
    fn processRFMspPacket(&mut self, packet: &[u8])
    {
        // Always examine MSP packets for bind information if in bind mode
        // [1] is the package index, first packet of the MSP
        if self.inBindingMode && packet[1] == 1 && packet[2] == ELRS_MSP_BIND {
            unpackBindPacket(packet); //onELRSBindMSP
            return;
        }

// #ifdef USE_MSP_OVER_TELEMETRY
        // Must be fully connected to process MSP, prevents processing MSP
        // during sync, where packets can be received before connection
        if self.connectionState != ELRS_CONNECTED {
            return;
        }

        let currentMspConfirmValue: bool = getCurrentMspConfirm();
        receiveMspData(packet[1], packet + 2);


        unsafe {
            if currentMspConfirmValue != getCurrentMspConfirm() {
                nextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
            }
            if hasFinishedMspData() {
                if mspBuffer[ELRS_MSP_COMMAND_INDEX] == ELRS_MSP_SET_RX_CONFIG && mspBuffer[ELRS_MSP_COMMAND_INDEX + 1] == ELRS_MSP_MODEL_ID { //mspReceiverComplete
                    if (rxExpressLrsSpiConfig().modelId != mspBuffer[9]) { //UpdateModelMatch
                        rxExpressLrsSpiConfigMutable().modelId = mspBuffer[9];
                        self.configChanged = true;
                        self.connectionState = ELRS_DISCONNECT_PENDING;
                    }
                } else if connectionHasModelMatch {
                    processMspPacket(mspBuffer);
                }

                mspReceiverUnlock();
            }
        }
// #endif
    }

    fn processRFSyncPacket(&mut self, packet: &[u8], timeStampMs: u32) -> bool
    {
        // Verify the first two of three bytes of the binding ID, which should always match
        if packet[4] != self.UID[3] || packet[5] != self.UID[4] {
            return false;
        }

        // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
        // Only require the first 18 bits of the UID to match to establish a connection
        // but the last 6 bits must modelmatch before sending any data to the FC
        if (packet[6] & !ELRS_MODELMATCH_MASK) != (self.UID[5] & !ELRS_MODELMATCH_MASK) {
            return false;
        }

        self.lastSyncPacketMs = timeStampMs;

        // Will change the packet air rate in loop() if this changes
        self.nextRateIndex = (packet[3] & 0xC0) >> 6;
        let tlmRateIn: u8 = (packet[3] & 0x38) >> 3;
        let switchEncMode: u8 = ((packet[3] & 0x06) >> 1) - 1; //spi implementation uses 0 based index for hybrid

        // Update switch mode encoding immediately
        if switchEncMode != rxExpressLrsSpiConfig().switchMode {
            rxExpressLrsSpiConfigMutable().switchMode = switchEncMode;
            self.configChanged = true;
        }

        // Update TLM ratio
        if self.modParams.tlmInterval != tlmRateIn {
            self.modParams.tlmInterval = tlmRateIn.into();
            unsafe { telemBurstValid = false };
        }

        // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
        let modelXor: u8 = (rxExpressLrsSpiConfig().modelId) & ELRS_MODELMATCH_MASK;
        let modelMatched: bool = packet[6] == (self.UID[5] ^ modelXor);

        unsafe {
            if self.connectionState == ELRS_DISCONNECTED || self.nonceRX != packet[2] || FHSSgetCurrIndex() != packet[1] || connectionHasModelMatch != modelMatched {
                FHSSsetCurrIndex(packet[1]);
                self.nonceRX = packet[2];

                tentativeConnection(timeStampMs);
                connectionHasModelMatch = modelMatched;

                if !expressLrsTimerIsRunning() {
                    return true;
                }
            }
        }

        return false;
    }

    fn processRFPacket(&mut self, payload: &[u8], timeStampUs: u32) -> RxSpiReceived
    {
        let mut packet = [0_u8; ELRS_RX_TX_BUFF_SIZE];

        self.receiveData(packet, ELRS_RX_TX_BUFF_SIZE);

        let type_: PacketType = (packet[0] & 0x03).into();
        let inCRC: u16 = (((packet[0] & 0xFC) as u16) << 6) | packet[7];

        // For SM_HYBRID the CRC only has the packet type in byte 0
        // For SM_HYBRID_WIDE the FHSS slot is added to the CRC in byte 0 on RC_DATA_PACKETs
        if type_ != ELRS_RC_DATA_PACKET || rxExpressLrsSpiConfig().switchMode != SwitchMode::HYBRID_WIDE {
            packet[0] = type_ as u8;
        } else {
            let nonceFHSSresult: u8 = self.nonceRX % self.modParams.fhssHopInterval;
            packet[0] = type_ | (nonceFHSSresult << 2);
        }
        let calculatedCRC = calcCrc14(packet, 7, unsafe { crcInitializer });

        if inCRC != calculatedCRC {
            return RX_SPI_RECEIVED_NONE;
        }

        expressLrsEPRRecordEvent(EPR_EXTERNAL, timeStampUs + PACKET_HANDLING_TO_TOCK_ISR_DELAY_US);

        let mut shouldStartTimer = false;
        let timeStampMs = millis();

        self.lastValidPacketMs = timeStampMs;

        match type_ {
            PacketType::RC_DATA_PACKET => {
                // Must be fully connected to process RC packets, prevents processing RC
                // during sync, where packets can be received before connection
                if self.connectionState == ELRS_CONNECTED && unsafe { connectionHasModelMatch } {
                    if rxExpressLrsSpiConfig().switchMode == SM_HYBRID_WIDE {
                        unsafe { wideSwitchIndex = hybridWideNonceToSwitchIndex(self.nonceRX) };
                        if self.modParams.tlmInterval.value() < 8 || unsafe { wideSwitchIndex } == 7 {
                            confirmCurrentTelemetryPayload((packet[6] & 0x40) >> 6);
                        }
                    } else {
                        confirmCurrentTelemetryPayload(packet[6] & (1 << 7));
                    }
                    memcpy(payload, &packet[1], 6); // stick data handling is done in expressLrsSetRcDataFromPayload
                }
            }
            PacketType::MSP_DATA_PACKET => {
                processRFMspPacket(&packet);
            }
            PacketType::TLM_PACKET => (), // Not implemented
            PacketType::SYNC_PACKET => {
                shouldStartTimer = processRFSyncPacket(&packet, timeStampMs) && !self.inBindingMode;
            }
        }

        // Store the LQ/RSSI/Antenna
        self.getRFlinkInfo(&self.rssi, &self.snr);
        // Received a packet, that's the definition of LQ
        lqIncrease();
        // Extend sync duration since we've received a packet at this rate
        // but do not extend it indefinitely
        self.rfModeCycleMultiplier = ELRS_MODE_CYCLE_MULTIPLIER_SLOW; //RFModeCycleMultiplierSlow

        if shouldStartTimer {
            expressLrsTimerResume();
        }

        self.fhssRequired = true;

        return RX_SPI_RECEIVED_DATA;
    }

    fn updateTelemetryBurst(&self)
    {
        unsafe {
            if telemBurstValid {
                return;
            }
            telemBurstValid = true;

            let hz: u32 = rateEnumToHz(self.modParams.enumRate);
            let ratiodiv: u32 = self.modParams.tlmInterval.value() as u32;
            telemetryBurstMax = (TELEM_MIN_LINK_INTERVAL * hz / ratiodiv / 1_000) as u8;

            // Reserve one slot for LINK telemetry
            if telemetryBurstMax > 1 {
                telemetryBurstMax -= 1;
            } else {
                telemetryBurstMax = 1;
            }

            // Notify the sender to adjust its expected throughput
            updateTelemetryRate(hz, ratiodiv, telemetryBurstMax);
        }
    }

    /* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
    fn cycleRfMode(&mut self, timeStampMs: u32)
    {
        if self.connectionState == ELRS_CONNECTED || self.inBindingMode {
            return;
        }
        // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
        if self.lockRFmode == false && (timeStampMs - self.rfModeCycledAtMs) > (self.cycleIntervalMs * self.rfModeCycleMultiplier) as u32 {
            self.rfModeCycledAtMs = timeStampMs;
            self.lastSyncPacketMs = timeStampMs;           // reset this variable
            self.rateIndex = (self.rateIndex + 1) % ELRS_RATE_MAX;
            setRFLinkRate(self.rateIndex); // switch between rates
            self.statsUpdatedAtMs = timeStampMs;
            lqReset();
            startReceiving();

            // Switch to FAST_SYNC if not already in it (won't be if was just connected)
            self.rfModeCycleMultiplier = 1;
        } // if time to switch RF mode
    }

    // // #ifdef USE_RX_SX1280
    // fn configureReceiverForSX1280(&mut self)
    // {
    //     self.init = (elrsRxInitFnPtr)
    //     sx1280Init;
    //     self.config = (elrsRxConfigFnPtr)
    //     sx1280Config;
    //     self.startReceiving = (elrsRxStartReceivingFnPtr)
    //     sx1280StartReceiving;
    //     self.rxISR = (elrsRxISRFnPtr)
    //     sx1280ISR;
    //     self.transmitData = (elrsRxTransmitDataFnPtr)
    //     sx1280TransmitData;
    //     self.receiveData = (elrsRxReceiveDataFnPtr)
    //     sx1280ReceiveData;
    //     self.getRFlinkInfo = (elrsRxGetRFlinkInfoFnPtr)
    //     sx1280GetLastPacketStats;
    //     self.setFrequency = (elrsRxSetFrequencyFnPtr)
    //     sx1280SetFrequencyReg;
    //     self.handleFreqCorrection = (elrsRxHandleFreqCorrectionFnPtr)
    //     sx1280AdjustFrequency;
    // }

    //setup
    fn expressLrsSpiInit(&mut self, rxConfig: &RxSpiConfig, rxRuntimeState: &RxRuntimeState, extiConfig: &RxSpiExtiConfig) -> bool
    {
        if !rxSpiExtiConfigured() {
            return false;
            return false;
        }

        rxSpiCommonIOInit(rxConfig);

        rxRuntimeState.channelCount = ELRS_MAX_CHANNELS;

        extiConfig.ioConfig = IOCFG_IPD;
        extiConfig.trigger = BETAFLIGHT_EXTI_TRIGGER_RISING;

        if rxExpressLrsSpiConfig().resetIoTag {
            self.resetPin = IOGetByTag(rxExpressLrsSpiConfig().resetIoTag);
        } else {
            self.resetPin = IO_NONE;
        }

        if rxExpressLrsSpiConfig().busyIoTag {
            self.busyPin = IOGetByTag(rxExpressLrsSpiConfig().busyIoTag);
        } else {
            self.busyPin = IO_NONE;
        }

        match rxExpressLrsSpiConfig().domain {
            ISM2400 => {
                configureReceiverForSX1280();
                unsafe { bindingRateIndex = ELRS_BINDING_RATE_24 };
            }
            _ => { return false },
        }

        if !self.init(self.resetPin, self.busyPin) {
            return false;
        }

        if rssiSource == RSSI_SOURCE_NONE {
            rssiSource = RSSI_SOURCE_RX_PROTOCOL;
        }

        if linkQualitySource == LQ_SOURCE_NONE {
            linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
        }

        if rxExpressLrsSpiConfig().UID[0] || rxExpressLrsSpiConfig().UID[1]
            || rxExpressLrsSpiConfig().UID[2] || rxExpressLrsSpiConfig().UID[3]
            || rxExpressLrsSpiConfig().UID[4] || rxExpressLrsSpiConfig().UID[5] {
            self.inBindingMode = false;
            self.UID = rxExpressLrsSpiConfig().UID;
            unsafe { crcInitializer = ((self.UID[4] as u16) << 8) | (self.UID[5] as u16) };
        } else {
            self.inBindingMode = true;
            self.UID = BindingUID;
            unsafe { crcInitializer = 0 };
        }

        expressLrsPhaseLockReset();

        expressLrsInitialiseTimer(RX_EXPRESSLRS_TIMER_INSTANCE, &self.timerUpdateCb);
        expressLrsTimerStop();

        generateCrc14Table();
        initializeReceiver();

        initTelemetry();
// #ifdef USE_MSP_OVER_TELEMETRY
        setMspDataToReceive(ELRS_MSP_BUFFER_SIZE, unsafe { mspBuffer }, ELRS_MSP_BYTES_PER_CALL);
// #endif

        // Timer IRQs must only be enabled after the receiver is configured otherwise race conditions occur.
        expressLrsTimerEnableIRQs();

        return true;
    }

    fn handleConnectionStateUpdate(&mut self, timeStampMs: u32)
    {
        if (self.connectionState != ELRS_DISCONNECTED) && (self.modParams.index != self.nextRateIndex) {  // forced change
            lostConnection();
            self.lastSyncPacketMs = timeStampMs;                    // reset this variable to stop rf mode switching and add extra time
            self.rfModeCycledAtMs = timeStampMs;         // reset this variable to stop rf mode switching and add extra time
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);

            if USE_RX_RSSI_DBM {
                setRssiDbmDirect(-130, RSSI_SOURCE_RX_PROTOCOL);
            }
            if USE_RX_LINK_QUALITY_INFO {
                setLinkQualityDirect(0);
            }
        }

        if (self.connectionState == ELRS_TENTATIVE && ((timeStampMs - self.lastSyncPacketMs) > self.rfPerfParams.rxLockTimeoutMs)) {
            lostConnection();
            self.rfModeCycledAtMs = timeStampMs;
            self.lastSyncPacketMs = timeStampMs;
        }

        cycleRfMode(timeStampMs);

        let localLastValidPacket = self.lastValidPacketMs; // Required to prevent race condition due to LastValidPacket getting updated from ISR
        if (self.connectionState == ELRS_DISCONNECT_PENDING) || // check if we lost conn.
            (self.connectionState == ELRS_CONNECTED) && (self.rfPerfParams.disconnectTimeoutMs < (timeStampMs - localLastValidPacket)) {
            lostConnection();
        }

        if (self.connectionState == ELRS_TENTATIVE) && (ABS(pl.offsetDeltaUs) <= 10) && (pl.offsetUs < 100) && (lqGet() > minLqForChaos()) {
            gotConnection(timeStampMs); // detects when we are connected
        }

        if (self.timerState == ELRS_TIM_TENTATIVE) && ((timeStampMs - self.gotConnectionMs) > ELRS_CONSIDER_CONNECTION_GOOD_MS) && (ABS(pl.offsetDeltaUs) <= 5) {
            self.timerState = ELRS_TIM_LOCKED;
        }

        if (self.connectionState == ELRS_CONNECTED) && (ABS(pl.offsetDeltaUs) > 10) && (pl.offsetUs >= 100) && (lqGet() <= minLqForChaos()) {
            lostConnection(); // SPI: resync when we're in chaos territory
        }
    }

    fn handleConfigUpdate(&mut self, timeStampMs: u32)
    {
        if (timeStampMs - self.configCheckedAtMs) > ELRS_CONFIG_CHECK_MS {
            self.configCheckedAtMs = timeStampMs;
            if self.configChanged {
                writeEEPROM();
                self.configChanged = false;
            }
        }
    }

    fn handleLinkStatsUpdate(&mut self, timeStampMs: u32)
    {
        if (timeStampMs - self.statsUpdatedAtMs) > ELRS_LINK_STATS_CHECK_MS {
            self.statsUpdatedAtMs = timeStampMs;

            if self.connectionState == ELRS_CONNECTED {
                self.rssiFiltered = simpleLPFilterUpdate(&rssiFilter, self.rssi);
                let rssiScaled: u16 = scaleRange(constrain(self.rssiFiltered, self.rfPerfParams.sensitivity, -50), self.rfPerfParams.sensitivity, -50, 0, 1023);
                setRssi(rssiScaled, RSSI_SOURCE_RX_PROTOCOL);
// #ifdef USE_RX_RSSI_DBM
                setRssiDbm(self.rssiFiltered, RSSI_SOURCE_RX_PROTOCOL);
// #endif
// #ifdef USE_RX_LINK_QUALITY_INFO
                setLinkQualityDirect(self.uplinkLQ);
// #endif
// #ifdef USE_RX_LINK_UPLINK_POWER
                rxSetUplinkTxPwrMw(txPowerIndexToValue(unsafe { txPower }));
// #endif
            } else {
                setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
// #ifdef USE_RX_RSSI_DBM
                setRssiDbmDirect(-130, RSSI_SOURCE_RX_PROTOCOL);
// #endif
// #ifdef USE_RX_LINK_QUALITY_INFO
                setLinkQualityDirect(0);
// #endif
            }
        }
    }

    fn handleTelemetryUpdate(&mut self)
    {
        if self.connectionState != ConnectionState::CONNECTED || (self.modParams.lmInterval == TlmRatio::NO_TLM) {
            return;
        }

        let mut nextPayload = [0_u8; 69];
        let mut nextPlayloadSize: u8 = 0;
        if !isTelemetrySenderActive() && getNextTelemetryPayload(&nextPlayloadSize, &nextPayload) {
            setTelemetryDataToTransmit(nextPlayloadSize, nextPayload, ELRS_TELEMETRY_BYTES_PER_CALL);
        }
        updateTelemetryBurst();
    }

    fn expressLrsSetRcDataFromPayload(rcData: &mut [u16], payload: &[u8])
    {
        // todo: Make rcData and payload optional, instead of checking for null pointers, if this comes up?
        // if rcData && payload {
        if rxExpressLrsSpiConfig().switchMode == SwitchMode::HYBRID_WIDE {
            unpackChannelDataHybridWide(rcData, payload);
        } else {
            unpackChannelDataHybridSwitch8(rcData, payload);
        }
        // }
    }

    fn enterBindingMode(&mut self)
    {
        if (self.connectionState == ELRS_CONNECTED) || self.inBindingMode {
            // Don't enter binding if:
            // - we're already connected
            // - we're already binding
            return;
        }

        // Set UID to special binding values
        self.UID = BindingUID;
        unsafe { crcInitializer = 0 };
        self.inBindingMode = true;

        setRFLinkRate(unsafe { bindingRateIndex });
        startReceiving();
    }

    fn expressLrsDataReceived(&mut self, payload: &[u8]) -> RxSpiReceived
    {
        let mut result = RX_SPI_RECEIVED_NONE;

        if !self.started && (systemState & SYSTEM_STATE_READY) {
            self.started = true;
            startReceiving(); // delay receiving after initialization to ensure a clean connect
        }

        if rxSpiCheckBindRequested(true) {
            enterBindingMode();
        }

        let irqReason: u8 = self.rxISR(unsafe { isrTimeStampUs });
        if irqReason == ELRS_DIO_RX_AND_TX_DONE {
            startReceiving();
        } else if irqReason == ELRS_DIO_TX_DONE {
            startReceiving();
        } else if irqReason == ELRS_DIO_RX_DONE {
            result = processRFPacket(payload, unsafe { isrTimeStampUs });
        }

        if self.fhssRequired {
            self.fhssRequired = false;
            let mut didFHSS = false;
            let mut tlmReq = false;

            // todo: How to handle this?
            // ATOMIC_BLOCK(NVIC_PRIO_TIMER)
            { // prevent from updating nonce in TICK
                didFHSS = handleFHSS();
                tlmReq = shouldSendTelemetryResponse();
            }

            if tlmReq {
                // in case we miss a packet before TLM we still need to estimate processing time using %
                let processingTime = (micros() - unsafe { isrTimeStampUs }) % self.modParams.interval;
                if processingTime < PACKET_HANDLING_TO_TOCK_ISR_DELAY_US && self.timerState == ELRS_TIM_LOCKED {
                    handleSendTelemetryResponse();
                } else {
                    self.alreadyTLMresp = true;
                    self.startReceiving();
                }
            }

            if rxExpressLrsSpiConfig().domain != FreqDomain::ISM2400 && !didFHSS && !tlmReq && lqPeriodIsSet() {
                self.handleFreqCorrection(self.freqOffset, self.currentFreq); //corrects for RX freq offset
            }
        }

        handleTelemetryUpdate();

        let timeStampMs = millis();

        handleConnectionStateUpdate(timeStampMs);
        handleConfigUpdate(timeStampMs);
        handleLinkStatsUpdate(timeStampMs);

        DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 0, unsafe { lostConnectionCounter });
        DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 1, self.rssiFiltered);
        DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 2, self.snr);
        DEBUG_SET(DEBUG_RX_EXPRESSLRS_SPI, 3, self.uplinkLQ);

        if self.inBindingMode { rxSpiLedBlinkBind() } else { rxSpiLedBlinkRxLoss(result) };

        return result;
    }

    fn expressLrsStop(&mut self)
    {
        if self.started {
            self.lostConnection();
        }
    }

    fn minLqForChaos(&mut self) -> u8
    {
        // Determine the most number of CRC-passing packets we could receive on
        // a single channel out of 100 packets that fill the LQcalc span.
        // The LQ must be GREATER THAN this value, not >=
        // The amount of time we coexist on the same channel is
        // 100 divided by the total number of packets in a FHSS loop (rounded up)
        // and there would be 4x packets received each time it passes by so
        // FHSShopInterval * ceil(100 / FHSShopInterval * numfhss) or
        // FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
        // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
        let numfhss: u32 = getFHSSNumEntries();
        let interval: u8 = self.modParams.fhssHopInterval;
        interval * ((interval * numfhss + 99) / (interval * numfhss))
    }

    fn setRssiChannelData(&self, rcData: &[u16])
    {
        rcData[ELRS_LQ_CHANNEL] = scaleRange(self.uplinkLQ, 0, 100, 988, 2011);
        rcData[ELRS_RSSI_CHANNEL] = scaleRange(constrain(self.rssiFiltered, self.rfPerfParams.sensitivity, -50), self.rfPerfParams.sensitivity, -50, 988, 2011);
    }

    /**
     * HybridWide switches decoding of over the air data
     *
     * Hybrid switches uses 10 bits for each analog channel,
     * 1 bits for the low latency switch[0]
     * 6 or 7 bits for the round-robin switch
     * 1 bit for the TelemetryStatus, which may be in every packet or just idx 7
     * depending on TelemetryRatio
     *
     * Output: crsf.PackedRCdataOut, crsf.LinkStatistics.uplink_TX_Power
     * Returns: TelemetryStatus bit
     */
    fn unpackChannelDataHybridWide(&self, rcData: &mut [u16], payload: &[u8])
    {
        unpackAnalogChannelData(rcData, payload);
        let switchByte: u8 = payload[5];

        // The low latency switch
        rcData[4] = convertSwitch1b((switchByte & 0x80) >> 7);

        // The round-robin switch, 6-7 bits with the switch index implied by the nonce. Some logic moved to processRFPacket
        unsafe {
            if wideSwitchIndex >= 7 {
                txPower = switchByte & 0x3F;
            } else {
                let mut bins: u8 = 0;
                let mut switchValue: u16 = 0;

                if self.modParams.tlmInterval.value() < 8 {
                    bins = 63;
                    switchValue = switchByte as u16 & 0x3F; // 6-bit
                } else {
                    bins = 127;
                    switchValue = switchByte as u16 & 0x7F; // 7-bit
                }

                switchValue = convertSwitchNb(switchValue, bins);
                rcData[5 + wideSwitchIndex] = switchValue;
            }
        }

        setRssiChannelData(rcData);
    }
}



// From `expresslrs_telemetry.h`:

const ELRS_TELEMETRY_SHIFT: u8 = 2;
const ELRS_TELEMETRY_BYTES_PER_CALL: u8 = 5;
const ELRS_TELEMETRY_MAX_PACKAGES: u8 = (255 >> ELRS_TELEMETRY_SHIFT);
const ELRS_TELEMETRY_MAX_MISSED_PACKETS: u8 = 20;

const ELRS_MSP_BYTES_PER_CALL: u8 = 5;
const ELRS_MSP_BUFFER_SIZE: usize = 65;
const ELRS_MSP_MAX_PACKAGES: usize = ((ELRS_MSP_BUFFER_SIZE / ELRS_MSP_BYTES_PER_CALL) + 1);
const ELRS_MSP_PACKET_OFFSET: u8 = 5;
const ELRS_MSP_COMMAND_INDEX: u8 = 7;

#[repr(u8)]
enum StubbornSensorState {
    ELRS_SENDER_IDLE = 0,
    ELRS_SENDING = 1,
    ELRS_WAIT_UNTIL_NEXT_CONFIRM = 2,
    ELRS_RESYNC = 3,
    ELRS_RESYNC_THEN_SEND = 4, // perform a RESYNC then go to SENDING
}

// From `expresslrs.c`:

// * Authors:
// * Phobos- - Original port.
// * Dominic Clifton/Hydra - Timer-based timeout implementation.
// * Phobos- - Port of v2.0
// */


// STATIC_UNIT_TESTED elrsReceiver_t receiver;

const BindingUID: [u8; 6] = [0, 1, 2, 3, 4, 5];
// Special binding UID values
static mut crcInitializer: u16 = 0;
static mut bindingRateIndex: u8 = 0;
static mut connectionHasModelMatch: bool = false;
static mut txPower: u8 = 0;
static mut wideSwitchIndex: u8 = 0;
static mut isrTimeStampUs: u32 = 0;
static mut lostConnectionCounter: u16 = 0;


// static mut simpleLowpassFilter_t rssiFilter;

// fn rssiFilterReset()
// {
// simpleLPFilterInit(rssiFilter, 3, 5); // todo
// }

const PACKET_HANDLING_TO_TOCK_ISR_DELAY_US: u32 = 350;

//
// Event pair recorder
//

#[derive(Copy, Clone)]
#[repr(usize)]
enum EprEvent {
    First = 0,
    Second = 1,
}

const EPR_EVENT_COUNT: usize = 2;

struct EprState {
    eventAtUs: [u32; EPR_EVENT_COUNT],
    eventRecorded: [bool; EPR_EVENT_COUNT],
}

static mut eprState: EprState = EprState {
    eventAtUs: [0, 0],
    eventRecorded: [false, false],
};

fn expressLrsEPRRecordEvent(event: EprEvent, currentTimeUs: u32)
{
    unsafe {
        eprState.eventAtUs[event as usize] = currentTimeUs;
        eprState.eventRecorded[event as usize] = true;
    }
}

fn expressLrsEPRHaveBothEvents() -> bool
{
    unsafe {
        eprState.eventRecorded[EprEvent::Second as usize] && eprState.eventRecorded[EprEvent::First as usize]
    }
}

fn expressLrsEPRGetResult() -> i32
{
    if !expressLrsEPRHaveBothEvents() {
        return 0;
    }

    unsafe {
        (eprState.eventAtUs[EprEvent::Second as usize] - eprState.eventAtUs
            [EprEvent::First as usize]) as i32
    }
}

fn expressLrsEPRReset()
{
    unsafe {
        memset(&eprState, 0, sizeof(eprState_t));
    }
}


//
// Phase Lock
//


// #define EPR_INTERNAL EPR_FIRST
// #define EPR_EXTERNAL EPR_SECOND

struct PhaseLockState {
    offsetFilter: SimpleLowpassFilter,
    offsetDxFilter: SimpleLowpassFilter,

    rawOffsetUs: i32,
    previousRawOffsetUs: i32,

    offsetUs: i32,
    offsetDeltaUs: i32,
    previousOffsetUs: i32,
}

// static phaseLockState_t pl;

fn expressLrsPhaseLockReset()
{
    simpleLPFilterInit(&pl.offsetFilter, 2, 5);
    simpleLPFilterInit(&pl.offsetDxFilter, 4, 5);

    expressLrsEPRReset();
}


fn unpackAnalogChannelData(rcData: &mut [u16], payload: &[u8])
{
    rcData[0] = convertAnalog((payload[0] << 3) | ((payload[4] & 0xC0) >> 5));
    rcData[1] = convertAnalog((payload[1] << 3) | ((payload[4] & 0x30) >> 3));
    rcData[2] = convertAnalog((payload[2] << 3) | ((payload[4] & 0x0C) >> 1));
    rcData[3] = convertAnalog((payload[3] << 3) | ((payload[4] & 0x03) << 1));
}

/**
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 * 4 analog channels, 1 low latency switch and round robin switch data = 47 bits (1 free)
 *
 * sets telemetry status bit
 */
fn unpackChannelDataHybridSwitch8(rcData: &mut [u16], payload: &[u8])
{
    unpackAnalogChannelData(rcData, payload);

    let switchByte: u8 = payload[5];

    // The low latency switch
    rcData[4] = convertSwitch1b((switchByte & 0x40) >> 6);

    // The round-robin switch, switchIndex is actually index-1
    // to leave the low bit open for switch 7 (sent as 0b11x)
    // where x is the high bit of switch 7
    let switchIndex: u8 = (switchByte & 0x38) >> 3;
    let switchValue: u16 = convertSwitch3b(switchByte & 0x07);

    match switchIndex {
        0 => {
            rcData[5] = switchValue;
        }
        1 => {
            rcData[6] = switchValue;
        }
        2 => {
            rcData[7] = switchValue;
        }
        3 => {
            rcData[8] = switchValue;
        }
        4 => {
            rcData[9] = switchValue;
        }
        5 => {
            rcData[10] = switchValue;
        }
        6 => (),
        7 => {
            rcData[11] = convertSwitchNb(switchByte & 0x0F, 15); //4-bit
        }
        _ => ()
    }

    setRssiChannelData(rcData);
}



// todo: Where does this come from?
// fn startReceiving()
// {
//     dbgPinLo(1);
//     self.startReceiving();
// }


// #endif /* USE_RX_EXPRESSLRS */
//
