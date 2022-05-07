#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here:
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/rx_main.cpp

use core::mem;

use cortex_m::delay::Delay;

use stm32_hal2::{pac::Tim4, timer::Timer};

use defmt::println;

use super::{
    common::*, fhss, lowpassfilter::LPF, lqcalc, ota::*, pfd, pfd::*, stubborn::*, sx1280::*,
    telemetry::*,
};

///LUA///
const LUA_MAX_PARAMS: u32 = 32;
////

//// CONSTANTS ////
// todo: Confirm these types.
const SEND_LINK_STATS_TO_FC_INTERVAL: u8 = 100;
const DIVERSITY_ANTENNA_INTERVAL: i32 = 5;
const DIVERSITY_ANTENNA_RSSI_TRIGGER: i32 = 5;
const PACKET_TO_TOCK_SLACK: u8 = 200; // Desired buffer time between Packet ISR and Tock ISR
                                      ///////////////////

const Regulatory_Domain_EU_CE_2400: bool = false;

// todo: Do we want this since we only have 1 antenna port?
static mut antenna: u8 = 0; // which antenna is currently in use

static mut PFDloop: PFD = unsafe { mem::zeroed() };
static mut config: RxConfig = unsafe { mem::zeroed() };
static mut telemetry: Telemetry = unsafe { mem::zeroed() };

static mut telemetryBurstCount: u8 = 0;
static mut telemetryBurstMax: u8 = 0;

static mut NextTelemetryType: u8 = ELRS_TELEMETRY_TYPE_LINK;
static mut telemBurstValid: bool = false;

/// Filters ////////////////
// todo: Replace these with CMSIS-DSP IIR lowpass
static mut LPF_Offset: LPF = LPF {
    SmoothDataINT: 0,
    SmoothDataFP: 0,
    Beta: 2,
    FP_Shift: 5,
    NeedReset: true,
};
static mut LPF_OffsetDx: LPF = LPF {
    SmoothDataINT: 0,
    SmoothDataFP: 0,
    Beta: 4,
    FP_Shift: 5,
    NeedReset: true,
};

static mut LPF_UplinkRSSI: LPF = LPF {
    SmoothDataINT: 0,
    SmoothDataFP: 0,
    Beta: 5,
    FP_Shift: 5,
    NeedReset: true,
};
static mut LPF_UplinkRSSI0: LPF = LPF {
    SmoothDataINT: 0,
    SmoothDataFP: 0,
    Beta: 5,
    FP_Shift: 5,
    NeedReset: true,
};
static mut LPF_UplinkRSSI1: LPF = LPF {
    SmoothDataINT: 0,
    SmoothDataFP: 0,
    Beta: 5,
    FP_Shift: 5,
    NeedReset: true,
};

/// LQ Calculation //////////
static mut LQCalc: lqcalc::LQCALC = lqcalc::LQCALC {
    LQ: 0,
    index: 0, // current position in LQArray
    count: 1,
    LQmask: (1 << 0),
    LQArray: [0; (100 + 31) / 32], // `100` here is `<N>` in original
};

static mut uplinkLQ: u8 = 0;

static mut scanIndex: u8 = RATE_DEFAULT;
static mut nextAirRateIndex: u8 = 0;

static mut PfdPrevRawOffset: i32 = 0;
static mut RxTimerState: RXtimerState = unsafe { mem::zeroed() };
static mut GotConnectionMillis: u32 = 0;
const ConsiderConnGoodMillis: u32 = 1000; // minimum time before we can consider a connection to be 'good'

///////////////////////////////////////////////

static mut NonceRX: u8 = 0; // nonce that we THINK we are up to.

static mut alreadyFHSS: bool = false;
static mut alreadyTLMresp: bool = false;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
static mut LastValidPacket: u32 = 0; //Time the last valid packet was recv
static mut LastSyncPacket: u32 = 0; //Time the last valid packet was recv

static mut SendLinkStatstoFCintervalLastSent: u32 = 0;
static mut SendLinkStatstoFCForcedSends: u8 = 0;

static mut RFnoiseFloor: i16 = 0; //measurement of the current RF noise floor
                                  ///////////////////////////////////////////////////////////////

/// Variables for Sync Behaviour ////
static mut cycleInterval: u32 = 0; // in ms
static mut RFmodeLastCycled: u32 = 0;
static mut RFmodeCycleMultiplierSlow: u8 = 10;
static mut RFmodeCycleMultiplier: u8 = 0;
static mut LockRFmode: bool = false;
///////////////////////////////////////

static mut InBindingMode: bool = false;
static mut InLoadBindingMode: bool = false;
static mut returnModeFromLoad: bool = false;

fn IsArmed() -> bool {
    GetChannelOutput(AUX1)
}

fn minLqForChaos() -> u8 {
    // Determine the most number of CRC-passing packets we could receive on
    // a single channel out of 100 packets that fill the LQcalc span.
    // The LQ must be GREATER THAN this value, not >=
    // The amount of time we coexist on the same channel is
    // 100 divided by the total number of packets in a FHSS loop (rounded up)
    // and there would be 4x packets received each time it passes by so
    // FHSShopInterval * ceil(100 / FHSShopInterval * numfhss) or
    // FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
    // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
    let numfhss = fhss::getChannelCount();
    let interval: u8 = ExpressLRS_currAirRate_Modparams.FHSShopInterval;
    interval * ((interval as u32 * numfhss + 99) / (interval as u32 * numfhss))
}

unsafe fn getRFlinkInfo() {
    let mut rssiDBM: i32 = Radio.LastPacketRSSI;
    if antenna == 0 {
        if rssiDBM > 0 {
            rssiDBM = 0;
        }
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        // crsf.LinkStatistics.uplink_RSSI_1 = -rssiDBM;
    } else {
        if rssiDBM > 0 {
            rssiDBM = 0;
        }
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        // May be overwritten below if DEBUG_BF_LINK_STATS is set
        // crsf.LinkStatistics.uplink_RSSI_2 = -rssiDBM;
    }

    // crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(constrain(rssiDBM, ExpressLRS_currAirRate_RFperfParams.RXsensitivity, -50),
    //                                                ExpressLRS_currAirRate_RFperfParams.RXsensitivity, -50, 0, 1023));
    // crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(uplinkLQ, 0, 100, 0, 1023));
    //
    // crsf.LinkStatistics.active_antenna = antenna;
    // crsf.LinkStatistics.uplink_SNR = Radio.LastPacketSNR;
    //
    // crsf.LinkStatistics.rf_Mode = ExpressLRS_currAirRate_Modparams.enum_rate;
}

unsafe fn SetRFLinkRate(index: u8, hwTimer: &mut Timer<TIM4>) // Set speed of RF link
{
    let ModParams: ModSettings = get_elrs_airRateConfig(index);
    let RFperf: RfPrefParams = get_elrs_RFperfParams(index);
    let invertIQ = (UID[5] & 0x01) != 0;

    hwTimer.set_period(ModParams.interval);

    Radio.Config(
        ModParams.bw,
        ModParams.sf,
        ModParams.cr,
        GetInitialFreq(),
        ModParams.PreambleLen,
        invertIQ,
        ModParams.PayloadLength,
        0,
        uidMacSeedGet(),
        CRCInitializer,
        (ModParams.radio_type == RadioType::SX128x_FLRC), // #endif
    );

    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    unsafe {
        cycleInterval =
            (11 * fhss::getChannelCount() * ModParams.FHSShopInterval * ModParams.interval)
                / (10 * 1000);
    }

    CurrAirRateModParams = ModParams;
    CurrAirRateRfPerfParams = RFperf;
    nextAirRateIndex = index; // presumably we just handled this
    unsafe {
        telemBurstValid = false;
    }
}

unsafe fn HandleFHSS() -> bool {
    let modresultFHSS: u8 = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams.FHSShopInterval;

    if ExpressLRS_currAirRate_Modparams.FHSShopInterval == 0
        || alreadyFHSS == true
        || InBindingMode
        || modresultFHSS != 0
        || connectionState == ConnectionState::disconnected
    {
        return false;
    }

    alreadyFHSS = true;
    Radio.SetFrequencyReg(FHSSgetNextFreq());

    let modresultTLM: u8 =
        (NonceRX + 1) % (TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval));

    if modresultTLM != 0 || ExpressLRS_currAirRate_Modparams.TLMinterval == TlmRatio::NO_TLM
    // if we are about to send a tlm response don't bother going back to rx
    {
        Radio.RXnb();
    }
    true
}

unsafe fn HandleSendTelemetryResponse(radio: &mut SX12xxDriverCommon) -> bool {
    let mut data: [u8; 69] = [0; 69];
    let mut maxLength: u8 = 0;
    let mut packageIndex: u8 = 0;
    let modresult =
        (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval);

    if connectionState == ConnectionState::disconnected
        || ExpressLRS_currAirRate_Modparams.TLMinterval == TlmRatio::NO_TLM
        || alreadyTLMresp == true
        || modresult != 0
    {
        return false; // don't bother sending tlm if disconnected or TLM is off
    }

    if Regulatory_Domain_EU_CE_2400 {
        BeginClearChannelAssessment();
    }

    alreadyTLMresp = true;
    radio.TXdataBuffer[0] = PacketHeaderType::TLM_PACKET as u8;

    if NextTelemetryType == ELRS_TELEMETRY_TYPE_LINK || !TelemetrySender.IsActive() {
        radio.TXdataBuffer[1] = ELRS_TELEMETRY_TYPE_LINK;
        // The value in linkstatistics is "positivized" (inverted polarity)
        // and must be inverted on the TX side. Positive values are used
        // so save a bit to encode which antenna is in use
        // Radio.TXdataBuffer[2] = crsf.LinkStatistics.uplink_RSSI_1 | (antenna << 7);
        // Radio.TXdataBuffer[3] = crsf.LinkStatistics.uplink_RSSI_2 | (connectionHasModelMatch << 7);
        // Radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
        // Radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;
        // Radio.TXdataBuffer[6] = if MspReceiver.GetCurrentConfirm() {
        //     1
        // } else {
        //     0
        // };

        NextTelemetryType = ELRS_TELEMETRY_TYPE_DATA;
        // Start the count at 1 because the next will be DATA and doing +1 before checking
        // against Max below is for some reason 10 bytes more code
        telemetryBurstCount = 1;
    } else {
        if telemetryBurstCount < telemetryBurstMax {
            telemetryBurstCount += 1;
        } else {
            NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
        }

        TelemetrySender.GetCurrentPayload(&mut packageIndex, &mut maxLength, &data);
        radio.TXdataBuffer[1] = (packageIndex << ELRS_TELEMETRY_SHIFT) + ELRS_TELEMETRY_TYPE_DATA;
        radio.TXdataBuffer[2] = if maxLength > 0 { data[0] } else { 0 };
        radio.TXdataBuffer[3] = if maxLength >= 1 { data[1] } else { 0 };
        radio.TXdataBuffer[4] = if maxLength >= 2 { data[2] } else { 0 };
        radio.TXdataBuffer[5] = if maxLength >= 3 { data[3] } else { 0 };
        radio.TXdataBuffer[6] = if maxLength >= 4 { data[4] } else { 0 };
    }

    let crc: u16 = ota_crc.calc(radio.TXdataBuffer, 7, CRCInitializer);
    radio.TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    radio.TXdataBuffer[7] = crc as u8;

    if !Regulatory_Domain_EU_CE_2400 || ChannelIsClear() {
        radio.TXnb();
    }
    true
}

fn HandleFreqCorr(value: bool) {
    if value {
        if FreqCorrection > FreqCorrectionMin {
            FreqCorrection -= 1; // FREQ_STEP units
        } else {
            println!("Max -FreqCorrection reached!");
        }
    } else {
        if FreqCorrection < FreqCorrectionMax {
            FreqCorrection += 1; // FREQ_STEP units
        } else {
            println!("Max +FreqCorrection reached!");
        }
    }
}

unsafe fn updatePhaseLock(hwTimer: &mut Timer<TIM4>) {
    if connectionState != ConnectionState::disconnected {
        PFDloop.calcResult();
        PFDloop.reset();

        let RawOffset: i32 = PFDloop.getResult();
        let Offset: i32 = LPF_Offset.update(RawOffset);
        let OffsetDx: i32 = LPF_OffsetDx.update(RawOffset - prevRawOffset);
        PfdPrevRawOffset = RawOffset;

        if RxTimerState == RXtimerState::Locked && LQCalc.currentIsSet() {
            if unsafe { NonceRX % 8 == 0 }
            //limit rate of freq offset adjustment slightly
            {
                if Offset > 0 {
                    hwTimer.incFreqOffset(); // todo - what?
                } else if Offset < 0 {
                    hwTimer.decFreqOffset();
                }
            }
        }

        if connectionState != ConnectionState::connected {
            hwTimer.phaseShift(RawOffset >> 1); // todo what?
        } else {
            hwTimer.phaseShift(Offset >> 2);
        }
    }

    println!(
        "{}:{}:{}:{}:{}",
        // Offset,
        RawOffset,
        OffsetDx,
        hwTimer.FreqOffset,
        unsafe { uplinkLQ },
    );
}

/// this is 180 out of phase with the other callback, occurs mid-packet reception.
/// Call this from the main ISR.
pub unsafe fn HWtimerCallbackTick(hwTimer: &mut Timer<TIM4>) {
    updatePhaseLock(hwTimer);
    NonceRX += 1;

    // Save the LQ value before the inc() reduces it by 1
    uplinkLQ = LQCalc.getLQ();
    // crsf.LinkStatistics.uplink_Link_quality = uplinkLQ;
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if !alreadyTLMresp {
        LQCalc.inc();
    }

    alreadyTLMresp = false;
    alreadyFHSS = false;
    // crsf.RXhandleUARTout();
}

//////////////////////////////////////////////////////////////
// flip to the other antenna
// no-op if GPIO_PIN_ANTENNA_SELECT not defined
#[inline(always)]
unsafe fn switchAntenna() {
    // if GPIO_PIN_ANTENNA_SELECT && USE_DIVERSITY {
    //     if config.GetAntennaMode() == 2 {
    //         //0 and 1 is use for gpio_antenna_select
    //         // 2 is diversity
    //         antenna = !antenna;
    //         if antenna == 0 {
    //             LPF_UplinkRSSI0.reset()
    //         } else {
    //             LPF_UplinkRSSI1.reset()
    //         }; // discard the outdated value after switching
    //         digitalWrite(GPIO_PIN_ANTENNA_SELECT, antenna);
    //     }
    // }
}

// from updateDiversity:
static mut prevRSSI: i32 = 0; // saved rssi so that we can compare if switching made things better or worse
static mut antennaLQDropTrigger: i32 = 0;
static mut antennaRSSIDropTrigger: i32 = 0;

unsafe fn updateDiversity() {
    if GPIO_PIN_ANTENNA_SELECT && USE_DIVERSITY {
        //     if config.GetAntennaMode() == 2 {
        //         //0 and 1 is use for gpio_antenna_select
        //         // 2 is diversity
        //
        //         let rssi: i32 = if antenna == 0 {
        //             LPF_UplinkRSSI0.SmoothDataINT
        //         } else {
        //             LPF_UplinkRSSI1.SmoothDataINT
        //         };
        //         let otherRSSI: i32 = if antenna == 0 {
        //             LPF_UplinkRSSI1.SmoothDataINT
        //         } else {
        //             LPF_UplinkRSSI0.SmoothDataINT
        //         };
        //
        //         //if rssi dropped by the amount of DIVERSITY_ANTENNA_RSSI_TRIGGER
        //         if (rssi < (prevRSSI - DIVERSITY_ANTENNA_RSSI_TRIGGER))
        //             && antennaRSSIDropTrigger >= DIVERSITY_ANTENNA_INTERVAL
        //         {
        //             switchAntenna();
        //             antennaLQDropTrigger = 1;
        //             antennaRSSIDropTrigger = 0;
        //         } else if (rssi > prevRSSI || antennaRSSIDropTrigger < DIVERSITY_ANTENNA_INTERVAL) {
        //             prevRSSI = rssi;
        //             antennaRSSIDropTrigger += 1;
        //         }
        //
        //         // if we didn't get a packet switch the antenna
        //         if !LQCalc.currentIsSet() && antennaLQDropTrigger == 0 {
        //             switchAntenna();
        //             antennaLQDropTrigger = 1;
        //             antennaRSSIDropTrigger = 0;
        //         } else if antennaLQDropTrigger >= DIVERSITY_ANTENNA_INTERVAL {
        //             // We switched antenna on the previous packet, so we now have relatively fresh rssi info for both antennas.
        //             // We can compare the rssi values and see if we made things better or worse when we switched
        //             if rssi < otherRSSI {
        //                 // things got worse when we switched, so change back.
        //                 switchAntenna();
        //                 antennaLQDropTrigger = 1;
        //                 antennaRSSIDropTrigger = 0;
        //             } else {
        //                 // all good, we can stay on the current antenna. Clear the flag.
        //                 antennaLQDropTrigger = 0;
        //             }
        //         } else if (antennaLQDropTrigger > 0) {
        //             antennaLQDropTrigger += 1;
        //         }
        //     } else {
        //         digitalWrite(GPIO_PIN_ANTENNA_SELECT, config.GetAntennaMode());
        //         antenna = config.GetAntennaMode();
        //     }
        // }
    }
}

pub unsafe fn HWtimerCallbackTock(radio: &mut SX12xxDriverCommon) {
    if Regulatory_Domain_EU_CE_2400 {
        // Emulate that TX just happened, even if it didn't because channel is not clear
        if !LBTSuccessCalc.currentIsSet() {
            Radio.TXdoneCallback();
        }
    }

    PFDloop.intEvent(micros()); // our internal osc just fired

    updateDiversity();
    let didFHSS: bool = HandleFHSS();
    let tlmSent: bool = HandleSendTelemetryResponse(radio);

    if !didFHSS && !tlmSent && LQCalc.currentIsSet() && Radio.FrequencyErrorAvailable() {
        HandleFreqCorr(Radio.GetFrequencyErrorbool()); // Adjusts FreqCorrection for RX freq offset
    }
}

unsafe fn LostConnection(Radio: &mut SX12xxDriverCommon, hwTimer: &mut Timer<TIM4>) {
    defmt::println!("lost conn fc={} fo={}", FreqCorrection, hwTimer.FreqOffset);

    RFmodeCycleMultiplier = 1;
    connectionState = ConnectionState::disconnected; //set lost connection
    RxTimerState = RXtimerState::Disconnected;
    // hwTimer.resetFreqOffset(); // todo!
    FreqCorrection = 0;

    PfdPrevRawOffset = 0;
    GotConnectionMillis = 0;
    uplinkLQ = 0;
    LQCalc.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    alreadyTLMresp = false;
    alreadyFHSS = false;

    if !InBindingMode && hwTimer.enabled() {
        while micros() - PFDloop.intEventTime > 250 {} // time it just after the tock()
        hwTimer.disable();
        SetRFLinkRate(nextAirRateIndex, hwTimer); // also sets to initialFreq
        Radio.RXnb();
    }
}

unsafe fn TentativeConnection(now: u32) {
    PFDloop.reset();
    connectionState = ConnectionState::tentative;
    connectionHasModelMatch = false;
    RxTimerState = RXtimerState::Disconnected;
    println!("tentative conn");
    FreqCorrection = 0;
    PfdPrevRawOffset = 0;
    LPF_Offset.init(0);
    unsafe { RFmodeLastCycled = now } // give another 3 sec for lock to occur

    // The caller MUST call hwTimer.resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}

unsafe fn GotConnection(now: u32) {
    if connectionState == ConnectionState::connected {
        return; // Already connected
    }

    if LOCK_ON_FIRST_CONNECTION {
        unsafe { LockRFmode = true };
    }

    connectionState = ConnectionState::connected; //we got a packet, therefore no lost connection
    RxTimerState = RXtimerState::Tentative;
    unsafe { GotConnectionMillis = now };

    println!("got conn");
}

unsafe fn ProcessRfPacket_RC(Radio: &mut SX12xxDriverCommon) {
    // Must be fully connected to process RC packets, prevents processing RC
    // during sync, where packets can be received before connection
    if connectionState != ConnectionState::connected {
        return;
    }

    let telemetryConfirmValue: bool = UnpackChannelData(
        Radio.RXdataBuffer,
        // &crsf,
        unsafe { NonceRX },
        TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval),
    );
    TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);

    // No channels packets to the FC if no model match
    // if connectionHasModelMatch {
    //     if GPIO_PIN_PWM_OUTPUTS {
    //         unsafe {
    //             newChannelsAvailable = true;
    //         }
    //     } else {
    // crsf.sendRCFrameToFC();
    // }
    // }
}

unsafe fn ProcessRfPacket_SYNC(radio: &mut SX12xxDriverCommon, now: u32) -> bool {
    // Verify the first two of three bytes of the binding ID, which should always match
    if radio.RXdataBuffer[4] != UID[3] || radio.RXdataBuffer[5] != UID[4] {
        return false;
    }

    // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
    // Only require the first 18 bits of the UID to match to establish a connection
    // but the last 6 bits must modelmatch before sending any data to the FC
    if (radio.RXdataBuffer[6] & !MODELMATCH_MASK) != (UID[5] & !MODELMATCH_MASK) {
        return false;
    }

    unsafe { LastSyncPacket = now }

    // Will change the packet air rate in loop() if this changes
    unsafe {
        nextAirRateIndex =
            (radio.RXdataBuffer[3] >> SYNC_PACKET_RATE_OFFSET) & SYNC_PACKET_RATE_MASK;
    }
    // Update switch mode encoding immediately
    OtaSetSwitchMode(
        (radio.RXdataBuffer[3] >> SYNC_PACKET_SWITCH_OFFSET) & SYNC_PACKET_SWITCH_MASK,
    );
    // Update TLM ratio
    let TLMrateIn: TlmRatio =
        ((radio.RXdataBuffer[3] >> SYNC_PACKET_TLM_OFFSET) & SYNC_PACKET_TLM_MASK).into();

    if CurrAirRateModParams.TLMinterval != TLMrateIn {
        println!("New TLMrate: {}", TLMrateIn.value());
        CurrAirRateModParams.TLMinterval = TLMrateIn;
        unsafe {
            telemBurstValid = false;
        }
    }

    // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    let modelXor: u8 = (!config.GetModelId()) & MODELMATCH_MASK;
    let modelMatched: bool = radio.RXdataBuffer[6] == (UID[5] ^ modelXor);
    DBGVLN("MM %u=%u {}", radio.RXdataBuffer[6], UID[5], modelMatched);

    if connectionState == ConnectionState::disconnected
        || unsafe { NonceRX } != radio.RXdataBuffer[2]
        || FHSSgetCurrIndex() != radio.RXdataBuffer[1]
        || connectionHasModelMatch != modelMatched
    {
        //println!("\r\n%ux%ux%u", NonceRX, Radio.RXdataBuffer[2], Radio.RXdataBuffer[1]);
        FHSSsetCurrIndex(radio.RXdataBuffer[1]);
        unsafe { NonceRX = radio.RXdataBuffer[2] };
        TentativeConnection(now);
        // connectionHasModelMatch must come after TentativeConnection, which resets it
        connectionHasModelMatch = modelMatched;
        return true;
    }

    return false;
}

unsafe fn ProcessRFPacket(
    radio: &mut SX12xxDriverCommon,
    status: RxStatus,
    hwTimer: &mut Timer<TIM4>,
) {
    if status != RxStatus::Ok {
        println!("HW CRC error");
        return;
    }
    let beginProcessing = micros();
    let inCRC: u16 = (((radio.RXdataBuffer[0] & 0b11111100) as u16) << 6) | radio.RXdataBuffer[7];
    let type_: u8 = radio.RXdataBuffer[0] & 0b11;

    // For smHybrid the CRC only has the packet type in byte 0
    // For smHybridWide the FHSS slot is added to the CRC in byte 0 on RC_DATA_PACKETs
    if type_ != PacketHeaderType::RC_DATA_PACKET as u8
        || OtaSwitchModeCurrent != OtaSwitchMode::smHybridWide
    {
        radio.RXdataBuffer[0] = type_;
    } else {
        let NonceFHSSresult: u8 = NonceRX % ExpressLRS_currAirRate_Modparams.FHSShopInterval;
        radio.RXdataBuffer[0] = type_ | (NonceFHSSresult << 2);
    }
    let calculatedCRC: u16 = ota_crc.calc(radio.RXdataBuffer, 7, CRCInitializer);

    if inCRC != calculatedCRC {
        println!("CRC error: ");
        for i in 0..8 {
            {
                println!("{},", radio.RXdataBuffer[i]);
            }
            return;
        }
        PFDloop.extEvent(beginProcessing + PACKET_TO_TOCK_SLACK);

        let mut doStartTimer = false;
        let now = millis();

        LastValidPacket = now;

        match type_ {
            RC_DATA_PACKET => {
                // Standard RC Data  Packet
                ProcessRfPacket_RC(radio);
            }
            MSP_DATA_PACKET => {
                ProcessRfPacket_MSP();
            }
            TLM_PACKET => { //telemetry packet from master
                 // not implimented yet
            }
            SYNC_PACKET => {
                //sync packet from master
                doStartTimer = ProcessRfPacket_SYNC(now) && !InBindingMode;
            }
            _ => (),
        }

        // Store the LQ/RSSI/Antenna
        getRFlinkInfo();
        // Received a packet, that's the definition of LQ
        LQCalc.add();
        // Extend sync duration since we've received a packet at this rate
        // but do not extend it indefinitely
        RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

        if doStartTimer {
            hwTimer.enable(); // will throw an interrupt immediately
        }
    }
}

// todo: Move this call to the main fn once you figure out where it goes.
unsafe fn RXdoneISR(Radio: &mut SX12xxDriverCommon, status: RxStatus, hwTimer: &mut Timer<TIM4>) {
    ProcessRFPacket(Radio, status, hwTimer);
}

// todo: Move this call to the main fn once you figure out where it goes.
fn TXdoneISR(Radio: &mut SX12xxDriverCommon) {
    Radio.RXnb();
}

unsafe fn setupConfigAndPocCheck(delay: &mut Delay) {
    println!("Doing power-up check for loan revocation and/or re-binding");
    // Increment the power on counter in eeprom
    config.SetPowerOnCounter(config.GetPowerOnCounter() + 1);
    config.Commit();

    if config.GetPowerOnCounter() >= 3 {
        if config.GetOnLoan() {
            config.SetOnLoan(false);
            config.SetPowerOnCounter(0);
            config.Commit();
        }
    } else {
        // We haven't reached our binding mode power cycles
        // and we've been powered on for 2s, reset the power on counter
        delay.delay_ms(2_000);
        config.SetPowerOnCounter(0);
        config.Commit();
    }
}

fn setupTarget() {
    // if GPIO_PIN_ANTENNA_SELECT {
    //     pinMode(GPIO_PIN_ANTENNA_SELECT, OUTPUT);
    //     digitalWrite(GPIO_PIN_ANTENNA_SELECT, LOW);
    // }

    setupTargetCommon();
}

unsafe fn setupBindingFromConfig() {
    // Use the user defined binding phase if set,
    // otherwise use the bind flag and UID in eeprom for UID
    if config.GetOnLoan() {
        println!("RX has been loaned, reading the UID from eeprom...");
        let storedUID = config.GetOnLoanUID();
        for i in 0..UID_LEN {
            UID[i] = storedUID[i];
        }
        println!(
            "UID = {}, {}, {}, {}, {}, {}",
            UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]
        );
        CRCInitializer = (UID[4] << 8) | UID[5];
        return;
    }

    if !MY_UID {
        // Check the byte that indicates if RX has been bound
        if config.GetIsBound() {
            // println!("RX has been bound previously, reading the UID from eeprom...");
            let storedUID: [u8; 69] = config.GetUID();
            for i in 0..UID_LEN {
                {
                    UID[i] = storedUID[i];
                }
                println!(
                    "UID = {}, {}, {}, {}, {}, {}",
                    UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]
                );
                CRCInitializer = (UID[4] << 8) | UID[5];
            }
        }
    }
}

fn setupRadio(hwTimer: &mut Timer<TIM4>) {
    Radio.currFreq = GetInitialFreq();

    let init_success: bool = Radio.Begin();
    POWERMGNT.init();
    if !init_success {
        println!("Failed to detect RF chipset!!!");
        unsafe {
            connectionState = ConnectionState::radioFailed;
        }
        return;
    }

    POWERMGNT.setPower(unsafe { config.GetPower() });

    // todo
    // if Regulatory_Domain_EU_CE_2400 {
    //     LBTEnabled = MaxPower > PWR_10mW;
    // }

    unsafe {
        SetRFLinkRate(RATE_DEFAULT, hwTimer);
        RFmodeCycleMultiplier = 1;
    }
}

unsafe fn updateTelemetryBurst() {
    if telemBurstValid {
        return;
    }
    telemBurstValid = true;

    telemetryBurstMax = TLMBurstMaxForRateRatio(
        CurrAirRateModParams.rf_rate,
        CurrAirRateModParams.TLMinterval,
    );

    // Notify the sender to adjust its expected throughput
    TelemetrySender.UpdateTelemetryRate(hz as u16, ratiodiv as u8, telemetryBurstMax);
}

/// If not connected will rotate through the RF modes looking for sync
/// and blink LED
unsafe fn cycleRfMode(now: u32, hwTimer: &mut Timer<TIM4>) {
    if connectionState == ConnectionState::connected
        || connectionState == ConnectionState::wifiUpdate
        || InBindingMode
    {
        return;
    }

    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if LockRFmode == false && now - RFmodeLastCycled > cycleInterval * RFmodeCycleMultiplier as u32
    {
        RFmodeLastCycled = now;
        LastSyncPacket = now; // reset this variable
        SendLinkStatstoFCForcedSends = 2;
        SetRFLinkRate(scanIndex % RATE_MAX, hwTimer); // switch between rates
        LQCalc.reset();
        // Display the current air rate to the user as an indicator something is happening
        scanIndex += 1;
        Radio.RXnb();
        println!("{}", ExpressLRS_currAirRate_Modparams.interval);

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode
}

unsafe fn updateBindingMode(radio: &mut SX12xxDriverCommon, hwTimer: &mut Timer<TIM4>) {
    // If the eeprom is indicating that we're not bound
    // and we're not already in binding mode, enter binding
    if !config.GetIsBound() && !unsafe { InBindingMode } {
        println!("RX has not been bound, enter binding mode...");
        EnterBindingMode(radio, hwTimer);
    }
    // If in binding mode and the bind packet has come in, leave binding mode
    else if config.GetIsBound() && unsafe { InBindingMode } {
        ExitBindingMode(radio, hwtimer);
    }

    // #ifndef MY_UID
    // If the power on counter is >=3, enter binding and clear counter
    if config.GetPowerOnCounter() >= 3 {
        config.SetPowerOnCounter(0);
        config.Commit();

        println!("Power on counter >=3, enter binding mode...");
        EnterBindingMode(radio, hwTimer);
    }
    // #endif
}

unsafe fn checkSendLinkStatsToFc(now: u32) {
    if now - SendLinkStatstoFCintervalLastSent > SEND_LINK_STATS_TO_FC_INTERVAL as u32 {
        if connectionState == ConnectionState::disconnected {
            getRFlinkInfo();
        }

        if (connectionState != ConnectionState::disconnected && connectionHasModelMatch)
            || SendLinkStatstoFCForcedSends != 0
        {
            // crsf.sendLinkStatisticsToFC();
            SendLinkStatstoFCintervalLastSent = now;
            if SendLinkStatstoFCForcedSends != 0 {
                SendLinkStatstoFCForcedSends -= 1;
            }
        }
    }
}

fn setup(hwTimer: &mut Timer<TIM4>, delay: &mut Delay) {
    setupTarget();

    // Init EEPROM and load config, checking powerup count
    unsafe {
        setupConfigAndPocCheck(delay);
    }

    println!("ExpressLRS Module Booting...");

    devicesRegister(ui_devices, ARRAY_SIZE(ui_devices));
    devicesInit();

    unsafe {
        setupBindingFromConfig();
    }

    unsafe {
        fhss::randomiseFHSSsequence(uidMacSeedGet());
    }

    setupRadio(hwTimer);

    if unsafe { connectionState } != ConnectionState::radioFailed {
        Radio.RXnb();
        // crsf.Begin();
        hwTimer.init();
    }

    devicesStart();
}

unsafe fn loop_(radio: &mut SX12xxDriverCommon, hwTimer: &mut Timer<TIM4>) {
    let now: u32 = millis();

    devicesUpdate(now);

    if config.IsModified() && unsafe { !InBindingMode } {
        Radio.SetTxIdleMode();
        LostConnection(radio, hwTimer);
        config.Commit();
        devicesTriggerEvent();
    }

    if connectionState as u8 > ConnectionState::MODE_STATES as u8 {
        return;
    }

    if connectionState != ConnectionState::disconnected
        && ExpressLRS_currAirRate_Modparams.index != unsafe { nextAirRateIndex }
    {
        // forced change
        println!(
            "Req air rate change %u->%u",
            ExpressLRS_currAirRate_Modparams.index,
            unsafe { nextAirRateIndex }
        );
        LostConnection(radio, hwTimer);
        unsafe {
            LastSyncPacket = now; // reset this variable to stop rf mode switching and add extra time
            RFmodeLastCycled = now; // reset this variable to stop rf mode switching and add extra time
            SendLinkStatstoFCintervalLastSent = 0;
            SendLinkStatstoFCForcedSends = 2;
        }
    }

    if connectionState == ConnectionState::tentative
        && (now - unsafe { LastSyncPacket } > ExpressLRS_currAirRate_RFperfParams.RxLockTimeoutMs)
    {
        println!("Bad sync, aborting");
        LostConnection(radio, HwTimer);

        unsafe {
            RFmodeLastCycled = now;
            LastSyncPacket = now;
        }
    }

    unsafe {
        cycleRfMode(now, hwTimer: &mut Timer<TIM4>);
        servosUpdate(now);
    }

    let localLastValidPacket: u32 = unsafe { LastValidPacket }; // Required to prevent race condition due to LastValidPacket getting updated from ISR
    if connectionState == ConnectionState::disconnectPending
        || (connectionState == ConnectionState::connected
            && (ExpressLRS_currAirRate_RFperfParams.DisconnectTimeoutMs as i32)
                < ((now - localLastValidPacket) as i32))
    // check if we lost conn.
    {
        LostConnection(radio, hwTimer);
    }

    if connectionState == ConnectionState::tentative
        && LPF_OffsetDx.value().abs() <= 10
        && LPF_Offset.value() < 100
        && unsafe { LQCalc.getLQRaw() } > minLqForChaos()
    //detects when we are connected
    {
        GotConnection(now);
    }

    unsafe { checkSendLinkStatsToFc(now) };

    if RxTimerState == RxTimerstate::Tentative
        && now - GotConnectionMillis > ConsiderConnGoodMillis
        && OffsetDx.abs() <= 5
    {
        RxTimerState = RxTimerState::Locked;
        println!("Timer locked");
    }

    let nextPayload: u8 = 0;
    let nextPlayloadSize: u8 = 0;
    unsafe {
        if !TelemetrySender.IsActive() && telemetry.GetNextPayload(&nextPlayloadSize, &nextPayload)
        {
            TelemetrySender.SetDataToTransmit(
                nextPlayloadSize,
                nextPayload,
                ELRS_TELEMETRY_BYTES_PER_CALL,
            );
        }
    }
    unsafe { updateTelemetryBurst() };
    updateBindingMode(radio, hwTimer);
}

unsafe fn EnterBindingMode(Radio: &mut SX12xxDriverCommon, hwTimer: &mut Timer<TIM4>) {
    if connectionState == ConnectionState::connected || unsafe { InBindingMode } {
        // Don't enter binding if:
        // - we're already connected
        // - we're already binding
        println!("Cannot enter binding mode!");
        return;
    }

    // Set UID to special binding values
    UID[0] = BindingUID[0];
    UID[1] = BindingUID[1];
    UID[2] = BindingUID[2];
    UID[3] = BindingUID[3];
    UID[4] = BindingUID[4];
    UID[5] = BindingUID[5];

    CRCInitializer = 0;
    config.SetIsBound(false);
    unsafe {
        InBindingMode = true;
    }

    // Start attempting to bind
    // Lock the RF rate and freq while binding
    SetRFLinkRate(RATE_BINDING, hwTimer);
    Radio.SetFrequencyReg(GetInitialFreq());
    // If the Radio Params (including InvertIQ) parameter changed, need to restart RX to take effect
    Radio.RXnb();

    println!("Entered binding mode at freq = {}", Radio.currFreq);
    devicesTriggerEvent();
}

unsafe fn ExitBindingMode(radio: &mut SX12xxDriverCommon, hwTimer: &mut Timer<TIM4>) {
    if !unsafe { InBindingMode } {
        // Not in binding mode
        println!("Cannot exit binding mode, not in binding mode!");
        return;
    }

    // Prevent any new packets from coming in
    radio.SetTxIdleMode();
    LostConnection(radio, hwTimer);
    // Write the values to eeprom
    config.Commit();

    CRCInitializer = (UID[4] << 8) | UID[5];
    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    // Force RF cycling to start at the beginning immediately
    scanIndex = RATE_MAX;
    unsafe { RFmodeLastCycled = 0 };

    // Do this last as LostConnection() will wait for a tock that never comes
    // if we're in binding mode
    unsafe { InBindingMode = false };
    println!("Exiting binding mode");
    devicesTriggerEvent();
}

fn UpdateModelMatch(model: u8) {
    println!("Set ModelId={}", model);

    unsafe {
        config.SetModelId(model);
        config.Commit();
    }
    // This will be called from ProcessRFPacket(), schedule a disconnect
    // in the main loop once the ISR has exited
    unsafe {
        connectionState = ConnectionState::disconnectPending;
    }
}
