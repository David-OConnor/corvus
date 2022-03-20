#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/src/rx_main.cpp

use defmt::println;

///LUA///
const LUA_MAX_PARAMS: u32 =  32;
////

//// CONSTANTS ////
const SEND_LINK_STATS_TO_FC_INTERVAL: u8 = 100;
const DIVERSITY_ANTENNA_INTERVAL: i32 = 5;
const DIVERSITY_ANTENNA_RSSI_TRIGGER: u8 = 5;
const PACKET_TO_TOCK_SLACK: u8 = 200; // Desired buffer time between Packet ISR and Tock ISR
///////////////////

device_affinity_t ui_devices[] = {
#ifdef HAS_LED
{&LED_device, 0},
#endif
{&LUA_device,0},
#ifdef HAS_RGB
{&RGB_device, 0},
#endif
#ifdef HAS_WIFI
{&WIFI_device, 0},
#endif
#ifdef HAS_BUTTON
{&Button_device, 0},
#endif
#ifdef HAS_VTX_SPI
{&VTxSPI_device, 0},
#endif
};

static mut antenna: u8 = 0;    // which antenna is currently in use

hwTimer hwTimer;
POWERMGNT POWERMGNT;
PFD PFDloop;
GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);
ELRS_EEPROM eeprom;
RxConfig config;
Telemetry telemetry;

#ifdef PLATFORM_ESP8266
unsigned long rebootTime = 0;
extern bool webserverPreventAutoStart;
#endif

// #if defined(GPIO_PIN_PWM_OUTPUTS)
static mut SERVO_PINS: [u8; 69] = GPIO_PIN_PWM_OUTPUTS;
static mut SERVO_COUNT: [u8; 69] = ARRAY_SIZE(SERVO_PINS);
static mut Servo *Servos[SERVO_COUNT];
static mut newChannelsAvailable: bool = false;
// #endif

// StubbornSender TelemetrySender(ELRS_TELEMETRY_MAX_PACKAGES);
static mut telemetryBurstCount: u8 = 0;
static mut telemetryBurstMax: u8 = 0;
// Maximum ms between LINK_STATISTICS packets for determining burst max
static mut TELEM_MIN_LINK_INTERVAL: u16 = 512;

// StubbornReceiver MspReceiver(ELRS_MSP_MAX_PACKAGES);
const MspData: [u8; ELRS_MSP_BUFFER] = [0; ELRS_MSP_BUFFER];

static mut NextTelemetryType: u8 = ELRS_TELEMETRY_TYPE_LINK;
static mut telemBurstValid: bool = false;
/// Filters ////////////////
// LPF LPF_Offset(2);
// LPF LPF_OffsetDx(4);

// LPF LPF_UplinkRSSI(5);
// LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
// LPF LPF_UplinkRSSI1(5);


/// LQ Calculation //////////
LQCALC<100> LQCalc;
static mut uplinkLQ: u8 = 0;

const scanIndex: u8 = RATE_DEFAULT;
const ExpressLRS_nextAirRateIndex: u8 = 0;

// todo: Should these be consts, or static mut? I think the last is const?
const RawOffset: i32 = 0;
const prevRawOffset: i32 = 0;
const Offset: i32 = 0;
const  OffsetDx: i32 = 0;
const prevOffset: i32 = 0;
// RXtimerState_e RXtimerState;
const GotConnectionMillis: u32 = 0;
const ConsiderConnGoodMillis: u32 = 1000; // minimum time before we can consider a connection to be 'good'

///////////////////////////////////////////////

static mut NonceRX: u8 = 0; // nonce that we THINK we are up to.

static mut alreadyFHSS: bool = false;
static mut alreadyTLMresp: bool = false;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
static mut LastValidPacket: u32 = 0;           //Time the last valid packet was recv
static mut LastSyncPacket: u32 = 0;            //Time the last valid packet was recv

static mut SendLinkStatstoFCintervalLastSent: u32 = 0;
static mut SendLinkStatstoFCForcedSends: u8 = 0;

static mut RFnoiseFloor: i16 = 0; //measurement of the current RF noise floor
static mut lastPacketCrcError: bool = false;
///////////////////////////////////////////////////////////////

/// Variables for Sync Behaviour ////
static mut cycleInterval: u32 = 0; // in ms
static mut RFmodeLastCycled: u32 = 0;
static mut RFmodeCycleMultiplierSlow: u8 = 10;
static mut RFmodeCycleMultiplier: u8 = 0;
static mut LockRFmode: bool = false;
///////////////////////////////////////

// #if defined(DEBUG_BF_LINK_STATS)
// Debug vars
static mut debug1: u8 = 0;
static mut debug2: u8 = 0;
static mut debug3: u8 = 0;
static mut debug4: i8 = 0;
///////////////////////////////////////
// #endif

static mut InBindingMode: bool = false;

// ICAC
// HE_RAM_ATTR
fn IsArmed() -> bool
{
    return CRSF_to_BIT(crsf.GetChannelOutput(AUX1));
}

fn minLqForChaos() -> u8
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
    let  numfhss: u32 = FHSSgetChannelCount();
    let interval: u8 = ExpressLRS_currAirRate_Modparams.FHSShopInterval;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

// ICACHE_RAM_ATTR
unsafe fn getRFlinkInfo()
{
    let mut rssiDBM0: i32 = LPF_UplinkRSSI0.SmoothDataINT;
    let mut rssiDBM1: i32 = LPF_UplinkRSSI1.SmoothDataINT;
    match antenna {
        0 => {
            rssiDBM0 = LPF_UplinkRSSI0.update(Radio.LastPacketRSSI);
        }
        1 => {
            rssiDBM1 = LPF_UplinkRSSI1.update(Radio.LastPacketRSSI);
        }
        _ => (),
    }

    let rssiDBM: i32 = if antenna == 0 { rssiDBM0 } else { rssiDBM1 };
    crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(constrain(rssiDBM, ExpressLRS_currAirRate_RFperfParams.RXsensitivity, -50),
                                                   ExpressLRS_currAirRate_RFperfParams.RXsensitivity, -50, 0, 1023));
    crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(uplinkLQ, 0, 100, 0, 1023));

    if rssiDBM0 > 0 {
        rssiDBM0 = 0;
    }
    if rssiDBM1 > 0 {
        rssiDBM1 = 0;
    }

    // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
    crsf.LinkStatistics.uplink_RSSI_1 = -rssiDBM0;
    crsf.LinkStatistics.active_antenna = antenna;
    crsf.LinkStatistics.uplink_SNR = Radio.LastPacketSNR;
    //crsf.LinkStatistics.uplink_Link_quality = uplinkLQ; // handled in Tick
    crsf.LinkStatistics.rf_Mode = ExpressLRS_currAirRate_Modparams.enum_rate;
    //println!(crsf.LinkStatistics.uplink_RSSI_1);
    if DEBUG_BF_LINK_STATS {
        crsf.LinkStatistics.downlink_RSSI = debug1;
        crsf.LinkStatistics.downlink_Link_quality = debug2;
        crsf.LinkStatistics.downlink_SNR = debug3;
        crsf.LinkStatistics.uplink_RSSI_2 = debug4;
    } else {
        crsf.LinkStatistics.downlink_RSSI = 0;
        crsf.LinkStatistics.downlink_Link_quality = 0;
        crsf.LinkStatistics.downlink_SNR = 0;
        crsf.LinkStatistics.uplink_RSSI_2 = -rssiDBM1;
    }
}

fn SetRFLinkRate(index: u8) // Set speed of RF link
{
    let ModParams: ModSettings = get_elrs_airRateConfig(index);
    let RFperf: RfPrefParams = get_elrs_RFperfParams(index);
    let invertIQ: bool = UID[5] & 0x01;

    hwTimer.updateInterval(ModParams.interval);
    Radio.Config(ModParams.bw, ModParams.sf, ModParams.cr, GetInitialFreq(),
                 ModParams.PreambleLen, invertIQ, ModParams.PayloadLength, 0
// if defined(RADIO_SX128X)
                 , uidMacSeedGet(), CRCInitializer, (ModParams.radio_type == RADIO_TYPE_SX128x_FLRC)
// #endif
    );

    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    unsafe {
        cycleInterval = (11 * FHSSgetChannelCount() * ModParams.FHSShopInterval * ModParams.interval) / (10 * 1000);
    }

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    ExpressLRS_nextAirRateIndex = index; // presumably we just handled this
    unsafe { telemBurstValid = false; }
}

// ICACHE_RAM_ATTR
unsafe fn  HandleFHSS() -> bool
{
    let modresultFHSS: u8 = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams.FHSShopInterval;

    if ((ExpressLRS_currAirRate_Modparams.FHSShopInterval == 0) || alreadyFHSS == true || InBindingMode || (modresultFHSS != 0) || (connectionState == disconnected))
    {
        return false;
    }

    alreadyFHSS = true;
    Radio.SetFrequencyReg(FHSSgetNextFreq());

    let  modresultTLM: u8 = (NonceRX + 1) % (TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval));

    if modresultTLM != 0 || ExpressLRS_currAirRate_Modparams.TLMinterval == TLM_RATIO_NO_TLM // if we are about to send a tlm response don't bother going back to rx
    {
        Radio.RXnb();
    }
    return true;
}

// ICACHE_RAM_ATTR
unsafe fn HandleSendTelemetryResponse() -> bool
{
    let mut data: [u8; 69] = [0; 69];
    let mut maxLength: u8 = 0;
    let mut packageIndex = 0;
    let modresult = (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval);

    if ((connectionState == disconnected) || (ExpressLRS_currAirRate_Modparams.TLMinterval == TLM_RATIO_NO_TLM) || (alreadyTLMresp == true) || (modresult != 0))
    {
        return false; // don't bother sending tlm if disconnected or TLM is off
    }

    if Regulatory_Domain_EU_CE_2400 {
        BeginClearChannelAssessment();
    }

    alreadyTLMresp = true;
    Radio.TXdataBuffer[0] = TLM_PACKET;

    if (NextTelemetryType == ELRS_TELEMETRY_TYPE_LINK || !TelemetrySender.IsActive())
    {
        Radio.TXdataBuffer[1] = ELRS_TELEMETRY_TYPE_LINK;
        // The value in linkstatistics is "positivized" (inverted polarity)
        // and must be inverted on the TX side. Positive values are used
        // so save a bit to encode which antenna is in use
        Radio.TXdataBuffer[2] = crsf.LinkStatistics.uplink_RSSI_1 | (antenna << 7);
        Radio.TXdataBuffer[3] = crsf.LinkStatistics.uplink_RSSI_2 | (connectionHasModelMatch << 7);
        Radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
        Radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;
        Radio.TXdataBuffer[6] = if MspReceiver.GetCurrentConfirm() { 1 } else { 0 };

        NextTelemetryType = ELRS_TELEMETRY_TYPE_DATA;
        // Start the count at 1 because the next will be DATA and doing +1 before checking
        // against Max below is for some reason 10 bytes more code
        telemetryBurstCount = 1;
    }
    else
    {
        if telemetryBurstCount < telemetryBurstMax
        {
            telemetryBurstCount += 1;
        }
        else
        {
            NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
        }

        TelemetrySender.GetCurrentPayload(&packageIndex, &maxLength, &data);
        Radio.TXdataBuffer[1] = (packageIndex << ELRS_TELEMETRY_SHIFT) + ELRS_TELEMETRY_TYPE_DATA;
        Radio.TXdataBuffer[2] = if maxLength > 0 { *data } else { 0 };
        Radio.TXdataBuffer[3] = if maxLength >= 1 { *(data + 1) } else { 0};
        Radio.TXdataBuffer[4] = if maxLength >= 2 { *(data + 2) } else {  0};
        Radio.TXdataBuffer[5] = if maxLength >= 3 { *(data + 3)} else {  0};
        Radio.TXdataBuffer[6] = if maxLength >= 4 { *(data + 4)} else {  0};
    }

    let crc: u16 = ota_crc.calc(Radio.TXdataBuffer, 7, CRCInitializer);
    Radio.TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    Radio.TXdataBuffer[7] = crc & 0xFF;

    if !Regulatory_Domain_EU_CE_2400 || ChannelIsClear() {
        Radio.TXnb();
    }
    true
}

// ICACHE_RAM_ATTR
fn HandleFreqCorr(value: bool)
{
    //DBGVLN(FreqCorrection);
    if !value {
        if FreqCorrection < FreqCorrectionMax   {
            FreqCorrection += 1; //min freq step is ~ 61hz but don't forget we use FREQ_HZ_TO_REG_VAL so the units here are not hz!
        }
        else
        {
            FreqCorrection = FreqCorrectionMax;
            FreqCorrection = 0; //reset because something went wrong
            println!("Max +FreqCorrection reached!");
        }
    }
    else
    {
        if FreqCorrection > FreqCorrectionMin
        {
            FreqCorrection -= 1; //min freq step is ~ 61hz
        }
        else
        {
            FreqCorrection = FreqCorrectionMin;
            FreqCorrection = 0; //reset because something went wrong
            println!("Max -FreqCorrection reached!");
        }
    }
}

// ICACHE_RAM_ATTR
fn updatePhaseLock()
{
    if connectionState != disconnected
    {
        PFDloop.calcResult();
        PFDloop.reset();
        RawOffset = PFDloop.getResult();
        Offset = LPF_Offset.update(RawOffset);
        OffsetDx = LPF_OffsetDx.update(RawOffset - prevRawOffset);

        if RXtimerState == tim_locked && LQCalc.currentIsSet()
        {
            if unsafe {NonceRX % 8 == 0} //limit rate of freq offset adjustment slightly
            {
                if Offset > 0
                {
                    hwTimer.incFreqOffset();
                }
                else if Offset < 0
                {
                    hwTimer.decFreqOffset();
                }
            }
        }

        if connectionState != connected
        {
            hwTimer.phaseShift(RawOffset >> 1);
        }
        else
        {
            hwTimer.phaseShift(Offset >> 2);
        }

        prevOffset = Offset;
        prevRawOffset = RawOffset;
    }

    DBGVLN("%d:%d:%d:%d:%d", Offset, RawOffset, OffsetDx, hwTimer.FreqOffset, unsafe { uplinkLQ });
}

// ICACHE_RAM_ATTR
unsafe fn HWtimerCallbackTick() // this is 180 out of phase with the other callback, occurs mid-packet reception
{
    updatePhaseLock();
    NonceRX+= 1;

    // if (!alreadyTLMresp && !alreadyFHSS && !LQCalc.currentIsSet()) // packet timeout AND didn't DIDN'T just hop or send TLM
    // {
    //     Radio.RXnb(); // put the radio cleanly back into RX in case of garbage data
    // }

    // Save the LQ value before the inc() reduces it by 1
    uplinkLQ = LQCalc.getLQ();
    crsf.LinkStatistics.uplink_Link_quality = uplinkLQ;
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if !alreadyTLMresp {
        LQCalc.inc();
    }

    alreadyTLMresp = false;
    alreadyFHSS = false;
    crsf.RXhandleUARTout();
}

//////////////////////////////////////////////////////////////
// flip to the other antenna
// no-op if GPIO_PIN_ANTENNA_SELECT not defined
#[inline(always)]
unsafe fn switchAntenna()
{
    if GPIO_PIN_ANTENNA_SELECT && USE_DIVERSITY {
        if config.GetAntennaMode() == 2 {
            //0 and 1 is use for gpio_antenna_select
            // 2 is diversity
            antenna = !antenna;
            if antenna == 0 { LPF_UplinkRSSI0.reset() } else { LPF_UplinkRSSI1.reset() }; // discard the outdated value after switching
            digitalWrite(GPIO_PIN_ANTENNA_SELECT, antenna);
        }
    }
}

// from updateDiversity:
static mut prevRSSI: i32 = 0;        // saved rssi so that we can compare if switching made things better or worse
static mut antennaLQDropTrigger: i32 = 0;
static mut antennaRSSIDropTrigger: i32 = 0;

// ICACHE_RAM_ATTR
unsafe fn updateDiversity()
{

    if GPIO_PIN_ANTENNA_SELECT && USE_DIVERSITY {
        if config.GetAntennaMode() == 2 {
            //0 and 1 is use for gpio_antenna_select
            // 2 is diversity

            let rssi: i32 = if antenna == 0 { LPF_UplinkRSSI0.SmoothDataINT } else { LPF_UplinkRSSI1.SmoothDataINT };
            let otherRSSI: i32 = if antenna == 0 { LPF_UplinkRSSI1.SmoothDataINT } else { LPF_UplinkRSSI0.SmoothDataINT };

            //if rssi dropped by the amount of DIVERSITY_ANTENNA_RSSI_TRIGGER
            if (rssi < (prevRSSI - DIVERSITY_ANTENNA_RSSI_TRIGGER)) && antennaRSSIDropTrigger >= DIVERSITY_ANTENNA_INTERVAL
            {
                switchAntenna();
                antennaLQDropTrigger = 1;
                antennaRSSIDropTrigger = 0;
            } else if (rssi > prevRSSI || antennaRSSIDropTrigger < DIVERSITY_ANTENNA_INTERVAL)
            {
                prevRSSI = rssi;
                antennaRSSIDropTrigger += 1;
            }

            // if we didn't get a packet switch the antenna
            if (!LQCalc.currentIsSet() && antennaLQDropTrigger == 0)
            {
                switchAntenna();
                antennaLQDropTrigger = 1;
                antennaRSSIDropTrigger = 0;
            } else if (antennaLQDropTrigger >= DIVERSITY_ANTENNA_INTERVAL)
            {
                // We switched antenna on the previous packet, so we now have relatively fresh rssi info for both antennas.
                // We can compare the rssi values and see if we made things better or worse when we switched
                if (rssi < otherRSSI)
                {
                    // things got worse when we switched, so change back.
                    switchAntenna();
                    antennaLQDropTrigger = 1;
                    antennaRSSIDropTrigger = 0;
                } else {
                    // all good, we can stay on the current antenna. Clear the flag.
                    antennaLQDropTrigger = 0;
                }
            } else if (antennaLQDropTrigger > 0)
            {
                antennaLQDropTrigger += 1;
            }
        } else {
            digitalWrite(GPIO_PIN_ANTENNA_SELECT, config.GetAntennaMode());
            antenna = config.GetAntennaMode();
        }
    }
}

// ICACHE_RAM_ATTR
unsafe fn HWtimerCallbackTock()
{
    if Regulatory_Domain_EU_CE_2400 {
        // Emulate that TX just happened, even if it didn't because channel is not clear
        if (!LBTSuccessCalc.currentIsSet())
        {
            Radio.TXdoneCallback();
        }
    }

    PFDloop.intEvent(micros()); // our internal osc just fired

    updateDiversity();
    let didFHSS: bool = HandleFHSS();
    let tlmSent: bool = HandleSendTelemetryResponse();

    if DEBUG_RX_SCOREBOARD {
        let mut lastPacketWasTelemetry = false;
        if !LQCalc.currentIsSet() && !lastPacketWasTelemetry {
            defmt::println!(if lastPacketCrcError { '.' } else { '_' });
        }
        lastPacketCrcError = false;
        lastPacketWasTelemetry = tlmSent;
    }
}

fn LostConnection()
{
    defmt::println!("lost conn fc={} fo={}", FreqCorrection, hwTimer.FreqOffset);

    unsafe { RFmodeCycleMultiplier = 1; }
    connectionState = disconnected; //set lost connection
    RXtimerState = tim_disconnected;
    hwTimer.resetFreqOffset();
    FreqCorrection = 0;
    Offset = 0;
    OffsetDx = 0;
    RawOffset = 0;
    prevOffset = 0;
    GotConnectionMillis = 0;
    unsafe { uplinkLQ = 0 };
    LQCalc.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    unsafe {
        alreadyTLMresp = false;
        alreadyFHSS = false;
    }

    if unsafe { !InBindingMode } {
        while micros() - PFDloop.getIntEventTime() > 250 {}; // time it just after the tock()
        hwTimer.stop();
        SetRFLinkRate(ExpressLRS_nextAirRateIndex); // also sets to initialFreq
        Radio.RXnb();
    }
}

// ICACHE_RAM_ATTR
fn TentativeConnection(now: u64)
{
    PFDloop.reset();
    connectionState = tentative;
    connectionHasModelMatch = false;
    RXtimerState = tim_disconnected;
    println!("tentative conn");
    FreqCorrection = 0;
    Offset = 0;
    prevOffset = 0;
    LPF_Offset.init(0);
    unsafe { RFmodeLastCycled = now as u32; } // give another 3 sec for lock to occur

    // The caller MUST call hwTimer.resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}

fn GotConnection(now: u64)
{
    if (connectionState == connected)
    {
        return; // Already connected
    }

    if LOCK_ON_FIRST_CONNECTION {
        unsafe { LockRFmode = true };
    }

    connectionState = connected; //we got a packet, therefore no lost connection
    RXtimerState = tim_tentative;
    GotConnectionMillis = unsafe { now as u32 };

    println!("got conn");
}

// ICACHE_RAM_ATTR
fn ProcessRfPacket_RC()
{
    // Must be fully connected to process RC packets, prevents processing RC
    // during sync, where packets can be received before connection
    if connectionState != connected {
        return;
    }

    let telemetryConfirmValue: bool = UnpackChannelData(Radio.RXdataBuffer, &crsf,
                                                        unsafe { NonceRX }, TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval));
    TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);

    // No channels packets to the FC if no model match
    if connectionHasModelMatch {
        if GPIO_PIN_PWM_OUTPUTS {
            unsafe { newChannelsAvailable = true; }
        } else {
            crsf.sendRCFrameToFC();
        }
    }
}

/**
 * Process the assembled MSP packet in MspData[]
 **/
// ICACHE_RAM_ATTR
fn MspReceiveComplete()
{
    if MspData[7] == MSP_SET_RX_CONFIG && MspData[8] == MSP_ELRS_MODEL_ID
    {
        UpdateModelMatch(MspData[9]);
    }
    else if MspData[0] == MSP_ELRS_SET_RX_WIFI_MODE
    {

    }
// if HAS_VTX_SPI {
//     else if (MspData[7] == MSP_SET_VTX_CONFIG)
//     {
//         vtxSPIBandChannelIdx = MspData[8];
//         if (MspData[6] >= 4) // If packet has 4 bytes it also contains power idx and pitmode.
//         {
//             vtxSPIPowerIdx = MspData[10];
//             vtxSPIPitmode = MspData[11];
//         }
//         devicesTriggerEvent();
//     }
// }
    else
    {
        // No MSP data to the FC if no model match
        if connectionHasModelMatch {
            let receivedHeader: CrsfExtHeader = MspData as CrsfExtHeader;
            if receivedHeader.dest_addr == CRSF_ADDRESS_BROADCAST || receivedHeader.dest_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER
            {
                crsf.sendMSPFrameToFC(MspData);
            }

            if ((receivedHeader.dest_addr == CRSF_ADDRESS_BROADCAST || receivedHeader.dest_addr == CRSF_ADDRESS_CRSF_RECEIVER))
            {
                crsf.ParameterUpdateData[0] = MspData[CRSF_TELEMETRY_TYPE_INDEX];
                crsf.ParameterUpdateData[1] = MspData[CRSF_TELEMETRY_FIELD_ID_INDEX];
                crsf.ParameterUpdateData[2] = MspData[CRSF_TELEMETRY_FIELD_CHUNK_INDEX];
                luaParamUpdateReq();
            }
        }
    }

    MspReceiver.Unlock();
}

// ICACHE_RAM_ATTR
fn ProcessRfPacket_MSP()
{
    // Always examine MSP packets for bind information if in bind mode
    // [1] is the package index, first packet of the MSP
    if unsafe { InBindingMode } && Radio.RXdataBuffer[1] == 1 && Radio.RXdataBuffer[2] == MSP_ELRS_BIND
    {
        OnELRSBindMSP(&Radio.RXdataBuffer[2] as u8);
        return;
    }

    // Must be fully connected to process MSP, prevents processing MSP
    // during sync, where packets can be received before connection
    if connectionState != connected {
        return;
    }

    let currentMspConfirmValue: bool = MspReceiver.GetCurrentConfirm();
    MspReceiver.ReceiveData(Radio.RXdataBuffer[1], Radio.RXdataBuffer + 2);
    if currentMspConfirmValue != MspReceiver.GetCurrentConfirm()
    {
        unsafe { NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK; }
    }
    if MspReceiver.HasFinishedData() {
        MspReceiveComplete();
    }
}

// ICACHE_RAM_ATTR
fn  ProcessRfPacket_SYNC(now: u64) -> bool
{
    // Verify the first two of three bytes of the binding ID, which should always match
    if Radio.RXdataBuffer[4] != UID[3] || Radio.RXdataBuffer[5] != UID[4] {
        return false;
    }

    // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
    // Only require the first 18 bits of the UID to match to establish a connection
    // but the last 6 bits must modelmatch before sending any data to the FC
    if (Radio.RXdataBuffer[6] & !MODELMATCH_MASK) != (UID[5] & !MODELMATCH_MASK) {
        return false;
    }

    unsafe { LastSyncPacket = now as u32; }
    if DEBUG_RX_SCOREBOARD {
        println!("s");
    }

    // Will change the packet air rate in loop() if this changes
    ExpressLRS_nextAirRateIndex = (Radio.RXdataBuffer[3] >> SYNC_PACKET_RATE_OFFSET) & SYNC_PACKET_RATE_MASK;
    // Update switch mode encoding immediately
    OtaSetSwitchMode((OtaSwitchMode_e)((Radio.RXdataBuffer[3] >> SYNC_PACKET_SWITCH_OFFSET) & SYNC_PACKET_SWITCH_MASK));
    // Update TLM ratio
    let TLMrateIn: TlmRatio = (expresslrs_tlm_ratio_e)((Radio.RXdataBuffer[3] >> SYNC_PACKET_TLM_OFFSET) & SYNC_PACKET_TLM_MASK);
    if (ExpressLRS_currAirRate_Modparams.TLMinterval != TLMrateIn)
    {
        println!("New TLMrate: %d", TLMrateIn);
        ExpressLRS_currAirRate_Modparams.TLMinterval = TLMrateIn;
        unsafe { telemBurstValid = false; }
    }

    // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    let modelXor: u8 = (!config.GetModelId()) & MODELMATCH_MASK;
    let modelMatched: bool = Radio.RXdataBuffer[6] == (UID[5] ^ modelXor);
    DBGVLN("MM %u=%u %d", Radio.RXdataBuffer[6], UID[5], modelMatched);

    if connectionState == disconnected
        || unsafe { NonceRX } != Radio.RXdataBuffer[2]
        || FHSSgetCurrIndex() != Radio.RXdataBuffer[1]
        || connectionHasModelMatch != modelMatched
    {
        //println!("\r\n%ux%ux%u", NonceRX, Radio.RXdataBuffer[2], Radio.RXdataBuffer[1]);
        FHSSsetCurrIndex(Radio.RXdataBuffer[1]);
        unsafe { NonceRX } = Radio.RXdataBuffer[2];
        TentativeConnection(now);
        // connectionHasModelMatch must come after TentativeConnection, which resets it
        connectionHasModelMatch = modelMatched;
        return true;
    }

    return false;
}

// ICACHE_RAM_ATTR
unsafe fn ProcessRFPacket(status: SX12xxDriverCommon::rx_status) {
    if status != SX12xxDriverCommon::SX12XX_RX_OK {
        defmt::println!("HW CRC error");
        if DEBUG_RX_SCOREBOARD {
            lastPacketCrcError = true;
        }
        return;
    }
    let beginProcessing = micros();
    let inCRC: u16 = (((Radio.RXdataBuffer[0] & 0b11111100) as u16) << 6) | Radio.RXdataBuffer[7];
    let type_: u8 = Radio.RXdataBuffer[0] & 0b11;

    // For smHybrid the CRC only has the packet type in byte 0
    // For smHybridWide the FHSS slot is added to the CRC in byte 0 on RC_DATA_PACKETs
    if type_ != RC_DATA_PACKET || OtaSwitchModeCurrent != smHybridWide {
        Radio.RXdataBuffer[0] = type_;
    } else {
        let NonceFHSSresult: u8 = NonceRX % ExpressLRS_currAirRate_Modparams.FHSShopInterval;
        Radio.RXdataBuffer[0] = type_ | (NonceFHSSresult << 2);
    }
    let calculatedCRC: u16 = ota_crc.calc(Radio.RXdataBuffer, 7, CRCInitializer);

    if inCRC != calculatedCRC
    {
        println!("CRC error: ");
        for i in 0..8 {
            {
                println!("{},", Radio.RXdataBuffer[i]);
            }
            DBGVCR;
            if DEBUG_RX_SCOREBOARD {
                lastPacketCrcError = true;
            }
            return;
        }
        PFDloop.extEvent(beginProcessing + PACKET_TO_TOCK_SLACK);

        let mut doStartTimer = false;
        let now = millis();

        LastValidPacket = now;

        match type_ {
            RC_DATA_PACKET => { // Standard RC Data  Packet
                ProcessRfPacket_RC();
            }
            MSP_DATA_PACKET => {
                ProcessRfPacket_MSP();
            }
            TLM_PACKET => { //telemetry packet from master
                // not implimented yet
            }
            SYNC_PACKET => { //sync packet from master
                doStartTimer = ProcessRfPacket_SYNC(now) && !InBindingMode;
            }
            _ => ()
        }

        // Store the LQ/RSSI/Antenna
        getRFlinkInfo();
        // Received a packet, that's the definition of LQ
        LQCalc.add();
        // Extend sync duration since we've received a packet at this rate
        // but do not extend it indefinitely
        RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

        if DEBUG_RX_SCOREBOARD {
            if type_ != SYNC_PACKET {
                println!(connectionHasModelMatch? 'R': 'r');
            }
        }
        if doStartTimer {
            hwTimer.resume(); // will throw an interrupt immediately
        }
    }
}

// ICACHE_RAM_ATTR
unsafe fn RXdoneISR(status: SX12xxDriverCommon::rx_status )
{
    ProcessRFPacket(status);
}

// ICACHE_RAM_ATTR
fn  TXdoneISR()
{
    Radio.RXnb();
}


fn setupConfigAndPocCheck()
{
    eeprom.Begin();
    config.SetStorageProvider(&eeprom); // Pass pointer to the Config class for access to storage
    config.Load();

    println!("ModelId=%u", config.GetModelId());

// #ifndef MY_UID
    // Increment the power on counter in eeprom
    config.SetPowerOnCounter(config.GetPowerOnCounter() + 1);
    config.Commit();

    // If we haven't reached our binding mode power cycles
    // and we've been powered on for 2s, reset the power on counter
    if config.GetPowerOnCounter() < 3 {
        delay(2000);
        config.SetPowerOnCounter(0);
        config.Commit();
    }
// #endif
}

fn setupTarget() {
    if GPIO_PIN_ANTENNA_SELECT {
        pinMode(GPIO_PIN_ANTENNA_SELECT, OUTPUT);
        digitalWrite(GPIO_PIN_ANTENNA_SELECT, LOW);
    }

    setupTargetCommon();
}

fn setupBindingFromConfig()
{
// Use the user defined binding phase if set,
// otherwise use the bind flag and UID in eeprom for UID
    if MY_UID {
        // Check the byte that indicates if RX has been bound
        if config.GetIsBound() {
            println!("RX has been bound previously, reading the UID from eeprom...");
            let storedUID: [u8; 69] = config.GetUID();
            for i in 0..UID_LEN {
                {
                    UID[i] = storedUID[i];
                }
                println!("UID = %d, %d, %d, %d, %d, %d", UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);
                CRCInitializer = (UID[4] << 8) | UID[5];
            }
        }
    }
}

fn setupRadio() {
    Radio.currFreq = GetInitialFreq();

    let init_success: bool = Radio.Begin();
    POWERMGNT.init();
    if !init_success {
        println!("Failed to detect RF chipset!!!");
        connectionState = radioFailed;
        return;
    }

    POWERMGNT.setPower(config.GetPower());

    if Regulatory_Domain_EU_CE_2400 {
        LBTEnabled = (MaxPower > PWR_10mW);
    }

    Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;

    SetRFLinkRate(RATE_DEFAULT);
    unsafe { RFmodeCycleMultiplier = 1; }
}

unsafe fn updateTelemetryBurst()
{
    if telemBurstValid {
        return;
    }
    telemBurstValid = true;

    let hz: u32 = RateEnumToHz(ExpressLRS_currAirRate_Modparams.enum_rate);
    let ratiodiv: u32 = TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams.TLMinterval);
    // telemInterval = 1000 / (hz / ratiodiv);
    // burst = TELEM_MIN_LINK_INTERVAL / telemInterval;
    // This ^^^ rearranged to preserve precision vvv
    telemetryBurstMax = TELEM_MIN_LINK_INTERVAL as u8 * hz / ratiodiv / 1000;

    // Reserve one slot for LINK telemetry
    if telemetryBurstMax > 1 {
        telemetryBurstMax -= 1;
    } else {
        telemetryBurstMax = 1;
    }
    //println!("TLMburst: %d", telemetryBurstMax);

    // Notify the sender to adjust its expected throughput
    TelemetrySender.UpdateTelemetryRate(hz, ratiodiv, telemetryBurstMax);
}

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
unsafe fn cycleRfMode(now: u64)
{
    if connectionState == connected || connectionState == wifiUpdate || InBindingMode {
        return;
    }

    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if LockRFmode == false && (now as u32 - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier)
    {
        RFmodeLastCycled = now as u32;
        LastSyncPacket = now as u32;           // reset this variable
        SendLinkStatstoFCForcedSends = 2;
        SetRFLinkRate(scanIndex % RATE_MAX); // switch between rates
        LQCalc.reset();
        // Display the current air rate to the user as an indicator something is happening
        scanIndex += 1;
        Radio.RXnb();
        INFOLN("%u", ExpressLRS_currAirRate_Modparams,interval);

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode
}

unsafe fn servosUpdate(now: u64) {
    if GPIO_PIN_PWM_OUTPUTS {
        // The ESP waveform generator is nice because it doesn't change the value
        // mid-cycle, but it does busywait if there's already a change queued.
        // Updating every 20ms minimizes the amount of waiting (0-800us cycling
        // after it syncs up) where 19ms always gets a 1000-1800us wait cycling
        let mut lastUpdate: u32 = 0;
        let elapsed = now - lastUpdate;
        if elapsed < 20 {
            return;
        }

        if (newChannelsAvailable)
        {
            newChannelsAvailable = false;
            for ch in 0..SERVO_COUNT {
                {
                    let chConfig: RxConfigPwm = config.GetPwmChannel(ch);
                    let mut us: u16 = CRSF_to_US(crsf.GetChannelOutput(chConfig.val.inputChannel));
                    if chConfig.val.inverted {
                        us = 3000 - us;
                    }

                    if Servos[ch] {
                        Servos[ch].writeMicroseconds(us);
                    } else if us >= 988 && us <= 2012 {
                        // us might be out of bounds if this is a switch channel and it has not been
                        // received yet. Delay initializing the servo until the channel is valid
                        Servo * servo = Servo::new();
                        Servos[ch] = servo;
                        servo.attach(SERVO_PINS[ch], 988, 2012, us);
                    }
                } /* for each servo */
            } /* if newChannelsAvailable */
        } else if elapsed > 1000 && connectionState == connected
        {
            // No update for 1s, go to failsafe
            for i in 0..SERVO_COUNT {
                {
                    // Note: Failsafe values do not respect the inverted flag, failsafes are absolute
                    let us: u16 = config.GetPwmChannel(ch).val.failsafe + 988;
                    if Servos[ch] {
                        Servos[ch].writeMicroseconds(us);
                    }
                }
            }
        } else {
            return; // prevent updating lastUpdate
        }

        // need to sample actual millis at the end to account for any
        // waiting that happened in Servo::writeMicroseconds()
        lastUpdate = millis();
    }

    fn updateBindingMode()
    {
        // If the eeprom is indicating that we're not bound
        // and we're not already in binding mode, enter binding
        if !config.GetIsBound() && !unsafe { InBindingMode }
        {
            INFOLN("RX has not been bound, enter binding mode...");
            EnterBindingMode();
        }
        // If in binding mode and the bind packet has come in, leave binding mode
        else if config.GetIsBound() && unsafe { InBindingMode }
        {
            ExitBindingMode();
        }

// #ifndef MY_UID
        // If the power on counter is >=3, enter binding and clear counter
        if (config.GetPowerOnCounter() >= 3)
        {
            config.SetPowerOnCounter(0);
            config.Commit();

            INFOLN("Power on counter >=3, enter binding mode...");
            EnterBindingMode();
        }
// #endif
    }
}

unsafe fn checkSendLinkStatsToFc(now: u32)
{
    if now - SendLinkStatstoFCintervalLastSent > SEND_LINK_STATS_TO_FC_INTERVAL as u32 {
        if connectionState == disconnected {
            getRFlinkInfo();
        }

        if (connectionState != disconnected && connectionHasModelMatch) || SendLinkStatstoFCForcedSends != 0
        {
            crsf.sendLinkStatisticsToFC();
            SendLinkStatstoFCintervalLastSent = now;
            if SendLinkStatstoFCForcedSends {
                SendLinkStatstoFCForcedSends -= 1;
            }
        }
    }
}

fn setup()
{
    setupTarget();

    // Init EEPROM and load config, checking powerup count
    setupConfigAndPocCheck();

    INFOLN("ExpressLRS Module Booting...");

    devicesRegister(ui_devices, ARRAY_SIZE(ui_devices));
    devicesInit();

    setupBindingFromConfig();

    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    setupRadio();

    if (connectionState != radioFailed)
    {
        // RFnoiseFloor = MeasureNoiseFloor(); //TODO move MeasureNoiseFloor to driver libs
        // println!("RF noise floor: %d dBm", RFnoiseFloor);

        hwTimer.callbackTock = &HWtimerCallbackTock;
        hwTimer.callbackTick = &HWtimerCallbackTick;

        MspReceiver.SetDataToReceive(ELRS_MSP_BUFFER, MspData, ELRS_MSP_BYTES_PER_CALL);
        Radio.RXnb();
        crsf.Begin();
        hwTimer.init();
    }

    devicesStart();
}

fn loop_()
{
    now: u64 = millis();
    HandleUARTin();
    if hwTimer.running == false {
        crsf.RXhandleUARTout();
    }

    devicesUpdate(now);

    if (config.IsModified() && unsafe { !InBindingMode }) {
        Radio.SetTxIdleMode();
        LostConnection();
        config.Commit();
        devicesTriggerEvent();
    }

    if connectionState > MODE_STATES {
        return;
    }

    if (connectionState != disconnected) && (ExpressLRS_currAirRate_Modparams.index != ExpressLRS_nextAirRateIndex) { // forced change
        println ! ("Req air rate change %u->%u", ExpressLRS_currAirRate_Modparams.index, ExpressLRS_nextAirRateIndex);
        LostConnection();
        unsafe {
            LastSyncPacket = now;           // reset this variable to stop rf mode switching and add extra time
            RFmodeLastCycled = now;         // reset this variable to stop rf mode switching and add extra time
            SendLinkStatstoFCintervalLastSent = 0;
            SendLinkStatstoFCForcedSends = 2;
        }
    }

    if connectionState == tentative && (now - unsafe { LastSyncPacket } > ExpressLRS_currAirRate_RFperfParams.RxLockTimeoutMs)
    {
        println!("Bad sync, aborting");
        LostConnection();

        unsafe {
            RFmodeLastCycled = now;
            LastSyncPacket = now;
        }
    }

    unsafe {
        cycleRfMode(now);
        servosUpdate(now);
    }

    let localLastValidPacket: u32 = unsafe { LastValidPacket }; // Required to prevent race condition due to LastValidPacket getting updated from ISR
    if (connectionState == disconnectPending) ||
        ((connectionState == connected) && (ExpressLRS_currAirRate_RFperfParams.DisconnectTimeoutMs as i32 < (now - localLastValidPacket) as i32)) // check if we lost conn.
    {
        LostConnection();
    }

    if (connectionState == tentative) && (abs(OffsetDx) <= 10) && (Offset < 100) && (LQCalc.getLQRaw() > minLqForChaos()) //detects when we are connected
    {
        GotConnection(now);
    }

    unsafe { checkSendLinkStatsToFc(now) };

    if (RXtimerState == tim_tentative) && ((now - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(OffsetDx) <= 5)
    {
        RXtimerState = tim_locked;
        println!("Timer locked");
    }

    let nextPayload: u8 = 0;
    let nextPlayloadSize: u8 = 0;
    if !TelemetrySender.IsActive() && telemetry.GetNextPayload(&nextPlayloadSize, &nextPayload)
    {
        TelemetrySender.SetDataToTransmit(nextPlayloadSize, nextPayload, ELRS_TELEMETRY_BYTES_PER_CALL);
    }
    unsafe { updateTelemetryBurst() };
    updateBindingMode();
}

fn EnterBindingMode()
{
    if (connectionState == connected) || unsafe { InBindingMode } {
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
    unsafe { InBindingMode = true; }

    // Start attempting to bind
    // Lock the RF rate and freq while binding
    SetRFLinkRate(RATE_BINDING);
    Radio.SetFrequencyReg(GetInitialFreq());
    // If the Radio Params (including InvertIQ) parameter changed, need to restart RX to take effect
    Radio.RXnb();

    println!("Entered binding mode at freq = %d", Radio.currFreq);
    devicesTriggerEvent();
}

fn ExitBindingMode()
{
    if !unsafe { InBindingMode } {
        // Not in binding mode
        println!("Cannot exit binding mode, not in binding mode!");
        return;
    }

    // Prevent any new packets from coming in
    Radio.SetTxIdleMode();
    LostConnection();
    // Write the values to eeprom
    config.Commit();

    CRCInitializer = (UID[4] << 8) | UID[5];
    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    // Force RF cycling to start at the beginning immediately
    scanIndex = RATE_MAX;
    unsafe { RFmodeLastCycled } = 0;

    // Do this last as LostConnection() will wait for a tock that never comes
    // if we're in binding mode
    unsafe { InBindingMode } = false;
    println!("Exiting binding mode");
    devicesTriggerEvent();
}

// ICACHE_RAM_ATTR
fn OnELRSBindMSP(packet: &[u8])
{
    for i in 1..=4 {
        UID[i + 1] = packet[i];
    }

    println!("New UID = %d, %d, %d, %d, %d, %d", UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

    // Set new UID in eeprom
    config.SetUID(UID);

    // Set eeprom byte to indicate RX is bound
    config.SetIsBound(true);

    // EEPROM commit will happen on the main thread in ExitBindingMode()
}

fn UpdateModelMatch(model: u8) {
    println!("Set ModelId=%u", model);

    config.SetModelId(model);
    config.Commit();
    // This will be called from ProcessRFPacket(), schedule a disconnect
    // in the main loop once the ISR has exited
    connectionState = disconnectPending;
}

