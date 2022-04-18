#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX12xxDriverCommon/SX12xxDriverCommon.h
//! https://raw.githubusercontent.com/ExpressLRS/ExpressLRS/master/src/lib/SX1280Driver/SX1280.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX1280Driver/SX1280.cpp

use defmt::println;


use super::{
    sx1280_hal::*,
    sx1280_regs::*,
};

use cortex_m::delay::Delay;

type rx_status = u8;


#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
/// Inside `SX12xxDriverCommon` on official impl.
enum RxStatus
    {
        SX12XX_RX_OK        = 0,
        SX12XX_RX_CRC_FAIL  = 1 << 0,
        SX12XX_RX_TIMEOUT   = 1 << 1,
    }

// Inside `DriverCommon` on orig.
const TXRXBuffSize: usize =  16; // todo define type?
const wtimeoutUS: i32 = 1_000; // todo define type?

 // special case for command == SX1280_RADIO_GET_STATUS, fixed 3 bytes packet size
const RADIO_GET_STATUS_BUF_SIZEOF: usize = 3; // todo: define type?

struct SX12xxDriverCommon {
    pub TXdataBuffer: [u8; TXRXBuffSize],
    pub RXdataBuffer: [u8; TXRXBuffSize],

    /////////// Radio Variables////////
    pub currFreq: u32,
    pub PayloadLength: u8,
    pub IQinverted: bool,

    /////////////Packet Stats//////////
    pub LastPacketRSSI: i8,
    pub LastPacketSNR: i8,
}

// impl SX12xxDriverCommon {
//     fn RemoveCallbacks(&mut self)     {
//         RXdoneCallback = nullCallbackRx;
//         TXdoneCallback = nullCallbackTx;
//     }
// }


// todo: Come back to this
struct SX1280Driver {
    ///////////Radio Variables////////
    timeout: u16,

    ///////////////////////////////////

    ////////////////Configuration Functions/////////////

    currOpmode: OperatingModes,
    packet_mode: u8,
    modeSupportsFei: bool,

}

impl Default for SX1280Driver {
    fn default() -> Self {
        Self {
            timeout: 0xFFFF,
            currOpmode: OperatingModes::SLEEP,
            packet_mode: 0, // todo?
            modeSupportsFei: false, //todo?
        }
    }
}


// static mut hal: SX1280Hal = SX1280Hal { };
// SX1280Driver *SX1280Driver::instance = NULL;

//DEBUG_SX1280_OTA_TIMING

/* Steps for startup

1. If not in STDBY_RC mode, then go to this mode by sending the command:
SetStandby(STDBY_RC)

2. Define the LoRa® packet type by sending the command:
SetPacketType(PACKET_TYPE_LORA)

3. Define the RF frequency by sending the command:
SetRfFrequency(rfFrequency)
The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.

4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
byte of the data payload by sending the command:
SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
Note:
txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.

5. Define the modulation parameter signal BW SF CR
*/

// #if defined(DEBUG_SX1280_OTA_TIMING)
static mut beginTX: u32 = 0;
static mut endTX: u32 = 0;
// #endif

static mut FIFOaddr: u8 = 0;

/*
 * Period Base from table 11-24, page 79 datasheet rev 3.2
 * SX1280_RADIO_TICK_SIZE_0015_US = 15625 nanos
 * SX1280_RADIO_TICK_SIZE_0062_US = 62500 nanos
 * SX1280_RADIO_TICK_SIZE_1000_US = 1000000 nanos
 * SX1280_RADIO_TICK_SIZE_4000_US = 4000000 nanos
 */
const  RX_TIMEOUT_PERIOD_BASE: u32 = SX1280_RADIO_TICK_SIZE_0015_US; // todo: Define type?

const RX_TIMEOUT_PERIOD_BASE_NANOS: u32 = 15_635; // todo: Define type?

impl SX1280Driver {

// SX1280Driver(): SX12xxDriverCommon()
// {
//     instance = this;
// }

fn End(hal: &mut SX1280Hal) {
    SetMode(SX1280_MODE_SLEEP);
    unsafe { hal.end(); }
    RemoveCallbacks();
    currFreq = 2_400_000_000;
    PayloadLength = 8; // Dummy default value which is overwritten during setup.
}

unsafe fn Begin(delay: &mut Delay) -> bool {
    hal.init();
    hal.IsrCallback = &IsrCallback;

    hal.reset();
    println!("SX1280 Begin");
    delay.delay_ms(100);
    let firmwareRev: u16 = (((hal.ReadRegister(Reg::LR_FIRMWARE_VERSION_MSB)) << 8) | (hal.ReadRegister(Reg::LR_FIRMWARE_VERSION_MSB + 1)));
    println!("Read Vers: %d", firmwareRev);
    if (firmwareRev == 0) || (firmwareRev == 65535)     {
        // SPI communication failed, just return without configuration
        return false;
    }

    SetMode(OperatingModes::STDBY_RC);                                                                                                //Put in STDBY_RC mode
    hal.WriteCommand(RadioCommands::SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA);                                                       //Set packet type to LoRa
    ConfigModParamsLoRa(LoRaBandwidths::BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7);                                                //Configure Modulation Params
    hal.WriteCommand(RadioCommands::SET_AUTOFS, 0x01);                                                                              //Enable auto FS
    hal.WriteRegister(0x0891, (hal.ReadRegister(0x0891) | 0xC0));                                                                 //default is low power mode, switch to high sensitivity instead
    SetPacketParamsLoRa(12, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL);                          //default params
    SetFrequencyReg(currFreq);                                                                                                    //Set Freq
    SetFIFOaddr(0x00, 0x00);                                                                                                      //Config FIFO addr
    SetDioIrqParams(IrqMasks::RADIO_ALL, IrqMasks::TX_DONE | IrqMasks::RX_DONE);                                               //set IRQ to both RXdone/TXdone on DIO1
// #if defined(USE_SX1280_DCDC)
        // We are using the DC to DC regulator. // todo: COnfirm this.
    hal.WriteCommand(RadioCommands::SET_REGULATORMODE, SX1280_USE_DCDC);     // Enable DCDC converter instead of LDO
// #endif
    return true;
}

fn Config(bw: u8, sf: u8, cr: u8, freq: u32,
                          PreambleLength: u8,InvertIQ: bool, _PayloadLength: u8, interval: u32,
                          flrcSyncWord: u32, flrcCrcSeed: u16, flrc: u8)
{
    let irqs: u8 = IrqMasks::TX_DONE | IrqMasks::RX_DONE;
    let mode = if flrc != 0{ PacketTypes::FLRC } else { PacketTypes::LORA };

    PayloadLength = _PayloadLength;
    IQinverted = InvertIQ;
    packet_mode = mode;
    SetMode(OperatingModes::STDBY_XOSC);
    hal.WriteCommand(RadioCommands::SET_PACKETTYPE, mode);

    if mode == PacketTypes::FLRC {
        println!("Config FLRC");
        ConfigModParamsFLRC(bw, cr, sf);
        SetPacketParamsFLRC(SX1280_FLRC_PACKET_FIXED_LENGTH, /*crc=*/1,
                            PreambleLength, _PayloadLength, flrcSyncWord, flrcCrcSeed);
        irqs |= SX1280_IRQ_CRC_ERROR;
    }
    else
    {
        println!("Config LoRa");
        ConfigModParamsLoRa(bw, sf, cr);
// #if defined(DEBUG_FREQ_CORRECTION)
//         SX1280_RadioLoRaPacketLengthsModes_t packetLengthType = SX1280_LORA_PACKET_VARIABLE_LENGTH;
// #else
        SX1280_RadioLoRaPacketLengthsModes_t packetLengthType = SX1280_LORA_PACKET_FIXED_LENGTH;
// #endif
        SetPacketParamsLoRa(PreambleLength, packetLengthType,
                            _PayloadLength, SX1280_LORA_CRC_OFF, InvertIQ);
    }
    SetFrequencyReg(freq);
    SetDioIrqParams(IrqMasks::RADIO_ALL, irqs);
    SetRxTimeoutUs(interval);
}

fn  SetRxTimeoutUs(interval: u32){
    if interval {
        timeout = interval * 1000 / RX_TIMEOUT_PERIOD_BASE_NANOS; // number of periods for the SX1280 to timeout
    }
    else
    {
        timeout = 0xFFFF;   // no timeout, continuous mode
    }
}

fn SetOutputPower(power: &mut i8) {
    if power < -18 {
        power = -18;
    } else if 13 < power{
        power = 13;
    }
    let mut buf: [u8; 2] = [(power + 18) as u8, SX1280_RADIO_RAMP_04_US as u8];
    hal.WriteCommand(RadioCommands::SET_TXPARAMS, buf, sizeof(buf));
    // println!("SetPower: %d", buf[0]);
}

fn SetMode(OPmode: SX1280_RadioOperatingModes, hal: &mut SX1280Hal) {
    if OPmode == OpMode::currOpmode {
       return;
    }

    let mut buf = [0_u8; 3];
    let mut switchDelay: u32 = 0;

    match OPmode {

   OperatingModes::SLEEP => {
        hal.WriteCommand(RadioCommands::SET_SLEEP, 0x01);
        }

    OperatingModes::CALIBRATION => {}

    OperatingModes::STDBY_RC => {
        hal.WriteCommand(RadioCommands::SET_STANDBY, OperatingModes::STDBY_RC);
        switchDelay = 1500;
    }

    OperatingModes::STDBY_XOSC => {
        hal.WriteCommand(RadioCommands::SET_STANDBY, OperatingModes::STDBY_XOSC);
        switchDelay = 50;
    }

    OperatingModes::MODE_FS => {
        hal.WriteCommand(RadioCommands::SET_FS, 0x00);
        switchDelay = 70;
    }

    OperatingModes::RX => {
        buf[0] = RX_TIMEOUT_PERIOD_BASE as u8;
        buf[1] = timeout >> 8;
        buf[2] = timeout & 0xFF;
        hal.WriteCommand(RadioCommands::SET_RX, buf, sizeof(buf));
        switchDelay = 100;
    }

    OperatingModes::TX => {
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = RX_TIMEOUT_PERIOD_BASE as u8;
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        hal.WriteCommand(RadioCommands::SET_TX, buf, sizeof(buf));
        switchDelay = 100;
    }

    OperatingModes::CAD => {
        }

    _ => (),
    }
    hal.BusyDelay(switchDelay);

    currOpmode = OPmode;
}

fn ConfigModParamsLoRa(bw: u8, sf: u8, cr: u8, hal: &mut SX1280Hal) {
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    let rfparams = [sf, bw, cr];

    hal.WriteCommand(RadioCommands::SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));

    match sf {
        SX1280_LORA_SF5 | SX1280_LORA_SF6=> {
            hal.WriteRegister(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x1E); // for SF5 or SF6
        }
        SX1280_LORA_SF | SX1280_LORA_SF8 => {
            hal.WriteRegister(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x37); // for SF7 or SF8
        }
        _ => {
            hal.WriteRegister(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x32); // for SF9, SF10, SF11, SF12
        }
    }
    // Enable frequency compensation
    hal.WriteRegister(SX1280_REG_FREQ_ERR_CORRECTION, 0x1);
}

fn SetPacketParamsLoRa(PreambleLength: u8, HeaderType: RadioLoRaPacketLengthModes,
                                       PayloadLength: u8, crc: RadioLoRaCrcModes,
                                       InvertIQ: u8)
{
    let mut buf = [0_u8; 7];

    buf[0] = PreambleLength;
    buf[1] = HeaderType;
    buf[2] = PayloadLength;
    buf[3] = crc;
    buf[4] = InvertIQ ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL;
    buf[5] = 0x00;
    buf[6] = 0x00;

    hal.WriteCommand(RadioCommands::SET_PACKETPARAMS, buf, sizeof(buf));

    // FEI only triggers in Lora mode when the header is present :(
    modeSupportsFei = HeaderType == SX1280_LORA_PACKET_VARIABLE_LENGTH;
}

fn ConfigModParamsFLRC(w: u8, cr: u8, bt: u8)
{
    let rfparams = [bw, cr, bt];
    hal.WriteCommand(RadioCommands::SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));
}

fn SetPacketParamsFLRC(HeaderType: u8,
                                       crc: &mut u8,
                                      PreambleLength: &mut u8,
                                       PayloadLength: u8,
                                       syncWord: u32,
                                       crcSeed: u16)
{
    if PreambleLength < 8 {
        PreambleLength = 8;
    }
    PreambleLength = ((PreambleLength / 4) - 1) << 4;
    crc = if crc != 0 { SX1280_FLRC_CRC_2_BYTE } else { SX1280_FLRC_CRC_OFF };

    let mut buf = [0; 7];
    buf[0] = *PreambleLength;                    // AGCPreambleLength
    buf[1] = SX1280_FLRC_SYNC_WORD_LEN_P32S;    // SyncWordLength
    buf[2] = SX1280_FLRC_RX_MATCH_SYNC_WORD_1;  // SyncWordMatch
    buf[3] = HeaderType;                        // PacketType
    buf[4] = PayloadLength;                     // PayloadLength
    buf[5] = (crc << 4);                        // CrcLength
    buf[6] = 0x08;                              // Must be whitening disabled
    hal.WriteCommand(RadioCommands::SET_PACKETPARAMS, buf, sizeof(buf));

    // CRC seed (use dedicated cipher)
    buf[0] = (crcSeed >> 8) as u8;
    buf[1] = crcSeed as u8;
    hal.WriteRegister(SX1280_REG_FLRC_CRC_SEED, buf, 2);

    // CRC POLY 0x3D65
    buf[0] = 0x3D;
    buf[1] = 0x65;
    hal.WriteRegister(SX1280_REG_FLRC_CRC_POLY, buf, 2);

    // Set SyncWord1
    buf[0] = (syncWord >> 24) as u8;
    buf[1] = (syncWord >> 16) as u8;
    buf[2] = (syncWord >> 8) as u8;
    buf[3] = syncWord as u8;
    hal.WriteRegister(SX1280_REG_FLRC_SYNC_WORD, buf, 4);

    // FEI only works in Lora and Ranging mode
    modeSupportsFei = false;
}

fn SetFrequencyHz(Reqfreq: u32) {
    // WORD_ALIGNED_ATTR uint8_t buf[3] = {0};
    let mut buf = [0_u8; 3];

    let freq: u32 = (Reqfreq / FREQ_STEP) as u32;
    buf[0] = ((freq >> 16) & 0xFF) as u8;
    buf[1] = ((freq >> 8) & 0xFF) as u8;
    buf[2] = (freq & 0xFF) as u8;

    hal.WriteCommand(RadioCommands::SET_RFFREQUENCY, buf, sizeof(buf));
    currFreq = Reqfreq;
}

fn SetFrequencyReg(freq: u32){
    let mut  buf: [u8; 3] = [0; 3];

    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    hal.WriteCommand(RadioCommands::SET_RFFREQUENCY, buf, sizeof(buf));
    currFreq = freq;
}

fn SetFIFOaddr(txBaseAddr: u8, rxBaseAddr: u8)
{
    let mut buf: [u8; 2] = [0; 2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal.WriteCommand(RadioCommands::SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

fn SetDioIrqParams(irqMask: IrqMasks, dio1Mask: IrqMasks, dio2Mask: IrqMasks, dio3Mask: IrqMasks) {
    let mut buf: [u8; 8] = [0; 8];

    buf[0] = ((irqMask as u16 >> 8) & 0x00FF) as u8;
    buf[1] = (irqMask as u16 & 0x00FF) as u8;
    buf[2] = ((dio1Mask as u16 >> 8) & 0x00FF) as u8;
    buf[3] = (dio1Mask as u16 & 0x00FF) as u8;
    buf[4] = ((dio2Mask as u16 >> 8) & 0x00FF) as u8;
    buf[5] = (dio2Mask as u16 & 0x00FF) as u8;
    buf[6] = ((dio3Mask as u16 >> 8) & 0x00FF) as u8;
    buf[7] = (dio3Mask as u16 & 0x00FF) as u8;

    hal.WriteCommand(RadioCommands::SET_DIOIRQPARAMS, buf, sizeof(buf));
}

fn GetIrqStatus() -> u16
{
    let mut status: [u8; 2] = [0; 2];

    hal.ReadCommand(SX1280_RADIO_GET_IRQSTATUS, status, 2);
    (status[0] as u16) << 8 | (status[1] as u16)
}

fn ClearIrqStatus(irqMask: u16) {
    let mut buf = [0_u8; 2];

    buf[0] = ((irqMask as u16 >> 8) & 0x00FF) as u8;
    buf[1] = (irqMask as u16 & 0x00FF) as u8;

    hal.WriteCommand(RadioCommands::CLR_IRQSTATUS, buf, sizeof(buf));
}

fn TXnbISR() {
    currOpmode = SX1280_MODE_FS; // radio goes to FS after TX
// #ifdef DEBUG_SX1280_OTA_TIMING
//     endTX = micros();
//     println!("TOA: %d", endTX - beginTX);
// #endif
    TXdoneCallback();
}



fn TXnb(){
    //catch TX timeout
    if currOpmode == SX1280_MODE_TX {
        //println!("Timeout!");
        SetMode(SX1280_MODE_FS);
        TXnbISR();
        return;
    }
    hal.TXenable();                      // do first to allow PA stablise
    hal.WriteBuffer(0x00, TXdataBuffer, PayloadLength); //todo fix offset to equal fifo addr
    instance->SetMode(SX1280_MODE_TX);
// #ifdef DEBUG_SX1280_OTA_TIMING
//     beginTX = micros();
// #endif
}

fn  RXnbISR(irqStatus: u16) {
    rx_status const fail =
        ((irqStatus & SX1280_IRQ_CRC_ERROR) ? SX12XX_RX_CRC_FAIL : SX12XX_RX_OK) +
        ((irqStatus & SX1280_IRQ_RX_TX_TIMEOUT) ? SX12XX_RX_TIMEOUT : SX12XX_RX_OK);
    // In continuous receive mode, the device stays in Rx mode
    if (timeout != 0xFFFF)
    {
        // From table 11-28, pg 81 datasheet rev 3.2
        // upon successsful receipt, when the timer is active or in single mode, it returns to STDBY_RC
        // but because we have AUTO_FS enabled we automatically transition to state SX1280_MODE_FS
        currOpmode = SX1280_MODE_FS;
    }
    if fail == SX12XX_RX_OK {
        let FIFOaddr: u8 = GetRxBufferAddr();
        hal.ReadBuffer(FIFOaddr, RXdataBuffer, PayloadLength);
        GetLastPacketStats();
    }
    RXdoneCallback(fail);
}

fn RXnb() {
    hal.RXenable();
    SetMode(SX1280_MODE_RX);
}

fn GetRxBufferAddr() -> u8 {
    WORD_ALIGNED_ATTR uint8_t status[2] = {0};
    hal.ReadCommand(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

fn GetStatus() {
    let mut status: u8 = 0;
    hal.ReadCommand(SX1280_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    println!("Status: %x, %x, %x", (0b11100000 & status) >> 5, (0b00011100 & status) >> 2, 0b00000001 & status);
}

fn GetFrequencyErrorbool() -> bool {
    // Only need the highest bit of the 20-bit FEI to determine the direction
    let feiMsb: u8 = hal.ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    // fei & (1 << 19) and flip sign if IQinverted
    if feiMsb & 0x08 {
        return IQinverted;
    } else {
        return ! IQinverted;
    }
}

fn GetRssiInst() -> i8 {
    let status: u8 = 0;

    hal.ReadCommand(RadioCommands::GET_RSSIINST, (uint8_t *)&status, 1);
    return -(int8_t)(status / 2);
}

fn GetLastPacketStats() {
    let mut status = [0_u8; 2];
    hal.ReadCommand(RadioCommands::GET_PACKETSTATUS, status, 2);
    if packet_mode == PacketTypes::FLRC {
        // No SNR in FLRC mode
        LastPacketRSSI = -(int8_t)(status[1] / 2);
        LastPacketSNR = 0;
        return;
    }
    // LoRa mode has both RSSI and SNR
    LastPacketRSSI = -((status[0] as i8 / 2) as i8);
    LastPacketSNR = status[1] as i8 / 4;
    // https://www.mouser.com/datasheet/2/761/DS_SX1280-1_V2.2-1511144.pdf
    // need to subtract SNR from RSSI when SNR <= 0;
    let negOffset: i8 = if LastPacketSNR < 0 { LastPacketSNR } else { 0 };
    LastPacketRSSI += negOffset;
}

fn IsrCallback() {
    let mut irqStatus: u16 = instance.GetIrqStatus();
    instance.ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if irqStatus & IrqMasks::TX_DONE {
        hal.TXRXdisable();
        instance.TXnbISR();
    }
    else if irqStatus & (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT) {
        instance.RXnbISR(irqStatus);
    }
}

}