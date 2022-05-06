#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX12xxDriverCommon/SX12xxDriverCommon.h
//! https://raw.githubusercontent.com/ExpressLRS/ExpressLRS/master/src/lib/SX1280Driver/SX1280.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX1280Driver/SX1280.cpp

use core::mem;
use defmt::println;

use super::{sx1280_hal::*, sx1280_regs::*};

use cortex_m::delay::Delay;

// Inside `DriverCommon` on orig.
const TXRXBuffSize: usize = 16; // todo define type?
const wtimeoutUS: i32 = 1_000; // todo define type?

// special case for command == RadioCommands::GET_STATUS, fixed 3 bytes packet size
const RADIO_GET_STATUS_BUF_SIZEOF: usize = 3; // todo: define type?

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Inside `SX12xxDriverCommon` on official impl.
pub enum RxStatus {
    Ok = 0b00,
    CrcFail = 0b01,
    RxTimeout = 0b10,
}

/// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX12xxDriverCommon/SX12xxDriverCommon.h
pub struct SX12xxDriverCommon {
    pub rx_status: RxStatus,
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
    packet_mode: PacketTypes,
    modeSupportsFei: bool,

    hal: SX1280Hal, // global var in original impl.
    common: SX12xxDriverCommon, // global var in original impl.
}

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

// static mut FIFOaddr: u8 = 0;

/*
 * Period Base from table 11-24, page 79 datasheet rev 3.2
 * RadioCommands::TICK_SIZE_0015_US = 15625 nanos
 * RadioCommands::TICK_SIZE_0062_US = 62500 nanos
 * RadioCommands::TICK_SIZE_1000_US = 1000000 nanos
 * RadioCommands::TICK_SIZE_4000_US = 4000000 nanos
 */
const RX_TIMEOUT_PERIOD_BASE: u32 = RadioCommands::TICK_SIZE_0015_US; // todo: Define type?

const RX_TIMEOUT_PERIOD_BASE_NANOS: u32 = 15_635; // todo: Define type?

impl SX1280Driver {
    // SX1280Driver(): SX12xxDriverCommon()
    // {
    //     instance = this;
    // }

    pub fn new(hal: SX1280Hal, common: SX12xxDriverCommon) -> Self {
        Self {
            timeout: 0xFFFF,
            currOpmode: OperatingModes::SLEEP,
            packet_mode: PacketTypes::LORA, // todo?
            modeSupportsFei: false,         //todo?
            hal,
            common,
        }
    }

    pub fn End(&mut self) {
        self.SetMode(OperatingModes::SLEEP);
        self.hal.end();
        RemoveCallbacks();
        self.common.currFreq = 2_400_000_000;
        self.common.PayloadLength = 8; // Dummy default value which is overwritten during setup.
    }

    pub fn Begin(&mut self, delay: &mut Delay) -> bool {
        self.hal.reset();
        println!("SX1280 Begin");
        delay.delay_ms(100);
        let firmwareRev: u16 = (self.hal.ReadRegisterOne(Reg::LR_FIRMWARE_VERSION_MSB) << 8) as u16
            | self.hal.ReadRegisterOne(Reg::LR_FIRMWARE_VERSION_LSB) as u16;
        println!("Read Vers: {}", firmwareRev);
        if (firmwareRev == 0) || (firmwareRev == 65535) {
            // SPI communication failed, just return without configuration
            return false;
        }

        self.SetMode(OperatingModes::STDBY_RC); //Put in STDBY_RC mode
        self.hal
            .WriteCommandOne(RadioCommands::SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA); //Set packet type to LoRa
        self.ConfigModParamsLoRa(LoRaBandwidths::BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7); //Configure Modulation Params
        self.hal.WriteCommandOne(RadioCommands::SET_AUTOFS, 0x01); //Enable auto FS
        self.hal
            .WriteRegisterOne(Reg::RxGain, (self.hal.ReadRegisterOne(Reg::RxGain) | 0xC0)); //default is low power mode, switch to high sensitivity instead
        self.SetPacketParamsLoRa(
            12,
            SX1280_LORA_PACKET_IMPLICIT,
            8,
            SX1280_LORA_CRC_OFF,
            SX1280_LORA_IQ_NORMAL,
        ); //default params
        self.SetFrequencyReg(self.common.currFreq); //Set Freq
        self.SetFIFOaddr(0x00, 0x00); //Config FIFO addr
        self.SetDioIrqParams(
            IrqMasks::RADIO_ALL as u16,
            IrqMasks::TX_DONE as u16 | IrqMasks::RX_DONE as u16,
            0,
            0,
        ); // todo: 0-padding? fn takes 4 params.                                             //set IRQ to both RXdone/TXdone on DIO1
           // #if defined(USE_SX1280_DCDC)
           // We are using the DC to DC regulator. // todo: COnfirm this.
        self.hal
            .WriteCommandOne(RadioCommands::SET_REGULATORMODE, SX1280_USE_DCDC); // Enable DCDC converter instead of LDO
                                                                                 // #endif
        return true;
    }

    pub fn Config(
        &mut self,
        bw: u8,
        sf: u8,
        cr: u8,
        freq: u32,
        PreambleLength: u8,
        InvertIQ: bool,
        _PayloadLength: u8,
        interval: u32,
        flrcSyncWord: u32,
        flrcCrcSeed: u16,
        flrc: u8,
    ) {
        let mut irqs: u8 = (IrqMasks::TX_DONE as u16 | IrqMasks::RX_DONE as u16) as u8;
        let mode = if flrc != 0 {
            PacketTypes::FLRC
        } else {
            PacketTypes::LORA
        };

        self.common.PayloadLength = _PayloadLength;
        self.common.IQinverted = InvertIQ;
        self.packet_mode = mode;
        self.SetMode(OperatingModes::STDBY_XOSC);
        self.hal
            .WriteCommandOne(RadioCommands::SET_PACKETTYPE, mode as u8);

        if mode == PacketTypes::FLRC {
            println!("Config FLRC");
            self.ConfigModParamsFLRC(bw, cr, sf);
            self.SetPacketParamsFLRC(
                FlrcPacketType::FIXED_LENGTH,
                /*crc=*/ 1,
                PreambleLength,
                _PayloadLength,
                flrcSyncWord,
                flrcCrcSeed,
            );
            irqs |= IrqMasks::CRC_ERROR as u8
        } else {
            println!("Config LoRa");
            ConfigModParamsLoRa(bw, sf, cr);
            // #if defined(DEBUG_FREQ_CORRECTION)
            //         SX1280_RadioLoRaPacketLengthsModes_t packetLengthType = SX1280_LORA_PACKET_VARIABLE_LENGTH;
            // #else
            let packetLengthType = LoRaPacketLengthsModes::FIXED_LENGTH;
            // #endif
            SetPacketParamsLoRa(
                PreambleLength,
                packetLengthType,
                _PayloadLength,
                CrcTypes::CRC_OFF,
                InvertIQ,
            );
        }
        self.SetFrequencyReg(freq);
        self.SetDioIrqParams(IrqMasks::RADIO_ALL as u16, irqs as u16, 0, 0); // todo: 0-padding? fn takes 4 params.
        self.SetRxTimeoutUs(interval);
    }

    pub fn SetRxTimeoutUs(&mut self, interval: u32) {
        if interval > 0 {
            self.timeout = (interval * 1000 / RX_TIMEOUT_PERIOD_BASE_NANOS) as u16;
        // number of periods for the SX1280 to timeout
        } else {
            self.timeout = 0xFFFF; // no timeout, continuous mode
        }
    }

   pub fn SetOutputPower(&mut self, mut power: i8) { // todo: Make sure the intent isn't to mut power in place.
        if power < -18 {
            power = -18;
        } else if 13 < power {
            power = 13;
        }
        let mut buf: [u8; 2] = [(power + 18) as u8, RampTime::RAMP_04_US as u8];
        self.hal
            .WriteCommand(RadioCommands::SET_TXPARAMS, &buf, sizeof(buf));
        // println!("SetPower: %d", buf[0]);
    }

    pub fn SetMode(&mut self, OPmode: OperatingModes) {
        if OPmode == self.currOpmode {
            return;
        }

        let mut buf = [0_u8; 3];
        let mut switchDelay: u32 = 0;

        match OPmode {
            OperatingModes::SLEEP => {
                self.hal.WriteCommandOne(RadioCommands::SET_SLEEP, 0x01);
            }

            OperatingModes::CALIBRATION => {}

            OperatingModes::STDBY_RC => {
                self.hal
                    .WriteCommandOne(RadioCommands::SET_STANDBY, OperatingModes::STDBY_RC as u8);
                switchDelay = 1500;
            }

            OperatingModes::STDBY_XOSC => {
                self.hal
                    .WriteCommandOne(RadioCommands::SET_STANDBY, OperatingModes::STDBY_XOSC as u8);
                switchDelay = 50;
            }

            OperatingModes::FS => {
                self.hal.WriteCommandOne(RadioCommands::SET_FS, 0x00);
                switchDelay = 70;
            }

            OperatingModes::RX => {
                buf[0] = RX_TIMEOUT_PERIOD_BASE as u8;
                buf[1] = timeout >> 8;
                buf[2] = timeout & 0xFF;
                self.hal
                    .WriteCommand(RadioCommands::SET_RX, &buf, sizeof(buf));
                switchDelay = 100;
            }

            OperatingModes::TX => {
                //uses timeout Time-out duration = periodBase * periodBaseCount
                buf[0] = RX_TIMEOUT_PERIOD_BASE as u8;
                buf[1] = 0xFF; // no timeout set for now
                buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
                self.hal
                    .WriteCommand(RadioCommands::SET_TX, &buf, sizeof(buf));
                switchDelay = 100;
            }

            OperatingModes::CAD => {}

            _ => (),
        }
        hal.BusyDelay(switchDelay);

        self.currOpmode = OPmode;
    }

    pub fn ConfigModParamsLoRa(&mut self, bw: LoRaBandwidths, sf: LoRaSpreadingFactors, cr: u8) {
        // Care must therefore be taken to ensure that modulation parameters are set using the command
        // SetModulationParam() only after defining the packet type SetPacketType() to be used

        let rfparams = [sf as u8, bw as u8, cr];

        self.hal.WriteCommand(
            RadioCommands::SET_MODULATIONPARAMS,
            &rfparams,
            rfparams.len(),
        );

        match sf {
            LoRaSpreadingFactors::SF5 | LoRaSpreadingFactors::SF6 => {
                hal.WriteRegisterOne(Reg::SF_ADDITIONAL_CONFIG, 0x1E); // for SF5 or SF6
            }
            LoRaSpreadingFactors::SF7 | LoRaSpreadingFactors::SF8 => {
                hal.WriteRegisterOne(Reg::SF_ADDITIONAL_CONFIG, 0x37); // for SF7 or SF8
            }
            _ => {
                hal.WriteRegisterOne(Reg::SF_ADDITIONAL_CONFIG, 0x32); // for SF9, SF10, SF11, SF12
            }
        }
        // Enable frequency compensation
        hal.WriteRegisterOne(Reg::FREQ_ERR_CORRECTION, 0x1);
    }

    pub fn SetPacketParamsLoRa(
        &mut self,
        PreambleLength: u8,
        HeaderType: LoRaPacketLengthModes,
        PayloadLength: u8,
        crc: LoRaCrcModes,
        InvertIQ: u8,
    ) {
        let mut buf = [0_u8; 7];

        buf[0] = PreambleLength;
        buf[1] = HeaderType as u8;
        buf[2] = PayloadLength;
        buf[3] = crc as u8;
        buf[4] = if InvertIQ != 0 {
            LoRaIqModes::INVERTED as u8
        } else {
            LoRaIqModes::NORMAL as u8
        };
        buf[5] = 0x00;
        buf[6] = 0x00;

        self.hal
            .WriteCommand(RadioCommands::SET_PACKETPARAMS, &buf, buf.len());

        // FEI only triggers in Lora mode when the header is present :(
        self.modeSupportsFei = HeaderType == LoRaPacketLengthsModes::VARIABLE_LENGTH;
    }

    pub fn ConfigModParamsFLRC(&mut self, bw: u8, cr: u8, bt: u8) {
        let rfparams = [bw, cr, bt];
        self.hal.WriteCommand(
            RadioCommands::SET_MODULATIONPARAMS,
            &rfparams,
            rfparams.len(),
        );
    }

    pub fn SetPacketParamsFLRC(
        &mut self,
        HeaderType: FlrcPacketType,
        // todo: Check the original code isn't meant to modify thes ein place, ie &mut
        mut crc: u8,
        mut PreambleLength: u8,
        PayloadLength: u8,
        syncWord: u32,
        crcSeed: u16,
    ) {
        // todo: modifying these in place doesn't cause consequences for the caller, right? Or
        // todo maybe we need the consequencse4?
        if PreambleLength < 8 {
            PreambleLength = 8;
        }
        PreambleLength = ((PreambleLength / 4) - 1) << 4;
        crc = if crc != 0 {
            FlrcCrc::CRC_2_BYTE as u8
        } else {
            FlrcCrc::CRC_OFF as u8
        };

        let mut buf = [0; 7];
        buf[0] = PreambleLength; // AGCPreambleLength
        buf[1] = FlrcSyncWordLen::WORD_LEN_P32S as u8; // SyncWordLength
        buf[2] = FlrcSyncWordCombination::MATCH_SYNC_WORD_1 as u8; // SyncWordMatch
        buf[3] = HeaderType as u8; // PacketType
        buf[4] = PayloadLength; // PayloadLength
        buf[5] = crc << 4; // CrcLength
        buf[6] = 0x08; // Must be whitening disabled
        self.hal
            .WriteCommand(RadioCommands::SET_PACKETPARAMS, &buf, buf.len());

        // CRC seed (use dedicated cipher)
        buf[0] = (crcSeed >> 8) as u8;
        buf[1] = crcSeed as u8;
        self.hal.WriteRegister(Reg::FLRC_CRC_SEED, &buf, 2);

        // CRC POLY 0x3D65
        buf[0] = 0x3D;
        buf[1] = 0x65;
        self.hal.WriteRegister(Reg::FLRC_CRC_POLY, &buf, 2);

        // Set SyncWord1
        buf[0] = (syncWord >> 24) as u8;
        buf[1] = (syncWord >> 16) as u8;
        buf[2] = (syncWord >> 8) as u8;
        buf[3] = syncWord as u8;
        self.hal.WriteRegister(Reg::FLRC_SYNC_WORD, &buf, 4);

        // FEI only works in Lora and Ranging mode
        self.modeSupportsFei = false;
    }

    pub fn SetFrequencyHz(&mut self, Reqfreq: u32) {
        // WORD_ALIGNED_ATTR uint8_t buf[3] = {0};
        let mut buf = [0_u8; 3];

        let freq: u32 = (Reqfreq / FREQ_STEP) as u32;
        buf[0] = ((freq >> 16) & 0xFF) as u8;
        buf[1] = ((freq >> 8) & 0xFF) as u8;
        buf[2] = (freq & 0xFF) as u8;

        self.hal
            .WriteCommand(RadioCommands::SET_RFFREQUENCY, &buf, buf.len());
        self.common.currFreq = Reqfreq;
    }

    pub fn SetFrequencyReg(&mut self, freq: u32) {
        let mut buf: [u8; 3] = [0; 3];

        buf[0] = ((freq >> 16) & 0xFF) as u8;
        buf[1] = ((freq >> 8) & 0xFF) as u8;
        buf[2] = (freq & 0xFF) as u8;

        self.hal
            .WriteCommand(RadioCommands::SET_RFFREQUENCY, &buf, buf.len());
        self.common.currFreq = freq;
    }

    pub fn SetFIFOaddr(&mut self, txBaseAddr: u8, rxBaseAddr: u8) {
        let mut buf: [u8; 2] = [0; 2];

        buf[0] = txBaseAddr;
        buf[1] = rxBaseAddr;
        self.hal
            .WriteCommand(RadioCommands::SET_BUFFERBASEADDRESS, &buf, buf.len());
    }

    pub fn SetDioIrqParams(
        &mut self,
        irqMask: u16,
        dio1Mask: u16,
        dio2Mask: u16,
        dio3Mask: u16,
    ) {
        let mut buf: [u8; 8] = [0; 8];

        buf[0] = ((irqMask >> 8) & 0x00FF) as u8;
        buf[1] = (irqMask & 0x00FF) as u8;
        buf[2] = ((dio1Mask >> 8) & 0x00FF) as u8;
        buf[3] = (dio1Mask & 0x00FF) as u8;
        buf[4] = ((dio2Mask >> 8) & 0x00FF) as u8;
        buf[5] = (dio2Mask & 0x00FF) as u8;
        buf[6] = ((dio3Mask >> 8) & 0x00FF) as u8;
        buf[7] = (dio3Mask & 0x00FF) as u8;

        self.hal
            .WriteCommand(RadioCommands::SET_DIOIRQPARAMS, &buf, buf.len());
    }

    pub fn GetIrqStatus() -> u16 {
        let mut status: [u8; 2] = [0; 2];

        hal.ReadCommand(RadioCommands::GET_IRQSTATUS, status, 2);
        (status[0] as u16) << 8 | (status[1] as u16)
    }

    pub fn ClearIrqStatus(&mut self, irqMask: IrqMasks) {
        let mut buf = [0_u8; 2];

        buf[0] = ((irqMask as u16 >> 8) & 0x00FF) as u8;
        buf[1] = (irqMask as u16 & 0x00FF) as u8;

        self.hal
            .WriteCommand(RadioCommands::CLR_IRQSTATUS, &mut buf, buf.len());
    }

    pub fn TXnbISR(&mut self) {
        self.currOpmode = OperatingModes::FS; // radio goes to FS after TX
                                              // #ifdef DEBUG_SX1280_OTA_TIMING
                                              //     endTX = micros();
                                              //     println!("TOA: %d", endTX - beginTX);
                                              // #endif
        TXdoneCallback();
    }

    pub fn TXnb(&mut self) {
        //catch TX timeout
        if currOpmode == OperatingModes::TX {
            //println!("Timeout!");
            self.SetMode(OperatingModes::FS);
            TXnbISR();
            return;
        }
        hal.TXenable(); // do first to allow PA stablise
        hal.WriteBuffer(0x00, self.common.TXdataBuffer, self.common.PayloadLength); //todo fix offset to equal fifo addr
        instance.SetMode(OperatingModes::TX);
        // #ifdef DEBUG_SX1280_OTA_TIMING
        //     beginTX = micros();
        // #endif
    }

    // todo: Place in apt place for ISR
    pub fn RXnbISR(&mut self, irqStatus: u16) {
        let fail = if (irqStatus & IrqMasks::CRC_ERROR as u16) != 0 {
            RxStatus::CrcFail as u16
        } else {
            RxStatus::Ok as u16 + if (irqStatus & IrqMasks::RX_TX_TIMEOUT as u16) != 0 {
                RxStatus::RxTimeout as u16
            } else {
                RxStatus::Ok as u16
            }
        };

        // In continuous receive mode, the device stays in Rx mode
        if timeout != 0xFFFF {
            // From table 11-28, pg 81 datasheet rev 3.2
            // upon successsful receipt, when the timer is active or in single mode, it returns to STDBY_RC
            // but because we have AUTO_FS enabled we automatically transition to state OperatingModes::FS
            self.currOpmode = OperatingModes::FS;
        }
        if fail == RxStatus::Ok as u16 {
            let FIFOaddr: u8 = self.GetRxBufferAddr();
            hal.ReadBuffer(FIFOaddr, self.common.RXdataBuffer, self.common.PayloadLength);
            GetLastPacketStats();
        }
        // todo: Where is this callback defined (other than as null?)
        // self.RXdoneCallback(fail);
    }

    pub fn RXnb(&mut self) {
        self.hal.RXenable();
        self.SetMode(OperatingModes::RX);
    }

    pub fn GetRxBufferAddr(&mut self) -> u8 {
        let mut status = [0_u8; 2];
        self.hal
            .ReadCommand(RadioCommands::GET_RXBUFFERSTATUS, &mut status, 2);
        return status[1];
    }

    pub fn GetStatus(&mut self) {
        let mut status: u8 = 0;
        self.hal
            .ReadCommand(RadioCommands::GET_STATUS, &mut [status], 1);
        println!(
            "Status: %x, %x, %x",
            (0b11100000 & status) >> 5,
            (0b00011100 & status) >> 2,
            0b00000001 & status
        );
    }

    pub fn GetFrequencyErrorbool(&self) -> bool {
        // Only need the highest bit of the 20-bit FEI to determine the direction
        let feiMsb: u8 = hal.ReadRegisterOne(Reg::LR_ESTIMATED_FREQUENCY_ERROR_MSB);
        // fei & (1 << 19) and flip sign if IQinverted
        if (feiMsb & 0x08) != 0 {
            self.common.IQinverted
        } else {
            !self.common.IQinverted
        }
    }

    pub fn GetRssiInst(&mut self) -> i8 {
        let status: u8 = 0;

        self.hal
            .ReadCommand(RadioCommands::GET_RSSIINST, &mut [status], 1);
        -((status / 2) as i8)
    }

    pub fn GetLastPacketStats(&mut self) {
        let mut status = [0_u8; 2];

        self.hal
            .ReadCommand(RadioCommands::GET_PACKETSTATUS, &mut status, 2);
        if self.packet_mode == PacketTypes::FLRC {
            // No SNR in FLRC mode
            self.common.LastPacketRSSI = -((status[1] / 2) as i8);
            self.common.LastPacketSNR = 0;
            return;
        }
        // LoRa mode has both RSSI and SNR
        self.common.LastPacketRSSI = -((status[0] as i8 / 2) as i8);
        self.common.LastPacketSNR = status[1] as i8 / 4;
        // https://www.mouser.com/datasheet/2/761/DS_SX1280-1_V2.2-1511144.pdf
        // need to subtract SNR from RSSI when SNR <= 0;
        let negOffset: i8 = if self.common.LastPacketSNR < 0 { self.common.LastPacketSNR } else { 0 };
        self.common.LastPacketRSSI += negOffset;
    }

    pub fn IsrCallback(&mut self) {
        let mut irqStatus: u16 = instance.GetIrqStatus();
        self.ClearIrqStatus(IrqMasks::RADIO_ALL);
        if (irqStatus & IrqMasks::TX_DONE as u16) != 0 {
            self.hal.TXRXdisable();
            self.TXnbISR();
        } else if irqStatus
            & (IrqMasks::RX_DONE as u16 | IrqMasks::CRC_ERROR as u16 | IrqMasks::RX_TX_TIMEOUT as u16)
            != 0
        {
            self.RXnbISR(irqStatus);
        }
    }
}
