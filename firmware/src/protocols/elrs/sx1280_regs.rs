#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX1280Driver/SX1280_Regs.h

// todo: How much of this should be under `drivers/lora_sx1280.rs`?

const REG_LR_FIRMWARE_VERSION_MSB: u32 = 0x0153; //The address of the register holding the firmware version MSB
const SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB: u32 = 0x0954;
const SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK: u32 = 0x0FFFFF;

const SX1280_REG_FLRC_CRC_POLY: u32 =    0x9C6;
const SX1280_REG_FLRC_CRC_SEED: u32 =    0x9C8;
const SX1280_REG_FLRC_SYNC_WORD: u32 =   0x9CF;

const SX1280_XTAL_FREQ: u32 = 52_000_000;
const FREQ_STEP: f64 = SX1280_XTAL_FREQ as f64 / (1<<18) as f64;

#[repr(u8)]
pub enum States
{
    IDLE = 0x00, //!< The radio is idle
    RX_RUNNING,  //!< The radio is in reception state
    TX_RUNNING,  //!< The radio is in transmission state
    CAD,         //!< The radio is doing channel activity detection
}

/// Represents the operating mode the radio is actually running
#[repr(u8)]
pub enum OperatingModes
{
    SLEEP = 0x00, //! The radio is in sleep mode
    CALIBRATION,  //! The radio is in calibration mode
    STDBY_RC,     //! The radio is in standby mode with RC oscillator
    STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
    FS,           //! The radio is in frequency synthesis mode
    RX,           //! The radio is in receive mode
    TX,           //! The radio is in transmit mode
    CAD           //! The radio is in channel activity detection mode
}

// todo
// const SX1280_RX_TX_CONTINUOUS: TickTime = TickSizes::SIZE_0015_US;
// #define SX1280_RX_TX_CONTINUOUS \
//     (TickTime_t) { RADIO_TICK_SIZE_0015_US, 0xFFFF }
// #define SX1280_RX_TX_SINGLE \
//     (TickTime_t) { RADIO_TICK_SIZE_0015_US, 0 }

/*!
/// Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
#[repr(u8)]
pub enum StandbyModes
{
    RC = 0x00,
    XOSC = 0x01,
}

/*!
/// Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
#[repr(u8)]
pub enum RegulatorModes
{
    LDO = 0x00,  //! Use LDO (default value)
    DCDC = 0x01, //! Use DCDC
}

/*!
/// Represents the possible packet type (i.e. modem) used
 */
#[repr(u8)]
pub enum PacketTypes
{
    GFSK = 0x00,
    LORA,
    RANGING,
    FLRC,
    BLE,
    NONE = 0x0F,
}

//only for GFSK
// pub enum
// {
//     SX1280_PREAMBLE_LENGTH_04_BITS = 0x00, //!< Preamble length: 04 bits
//     SX1280_PREAMBLE_LENGTH_08_BITS = 0x10, //!< Preamble length: 08 bits
//     SX1280_PREAMBLE_LENGTH_12_BITS = 0x20, //!< Preamble length: 12 bits
//     SX1280_PREAMBLE_LENGTH_16_BITS = 0x30, //!< Preamble length: 16 bits
//     SX1280_PREAMBLE_LENGTH_20_BITS = 0x40, //!< Preamble length: 20 bits
//     SX1280_PREAMBLE_LENGTH_24_BITS = 0x50, //!< Preamble length: 24 bits
//     SX1280_PREAMBLE_LENGTH_28_BITS = 0x60, //!< Preamble length: 28 bits
//     SX1280_PREAMBLE_LENGTH_32_BITS = 0x70, //!< Preamble length: 32 bits
// } SX1280_RadioPreambleLengths_t;

#[repr(u8)]
pub enum RadioLoRaIQModes
{
   NORMAL = 0x40,
    INVERTED = 0x00,
}

#[repr(u8)]
pub enum CrcTypes
{
    CRC_OFF = 0x00, //!< No CRC in use
    CRC_1_BYTES = 0x10,
    CRC_2_BYTES = 0x20,
    CRC_3_BYTES = 0x30,
}

/// Represents the ramping time for power amplifier
#[repr(u8)]
pub enum RampTimes
{
    RAMP_02_US = 0x00,
    RAMP_04_US = 0x20,
    RAMP_06_US = 0x40,
    RAMP_08_US = 0x60,
    RAMP_10_US = 0x80,
    RAMP_12_US = 0xA0,
    RAMP_16_US = 0xC0,
    RAMP_20_US = 0xE0,
}

/// Represents the number of symbols to be used for channel activity detection operation
#[repr(u8)]
pub enum LoRaCadSymbols
{
    CAD_01_SYMBOL = 0x00,
    CAD_02_SYMBOLS = 0x20,
    CAD_04_SYMBOLS = 0x40,
    CAD_08_SYMBOLS = 0x60,
    CAD_16_SYMBOLS = 0x80,
}

/// Represents the possible spreading factor values in LORA packet types
#[repr(u8)]
pub enum LoraSpreadingFactors
{
    SF5 = 0x50,
    SF6 = 0x60,
    SF7 = 0x70,
    SF8 = 0x80,
    SF9 = 0x90,
    SF10 = 0xA0,
    SF11 = 0xB0,
    SF12 = 0xC0,
}

/// Represents the bandwidth values for LORA packet type
#[repr(u8)]
pub enum LoRaBandwidths
{
    BW_0200 = 0x34,
    BW_0400 = 0x26,
    BW_0800 = 0x18,
   BW_1600 = 0x0A,
}

/// Represents the coding rate values for LORA packet type
#[repr(u8)]
pub enum LoRaCodingRates
{
    CR_4_5 = 0x01,
    CR_4_6 = 0x02,
    CR_4_7 = 0x03,
    CR_4_8 = 0x04,
    CR_LI_4_5 = 0x05,
    CR_LI_4_6 = 0x06,
    CR_LI_4_7 = 0x07,
}

#[repr(u8)]
pub enum LoRaPacketLengthsModes
{
    VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
    FIXED_LENGTH = 0x80,    //!< The packet is known on both sides, no header included in the packet
    EXPLICIT = SX1280_LORA_PACKET_VARIABLE_LENGTH,
    IMPLICIT = SX1280_LORA_PACKET_FIXED_LENGTH,
}

#[repr(u8)]
pub enum LoRaCrcModes
{
    ON = 0x20,  //!< CRC activated
    OFF = 0x00, //!< CRC not used
}

/// Represents the bandwidth values for FLRC packet type
#[repr(u8)]
pub enum FlrcBandwidths
{
    BR_1_300_BW_1_2 = 0x45,
    BR_1_000_BW_1_2 = 0x69,
    BR_0_650_BW_0_6 = 0x86,
    BR_0_520_BW_0_6 = 0xAA,
    BR_0_325_BW_0_3 = 0xC7,
    BR_0_260_BW_0_3 = 0xEB,
}

/// brief Represents the coding rate values for FLRC packet type
#[repr(u8)]
pub enum FlrcCodingRates
{
   CR_1_2 = 0x00,
    CR_3_4 = 0x02,
    CR_1_0 = 0x04,
}

/// brief Represents the Gaussian filter value in FLRC packet types
#[repr(u8)]
pub enum FlrcGaussianFilter
{
    BT_DIS  = 0x00,
    BT_1    = 0x10,
    BT_0_5  = 0x20,
}

#[repr(u8)]
pub enum RadioFlrcSyncWordLen
{
   NOSYNC        = 0x00,
    WORD_LEN_P32S = 0x04,
}

#[repr(u8)]
pub enum RadioFlrcSyncWordCombination
{
    DISABLE_SYNC_WORD     = 0x00,
    MATCH_SYNC_WORD_1     = 0x10,
    MATCH_SYNC_WORD_2     = 0x20,
    MATCH_SYNC_WORD_1_2   = 0x30,
    MATCH_SYNC_WORD_3     = 0x40,
    MATCH_SYNC_WORD_1_3   = 0x50,
    MATCH_SYNC_WORD_2_3   = 0x60,
    MATCH_SYNC_WORD_1_2_3 = 0x70,
}

#[repr(u8)]
pub enum RadioFlrcPacketType
{
    FIXED_LENGTH    = 0x00,
    VARIABLE_LENGTH = 0x20,
}

#[repr(u8)]
pub enum FlrcCrc
{
    CRC_OFF    = 0x00,
    CRC_1_BYTE = 0x10,
    CRC_2_BYTE = 0x20,
    CRC_3_BYTE = 0x30,
}

#[repr(u8)]
pub enum ErrorPacketStatus
{
    // Error Packet Status
    BUSY      = 1 << 0,
    PKT_RCVD  = 1 << 1,
    HDR_RCVD  = 1 << 2,
    ABORT     = 1 << 3,
    CRC       = 1 << 4,
    LENGTH    = 1 << 5,
    SYNC      = 1 << 6,
}

#[repr(u8)]
pub enum Commands
{
    GET_STATUS = 0xC0,
    WRITE_REGISTER = 0x18,
    READ_REGISTER = 0x19,
    WRITE_BUFFER = 0x1A,
    READ_BUFFER = 0x1B,
    SET_SLEEP = 0x84,
    SET_STANDBY = 0x80,
    SET_FS = 0xC1,
    SET_TX = 0x83,
    SET_RX = 0x82,
    SET_RXDUTYCYCLE = 0x94,
    SET_CAD = 0xC5,
    SET_TXCONTINUOUSWAVE = 0xD1,
    SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SET_PACKETTYPE = 0x8A,
    GET_PACKETTYPE = 0x03,
    SET_RFFREQUENCY = 0x86,
    SET_TXPARAMS = 0x8E,
    SET_CADPARAMS = 0x88,
    SET_BUFFERBASEADDRESS = 0x8F,
    SET_MODULATIONPARAMS = 0x8B,
    SET_PACKETPARAMS = 0x8C,
    GET_RXBUFFERSTATUS = 0x17,
    GET_PACKETSTATUS = 0x1D,
    GET_RSSIINST = 0x1F,
    SET_DIOIRQPARAMS = 0x8D,
    GET_IRQSTATUS = 0x15,
    CLR_IRQSTATUS = 0x97,
    CALIBRATE = 0x89,
    SET_REGULATORMODE = 0x96,
    SET_SAVECONTEXT = 0xD5,
    SET_AUTOTX = 0x98,
    SET_AUTOFS = 0x9E,
    SET_LONGPREAMBLE = 0x9B,
    SET_UARTSPEED = 0x9D,
    SET_RANGING_ROLE = 0xA3,
}

#[repr(u16)]
pub enum IrqMasks
{
    RADIO_NONE = 0x0000,
    TX_DONE = 0x0001,
    RX_DONE = 0x0002,
    SYNCWORD_VALID = 0x0004,
    SYNCWORD_ERROR = 0x0008,
    HEADER_VALID = 0x0010,
    HEADER_ERROR = 0x0020,
    CRC_ERROR = 0x0040,
    RANGING_SLAVE_RESPONSE_DONE = 0x0080,
    RANGING_SLAVE_REQUEST_DISCARDED = 0x0100,
    RANGING_MASTER_RESULT_VALID = 0x0200,
    RANGING_MASTER_TIMEOUT = 0x0400,
    RANGING_SLAVE_REQUEST_VALID = 0x0800,
    CAD_DONE = 0x1000,
    CAD_DETECTED = 0x2000,
    RX_TX_TIMEOUT = 0x4000,
    PREAMBLE_DETECTED = 0x8000,
    RADIO_ALL = 0xFFFF,
}

#[repr(u8)]
pub enum Dios
{
    DIO1 = 0x02,
    DIO2 = 0x04,
    DIO3 = 0x08,
}

#[repr(u8)]
pub enum TickSizes
{
    SIZE_0015_US = 0x00,
    SIZE_0062_US = 0x01,
    SIZE_1000_US = 0x02,
    SIZE_4000_US = 0x03,
}