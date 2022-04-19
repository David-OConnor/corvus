#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! This module contains code for use with the Crossfire protocol, over UART. Also used
//! with ELRS receiver modules, which communicate with the MCU using CRSF.
//! Translated from the betaflight impl here:

//! https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h
//! https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.h
//! https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c
//!
//! https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/crsf.h
//! https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/crsf.c


// todo We've commented out the CRSF-V3 feature-gated code, since ELRS uses V2 only currently.
// todo: Make sure you put back before V3 is supported in ELRS!

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
// `crsf_protocol.h`
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
 * Crossfire constants provided by Team Black Sheep under terms of the 2-Clause BSD License
 */

use std::mem;

mod bf_rx;
mod rc_modes;

use bf_rx::*;

use stm32_hal2::{
    usart::Usart,
    pac::{USART3}
};

// todo: QC these types.
const BAUDRATE: u32 = 420000;

const SYNC_BYTE: u8 = 0xC8;

const FRAME_SIZE_MAX: usize = 64; // 62 bytes frame plus 2 bytes frame header(<length><type>)
const PAYLOAD_SIZE_MAX: usize = FRAME_SIZE_MAX - 6;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum FrameType {
    GPS = 0x02,
    BATTERY_SENSOR = 0x08,
    LINK_STATISTICS = 0x14,
    RC_CHANNELS_PACKED = 0x16,
    SUBSET_RC_CHANNELS_PACKED = 0x17,
    LINK_STATISTICS_RX = 0x1C,
    LINK_STATISTICS_TX = 0x1D,
    ATTITUDE = 0x1E,
    FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    DEVICE_PING = 0x28,
    DEVICE_INFO = 0x29,
    PARAMETER_SETTINGS_ENTRY = 0x2B,
    PARAMETER_READ = 0x2C,
    PARAMETER_WRITE = 0x2D,
    COMMAND = 0x32,
    // MSP commands
    MSP_REQ = 0x7A,         // response request using msp sequence as command
    MSP_RESP = 0x7B,        // reply with 58 byte chunked binary
    MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    DISPLAYPORT_CMD = 0x7D, // displayport control command
}

const COMMAND_SUBCMD_GENERAL: u8 = 0x0A; // general command

#[derive(Clone, Copy)]
#[repr(u8)]
enum Speed {
    PROPOSAL = 0x70, // proposed new CRSF port speed
    RESPONSE = 0x71, // response to the proposed CRSF port speed
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum SubCmd {
    UPDATE = 0x01, // transmit displayport buffer to remote
    CLEAR = 0x02,  // clear client screen
    OPEN = 0x03,   // client request to open cms menu
    CLOSE = 0x04,  // client request to close cms menu
    POLL = 0x05,   // client request to poll/refresh cms menu
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Offset {
    ROWS = 1,
    COLS = 2,
}

// todo type?
const FRAME_GPS_PAYLOAD_SIZE: u8 = 15;
const FRAME_BATTERY_SENSOR_PAYLOAD_SIZE: u8 = 8;
const FRAME_LINK_STATISTICS_PAYLOAD_SIZE: u8 = 10;
const FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE: u8 = 6;
const FRAME_RC_CHANNELS_PAYLOAD_SIZE: u8 = 22; // 11 bits per channel * 16 channels = 22 bytes.
const FRAME_ATTITUDE_PAYLOAD_SIZE: u8 = 6;

// todo type?
const FRAME_LENGTH_ADDRESS: u8 = 1; // length of ADDRESS field
const FRAME_LENGTH_FRAMELENGTH: u8 = 1; // length of FRAMELENGTH field
const FRAME_LENGTH_TYPE: u8 = 1; // length of TYPE field
const FRAME_LENGTH_CRC: u8 = 1; // length of CRC field
const FRAME_LENGTH_TYPE_CRC: u8 = 2; // length of TYPE and CRC fields combined
const FRAME_LENGTH_EXT_TYPE_CRC: u8 = 4; // length of Extended Dest/Origin; TYPE and CRC fields combined
const FRAME_LENGTH_NON_PAYLOAD: u8 = 4; // combined length of all fields except payload

// todo type?
const FRAME_TX_MSP_FRAME_SIZE: u8 = 58;
const FRAME_RX_MSP_FRAME_SIZE: u8 = 8;
const FRAME_ORIGIN_DEST_SIZE: u8 = 2;

// Clashes with CRSF_ADDRESS_FLIGHT_CONTROLLER
// const CRSF_SYNC_BYTE: u8 = 0xC8; // (prev defined above)

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum Address {
    BROADCAST = 0x00,
    USB = 0x10,
    TBS_CORE_PNP_PRO = 0x80,
    RESERVED1 = 0x8A,
    CURRENT_SENSOR = 0xC0,
    GPS = 0xC2,
    TBS_BLACKBOX = 0xC4,
    FLIGHT_CONTROLLER = 0xC8,
    RESERVED2 = 0xCA,
    RACE_TAG = 0xCC,
    RADIO_TRANSMITTER = 0xEA,
    CRSF_RECEIVER = 0xEC,
    CRSF_TRANSMITTER = 0xEE,
}

// `crsf_bf.h`

// todo: All these consts below are defines, so check their type!
// const PORT_OPTIONS: u8 = (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);
// const PORT_MODE: u8 = MODE_RXTX;

const MAX_CHANNEL: usize = 16;
// const CRSFV3_MAX_CHANNEL: u8 =       24;

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

const RSSI_MIN: i8 = -130;
const RSSI_MAX: u8 = 0;
const SNR_MIN: i8 = -30;
const SNR_MAX: u8 = 20;

/* For documentation purposes
typedef enum {
    CRSF_RF_MODE_4_FPS = 0,
    CRSF_RF_MODE_50_FPS,
    CRSF_RF_MODE_150_FPS,
} crsfRfMode_e;
*/

#[derive(Clone, Copy)] // required to be used in a union.
struct FrameDef {
    pub deviceAddress: u8,
    pub frameLength: u8,
    pub type_: u8,
    pub payload: [u8; PAYLOAD_SIZE_MAX + 1], // +1 for CRC at end of payload
}

#[repr(C)]
union Frame {
    bytes: [u8; FRAME_SIZE_MAX],
    frame: FrameDef,
}

// `crsf_bf.c`

// todo: #define type
const TIME_NEEDED_PER_FRAME_US: u16 = 1750; // a maximally sized 64byte payload will take ~1550us, round up to 1750.
const TIME_BETWEEN_FRAMES_US: u16 = 6667; // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

const DIGITAL_CHANNEL_MIN: u16 = 172;
const DIGITAL_CHANNEL_MAX: u16 = 1811;

// const PAYLOAD_OFFSET: usize = offsetof(crsfFrameDef_t, type)

const LINK_STATUS_UPDATE_TIMEOUT_US: u32 = 250000; // 250ms, 4 Hz mode 1 telemetry

const FRAME_ERROR_COUNT_THRESHOLD: u8 = 100;

static mut crsfFrameDone: bool = false;
static mut crsfFrame: CrsfFrame = {};
static mut crsfChannelDataFrame: CrsfFrame = {};
static mut crsfChannelData: [u32; MAX_CHANNEL] = [0; MAX_CHANNEL];

// static mut serialPort: SerialPort = ?;
static mut crsfFrameStartAtUs: timeUs = 0;
static mut telemetryBuf: [u8; FRAME_SIZE_MAX] = [0; FRAME_SIZE_MAX];
static mut telemetryBufLen: u8 = 0;
static mut channelScale: f32 = RC_CHANNEL_SCALE_LEGACY;

// #ifdef USE_RX_LINK_UPLINK_POWER
const UPLINK_POWER_LEVEL_MW_ITEMS_COUNT: usize = 9; // todo #define what type?
                                                         // Uplink power levels by uplinkTXPower expressed in mW (250 mW is from ver >=4.00, 50 mW in a future version and for ExpressLRS)
const uplinkTXPowerStatesMw: [u16; UPLINK_POWER_LEVEL_MW_ITEMS_COUNT] =
    [0, 10, 25, 100, 500, 1000, 2000, 250, 50];
// #endif

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
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
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

// struct crsfPayloadRcChannelsPacked_s {
//     // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
//     unsigned int chan0 : 11;
//     unsigned int chan1 : 11;
//     unsigned int chan2 : 11;
//     unsigned int chan3 : 11;
//     unsigned int chan4 : 11;
//     unsigned int chan5 : 11;
//     unsigned int chan6 : 11;
//     unsigned int chan7 : 11;
//     unsigned int chan8 : 11;
//     unsigned int chan9 : 11;
//     unsigned int chan10 : 11;
//     unsigned int chan11 : 11;
//     unsigned int chan12 : 11;
//     unsigned int chan13 : 11;
//     unsigned int chan14 : 11;
//     unsigned int chan15 : 11;
// } __attribute__ ((__packed__));

// todo: Make sure this mapping works out. It's a bit dif.
struct PayloadRcChannelsPacked {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    // u32, even though 11 bits, since that's what we cast it into.
    pub chan0: u32,
    pub chan1: u32,
    pub chan2: u32,
    pub chan3: u32,
    pub chan4: u32,
    pub chan5: u32,
    pub chan6: u32,
    pub chan7: u32,
    pub chan8: u32,
    pub chan9: u32,
    pub chan10: u32,
    pub chan11: u32,
    pub chan12: u32,
    pub chan13: u32,
    pub chan14: u32,
    pub chan15: u32,
}

// typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

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

struct Linkstatistics {
    pub uplink_RSSI_1: u8,
    pub uplink_RSSI_2: u8,
    pub uplink_Link_quality: u8,
    pub uplink_SNR: i8,
    pub active_antenna: u8,
    pub rf_Mode: u8,
    pub uplink_TX_Power: u8,
    pub downlink_RSSI: u8,
    pub downlink_Link_quality: u8,
    pub downlink_SNR: i8,
}

// #if defined(USE_CRSF_V3)
struct LinkStatisticsRx {
    pub downlink_RSSI_1: u8,
    pub downlink_RSSI_1_percentage: u8,
    pub downlink_Link_quality: u8,
    pub downlink_SNR: i8,
    pub uplink_power: u8,
} // this struct is currently not used

struct LinkStatisticsTx {
    pub uplink_RSSI: u8,
    pub uplink_RSSI_percentage: u8,
    pub uplink_Link_quality: u8,
    pub uplink_SNR: i8,
    pub downlink_power: u8, // currently not used
    pub uplink_FPS: u8, // currently not used
}
// #endif

static mut lastLinkStatisticsFrameUs: timeUs = 0;

fn handleCrsfLinkStatisticsFrame(stats: &LinkStatistics, currentTimeUs: timeUs) {
    unsafe {
        lastLinkStatisticsFrameUs = currentTimeUs;
    }
    let mut rssiDbm: i16 = -1
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
            let rsnrPercentScaled: u16 = constrain((stats.uplink_SNR + 10) * 20, 0, RSSI_MAX_VALUE);
            setRssi(rsnrPercentScaled, RssiSource::RX_PROTOCOL_CRSF);
            // #ifdef USE_RX_RSSI_DBM
            rssiDbm = stats.uplink_SNR;
        // #endif
        } else {
            let rssiPercentScaled: u16 = scaleRange(rssiDbm, RSSI_MIN, 0, 0, RSSI_MAX_VALUE);
            setRssi(rssiPercentScaled, RssiSource::RX_PROTOCOL_CRSF);
        }
    }
    // #ifdef USE_RX_RSSI_DBM
    setRssiDbm(rssiDbm, RssiSource::RX_PROTOCOL_CRSF);
    // #endif

    // #ifdef USE_RX_LINK_QUALITY_INFO
    if linkQualitySource == LinkQualitySource::RX_PROTOCOL_CRSF {
        setLinkQualityDirect(stats.uplink_Link_quality);
        rxSetRfMode(stats.rf_Mode);
    }
    // #endif

    // #ifdef USE_RX_LINK_UPLINK_POWER
    let crsfUplinkPowerStatesItemIndex: u8 =
        if stats.uplink_TX_Power < UPLINK_POWER_LEVEL_MW_ITEMS_COUNT {
            stats.uplink_TX_Power
        } else {
            0
        };
    rxSetUplinkTxPwrMw(uplinkTXPowerStatesMw[crsfUplinkPowerStatesItemIndex as usize]);
    // #endif

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

// #if defined(USE_CRSF_V3)
fn handleCrsfLinkStatisticsTxFrame(stats: &LinkStatisticsTx, currentTimeUs: timeUs)
{
    unsafe { lastLinkStatisticsFrameUs = currentTimeUs; }
    if rssiSource == RssiSource::RX_PROTOCOL_CRSF {
        let rssiPercentScaled: u16 = scaleRange(stats.uplink_RSSI_percentage, 0, 100, 0, RSSI_MAX_VALUE);
        setRssi(rssiPercentScaled, RssiSource::RX_PROTOCOL_CRSF);
    }
// #ifdef USE_RX_RSSI_DBM
    let mut rssiDbm: i16 = -1 * stats.uplink_RSSI as i16;
    if rxConfig().crsf_use_rx_snr {
        rssiDbm = stats.uplink_SNR as i16;
    }
    setRssiDbm(rssiDbm, RssiSource::RX_PROTOCOL_CRSF);
// #endif

// #ifdef USE_RX_LINK_QUALITY_INFO
    if linkQualitySource == LinkQualitySource::RX_PROTOCOL_CRSF {
        setLinkQualityDirect(stats.uplink_Link_quality);
    }
// #endif
//
//     // todo?
//     // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI);
//     // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_SNR);
//     // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
//     // DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.uplink_RSSI_percentage);
// }
// #endif
// #endif

// #if defined(USE_CRSF_LINK_STATISTICS)
fn crsfCheckRssi(currentTimeUs: u32) {
    if cmpTimeUs(currentTimeUs, unsafe { lastLinkStatisticsFrameUs })
        > LINK_STATUS_UPDATE_TIMEOUT_US
    {
        if rssiSource == RssiSource::RX_PROTOCOL_CRSF {
            setRssiDirect(0, RssiSource::RX_PROTOCOL_CRSF);
            // #ifdef USE_RX_RSSI_DBM
            if rxConfig().crsf_use_rx_snr {
                setRssiDbmDirect(SNR_MIN, RssiSource::RX_PROTOCOL_CRSF);
            } else {
                setRssiDbmDirect(RSSI_MIN, RssiSource::RX_PROTOCOL_CRSF);
            }
            // #endif
        }
        // #ifdef USE_RX_LINK_QUALITY_INFO
        if linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF {
            setLinkQualityDirect(0);
        }
        // #endif
    }
}
// #endif

fn crsfFrameCRC() -> u8 {
    // CRC includes type and payload
    unsafe {
        let mut crc: u8 = crc8_dvb_s2(0, crsfFrame.frame_type);
        for i in 0..crsfFrame.frame.frameLength - FRAME_LENGTH_TYPE_CRC {
            crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[i]);
        }

        crc
    }
}

fn crsfFrameCmdCRC() -> u8 {
    // CRC includes type and payload
    unsafe {
        let mut crc: u8 = crc8_poly_0xba(0, crsfFrame.frame_type);
        for i in 0..crsfFrame.frame.frameLength - FRAME_LENGTH_TYPE_CRC - 1 {
            crc = crc8_poly_0xba(crc, crsfFrame.frame.payload[i]);
        }
        crc
    }
}

// Receive ISR callback, called back from serial port
unsafe fn crsfDataReceive(c: u16, data: RxRuntimeState) // todo: void data
{
    let rxRuntimeState: RxRuntimeState = data;

    static mut crsfFramePosition: u8 = 0;
    // #if defined(USE_CRSF_V3)
    // static mut crsfFrameErrorCnt: u8 = 0;
    // #endif
    let currentTimeUs: timeUs = microsISR();

    // #ifdef DEBUG_CRSF_PACKETS
    //     debug[2] = currentTimeUs - crsfFrameStartAtUs;
    // #endif

    if cmpTimeUs(currentTimeUs, crsfFrameStartAtUs) > TIME_NEEDED_PER_FRAME_US {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0;
    }

    if crsfFramePosition == 0 {
        crsfFrameStartAtUs = currentTimeUs;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    // sometimes we can receive some garbage data. So, we need to check max size for preventing buffer overrun.
    // todo: Int type. i32?
    let fullFrameLength = if crsfFramePosition < 3 {
        5
    } else {
        (
            crsfFrame.frame.frameLength + FRAME_LENGTH_ADDRESS + FRAME_LENGTH_FRAMELENGTH,
            FRAME_SIZE_MAX,
        )
            .min()
    };

    if crsfFramePosition < fullFrameLength {
        crsfFramePosition += 1;
        crsfFrame.bytes[crsfFramePosition] = c as u8;
        if crsfFramePosition >= fullFrameLength {
            crsfFramePosition = 0;
            let crc: uu8 = crsfFrameCRC();
            if crc == crsfFrame.bytes[fullFrameLength - 1] {
                // #if defined(USE_CRSF_V3)
                //                 crsfFrameErrorCnt = 0;
                // #endif
                match crsfFrame.frame_type {
                    FrameType::RC_CHANNELS_PACKED | FrameType::SUBSET_RC_CHANNELS_PACKED => {
                        if crsfFrame.frame.deviceAddress == Address::FLIGHT_CONTROLLER {
                            rxRuntimeState.lastRcFrameTimeUs = currentTimeUs;
                            crsfFrameDone = true;
                            memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
                        }
                    }

                    // #if defined(USE_TELEMETRY_CRSF) && defined(USE_MSP_OVER_TELEMETRY)
                    FrameType::MSP_REQ | FrameType::MSP_WRITE => {
                        let frameStart: [u8; 69] =
                            crsfFrame.frame.payload + FRAME_ORIGIN_DEST_SIZE;
                        if bufferCrsfMspFrame(frameStart, crsfFrame.frame.frameLength - 4) {
                            crsfScheduleMspResponse(crsfFrame.frame.payload[1]);
                        }
                    }
                    // #endif
                    // #if defined(USE_CRSF_CMS_TELEMETRY)
                    FrameType::DEVICE_PING => {
                        crsfScheduleDeviceInfoResponse();
                    }
                    FrameType::DISPLAYPORT_CMD => {
                        let frameStart: [u8; 69] =
                            crsfFrame.frame.payload + FRAME_ORIGIN_DEST_SIZE;
                        crsfProcessDisplayPortCmd(frameStart);
                    }
                    // #endif
                    // #if defined(USE_CRSF_LINK_STATISTICS)
                    FrameType::LINK_STATISTICS => {
                        // if to FC and 10 bytes + FRAME_ORIGIN_DEST_SIZE
                        if rssiSource == RssiSource::RX_PROTOCOL_CRSF
                            && crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER
                            && crsfFrame.frame.frameLength
                                == FRAME_ORIGIN_DEST_SIZE
                                    + FRAME_LINK_STATISTICS_PAYLOAD_SIZE
                        {
                            let statsFrame: Linkstatistics = crsfFrame.frame.payload;
                            handleCrsfLinkStatisticsFrame(&statsFrame, currentTimeUs);
                        }
                    }
                    // #if defined(USE_CRSF_V3)
                    //                     FrameType::LINK_STATISTICS_RX => (),
                    //                     FrameType::LINK_STATISTICS_TX => {
                    //                         if rssiSource == RssiSource::RX_PROTOCOL_CRSF &&
                    //                             crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                    //                             crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE {
                    //                             let statsFrame: LinkStatisticsTx = crsfFrame.frame.payload;
                    //                             handleCrsfLinkStatisticsTxFrame(&statsFrame, currentTimeUs);
                    //                         }
                    //                     }
                    // #endif
                    // #endif
                    // #if defined(USE_CRSF_V3)
                    //                     FrameType::COMMAND => {
                    //                         if crsfFrame.bytes[fullFrameLength - 2] == crsfFrameCmdCRC() &&
                    //                             crsfFrame.bytes[3] == CRSF_ADDRESS_FLIGHT_CONTROLLER {
                    //                             crsfProcessCommand(crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE);
                    //                         }
                    //                     }
                    // #endif
                    () => (),
                }
            } else {
                // #if defined(USE_CRSF_V3)
                //                 if crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD {
                //                     crsfFrameErrorCnt += 1;
                //                 }
                // #endif
            }
        } else {
            // #if defined(USE_CRSF_V3)
            //             if crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD {
            //                 crsfFrameErrorCnt += 1;
            //             }
            // #endif
        }
        // #if defined(USE_CRSF_V3)
        //         if crsfFrameErrorCnt >= CRSF_FRAME_ERROR_COUNT_THRESHOLD {
        // fall back to default speed if speed mismatch detected
        //             setCrsfDefaultSpeed();
        //             crsfFrameErrorCnt = 0;
        //         }
        // #endif
    }
}

unsafe fn crsfFrameStatus(rxRuntimeState: &RxRuntimeState) -> u8 {
    // UNUSED(rxRuntimeState);

    // #if defined(USE_CRSF_LINK_STATISTICS)
    crsfCheckRssi(micros());
    // #endif
    if crsfFrameDone {
        crsfFrameDone = false;

        // unpack the RC channels
        if crsfChannelDataFrame.frame_type == FrameType::RC_CHANNELS_PACKED {
            // use ordinary RC frame structure (0x16)
            let rcChannels: PayloadRcChannelsPacked = crsfChannelDataFrame.frame.payload;

            channelScale = RC_CHANNEL_SCALE_LEGACY;
            crsfChannelData[0] = rcChannels.chan0;
            crsfChannelData[1] = rcChannels.chan1;
            crsfChannelData[2] = rcChannels.chan2;
            crsfChannelData[3] = rcChannels.chan3;
            crsfChannelData[4] = rcChannels.chan4;
            crsfChannelData[5] = rcChannels.chan5;
            crsfChannelData[6] = rcChannels.chan6;
            crsfChannelData[7] = rcChannels.chan7;
            crsfChannelData[8] = rcChannels.chan8;
            crsfChannelData[9] = rcChannels.chan9;
            crsfChannelData[10] = rcChannels.chan10;
            crsfChannelData[11] = rcChannels.chan11;
            crsfChannelData[12] = rcChannels.chan12;
            crsfChannelData[13] = rcChannels.chan13;
            crsfChannelData[14] = rcChannels.chan14;
            crsfChannelData[15] = rcChannels.chan15;
        } else {
            // use subset RC frame structure (0x17)
            let mut readByteIndex: u8 = 0;
            let payload: [u8; 69] = crsfChannelDataFrame.frame.payload;

            // get the configuration byte
            readByteIndex += 1;
            let mut configByte: u8 = payload[readByteIndex as usize];

            // get the channel number of start channel
            let startChannel: u8 = configByte & SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            let mut channelBits: u8 = 0;
            let mut channelMask: u16 = 0;
            let mut channelRes: u8 = configByte & SUBSET_RC_RES_CONFIGURATION_MASK;

            configByte >>= SUBSET_RC_RES_CONFIGURATION_BITS;
            match channelRes {
                SUBSET_RC_RES_CONF_10B => {
                    channelBits = SUBSET_RC_RES_BITS_10B;
                    channelMask = SUBSET_RC_RES_MASK_10B;
                    channelScale = SUBSET_RC_CHANNEL_SCALE_10B;
                }
                // default:
                SUBSET_RC_RES_CONF_11B => {
                    channelBits = SUBSET_RC_RES_BITS_11B;
                    channelMask = SUBSET_RC_RES_MASK_11B;
                    channelScale = SUBSET_RC_CHANNEL_SCALE_11B;
                }
                SUBSET_RC_RES_CONF_12B => {
                    channelBits = SUBSET_RC_RES_BITS_12B;
                    channelMask = SUBSET_RC_RES_MASK_12B;
                    channelScale = SUBSET_RC_CHANNEL_SCALE_12B;
                }
                SUBSET_RC_RES_CONF_13B => {
                    channelBits = SUBSET_RC_RES_BITS_13B;
                    channelMask = SUBSET_RC_RES_MASK_13B;
                    channelScale = SUBSET_RC_CHANNEL_SCALE_13B;
                }
                _ => (),
            }

            // do nothing for the reserved configuration bit
            configByte >>= SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            let numOfChannels: u8 =
                ((crsfChannelDataFrame.frame.frameLength - FRAME_LENGTH_TYPE_CRC - 1) * 8)
                    / channelBits;

            // unpack the channel data
            let mut bitsMerged: u8 = 0;
            let mut readValue: u32 = 0;

            for n in 0..numOfChannels {
                while bitsMerged < channelBits {
                    readByteIndex += 1;
                    let mut readByte: u8 = payload[readByteIndex as usize];
                    readValue |= (readByte as u32) << bitsMerged as u32;
                    bitsMerged += 8;
                }
                crsfChannelData[(startChannel + n) as usize] = readValue & channelMask as u32;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
        }
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

fn crsfReadRawRC(rxRuntimeState: &RxRuntimeState, chan: u8) -> f32 {
    // UNUSED(rxRuntimeState);
    unsafe {
        if channelScale == RC_CHANNEL_SCALE_LEGACY {
            /* conversion from RC value to PWM
             * for 0x16 RC frame
             *       RC     PWM
             * min  172 ->  988us
             * mid  992 -> 1500us
             * max 1811 -> 2012us
             * scale factor = (2012-988) / (1811-172) = 0.62477120195241
             * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
             */
            (channelScale * crsfChannelData[chan as usize] as f32) + 881.
        } else {
            /* conversion from RC value to PWM
             * for 0x17 Subset RC frame
             */
            (channelScale * crsfChannelData[chan as usize] as f32) + 988.
        }
    }
}

fn crsfRxWriteTelemetryData(data: &[u8], len: usize) {
    // todo: HAL
    // len = (len, (int)sizeof(telemetryBuf)).min();
    // memcpy(telemetryBuf, data, len);
    // telemetryBufLen = len;
}

fn crsfRxSendTelemetryData() {
    // todo: HAL
    // if there is telemetry data to write
    // unsafe {
    //     if telemetryBufLen > 0 {
    //         serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
    //         telemetryBufLen = 0; // reset telemetry buffer
    //     }
    // }
}

fn crsfRxIsTelemetryBufEmpty() -> bool {
    unsafe { telemetryBufLen == 0 }
}


fn crsfRxInit(rxConfig: &RxConfig, rxRuntimeState: &RuntimeState) -> bool {
    for i in 0..CRSF_MAX_CHANNEL {
        unsafe {
            crsfChannelData[i] = (16 * rxConfig.midrc) / 10 - 1408;
        }
    }

    rxRuntimeState.channelCount = MAX_CHANNEL;
    rxRuntimeState.rxRefreshRate = TIME_BETWEEN_FRAMES_US; // TODO this needs checking

    rxRuntimeState.rcReadRawFn = crsfReadRawRC;
    rxRuntimeState.rcFrameStatusFn = crsfFrameStatus;
    rxRuntimeState.rcFrameTimeUsFn = rxFrameTimeUs;

    // todo: You probably need to process this using your HAL's UART functionality.
    // const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);

    // if !portConfig {
    //     return false;
    // }

    // serialPort = openSerialPort(portConfig.identifier,
    //                             FUNCTION_RX_SERIAL,
    //                             crsfDataReceive,
    //                             rxRuntimeState,
    //                             CRSF_BAUDRATE,
    //                             CRSF_PORT_MODE,
    //                             CRSF_PORT_OPTIONS | (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
    // );

    if rssiSource == RssiSource::NONE {
        rssiSource = RssiSource::RX_PROTOCOL_CRSF;
    }
    // #ifdef USE_RX_LINK_QUALITY_INFO
    if linkQualitySource == LinkQualitySource::NONE {
        linkQualitySource = LinkQualitySource::RX_PROTOCOL_CRSF;
    }
    // #endif

    // return serialPort != NULL;
    false
}

// #if defined(USE_CRSF_V3)
fn crsfRxUpdateBaudrate(baudrate: u32)
{
    serialSetBaudRate(serialPort, baudrate);
}

fn crsfRxUseNegotiatedBaud() -> bool
{
    rxConfig().crsf_use_negotiated_baud
}
// #endif

fn crsfRxIsActive() -> bool {
    // unsafe { serialPort != NULL } // todo?
    true
}
// #endif
