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

// `crsf_protocol.h` here:

const BAUDRATE: u16 = 420000;

const SYNC_BYTE: u8 = 0xC8;

const FRAME_SIZE_MAX: usize = 64; // 62 bytes frame plus 2 bytes frame header(<length><type>)
const PAYLOAD_SIZE_MAX: usize = FRAME_SIZE_MAX - 6;

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
enum CrsfAddress {
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
    RECEIVER = 0xEC,
    TRANSMITTER = 0xEE,
}

// `crsf.h` here:'

const USE_CRSF_V3: bool = false; // todo: True eventually; newer version.

// todo: Check teh types of these. Should all or most ints be u16?

const PORT_OPTIONS: u8 = (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);
const PORT_MODE: u8 = MODE_RXTX;

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

type timeUs = u32; // todo: Is this what we want?

/* For documentation purposes
typedef enum {
    RF_MODE_4_FPS = 0,
    RF_MODE_50_FPS,
    RF_MODE_150_FPS,
} crsfRfMode_e;
*/

struct CrsfFrameDef {
    device_address: CrsfAddress,
    frame_length: u8,
    type_: FrameType,
    payload: [u8; PAYLOAD_SIZE_MAX + 1], // +1 for CRC at end of payload
}

struct CrsfFrame {
    bytes: [u8; FRAME_SIZE_MAX],
    frame: CrsfFrameDef,
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
static mut FRAME: CrsfFrame = csrfFrame {
    FRAME_SIZE_MAX: [0; bytes],
    frame: crsfFrameDef,
};
static mut CHANNEL_DATA_FRAME: CrsfFrame = csrfFrame {
    FRAME_SIZE_MAX: [0; bytes],
    frame: crsfFrameDef,
};
static mut CHANNEL_DATA: [u32; MAX_CHANNEL] = [0; MAX_CHANNEL];

static mut FRAME_START_AT_US: timeUs = 0;
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
    chanu8: u8,
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
    downlink_RSSI_1_percentage: u8,
    downlink_Link_quality: u8,
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

static mut LAST_LINK_STATISTICS_FRAME_US: timeUs = 0;

fn handle_link_statistics_frame(statsPtr: &crsfLinkStatistics, currentTimeUs: timeUs) {
    let stats: crsfLinkStatistics = *statsPtr;
    unsafe { LAST_LINK_STATISTICS_FRAME_US = currentTimeUs };
    let mut rssiDbm: i16 = -1
        * (if stats.active_antenna {
            stats.uplink_RSSI_2
        } else {
            stats.uplink_RSSI_1
        });
    if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF {
        if rxConfig().crsf_use_rx_snr {
            // -10dB of SNR mapped to 0 RSSI (fail safe is likely to happen at this measure)
            //   0dB of SNR mapped to 20 RSSI (default alarm)
            //  41dB of SNR mapped to 99 RSSI (SNR can climb to around 60, but showing that is not very meaningful)
            let rsnrPercentScaled: u16 = constrain((stats.uplink_SNR + 10) * 20, 0, RSSI_MAX_VALUE);
            setRssi(rsnrPercentScaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);

            if USE_RX_RSSI_DBM {
                rssiDbm = stats.uplink_SNR;
            }
        } else {
            let rssiPercentScaled: u16 = scaleRange(rssiDbm, RSSI_MIN, 0, 0, RSSI_MAX_VALUE);
            setRssi(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);
        }
    }
    if USE_RX_RSSI_DBM {
        setRssiDbm(rssiDbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    }

    if USE_RX_LINK_QUALITY_INFO {
        if linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF {
            setLinkQualityDirect(stats.uplink_Link_quality);
            rxSetRfMode(stats.rf_Mode);
        }
    }

    if USE_RX_LINK_UPLINK_POWER {
        let crsfUplinkPowerStatesItemIndex: u8 =
            if stats.uplink_TX_Power < UPLINK_POWER_LEVEL_MW_ITEMS_COUNT {
                stats.uplink_TX_Power
            } else {
                0
            };
        rxSetUplinkTxPwrMw(UPLINK_TX_POWER_STATES_MW[crsfUplinkPowerStatesItemIndex]);
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
fn handle_link_statistics_tx_frame(statsPtr: &crsfLinkStatisticsTx, currentTimeUs: timeUs) {
    let stats: crsfLinkStatisticsTx = *statsPtr;
    unsafe { LAST_LINK_STATISTICS_FRAME_US = currentTimeUs };
    if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF {
        let rssiPercentScaled: u16 =
            scaleRange(stats.uplink_RSSI_percentage, 0, 100, 0, RSSI_MAX_VALUE);
        setRssi(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    }
    if USE_RX_RSSI_DBM {
        let mut rssiDbm: i16 = -1 * stats.uplink_RSSI;
        if rxConfig().crsf_use_rx_snr {
            rssiDbm = stats.uplink_SNR;
        }
        setRssiDbm(rssiDbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
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
fn check_rssi(currentTimeUs: u32) {
    if cmpTimeUs(currentTimeUs, unsafe { LAST_LINK_STATISTICS_FRAME_US })
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
    let mut crc: u8 = crc8_dvb_s2(0, unsafe { FRAME.frame.type_ });

    unsafe {
        for ii in 0..CRSF_FRAME.frame.frame_length - FRAME_LENGTH_TYPE_CRC {
            crc = crc8_dvb_s2(crc, FRAME.frame.payload[ii]);
        }
    }
    crc
}

fn frame_cmd_crc() -> u8 {
    // CRC includes type and payload
    let mut crc: u8 = crc8_poly_0xba(0, unsafe { FRAME.frame.type_ });

    unsafe {
        for ii in 0..CRSF_FRAME.frame.frame_length - FRAME_LENGTH_TYPE_CRC - 1 {
            crc = crc8_poly_0xba(crc, FRAME.frame.payload[ii]);
        }
    }
    crc
}

// Receive ISR callback, called back from serial port
// fn crsfDataReceive(c: u16, void *data)
fn crsfDataReceive(c: u16) {
    let rxRuntimeState: rxRuntimeState = data as &[rxRuntimeState];

    // let mut crsfFramePosition: u8 = 0; // todo: As u8 in original, but usize makes more sens?
    let mut frame_position = 0;

    let mut crsfFrameErrorCnt: u8 = 0; // For V3

    let current_time_us: timeUs = 0; // todo: This calls a timer or RTC etc.

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
                let crc: u8 = frame_crc();
                if crc == FRAME.bytes[full_frame_length - 1] {
                    if USE_CRSF_V3 {
                        crsfFrameErrorCnt = 0;
                    }

                    match FRAME.frame.type_ {
                        FrameType::RC_CHANNELS_PACKED => (),
                        FrameType::SUBSET_RC_CHANNELS_PACKED => {
                            if FRAME.frame.device_address == CrsfAddress::FLIGHT_CONTROLLER {
                                rxRuntimeState.lastRcFrameTimeUs = current_time_us;
                                FRAME_DONE = true;
                                memcpy(&CHANNEL_DATA_FRAME, &CRSF_FRAME, sizeof(CRSF_FRAME));
                            }
                        }

                        // if USE_TELEMETRY_CRSF && USE_MSP_OVER_TELEMETRY {
                        FrameType::MSP_REQ => (),
                        FrameType::MSP_WRITE => {
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
                        FrameType::DISPLAYPORT_CMD => {
                            let frame_start: u8 =
                                &CRSF_FRAME.frame.payload as u8 + FRAME_ORIGIN_DEST_SIZE;
                            crsfProcessDisplayPortCmd(frame_start);
                        }
                        // }
                        // #if defined(USE_CRSF_LINK_STATISTICS)
                        FrameType::LINK_STATISTICS => {
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
                        FrameType::LINK_STATISTICS_RX => (),
                        FrameType::LINK_STATISTICS_TX => {
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
                        if crsfFrameErrorCnt < FRAME_ERROR_COUNT_THRESHOLD {
                            crsfFrameErrorCnt += 1;
                        }
                    }
                }
            } else {
                if USE_CRSF_V3 {
                    if crsfFrameErrorCnt < FRAME_ERROR_COUNT_THRESHOLD {
                        crsfFrameErrorCnt += 1;
                    }
                }
            }
            if USE_CRSF_V3 {
                if crsfFrameErrorCnt >= FRAME_ERROR_COUNT_THRESHOLD {
                    // fall back to default speed if speed mismatch detected
                    setCrsfDefaultSpeed();
                    crsfFrameErrorCnt = 0;
                }
            }
        }
    }
}

fn frame_status() -> u8 {
    if USE_CRSF_LINK_STATISTICS {
        check_rssi(micros());
    }

    unsafe {
        if FRAME_DONE {
            FRAME_DONE = false;

            // unpack the RC channels
            if CHANNEL_DATA_FRAME.frame.type_ == FrameType::RC_CHANNELS_PACKED {
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
            return RX_FRAME_COMPLETE;
        }
        return RX_FRAME_PENDING;
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
            crsfDataReceive,
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
