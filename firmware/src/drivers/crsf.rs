//! This module interacts with a TBS Crossfire (CRSF) protocol radio receiver, eg ExpressLRS (ELRS). This is what
//!allows the aircraft to receive commands from a handheld radio receiver, etc.
//!
//! Translated from [Betaflight's implementation](https://github.com/betaflight/betaflight/blob/b8c58abf0e4a11a8bee54b9abbaed8683c65d5d2/src/main/rx/crsf.c)
//! .h file here: https://github.com/betaflight/betaflight/blob/b8c58abf0e4a11a8bee54b9abbaed8683c65d5d2/src/main/rx/crsf.h

use stm32_hal2::{
    usart::Usart,
    pac::{USART1},
};

/// Possible ELS frequencies
#[derive(Copy, Clone)]
enum RadioFreq {
    /// 900 Hz. Requires larger antenna. Longer range
    F900,
    /// 2.4kHz. Can use a smaller antenna
    F2_400,
}



// `crsf.h` here:'


const USE_CRSF_V3: bool = false; // todo: True eventually; newer version.

// todo: Check teh types of these. Should all or most ints be u16?

const CRSF_PORT_OPTIONS   : u8 =    (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);
const CRSF_PORT_MODE   : u8 =       MODE_RXTX;

const CRSF_MAX_CHANNEL: u8 =        16;
const CRSFV3_MAX_CHANNEL : u8 =        24;

const CRSF_SUBSET_RC_STARTING_CHANNEL_BITS  : u8 =         5;
const CRSF_SUBSET_RC_STARTING_CHANNEL_MASK  : u8 =         0x1F;
const CRSF_SUBSET_RC_RES_CONFIGURATION_BITS  : u8 =        2;
const CRSF_SUBSET_RC_RES_CONFIGURATION_MASK  : u8 =        0x03;
const CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS : u8 =    1;

const CRSF_RC_CHANNEL_SCALE_LEGACY: f32 =                0.62477120195241;
const CRSF_SUBSET_RC_RES_CONF_10B    : u8 =               0;
const CRSF_SUBSET_RC_RES_BITS_10B      : u8 =              10;
const CRSF_SUBSET_RC_RES_MASK_10B    : u16  =           0x03FF;
const CRSF_SUBSET_RC_CHANNEL_SCALE_10B: f32 =             1.0;
const CRSF_SUBSET_RC_RES_CONF_11B        : u8    =     1;
const CRSF_SUBSET_RC_RES_BITS_11B  : u8    =           11;
const CRSF_SUBSET_RC_RES_MASK_11B    : u16    =         0x07FF;
const CRSF_SUBSET_RC_CHANNEL_SCALE_11B: f32 =             0.5;
const CRSF_SUBSET_RC_RES_CONF_12B     : u8     =       2;
const CRSF_SUBSET_RC_RES_BITS_12B : u8      =          12;
const CRSF_SUBSET_RC_RES_MASK_12B        : u16  =       0x0FFF;
const CRSF_SUBSET_RC_CHANNEL_SCALE_12B: f32 =             0.25;
const CRSF_SUBSET_RC_RES_CONF_13B     : u8    =        3;
const CRSF_SUBSET_RC_RES_BITS_13B      : u8   =        13;
const CRSF_SUBSET_RC_RES_MASK_13B   : u16      =        0x1FFF;
const CRSF_SUBSET_RC_CHANNEL_SCALE_13B: f32 =            0.125;

const CRSF_RSSI_MIN: u8 = -130;
const CRSF_RSSI_MAX: u8 = 0;
const CRSF_SNR_MIN: u8 = -30;
const CRSF_SNR_MAX: u8 = 20;

/* For documentation purposes
typedef enum {
    CRSF_RF_MODE_4_FPS = 0,
    CRSF_RF_MODE_50_FPS,
    CRSF_RF_MODE_150_FPS,
} crsfRfMode_e;
*/

struct crsfFrameDef_s {
    deviceAddress: u8,
    frameLength: u8,
    type_: u8,
    payload: [u8; CRSF_PAYLOAD_SIZE_MAX + 1], // +1 for CRC at end of payload
}

struct crsfFrame_u {
     CRSF_FRAME_SIZE_MAX: [u8; bytes],
     frame: crsfFrameDef_t,
}

struct rxConfig_s {}
struct rxRuntimeState_s {}


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
const  CRSF_TIME_NEEDED_PER_FRAME_US: u16 =  1750; // a maximally sized 64byte payload will take ~1550us, round up to 1750.
const  CRSF_TIME_BETWEEN_FRAMES_US : u16 =     6667 ;// At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

const  CRSF_DIGITAL_CHANNEL_MIN: u16 =  172;
const  CRSF_DIGITAL_CHANNEL_MAX: u16 =  1811;

const  CRSF_PAYLOAD_OFFSET: u16 =  offsetof(crsfFrameDef_t, type);

const  CRSF_LINK_STATUS_UPDATE_TIMEOUT_US: u16 =   250000 ;// 250ms, 4 Hz mode 1 telemetry

const  CRSF_FRAME_ERROR_COUNT_THRESHOLD: u16 =     100;

static crsfFrameDone: bool = false;
STATIC_UNIT_TESTED crsfFrame_t crsfFrame;
STATIC_UNIT_TESTED crsfFrame_t crsfChannelDataFrame;
static crsfChannelData: [u32; CRSF_MAX_CHANNEL] = [0; CRSF_MAX_CHANNEL];

static serialPort_t *serialPort;
static crsfFrameStartAtUs: timeUs_t = 0;
static telemetryBuf: [u8; CRSF_FRAME_SIZE_MAX] = [0; CRSF_FRAME_SIZE_MAX];
static telemetryBufLen: u8 = 0;
static channelScale: f32 = CRSF_RC_CHANNEL_SCALE_LEGACY;

if USE_RX_LINK_UPLINK_POWER {
   let CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT: usize = 9;

   // Uplink power levels by uplinkTXPower expressed in mW (250 mW is from ver >=4.00, 50 mW in a future version and for ExpressLRS)
   const uplinkTXPowerStatesMw: [u16; CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT] = [0, 10, 25, 100, 500, 1000, 2000, 250, 50];
}

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

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    chan0 : u8,
    chan1 : u8,
    chan2 : u8,
    chan3 : u8,
    chan4 : u8,
    chan5 : u8,
    chan6 : u8,
    chan7 : u8,
    chan8 : u8,
    chan9 : u8,
    chan10 : u8,
    chanu8 : u8,
    chan12 : u8,
    chan13 : u8,
    chan14 : u8,
    chan15 : u8,
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


#if defined(USE_CRSF_LINK_STATISTICS)
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

struct crsfPayloadLinkstatistics_s {
    uplink_RSSI_1: u8,
    uplink_RSSI_2: u8,
    uplink_Link_quality: u8,
    uplink_SNR: i8,
    active_antenna: u8,
    rf_Mode: u8,
    uplink_TX_Power: u8,
    downlink_RSSI: u8,
    downlink_Link_quality: u8,
    downlink_SNR: i8,
}

if USE_CRSF_V3 {
    struct crsfPayloadLinkstatisticsRx_s {
        downlink_RSSI_1: u8,
        downlink_RSSI_1_percentage: u8,
        downlink_Link_quality: u8,
        downlink_SNR: i8;
        uplink_power: u8,
    } // this struct is currently not used

    struct crsfPayloadLinkstatisticsTx_s {
        uplink_RSSI: u8,
        uplink_RSSI_percentage: u8,
        uplink_Link_quality: u8,
        uplink_SNR: i8,
        downlink_power: u8,
        // currently not used
        uplink_FPS: u8, // currently not used
    }
}

static timeUs_t lastLinkStatisticsFrameUs;

fn handleCrsfLinkStatisticsFrame(statsPtr: &crsfLinkStatistics_t, currentTimeUs: timeUs_t)
{
    let stats: crsfLinkStatistics_t = *statsPtr;
    lastLinkStatisticsFrameUs = currentTimeUs;
    let mut rssiDbm: i16 = -1 * (if stats.active_antenna {stats.uplink_RSSI_2} else { stats.uplink_RSSI_1 });
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
            let rssiPercentScaled: u16 = scaleRange(rssiDbm, CRSF_RSSI_MIN, 0, 0, RSSI_MAX_VALUE);
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
        let crsfUplinkPowerStatesItemIndex: u8 = if stats.uplink_TX_Power < CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT { stats.uplink_TX_Power} else { 0 };
        rxSetUplinkTxPwrMw(uplinkTXPowerStatesMw[crsfUplinkPowerStatesItemIndex]);
    }

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI_1);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_RSSI_2);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.rf_Mode);

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 0, stats.active_antenna);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 1, stats.uplink_SNR);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 2, stats.uplink_TX_Power);

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 0, stats.downlink_RSSI);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 1, stats.downlink_Link_quality);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 2, stats.downlink_SNR);
}

if USE_CRSF_V3 {
    fn handleCrsfLinkStatisticsTxFrame(statsPtr: &crsfLinkStatisticsTx_t*, currentTimeUs: timeUs_t ){
        let stats: crsfLinkStatisticsTx_t  = *statsPtr;
        lastLinkStatisticsFrameUs = currentTimeUs;
        if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF {
            let rssiPercentScaled: u16 = scaleRange(stats.uplink_RSSI_percentage, 0, 100, 0, RSSI_MAX_VALUE);
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

        DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI);
        DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_SNR);
        DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
        DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.uplink_RSSI_percentage);
    }
}

if USE_CRSF_LINK_STATISTICS {
fn crsfCheckRssi(currentTimeUs: u32) {

    if cmpTimeUs(currentTimeUs, lastLinkStatisticsFrameUs) > CRSF_LINK_STATUS_UPDATE_TIMEOUT_US {
        if rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF {
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL_CRSF);
if USE_RX_RSSI_DBM {
            if rxConfig().crsf_use_rx_snr {
                setRssiDbmDirect(CRSF_SNR_MIN, RSSI_SOURCE_RX_PROTOCOL_CRSF);
            } else {
                setRssiDbmDirect(CRSF_RSSI_MIN, RSSI_SOURCE_RX_PROTOCOL_CRSF);
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
}

fn  crsfFrameCRC() -> u8
{
    // CRC includes type and payload
    let mut crc: u8 = crc8_dvb_s2(0, crsfFrame.frame.type);
    for ii in 0..crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    crc
}

fn crsfFrameCmdCRC() -> u8
{
    // CRC includes type and payload
    let mut crc: u8 = crc8_poly_0xba(0, crsfFrame.frame.type);
    for ii in 0..crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1 {
        crc = crc8_poly_0xba(crc, crsfFrame.frame.payload[ii]);
    }
    crc
}

// Receive ISR callback, called back from serial port
STATIC_UNIT_TESTED void crsfDataReceive(uint16_t c, void *data)
{
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;

    static uint8_t crsfFramePosition = 0;
#if defined(USE_CRSF_V3)
    static uint8_t crsfFrameErrorCnt = 0;
#endif
    const timeUs_t currentTimeUs = microsISR();

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = currentTimeUs - crsfFrameStartAtUs;
#endif

    if (cmpTimeUs(currentTimeUs, crsfFrameStartAtUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0;
    }

    if (crsfFramePosition == 0) {
        crsfFrameStartAtUs = currentTimeUs;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    // sometimes we can receive some garbage data. So, we need to check max size for preventing buffer overrun.
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : MIN(crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

    if (crsfFramePosition < fullFrameLength) {
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t)c;
        if (crsfFramePosition >= fullFrameLength) {
            crsfFramePosition = 0;
            const uint8_t crc = crsfFrameCRC();
            if (crc == crsfFrame.bytes[fullFrameLength - 1]) {
#if defined(USE_CRSF_V3)
                crsfFrameErrorCnt = 0;
#endif
                switch (crsfFrame.frame.type) {
                case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                    if (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                        rxRuntimeState->lastRcFrameTimeUs = currentTimeUs;
                        crsfFrameDone = true;
                        memcpy(&crsfChannelDataFrame, &crsfFrame, sizeof(crsfFrame));
                    }
                    break;

#if defined(USE_TELEMETRY_CRSF) && defined(USE_MSP_OVER_TELEMETRY)
                case CRSF_FRAMETYPE_MSP_REQ:
                case CRSF_FRAMETYPE_MSP_WRITE: {
                    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    if (bufferCrsfMspFrame(frameStart, crsfFrame.frame.frameLength - 4)) {
                        crsfScheduleMspResponse(crsfFrame.frame.payload[1]);
                    }
                    break;
                }
#endif
#if defined(USE_CRSF_CMS_TELEMETRY)
                case CRSF_FRAMETYPE_DEVICE_PING:
                    crsfScheduleDeviceInfoResponse();
                    break;
                case CRSF_FRAMETYPE_DISPLAYPORT_CMD: {
                    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    crsfProcessDisplayPortCmd(frameStart);
                    break;
                }
#endif
#if defined(USE_CRSF_LINK_STATISTICS)

                case CRSF_FRAMETYPE_LINK_STATISTICS: {
                    // if to FC and 10 bytes + CRSF_FRAME_ORIGIN_DEST_SIZE
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE)) {
                        const crsfLinkStatistics_t* statsFrame = (const crsfLinkStatistics_t*)&crsfFrame.frame.payload;
                        handleCrsfLinkStatisticsFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_LINK_STATISTICS_RX: {
                    break;
                }
                case CRSF_FRAMETYPE_LINK_STATISTICS_TX: {
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE)) {
                        const crsfLinkStatisticsTx_t* statsFrame = (const crsfLinkStatisticsTx_t*)&crsfFrame.frame.payload;
                        handleCrsfLinkStatisticsTxFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#endif
#endif
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_COMMAND:
                    if ((crsfFrame.bytes[fullFrameLength - 2] == crsfFrameCmdCRC()) &&
                        (crsfFrame.bytes[3] == CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
                        crsfProcessCommand(crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE);
                    }
                    break;
#endif
                default:
                    break;
                }
            } else {
#if defined(USE_CRSF_V3)
                if (crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD)
                    crsfFrameErrorCnt++;
#endif
            }
        } else {
#if defined(USE_CRSF_V3)
            if (crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD)
                crsfFrameErrorCnt++;
#endif
        }
#if defined(USE_CRSF_V3)
        if (crsfFrameErrorCnt >= CRSF_FRAME_ERROR_COUNT_THRESHOLD) {
            // fall back to default speed if speed mismatch detected
            setCrsfDefaultSpeed();
            crsfFrameErrorCnt = 0;
        }
#endif
    }
}

STATIC_UNIT_TESTED uint8_t crsfFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

#if defined(USE_CRSF_LINK_STATISTICS)
    crsfCheckRssi(micros());
#endif
    if crsfFrameDone {
        crsfFrameDone = false;

        // unpack the RC channels
        if (crsfChannelDataFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // use ordinary RC frame structure (0x16)
            const crsfPayloadRcChannelsPacked_t* const rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfChannelDataFrame.frame.payload;
            channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
            crsfChannelData[0] = rcChannels->chan0;
            crsfChannelData[1] = rcChannels->chan1;
            crsfChannelData[2] = rcChannels->chan2;
            crsfChannelData[3] = rcChannels->chan3;
            crsfChannelData[4] = rcChannels->chan4;
            crsfChannelData[5] = rcChannels->chan5;
            crsfChannelData[6] = rcChannels->chan6;
            crsfChannelData[7] = rcChannels->chan7;
            crsfChannelData[8] = rcChannels->chan8;
            crsfChannelData[9] = rcChannels->chan9;
            crsfChannelData[10] = rcChannels->chan10;
            crsfChannelData[11] = rcChannels->chan11;
            crsfChannelData[12] = rcChannels->chan12;
            crsfChannelData[13] = rcChannels->chan13;
            crsfChannelData[14] = rcChannels->chan14;
            crsfChannelData[15] = rcChannels->chan15;
        } else {
            // use subset RC frame structure (0x17)
            let readByteIndex: u8 = 0;
            letpayload: &[u8] = crsfChannelDataFrame.frame.payload;

            // get the configuration byte
            let configByte: u8 = payload[readByteIndex++];

            // get the channel number of start channel
            let startChannel: u8 = configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            let channelBits: u8 = 0;
            let channelMask: u16 = 0;
            let channelRes: u8 = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
            configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;

            match channelRes {
                CRSF_SUBSET_RC_RES_CONF_10B => {
                    channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
                    channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_10B;
                }
                CRSF_SUBSET_RC_RES_CONF_11B => {
                    channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
                    channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_11B;
                }
                CRSF_SUBSET_RC_RES_CONF_12B => {
                    channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
                    channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_12B;
                }
               CRSF_SUBSET_RC_RES_CONF_13B => {
                    channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
                    channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
                    channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_13B;
                }
            }

            // do nothing for the reserved configuration bit
            configByte >>= CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            let numOfChannels: u8 = ((crsfChannelDataFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1) * 8) / channelBits;

            // unpack the channel data
            let bitsMerged: u8 = 0;
            let readValue: u32 = 0;
            for n in 0..numOfChannels {
                while (bitsMerged < channelBits) {
                    let readByte: u8 = payload[readByteIndex++];
                    readValue |= ((uint32_t) readByte) << bitsMerged;
                    bitsMerged += 8;
                }
                crsfChannelData[startChannel + n] = readValue & channelMask;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
        }
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

fn crsfReadRawRC(rxRuntimeState: &rxRuntimeState_t, chan: u8) -> f32
{
    UNUSED(rxRuntimeState);
    if channelScale == CRSF_RC_CHANNEL_SCALE_LEGACY {
        /* conversion from RC value to PWM
        * for 0x16 RC frame
        *       RC     PWM
        * min  172 ->  988us
        * mid  992 -> 1500us
        * max 1811 -> 2012us
        * scale factor = (2012-988) / (1811-172) = 0.62477120195241
        * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
        */
        return (channelScale * (float)crsfChannelData[chan]) + 881;
    } else {
        /* conversion from RC value to PWM
        * for 0x17 Subset RC frame
        */
        return (channelScale * (float)crsfChannelData[chan]) + 988;
    }
}

fn crsfRxWriteTelemetryData(void: &data, len: usize)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

fn crsfRxSendTelemetryData()
{
    // if there is telemetry data to write
    if telemetryBufLen > 0 {
        serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

fn crsfRxIsTelemetryBufEmpty() -> bool
{
    return telemetryBufLen == 0
}

fn crsfRxInit(xConfig: &rxConfig_t, rxRuntimeState: &mut rxRuntimeState_t) -> bool
{
    for ii in 0..CRSF_MAX_CHANNEL {
        crsfChannelData[ii] = (16 * rxConfig.midrc) / 10 - 1408;
    }

    rxRuntimeState.channelCount = CRSF_MAX_CHANNEL;
    rxRuntimeState.rxRefreshRate = CRSF_TIME_BETWEEN_FRAMES_US; //!!TODO this needs checking

    rxRuntimeState.rcReadRawFn = crsfReadRawRC;
    rxRuntimeState.rcFrameStatusFn = crsfFrameStatus;
    rxRuntimeState.rcFrameTimeUsFn = rxFrameTimeUs;

    let *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if !portConfig {
        return false;
    }

    serialPort = openSerialPort(portConfig.identifier,
        FUNCTION_RX_SERIAL,
        crsfDataReceive,
        rxRuntimeState,
        CRSF_BAUDRATE,
        CRSF_PORT_MODE,
        CRSF_PORT_OPTIONS | (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
        );

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL_CRSF;
    }
if USE_RX_LINK_QUALITY_INFO {
    if (linkQualitySource == LQ_SOURCE_NONE) {
        linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
    }
}

    return serialPort != NULL;
}

if USE_CRSF_V3 {
fn crsfRxUpdateBaudrate(baudrate: u32)
{
    serialSetBaudRate(serialPort, baudrate);
}
}

fn crsfRxIsActive(void) -> bool {
    serialPort != NULL
}
}
