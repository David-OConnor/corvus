#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/OTA/OTA.cpp
//! and https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/OTA/OTA.h
//!
//! James K, from ELRS: "Yes, the OTA code packs and unpacks the data buffers in the format to send over the radio -
//! literally Over The Air.
//! ESP processors run most of their code from large flash memories.
//! To speed things up there is a section of ram that blocks of code are cached into - that's icache.
//! The ICACHE_RAM_ATTR is used to mark a function to always be loaded into icache.
//! On esp this is important because you can't run interrupt handlers from flash, so anything that
//! runs in an interrupt context has to be fixed in icache.
//!
//! Historically, elrs has run most of the code in interrupt context, so there's a lot of
//! pressure on icache. We used to hit the max on 8285 all the time, but a recent-ish
//! release of the sdk allowed us to repartition the static vs dynamic cache sizes so now
//! there's more room.

// todo: Do we want this, or is this for using the CRSF protocol?

/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * This file provides utilities for packing and unpacking the data to
 * be sent over the radio link.
 */

use super::{
    crsf::CRSF,
};

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum PacketHeaderType {
    /// standard channel data packet
    RC_DATA_PACKET = 0b00,
    /// MSP data packet
    MSP_DATA_PACKET = 0b01,
    /// TLM packet
    TLM_PACKET = 0b11,
    /// sync packet
    SYNC_PACKET = 0b10,
}

// Mask used to XOR the ModelId into the SYNC packet for ModelMatch
const MODELMATCH_MASK: u8 = 0x3f;

#[derive(Clone, Copy, PartialEq)]
pub enum OtaSwitchMode {
    sm1Bit,
    smHybrid,
    smHybridWide,
}

#[inline(always)]
/// Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
/// 0, 1, 2, 3, 4, 5, 6, 7,
/// 1, 2, 3, 4, 5, 6, 7, 0
/// Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
/// this makes sure each of the 8 values is sent at least once every 16 packets
/// regardless of the TLM ratio
/// Index 7 also can never fall on a telemetry slot
fn HybridWideNonceToSwitchIndex(nonce: u8) -> u8 {
    ((nonce & 0b111) + ((nonce >> 3) & 0b1)) % 8
}

// Current ChannelData unpacker function being used by RX
// todo: What is this?
// static mut UnpackChannelData_: UnpackChannelData = UnpackChannelData {
//
// };

// todo: Put back once you figure out where CRSF comes from.
// fn UnpackChannelDataHybridCommon(Buffer: &mut [u8], crsf_bf: &mut CRSF) {
//     // The analog channels
//     crsf_bf.PackedRCdataOut.ch0 = (Buffer[1] << 3) | ((Buffer[5] & 0b11000000) >> 5);
//     crsf_bf.PackedRCdataOut.ch1 = (Buffer[2] << 3) | ((Buffer[5] & 0b00110000) >> 3);
//     crsf_bf.PackedRCdataOut.ch2 = (Buffer[3] << 3) | ((Buffer[5] & 0b00001100) >> 1);
//     crsf_bf.PackedRCdataOut.ch3 = (Buffer[4] << 3) | ((Buffer[5] & 0b00000011) << 1);
// }

/**
 * Hybrid switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 *
 * Input: Buffer
 * Output: crsf_bf->PackedRCdataOut
 * Returns: TelemetryStatus bit
 */
pub fn UnpackChannelDataHybridSwitch8(
    Buffer: &mut [u8],
    crsf: &mut CRSF,
    nonce: u8,
    tlmDenom: u8,
) -> bool {
    let switchByte = Buffer[6];
    UnpackChannelDataHybridCommon(Buffer, crsf);

    // todo: Do we want this, since it uses a lot of Crsf?

    // The low latency switch
    // todo where from?
    // crsf_bf.PackedRCdataOut.ch4 = BIT_to_CRSF((switchByte & 0b01000000) >> 6);

    // The round-robin switch, switchIndex is actually index-1
    // to leave the low bit open for switch 7 (sent as 0b11x)
    // where x is the high bit of switch 7
    let switchIndex: u8 = (switchByte & 0b111000) >> 3;
    let switchValue: u16 = SWITCH3b_to_CRSF(switchByte & 0b111);

    match switchIndex {
        0 => {
            crsf.PackedRCdataOut.ch5 = switchValue;
        }
        1 => {
            crsf.PackedRCdataOut.ch6 = switchValue;
        }
        2 => {
            crsf.PackedRCdataOut.ch7 = switchValue;
        }
        3 => {
            crsf.PackedRCdataOut.ch8 = switchValue;
        }
        4 => {
            crsf.PackedRCdataOut.ch9 = switchValue;
        }
        5 => {
            crsf.PackedRCdataOut.ch10 = switchValue;
        }
        6 => (), // Because AUX1 (index 0) is the low latency switch, the low bit
        7 => {
            // of the switchIndex can be used as data, and arrives as index "6"
            // todo: Where does N_TO_CRSF come from?
            // crsf_bf.PackedRCdataOut.ch11 = N_to_CRSF(switchByte & 0b1111, 15);
        }
        _ => (),
    }

    // TelemetryStatus bit
    return switchByte & (1 << 7) != 0;
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
 * Output: crsf_bf.PackedRCdataOut, crsf_bf.LinkStatistics.uplink_TX_Power
 * Returns: TelemetryStatus bit
 */
pub fn UnpackChannelDataHybridWide(
    Buffer: &mut [u8],
    // crsf_bf: &mut [CRSF], // todo: Where does CRSF come from?
    nonce: u8,
    tlmDenom: u8,
) -> bool {
    let mut TelemetryStatus: bool = false;
    let switchByte: u8 = Buffer[6];
    UnpackChannelDataHybridCommon(Buffer, crsf);

    // The low latency switch (AUX1)
    // todo: Where does BIT_TO_CRSF come from?
    // crsf_bf.PackedRCdataOut.ch4 = BIT_to_CRSF((switchByte & 0b10000000) >> 7);

    // The round-robin switch, 6-7 bits with the switch index implied by the nonce
    let switchIndex: u8 = HybridWideNonceToSwitchIndex(nonce);
    let telemInEveryPacket = (tlmDenom < 8);
    if telemInEveryPacket || switchIndex == 7 {
        TelemetryStatus = (switchByte & 0b01000000) >> 6 != 0;
    }
    if switchIndex == 7 {
        // todo: Put back once you figure out where CRSF comes from
        // crsf_bf.LinkStatistics.uplink_TX_Power = switchByte & 0b111111;
    } else {
        let mut bins: u8 = 0;
        let mut switchValue: u16 = 0;
        if telemInEveryPacket {
            bins = 63;
            switchValue = switchByte as u16 & 0b111111; // 6-bit
        } else {
            bins = 127;
            switchValue = switchByte as u16 & 0b1111111; // 7-bit
        }

        // todo: Where does N_to_CRSF come from?
        // switchValue = N_to_CRSF(switchValue, bins);
        // match switchIndex {
        //     0 => {
        //         crsf_bf.PackedRCdataOut.ch5 = switchValue;
        //     }
        //     1 => {
        //         crsf_bf.PackedRCdataOut.ch6 = switchValue;
        //     }
        //     2 => {
        //         crsf_bf.PackedRCdataOut.ch7 = switchValue;
        //     }
        //     3 => {
        //         crsf_bf.PackedRCdataOut.ch8 = switchValue;
        //     }
        //     4 => {
        //         crsf_bf.PackedRCdataOut.ch9 = switchValue;
        //     }
        //     5 => {
        //         crsf_bf.PackedRCdataOut.ch10 = switchValue;
        //     }
        //     6 => {
        //         crsf_bf.PackedRCdataOut.ch11 = switchValue;
        //     }
        //     _ => (),
        // }
    }

    return TelemetryStatus;
}

pub static mut OtaSwitchModeCurrent: OtaSwitchMode = OtaSwitchMode::sm1Bit;

pub fn OtaSetSwitchMode(switchMode: OtaSwitchMode) {
    match switchMode {
        OtaSwitchMode::smHybridWide => {
            // todo: Where do these come from??
            // unsafe { UnpackChannelData = &UnpackChannelDataHybridWide; }
        }
        _ => {
            // unsafe { UnpackChannelData = &UnpackChannelDataHybridSwitch8; }
        }
    };

    unsafe { OtaSwitchModeCurrent = switchMode };
}
