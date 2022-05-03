//! This module handles mapping control inputs from the ELRS radio controller to program functions.
//! It is not based on the ELRS spec; it's an interface layer between that, and the rest of this program.
//!
//! https://www.expresslrs.org/2.0/software/switch-config/

// /// Represents data from a channel, including what channel it is, and the data passed.
// /// [ELRS FAQ](https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support)
// /// Assumes "Wide hybrid mode", as described in the FAQ.
// enum Channel {
//     /// Channels 1-4 are 10-bit channels.
//     Ch1(u16),
//     Ch2(u16),
//     Ch3(u16),
//     Ch4(u16),
//     /// Aux 1 is 2-positions, and must be used for arming. AKA "Channel 5"
//     Aux1(crate::ArmStatus),
//     /// Aux 2-8 are 64 or 128-position channels. (6 or 7 bit)
//     Aux2(u8),
//     Aux3(u8),
//     Aux4(u8),
//     Aux5(u8),
//     Aux6(u8),
//     Aux7(u8),
//     Aux8(u8),
// }

use crate::flight_ctrls::{AltHoldSwitch, ArmStatus, InputModeSwitch};

/// Represents data from all ELRS channels, including what channel it is, and the data passed.
/// [ELRS FAQ](https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support)
/// Assumes "Wide hybrid mode", as described in the FAQ.
#[derive(Default)]
pub struct ElrsChannelData {
    /// Channels 1-4 are 10-bit channels.
    pub channel_1: u16,
    pub channel_2: u16,
    pub channel_3: u16,
    pub channel_4: u16,
    /// Aux 1 is 2-positions, and must be used for arming. AKA "Channel 5"
    pub aux1: ArmStatus,
    /// Aux 2-8 are 64 or 128-position channels. (6 or 7 bit)
    pub aux_2: u8,
    pub aux_3: u8,
    pub aux_4: u8,
    pub aux_5: u8,
    pub aux_6: u8,
    pub aux_7: u8,
    pub aux_8: u8,
}

/// Represents CRSF channel data
#[derive(Default)]
pub struct _CrsfChannelData {
    pub channel_1: u16,
    pub channel_2: u16,
    pub channel_3: u16,
    pub channel_4: u16,
    pub aux_1: u16,
    pub aux_2: u16,
    pub aux_3: u16,
    pub aux_4: u16,
    pub aux_5: u16,
    pub aux_6: u16,
    pub aux_7: u16,
    pub aux_8: u16,
    pub aux_9: u16,
    pub aux_10: u16,
    pub aux_11: u16,
    pub aux_12: u16,
}

/// Represents channel data in our end-use format.
#[derive(Default)]
pub struct ChannelData {
    /// Aileron, -1. to 1.
    pub roll: f32,
    /// Elevator, -1. to 1.
    pub pitch: f32,
    /// Throttle, 0. to 1., or -1. to 1. depending on if stick auto-centers.
    pub throttle: f32,
    /// Rudder, -1. to 1.
    pub yaw: f32,
    pub arm_status: ArmStatus,
    pub input_mode: InputModeSwitch,
    pub alt_hold: AltHoldSwitch,
    // todo: Auto-recover commanded, auto-TO/land/RTB, obstacle avoidance etc.
}

#[derive(Default)]
/// https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support
pub struct LinkStats {
    /// Timestamp these stats were recorded. (TBD format; processed locally; not part of packet from tx).
    pub timestamp: u32,
    /// Uplink - received signal strength antenna 1 (RSSI). RSSI dBm as reported by the RX. Values
    /// vary depending on mode, antenna quality, output power and distance. Ranges from -128 to 0.
    pub uplink_rssi_1: u8,
    /// Uplink - received signal strength antenna 2 (RSSI).  	Second antenna RSSI, used in diversity mode
    /// (Same range as rssi_1)
    pub uplink_rssi_2: u8,
    /// Uplink - link quality (valid packets). The number of successful packets out of the last
    /// 100 from TX → RX
    pub uplink_link_quality: u8,
    /// Uplink - signal-to-noise ratio. SNR reported by the RX. Value varies mostly by radio chip
    /// and gets lower with distance (once the agc hits its limit)
    pub uplink_snr: i8,
    /// Active antenna for diversity RX (0 - 1)
    pub active_antenna: u8,
    pub rf_mode: u8,
    /// Uplink - transmitting power. (mW?) 50mW reported as 0, as CRSF/OpenTX do not have this option
    pub uplink_tx_power: u8,
    /// Downlink - received signal strength (RSSI). RSSI dBm of telemetry packets received by TX.
    pub downlink_rssi: u8,
    /// Downlink - link quality (valid packets). An LQ indicator of telemetry packets received RX → TX
    /// (0 - 100)
    pub downlink_link_quality: u8,
    /// Downlink - signal-to-noise ratio. 	SNR reported by the TX for telemetry packets
    pub downlink_snr: i8,
}
