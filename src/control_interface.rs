//! This module handles mapping control inputs from the radio controller to program functions.
//! It is not based on the ELRS or CRSF spec; it's an interface layer between that, and the rest of this program.
//! Interfaces in this module is specific to this program.
//!
//! https://www.expresslrs.org/3.0/software/switch-config/

use core::sync::atomic::Ordering;

use cfg_if::cfg_if;
use defmt::println;

use crate::{
    protocols::crsf::{self, ChannelDataCrsf, LinkStats},
    safety::ArmStatus,
    setup,
    system_status::{self, SensorStatus, SystemStatus},
    util,
};

const CONTROL_VAL_MIN: f32 = -1.;
const CONTROL_VAL_MIN_THROTTLE: f32 = 0.;
const CONTROL_VAL_MAX: f32 = 1.;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// For the switch position. We interpret actual mode from this, and other data, like prescense of GPS.
/// val is for passing over USB serial.
pub enum AltHoldSwitch {
    Disabled = 0,
    EnabledMsl = 1,
    EnabledAgl = 2,
}

impl Default for AltHoldSwitch {
    fn default() -> Self {
        Self::Disabled
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// One autopilot switch
pub enum AutopilotSwitchA {
    Disabled = 0,
    /// Loiter for quad. Orbit for fixed
    LoiterOrbit = 1,
    DirectToPoint = 2,
}

impl Default for AutopilotSwitchA {
    fn default() -> Self {
        Self::Disabled
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// One autopilot switch
pub enum AutopilotSwitchB {
    Disabled = 0,
    HdgHold = 1,
    Land = 2,
}

impl Default for AutopilotSwitchB {
    fn default() -> Self {
        Self::Disabled
    }
}

#[derive(Clone, Copy, PartialEq)]
/// For the switch position. We interpret actual mode from this, and other data, like prescense of GPS.
/// val is for passing over USB serial.
pub enum PidTuneMode {
    Disabled = 0,
    P = 1,
    I = 2,
    D = 3,
}

impl Default for PidTuneMode {
    fn default() -> Self {
        Self::Disabled
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Command a change to PID eg airborne.
pub enum PidTuneActuation {
    Neutral = 0,
    Increase = 1,
    Decrease = 2,
}

impl Default for PidTuneActuation {
    fn default() -> Self {
        Self::Neutral
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// Usd to cycle steerpoints.
pub enum SteerpointCycleActuation {
    Neutral = 0,
    Increase = 1,
    Decrease = 2,
}

impl Default for SteerpointCycleActuation {
    fn default() -> Self {
        Self::Neutral
    }
}

// todo: Consider putting these mode switches in `control_interface`.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
/// For the switch position. We interpret actual mode from this, and other data, like prescense of GPS.
/// val is for passing over USB serial.
pub enum InputModeSwitch {
    /// Acro mode
    Acro = 0,
    /// Command if GPS is present; Attitude if not
    AttitudeCommand = 1,
}

impl Default for InputModeSwitch {
    fn default() -> Self {
        Self::Acro
    }
}

/// Map a raw CRSF channel value to a useful value.
fn channel_to_val(mut chan_val: u16, is_throttle: bool) -> f32 {
    if chan_val < crsf::CHANNEL_VAL_MIN {
        chan_val = crsf::CHANNEL_VAL_MIN
    } else if chan_val > crsf::CHANNEL_VAL_MAX {
        chan_val = crsf::CHANNEL_VAL_MAX
    }

    let control_val_min = if is_throttle {
        CONTROL_VAL_MIN_THROTTLE
    } else {
        CONTROL_VAL_MIN
    };
    util::map_linear(
        chan_val as f32,
        (crsf::CHANNEL_VAL_MIN_F32, crsf::CHANNEL_VAL_MAX_F32),
        (control_val_min, CONTROL_VAL_MAX),
    )
}

/// Represents channel data in our end-use format. This is not constrained by
/// ELRS or CRSF's formats.
#[derive(Default)]
pub struct ChannelData {
    /// "Aileron", -1. to 1.
    pub roll: f32,
    /// "Elevator", -1. to 1.
    pub pitch: f32,
    /// Throttle, 0. to 1., or -1. to 1. depending on if stick auto-centers.
    pub throttle: f32,
    /// "Rudder", -1. to 1.
    pub yaw: f32,
    /// Master arm switch for rotor power and perhaps other systems. Ideally on 2-position non-spring
    /// switch. (Quad), 3-pos switch (fixed-wing)
    /// todo: Currently, the ELRS arm channel is hard set and 2-pos, so we use 2 2-pos switches.
    pub arm_status: ArmStatus,
    /// Ie angular-rate-based (Acro), or Command (with GPS present) or attitude-based
    /// (no GPS present). Ideally on 2-position non-spring switch.
    pub input_mode: InputModeSwitch, // todo: Reconsider how this works.
    /// Eg disabled, AGL, or MSL.  Ideally on 3-position non-spring switch.
    pub alt_hold: AltHoldSwitch,
    pub autopilot_a: AutopilotSwitchA,
    pub autopilot_b: AutopilotSwitchB,
    pub steerpoint_cycle: SteerpointCycleActuation,
    /// For live PID tuning, select P, I, or D to tune. Ideally on 3-position non-spring
    /// switch.
    /// todo: Disabled ideally too, but controllers don't usually have 4-posit switches?
    pub pid_tune_mode: PidTuneMode,
    /// For live PID tuning, ideally on 3-position spring switch. Could also be as 2 buttons.
    pub pid_tune_actuation: PidTuneActuation, // todo: Auto-recover commanded, auto-TO/land/RTB, obstacle avoidance etc.
    /// Auto command level attitude. Ideally on a button
    pub level_attitude_commanded: bool,
}

impl ChannelData {
    pub fn from_crsf(crsf_data: &ChannelDataCrsf) -> Self {
        // https://www.expresslrs.org/3.0/software/switch-config/:
        // "WARNING: Put your arm switch on AUX1, and set it as ~1000 is disarmed, ~2000 is armed."
        // todo: On fixed wing, you want this to be a 3-pos switch, but this may not be
        // todo possible with ELRS, with this channel hard-coded as a 2-pos arm sw?
        let motors_armed = match crsf_data.aux_1 {
            0..=1_500 => false,
            // 0..=1_500 => ArmStatus::Disarmed,
            _ => true,
            // _ => motors_armed,
        };
        let input_mode = match crsf_data.aux_2 {
            0..=1_000 => InputModeSwitch::Acro,
            _ => InputModeSwitch::AttitudeCommand,
        };

        let alt_hold = match crsf_data.aux_3 {
            0..=667 => AltHoldSwitch::Disabled,
            668..=1_333 => AltHoldSwitch::EnabledMsl,
            _ => AltHoldSwitch::EnabledAgl,
        };

        let autopilot_a = match crsf_data.aux_4 {
            0..=667 => AutopilotSwitchA::Disabled,
            668..=1_333 => AutopilotSwitchA::LoiterOrbit,
            _ => AutopilotSwitchA::DirectToPoint,
        };

        let autopilot_b = match crsf_data.aux_5 {
            0..=667 => AutopilotSwitchB::Disabled,
            668..=1_333 => AutopilotSwitchB::HdgHold,
            _ => AutopilotSwitchB::Land,
        };

        let steerpoint_cycle = match crsf_data.aux_6 {
            0..=667 => SteerpointCycleActuation::Decrease,
            668..=1_333 => SteerpointCycleActuation::Neutral,
            _ => SteerpointCycleActuation::Increase,
        };

        let pid_tune_mode = match crsf_data.aux_7 {
            0..=511 => PidTuneMode::Disabled,
            512..=1_023 => PidTuneMode::P,
            1_024..=1533 => PidTuneMode::I,
            _ => PidTuneMode::D,
        };

        let pid_tune_actuation = match crsf_data.aux_8 {
            0..=667 => PidTuneActuation::Decrease,
            668..=1_333 => PidTuneActuation::Neutral,
            _ => PidTuneActuation::Increase,
        };

        let level_attitude_commanded = match crsf_data.aux_9 {
            0..=1_000 => false,
            _ => true,
        };

        // todo: Ideally, this would be on the same channel as motor arm in a 3-pos
        // todo switch, but ELRS hard codes is
        #[cfg(feature = "fixed-wing")]
        let controls_armed = match crsf_data.aux_10 {
            0..=1_000 => false,
            _ => true,
        };

        cfg_if! {
            if #[cfg(feature = "quad")] {
                let arm_status = if motors_armed { ArmStatus::Armed } else {ArmStatus::Disarmed };
            } else {
                let arm_status = if motors_armed {
                    // Implicitly here, if the motor switch armed and control isn't, arm
                    // controls.
                    ArmStatus::MotorsControlsArmed
                } else if controls_armed {
                    ArmStatus::ControlsArmed
                } else {
                    ArmStatus::Disarmed
                };
            }
        }

        // Note that we could map to CRSF channels (Or to their ELRS-mapped origins), but this is
        // currently set up to map directly to how we use the controls.
        ChannelData {
            // Clamp, and map CRSF data to a scale between -1. and 1.  or 0. to +1.
            roll: channel_to_val(crsf_data.channel_1, false),
            pitch: channel_to_val(crsf_data.channel_2, false),
            throttle: channel_to_val(crsf_data.channel_3, true),
            yaw: channel_to_val(crsf_data.channel_4, false),
            arm_status,
            input_mode,
            alt_hold,
            autopilot_a,
            autopilot_b,
            steerpoint_cycle,
            pid_tune_mode,
            pid_tune_actuation,
            level_attitude_commanded,
        }
    }
}

// todo: Is this the right module for this?
/// Loads channel data and link stats into our shared structures,
/// from the DMA buffer. Performs link-status updates.
pub fn handle_crsf_data(
    control_channel_data: &mut Option<ChannelData>,
    link_stats: &mut LinkStats,
    system_status: &mut SystemStatus,
    timestamp: f32,
) {
    let mut rx_fault = false;

    if let Some(crsf_data) = crsf::handle_packet(setup::CRSF_RX_CH, &mut rx_fault) {
        match crsf_data {
            crsf::PacketData::ChannelData(data_crsf) => {
                *control_channel_data = Some(ChannelData::from_crsf(&data_crsf));

                crsf::NEW_PACKET_RECEIVED.store(false, Ordering::Release);

                // A bit imprecise since this is synced to IMU loop time, but is good enough
                // for this purpose.
                system_status.update_timestamps.rf_control_link = Some(timestamp);
                system_status.rf_control_link = SensorStatus::Pass;
            }

            crsf::PacketData::LinkStats(stats) => {
                *link_stats = stats;
            }
        }
    }

    if rx_fault {
        system_status::RX_FAULT.store(true, Ordering::Release);
    }
}
