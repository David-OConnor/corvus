//! Module for interacting with the "next-gen" CAN ESCs.

// use hal::{
//     can_fd,
//     pac::{CAN_FD},
// };

// Hydra:
// Hardware Requirements:
// 1. High Bandwidth.
// 2. Bi-directional.
// 3. Leverage as much hardware as possible on both the FC and ESC in order to reduce CPU load on both sides.
// 4. Must support a) FC to single ESC, b) FC to 4-in-1 4x MCUs (i.e. essentially the same as (a), c) FC to 4-in-1 single MCU.
// 5. Use as few pins as possible.
// 6. Must support short traces/cables and long cables.  e.g. brushless whoops, 3-7" FPV style, X-class style.

// 1) Auto-discovery of ESC capabilities and metadata, such as:
// a) Maximum motor data rate.
// b) Maximum hardware speed (e.g. max frequency of FDCAN/CAN.
// c) ESC Brand.
// d) ESC Model.
// e) ESC firmware revision.
// f) ESC motor count (e.g. 1 for a normal ESC, or 4 for a  4in1 single-MCU ESC)
//
// 2) Auto-discovery of ESC telemetry data-points, and possibly maximum data rates/sampling frequency for each, such as:
// a) RPM data (every frame, every 2nd frame, every 4th frame, etc).
// b) Desync event reporting.
// c) Demag event reporting.
// d) FET Temperature (maximum sampling frequency, e.g. 1 second interval)
// e) MCU Temperature. (maximum sampling frequency, e.g. 1 second interval)
// f) Voltage (maximum sampling frequency, e.g.  second interval)
// g) Current (maximum sampling frequency, e.g. 1 second interval)
// h) More telemetry items
//
// 3) Bi-directional Data Passthrough + Protocol reference.
// Use-cases:
// 1) for dumping debug information from ESC to FC for storage in logs.
// 2) for firmware updates to ESC.
//
// 4) Configuration discovery/get/set.
// e.g. FC or ESC-Tool says: ESC, give me a list of things I can configure.  ESC replies with a list of identifiers, one for each thing that can be configured.
// Each identifier is on a published list of identifiers, the format for each item is known.  e.g.  '1 = pole count, 8bit integer', '2 = Normal/Reverse (boolean, 0 = normal)'
//
// FC/ESC-Tool requests each item, displays a UI.
// User changes UI items and clicks SAVE.
// FC/ESC-Tool sends 'Set configuration X to value Y'.
//
// Ideally we wouldn't then need a thousand different tools to configure ESCs.  Each ESC vendor would then not need to make their own ESC tool for normal use-cases, but could still  make their own tool with advanced support where the any 'general' UI doesn't support a specific configuration identifier.
// I'm not aware of any reason why the ESC would need to ask the FC for anything, but the exact same approach could be used if it did.

const TELEM_SIZE: usize = 69; // todo

#[derive(Default)]
pub struct RpmData {}

/// ESC configuration data. Contains ESC capabilities.
pub struct EscConfig {
    pub brand: &'static str,
    pub model: &'static str,
    pub firmware_rev: &'static str,
    pub motor_count: u8,
}

/// Telemetry, send together at regular intervals.
#[derive(Default)]
pub struct EscTelem {
    pub rpm_data: RpmData,
    /// FET (?) temperature in °C, multiplied by 10.
    pub temp_fet: u16,
    /// Primary microcontroller temperature in °C, multiplied by 10.
    pub temp_mcu: u16,
    /// Battery voltage measured by ESC, multplied by 100.
    pub batt_voltage: u16,
    /// Current draw in mA.
    pub current: u16,
    // todo: Etc.
}

impl EscTelem {
    /// Convert to a `u8` buffer, to be passed over CAN.
    pub fn to_buf(&self) -> [u8; TELEM_SIZE] {
        [0; TELEM_SIZE] // todo
    }

    /// Create this struct from a `u8` buffer, eg passed over CAN.
    pub fn from_buf(buf: &[u8; TELEM_SIZE]) {
        Default::default() // todo
    }
}
