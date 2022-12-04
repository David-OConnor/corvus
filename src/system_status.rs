//! This module contains code related to system status and built-in-tests.

use core::sync::atomic::AtomicBool;

// A problem with the CRSF control data packet.
pub static RX_FAULT: AtomicBool = AtomicBool::new(false);

// Eg a failed CRC or decoding of RPM data received from the ESC.
pub static RPM_FAULT: AtomicBool = AtomicBool::new(false);

// We have these faults as atomics so as to not require locking a more-generally-used struct.

#[derive(Default)]
pub struct SystemStatus {
    pub imu: SensorStatus,
    pub baro: SensorStatus,
    /// The GPS module is connected. Detected on init.
    pub gps: SensorStatus,
    /// The time-of-flight sensor module is connected. Detected on init.
    pub tof: SensorStatus,
    ///  magnetometer is connected. Likely on the same module as GPS. Detected on init.
    pub magnetometer: SensorStatus,
    pub esc_telemetry: SensorStatus,
    pub esc_rpm: SensorStatus,
    pub rf_control_link: SensorStatus, // todo: For now, we use `link_lost` instead.
    // todo: Consider a separate faults struct if this grows in complexity
    // todo: You should have more specific faults than this. Eg what went wrong.
    // pub rf_control_fault: bool,
    // pub esc_rpm_fault: bool,
    /// SPI flash, which we may use in the future for data logging.
    pub flash_spi: SensorStatus,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)] // for USB ser
pub enum SensorStatus {
    Pass = 0,
    Fail = 1,
    /// Either an external sensor not plugged in, or a complete failture, werein it's not recognized.
    NotConnected = 2,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self::NotConnected
    }
}
