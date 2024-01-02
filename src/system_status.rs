//! This module contains code related to system status and built-in-tests.

use core::sync::atomic::AtomicBool;

// A problem with the CRSF control data packet.
pub static RX_FAULT: AtomicBool = AtomicBool::new(false);

// Eg a failed CRC or decoding of RPM data received from the ESC.
pub static RPM_FAULT: AtomicBool = AtomicBool::new(false);

// These times are used to trigger faults if it's been too long since a given
// update. They are in seconds.
pub const MAX_UPDATE_PERIOD_IMU: f32 = 1. / crate::main_loop::DT_IMU + 0.0001;
pub const MAX_UPDATE_PERIOD_GNSS: f32 = 0.4;
pub const MAX_UPDATE_PERIOD_BARO: f32 = 0.5;
pub const MAX_UPDATE_PERIOD_MAG: f32 = 0.4;
pub const MAX_UPDATE_PERIOD_RC_LINK: f32 = 0.3;
pub const MAX_UPDATE_PERIOD_OSD: f32 = 1.;

// We have these faults as atomics so as to not require locking a more-generally-used struct.

use defmt::println;

fn set_status(
    status: &mut SensorStatus,
    timestamp_current: f32,
    timestamp_update: Option<f32>,
    max_update_period: f32,
) {
    match timestamp_update {
        Some(t) => {
            if timestamp_current - t > max_update_period {
                *status = SensorStatus::NotConnected;
            } else {
                *status = SensorStatus::Pass;
            }
        }
        None => {
            *status = SensorStatus::NotConnected;
        }
    }
}

#[derive(Default)]
pub struct SystemStatus {
    pub imu: SensorStatus,
    pub imu_can: SensorStatus,
    pub ahrs_can: SensorStatus,
    pub baro: SensorStatus,
    pub baro_can: SensorStatus,
    /// The GPS module is connected. Detected on init.
    pub gnss_can: SensorStatus,
    /// The time-of-flight sensor module is connected. Detected on init.
    pub tof: SensorStatus,
    ///  magnetometer is connected. Likely on the same module as GPS. Detected on init.
    pub magnetometer: SensorStatus,
    pub magnetometer_can: SensorStatus,
    pub esc_telemetry: SensorStatus,
    pub esc_rpm: SensorStatus,
    pub esc_can: SensorStatus,
    pub servos_can: SensorStatus,
    pub rf_control_link: SensorStatus, // todo: For now, we use `link_lost` instead.
    pub rf_control_link_can: SensorStatus,
    // todo: Consider a separate faults struct if this grows in complexity
    // todo: You should have more specific faults than this. Eg what went wrong.
    // pub rf_control_fault: bool,
    // pub esc_rpm_fault: bool,
    /// SPI flash, which we may use in the future for data logging.
    pub flash_spi: SensorStatus,
    pub osd: SensorStatus,
    pub update_timestamps: UpdateTimestamps,
}

impl SystemStatus {
    pub fn update_from_timestamp(&mut self, timestamp: f32) {
        set_status(
            &mut self.imu,
            timestamp,
            self.update_timestamps.imu,
            MAX_UPDATE_PERIOD_IMU,
        );
        set_status(
            &mut self.baro,
            timestamp,
            self.update_timestamps.baro,
            MAX_UPDATE_PERIOD_BARO,
        );
        set_status(
            &mut self.baro_can,
            timestamp,
            self.update_timestamps.baro_can,
            MAX_UPDATE_PERIOD_BARO,
        );
        set_status(
            &mut self.magnetometer,
            timestamp,
            self.update_timestamps.mag,
            MAX_UPDATE_PERIOD_MAG,
        );
        set_status(
            &mut self.magnetometer_can,
            timestamp,
            self.update_timestamps.mag_can,
            MAX_UPDATE_PERIOD_MAG,
        );
        set_status(
            &mut self.gnss_can,
            timestamp,
            self.update_timestamps.gnss_can,
            MAX_UPDATE_PERIOD_GNSS,
        );
        set_status(
            &mut self.osd,
            timestamp,
            self.update_timestamps.osd,
            MAX_UPDATE_PERIOD_OSD,
        );
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)] // for USB ser
pub enum SensorStatus {
    Pass = 0,
    Fault = 1,
    /// Either an external sensor not plugged in, or a complete failture, werein it's not recognized.
    NotConnected = 2,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self::NotConnected
    }
}

/// Times, in seconds since start, of the last valid reading received.
/// A `None` value means have never received an update.
#[derive(Default)]
pub struct UpdateTimestamps {
    pub imu: Option<f32>,
    pub gnss_can: Option<f32>,
    pub baro: Option<f32>,
    pub baro_can: Option<f32>,
    pub mag: Option<f32>,
    pub mag_can: Option<f32>,
    pub imu_can: Option<f32>,
    pub ahrs_can: Option<f32>,
    pub rf_control_link: Option<f32>,
    pub flight_ctrls: Option<f32>, // not really a sensor, but useful to track.
    pub osd: Option<f32>,          // not really a sensor, but useful to track.
}
