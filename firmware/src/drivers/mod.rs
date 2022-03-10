//! This module contains drivers for various hardware peripherals, each in its own sub-module.

pub mod baro_dps310;
pub mod gps_x;
pub mod imu_icm42605;
pub mod imu_ism330dhcx;
pub mod osd_max7456;
// pub mod optical_flow_driver;
// `tof_driver` uses partially-translated C code that doesn't conform to Rust naming conventions.
mod camera_gimbal;
pub mod tof_vl53l1;

// pub mod crsf;  CRSF Deprecated in favor of ELRS
