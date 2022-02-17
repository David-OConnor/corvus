//! This module contains drivers for various hardware peripherals, each in its own sub-module.

pub mod baro_dps310;
pub mod gps_x;
pub mod imu_icm42605;
pub mod osd_max7456;
// pub mod optical_flow_driver;
// `tof_driver` uses partially-translated C code that doesn't conform to Rust naming conventions.
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(unused_parens)]
pub mod tof_vl53l1;
