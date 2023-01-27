//! This module contains drivers for various hardware peripherals, each in its own sub-module.

pub mod baro_dps310;
// pub mod camera_gimbal;
pub mod gps_ublox;
pub mod imu_icm426xx;
// pub mod imu_ism330dhcx;
pub mod mag_lis3mdl;
// pub mod optical_flow_driver;
pub mod osd;
// `tof_driver` uses partially-translated C code that doesn't conform to Rust naming conventions.
pub mod flash_spi;
#[cfg(feature = "g4")]
pub mod spi2_kludge; // For SPI flash
#[cfg(feature = "g4")]
pub mod uart4_kludge; // For OSD
                      // #[cfg(feature = "g4")]
                      // pub mod spi3_kludge;
pub mod tof_vl53l1;
