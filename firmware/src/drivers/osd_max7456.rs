//! This module contains code for the ICM42605 inertial measuring unit.

use stm32_hal2::{gpio::Pin, pac::SPI2, spi::Spi};
