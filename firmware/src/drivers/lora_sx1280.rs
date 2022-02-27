//! This module interfaces with the SX1280 and SX1281 LoRa protocls, for communicating
//! with ELRS transmitters. (ELRS transmitters send control inputs from the radio controller)

use stm32hal2::{
    gpio::Pin,
    spi::Spi,
    pac::{SPI3},
};