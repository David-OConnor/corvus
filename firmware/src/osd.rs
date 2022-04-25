//! This module contains code for interfacing with DJI's OSD system, via UART.
//!
//!
//! https://ardupilot.org/copter/docs/common-msp-overview.html
//! https://github.com/betaflight/betaflight/tree/master/src/main/msp - Maybe? Maybe

// MSP requires a free serial port, and its speed defaults to 115200 baud.
// SERIAL2_PROTOCOL = 33

use stm32_hal2::{pac::USART2, usart::Usart};

// todo: Osd struct, or individual functions?

/// Initial config for the OSD
pub fn setup(uart: &mut Usart<USART2>) {
    // todo: DMA?
}

/// Draw the current airspeed
pub fn draw_airspeed() {}

/// Draw the current Mean Sea Level altitude
pub fn draw_alt_msl() {}

/// Draw the current Above Ground Level altitude
pub fn draw_alt_agl() {}

/// Draw the heading indicator
pub fn draw_heading() {}

/// Draw sideslip. (Horizontal motion relative to flight path)
pub fn draw_side_slip() {}

/// Draw elevation bars on the HUD
pub fn draw_elevation_bars() {}

/// Draw the vertical velocity indicator (flight path indicator)
pub fn draw_vvi() {}

/// Draw a depiction of battery life, including voltage.
pub fn draw_battery() {}
