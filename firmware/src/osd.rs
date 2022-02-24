//! This module contains hardware-agnostic code for drawing the on-screen display.

use crate::drivers::osd_max7456;

// todo: Osd struct, or individual functions?

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
