//! This module contains filtering code for the IMU, including an IIR bessel lowpass, and IIR
//! notch filters for RPM filtering.
//!
//! Reference: https://brushlesswhoop.com/betaflight-rpm-filter/

use cmsis_dsp_sys as dsp_sys;
use cmsis_dsp_api as dsp_api;

/// Calulate the frequency to filter out, in Hz, based on one rotor's RPM.
/// `erpm` is the rpm transmitted by the ESC, aka electrical RPM, via bidirectional
/// DSHOT.
fn rpm_filter_freq(erpm: f32, num_poles: u8) -> f32 {
    let rpm = erpm / (2 * poles); // QC this.

    // `rpm` is per minute - convert to per second.
    rpm / 60
}