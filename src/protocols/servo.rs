//! This module provides a hardware interface for servos.
//! This are used by fixed-wing, eg for use with elevons.

use stm32_hal2::timer::TimChannel;

use crate::{setup::ServoTimer, util};

use cfg_if::cfg_if;

// Choose PSC adn ARR to get a servo update frequency of 500Hz. See `dshot.rs` for the calculation.

// 170Mhz tim clock on G4.
// 240Mhz tim clock on H743
// 260Mhz tim clock on H723 @ 520Mhz. 275Mhz @ 550Mhz
cfg_if! {
    if #[cfg(feature = "h7")] {
        // 240Mhz tim clock.
        pub const PSC_SERVOS: u16 = 7;
        pub const ARR_SERVOS: u32 = 59_999;
        // 260Mhz tim clock.
        // pub const PSC_SERVOS: u16 = 7;
        // pub const ARR_SERVOS: u32 = 64_999;
        // 275Mhz tim clock.
        // pub const PSC_SERVOS: u16 = 8;
        // pub const ARR_SERVOS: u32 = 61_110;
    } else if #[cfg(feature = "g4")] {
        pub const PSC_SERVOS: u16 = 6;
        pub const ARR_SERVOS: u32 = 48_570;
    }
}

// These values are to set middle, min and max values of 1.5ms, 1ms, and 2ms used
// by common hobby servos. They adjust duty cycle, which sets servo position.
// Calculations, assuming frequency of 500Hz; 500Hz = 2ms.
// ARR indicates

// Time, in seconds for pulses to indicate min and max deflection.
// 1.5 is mid-deflection, 2.0 is absolute max, and 1.0 is absolute min.
// These absolute values may exceed control surface limits.]
const PULSE_TIME_MIN: f32 = 1.3;
const PULSE_TIME_MAX: f32 = 1.7;

// todo: QC this.
const ARR_MIN_F32: f32 = (ARR_SERVOS as f32 / 2.) * (PULSE_TIME_MIN / 1.0);
const ARR_MAX_F32: f32 = ARR_SERVOS as f32 * (PULSE_TIME_MAX / 2.0);
const ARR_MID_F32: f32 = (ARR_MAX_F32 + ARR_MIN_F32) / 2.;

const ARR_MIN: u32 = ARR_MIN_F32 as u32;
const ARR_MID: u32 = ARR_MID_F32 as u32;
// Don't us full ARR: there needs to be some signal low time between pulses.
const ARR_MAX: u32 = ARR_MAX_F32 as u32;

/// Equivalent of `Motor` for quadcopters.
#[derive(Clone, Copy)]
pub enum ServoWing {
    S1,
    S2,
}

// /// Specify the wing associated with a servo. Equivalent of `RotorPosition` for quadcopters.
// /// repr(u8) is for use in Preflight.
// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// pub enum ServoWingPosition {
//     Left = 0,
//     Right = 1,
// }

pub fn set_posit(posit: f32, range_in: (f32, f32), timer: &mut ServoTimer, channel: TimChannel) {
    let duty_arr = util::map_linear(posit, range_in, (ARR_MIN_F32, ARR_MAX_F32)) as u32;

    #[cfg(feature = "h7")]
    let duty_arr = duty_arr as u16;

    timer.set_duty(channel, duty_arr);
}
