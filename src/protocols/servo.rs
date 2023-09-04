//! This module provides a hardware interface for servos.
//! This are used by fixed-wing, eg for use with elevons.

use stm32_hal2::timer::TimChannel;

use crate::{setup::ServoTimer, util};

use cfg_if::cfg_if;

// Choose PSC and ARR to get a servo update frequency of 500Hz. See `dshot.rs` for the calculation.

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


// duty = 0 means 0ms up.
// duty = ARR_SERVOS means 1/500hz = 2ms up.
// duty for 1ms up (min) means 50% duty cycle = ARR_SERVOS / 2
// duty for 2ms up (max) means 100% duty cycle = ARR_SERVOS
const ARR_MIN: f32 = ARR_SERVOS as f32 / 2.;
const ARR_MAX: f32 = ARR_SERVOS as f32 - 100.; // - some so there's still a definite low pulse at max value.

pub fn set_posit(posit: f32, range_in: (f32, f32), timer: &mut ServoTimer, channel: TimChannel) {
    let duty_arr = util::map_linear(posit, range_in, (ARR_MIN_F32, ARR_MAX_F32)) as u32;

    #[cfg(feature = "h7")]
    let duty_arr = duty_arr as u16;

    timer.set_duty(channel, duty_arr);
}
