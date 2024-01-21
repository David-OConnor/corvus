//! This module provides a hardware interface for servos.
//! This are used by fixed-wing, eg for use with elevons.

use hal::timer::TimChannel;

use crate::{board_config::ARR_SERVOS, setup::ServoTimer, util};

// These values are to set middle, min and max values of 1.5ms, 1ms, and 2ms used
// by common hobby servos. They adjust duty cycle, which sets servo position.
// Calculations, assuming frequency of 500Hz; 500Hz = 2ms.
// ARR indicates

const FREQ: f32 = 500.; // This must match the above PSC and ARR.
const PERIOD: f32 = 1. / FREQ;

// At 300Hz, it appears the limits ("20g" servo)  are 0.4 to 2.6ms. This corresponds to a range of 3/4 Tau.
// The min values are full Clockwise; max are full CCW.
// Other servos use a nominal range between 1 and 2 ms.
const HIGH_TIME_MIN: f32 = 0.001;
const HIGH_TIME_MAX: f32 = 0.002;

const DUTY_MIN: f32 = HIGH_TIME_MIN / PERIOD;
const DUTY_MAX: f32 = HIGH_TIME_MAX / PERIOD;

const ARR_MIN: f32 = ARR_SERVOS as f32 * DUTY_MIN;
const ARR_MAX: f32 = ARR_SERVOS as f32 * DUTY_MAX;

// Let's reason this out for 300Hz Period full = 1/300 = 3.33ms
// 2ms / 3.33ms = 60% high time for max. 1ms /3.33ms = 30% high time for min.

pub fn set_posit(posit: f32, range_in: (f32, f32), timer: &mut ServoTimer, channel: TimChannel) {
    let mut duty_arr = util::map_linear(posit, range_in, (ARR_MIN, ARR_MAX)) as u32;

    if duty_arr == ARR_SERVOS {
        // Allow a bit of low pulse time. This can occur at max duty cycle at 500Hz.
        duty_arr -= 100;
    }

    #[cfg(feature = "h7")]
    let duty_arr = duty_arr as u16;

    timer.set_duty(channel, duty_arr);
}
