//! This module contains flight control code for quadcopters.

// todo: Feed foward - https://drones.stackexchange.com/questions/495/what-does-feed-forward-do-and-how-does-it-work
// todo: Feed foward is an adjustment (to P term?) proportional to (change in?) stick position. It's used
// todo to make maneuver initiation more responsive. Anticapates control. Perhaps something simple like
// todo immediately initiating a power adjustment in response to control actuation? (derivative of ctrl position?)

use core::f32::consts::TAU;

use ahrs::Params;

use super::common::InputMap;
use crate::{
    controller_interface::InputModeSwitch,
    state::StateVolatile,
    system_status::{SensorStatus, SystemStatus},
    util,
};

use defmt::println;

// todo: Variabel/struct field found from cal routine that is power to hover.

// Our maneuverability clamps are different from normal throttle settings: They're used
// to reduce the risk and severity of individual rotors clamping due to throttle settings that
// are too high or too low. They reduce user throttle authority, but provide more predictable
// responses when maneucvering near min and max throttle
// These are power settings, not RPM.
pub const THROTTLE_MAX_MNVR_CLAMP: f32 = 0.80;
// todo: You should probably disable the min maneuver clamp when on the ground (how to check?)
// and have it higher otherwise.
pub const THROTTLE_MIN_MNVR_CLAMP: f32 = 0.06;

// Don't execute the calibration procedure from below this altitude, in meters AGL, eg for safety.
// todo: Calibration unimplemented
// const MIN_CAL_ALT: f32 = 6.;

const ACRO_RATE: f32 = 10.;

impl Default for InputMap {
    fn default() -> Self {
        Self {
            pitch_rate: (-ACRO_RATE, ACRO_RATE),
            roll_rate: (-ACRO_RATE, ACRO_RATE),
            yaw_rate: (-ACRO_RATE, ACRO_RATE),
            throttle_clamped: (THROTTLE_MIN_MNVR_CLAMP, THROTTLE_MAX_MNVR_CLAMP),
            pitch_angle: (-TAU / 4., TAU / 4.),
            roll_angle: (-TAU / 4., TAU / 4.),
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
            vertical_velocity: (-5., 5.),
        }
    }
}

// /// Specify the rotor by position. Used in power application code.
// /// repr(u8) is for use in Preflight.
// #[derive(Clone, Copy, PartialEq)]
// #[repr(u8)]
// pub enum RotorPosition {
//     FrontLeft = 0,
//     FrontRight = 1,
//     AftLeft = 2,
//     AftRight = 3,
// }

/// Mode used for control inputs. These are the three "industry-standard" modes.
#[derive(Clone, Copy, PartialEq)]
pub enum InputMode {
    /// Rate, also know as manual, hard or Acro. Attitude and power stay the same after
    /// releasing controls.
    Acro,
    /// Attitude also know as self-level, angle, or Auto-level. Attitude resets to a level
    /// hover after releasing controls.  When moving the
    /// roll/pitch stick to its maximum position, the drone will also reach the maximum angle
    /// it’s allowed to tilt (defined by the user), and it won’t flip over. As you release the
    /// stick back to centre, the aircraft will also return to its level position.
    /// We use attitude mode as a no-GPS fallback.
    Attitude,
    // GPS-hold, also known as Loiter. Maintains a specific position.
    /// In `Command` mode, the device loiters when idle. Otherwise, it flies at specific velocities,
    /// and altitudes commanded by the controller. Allows for precise control, including in confined
    /// spaces.
    Loiter,
    // /// This mode is easy stable, and designed to make control easy, including in confined spaces.
    // /// Similar to `Command` mode, it loiters when idle. It uses an internal model of
    // /// todo: Same as Command mode? Consolidate?
    // VideoGame,
    Route,
}

impl Default for InputMode {
    fn default() -> Self {
        Self::Acro
    }
}

/// Calculate the horizontal target velocity (m/s), for a given distance (m) from a point horizontally.
pub fn enroute_speed_hor(dist: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if dist > 20. {
        max_v
    } else if dist > 10. {
        2.0_f32.max(max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    }
}

/// Calculate the vertical target velocity (m/s), for a given distance (m) from a point vertically.
/// `dist` is postive if the aircraft altitude is below the set point; otherwise negative.
pub fn enroute_speed_ver(dist: f32, max_v: f32, z_agl: f32) -> f32 {
    // todo: fill this out. LUT?

    if z_agl < 7. {
        let mut result = 3.0_f32.max(max_v);

        if dist < 0. {
            result *= -1.;
        }
    }

    let dist_abs = util::abs(dist);

    let mut result = if dist_abs > 20. {
        max_v
    } else if dist_abs > 10. {
        2.0_f32.max(max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    };

    if dist < 0. {
        result *= -1.;
    }
    result
}

/// Calculate the landing vertical velocity (m/s), for a given height  (m) above the ground.
pub fn landing_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5;
    }
    (height / 4.).max(max_v)
}

/// Calculate the takeoff vertical velocity (m/s), for a given height (m) above the ground.
pub fn takeoff_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.;
    }
    (height / 4. + 0.01).max(max_v)
}

pub fn set_input_mode(
    input_mode_control: InputModeSwitch,
    state_volatile: &mut StateVolatile,
    system_status: &SystemStatus,
) {
    state_volatile.input_mode_switch = input_mode_control; // todo: Do we need or use this field?

    state_volatile.input_mode = match input_mode_control {
        InputModeSwitch::Acro => InputMode::Acro,
        InputModeSwitch::AttitudeLoiter => {
            if system_status.gnss_can == SensorStatus::Pass {
                InputMode::Loiter
            } else {
                InputMode::Attitude
            }
        }
        InputModeSwitch::Route => InputMode::Route,
    }
}
