//! This module contains code related to flight controls. Code specific to the PID
//! loops are in the `pid` module.
//!
//! [Betaflight Signal flow diagram](https://github.com/betaflight/betaflight/wiki/Signal-Flow-Diagram)
//! Note that this is just an example, and isn't necesssarily something to emulate.

pub mod ctrl_logic;
pub mod autopilot;
pub mod common;
pub mod pid;
// pub mod pid_tuning; // todo: Implement A/R

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        mod fixed_wing;
        pub use fixed_wing::*;
    } else {
        mod quad;
        pub use quad::*;
    }
}
