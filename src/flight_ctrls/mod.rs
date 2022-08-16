//! This module contains code related to flight controls. Code specific to the PID
//! loops are in the `pid` module.
//!
//! [Betaflight Signal flow diagram](https://github.com/betaflight/betaflight/wiki/Signal-Flow-Diagram)
//! Note that this is just an example, and isn't necesssarily something to emulate.

pub mod common;

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        pub mod fixed_wing;
        use fixed_wing::*;
    } else {
        pub mod quad;
        use quad::*;
    }
}
