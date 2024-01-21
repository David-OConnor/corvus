//! This module contains code that is specific to given MCUs and peripheral
//! layouts.

use dronecan::hardware::CanClock;
use hal::clocks::CrsSyncSrc;

// 100 Mhz if 400Mhz core. 120 if 480.
#[cfg(feature = "h7")]
pub const CAN_CLOCK: CanClock = CanClock::Mhz100;
#[cfg(feature = "g4")]
pub const CAN_CLOCK: CanClock = CanClock::Mhz170; // todo: 160Mhz A/R.

#[cfg(feature = "h7")]
pub const CRS_SYNC_SRC: CrsSyncSrc = CrsSyncSrc::OtgHs;
#[cfg(feature = "g4")]
pub const CRS_SYNC_SRC: CrsSyncSrc = CrsSyncSrc::Usb;

#[cfg(feature = "g4")]
pub const AHB_FREQ: u32 = 170_000_000;
#[cfg(feature = "h7")]
pub const AHB_FREQ: u32 = 400_000_000;
