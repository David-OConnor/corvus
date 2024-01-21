//! This module contains code that is specific to given MCUs and peripheral
//! layouts.

use cfg_if::cfg_if;
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

// Update frequency: 600kHz
// 170Mhz tim clock on G4.
// 240Mhz tim clock on H743
// 260Mhz tim clock on H723 @ 520Mhz. 275Mhz @ 550Mhz

cfg_if! {
    if #[cfg(feature = "h7")] {
        // pub const TIM_CLK: u32 = 260_000_000; // Hz. H723 @ 550Mhz
        // pub const TIM_CLK: u32 = 275_000_000; // Hz.  H723 @ 520Mhz
        pub const TIM_CLK_SPEED: u32 = 200_000_000; // Hz.  H743 @ 400Mhz.
        pub const DSHOT_SPEED: u32 = 600_000; // Hz.
        // todo: What should this be on H7?
        pub const DSHOT_ARR_READ: u32 = 17_000; // 17k for DSHOT300

    } else if #[cfg(feature = "g4")] {
        pub const TIM_CLK_SPEED: u32 = 170_000_000;
        pub const DSHOT_SPEED: u32 = 300_000; // Hz.
        // todo: How should thsi be set up?
        // todo: Uhoh - getting a weird stagger past 14.5k or so where starts jittering
        // todo between increase and decrease?
        pub const DSHOT_ARR_READ: u32 = 17_000; // 17k for DSHOT300
    }
}

cfg_if! {
    if #[cfg(feature = "h7")] {
        pub const BATT_ADC_CH: u8 = 18;
        pub const CURR_ADC_CH: u8 = 16;
    } else {
        pub const BATT_ADC_CH: u8 = 2;
        pub const CURR_ADC_CH: u8 = 12;
    }
}
