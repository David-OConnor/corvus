//! This module contains code that is specific to given MCUs and peripheral
//! layouts.

use cfg_if::cfg_if;
use dronecan::hardware::CanClock;
use hal::{clocks::CrsSyncSrc, spi::BaudRate, gpio::{Port::{self, A, B, C, D,E, F, G}}};

type PortPin = (Port, u8);
type PortPinAlt = (Port, u8, u8);

#[cfg(feature = "h7")]
pub const CAN_CLOCK: CanClock = CanClock::Mhz80;
#[cfg(feature = "g4")]
pub const CAN_CLOCK: CanClock = CanClock::Mhz170;

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

// Choose PSC and ARR to get a target frequency. For example,  300 or 500Hz.
// Generally you can go up to a maximum value. Pulse high time is the predominant
// factor in servo rotation. Higher pulses generally correspond to CCW motion.

// 170Mhz tim clock on G4.
// 240Mhz or 240Mhz tim clock on H743
// 260Mhz tim clock on H723 @ 520Mhz. 275Mhz @ 550Mhz
cfg_if! {
    if #[cfg(feature = "h7")] {
        // 240Mhz tim clock.
        // pub const PSC_SERVOS: u16 = 7;
        // pub const ARR_SERVOS: u32 = 59_999;
        // 200Mhz tim clock.
        pub const PSC_SERVOS: u16 = 7;
        pub const ARR_SERVOS: u32 = 49_999;
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

// Set the IMU bad to the highest convenient speed under 24Mhz.
cfg_if! {
    if #[cfg(feature = "h7")] {
        // On H7, we run PLLQ1 as the SPI1 clock (default). We configure it to run at 80Mhz.
        pub const IMU_BAUD_DIV: BaudRate = BaudRate::Div4;
    } else {
        // for ICM426xx, for 24Mhz limit. 170Mhz / 8 = ~21Mhz.
        pub const IMU_BAUD_DIV: BaudRate = BaudRate::Div8;
    }
}

// Pins
cfg_if! {
    if #[cfg(feature = "h7")] {
        pub const PIN_BATT_ADC: PortPin = (A, 4); // ADC12, channel 18
        pub const PIN_CURR_ADC: PortPin = (A, 0);  // ADC1, channel 16

        pub const PIN_SCK2: PortPin = (A, 9);

        pub const PIN_CRSF_TX: PortPinAlt = (B, 4, 11); // UART 7
        pub const PIN_CRSF_RX: PortPinAlt = (B, 3, 11);

        pub const PIN_OSD_TX: PortPinAlt = (A, 2, 7); // UART 2
        pub const PIN_OSD_RX: PortPinAlt = (A, 3, 7);

        pub const PIN_CS_IMU: PortPin = (C, 4);
    } else {
        pub const PIN_BATT_ADC: PortPin = (A, 1);  // ADC12, channel 1
        pub const PIN_CURR_ADC: PortPin = (B, 2);  // ADC2, channel 12

        pub const PIN_SCK2: PortPin = (B, 13);

        pub const PIN_CRSF_TX: PortPinAlt = (B, 3, 7); // UART 2
        pub const PIN_CRSF_RX: PortPinAlt = (B, 4, 7);

        pub const PIN_OSD_TX: PortPinAlt = (C, 10, 5);  // UART 4
        pub const PIN_OSD_RX: PortPinAlt = (C, 11, 5);

        pub const PIN_CS_IMU: PortPin = (B, 12);
    }
}

pub const PIN_MISO2: PortPin = (B, 14);
pub const PIN_MOSI2: PortPin = (B, 15);