//! This module contains code for interfacing with MSP Displayport OSDs, via UART.
//! It uses the MSPv1 protocol. Its API accepts a struct containing OSD data, passed from
//! elsewhere in our program, and matches this to the MSP format. It sends this data, including
//! a mandatory (for DJI) arm status; required for DJI to use high power mode.
//!
//! Note that this isn't ideal in that it doesn't support "canvas", or pixel buffers.
//! Worry about that later as the ecosystem changes.
//! Info on canvas and MSP: https://discuss.ardupilot.org/t/msp-displayport/73155/33
//!
//! Clean simple implemenation of OSD: https://github.com/catphish/openuav/blob/master/firmware/src/msp.c
//!
//! [Reference for DisplayPort commands](https://betaflight.com/docs/development/API/DisplayPort)
//!
//! Note: We use `response` packets for all messages, because that's what WTF-FPV requires.
//! DJI's O3 can handle either. `request` makes more sense semantically, but there's no standard
//! in place to enforce that.

use core::{
    f32::consts::TAU,
    sync::atomic::{AtomicBool, Ordering},
};

use ahrs::ppks::PositVelEarthUnits;
use defmt::println;
use hal::dma::DmaChannel;

use crate::{
    flight_ctrls::autopilot::{self, AutopilotStatus},
    protocols::msp::{Direction, Packet, METADATA_SIZE_V1, MSG_ID_DP, MSG_ID_STATUS},
    safety::ArmStatus,
    sensors_shared::BattCellCount,
    setup::{self, UartOsd},
    util,
};

pub const BAUD: u32 = 115_200; // 230400 allowed if "Fast_serial" is enabled

const METADATA_SIZE_WRITE_PACKET: usize = 4;

// This is such a hack, but I'm not sure why I can't properly read the OSD rx UART.
// instead we cyucle the interrupt.
pub static OSD_INTERRUPT_CYCLE: AtomicBool = AtomicBool::new(false);

// An OSD position of 234 indicates the element is not visible.
// const NOT_VISIBLE: u16 = 234;

// // If you need more BF status flags, see the ref library above.
// const ARM_ACRO_BF: u8 = 1;

// const AUTOPILOT_DATA_SIZE: usize = NAME_SIZE;

// We use this to make sure OSD writes don't step on each other.
pub static OSD_WRITE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

static mut OSD_TX_BUF: [u8; 180] = [0; 180]; // Adjust size A/R as you adjust what's displayed.

// Just big enough to read the fucntion type, so we can reply if it's a status frame.
// pub static mut OSD_READ_BUF: [u8; 5] = [0; 5];
pub static mut OSD_ARM_BUF: [u8; 21] = [0; 21];

// WTF font map
// font map. f: Directional arrow
// font map. g: Directional arrow NNE
// e w nw
//h arrow fwd
// p still an arrow
// q and r are skippable
// s: Velocity vector
// u: up carret

// DJI Jfont map

// DJI O3 font map
// a - 0: Arrows
// r - good velocity vector
// stufw: Carrot-sytyle arrows
// x: thermometer
// todo; Possibly need to shift these up to 3

// todo: Periodically send heartbeat? Receive canvas?
//
// /// Split a float at the decimal point, for use with integeral-to-string
// /// heapless no_std conversions, like `numtoa`.
// fn split_at_decimal(v: f32, precision: u8) -> (u16, u16) {
//     let multiplier = 10_i16.pow(precision as u32); // 10^decimal precision
//     let rhs = v % 1.;
//
//     let mut l = (v - rhs) as u16;
//     let mut r = (rhs * multiplier as f32).round() as u16;
//
//     // This occurs for rhs >= .95 . We need to zero it, and round up the
//     // LHS in this case.
//     if r == 10 {
//         r = 0;
//         l += 1;
//     }
//
//     (l, r)
// }

// /// Process a float into a pair of strings: left of decimal, right of decimal.
// fn float_to_strs<'a>(
//     v: f32,
//     buffer_l: &'a mut [u8],
//     buffer_r: &'a mut [u8],
//     precision: u8,
// ) -> (&'a str, &'a str) {
//     let (lhs, rhs) = split_at_decimal(v, precision);
//
//     let result_l = lhs.numtoa_str(10, buffer_l);
//     let result_r = rhs.numtoa_str(10, buffer_r);
//
//     (result_l, result_r)
// }
//
// /// Process a float into a &str. Return the number of characters.
// pub fn float_to_str<'a>(
//     v: f32,
//     buff_l: &'a mut [u8],
//     buff_r: &'a mut [u8],
//     buff_result: &'a mut [u8],
//     precision: u8,
// ) -> usize {
//     let (l, r) = float_to_strs(v, buff_l, buff_r, precision);
//
//     // todo: This works, but is awk. Have it return the str if able, and
//     // todo not require the post-processing. Running into ownership(?) issues.
//
//     // todo: 0-pad the rhs if required.
//     let mut i = 0;
//     for c in l.as_bytes() {
//         buff_result[i] = *c;
//         i += 1;
//     }
//
//     if precision == 0 {
//         return l.len();
//     }
//
//     buff_result[i] = b"."[0];
//     i += 1;
//     for c in r.as_bytes() {
//         buff_result[i] = *c;
//         i += 1;
//     }
//     (l.len() + 1 + r.len()) as usize
// }

/// Format a single digit as an ASCII character.
fn digit_to_char(digit: u8) -> u8 {
    match digit {
        0 => "0",
        1 => "1",
        2 => "2",
        3 => "3",
        4 => "4",
        5 => "5",
        6 => "6",
        7 => "7",
        8 => "8",
        9 => "9",
        _ => " ",
    }
    .as_bytes()[0]
}

/// Format an integer as an ASCII string buffer. Unlike 1numtoa1, doens't panic.
fn format_int(buf: &mut [u8], int: u16) {
    let mut current = int;
    for i in 0..buf.len() {
        buf[buf.len() - 1 - i] = digit_to_char((current % 10) as u8);

        current /= 10;
        if current == 0 {
            return;
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum SubCommand {
    /// Prevent OSD Slave boards from displaying a 'disconnected' status
    Heartbeat = 0,
    /// Clears the display and allows local rendering on the display device based on telemetry
    /// informtation etc.
    _Release = 1,
    ClearScreen = 2,
    WriteString = 3,
    /// Triggers the display of a frame after it has been cleared/rendered
    DrawScreen = 4,
    // todo: Use options to set resolution?
    /// Sets display resolution.
    _Options = 5,
    /// Display system element displayportSystemElement_e at given coordinates
    _Sys = 6,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum ArmStatusMsp {
    Disarmed = 0,
    Armed = 1,
}

impl ArmStatusMsp {
    fn from_arm_status(status: ArmStatus) -> Self {
        #[cfg(feature = "quad")]
        match status {
            ArmStatus::Disarmed => Self::Disarmed,
            ArmStatus::Armed => Self::Armed,
        }

        #[cfg(feature = "fixed-wing")]
        match status {
            ArmStatus::Disarmed => Self::Disarmed,
            _ => Self::Armed,
        }
    }
}

/// Convert radians to degrees.
fn to_degrees(val_rad: f32) -> f32 {
    TAU / 360. * val_rad
}

/// A terse description of autopilot modes used, so we don't need to pass the whole struct
/// to the OSD
pub struct AutopilotData {
    pub takeoff: bool,
    pub land: bool,
    pub direct_to_point: bool,
    #[cfg(feature = "fixed-wing")]
    pub orbit: bool,
    #[cfg(feature = "quad")]
    pub loiter: bool,
    pub alt_hold: bool,
}

impl AutopilotData {
    pub fn from_status(status: &AutopilotStatus) -> Self {
        Self {
            takeoff: status.takeoff,
            land: status.land.is_some(),
            direct_to_point: status.direct_to_point.is_some(),
            #[cfg(feature = "fixed-wing")]
            orbit: status.orbit.is_some(),
            alt_hold: status.alt_hold.is_some(),
            #[cfg(feature = "quad")]
            loiter: status.loiter.is_some(),
        }
    }

    // pub fn to_str(&self) -> [u8; AUTOPILOT_DATA_SIZE] {
    //     let mut result = [0; AUTOPILOT_DATA_SIZE];
    //
    //     if self.takeoff {
    //         result[0..2].clone_from_slice("TO".as_bytes());
    //     } else if self.land {
    //         result[0..2].clone_from_slice("Ld".as_bytes());
    //     } else if self.direct_to_point {
    //         result[0..2].clone_from_slice("Pt".as_bytes());
    //     }
    //
    //     // Orbit and loiter here should be part of the if/else above, but
    //     // it shouldn't matter. Separate due to limitations on feature-gate syntax.
    //     #[cfg(feature = "fixed-wing")]
    //     if self.orbit {
    //         result[0..2].clone_from_slice("Ot".as_bytes());
    //     }
    //
    //     #[cfg(feature = "quad")]
    //     if self.loiter {
    //         result[0..2].clone_from_slice("Lr".as_bytes());
    //     }
    //
    //     if self.alt_hold {
    //         // todo: Show alt commanded.
    //         result[2..5].clone_from_slice("Alt".as_bytes());
    //     }
    //
    //     result
    // }
}

/// Contains all data we pass to the OSD. Passed from the main FC firmware.
pub struct OsdData {
    pub arm_status: ArmStatus,
    pub battery_voltage: f32,
    pub current_draw: f32, // mA
    pub alt_msl_baro: f32, // m
    pub posit_vel: PositVelEarthUnits,
    pub autopilot: AutopilotData,
    /// Distance and bearing to the base point (usually takeoff location), in m, radians respectively.
    pub base_dist_bearing: (f32, f32),
    pub link_quality: u8, // Same format as CRSF uses.
    pub num_satellites: u8,
    pub batt_cell_count: BattCellCount,
    pub throttle: f32,
    pub total_acc: f32,
}

fn make_heartbeat_packet<'a>() -> Packet<'a> {
    Packet::new(
        Direction::FcToVtx,
        MSG_ID_DP as u16,
        1,
        &[SubCommand::Heartbeat as u8],
    )
}

fn make_clear_packet<'a>() -> Packet<'a> {
    Packet::new(
        Direction::FcToVtx,
        MSG_ID_DP as u16,
        1,
        &[SubCommand::ClearScreen as u8],
    )
}

fn make_write_packet<'a>(
    payload: &'a mut [u8],
    row: u8,
    col: u8,
    attribute: u8,
    // text: &str,
    text: &[u8],
) -> Packet<'a> {
    payload[0] = SubCommand::WriteString as u8;
    payload[1] = row;
    payload[2] = col;
    payload[3] = attribute;
    payload[METADATA_SIZE_WRITE_PACKET..METADATA_SIZE_WRITE_PACKET + text.len()]
        // .copy_from_slice(text.as_bytes());
        .copy_from_slice(text);

    Packet::new(
        Direction::FcToVtx,
        MSG_ID_DP as u16,
        4 + text.len(),
        payload,
    )
}

fn make_draw_packet<'a>() -> Packet<'a> {
    Packet::new(
        Direction::FcToVtx,
        MSG_ID_DP as u16,
        1,
        &[SubCommand::DrawScreen as u8],
    )
}

// todo: Instead of const generic, could skip the Packet struct, etc.
// fn add_to_write_buf<const N: usize>(buf: &mut [u8], row: u8, col: u8, text: &str, i: &mut usize) {
fn add_to_write_buf<const N: usize>(buf: &mut [u8], row: u8, col: u8, text: &[u8], i: &mut usize) {
    // let mut payload = [0; METADATA_SIZE_WRITE_PACKET + N];
    let mut payload = [0; N];
    let packet = make_write_packet(&mut payload, row, col, 0, text);

    packet.to_buf_v1(&mut buf[*i..*i + METADATA_SIZE_V1 + METADATA_SIZE_WRITE_PACKET + N]);

    *i += METADATA_SIZE_V1 + METADATA_SIZE_WRITE_PACKET + N;
}

pub fn make_arm_status_buf(armed: bool) {
    let buf = unsafe { &mut OSD_ARM_BUF };

    let mut payload = [0; 11];
    if armed {
        payload[6] = ArmStatusMsp::Armed as u8;
    } else {
        payload[6] = ArmStatusMsp::Disarmed as u8;
    }

    // payload[6] = ArmStatusMsp::from_arm_status(data.arm_status) as u8;
    let packet = Packet::new(Direction::FcToVtx, MSG_ID_STATUS as u16, 11, &payload);
    packet.to_buf_v1(&mut buf[..METADATA_SIZE_V1 + 11]);
}

/// Sends data for all relevant elements to the OSD. Accepts a data struct built from select
/// elements from the rest of our program, and sends to the display in OSD format, using
/// only elements supported by DJI's MSP implementation.
///
/// Note; You can use Mission Planner's UI to help with item placement.
pub fn send_osd_data(uart: &mut UartOsd, data: &OsdData) {
    // todo: Running list of things to add. May be supported by MSP, or co-opt elements they're not
    // made for.
    // - AGL altitude
    // - Autopilot modes
    // - Flight mode
    // - RSSI, LQ, and Tx power
    // - Steer point name and number
    // - Symbols on home plate, steerpoint etc?
    // - Land point data

    let blank = " ".as_bytes()[0];

    // This return must be before we increment I.
    if OSD_WRITE_IN_PROGRESS.load(Ordering::Acquire) {
        return;
    }

    OSD_WRITE_IN_PROGRESS.store(true, Ordering::Release);

    let buf = unsafe { &mut OSD_TX_BUF };

    let mut i = 0;

    // todo: TS arm status

    // todo: A bit sloppy; rate may vary depending on upstream settings.
    static mut I: u32 = 0;
    unsafe {
        I += 1;
        if I % 20 == 0 {
            // todo: Every time?
        }
    }

    make_heartbeat_packet().to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 1]);
    i += METADATA_SIZE_V1 + 1;

    make_clear_packet().to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 1]);
    i += METADATA_SIZE_V1 + 1;

    // todo: Anamolies on clear.

    // Link quality
    let mut lq_buf = [blank; 4];
    lq_buf[0] = "t".as_bytes()[0]; // todo: Find the correct icon in the font.
    format_int(&mut lq_buf[1..4], data.link_quality as u16);
    add_to_write_buf::<{ 4 + METADATA_SIZE_WRITE_PACKET }>(buf, 12, 13, &lq_buf, &mut i);

    // Battery voltage and % remaining.
    let mut buf_batt = [blank; 9];

    let batt_v = (data.battery_voltage * 10. / data.batt_cell_count.num_cells()) as u16;
    format_int(&mut buf_batt[0..3], batt_v);
    buf_batt[3] = "V".as_bytes()[0];

    let batt_life = util::batt_left_from_v(data.battery_voltage, data.batt_cell_count);
    let batt_pct = (batt_life * 100.) as u16;
    format_int(&mut buf_batt[5..8], batt_pct);
    buf_batt[8] = "%".as_bytes()[0];
    add_to_write_buf::<{ 9 + METADATA_SIZE_WRITE_PACKET }>(buf, 11, 10, &buf_batt, &mut i);

    // Altitude
    let mut alt_buf = [blank; 5];
    format_int(&mut alt_buf[0..4], data.alt_msl_baro as u16);
    alt_buf[4] = "M".as_bytes()[0]; // lowercase available in font?
    add_to_write_buf::<{ 5 + METADATA_SIZE_WRITE_PACKET }>(buf, 7, 25, &alt_buf, &mut i);

    // Airspeed
    let mut airspeed_buf = [blank; 6];
    let airspeed = data.posit_vel.velocity.magnitude() as u16;
    format_int(&mut airspeed_buf[0..3], airspeed);
    airspeed_buf[3..6].clone_from_slice("M/S".as_bytes()); // lowercase available in font?
    add_to_write_buf::<{ 6 + METADATA_SIZE_WRITE_PACKET }>(buf, 7, 0, &airspeed_buf, &mut i);

    // Number of sattelites
    let mut num_sats_buf = [blank; 3];
    num_sats_buf[0] = "v".as_bytes()[0]; // todo: Find the correct icon in the font.
    format_int(&mut num_sats_buf[1..3], data.num_satellites as u16);
    add_to_write_buf::<{ 3 + METADATA_SIZE_WRITE_PACKET }>(buf, 0, 13, &num_sats_buf, &mut i);

    // Throttle display.
    let mut throttle_buf = [blank; 4];
    let throttle = (data.throttle * 100.) as u16;
    format_int(&mut throttle_buf[1..4], throttle);
    throttle_buf[0] = "T".as_bytes()[0];
    add_to_write_buf::<{ 4 + METADATA_SIZE_WRITE_PACKET }>(buf, 14, 0, &throttle_buf, &mut i);

    // Total acceleration (G force) display
    let mut g_buf = [blank; 4];
    let g = (data.total_acc * 10. / 9.8) as u16;
    format_int(&mut g_buf[0..3], g);
    g_buf[3] = "G".as_bytes()[0];
    add_to_write_buf::<{ 4 + METADATA_SIZE_WRITE_PACKET }>(buf, 13, 0, &g_buf, &mut i);

    // todo: Test these once you verify working on O3.
    #[cfg(feature = "quad")]
    match data.arm_status {
        ArmStatus::Armed => {
            // add_to_write_buf::<{ 5 + METADATA_SIZE_WRITE_PACKET }>(buf, 7, 12, "ARMED".as_bytes(), &mut i);
        }
        ArmStatus::Disarmed => {
            add_to_write_buf::<{ 8 + METADATA_SIZE_WRITE_PACKET }>(
                buf,
                6,
                11,
                "DISARMED".as_bytes(),
                &mut i,
            );
        }
    }

    #[cfg(feature = "fixed-wing")]
    match data.arm_status {
        ArmStatus::MotorsControlsArmed => {
            // add_to_write_buf::<{ 5 + METADATA_SIZE_WRITE_PACKET }>(buf, 7, 12, "ARMED".as_bytes(), &mut i);
        }
        ArmStatus::ControlsArmed => {
            add_to_write_buf::<{ 14 + METADATA_SIZE_WRITE_PACKET }>(
                buf,
                7,
                12,
                "CONTROLS ARMED".as_bytes(),
                &mut i,
            );
        }
        ArmStatus::Disarmed => {
            add_to_write_buf::<{ 8 + METADATA_SIZE_WRITE_PACKET }>(
                buf,
                7,
                12,
                "DISARMED".as_bytes(),
                &mut i,
            );
        }
    }

    // todo: Base dist/bearing

    make_draw_packet().to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 1]);
    i += METADATA_SIZE_V1 + 1;

    unsafe {
        if I % 30 == 0 {
            // println!("OSD buf len: {:?}", i);
            // println!("Buf: {:x}", buf);
            // println!("Alt {}", data.alt_msl_baro );
        }
    }

    unsafe {
        uart.write_dma(
            buf,
            setup::OSD_TX_CH,
            Default::default(),
            setup::OSD_DMA_PERIPH,
        )
    };
}

// /// Map an integer to a character (ASCII byte)
// fn map_int_to_chars(num: u8, buf: &mut [u8]) -> u8 {
//     let str = match num {
//         0 => "0",
//         _ => "-",
//     };
//
//     .as_bytes()[0]
// }
