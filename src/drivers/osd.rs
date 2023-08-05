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

use core::{
    f32::consts::TAU,
    sync::atomic::{AtomicBool, Ordering},
};

use crate::{
    flight_ctrls::autopilot::{self, AutopilotStatus},
    protocols::msp::{MsgType, Packet, METADATA_SIZE_V1, MSG_ID_DP, MSG_ID_STATUS},
    safety::ArmStatus,
    setup::{self, UartOsd},
};

use ahrs::ppks::PositVelEarthUnits;

use stm32_hal2::dma::DmaChannel;

// todo temp
use stm32_hal2::pac::UART4;

use defmt::println;

pub const BAUD: u32 = 115_200; // 230400 allowed if "Fast_serial" is enabled

const METADATA_SIZE_WRITE_PACKET: usize = 4;

// An OSD position of 234 indicates the element is not visible.
// const NOT_VISIBLE: u16 = 234;

// // If you need more BF status flags, see the ref library above.
// const ARM_ACRO_BF: u8 = 1;

// const AUTOPILOT_DATA_SIZE: usize = NAME_SIZE;

// We use this to make sure OSD writes don't step on each other.
pub static OSD_WRITE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

static mut OSD_BUF: [u8; 300] = [0; 300]; // todo: size A/R

// todo: Periodically send heartbeat? Receive canvas?

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
}

fn make_heartbeat_packet<'a>() -> Packet<'a> {
    Packet::new(
        MsgType::Request,
        MSG_ID_DP as u16,
        1,
        &[SubCommand::Heartbeat as u8],
    )
}

fn make_clear_packet<'a>() -> Packet<'a> {
    Packet::new(
        MsgType::Request,
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

    Packet::new(MsgType::Request, MSG_ID_DP as u16, 4 + text.len(), payload)
}

fn make_draw_packet<'a>() -> Packet<'a> {
    Packet::new(
        MsgType::Request,
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

/// Sends data for all relevant elements to the OSD. Accepts a data struct built from select
/// elements from the rest of our program, and sends to the display in OSD format, using
/// only elements supported by DJI's MSP implementation.
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

    if OSD_WRITE_IN_PROGRESS.load(Ordering::Acquire) {
        // todo: Address
        // todo: DO you need this?
        return;
    }
    OSD_WRITE_IN_PROGRESS.store(true, Ordering::Release);

    let buf = unsafe { &mut OSD_BUF };

    let mut i = 0;

    // todo: A bit sloppy; rate may vary depending on upstream settings.
    static mut I: u32 = 0;
    unsafe {
        I += 1;
        if I % 100 == 0 {
            let mut payload = [0; 11];
            let packet = Packet::new(MsgType::Response, MSG_ID_STATUS as u16, 11, &payload);

            // packet.to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 11]);
            // i += i + METADATA_SIZE_V1 + 11;

            // todo: Exeperiment with timing.
            // make_heartbeat_packet().to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 1]);
            // i += i + METADATA_SIZE_V1 + 1;
        }
    }

    // Send a status packet indicating it's armed. Not sure what the fields are.
    let mut payload = [0; 11];
    payload[6] = ArmStatusMsp::Armed as u8;
    // payload[6] = ArmStatusMsp::from_arm_status(data.arm_status) as u8;
    let packet = Packet::new(MsgType::Response, MSG_ID_STATUS as u16, 11, &payload);

    make_clear_packet().to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 1]);
    i += i + METADATA_SIZE_V1 + 1;

    add_to_write_buf::<{ 18 + METADATA_SIZE_WRITE_PACKET }>(
        buf,
        6,
        3,
        &[
            9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
        ],
        &mut i,
    );

    let mut str_buf = [0; 3];
    let voltage_int = (data.battery_voltage * 10.) as u8; // todo temp
    add_to_write_buf::<{ 4 + METADATA_SIZE_WRITE_PACKET }>(buf, 20, 6, "-._V".as_bytes(), &mut i);

    // todo: Current?

    let mut str_buf = [0; 3];
    let alt_int = (data.alt_msl_baro * 10.) as u8; // todo temp
    add_to_write_buf::<{ 4 + METADATA_SIZE_WRITE_PACKET }>(buf, 10, 10, "-._m".as_bytes(), &mut i);

    let mut str_buf = [0; 3];
    let airspeed_int = (data.posit_vel.velocity.magnitude() * 10.) as u8; // todo temp
    add_to_write_buf::<{ 6 + METADATA_SIZE_WRITE_PACKET }>(buf, 10, 2, "-._m/s".as_bytes(), &mut i);

    //
    // // MSP format stores coordinates in 10^6 degrees. Our internal format is radians.
    // // TAU radians in 360 degrees. 1 degree = TAU/360 rad
    // let raw_gps = RawGps {
    //     lat: (data.posit_vel.lat_e8 / 10) as i32,
    //     lon: (data.posit_vel.lon_e8 / 10) as i32,
    //     alt: data.posit_vel.elevation_msl as i16,
    //     ..Default::default()
    // };
    //
    // let mut buf = [0; RAW_GPS_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::RawGps,
    //     &raw_gps.to_buf(),
    //     RAW_GPS_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //
    // let attitude = Attitude {
    //     roll: (to_degrees(data.roll) * EULER_ANGLE_SCALE_FACTOR) as i16,
    //     pitch: (to_degrees(data.pitch) * EULER_ANGLE_SCALE_FACTOR) as i16,
    //     yaw: (to_degrees(data.yaw) * EULER_ANGLE_SCALE_FACTOR) as i16,
    // };
    //
    // let mut buf = [0; ATTITUDE_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::Attitude,
    //     &attitude.to_buf(),
    //     ATTITUDE_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //

    // let mut buf = [0; ALTITUDE_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::Altitude,
    //     &altitude.to_buf(),
    //     ALTITUDE_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //
    // // todo: Fill this in once you have bidir dshot setup. Could add temp as well.
    // let esc_sensor_data = EscSensorData {
    //     motor_count: 0,
    //     temperature: 0,
    //     rpm: 0,
    // };
    //
    // let mut buf = [0; EC_SENSOR_DATA_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::EscSensorData,
    //     &esc_sensor_data.to_buf(),
    //     EC_SENSOR_DATA_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //
    // // Send initial configuration data to the display. This positions each element, and sets elements
    // // unsupported by the DJI MSP API to not visible.

    //
    // let mut buf = [0; OSD_CONFIG_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::OsdConfig,
    //     &osd_config.to_buf(),
    //     OSD_CONFIG_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //
    // static mut i: u32 = 0;
    // unsafe {
    //     i += 1;
    //
    //     if i % 4_000 == 0 {
    //         println!("OSD buf: {:?}", BUF_OSD);
    //     }
    // }
    //
    // // Send all the data we've compiled into the buffer.
    // unsafe {
    //     uart.write_dma(
    //         &BUF_OSD,
    //         dma_chan,
    //         Default::default(),
    //         setup::OSD_DMA_PERIPH,
    //     )
    // };

    make_draw_packet().to_buf_v1(&mut buf[i..i + METADATA_SIZE_V1 + 1]);
    i += i + METADATA_SIZE_V1 + 1;

    unsafe {
        if I % 1000 == 0 {
            println!("OSD buf len: {:?}", i);
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
