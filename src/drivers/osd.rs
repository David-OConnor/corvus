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

use core::f32::consts::TAU;

use crate::{
    flight_ctrls::autopilot::AutopilotStatus,
    protocols::msp::{MsgType, Packet, METADATA_SIZE_V1},
    safety::ArmStatus,
    setup::{self, UartOsd},
};

use ahrs::ppks::PositVelEarthUnits;

use crate::flight_ctrls::autopilot;
use stm32_hal2::dma::DmaChannel;

use defmt::println;

pub const BAUD: u32 = 115_200;

const MSP_CMD: u8 = 182;

// An OSD position of 234 indicates the element is not visible.
// const NOT_VISIBLE: u16 = 234;

// These scale factors are used by MSP to store degrees as integers with suitable precision.
const EULER_ANGLE_SCALE_FACTOR: f32 = 10.;

// // If you need more BF status flags, see the ref library above.
// const ARM_ACRO_BF: u8 = 1;

// const AUTOPILOT_DATA_SIZE: usize = NAME_SIZE;

#[derive(Clone, Copy)]
#[repr(u8)]
enum SubCommand {
    Heartbeat = 0,
    Release = 1,
    ClearScreen = 2,
    WriteString = 3,
    DrawScreen = 4,
    Options = 5,
    Sys = 6,
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
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub autopilot: AutopilotData,
    /// Distance and bearing to the base point (usually takeoff location), in m, radians respectively.
    pub base_dist_bearing: (f32, f32),
    pub link_quality: u8, // Same format as CRSF uses.
    pub num_satellites: u8,
}

fn make_clear_packet<'a>() -> Packet<'a> {
    Packet::new(
        MsgType::Request,
        MSP_CMD as u16,
        1,
        &[SubCommand::ClearScreen as u8],
    )
}

fn make_write_packet<'a>(
    payload: &'a mut [u8],
    row: u8,
    col: u8,
    attribute: u8,
    text: &str,
) -> Packet<'a> {
    payload[0] = SubCommand::WriteString as u8;
    payload[1] = row;
    payload[2] = col;
    payload[3] = attribute;
    payload[4..4 + text.len()].copy_from_slice(text.as_bytes());

    Packet::new(MsgType::Request, MSP_CMD as u16, 4 + text.len(), payload)
}

fn make_draw_packet<'a>() -> Packet<'a> {
    Packet::new(
        MsgType::Request,
        MSP_CMD as u16,
        1,
        &[SubCommand::DrawScreen as u8],
    )
}

/// Sends data for all relevant elements to the OSD. Accepts a data struct built from select
/// elements from the rest of our program, and sends to the display in OSD format, using
/// only elements supported by DJI's MSP implementation.
pub fn send_osd_data(uart: &mut UartOsd, dma_chan: DmaChannel, data: &OsdData) {
    // todo: Running list of things to add. May be supported by MSP, or co-opt elements they're not
    // made for.
    // - AGL altitude
    // - Autopilot modes
    // - Flight mode
    // - RSSI, LQ, and Tx power
    // - Steer point name and number
    // - Symbols on home plate, steerpoint etc?
    // - Land point data

    // todo: You're currently using blocking writes; use DMA.

    let mut buf = [0; METADATA_SIZE_V1 + 1];
    let clear_packet = make_clear_packet();
    clear_packet.send_v1(&mut buf, uart, dma_chan);

    let mut buf = [0; 10 + METADATA_SIZE_V1];
    let mut payload = [0; 10];
    let write_packet = make_write_packet(&mut payload, 3, 3, 0, "Corvus");
    write_packet.send_v1(&mut buf, uart, dma_chan);

    let mut buf = [0; 1 + METADATA_SIZE_V1];
    let draw_packet = make_draw_packet();
    draw_packet.send_v1(&mut buf, uart, dma_chan);

    static mut i: u32 = 0;
    unsafe {
        i += 1;

        if i % 4_000 == 0 {
            // println!("OSD buf: {:?}", BUF_OSD);
            // println!("OSD buf: {:?}", buf);
        }
    }

    // Send all the data we've compiled into the buffer.
    // unsafe {
    //     uart.write_dma(
    //         &BUF_OSD,
    //         dma_chan,
    //         Default::default(),
    //         setup::OSD_DMA_PERIPH,
    //     )
    // };

    // todo: End test
    //
    // // Asci digits. Digit, ASCII value
    // // 0 	48
    // // 1 	49
    // // 2 	50
    // // 3 	51
    // // 4 	52
    // // 5 	53
    // // 6 	54
    // // 7 	55
    // // 8 	56
    // // 9 	57
    //
    // // let pid_display: [u8; NAME_SIZE] = "test".to_buf();
    //
    // let mut buf = [0; NAME_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::Name,
    //     &data.autopilot.to_str(),
    //     NAME_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //
    // let status_bf = StatusBf {
    //     flight_mode_flags: ARM_ACRO_BF as u32,
    //     arming_disable_flags_count: 1,
    //     arming_disable_flags: 0,
    //     ..Default::default()
    // };
    //
    // let mut buf = [0; STATUS_BF_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::Status,
    //     &status_bf.to_buf(),
    //     STATUS_BF_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
    //
    // let battery_state = BatteryState {
    //     amperage: data.current_draw as u16, // todo: Find the conversion factor
    //     battery_voltage: data.battery_voltage as u16, // todo: Find the conversion factor
    //     ..Default::default()
    // };
    //
    // let mut buf = [0; BATTERY_STATE_SIZE + METADATA_SIZE_V1];
    // add_to_buf(
    //     Function::BatteryState,
    //     &battery_state.to_buf(),
    //     BATTERY_STATE_SIZE,
    //     &mut buf,
    //     &mut buf_i,
    // );
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
    // let altitude = Altitude {
    //     estimated_actual_position: 0,                   // todo
    //     estimated_actual_velocity: 0,                   // todo
    //     baro_latest_altitude: data.alt_msl_baro as i32, // todo: Find conversion factor, if there is one.
    // };
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
    // // OSD elements positions.
    // // Horizontally 2048-2074(spacing 1),
    // // vertically 2048-2528(spacing 32). 26 characters X 15 lines
    // // todo: Clarify how this works.
    // // in betaflight configurator set OSD elements to your desired positions and in CLI type
    // // "set osd" to retreieve the numbers.
    // let osd_config = OsdConfig {
    //     units: 0,
    //     item_count: 56,
    //     stat_count: 24,
    //     timer_count: 2,
    //     warning_count: 16,     // 16
    //     profile_count: 1,      // 1
    //     osdprofileindex: 1,    // 1
    //     overlay_radio_mode: 0, //  0
    //     rssi_value_pos: 2_176,
    //     main_batt_voltage_pos: 2073,
    //     crosshairs_pos: NOT_VISIBLE,
    //     artificial_horizon_pos: NOT_VISIBLE,
    //     horizon_sidebars_pos: NOT_VISIBLE,
    //     item_timer_1_pos: NOT_VISIBLE,
    //     item_timer_2_pos: NOT_VISIBLE,
    //     flymode_pos: NOT_VISIBLE,
    //     craft_name_pos: 2_543,
    //     throttle_pos_pos: NOT_VISIBLE,
    //     vtx_channel_pos: NOT_VISIBLE,
    //     current_draw_pos: 2_103,
    //     mah_drawn_pos: 2_138,
    //     gps_speed_pos: 2_475,
    //     gps_sats_pos: 2_112,
    //     altitude_pos: 2_476,
    //     roll_pids_pos: NOT_VISIBLE,
    //     pitch_pids_pos: NOT_VISIBLE,
    //     yaw_pids_pos: NOT_VISIBLE,
    //     power_pos: NOT_VISIBLE,
    //     pidrate_profile_pos: NOT_VISIBLE,
    //     warnings_pos: NOT_VISIBLE,
    //     avg_cell_voltage_pos: NOT_VISIBLE,
    //     gps_lon_pos: 2_080,
    //     gps_lat_pos: 2_048,
    //     debug_pos: NOT_VISIBLE,
    //     pitch_angle_pos: NOT_VISIBLE,
    //     roll_angle_pos: NOT_VISIBLE,
    //     main_batt_usage_pos: NOT_VISIBLE,
    //     disarmed_pos: 2_125,
    //     home_dir_pos: 2_093,
    //     home_dist_pos: 2_331,
    //     numerical_heading_pos: NOT_VISIBLE,
    //     numerical_vario_pos: 2_482,
    //     compass_bar_pos: NOT_VISIBLE,
    //     esc_tmp_pos: NOT_VISIBLE,
    //     esc_rpm_pos: NOT_VISIBLE,
    //     remaining_time_estimate_pos: NOT_VISIBLE,
    //     rtc_datetime_pos: NOT_VISIBLE,
    //     adjustment_range_pos: NOT_VISIBLE,
    //     core_temperature_pos: NOT_VISIBLE,
    //     anti_gravity_pos: NOT_VISIBLE,
    //     g_force_pos: NOT_VISIBLE,
    //     motor_diag_pos: NOT_VISIBLE,
    //     log_status_pos: NOT_VISIBLE,
    //     flip_arrow_pos: NOT_VISIBLE,
    //     link_quality_pos: NOT_VISIBLE,
    //     flight_dist_pos: NOT_VISIBLE,
    //     stick_overlay_left_pos: NOT_VISIBLE,
    //     stick_overlay_right_pos: NOT_VISIBLE,
    //     display_name_pos: NOT_VISIBLE,
    //     esc_rpm_freq_pos: NOT_VISIBLE,
    //     rate_profile_name_pos: NOT_VISIBLE,
    //     pid_profile_name_pos: NOT_VISIBLE,
    //     profile_name_pos: NOT_VISIBLE,
    //     rssi_dbm_value_pos: NOT_VISIBLE,
    //     rc_channels_pos: NOT_VISIBLE,
    //     ..Default::default()
    // };
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
}
