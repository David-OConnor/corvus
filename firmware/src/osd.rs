//! This module contains code for interfacing with DJI's OSD system, via UART.
//! It uses the MSP protocol. Its API accepts a struct containing OSD data, passed from
//! elsewhere in our program, and matches this to the MSP format. It sends this data, including
//! a mandatory (for DJI) arm status; required for DJI to use high power mode.
//!
//! Note that this isn't ideal in that it doesn't support "canvas", or pixel buffers.
//! Worry about that later as the ecosystem changes.
//! Info on canvas and MSP: https://discuss.ardupilot.org/t/msp-displayport/73155/33
//!
//! Based on DeathByHotGlue's library:
//! https://github.com/chris1seto/PX4-Autopilot/tree/turbotimber/src/modules/msp_osd

use core::f32::consts::TAU;

use defmt::{info, println, warn};

use crate::{
    ppks::Location,
    protocols::{
        msp::{self, MsgType, Packet},
        msp_defines::*,
    },
    safety::ArmStatus,
};

use stm32_hal2::{
    dma::{Dma, DmaChannel},
    pac::{DMA1, USART2},
    usart::Usart,
};

// An OSD position of 234 indicates the element is not visible.
const NOT_VISIBLE: u16 = 234;

// These scale factors are used by MSP to store degrees as integers with suitable precision.
const GPS_SCALE_FACTOR: f32 = 10_000_000.;
const EULER_ANGLE_SCALE_FACTOR: f32 = 10.;

// If you need more BF status flags, see the ref library above.
const ARM_ACRO_BF: u8 = 1;

/// Convert radians to degrees.
fn to_degrees(val_rad: f32) -> f32 {
    TAU / 360. * val_rad
}

/// Contains all data we pass to the OSD. Passed from the main FC firmware.
pub struct OsdData {
    pub arm_status: ArmStatus,
    pub battery_voltage: f32,
    pub current_draw: f32, // mA
    pub alt_msl_baro: f32, // m
    pub gps_fix: Location,
    pub motor_count: u8,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

/// Sends data for all relevant elements to the OSD. Accepts a data struct built from select
/// elements from the rest of our program, and sends to the display in OSD format, using
/// only elements supported by DJI's MSP implementation.
pub fn send_osd_data(
    uart: &mut Usart<USART2>,
    dma_chan: DmaChannel,
    dma: &mut Dma<DMA1>,
    data: &OsdData,
) {
    // todo: Running list of things to add. May be supported by MSP, or co-opt elements they're not
    // made for.
    // - AGL altitude
    // - Autopilot modes
    // - Flight mode
    // - RSSI, LQ, and Tx power
    // - Steer point name and number
    // - Symbols on home plate, steerpoint etc?
    // - Land point data

    let status_bf = StatusBf {
        flight_mode_flags: ARM_ACRO_BF as u32,
        arming_disable_flags_count: 1,
        arming_disable_flags: 0,
        ..Default::default()
    };

    let packet = Packet::new(MsgType::Request, Function::Status, STATUS_BF_SIZE);
    msp::send_packet(uart, dma_chan, dma, &packet, &status_bf.to_buf());

    let mut battery_state = BatteryState {
        amperage: data.current_draw as u16, // todo: Find the conversion factor
        battery_voltage: data.battery_voltage as u16, // todo: Find the conversion factor
        ..Default::default()
    };

    let packet = Packet::new(MsgType::Request, Function::BatteryState, BATTERY_STATE_SIZE);
    msp::send_packet(uart, dma_chan, dma, &packet, &battery_state.to_buf());

    // MSP format stores coordinates in 10^6 degrees. Our internal format is radians.
    // TAU radians in 360 degrees. 1 degree = TAU/360 rad
    let raw_gps = RawGps {
        lat: (to_degrees(data.gps_fix.y) * GPS_SCALE_FACTOR) as i32,
        lon: (to_degrees(data.gps_fix.x) * GPS_SCALE_FACTOR) as i32,
        alt: data.gps_fix.z as i16,
        ..Default::default()
    };

    let packet = Packet::new(MsgType::Request, Function::RawGps, RAW_GPS_SIZE);
    msp::send_packet(uart, dma_chan, dma, &packet, &raw_gps.to_buf());

    let attitude = Attitude {
        roll: (to_degrees(data.roll) * EULER_ANGLE_SCALE_FACTOR) as i16,
        pitch: (to_degrees(data.pitch) * EULER_ANGLE_SCALE_FACTOR) as i16,
        yaw: (to_degrees(data.yaw) * EULER_ANGLE_SCALE_FACTOR) as i16,
    };

    let packet = Packet::new(MsgType::Request, Function::Attitude, ATTITUDE_SIZE);
    msp::send_packet(uart, dma_chan, dma, &packet, &attitude.to_buf());

    let altitude = Altitude {
        estimated_actual_position: 0,                   // todo
        estimated_actual_velocity: 0,                   // todo
        baro_latest_altitude: data.alt_msl_baro as i32, // todo: Find conversion factor, if there is one.
    };

    let packet = Packet::new(MsgType::Request, Function::Altitude, ALTITUDE_SIZE);
    msp::send_packet(uart, dma_chan, dma, &packet, &altitude.to_buf());

    // todo: Fill this in once you have bidir dshot setup. Could add temp as well.
    let esc_sensor_data = EscSensorData {
        motor_count: data.motor_count,
        temperature: 0,
        rpm: 0,
    };

    let packet = Packet::new(
        MsgType::Request,
        Function::EscSensorData,
        EC_SENSOR_DATA_SIZE,
    );
    msp::send_packet(uart, dma_chan, dma, &packet, &esc_sensor_data.to_buf());

    send_config(uart, dma_chan, dma);
}

/// Send initial configuration data to the display. This positions each element, and sets elements
/// unsupported by the DJI MSP API to not visible.
fn send_config(uart: &mut Usart<USART2>, dma_chan: DmaChannel, dma: &mut Dma<DMA1>) {
    // OSD elements positions.
    // Horizontally 2048-2074(spacing 1),
    // vertically 2048-2528(spacing 32). 26 characters X 15 lines
    // todo: Clarify how this works.
    // in betaflight configurator set OSD elements to your desired positions and in CLI type
    // "set osd" to retreieve the numbers.
    let config = OsdConfig {
        units: 0,
        item_count: 56,
        stat_count: 24,
        timer_count: 2,
        warning_count: 16,     // 16
        profile_count: 1,      // 1
        osdprofileindex: 1,    // 1
        overlay_radio_mode: 0, //  0
        rssi_value_pos: 2_176,
        main_batt_voltage_pos: 2073,
        crosshairs_pos: NOT_VISIBLE,
        artificial_horizon_pos: NOT_VISIBLE,
        horizon_sidebars_pos: NOT_VISIBLE,
        item_timer_1_pos: NOT_VISIBLE,
        item_timer_2_pos: NOT_VISIBLE,
        flymode_pos: NOT_VISIBLE,
        craft_name_pos: 2_543,
        throttle_pos_pos: NOT_VISIBLE,
        vtx_channel_pos: NOT_VISIBLE,
        current_draw_pos: 2_103,
        mah_drawn_pos: 2_138,
        gps_speed_pos: 2_475,
        gps_sats_pos: 2_112,
        altitude_pos: 2_476,
        roll_pids_pos: NOT_VISIBLE,
        pitch_pids_pos: NOT_VISIBLE,
        yaw_pids_pos: NOT_VISIBLE,
        power_pos: NOT_VISIBLE,
        pidrate_profile_pos: NOT_VISIBLE,
        warnings_pos: NOT_VISIBLE,
        avg_cell_voltage_pos: NOT_VISIBLE,
        gps_lon_pos: 2_080,
        gps_lat_pos: 2_048,
        debug_pos: NOT_VISIBLE,
        pitch_angle_pos: NOT_VISIBLE,
        roll_angle_pos: NOT_VISIBLE,
        main_batt_usage_pos: NOT_VISIBLE,
        disarmed_pos: 2_125,
        home_dir_pos: 2_093,
        home_dist_pos: 2_331,
        numerical_heading_pos: NOT_VISIBLE,
        numerical_vario_pos: 2_482,
        compass_bar_pos: NOT_VISIBLE,
        esc_tmp_pos: NOT_VISIBLE,
        esc_rpm_pos: NOT_VISIBLE,
        remaining_time_estimate_pos: NOT_VISIBLE,
        rtc_datetime_pos: NOT_VISIBLE,
        adjustment_range_pos: NOT_VISIBLE,
        core_temperature_pos: NOT_VISIBLE,
        anti_gravity_pos: NOT_VISIBLE,
        g_force_pos: NOT_VISIBLE,
        motor_diag_pos: NOT_VISIBLE,
        log_status_pos: NOT_VISIBLE,
        flip_arrow_pos: NOT_VISIBLE,
        link_quality_pos: NOT_VISIBLE,
        flight_dist_pos: NOT_VISIBLE,
        stick_overlay_left_pos: NOT_VISIBLE,
        stick_overlay_right_pos: NOT_VISIBLE,
        display_name_pos: NOT_VISIBLE,
        esc_rpm_freq_pos: NOT_VISIBLE,
        rate_profile_name_pos: NOT_VISIBLE,
        pid_profile_name_pos: NOT_VISIBLE,
        profile_name_pos: NOT_VISIBLE,
        rssi_dbm_value_pos: NOT_VISIBLE,
        rc_channels_pos: NOT_VISIBLE,
        ..Default::default()
    };

    let packet = Packet::new(MsgType::Request, Function::OsdConfig, OSD_CONFIG_SIZE);
    msp::send_packet(uart, dma_chan, dma, &packet, &config.to_buf());
}
