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

use defmt::{println, warn, info};

use crate::{
    prtocols::{
        msp::{self, MsgType, Packet},
        msp_defines::*,
    },
    ppks::Location,
};

use stm32_hal2::{
    usart::Usart,
    pac::USART2,
};
use protocols::msp_defines::Function::EscSensorData;

// An OSD position of 234 indicates the element is not visible.
const NOT_VISIBLE: u16 = 234;

// These scale factors are used by MSP to store degrees as integers with suitable precision.
const GPS_SCALE_FACTOR: f32 = 10_000_000.;
const EULER_ANGLE_SCALE_FACTOR: f32 = 10.;

/// Contains all data we pass to the OSD. Passed from the main FC firmware.
struct OsdData {
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
pub fn send_osd_Data(uart: &mut Uart<USART2>, data: &OsdData){

    // todo: Consider if you want to pass fc variant and craft name. Skipping fnor now.
    // let packet = Packet::new(MsgType::Request, Function::FcVariant, FC_VARIANT_SIZE);
    // msp::send_packet(uart, packet, &variant.to_buf());

    // let packet = Packet::new(MsgType::Request, Function::Name, CRAFT_NAME_SIZE);
    // msp::send_packet(uart, packet, &name.to_buf());

        if _vehicle_status_struct.arming_state == _vehicle_status_struct.ARMING_STATE_ARMED {
        status_BF.flight_mode_flags |= ARM_ACRO_BF;

        match _vehicle_status_struct.nav_state {
            _vehicle_status_struct.NAVIGATION_STATE_MANUAL => {
                status_BF.flight_mode_flags |= 0;
            }

            _vehicle_status_struct.NAVIGATION_STATE_ACRO => {
                status_BF.flight_mode_flags |= 0;
            }

            _vehicle_status_struct.NAVIGATION_STATE_STAB => {
                status_BF.flight_mode_flags |= STAB_BF;
            }

            _vehicle_status_struct.NAVIGATION_STATE_AUTO_RTL => {
                status_BF.flight_mode_flags |= RESC_BF;
            }

            _vehicle_status_struct.NAVIGATION_STATE_TERMINATION => {
                status_BF.flight_mode_flags |= FS_BF;
            }

            _ => {
                status_BF.flight_mode_flags = 0;
            }
        }
    }

    status_BF.arming_disable_flags_count = 1;
    status_BF.arming_disable_flags  = !(_vehicle_status_struct.arming_state == _vehicle_status_struct.ARMING_STATE_ARMED);

    let packet = Packet::new(MsgType::Request, Function::Status, 100);
    msp::send_packet(uart, packet, &status_BF);

    // MSP_ANALOG
    analog.vbat = (_battery_status_struct.voltage_v * 10.0) / _battery_status_struct.cell_count; // bottom right... v * 10
    analog.rssi = ((_input_rc_struct.link_quality * 1024.0) / 100.0) as u16;
    analog.amperage = _battery_status_struct.current_a * 100; // main amperage
    analog.mAhDrawn = _battery_status_struct.discharged_mah; // unused

    let packet = Packet::new(MsgType::Request, Function::Analog, 100);

    msp::send_packet(uart, packet, &analog);

    let mut battery_state = BatteryState {
        amperage: data.current_draw,
        battery_voltage: batt_voltage,
        ..Default::default()
    };

    let packet = Packet::new(MsgType::Request, Function::BatteryState, BATTERY_STATE_SIZE);
    msp::send_packet(uart, packet, &battery_state.to_buf());

    // MSP_RAW_GPS
    if _vehicle_gps_position_struct.fix_type >= 2 {
        raw_gps.lat = _vehicle_gps_position_struct.lat;
        raw_gps.lon = _vehicle_gps_position_struct.lon;
        raw_gps.alt =  _vehicle_gps_position_struct.alt / 10;
        let mut ground_course = math::degrees(_vehicle_gps_position_struct.cog_rad);

        if ground_course < 0 {
            ground_course += 360.0;
        }

        //raw_gps.groundCourse = ground_course;
        altitude.estimatedActualPosition = _vehicle_gps_position_struct.alt / 10;
    } else {
        raw_gps.lat = 0;
        raw_gps.lon = 0;
        raw_gps.alt = 0;
        raw_gps.groundCourse = 0;
        altitude.estimatedActualPosition = 0;
    }

    if _vehicle_gps_position_struct.fix_type == 0
        || _vehicle_gps_position_struct.fix_type == 1 {
        raw_gps.fixType = MSP_GPS_NO_FIX;
    } else if _vehicle_gps_position_struct.fix_type == 2 {
        raw_gps.fixType = MSP_GPS_FIX_2D;
    } else if _vehicle_gps_position_struct.fix_type >= 3 && _vehicle_gps_position_struct.fix_type <= 5 {
        raw_gps.fixType = MSP_GPS_FIX_3D;
    } else {
        raw_gps.fixType = MSP_GPS_NO_FIX;
    }

    raw_gps.numSat = _vehicle_gps_position_struct.satellites_used;

    if _airspeed_validated_struct.airspeed_sensor_measurement_valid
        && _airspeed_validated_struct.indicated_airspeed_m_s != NAN
        && _airspeed_validated_struct.indicated_airspeed_m_s > 0 {
        raw_gps.groundSpeed = _airspeed_validated_struct.indicated_airspeed_m_s * 100;
    } else {
        raw_gps.groundSpeed = 0;
    }

    // MSP format stores coordinates in 10^6 degrees. Our internal format is radians.
    // TAU radians in 360 degrees. 1 degree = TAU/360 rad



    let raw_gps = RawGps {
        lat: ((data.gps_fix.y * TAU/360.) * GPS_SCALE_FACTOR) as i32,
        lon: ((data.gps_fix.x * TAU/360.) * GPS_SCALE_FACTOR) as i32,
        alt: data.gps_fix.z as i16,
        ..Default::default()
    };

    let packet = Packet::new(MsgType::Request, Function::RawGps, RAW_GPS_SIZE);
    msp::send_packet(uart, packet &raw_gps.to_buf());

    // todo Come back to this later.
    // if _home_position_struct.valid_hpos && _vehicle_gps_position_struct.fix_type >= 3 {
    //     let lat = _vehicle_gps_position_struct.lat / 10_000_000.0_f64;
    //     let lon = _vehicle_gps_position_struct.lon / 10000000.0_f64;
    //     let bearing_to_home = math::degrees(get_bearing_to_next_waypoint(lat, lon,
    //                                                                      _home_position_struct.lat, _home_position_struct.lon));
    //
    //     let distance_to_home = get_distance_to_next_waypoint(lat, lon,
    //                                                          _home_position_struct.lat, _home_position_struct.lon);
    //     comp_gps.distanceToHome = distance_to_home as i16; // meters
    //     comp_gps.directionToHome = bearing_to_home; // degrees
    //
    //     if bearing_to_home < 0 {
    //         bearing_to_home += 360.0;
    //     }
    //
    // } else {
    //     comp_gps.distanceToHome = 0; // meters
    //     comp_gps.directionToHome = 0; // degrees
    // }
    //
    // // MSP_COMP_GPS
    // comp_gps.heartbeat = _heartbeat;
    // _heartbeat = !_heartbeat;
    //
    // let packet = Packet::new(MsgType::Request, Function::CompGps, 100);
    // msp::send_packet(uart, packet, &comp_gps);

    // todo: radiasn to degrees fn

    let attitude = Attitude {
        roll: ((data.roll * TAU/360.) * EULER_ANGLE_SCALE_FACTOR) as i16,
        pitch: ((data.pitch * TAU/360.) * EULER_ANGLE_SCALE_FACTOR) as i16,
        yaw: ((data.yaw * TAU/360.) * EULER_ANGLE_SCALE_FACTOR) as i16,
    };

    let packet = Packet::new(MsgType::Request, Function::Attitude, ATTITUDE_SIZE);
    msp::send_packet(uart, packet, &attitude.to_buf());

    let altitude = Altitude {
        estimated_actual_position: 0, // todo
        estimated_actual_velocity: 0, // todo
        baro_latest_altitude: data.alt_msl_baro,
    };

    let packet = Packet::new(MsgType::Request, Function::Altitude, ALTITUDE_SIZE);
    msp::send_packet(uart, packet, &altitude.to_buf());

    // todo: Fill this in once you have bidir dshot setup. Could add temp as well.
    let esc_sensor_data = EscSensorData {
        motor_count: data.motor_count,
        temperature: 0,
        rpm: 0,
    };

    let packet = Packet::new(MsgType::Request, Function::EscSensorData, EC_SENSOR_DATA_SIZE);
    msp::send_packet(uart, packet, &esc_sensor_data.to_buf());

    send_config(uart);
}

/// Send initial configuration data to the display. This positions each element, and sets elements
/// unsupported by the DJI MSP API to not visible.
fn send_config(uart: &mut Uart<USART2>){
    // OSD elements positions.
    // Horizontally 2048-2074(spacing 1),
    // vertically 2048-2528(spacing 32). 26 characters X 15 lines
    // todo: Clarify how this works.
    // in betaflight configurator set OSD elements to your desired positions and in CLI type
    // "set osd" to retreieve the numbers.
    let config = OsdConfig {
        units : 0,
        item_count : 56,
        stat_count : 24,
        timer_count : 2,
        warning_count : 16,              // 16
        profile_count : 1,              // 1
        osdprofileindex : 1,                // 1
        overlay_radio_mode : 0,             //  0
        rssi_value_pos : 2_176,
        main_batt_voltage_pos : 2073,
        crosshairs_pos : NOT_VISIBLE,
        artificial_horizon_pos : NOT_VISIBLE,
        horizon_sidebars_pos : NOT_VISIBLE,
        item_timer_1_pos : NOT_VISIBLE,
        item_timer_2_pos : NOT_VISIBLE,
        flymode_pos : NOT_VISIBLE,
        craft_name_pos : 2_543,
        throttle_pos_pos : NOT_VISIBLE,
        vtx_channel_pos : NOT_VISIBLE,
        current_draw_pos : 2_103,
        mah_drawn_pos : 2_138,
        gps_speed_pos : 2_475,
        gps_sats_pos : 2_112,
        altitude_pos : 2_476,
        roll_pids_pos : NOT_VISIBLE,
        pitch_pids_pos : NOT_VISIBLE,
        yaw_pids_pos : NOT_VISIBLE,
        power_pos : NOT_VISIBLE,
        pidrate_profile_pos : NOT_VISIBLE,
        warnings_pos : NOT_VISIBLE,
        avg_cell_voltage_pos : NOT_VISIBLE,
        gps_lon_pos : 2_080,
        gps_lat_pos : 2_048,
        debug_pos : NOT_VISIBLE,
        pitch_angle_pos : NOT_VISIBLE,
        roll_angle_pos : NOT_VISIBLE,
        main_batt_usage_pos : NOT_VISIBLE,
        disarmed_pos : 2_125,
        home_dir_pos : 2_093,
        home_dist_pos : 2_331,
        numerical_heading_pos : NOT_VISIBLE,
        numerical_vario_pos : 2_482,
        compass_bar_pos : NOT_VISIBLE,
        esc_tmp_pos : NOT_VISIBLE,
        esc_rpm_pos : NOT_VISIBLE,
        remaining_time_estimate_pos : NOT_VISIBLE,
        rtc_datetime_pos : NOT_VISIBLE,
        adjustment_range_pos : NOT_VISIBLE,
        core_temperature_pos : NOT_VISIBLE,
        anti_gravity_pos : NOT_VISIBLE,
        g_force_pos : NOT_VISIBLE,
        motor_diag_pos : NOT_VISIBLE,
        log_status_pos : NOT_VISIBLE,
        flip_arrow_pos : NOT_VISIBLE,
        link_quality_pos : NOT_VISIBLE,
        flight_dist_pos : NOT_VISIBLE,
        stick_overlay_left_pos : NOT_VISIBLE,
        stick_overlay_right_pos : NOT_VISIBLE,
        display_name_pos : NOT_VISIBLE,
        esc_rpm_freq_pos : NOT_VISIBLE,
        rate_profile_name_pos : NOT_VISIBLE,
        pid_profile_name_pos : NOT_VISIBLE,
        profile_name_pos : NOT_VISIBLE,
        rssi_dbm_value_pos : NOT_VISIBLE,
        rc_channels_pos : NOT_VISIBLE,
    };

    let packet = Packet::new(MsgType::Request, Function::OsdConfig, 100);
    msp::send_packet(uart, packet, &config);
}