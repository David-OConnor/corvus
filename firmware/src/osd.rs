//! This module contains code for interfacing with DJI's OSD system, via UART.
//! It uses the MSP protocol.
//! Note that this isn't ideal in that it doesn't support "canvas", or pixel buffers.
//! Worry about that later as the ecosystem changes.
//!
//! https://discuss.ardupilot.org/t/msp-displayport/73155/33
//!
//! Note: MUst send arm MSP signal for DJI to use high power mode!
//!
//! Based on DeathByHotGlue's library. Should be full up!
//! https://github.com/chris1seto/PX4-Autopilot/tree/turbotimber/src/modules/msp_osd

use defmt::{println, warn, info};

use crate::prtocols::msp::{self};

//  *
//  *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

struct OsdError {
    pub message: &'static str,
}

// struct MspOsd : public ModuleBase<MspOsd>, public ModuleParams, public px4::ScheduledWorkItem {
struct MspOsd {
    _msp: MspV1,
// int _msp_fd{-1};

    is_initialized: bool,

// struct battery_status_s _battery_status_struct = {0};
// struct vehicle_status_s _vehicle_status_struct;
// struct vehicle_gps_position_s _vehicle_gps_position_struct = {0};
// struct airspeed_validated_s _airspeed_validated_struct = {0};
// struct vehicle_air_data_s _vehicle_air_data_struct = {0};
// struct home_position_s _home_position_struct = {0};
// struct vehicle_global_position_s _vehicle_global_position_struct = {0};
// struct vehicle_attitude_s _vehicle_attitude_struct = {0};
// struct estimator_status_s _estimator_status_struct = {0};
// struct vehicle_local_position_s _vehicle_local_position_struct = {0};
// struct input_rc_s _input_rc_struct = {0};

    _heartbeat: bool,

// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

}

//OSD elements positions
//in betaflight configurator set OSD elements to your desired positions and in CLI type "set osd" to retreieve the numbers.
//234 -> not visible. Horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines

// Currently working elements

// Left
const osd_gps_lat_pos: u16 = 2048;
const osd_gps_lon_pos: u16 = 2080;
static mut osd_gps_sats_pos: u16 = 2112;
const osd_rssi_value_pos: u16 = 2176;
const osd_flymode_pos: u16 = 234;
const osd_esc_tmp_pos: u16 = 234;

// Center
const osd_home_dir_pos: u16 = 2093;
const osd_craft_name_pos: u16 = 2543;
const osd_horizon_sidebars_pos: u16 = 234;
const osd_disarmed_pos: u16 = 2125;

// Right
const osd_main_batt_voltage_pos: u16 = 2073;
const osd_current_draw_pos: u16 = 2103;
const osd_mah_drawn_pos: u16 = 2138;
const osd_altitude_pos: u16 = 2476;
const osd_numerical_vario_pos: u16 = 2482;
const osd_gps_speed_pos: u16 = 2475;
const osd_home_dist_pos: u16 = 2331;
const osd_power_pos: u16 = 234;

// Disabled
const osd_pitch_angle_pos: u16 = 234;
const osd_roll_angle_pos: u16 = 234;
const osd_crosshairs_pos: u16 = 234;
const osd_avg_cell_voltage_pos: u16 = 234;

// Not implemented or not available
const osd_throttle_pos_pos: u16 = 234;
const osd_vtx_channel_pos: u16 = 234;
const osd_roll_pids_pos: u16 = 234;
const osd_pitch_pids_pos: u16 = 234;
const osd_yaw_pids_pos: u16 = 234;
const osd_pidrate_profile_pos: u16 = 234;
const osd_warnings_pos: u16 = 234;
const osd_debug_pos: u16 = 234;
const osd_artificial_horizon_pos: u16 = 234;
const osd_item_timer_1_pos: u16 = 234;
const osd_item_timer_2_pos: u16 = 234;
const osd_main_batt_usage_pos: u16 = 234;
const osd_numerical_heading_pos: u16 = 234;
const osd_compass_bar_pos: u16 = 234;
const osd_esc_rpm_pos: u16 = 234;
const osd_remaining_time_estimate_pos: u16 = 234;
const osd_rtc_datetime_pos: u16 = 234;
const osd_adjustment_range_pos: u16 = 234;
const osd_core_temperature_pos: u16 = 234;
const osd_anti_gravity_pos: u16 = 234;
const osd_g_force_pos: u16 = 234;
const osd_motor_diag_pos: u16 = 234;
const osd_log_status_pos: u16 = 234;
const osd_flip_arrow_pos: u16 = 234;
const osd_link_quality_pos: u16 = 234;
const osd_flight_dist_pos: u16 = 234;
const osd_stick_overlay_left_pos: u16 = 234;
const osd_stick_overlay_right_pos: u16 = 234;
const osd_display_name_pos: u16 = 234;
const osd_esc_rpm_freq_pos: u16 = 234;
const osd_rate_profile_name_pos: u16 = 234;
const osd_pid_profile_name_pos: u16 = 234;
const osd_profile_name_pos: u16 = 234;
const osd_rssi_dbm_value_pos: u16 = 234;
const osd_rc_channels_pos: u16 = 234;

impl MpOsd {
    pub fn new() {
    // ModuleParams(nullptr),
    // ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
    // _msp(NULL)
    }

    MspOsd::~MspOsd()
    {
    }

    fn init() -> bool {
        ScheduleOnInterval(100_ms);

        true
    }

    fn SendConfig(){
        let msp_osd_config: MspOsdConfig = Default::default();

        msp_osd_config.units = 0;

        msp_osd_config.osd_item_count = 56;
        msp_osd_config.osd_stat_count = 24;
        msp_osd_config.osd_timer_count = 2;
        msp_osd_config.osd_warning_count = 16;              // 16
        msp_osd_config.osd_profile_count = 1;              // 1
        msp_osd_config.osdprofileindex = 1;                // 1
        msp_osd_config.overlay_radio_mode = 0;             //  0

        msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
        msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
        msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
        msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
        msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
        msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
        msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
        msp_osd_config.osd_flymode_pos = osd_flymode_pos;
        msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
        msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
        msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
        msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
        msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
        msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
        msp_osd_config.osd_gps_sats_pos = unsafe { osd_gps_sats_pos };
        msp_osd_config.osd_altitude_pos = osd_altitude_pos;
        msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
        msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
        msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
        msp_osd_config.osd_power_pos = osd_power_pos;
        msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
        msp_osd_config.osd_warnings_pos = osd_warnings_pos;
        msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
        msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
        msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
        msp_osd_config.osd_debug_pos = osd_debug_pos;
        msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
        msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
        msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
        msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
        msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
        msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
        msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
        msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
        msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
        msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
        msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
        msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
        msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
        msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
        msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
        msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
        msp_osd_config.osd_g_force_pos = osd_g_force_pos;
        msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
        msp_osd_config.osd_log_status_pos = osd_log_status_pos;
        msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
        msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
        msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
        msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
        msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
        msp_osd_config.osd_display_name_pos = osd_display_name_pos;
        msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
        msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
        msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
        msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
        msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
        msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

        _msp.Send(MSP_OSD_CONFIG, &msp_osd_config);
    }

    fn Run(&self){
        if should_exit() {
            ScheduleClear();
            exit_and_cleanup();
            return;
        }

        // Check if parameters have changed
        if _parameter_update_sub.updated() {
            // clear update
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);
            updateParams(); // update module parameters (in DEFINE_PARAMETERS)
        }

        if !self._is_initialized {
            _is_initialized = true;

            struct termios t;

            _msp_fd = open("/dev/ttyS3", O_RDWR | O_NONBLOCK);

            if _msp_fd < 0 {
                return;
            }

            tcgetattr(_msp_fd, &t);
            cfsetspeed(&t, B115200);
            t.c_cflag &= !(CSTOPB | PARENB | CRTSCTS);
            t.c_lflag &= !(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
            t.c_iflag &= !(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
            t.c_oflag = 0;
            tcsetattr(_msp_fd, TCSANOW, &t);

            _msp = MspV1(_msp_fd);

            warn!("Startup");
        }

        let battery_state: MspBatteryState = {0};
        let name: MspName = {0};
        let statusBf: MspStatusBf = {0};
        let analog: MspAnalog = {0};
        let raw_gps: MspRawGps = {0};
        let comp_gps: MspCompGps = {0};
        let attitude: MspAttitude = {0};
        let altitude: MspAltitude = {0};
        let esc_sensor_data: MspEscSensorDataDji = {0};
        let variant: MspFcVariant = {0};

        _battery_status_sub.copy(&_battery_status_struct);
        _vehicle_status_sub.copy(&_vehicle_status_struct);
        _vehicle_gps_position_sub.copy(&_vehicle_gps_position_struct);
        _airspeed_validated_sub.copy(&_airspeed_validated_struct);
        _vehicle_air_data_sub.copy(&_vehicle_air_data_struct);
        _home_position_sub.copy(&_home_position_struct);
        _vehicle_global_position_sub.copy(&_vehicle_global_position_struct);
        _vehicle_local_position_sub.copy(&_vehicle_local_position_struct);
        _vehicle_attitude_sub.copy(&_vehicle_attitude_struct);
        _estimator_status_sub.copy(&_estimator_status_struct);
        _input_rc_sub.copy(&_input_rc_struct);

        matrix::Eulerf euler_attitude(matrix::Quatf(_vehicle_attitude_struct.q));

        memcpy(variant.flightControlIdentifier, "BTFL", sizeof(variant.flightControlIdentifier));
        _msp.Send(MSP_FC_VARIANT, &variant);

        // MSP_NAME
        snprintf(name.craft_name, sizeof(name.craft_name), " ");
        name.craft_name[14] = '\0';
        _msp.Send(MSP_NAME, &name);

        // MSP_STATUS
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
        _msp.Send(MSP_STATUS, &status_BF);

        // MSP_ANALOG
        analog.vbat = (_battery_status_struct.voltage_v * 10.0) / _battery_status_struct.cell_count; // bottom right... v * 10
        analog.rssi = ((_input_rc_struct.link_quality * 1024.0) / 100.0) as u16;
        analog.amperage = _battery_status_struct.current_a * 100; // main amperage
        analog.mAhDrawn = _battery_status_struct.discharged_mah; // unused
        _msp.Send(MSP_ANALOG, &analog);

        // MSP_BATTERY_STATE
        battery_state.amperage = _battery_status_struct.current_a; // not used?
        battery_state.batteryVoltage = (_battery_status_struct.voltage_v * 400.0) as u16; // OK
        battery_state.mAhDrawn = _battery_status_struct.discharged_mah ; // OK
        battery_state.batteryCellCount = _battery_status_struct.cell_count;
        battery_state.batteryCapacity = _battery_status_struct.capacity; // not used?

        // Voltage color 0==white, 1==red
        if _battery_status_struct.voltage_v < 14.4 {
            battery_state.batteryState = 1;
        } else {
            battery_state.batteryState = 0;
        }
        battery_state.legacyBatteryVoltage = _battery_status_struct.voltage_v * 10;
        _msp.Send(MSP_BATTERY_STATE, &battery_state);

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
            _msp.Send(MSP_RAW_GPS, &raw_gps);

        if _home_position_struct.valid_hpos && _vehicle_gps_position_struct.fix_type >= 3 {
        let lat = _vehicle_gps_position_struct.lat / 10_000_000.0_f64;
        let lon = _vehicle_gps_position_struct.lon / 10000000.0_f64;
        let bearing_to_home = math::degrees(get_bearing_to_next_waypoint(lat, lon,
        _home_position_struct.lat, _home_position_struct.lon));

        let distance_to_home = get_distance_to_next_waypoint(lat, lon,
        _home_position_struct.lat, _home_position_struct.lon);
        comp_gps.distanceToHome = distance_to_home as i16; // meters
        comp_gps.directionToHome = bearing_to_home; // degrees
        if bearing_to_home < 0 {
            bearing_to_home += 360.0;
        }
        //printf("%f %f %f %f %f\r\n", lat, lon, (double)distance_to_home, (double)bearing_to_home, (double)math::degrees(euler_attitude.psi()));
        } else {
            comp_gps.distanceToHome = 0; // meters
            comp_gps.directionToHome = 0; // degrees
        }

        // MSP_COMP_GPS
        comp_gps.heartbeat = _heartbeat;
        _heartbeat = !_heartbeat;
        _msp.Send(MSP_COMP_GPS, &comp_gps);

        // MSP_ATTITUDE
        attitude.pitch = math::degrees(euler_attitude.theta()) * 10;
        attitude.roll = math::degrees(euler_attitude.phi()) * 10;
        //attitude.yaw = math::degrees(euler_attitude.psi()) * 10;

        _msp.Send(MSP_ATTITUDE, &attitude);

        // MSP_ALTITUDE
        if _estimator_status_struct.solution_status_flags & (1 << 5) {
            altitude.estimatedActualVelocity = -_vehicle_local_position_struct.vz * 10.0; //m/s to cm/s
        } else {
            altitude.estimatedActualVelocity = 0;
        }

        _msp.Send(MSP_ALTITUDE, &altitude);

        // MSP_MOTOR_TELEMETRY
        esc_sensor_data.rpm = 0;
        esc_sensor_data.temperature = 50;
        _msp.Send(MSP_ESC_SENSOR_DATA, &esc_sensor_data);

        SendConfig();
    }

    fn task_spawn(&self, argc: usize, argv: &[u8]) -> Result<usize, OsdError> {
        let instance = MspOsd::new();

        if instance {
            _object.store(instance);
            _task_id = task_id_is_work_queue;

            if self.init() {
                return PX4_OK;
            }
        } else {
            return Err(OsdError::new("alloc failed"));
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
    }

    fn print_status(&self) -> usize {
        info!("Running");
        0
    }

    fn custom_command(&self, argc: usize, argv: &[u8]) -> usize {
        0
    }

    fn print_usage(reason: u8) -> usize {
        if reason {
            warn!("{}\n", reason);
        }

    //     PRINT_MODULE_DESCRIPTION(
    //     R"DESCR_STR(
    // ### Description
    // Msp OSD!
    // ### Implementation
    // Does the things for the DJI Air Unit OSD
    // ### Examples
    // CLI usage example:
    // $ msp_osd
    // )DESCR_STR");

        // PRINT_MODULE_USAGE_NAME("module", "msp_osd");
        // PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        0
    }
}



fn msp_osd_main(argc: usize, argv: &[u8]) -> usize {
	MspOsd::main(argc, argv)
}

