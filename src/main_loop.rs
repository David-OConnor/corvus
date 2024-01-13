//! This module cnotains the main loop. It is likely triggered by either
//! IMU data being ready, or at a regular interval determined by a timer etc.

use core::sync::atomic::Ordering;

use ahrs::{self, ppks::PositVelEarthUnits, ImuReadings};
use lin_alg2::f32::Quaternion;
use num_traits::Float;
use rtic::mutex_prelude::*;

use crate::{
    app, controller_interface,
    drivers::osd::{AutopilotData, OsdData},
    flight_ctrls::{self, ctrl_logic, motor_servo::MotorServoState, InputMode},
    imu_shared, osd,
    protocols::{crsf, rpm_reception},
    safety::{self, ArmStatus},
    sensors_shared::{self, V_A_ADC_READ_BUF},
    state::OperationMode,
    system_status::{self, SensorStatus, SystemStatus},
    util,
};

// const UPDATE_RATE_IMU: f32 = 8_000.;
const UPDATE_RATE_IMU: f32 = 8_192.; // From measuring.
pub const DT_IMU: f32 = 1. / UPDATE_RATE_IMU;
pub const BARO_RATIO: u32 = 11;

pub const DT_FLIGHT_CTRLS: f32 = 1. / UPDATE_RATE_FLIGHT_CTRLS;

// Every x main update loops, log parameters etc to flash.
const LOGGING_UPDATE_RATIO: u32 = 100;

// Every x flight ctrl loops, print system status and sensor readings to console,
// if enabled with the `print-status` feature.
const PRINT_STATUS_RATIO: u32 = 16_000;

// Every x main loops, log RPM (or servo posit) to angular accel (thrust) data.
const THRUST_LOG_RATIO: u32 = 20;

#[cfg(feature = "quad")]
pub const FLIGHT_CTRL_IMU_RATIO: u32 = 4; // Likely values: 1, 2, 4, 8.

#[cfg(feature = "fixed-wing")]
pub const FLIGHT_CTRL_IMU_RATIO: u32 = 8; // Likely values: 4, 8, 16.

use defmt::println;

// cfg_if! {
//     if #[cfg(feature = "h7")] {
//         // The rate our main program updates, in Hz.
//         // todo note that we will have to scale up values slightly on teh H7 board with 32.768kHz oscillator:
//         // ICM-42688 DS: The ODR values shown in the
//         // datasheet are supported with external clock input frequency of 32kHz. For any other external
//         // clock input frequency, these ODR values will scale by a factor of (External clock value in kHz / 32).
//         // For example, if an external clock frequency of 32.768kHz is used,
//         // instead of ODR value of 500Hz, it will be 500 * (32.768 / 32) = 512Hz.
//         const UPDATE_RATE_FLIGHT_CTRLS: f32 = 8_192. / FLIGHT_CTRL_IMU_RATIO as f32;
//     } else {
//         // Todo: Measured: 8.042kHz (2022-10-26)
//         const UPDATE_RATE_FLIGHT_CTRLS: f32 = UPDATE_RATE_IMU / FLIGHT_CTRL_IMU_RATIO as f32;
//     }
// }

const UPDATE_RATE_FLIGHT_CTRLS: f32 = UPDATE_RATE_IMU / FLIGHT_CTRL_IMU_RATIO as f32;

pub const NUM_IMU_LOOP_TASKS: u32 = 6; // We cycle through lower-priority tasks in the main loop.

// We run into numerical precision issues if diffing attitude commanded
// every update loop. Updating this once every few updates creates a larger difference
// in quaternion commanded, to compensate. A higher value will be more resistant
// to numerical precision problems, but may cause sluggish behavior.
pub const ATT_CMD_UPDATE_RATIO: u32 = 20;

/// Used to track the duration of main loop tasks. Times are in seconds
#[derive(Default)]
pub struct TaskDurations {
    /// Runs each IMU update
    pub imu: f32,
    /// Runs each flight control update; a portion of IMU updates
    pub flight_ctrls: f32,
    /// These tasks are run, with equal frequency, as a portion of flight control updates.
    pub tasks: [f32; NUM_IMU_LOOP_TASKS as usize],
    pub main_loop_interval: f32,   // seconds
    pub flight_ctrl_interval: f32, // seconds
}

fn handle_rpm_readings(
    motor_servo_state: &mut MotorServoState,
    system_status: &mut SystemStatus,
    motor_pole_count: u8,
) {
    let mut rpm_fault = false;

    // todo: Clean up the Optionalble Status vs the non-optioned Rpms.
    // todo: Consider using only the former.

    // Update RPMs here, so we don't have to lock the read ISR.
    // cx.shared.rotor_rpms.lock(|rotor_rpms| {
    // let (rpm1_status, rpm2_status, rpm3_status, rpm4_status) = rpm_reception::update_rpms(rpms, &mut rpm_fault, cfg.pole_count);
    let rpm_readings = rpm_reception::rpm_readings_from_bufs(&mut rpm_fault, motor_pole_count);

    motor_servo_state.update_rpm_readings(&rpm_readings);

    system_status.esc_rpm = SensorStatus::Pass;

    // We currently set RPM readings status to fail if any rotor (or motor 1 for fixed-wing)
    // RPM is unavailable.
    #[cfg(feature = "quad")]
    {
        if rpm_readings.front_left.is_none() | rpm_readings.front_right.is_none()
            || rpm_readings.aft_left.is_none()
            || rpm_readings.aft_right.is_none()
        {
            system_status.esc_rpm = SensorStatus::NotConnected;
        }
    }

    #[cfg(feature = "fixed-wing")]
    {
        if rpm_readings.motor_thrust1.is_none() {
            // todo: Motor 2?
            system_status.esc_rpm = SensorStatus::NotConnected;
        }
    }

    if rpm_fault {
        system_status::RPM_FAULT.store(true, Ordering::Release);
    }
}

pub fn run(mut cx: app::imu_tc_isr::Context) {
    *cx.local.imu_isr_loop_i += 1;
    let i = *cx.local.imu_isr_loop_i; // code shortener.

    let timestamp = cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

    (
        cx.shared.params,
        cx.shared.autopilot_status,
        cx.shared.control_channel_data,
        cx.shared.link_stats,
        cx.shared.motor_pid_state,
        // cx.shared.motor_pid_coeffs,
        cx.shared.user_cfg,
        cx.shared.state_volatile,
        cx.shared.system_status,
    )
        .lock(
            |params,
             autopilot_status,
             control_channel_data,
             link_stats,
             pid_state,
             cfg,
             state_volatile,
             system_status| {
                cx.local.task_durations.main_loop_interval =
                    timestamp - system_status.update_timestamps.imu.unwrap_or(0.);
                system_status.update_timestamps.imu = Some(timestamp);

                let mut imu_data = ImuReadings::from_buffer(
                    unsafe { &imu_shared::IMU_READINGS },
                    imu_shared::ACCEL_FULLSCALE,
                    imu_shared::GYRO_FULLSCALE,
                );

                cx.shared.imu_filters.lock(|imu_filters| {
                    imu_filters.apply(&mut imu_data);
                });

                // Update `params_prev` with past-update data prior to updating params
                // todo: Update params each IMU update, or at FC interval?
                *cx.local.params_prev = params.clone();

                cx.shared.ahrs.lock(|ahrs| {
                    // todo: We probably don't need to update AHRS each IMU update, but that's what
                    // todo we're currently doing, since that's updated in `update_from_imu_readings`.
                    params.update_from_imu_readings(&imu_data, None, ahrs);
                });

                // handle_rpm_readings(
                //     &mut state_volatile.motor_servo_state,
                //     system_status,
                //     cfg.motor_pole_count,
                // );

                // todo: Impl once you've sorted out your control logic.
                // todo: Delegate this to another module, eg `attitude_ctrls`.
                // Update the target attitude based on control inputs
                // todo: Deconflict this with autopilot; probably by checking commanded
                // todo pitch, roll, yaw etc!

                // todo: You probably need to examine your types `RatesCommanded` and `CtrlInputs`.
                // todo: When are `RatesCommadned` values None?

                // todo: This whole section between here and the `ctrl_logic` calls is janky!
                // todo you need to properly blend manual controls and autopilot, and handle
                // todo lost-link procedures properly (Which may have been encoded in autopilot elsewhere)

                // Loads channel data and link stats into our shared structures,
                // from the DMA buffer.
                if !crsf::TRANSFER_IN_PROG.load(Ordering::Acquire)
                    && crsf::NEW_PACKET_RECEIVED.load(Ordering::Acquire)
                {
                    controller_interface::handle_crsf_data(
                        control_channel_data,
                        link_stats,
                        system_status,
                        timestamp,
                    );
                }

                // todo: Are the attitude_commanded fields other than quat used? Should we remove them?

                // todo: Here, or in a subfunction, blend in autopiot commands! Currently not applied,
                // todo other than throttle.

                let timestamp_imu_complete =
                    cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                cx.local.task_durations.imu = timestamp_imu_complete - timestamp;

                if i % FLIGHT_CTRL_IMU_RATIO == 0 {
                    let mut throttle = 0.;

                    // Update our commanded attitude
                    match control_channel_data {
                        Some(ch_data) => {
                            static mut I2: u32 = 0;
                            unsafe { I2 += 1 };
                            if unsafe { I2 } % ATT_CMD_UPDATE_RATIO == 0 {
                                let (attitude_commanded, attitude_commanded_dt) =
                                    match state_volatile.input_mode {
                                        InputMode::Acro => ctrl_logic::update_att_commanded_acro(
                                            ch_data,
                                            &cfg.input_map,
                                            state_volatile.attitude_commanded.quat,
                                            params.attitude,
                                            state_volatile.has_taken_off,
                                            cfg.takeoff_attitude,
                                        ),
                                        InputMode::Attitude => {
                                            ctrl_logic::update_att_commanded_att_mode(
                                                ch_data,
                                                &cfg.input_map,
                                                state_volatile.attitude_commanded.quat,
                                                params.attitude,
                                                state_volatile.has_taken_off,
                                                cfg.takeoff_attitude,
                                            )
                                        }
                                        InputMode::Loiter => {
                                            // throttle = flight_ctrls::alt_hold_throttle
                                            (Quaternion::new_identity(), (0., 0., 0.))
                                        }
                                        InputMode::Route => {
                                            // throttle = ch_data.throttle;
                                            (Quaternion::new_identity(), (0., 0., 0.))
                                        }
                                    };

                                state_volatile.attitude_commanded.quat = attitude_commanded;
                                state_volatile.attitude_commanded.quat_dt = attitude_commanded_dt;
                            }

                            match state_volatile.input_mode {
                                InputMode::Acro => {
                                    // ch_data.throttle
                                }
                                InputMode::Attitude => {
                                    // todo: Delegate to a diff fn A/R.
                                    let (alt, vv) = ctrl_logic::update_alt_baro_commanded(
                                        ch_data.throttle,
                                        &cfg.input_map,
                                        // params.v_z_baro,
                                        state_volatile.alt_baro_commanded.0,
                                    );

                                    //
                                    // flight_ctrls::throttle_from_alt_hold(
                                    //     params.attitude,
                                    //     // todo: Like attitude, change the target alt using the throttle.
                                    //     ch_data.throttle,
                                    // )

                                    state_volatile.alt_baro_commanded = (alt, vv);
                                }
                                InputMode::Loiter => {
                                    // todo: Delegate to a diff fn A/R.
                                    let (alt, vv) = ctrl_logic::update_alt_baro_commanded(
                                        ch_data.throttle,
                                        &cfg.input_map,
                                        state_volatile.alt_baro_commanded.0,
                                    );

                                    state_volatile.alt_baro_commanded = (alt, vv);
                                }
                                InputMode::Route => {}
                            }
                        }
                        None => {}
                    }

                    if state_volatile.op_mode == OperationMode::Preflight {
                        // todo: Figure out where this preflight motor-spin up code should be in this ISR.
                        // todo: Here should be fine, but maybe somewhere else is better.
                        cx.shared.motor_timer.lock(|motor_timer| {
                            if state_volatile.preflight_motors_running {
                                // todo: Use actual arm status!!
                                // println!("Motor pow fl: {:?}", state_volatile.motor_servo_state.rotor_front_left.cmd.power());

                                state_volatile
                                    .motor_servo_state
                                    .send_to_rotors(ArmStatus::Armed, motor_timer);
                            } else {
                                // todo: Does this interfere with USB reads?
                                // todo: Experiment and reason this out, if you should do this.
                                // dshot::stop_all(motor_timer);
                            }
                        });
                    } else {
                        (cx.shared.flight_ctrl_filters, cx.shared.motor_timer).lock(
                            |flight_ctrl_filters, motor_timer| {
                                flight_ctrls::run(
                                    params,
                                    cx.local.params_prev,
                                    state_volatile,
                                    control_channel_data,
                                    &cfg.ctrl_coeffs,
                                    flight_ctrl_filters,
                                    motor_timer,
                                    &cfg.input_map,
                                    &cfg.pid_coeffs,
                                    &autopilot_status,
                                    state_volatile.has_taken_off,
                                    throttle,
                                );
                            },
                        );
                    }

                    cx.local.task_durations.flight_ctrl_interval = timestamp_imu_complete
                        - system_status.update_timestamps.flight_ctrls.unwrap_or(0.);
                    system_status.update_timestamps.flight_ctrls = Some(timestamp_imu_complete);
                }

                let timestamp_fc_complete =
                    cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                // todo: Handle this being ~0 for non-FC loops?
                cx.local.task_durations.flight_ctrls =
                    timestamp_fc_complete - timestamp_imu_complete;

                // Perform various lower priority tasks like updating altimeter data etc. Space
                // these out between updates to keep loop time relatively consistent, and
                // avoid desynchronizing these tasks. This creates slots; one slot runs
                // during each main update loop. Our flow is thus like this:
                //
                //- Each IMU update, cleanup DMA, and apply filters to IMU readings.
                //- Each Every few updates (determined in IMU update ratio), perform the above
                // logic, which includes applying flight control data
                // - After applying flight control data, perform a lower priority task.
                // - Next update, apply a different lower pri task, etc.
                //  Compared to performing the tasks asynchronously, this is probably better
                // determined. Compared to performing them all together, this prevents
                // the update loop from becoming too long.

                // We're tracking tasks as ones that make it past the initial flight
                // control ratio filter, so factor that out.
                let i_compensated = i / FLIGHT_CTRL_IMU_RATIO;

                if (i_compensated - 0) % NUM_IMU_LOOP_TASKS == 0 {
                    let mut batt_v = 0.;
                    let mut curr_v = 0.;
                    batt_v = cx
                        .local
                        .batt_curr_adc
                        .reading_to_voltage(unsafe { V_A_ADC_READ_BUF }[0])
                        * sensors_shared::ADC_BATT_V_DIV;
                    curr_v = cx
                        .local
                        .batt_curr_adc
                        .reading_to_voltage(unsafe { V_A_ADC_READ_BUF }[1])
                        * sensors_shared::ADC_CURR_DIV;

                    // todo: Find the current conversion factor. Probably slope + y int
                    let esc_current = curr_v;

                    state_volatile.batt_v = batt_v;
                    state_volatile.esc_current = esc_current;

                    let timestamp_task_complete =
                        cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                    cx.local.task_durations.tasks[0] =
                        timestamp_task_complete - timestamp_fc_complete;
                } else if (i_compensated - 1) % NUM_IMU_LOOP_TASKS == 0 {
                    if state_volatile.op_mode == OperationMode::Preflight {
                        return;
                    }

                    let (controller_arm_status, throttle) = match control_channel_data {
                        Some(ch_data) => (ch_data.arm_status, ch_data.throttle),
                        None => (ArmStatus::Disarmed, 0.),
                    };

                    safety::handle_arm_status(
                        cx.local.arm_signals_received,
                        cx.local.disarm_signals_received,
                        controller_arm_status,
                        &mut state_volatile.arm_status,
                        &mut state_volatile.has_taken_off,
                        throttle,
                    );

                    let angle_from_upright =
                        params.attitude.rotate_vec(ahrs::UP).dot(ahrs::UP).acos();

                    safety::handle_takeoff_attitude_lock(
                        state_volatile.arm_status,
                        throttle,
                        &mut cx.local.time_with_high_throttle,
                        &mut cx.local.time_with_low_throttle,
                        angle_from_upright,
                        &mut state_volatile.has_taken_off,
                        DT_FLIGHT_CTRLS * NUM_IMU_LOOP_TASKS as f32,
                    );

                    #[cfg(feature = "quad")]
                    if let Some(ch_data) = control_channel_data {
                        flight_ctrls::set_input_mode(
                            ch_data.input_mode,
                            state_volatile,
                            system_status,
                        );
                    }

                    let timestamp_task_complete =
                        cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                    cx.local.task_durations.tasks[1] =
                        timestamp_task_complete - timestamp_fc_complete;
                    // For OSD, we have a larger pause between writes so as not to saturate
                    // the UART line.
                } else if (i_compensated - 2) % (NUM_IMU_LOOP_TASKS * 5) == 0 {
                    let throttle = match control_channel_data {
                        Some(ch_data) => ch_data.throttle,
                        None => 0.,
                    };

                    let osd_data = OsdData {
                        arm_status: state_volatile.arm_status,
                        battery_voltage: state_volatile.batt_v,
                        current_draw: state_volatile.esc_current,
                        alt_msl_baro: params.alt_msl_baro,
                        posit_vel: PositVelEarthUnits::default(),
                        autopilot: AutopilotData::from_status(autopilot_status),
                        base_dist_bearing: (
                            0., 0., // todo: Fill these out
                        ),
                        link_quality: link_stats.uplink_link_quality,
                        num_satellites: 0, // todo temp
                        batt_cell_count: cfg.batt_cell_count,
                        throttle,
                        total_acc: (params.a_x.powi(2) + params.a_y.powi(2) + params.a_z.powi(2))
                            .sqrt(),
                    };

                    // todo: Your blocking read here is breaking everything; use DMA.
                    cx.shared.uart_osd.lock(|uart_osd| {
                        osd::send_osd_data(uart_osd, &osd_data);
                    });

                    let timestamp_task_complete =
                        cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                    cx.local.task_durations.tasks[2] =
                        timestamp_task_complete - timestamp_fc_complete;
                } else if (i_compensated - 3) % NUM_IMU_LOOP_TASKS == 0 {
                    let mut throttle_prev = 0.;
                    if let Some(ch_data) = control_channel_data {
                        autopilot_status.set_modes_from_ctrls(ch_data, &params);
                        throttle_prev = ch_data.throttle;
                    }

                    #[cfg(feature = "quad")]
                    autopilot_status.apply(
                        &mut state_volatile.autopilot_commands,
                        params,
                        // filters,
                        // coeffs,
                        system_status,
                        throttle_prev,
                        DT_FLIGHT_CTRLS * NUM_IMU_LOOP_TASKS as f32,
                    );

                    // #[cfg(feature = "fixed-wing")]
                    //      autopilot_status.apply(
                    //    &mut state_volatile.autopilot_commands,
                    //     params,
                    //     // pid_attitude,
                    //     // filters,
                    //     // coeffs,
                    //     system_status,
                    // );

                    let timestamp_task_complete =
                        cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                    cx.local.task_durations.tasks[3] =
                        timestamp_task_complete - timestamp_fc_complete;
                } else if (i_compensated - 4) % NUM_IMU_LOOP_TASKS == 0 {
                    // if *cx.local.update_isr_loop_i % LOGGING_UPDATE_RATIO == 0 {
                    // todo: Eg log params to flash etc.
                    // }

                    // todo: Determine timing for OSD update, and if it should be in this loop,
                    // todo, or slower.

                    // todo: This should probably be delegatd to a fn; get it
                    // todo out here
                    if i % THRUST_LOG_RATIO == 0 {
                        flight_ctrls::log_accel_pts(state_volatile, params, timestamp);
                    }

                    let timestamp_task_complete =
                        cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                    cx.local.task_durations.tasks[4] =
                        timestamp_task_complete - timestamp_fc_complete;
                } else if (i_compensated - 5) % NUM_IMU_LOOP_TASKS == 0 {
                    // Don't poll the baro too fast; we get DMA anomolies and no data.
                    static mut I2: u32 = 0;
                    unsafe {
                        I2 += 1;
                        // This is a sloppy way of lowering the refresh rate. Bottom line, for quads:
                        // 8khz loop / (11 * 6(num_tasks) * 4(flight ctrl ratio) = 30Hz.
                        // This is fragile, ie if we change any of the above params.
                        // The baro refreshes at 32Hz.
                        if I2 % BARO_RATIO == 0 {
                            cx.shared.i2c2.lock(|i2c2| {
                                sensors_shared::start_transfer_baro(i2c2);
                            });
                        }
                    }

                    system_status.update_from_timestamp(timestamp);

                    // This isn't part of `update_from_timestamps` due to the params
                    // in `execute_lost_link`.
                    match system_status.update_timestamps.rf_control_link {
                        Some(t) => {
                            if timestamp - t > system_status::MAX_UPDATE_PERIOD_RC_LINK {
                                system_status.rf_control_link = SensorStatus::NotConnected;

                                if state_volatile.has_taken_off {
                                    safety::excecute_link_lost(
                                        system_status,
                                        autopilot_status,
                                        params,
                                        &cfg.base_pt,
                                    );
                                }
                            }
                        }
                        None => {
                            system_status.rf_control_link = SensorStatus::NotConnected;
                        }
                    }

                    let timestamp_task_complete =
                        cx.shared.tick_timer.lock(|timer| timer.get_timestamp());

                    cx.local.task_durations.tasks[5] =
                        timestamp_task_complete - timestamp_fc_complete;
                }

                cx.shared.tick_timer.lock(|tick_timer| {
                    #[cfg(feature = "print-status")]
                    if i % PRINT_STATUS_RATIO == 0 {
                        util::print_status(
                            params,
                            system_status,
                            control_channel_data,
                            state_volatile,
                            autopilot_status,
                            tick_timer,
                            &cx.local.task_durations,
                        );
                    }
                });
            },
        )
}
