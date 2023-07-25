//! This module cnotains the main loop. It is likely triggered by either
//! IMU data being ready, or at a regular interval determined by a timer etc.

use core::sync::atomic::Ordering;

use stm32_hal2::dma::{self, DmaInterrupt};

use rtic::mutex_prelude::*; // todo required trait

use crate::{
    app, control_interface,
    drivers::osd::{AutopilotData, OsdData},
    flight_ctrls::{self, ctrl_logic},
    imu_shared,
    protocols::{crsf, rpm_reception, usb_preflight},
    safety::{self, ArmStatus},
    sensors_shared::{self, V_A_ADC_READ_BUF},
    setup,
    state::OperationMode,
    system_status::{self, SensorStatus},
    util,
};

use ahrs::{self, ppks::PositVelEarthUnits, ImuReadings};

use cfg_if::cfg_if;
use defmt::println;

// const UPDATE_RATE_MAIN_LOOP: f32 = 600.; // todo: Experiment with this.

const UPDATE_RATE_IMU: f32 = 8_000.;
pub const DT_IMU: f32 = 1. / UPDATE_RATE_IMU;

pub const DT_FLIGHT_CTRLS: f32 = 1. / UPDATE_RATE_FLIGHT_CTRLS;

// Every x main update loops, log parameters etc to flash.
const LOGGING_UPDATE_RATIO: u32 = 100;

// Every x IMU loops, print system status and sensor readings to console,
// if enabled with the `print-status` feature.
const PRINT_STATUS_RATIO: u32 = 16_000;

// Every x main loops, log RPM (or servo posit) to angular accel (thrust) data.
const THRUST_LOG_RATIO: u32 = 20;

#[cfg(feature = "quad")]
const FLIGHT_CTRL_IMU_RATIO: u32 = 4; // Likely values: 1, 2, 4, 8.

#[cfg(feature = "fixed-wing")]
const FLIGHT_CTRL_IMU_RATIO: u32 = 8; // Likely values: 4, 8, 16.

const NUM_IMU_LOOP_TASKS: u32 = 6; // We cycle through lower-priority tasks in the main loop.

// We run into numerical precision issues if diffing attitude commanded
// every update loop. Updating this once every few updates creates a larger difference
// in quaternion commanded, to compensate. A higher value will be more resistant
// to numerical precision problems, but may cause sluggish behavior.
pub const ATT_CMD_UPDATE_RATIO: u32 = 20;

cfg_if! {
    if #[cfg(feature = "h7")] {
        // The rate our main program updates, in Hz.
        // todo note that we will have to scale up values slightly on teh H7 board with 32.768kHz oscillator:
        // ICM-42688 DS: The ODR values shown in the
        // datasheet are supported with external clock input frequency of 32kHz. For any other external
        // clock input frequency, these ODR values will scale by a factor of (External clock value in kHz / 32).
        // For example, if an external clock frequency of 32.768kHz is used,
        // instead of ODR value of 500Hz, it will be 500 * (32.768 / 32) = 512Hz.
        const UPDATE_RATE_FLIGHT_CTRLS: f32 = 8_192. / FLIGHT_CTRL_IMU_RATIO as f32;
    } else {
        // Todo: Measured: 8.042kHz (2022-10-26)
        const UPDATE_RATE_FLIGHT_CTRLS: f32 = UPDATE_RATE_IMU / FLIGHT_CTRL_IMU_RATIO as f32;
    }
}

pub fn run(mut cx: app::imu_tc_isr::Context) {
    dma::clear_interrupt(
        setup::IMU_DMA_PERIPH,
        setup::IMU_RX_CH,
        DmaInterrupt::TransferComplete,
    );

    cx.local.cs_imu.set_high();

    cx.shared.spi1.lock(|spi1| {
        // Note that this step is mandatory, per STM32 RM.
        spi1.stop_dma(
            setup::IMU_TX_CH,
            Some(setup::IMU_RX_CH),
            setup::IMU_DMA_PERIPH,
        );
    });

    *cx.local.imu_isr_loop_i += 1;
    let i = *cx.local.imu_isr_loop_i; // code shortener.

    // todo: TS
    // if i % 2000 == 0 {
    //     let regs = unsafe { &(*pac::USART2::ptr()) };
    //     println!("UART2 SR: {:?}", regs.isr.read().bits());
    // };

    // todo: Split up this lock
    (
        cx.shared.current_params,
        cx.shared.control_channel_data,
        cx.shared.link_stats,
        cx.shared.autopilot_status,
        cx.shared.motor_pid_state,
        // cx.shared.motor_pid_coeffs,
        cx.shared.user_cfg,
        cx.shared.state_volatile,
        cx.shared.system_status,
    )
        .lock(
            |params,
             control_channel_data,
             link_stats,
             autopilot_status,
             pid_state,
             cfg,
             state_volatile,
             system_status| {
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

                // todo: We probably don't need to update AHRS each IMU update, but that's what
                // todo we're currently doing, since that's updated in `update_from_imu_readings`.
                params.update_from_imu_readings(&imu_data, None, cx.local.ahrs, DT_IMU);

                // println!("Att: {} {} {} {}", params.attitude.w, params.attitude.x, params.attitude.y, params.attitude.z);

                let mut rpm_fault = false;

                // todo: Clean up the Optionalble Status vs the non-optioned Rpms.
                // todo: Consider using only the former.

                // Update RPMs here, so we don't have to lock the read ISR.
                // cx.shared.rotor_rpms.lock(|rotor_rpms| {
                // let (rpm1_status, rpm2_status, rpm3_status, rpm4_status) = rpm_reception::update_rpms(rpms, &mut rpm_fault, cfg.pole_count);
                let rpm_readings =
                    rpm_reception::rpm_readings_from_bufs(&mut rpm_fault, cfg.motor_pole_count);

                state_volatile
                    .motor_servo_state
                    .update_rpm_readings(&rpm_readings);

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
                }

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
                    cx.shared.lost_link_timer.lock(|timer| {
                        control_interface::handle_crsf_data(
                            control_channel_data,
                            link_stats,
                            system_status,
                            timer,
                        );
                    });
                }

                // Update our commanded attitude
                match control_channel_data {
                    Some(ch_data) => {
                        if i % ATT_CMD_UPDATE_RATIO == 0 {
                            let (attitude_commanded, attitude_commanded_dt) =
                                ctrl_logic::update_att_commanded(
                                    ch_data,
                                    &cfg.input_map,
                                    state_volatile.attitude_commanded.quat,
                                    state_volatile.has_taken_off,
                                    cfg.takeoff_attitude,
                                );

                            state_volatile.attitude_commanded.quat = attitude_commanded;
                            state_volatile.attitude_commanded.quat_dt = attitude_commanded_dt;
                        }
                    }
                    None => {}
                };

                // todo: Are the attitude_commanded fields other than quat used? Should we remove them?

                // todo: Here, or in a subfunction, blend in autopiot commands! Currently not applied,
                // todo other than throttle.

                if i % FLIGHT_CTRL_IMU_RATIO == 0
                    && state_volatile.op_mode != OperationMode::Preflight
                {
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
                            );
                        },
                    );
                }

                // todo: Global const
                //
                // todo: Time these tasks so that they're roughly even.

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

                // We're tracking tasks as ones that make it past the initial flight]
                // control ratio filter, so factor that out.
                let i_compensated = i / FLIGHT_CTRL_IMU_RATIO;

                // if i_compensated % 2_000 == 0 {
                //     println!("Mix. T:{} P:{} R:{} Y:{}", state_volatile.ctrl_mix.throttle, state_volatile.ctrl_mix.pitch, state_volatile.ctrl_mix.roll, state_volatile.ctrl_mix.yaw);
                //
                //     let s = &state_volatile.motor_servo_state;
                //     println!("P. FL: {} FR: {} AL: {} AR: {}", s.rotor_front_left.power_setting ,
                //              s.rotor_front_right.power_setting, s.rotor_aft_left.power_setting, s.rotor_aft_right.power_setting);
                // }

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
                        throttle,
                    );

                    if !state_volatile.has_taken_off {
                        safety::handle_takeoff_attitude_lock(
                            throttle,
                            &mut cx.local.time_with_high_throttle,
                            &mut state_volatile.has_taken_off,
                            DT_IMU * NUM_IMU_LOOP_TASKS
                        );
                    }

                    #[cfg(feature = "quad")]
                    if let Some(ch_data) = control_channel_data {
                        flight_ctrls::set_input_mode(
                            ch_data.input_mode,
                            state_volatile,
                            system_status,
                        );
                    }
                } else if (i_compensated - 2) % NUM_IMU_LOOP_TASKS == 0 {
                    let euler = params.attitude.to_euler();

                    let osd_data = OsdData {
                        arm_status: state_volatile.arm_status,
                        battery_voltage: state_volatile.batt_v,
                        current_draw: state_volatile.esc_current,
                        alt_msl_baro: params.alt_msl_baro,
                        gps_fix: PositVelEarthUnits::default(),
                        pitch: euler.pitch,
                        roll: euler.roll,
                        yaw: euler.yaw,
                        // pid_p: coeffs.roll.k_p_rate,
                        // pid_i: coeffs.roll.k_i_rate,
                        // pid_d: coeffs.roll.k_d_rate,
                        autopilot: AutopilotData {
                            takeoff: autopilot_status.takeoff,
                            land: autopilot_status.land.is_some(),
                            direct_to_point: autopilot_status.direct_to_point.is_some(),
                            #[cfg(feature = "fixed-wing")]
                            orbit: autopilot_status.orbit.is_some(),
                            alt_hold: autopilot_status.alt_hold.is_some(),
                            #[cfg(feature = "quad")]
                            loiter: autopilot_status.loiter.is_some(),
                        },
                        base_dist_bearing: (
                            0., 0., // todo: Fill these out
                        ),
                    };

                    // todo: put back
                    // osd::send_osd_data(cx.local.uart_osd, setup::OSD_CH,&osd_data);
                } else if (i_compensated - 3) % NUM_IMU_LOOP_TASKS == 0 {
                    if state_volatile.op_mode == OperationMode::Preflight {
                        return;
                    }

                    if let Some(ch_data) = control_channel_data {
                        autopilot_status.set_modes_from_ctrls(ch_data, &params);
                    }

                    // #[cfg(feature = "quad")]
                    let ap_cmds = autopilot_status.apply(
                        params,
                        // filters,
                        // coeffs,
                        system_status,
                    );
                    //
                    // #[cfg(feature = "fixed-wing")]
                    //     let ap_cmds = autopilot_status.apply(
                    //     params,
                    //     // pid_attitude,
                    //     // filters,
                    //     // coeffs,
                    //     system_status,
                    // );

                    // Don't apply autopilot modes if on the ground.
                    if !state_volatile.has_taken_off {
                        // The intermediate variable is due to a attribute binding
                        // issue with teh direct approach.
                        state_volatile.autopilot_commands = ap_cmds;
                    }

                    if state_volatile.op_mode == OperationMode::Preflight {
                        // exit this fn during preflight *after* measuring voltages using ADCs.
                        return;
                    }
                } else if (i_compensated - 4) % NUM_IMU_LOOP_TASKS == 0 {
                    // if *cx.local.update_isr_loop_i % LOGGING_UPDATE_RATIO == 0 {
                    // todo: Eg log params to flash etc.
                    // }

                    // todo: Determine timing for OSD update, and if it should be in this loop,
                    // todo, or slower.

                    // todo: This should probably be delegatd to a fn; get it
                    // todo out here
                    if i % THRUST_LOG_RATIO == 0 {
                        let elapsed = cx
                            .shared
                            .tick_timer
                            .lock(|tick_timer| tick_timer.time_elapsed().as_secs());

                        let timestamp = util::tick_count_fm_overflows_s() + elapsed;

                        flight_ctrls::log_accel_pts(state_volatile, params, timestamp);
                    }
                } else if (i_compensated - 5) % NUM_IMU_LOOP_TASKS == 0 {
                    (cx.shared.i2c1, cx.shared.i2c2).lock(|i2c1, i2c2| {
                        // Start DMA sequences for I2C sensors, ie baro, mag, GPS, TOF.
                        // DMA TC isrs are sequenced.
                        sensors_shared::start_transfers(i2c1, i2c2);
                    });
                } else {
                    println!("No task");
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
                        );
                    }
                });
            },
        );
}
