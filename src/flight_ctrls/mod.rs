//! This module contains code related to flight controls. Code specific to the PID
//! loops are in the `pid` module.
//!
//! [Betaflight Signal flow diagram](https://github.com/betaflight/betaflight/wiki/Signal-Flow-Diagram)
//! Note that this is just an example, and isn't necesssarily something to emulate.

pub mod autopilot;
pub mod common;
pub mod ctrl_effect_est;
pub mod ctrl_logic;
pub mod filters;
pub mod motor_servo;
pub mod pid;

use crate::{
    control_interface::ChannelData, setup::MotorTimer, state::StateVolatile, util, DT_IMU,
};

use {
    ctrl_effect_est::AccelMapPt, ctrl_logic::CtrlCoeffs, filters::FlightCtrlFilters,
    motor_servo::MotorPower,
};

use ahrs::Params;

use cfg_if::cfg_if;
use defmt::println;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        mod fixed_wing;
        pub use fixed_wing::*;
    } else {
        mod quad;
        pub use quad::*;
    }
}

// todo? move?
/// Our entry point for control logic
pub fn run(
    params: &Params,
    params_prev: &Params,
    state_volatile: &mut StateVolatile,
    control_channel_data: &Option<ChannelData>,
    ctrl_coeffs: &CtrlCoeffs,
    flight_ctrl_filters: &mut FlightCtrlFilters,
    motor_timer: &mut MotorTimer,
) {
    let throttle = match state_volatile.autopilot_commands.throttle {
        Some(t) => t,
        None => match control_channel_data {
            Some(ch_data) => ch_data.throttle,
            None => 0.,
        },
    };

    cfg_if! {
            if #[cfg(feature = "quad")] {
                let mut ctrl_mix = ctrl_logic::ctrl_mix_from_att(
                    state_volatile.attitude_commanded.quat,
                    &state_volatile.attitude_commanded.quat_dt,
                    // params.attitude_quat,
                    throttle,
                    state_volatile.motor_servo_state.frontleft_aftright_dir,
                    params,
                    params_prev,
                    ctrl_coeffs,
                    &state_volatile.drag_coeffs,
                    &state_volatile.accel_maps,
                    flight_ctrl_filters,
                    // The DT passed is the IMU rate, since we update params_prev each IMU update.
                    DT_IMU,
                );

                ctrl_mix.throttle = 0.2; // todo: Testing before we set up controller
                let power_commanded = MotorPower::from_mix(&ctrl_mix, state_volatile.motor_servo_state.frontleft_aftright_dir);

                  static mut i: u32 = 0;
                    unsafe { i += 1 };
                    if unsafe { i } % 4_000 == 0 {
                        println!("Pwr cmd: fl{:?} fr{} al{} ar{}", power_commanded.front_left, power_commanded.front_right, power_commanded.aft_left,
                    power_commanded.aft_right);
                    }

                // state_volatile.motor_servo_state.set_cmds_from_rpms(
                //     &rpms_commanded,
                //     pid_state,
                //     pid_coeffs,
                // );

                // static mut i: u32 = 0;
                // unsafe { i += 1 };
                // if unsafe { i } % 4000 == 0 {
                //     println!("\n\nMix. pitch: {} roll: {} yaw: {} throttle: {}", ctrl_mix.pitch, ctrl_mix.roll, ctrl_mix.yaw, ctrl_mix.throttle);
                //
                //     println!("Power comm. FL: {} FR: {} AL: {} AR: {}", power_commanded.front_left, power_commanded.front_right,
                //     power_commanded.aft_left, power_commanded.aft_right);
                // }

                state_volatile.ctrl_mix = ctrl_mix;

                state_volatile.motor_servo_state.set_cmds_from_power(&power_commanded);

                // This is what causes the actual change in motor speed, via DSHOT.

                // state_volatile.motor_servo_state.send_to_rotors(state_volatile.arm_status, motor_timer);
            } else {
            let ctrl_mix = ctrl_logic::ctrl_mix_from_att(
                state_volatile.attitude_commanded.quat.unwrap(),
                params.attitude_quat,
                params.attitude_quat_dt,
                throttle,
                params,
                params_prev,
                // &state_volatile.ctrl_mix,
                ctrl_coeffs,
                &state_volatile.drag_coeffs,
                &state_volatile.accel_maps,
                flight_ctrl_filters,
                DT_IMU,
            );

            let ctrl_sfc_posits = CtrlSfcPosits::from_mix(&ctrl_mix, state_volatile.motor_servo_state.frontleft_aftright_dir);
            state_volatile.ctrl_mix = ctrl_mix;

            state_volatile.motor_servo_state.set_cmds_from_control_posits(
                &ctrl_sfc_posits,
                pid_state,
                pid_coeffs,
            );

            // This is what causes the actual change in motor speed, via DSHOT.
            state_volatile.motor_servo_state.send_to_motors(ArmStatus::MotorsControlsArmed, motor_timer);

            // This is what causes the actual change in servo position, via PWM.
            state_volatile.motor_servo_state.send_to_servos(ArmStatus::MotorsControlsArmed, servo_timer);
        }
    }
}

/// Entry point for logging acceleration map points. (Mapping target angular acceleration to
/// RPM, motor power settings, or servo positions.
pub fn log_accel_pts(state_volatile: &mut StateVolatile, params: &Params, timestamp: f32) {
    // Log angular accel from RPM or servo posit delta.
    // Code-shorteners
    #[cfg(feature = "quad")]
    let ctrl_cmds = state_volatile.motor_servo_state.get_power_settings();
    #[cfg(feature = "fixed-wing")]
    let ctrl_cmds = state_volatile.motor_servo_state.get_ctrl_positions();

    state_volatile.accel_maps.log_pt(
        AccelMapPt {
            angular_accel: params.a_pitch,
            ctrl_cmd: ctrl_cmds.pitch_delta(),
            timestamp,
        },
        AccelMapPt {
            angular_accel: params.a_roll,
            ctrl_cmd: ctrl_cmds.roll_delta(),
            timestamp,
        },
        AccelMapPt {
            angular_accel: params.a_yaw,
            #[cfg(feature = "quad")]
            ctrl_cmd: ctrl_cmds.yaw_delta(state_volatile.motor_servo_state.frontleft_aftright_dir),
            #[cfg(feature = "fixed-wing")]
            ctrl_cmd: ctrl_cmds.yaw_delta(),
            timestamp,
        },
    );
}
