// #![allow(uncommon_codepoints)] // eg ϕ

//! This module contains code related to sensor fusion, eg using an extended kalman filter
//! to combine inputs from multiple sensors. It also includes code for interpreting, integrating,
//! and taking the derivatives of sensor readings. Code here is device-agnostic.
//!
//! Some IMUs can integrate with a magnetometer and do some sensor fusion; we use
//! software since it's more general, flexible, and device-agnostic.
//!
//! From HyperShield: "Unscented will not offer any improvement over EKF. The reason for this is
//! that your quadrotor will mainly stay near hover (because of the assumption that gravity
//! will be the dominant acceleration) so the system will appear quasi-linear. If you want to go
//! agile flight then UKF might offer improvements, but then you can't rely on the gravity assumption.
//! There are attitude estimators like the hua filter that circumvents this (it estimates the
//! acceleration in the inertial frame and uses that for prediction instead assuming it's equal
//! to the gravity vector)."
//!
//! Python guide: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
//!
//! [Youtube video: Phil's Lab](https://www.youtube.com/watch?v=hQUkiC5o0JI)

// todo: Try this : https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py

use cmsis_dsp_sys::{arm_cos_f32 as cos, arm_sin_f32 as sin};

use defmt::println;

use stm32_hal2::{
    dma::{Dma, DmaChannel},
    gpio::Pin,
    pac::{DMA1, SPI1},
    spi::Spi,
};

use crate::{flight_ctrls::common::Params, imu, lin_alg::Vec3, ahrs_fusion::Ahrs};

// C file with impl of EKF for quaternion rotation:
// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/EKF.h
// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/updateEKFQuatAtt.m

// Rotors: Quaternion generalization?
// https://marctenbosch.com/quaternions/

// Re quaternions, euler angles, and error state;
// Euler angles are mostly used to synthesize controllers such as PIDs.
// When it comes to state/attitude estimation, quaternions are more common.
// However you can't use a quaternion directly as a state in your EKF (technically you can with
// some care) since the the unit constraint will be violated. You can instead use an
// error-state EKF
// http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf

const G: f32 = 9.8; // m/s

// IMU readings buffer. 3 accelerometer, and 3 gyro measurements; 2 bytes each. 0-padded on the left,
// since that's where we pass the register
// in the write buffer.
pub static mut IMU_READINGS: [u8; 13] = [0; 13];

fn tan(val: f32) -> f32 {
    unsafe { sin(val) / cos(val) }
}

// todo: Consider an `imu_shared` module.
/// Represents sensor readings from a 6-axis accelerometer + gyro. Similar to
/// `ParamsInst`.
#[derive(Default)]
pub struct ImuReadings {
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,
    pub v_pitch: f32,
    pub v_roll: f32,
    pub v_yaw: f32,
}

impl ImuReadings {
    /// We use this to assemble readings from the DMA buffer.
    pub fn from_buffer(buf: &[u8]) -> Self {
        // todo: Note: this mapping may be different for diff IMUs, eg if they use a different reading register ordering.
        // todo: Currently hard-set for ICM426xx.

        // Ignore byte 0; it's for the first reg passed during the `write` transfer.
        Self {
            a_x: imu::interpret_accel(i16::from_be_bytes([buf[1], buf[2]])),
            a_y: imu::interpret_accel(i16::from_be_bytes([buf[3], buf[4]])),
            a_z: imu::interpret_accel(i16::from_be_bytes([buf[5], buf[6]])),
            // Positive pitch nose down
            v_pitch: -imu::interpret_gyro(i16::from_be_bytes([buf[7], buf[8]])),
            // Positive roll: left wing up
            v_roll: imu::interpret_gyro(i16::from_be_bytes([buf[9], buf[10]])),
            // Positive yaw: CW rotation.
            v_yaw: -imu::interpret_gyro(i16::from_be_bytes([buf[11], buf[12]])),
        }
    }
}

/// Update and get the attitude from the AHRS.
pub fn update_get_attitude(ahrs: &mut Ahrs, params: &mut Params) {
    // Gyro measurements - not really a vector.
    let mut gyro_data = Vec3 {
        x: params.v_pitch,
        y: params.v_roll,
        z: params.v_yaw,
    };
    let mut accel_data = Vec3 {
        x: params.a_x,
        y: params.a_y,
        z: params.a_z,
    };

    // Apply calibration
    // todo: Come back to this.
    // gyro_data = madgwick::apply_cal_inertial(
    //     gyro_data,
    //     ahrs.calibration.gyro_misalignment.clone(),
    //     ahrs.calibration.gyro_sensitivity,
    //     ahrs.calibration.gyro_offset,
    // );
    // accel_data = madgwick::apply_cal_inertial(
    //     accel_data,
    //     ahrs.calibration.accel_misalignment.clone(),
    //     ahrs.calibration.accel_sensitivity,
    //     ahrs.calibration.accel_offset,
    // );

    // let magnetometer = madgwick::apply_cal_magnetic(magnetometer, softIronMatrix, hardIronOffset);

    // Update gyroscope offset correction algorithm
    let gyro_data = ahrs.offset.update(gyro_data);

    // todo: Can we use the hard-set 8kHz IMU-spec DT, or do we need to measure?
    ahrs.update_no_magnetometer(gyro_data, accel_data, crate::DT_IMU);

    let att_euler = ahrs.quaternion.to_euler();
    // let att_earth = ahrs.get_earth_accel();

    // Update params with our calibrated gryo and accel data, in addition to attitude.

    params.v_roll = gyro_data.x;
    params.v_pitch = gyro_data.y;
    params.v_yaw = gyro_data.z;

    params.a_x = accel_data.x;
    params.a_y = accel_data.y;
    params.a_z = accel_data.z;

    // Note: Swapped pitch/roll swapped due to how the madgwick filter or quaternion -> euler angle
    // is calculated.
    params.s_roll = att_euler.pitch;
    params.s_pitch = att_euler.roll;
    params.s_yaw = att_euler.yaw;
    params.quaternion = ahrs.quaternion;

    // params.s_roll = att_earth.x;
    // params.s_pitch = att_earth.y;
    // params.s_yaw = att_earth.z;
}

/// Read all 3 measurements, by commanding a DMA transfer. The transfer is closed, and readings
/// are processed in the Transfer Complete ISR.
pub fn read_imu_dma(starting_addr: u8, spi: &mut Spi<SPI1>, cs: &mut Pin, dma: &mut Dma<DMA1>) {
    // First byte is the first data reg, per this IMU's. Remaining bytes are empty, while
    // the MISO line transmits readings.
    let write_buf = [starting_addr, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    cs.set_low();

    unsafe {
        spi.transfer_dma(
            &write_buf,
            &mut IMU_READINGS,
            DmaChannel::C1,
            DmaChannel::C2,
            Default::default(),
            Default::default(),
            dma,
        );
    }
}
