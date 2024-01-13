//! This module contains filtering code for the IMU, including an IIR bessel lowpass, and IIR
//! notch filters for RPM filtering.
//!
//! Reference: https://brushlesswhoop.com/betaflight-rpm-filter/

use ahrs::ImuReadings;
use cmsis_dsp_api as dsp_api;

use crate::util::{iir_apply, iir_new, IirInstWrapper};

// const BLOCK_SIZE: u32 = crate::FLIGHT_CTRL_IMU_RATIO as u32;
const BLOCK_SIZE: u32 = 1;

static mut FILTER_STATE_ACCEL_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_ACCEL_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_ACCEL_Z: [f32; 4] = [0.; 4];

static mut FILTER_STATE_GYRO_PITCH: [f32; 4] = [0.; 4];
static mut FILTER_STATE_GYRO_ROLL: [f32; 4] = [0.; 4];
static mut FILTER_STATE_GYRO_YAW: [f32; 4] = [0.; 4];

static mut FILTER_STATE_VV_BARO: [f32; 4] = [0.; 4];

// todo: What cutoffs to use? I think you're in the ballpark, but maybe a little higher.
// Using 100 for acc now.
// filter_ = signal.iirfilter(1, 300, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

#[allow(clippy::excessive_precision)]
static COEFFS_LP_ACCEL: [f32; 5] = [
    0.037804754170896473,
    0.037804754170896473,
    0.0,
    0.9243904916582071,
    -0.0,
];

#[allow(clippy::excessive_precision)]
static COEFFS_LP_GYRO: [f32; 5] = [
    0.10583178270745373,
    0.10583178270745373,
    0.0,
    0.7883364345850926,
    -0.0,
];

// filter_ = signal.iirfilter(1, 30, btype="lowpass", ftype="bessel", output="sos", fs=155)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])
// Assumes updated every main loop; not IMU rate.
#[allow(clippy::excessive_precision)]
static COEFFS_VV_BARO: [f32; 5] = [
    0.3002754521653259,
    0.3002754521653259,
    0.0,
    0.3994490956693484,
    -0.0,
];

/// Store lowpass IIR filter instances, for use with lowpass and notch filters for IMU readings.
pub struct ImuFilters {
    pub accel_x: IirInstWrapper,
    pub accel_y: IirInstWrapper,
    pub accel_z: IirInstWrapper,

    pub gyro_pitch: IirInstWrapper,
    pub gyro_roll: IirInstWrapper,
    pub gyro_yaw: IirInstWrapper,

    pub vv_baro: IirInstWrapper,
    // todo: Impl these notch filters once you have bidir dshot
    // pub notch_motor1: IirInstWrapper,
    // pub notch_motor2: IirInstWrapper,
    // pub notch_motor3: IirInstWrapper,
    // pub notch_motor4: IirInstWrapper,
}

impl Default for ImuFilters {
    fn default() -> Self {
        unsafe {
            Self {
                accel_x: IirInstWrapper {
                    inner: iir_new(&COEFFS_LP_ACCEL, &mut FILTER_STATE_ACCEL_X),
                },
                accel_y: IirInstWrapper {
                    inner: iir_new(&COEFFS_LP_ACCEL, &mut FILTER_STATE_ACCEL_X),
                },
                accel_z: IirInstWrapper {
                    inner: iir_new(&COEFFS_LP_ACCEL, &mut FILTER_STATE_ACCEL_X),
                },
                gyro_pitch: IirInstWrapper {
                    inner: iir_new(&COEFFS_LP_GYRO, &mut FILTER_STATE_GYRO_PITCH),
                },
                gyro_roll: IirInstWrapper {
                    inner: iir_new(&COEFFS_LP_GYRO, &mut FILTER_STATE_GYRO_ROLL),
                },
                gyro_yaw: IirInstWrapper {
                    inner: iir_new(&COEFFS_LP_GYRO, &mut FILTER_STATE_GYRO_YAW),
                },
                vv_baro: IirInstWrapper {
                    inner: iir_new(&COEFFS_VV_BARO, &mut FILTER_STATE_VV_BARO),
                },
            }
        }
    }
}

impl ImuFilters {
    /// Apply the filters to IMU readings, modifying in place. Block size = 1.
    /// Note: Baro is handled separately.
    pub fn apply(&mut self, data: &mut ImuReadings) {
        data.a_x = filter_one(&mut self.accel_x, data.a_x);
        data.a_y = filter_one(&mut self.accel_y, data.a_y);
        data.a_z = filter_one(&mut self.accel_z, data.a_z);
        data.v_pitch = filter_one(&mut self.gyro_pitch, data.v_pitch);
        data.v_roll = filter_one(&mut self.gyro_roll, data.v_roll);
        data.v_yaw = filter_one(&mut self.gyro_yaw, data.v_yaw);
    }
}

/// Calulate the frequency to filter out, in Hz, based on one rotor's RPM.
/// `erpm` is the rpm transmitted by the ESC, aka electrical RPM, via bidirectional
/// DSHOT.
fn rpm_filter_freq(erpm: f32, num_poles: u8) -> f32 {
    let rpm = erpm / (2. * num_poles as f32); // QC this.

    // `rpm` is per minute - convert to per second.
    rpm / 60.
}
