//! This module contains filtering code for the IMU, including an IIR bessel lowpass, and IIR
//! notch filters for RPM filtering.
//!
//! Reference: https://brushlesswhoop.com/betaflight-rpm-filter/

use cmsis_dsp_api as dsp_api;

use crate::{sensor_fusion::ImuReadings, util::IirInstWrapper};

static mut FILTER_STATE_ACCEL_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_ACCEL_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_ACCEL_Z: [f32; 4] = [0.; 4];

static mut FILTER_STATE_GYRO_PITCH: [f32; 4] = [0.; 4];
static mut FILTER_STATE_GYRO_ROLL: [f32; 4] = [0.; 4];
static mut FILTER_STATE_GYRO_YAW: [f32; 4] = [0.; 4];

// todo: What cutoffs to use? I think you're in the ballpark, but maybe a little higher.
// filter_ = signal.iirfilter(1, 40, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

static COEFFS_LP_ACCEL: [f32; 5] = [
    0.015466291403103363,
    0.015466291403103363,
    0.0,
    0.9690674171937933,
    -0.0,
];

// filter_ = signal.iirfilter(1, 400, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
static COEFFS_LP_GYRO: [f32; 5] = [
    0.015466291403103363,
    0.015466291403103363,
    0.0,
    0.9690674171937933,
    -0.0,
];

// todo: Calibration for IMU: Hardware, software, or both?

/// Store lowpass IIR filter instances, for use with lowpass and notch filters for IMU readings.
pub struct ImuFilters {
    pub accel_x: IirInstWrapper,
    pub accel_y: IirInstWrapper,
    pub accel_z: IirInstWrapper,

    pub gyro_pitch: IirInstWrapper,
    pub gyro_roll: IirInstWrapper,
    pub gyro_yaw: IirInstWrapper,
    // todo: Impl these notch filters once you have bidir dshot
    // pub notch_motor1: IirInstWrapper,
    // pub notch_motor2: IirInstWrapper,
    // pub notch_motor3: IirInstWrapper,
    // pub notch_motor4: IirInstWrapper,
}

impl Default for ImuFilters {
    fn default() -> Self {
        let mut result = Self {
            accel_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            accel_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            accel_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            gyro_pitch: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            gyro_roll: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            gyro_yaw: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            // todo: Re-initialize fn?
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.accel_x.inner,
                &COEFFS_LP_ACCEL,
                &mut FILTER_STATE_ACCEL_X,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.accel_y.inner,
                &COEFFS_LP_ACCEL,
                &mut FILTER_STATE_ACCEL_Y,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.accel_z.inner,
                &COEFFS_LP_ACCEL,
                &mut FILTER_STATE_ACCEL_Z,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.gyro_pitch.inner,
                &COEFFS_LP_GYRO,
                &mut FILTER_STATE_GYRO_PITCH,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.gyro_roll.inner,
                &COEFFS_LP_GYRO,
                &mut FILTER_STATE_GYRO_ROLL,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.gyro_yaw.inner,
                &COEFFS_LP_GYRO,
                &mut FILTER_STATE_GYRO_YAW,
            );
        }

        result
    }
}
impl ImuFilters {
    /// Apply the filters to IMU readings, modifying in place. Block size = 1.
    pub fn apply(&mut self, data: &mut ImuReadings) {
        let block_size = 1;

        let mut a_x = [0.];
        let mut a_y = [0.];
        let mut a_z = [0.];
        let mut v_pitch = [0.];
        let mut v_roll = [0.];
        let mut v_yaw = [0.];

        unsafe {
            dsp_api::biquad_cascade_df1_f32(
                &mut self.accel_x.inner,
                &[data.a_x],
                &mut a_x,
                block_size,
            );
            dsp_api::biquad_cascade_df1_f32(
                &mut self.accel_y.inner,
                &[data.a_y],
                &mut a_y,
                block_size,
            );
            dsp_api::biquad_cascade_df1_f32(
                &mut self.accel_z.inner,
                &[data.a_z],
                &mut a_z,
                block_size,
            );
            dsp_api::biquad_cascade_df1_f32(
                &mut self.gyro_pitch.inner,
                &[data.v_pitch],
                &mut v_pitch,
                block_size,
            );
            dsp_api::biquad_cascade_df1_f32(
                &mut self.gyro_roll.inner,
                &[data.v_roll],
                &mut v_roll,
                block_size,
            );
            dsp_api::biquad_cascade_df1_f32(
                &mut self.gyro_yaw.inner,
                &[data.v_yaw],
                &mut v_yaw,
                block_size,
            );
        }

        data.a_x = a_x[0];
        data.a_y = a_y[0];
        data.a_z = a_z[0];
        data.v_pitch = v_pitch[0];
        data.v_roll = v_roll[0];
        data.v_yaw = v_yaw[0];
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
