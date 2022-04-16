//! This module contains filtering code for the IMU, including an IIR bessel lowpass, and IIR
//! notch filters for RPM filtering.
//!
//! Reference: https://brushlesswhoop.com/betaflight-rpm-filter/

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as dsp_sys;

use crate::flight_ctrls::IirInstWrapper;

static mut FILTER_STATE_ACCEL_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_ACCEL_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_ACCEL_Z: [f32; 4] = [0.; 4];

static mut FILTER_STATE_GYRO_PITCH: [f32; 4] = [0.; 4];
static mut FILTER_STATE_GYRO_ROLL: [f32; 4] = [0.; 4];
static mut FILTER_STATE_GYRO_YAW: [f32; 4] = [0.; 4];

// todo: Do you need anti-aliasing?

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

impl ImuFilters {
    pub fn new() -> Self {
        // todo: What cutoffs to use?
        // filter_ = signal.iirfilter(1, 400, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
        // coeffs = []
        // for row in filter_:
        //     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

        let coeffs_accel = [
            0.13672873599731955,
            0.13672873599731955,
            0.0,
            0.726542528005361,
            -0.0,
        ];

        // filter_ = signal.iirfilter(1, 400, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
        let coeffs_gyro = [
            0.13672873599731955,
            0.13672873599731955,
            0.0,
            0.726542528005361,
            -0.0,
        ];

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
                &coeffs_accel,
                &mut FILTER_STATE_ACCEL_X,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.accel_y.inner,
                &coeffs_accel,
                &mut FILTER_STATE_ACCEL_Y,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.accel_z.inner,
                &coeffs_accel,
                &mut FILTER_STATE_ACCEL_Z,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.gyro_pitch.inner,
                &coeffs_gyro,
                &mut FILTER_STATE_GYRO_PITCH,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.gyro_roll.inner,
                &coeffs_gyro,
                &mut FILTER_STATE_GYRO_ROLL,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.gyro_yaw.inner,
                &coeffs_gyro,
                &mut FILTER_STATE_GYRO_YAW,
            );
        }

        result
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
