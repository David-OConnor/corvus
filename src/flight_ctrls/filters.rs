//! THis module contains code for digitally filtering data involved in flight controls.

use cmsis_dsp_api as dsp_api;

// These filter states are for the PID D term.
static mut FILTER_STATE_ROLL_ATTITUDE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_PITCH_ATTITUDE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_YAW_ATTITUDE: [f32; 4] = [0.; 4];

static mut FILTER_STATE_ROLL_RATE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_PITCH_RATE: [f32; 4] = [0.; 4];
static mut FILTER_STATE_YAW_RATE: [f32; 4] = [0.; 4];

static mut FILTER_STATE_THRUST: [f32; 4] = [0.; 4];

// filter_ = signal.iirfilter(1, 100, btype="lowpass", ftype="bessel", output="sos", fs=8_000)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

// todo: Diff coeffs for diff diff parts, as required.
#[allow(clippy::excessive_precision)]
static COEFFS_D: [f32; 5] = [
    0.037804754170896473,
    0.037804754170896473,
    0.0,
    0.9243904916582071,
    -0.0,
];


/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop. Note that we don't
/// need this for our horizontal velocity PIDs.
pub struct PidDerivFilters {
    pub roll_attitude: IirInstWrapper,
    pub pitch_attitude: IirInstWrapper,
    pub yaw_attitude: IirInstWrapper,

    pub roll_rate: IirInstWrapper,
    pub pitch_rate: IirInstWrapper,
    pub yaw_rate: IirInstWrapper,

    pub thrust: IirInstWrapper, // todo - do we need this?
}

impl Default for PidDerivFilters {
    fn default() -> Self {
        let mut result = Self {
            roll_attitude: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            pitch_attitude: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            yaw_attitude: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            roll_rate: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            pitch_rate: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            yaw_rate: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },

            thrust: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            // todo: Re-initialize fn?
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.roll_attitude.inner,
                &COEFFS_D,
                &mut FILTER_STATE_ROLL_ATTITUDE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.pitch_attitude.inner,
                &COEFFS_D,
                &mut FILTER_STATE_PITCH_ATTITUDE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.yaw_attitude.inner,
                &COEFFS_D,
                &mut FILTER_STATE_YAW_ATTITUDE,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.roll_rate.inner,
                &COEFFS_D,
                &mut FILTER_STATE_ROLL_RATE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.pitch_rate.inner,
                &COEFFS_D,
                &mut FILTER_STATE_PITCH_RATE,
            );
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.yaw_rate.inner,
                &COEFFS_D,
                &mut FILTER_STATE_YAW_RATE,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.thrust.inner,
                &COEFFS_D,
                &mut FILTER_STATE_THRUST,
            );
        }

        result
    }
}