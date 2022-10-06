//! THis module contains code for digitally filtering data involved in flight controls.

use cmsis_dsp_api as dsp_api;

use crate::util::IirInstWrapper;

static mut FILTER_STATE_CTRL_EFFECTIVENESS: [f32; 4] = [0.; 4];

// filter_ = signal.iirfilter(1, 80, btype="lowpass", ftype="bessel", output="sos", fs=6_666)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])

#[allow(clippy::excessive_precision)]
static COEFFS_CTRL_EFFECTIVENESS: [f32; 5] = [
    0.03634962200119309,
    0.03634962200119309,
    0.0,
    0.9273007559976137,
    -0.0,
];

/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop. Note that we don't
/// need this for our horizontal velocity PIDs.
pub struct FlightCtrlFilters {
    /// This applies a lowpass filter to mapping of motor power to effec
    /// todo: RPM instead of motor power once you have bidir dshot working.
    pub ctrl_effectiveness: IirInstWrapper,
}

impl Default for FlightCtrlFilters {
    fn default() -> Self {
        let mut result = Self {
            ctrl_effectiveness: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            rpm: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            // todo: Re-initialize fn?
            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.ctrl_effectiveness.inner,
                &COEFFS_CTRL_EFFECTIVENESS,
                &mut FILTER_STATE_CTRL_EFFECTIVENESS,
            );
        }

        result
    }
}

impl FlightCtrlFilters {
    /// Apply the filters to IMU readings, modifying in place. Block size = 1.
    // pub fn apply(&mut self, data: &mut ImuReadings) {
    pub fn apply(&mut self, ctrl_effectiveness_raw: f32) -> f32 {
        let block_size = 1;

        let mut ctrl_effectiveness = [0.];

        dsp_api::biquad_cascade_df1_f32(
            &mut self.ctrl_effectiveness.inner,
            &[ctrl_effectiveness_raw],
            &mut ctrl_effectiveness,
            block_size,
        );

        ctrl_effectiveness[0]
        // data.a_x = a_x[0];
    }
}
