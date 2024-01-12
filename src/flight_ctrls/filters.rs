//! THis module contains code for digitally filtering data involved in flight controls.

use cmsis_dsp_api as dsp_api;

use crate::util::IirInstWrapper;

// static mut FILTER_STATE_CTRL_EFFECTIVENESS: [f32; 4] = [0.; 4];
static mut FILTER_STATE_DRAG_COEFF_PITCH: [f32; 4] = [0.; 4];
static mut FILTER_STATE_DRAG_COEFF_ROLL: [f32; 4] = [0.; 4];
static mut FILTER_STATE_DRAG_COEFF_YAW: [f32; 4] = [0.; 4];

static mut FILTER_STATE_D_TERM_X: [f32; 4] = [0.; 4];
static mut FILTER_STATE_D_TERM_Y: [f32; 4] = [0.; 4];
static mut FILTER_STATE_D_TERM_Z: [f32; 4] = [0.; 4];

#[allow(clippy::excessive_precision)]
static COEFFS_CTRL_EFFECTIVENESS: [f32; 5] = [
    0.13672873599731955,
    0.13672873599731955,
    0.0,
    0.726542528005361,
    -0.0,
];


// filter_ = signal.iirfilter(1, 100, btype="lowpass", ftype="bessel", output="sos", fs=2_048)
// coeffs = []
// for row in filter_:
//     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])
// todo: Experiment here with diff frequencies.
// Assumes updated every main loop; not IMU rate.
#[allow(clippy::excessive_precision)]
#[cfg(feature = "quad")]
static COEFFS_D_TERM: [f32; 5] = [
    0.13390872336157789,
    0.13390872336157789,
    0.0,
    0.7321825532768441,
    -0.0,
];

#[allow(clippy::excessive_precision)]
#[cfg(feature = "fixed-wing")]
static COEFFS_D_TERM: [f32; 5] = [
    0.24058238255001216,
    0.24058238255001216,
    0.0,
    0.5188352348999759,
    -0.0,
];

/// Store lowpass IIR filter instances, for use with the deriv terms of our PID loop. Note that we don't
/// need this for our horizontal velocity PIDs.
pub struct FlightCtrlFilters {
    /// This applies a lowpass filter to mapping of motor power to effec
    /// todo: RPM instead of motor power once you have bidir dshot working.
    // pub ctrl_effectiveness: IirInstWrapper,
    // todo: CUrrently you don't use `ctrl_effectiveness`, and you should split drag coeff by axis!
    // todo ie sep for pitch, roll, yaw
    // pub drag_coeff_pitch: IirInstWrapper,
    // pub drag_coeff_roll: IirInstWrapper,
    // pub drag_coeff_yaw: IirInstWrapper,
    pub d_term_x: IirInstWrapper,
    pub d_term_y: IirInstWrapper,
    pub d_term_z: IirInstWrapper,
}

impl Default for FlightCtrlFilters {
    fn default() -> Self {
        let mut result = Self {
            // ctrl_effectiveness: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // drag_coeff_pitch: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // drag_coeff_roll: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            // drag_coeff_yaw: IirInstWrapper {
            //     inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            // },
            d_term_x: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            d_term_y: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
            d_term_z: IirInstWrapper {
                inner: dsp_api::biquad_cascade_df1_init_empty_f32(),
            },
        };

        unsafe {
            // dsp_api::biquad_cascade_df1_init_f32(
            //     &mut result.ctrl_effectiveness.inner,
            //     &COEFFS_CTRL_EFFECTIVENESS,
            //     &mut FILTER_STATE_CTRL_EFFECTIVENESS,
            // );

            // dsp_api::biquad_cascade_df1_init_f32(
            //     &mut result.drag_coeff_pitch.inner,
            //     &COEFFS_DRAG_COEFF,
            //     &mut FILTER_STATE_DRAG_COEFF_PITCH,
            // );
            //
            // dsp_api::biquad_cascade_df1_init_f32(
            //     &mut result.drag_coeff_roll.inner,
            //     &COEFFS_DRAG_COEFF,
            //     &mut FILTER_STATE_DRAG_COEFF_ROLL,
            // );
            //
            // dsp_api::biquad_cascade_df1_init_f32(
            //     &mut result.drag_coeff_yaw.inner,
            //     &COEFFS_DRAG_COEFF,
            //     &mut FILTER_STATE_DRAG_COEFF_YAW,
            // );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.d_term_x.inner,
                &COEFFS_D_TERM,
                &mut FILTER_STATE_D_TERM_X,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.d_term_y.inner,
                &COEFFS_D_TERM,
                &mut FILTER_STATE_D_TERM_Y,
            );

            dsp_api::biquad_cascade_df1_init_f32(
                &mut result.d_term_z.inner,
                &COEFFS_D_TERM,
                &mut FILTER_STATE_D_TERM_Z,
            );
        }

        result
    }
}

impl FlightCtrlFilters {
    /// Apply the filters. Run this each main loop.
    // pub fn apply(&mut self, ctrl_effectiveness_raw: f32, drag_coeff_raw) -> (f32, f32) {
    pub fn apply(
        &mut self,
        // drag_coeff_pitch: f32,
        // drag_coeff_roll: f32,
        // drag_coeff_yaw: f32,
        d_term_x: f32,
        d_term_y: f32,
        d_term_z: f32,
    ) -> (f32, f32, f32) {
        // todo: Larger block size?
        let block_size = 1;

        // let mut ctrl_effectiveness = [0.];
        //
        // dsp_api::biquad_cascade_df1_f32(
        //     &mut self.ctrl_effectiveness.inner,
        //     &[ctrl_effectiveness_raw],
        //     &mut ctrl_effectiveness,
        //     block_size,
        // );

        // let mut drag_coeff_pitch_out = [0.];
        // let mut drag_coeff_roll_out = [0.];
        // let mut drag_coeff_yaw_out = [0.];

        let mut d_term_x_out = [0.];
        let mut d_term_y_out = [0.];
        let mut d_term_z_out = [0.];

        // dsp_api::biquad_cascade_df1_f32(
        //     &mut self.drag_coeff_pitch.inner,
        //     &[drag_coeff_pitch],
        //     &mut drag_coeff_pitch_out,
        //     block_size,
        // );
        //
        // dsp_api::biquad_cascade_df1_f32(
        //     &mut self.drag_coeff_roll.inner,
        //     &[drag_coeff_roll],
        //     &mut drag_coeff_roll_out,
        //     block_size,
        // );
        //
        // dsp_api::biquad_cascade_df1_f32(
        //     &mut self.drag_coeff_yaw.inner,
        //     &[drag_coeff_yaw],
        //     &mut drag_coeff_yaw_out,
        //     block_size,
        // );

        dsp_api::biquad_cascade_df1_f32(
            &mut self.d_term_x.inner,
            &[d_term_x],
            &mut d_term_x_out,
            block_size,
        );

        dsp_api::biquad_cascade_df1_f32(
            &mut self.d_term_y.inner,
            &[d_term_y],
            &mut d_term_y_out,
            block_size,
        );

        dsp_api::biquad_cascade_df1_f32(
            &mut self.d_term_z.inner,
            &[d_term_z],
            &mut d_term_z_out,
            block_size,
        );

        (
            // drag_coeff_pitch_out[0],
            // drag_coeff_roll_out[0],
            // drag_coeff_yaw_out[0],
            d_term_x_out[0],
            d_term_y_out[0],
            d_term_z_out[0],
        )
    }
}
