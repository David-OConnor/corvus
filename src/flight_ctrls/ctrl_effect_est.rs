//! Estimation of control effectiveness based on recorded data. Regularly generates a map of
//! control commands (eg elevon deltas, rotor pair deltas etc) to angular accelerations per axis,
//! and/or its inverse.

use core::sync::atomic::{AtomicUsize, Ordering};

use num_traits::float::Float;

use crate::util;

use defmt::println;

static SAMPLE_PT_I: AtomicUsize = AtomicUsize::new(0);

pub const NUM_SAMPLE_PTS: usize = 30;

// This point of 0 command, 0 acceleration, is an anchor.
const PT_0: AccelMapPt = AccelMapPt {
    angular_accel: 0.,
    ctrl_cmd: 0.,
    timestamp: 0.,
};

// todo: Leaning towards this: Store a collection of pitch, roll, and yaw angular accels, as well
// todo as corresponding servo settings (fixed) or motor RPMs (quad). Also store altitude and time-of-measurement.
// todo: From this, create a model. Store readings and/or model in external flash periodically,
// todo for init at next takeoff.

// We use this approach instead of feature-gating an `AccelMap` struct, to make the
// struct names here more explicit.
// #[cfg(feature = "quad")]
// pub type AccelMap = RpmAccelMap;
// #[cfg(feature = "fixed-wing")]
// pub type AccelMap = ServoCmdAccelMap;

// pub struct ServoCmdAccelMap {}
//
// impl ServoCmdAccelMap {
//     /// See `pitch_delta` etc on `CtrlSfcPosits` for how these deltas
//     /// are calculated.
//     pub fn pitch_cmd_to_accel(&self, cmd: f32) -> f32 {
//         1.
//     }
//
//     pub fn roll_cmd_to_accel(&self, cmd: f32) -> f32 {
//         1.
//     }
//
//     pub fn yaw_cmd_to_accel(&self, cmd: f32) -> f32 {
//         1.
//     }
//
//     pub fn pitch_accel_to_cmd(&self, accel: f32) -> f32 {
//         1.
//     }
//
//     pub fn roll_accel_to_cmd(&self, accel: f32) -> f32 {
//         1.
//     }
//
//     pub fn yaw_accel_to_cmd(&self, accel: f32) -> f32 {
//         1.
//     }
// }

#[derive(Default, Clone, Copy)] // `Copy` for use in array init.
pub struct AccelMapPt {
    /// In radians-per-second
    pub angular_accel: f32,
    /// This could be servo command, servo measured position, RPM delta, or rotor power delta
    pub ctrl_cmd: f32,
    /// In seconds.
    pub timestamp: f32,
}

/// Polynomial coefficients that map angular acceleration to either RPM, or servo positions.
pub struct AccelMap {
    /// AKA A
    pub square: f32,
    /// AKA B
    pub lin: f32,
    /// AKA C
    pub constant: f32,
}

// todo: Here next (Apr 2023). Using this collection of points, create a linear map. Probably least-suqres.
// todo: You need a way to get timestamps.

impl AccelMap {
    // /// Pts are (angular accel, RPM or servo posit delta) // todo: Do you want it the other way?
    // pub fn new(pt0: AccelMapPt, pt1: AccelMapPt, pt2: AccelMapPt) -> Self {
    //     // let (square, lin, constant) = util::create_polynomial_terms(pt0, pt1, pt2);
    //     let (square, lin, constant) = (0., 0., 0.);
    //
    //     Self {
    //         square,
    //         lin,
    //         constant,
    //     }
    // }

    /// Fit a least-squares approximation from logged points.
    pub fn update_coeffs(&mut self, pts: &[AccelMapPt]) {
        // todo: Do we need to take into account different ranges that may not be in the recent
        // todo data?

        // todo: Sloppy estimation here with only 3 pts.
        let (square, lin, constant) = util::create_polynomial_terms(
            (
                pts[NUM_SAMPLE_PTS - 1].angular_accel,
                pts[NUM_SAMPLE_PTS - 1].ctrl_cmd,
            ),
            (
                pts[NUM_SAMPLE_PTS - 5].angular_accel,
                pts[NUM_SAMPLE_PTS - 5].ctrl_cmd,
            ),
            (0., 0.),
        );

        // println!("Latest pt: {}, {}", pts[NUM_SAMPLE_PTS - 1].angular_accel, pts[NUM_SAMPLE_PTS - 1].ctrl_cmd);

        // let constant = 0.;
        // let squares = 0.;
        // let lin = (pts[pts.len()].ctrl_cmd / pts[pts.len()].angular_accel);

        self.square = square;
        self.lin = lin;
        self.constant = constant;

        // todo: Temp / v sloppy!!
        self.square = 0.;
        self.lin = 0.01;
        self.constant = 0.;
    }

    /// Given a target angular acceleration, calculate a polynomial-fit RPM delta or servo position.
    /// This is called by flight control logic to determine how to set the control mix.
    pub fn interpolate(&self, target_accel: f32) -> f32 {
        static mut i: u32 = 0;
        unsafe { i += 1 };
        if unsafe { i } % 3_000 == 0 {
            // println!(
            //     "sq: {}, lin: {}, const: {}",
            //     self.square, self.lin, self.constant
            // );
        }

        self.square * target_accel.powi(2) + self.lin * target_accel + self.constant
    }
}

impl Default for AccelMap {
    fn default() -> Self {
        // Rpm. Units are RPM / (rad/s^2)
        #[cfg(feature = "quad")]
        return Self {
            square: 0.,
            lin: 1_000.,
            constant: 0.,
        };

        // Servo posits. Units are servo_posit / (rad/s^2)
        #[cfg(feature = "quad")]
        return Self {
            square: 0.,
            lin: 1_000.,
            constant: 0.,
        };
    }
}

pub struct AccelMaps {
    pub map_pitch: AccelMap,
    pub map_roll: AccelMap,
    pub map_yaw: AccelMap,

    /// A buffer of sample points. More than required (3) for now.
    /// Format is (angular accel, RPM or servo posit delta).
    pub sample_pts_pitch: [AccelMapPt; NUM_SAMPLE_PTS],
    pub sample_pts_roll: [AccelMapPt; NUM_SAMPLE_PTS],
    pub sample_pts_yaw: [AccelMapPt; NUM_SAMPLE_PTS],
}

impl Default for AccelMaps {
    /// Long sizes of `AccelMapPt` are blocked from a Deafult derive, evidently.
    fn default() -> Self {
        let pts = [AccelMapPt::default(); NUM_SAMPLE_PTS];

        Self {
            map_pitch: Default::default(),
            map_roll: Default::default(),
            map_yaw: Default::default(),
            sample_pts_pitch: pts,
            sample_pts_roll: pts,
            sample_pts_yaw: pts,
        }
    }
}

impl AccelMaps {
    /// Add a new sample point for each of pitch, roll and yaw. Update coefficients to
    /// incorporate latest 3 points.
    pub fn log_pt(&mut self, pt_pitch: AccelMapPt, pt_roll: AccelMapPt, pt_yaw: AccelMapPt) {
        // todo: Sanity-check and reject as-requird, relying on other points you've logged recently.
        let i = SAMPLE_PT_I.fetch_add(1, Ordering::Relaxed);

        // todo: This needs work! Likely scenario si many points near identitcal are logged in a row,
        // todo and the square and linear terms become bogus!

        if i >= NUM_SAMPLE_PTS - 1 {
            // Loop the atomic index back to the first position. (The retrieved value, `i` is still
            // OK without modification)
            SAMPLE_PT_I.store(0, Ordering::Release);
        }

        self.sample_pts_pitch[i] = pt_pitch;
        self.sample_pts_roll[i] = pt_roll;
        self.sample_pts_yaw[i] = pt_yaw;

        // todo: Slice these from i!
        self.map_pitch.update_coeffs(&self.sample_pts_pitch);
        self.map_roll.update_coeffs(&self.sample_pts_roll);
        self.map_yaw.update_coeffs(&self.sample_pts_yaw);
    }
}
