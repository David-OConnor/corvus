//! Estimation of control effectiveness based on recorded data. Regularly generates a map of
//! control commands (eg elevon deltas, rotor pair deltas etc) to angular accelerations per axis,
//! and/or its inverse.

use core::sync::atomic::{AtomicUsize, Ordering};

use num_traits::float::Float;

use crate::util;

static SAMPLE_PT_I: AtomicUsize = AtomicUsize::new(0);

pub const NUM_SAMPLE_PTS: usize = 30;

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

pub struct AccepMapPt {
    /// In radians-per-second
    pub angular_accel: f32,
    /// This could be servo command, servo measured position, RPM delta, or rotor power delta
    pub ctrl_cmd: f32,
    pub timestamp: u32, // todo?
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
    /// Pts are (angular accel, RPM or servo posit delta) // todo: Do you want it the other way?
    pub fn new(pt0: AccepMapPt, pt1: AccepMapPt, pt2: AccepMapPt) -> Self {
        // let (square, lin, constant) = util::create_polynomial_terms(pt0, pt1, pt2);
        let (square, lin, constant) = (0., 0., 0.);

        Self {
            square,
            lin,
            constant,
        }
    }

    pub fn update_coeffs(&mut self, pt0: (f32, f32), pt1: (f32, f32), pt2: (f32, f32)) {
        // let (square, lin, constant) = util::create_polynomial_terms(pt0, pt1, pt2);
        let (square, lin, constant) = (0., 0., 0.);

        self.square = square;
        self.lin = lin;
        self.constant = constant;
    }

    /// Fit a least-squares approximation from logged points.
    pub fn update_from_logged_pts(&mut self, pts: &[AccelMapPt]) {
        // todo: Do we need to take into account different ranges that may not be in the recent
        // todo data?
    }

    /// Given a target angular acceleration, calculate a polynomial-fit RPM delta or servo position.
    /// This is called by flight control logic to determine how to set the control mix.
    pub fn interpolate(&self, target_accel: f32) -> f32 {
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

/// A logged mapping point, for one axis.
/// todo: timestamp?
#[derive(Default, Clone, Copy)]
pub struct AccelMapPt {
    pub rpm_or_servo_delta: f32,
    pub angular_accel: f32,
    // pub timestamp: usize, // todo: maybe
}

pub struct AccelMaps {
    pub rpm_to_accel_pitch: AccelMap,
    pub rpm_to_accel_roll: AccelMap,
    pub rpm_to_accel_yaw: AccelMap,

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
            rpm_to_accel_pitch: Default::default(),
            rpm_to_accel_roll: Default::default(),
            rpm_to_accel_yaw: Default::default(),
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

        let active_pt_pitch_0 = self.sample_pts_pitch[(i - 2) % NUM_SAMPLE_PTS];
        let active_pt_pitch_1 = self.sample_pts_pitch[(i - 1) % NUM_SAMPLE_PTS];
        let active_pt_pitch_2 = self.sample_pts_pitch[i % NUM_SAMPLE_PTS];

        let active_pt_roll_0 = self.sample_pts_roll[(i - 2) % NUM_SAMPLE_PTS];
        let active_pt_roll_1 = self.sample_pts_roll[(i - 1) % NUM_SAMPLE_PTS];
        let active_pt_roll_2 = self.sample_pts_roll[i % NUM_SAMPLE_PTS];

        let active_pt_yaw_0 = self.sample_pts_yaw[(i - 2) % NUM_SAMPLE_PTS];
        let active_pt_yaw_1 = self.sample_pts_yaw[(i - 1) % NUM_SAMPLE_PTS];
        let active_pt_yaw_2 = self.sample_pts_yaw[i % NUM_SAMPLE_PTS];

        self.rpm_to_accel_pitch.update_coeffs(
            (
                active_pt_pitch_0.angular_accel,
                active_pt_pitch_0.rpm_or_servo_delta,
            ),
            (
                active_pt_pitch_1.angular_accel,
                active_pt_pitch_1.rpm_or_servo_delta,
            ),
            (
                active_pt_pitch_2.angular_accel,
                active_pt_pitch_2.rpm_or_servo_delta,
            ),
        );

        self.rpm_to_accel_roll.update_coeffs(
            (
                active_pt_roll_0.angular_accel,
                active_pt_roll_0.rpm_or_servo_delta,
            ),
            (
                active_pt_roll_1.angular_accel,
                active_pt_roll_1.rpm_or_servo_delta,
            ),
            (
                active_pt_roll_2.angular_accel,
                active_pt_roll_2.rpm_or_servo_delta,
            ),
        );

        self.rpm_to_accel_yaw.update_coeffs(
            (
                active_pt_yaw_0.angular_accel,
                active_pt_yaw_0.rpm_or_servo_delta,
            ),
            (
                active_pt_yaw_1.angular_accel,
                active_pt_yaw_1.rpm_or_servo_delta,
            ),
            (
                active_pt_yaw_2.angular_accel,
                active_pt_yaw_2.rpm_or_servo_delta,
            ),
        );
    }
}
