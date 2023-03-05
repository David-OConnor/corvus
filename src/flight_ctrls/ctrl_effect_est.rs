//! Estimation of control effectiveness based on recorded data. Regularly generates a map of
//! control commands (eg elevon deltas, rotor pair deltas etc) to angular accelerations per axis,
//! and/or its inverse.

use core::sync::atomic::{AtomicUsize, Ordering};

use num_traits::float::Float; // For sqrt.

use crate::util;

static SAMPLE_PT_I: AtomicUsize = AtomicUsize::new(0);

const NUM_SAMPLE_PTS: usize = 10;

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

/// Polynomial coefficients that map angular acceleration to either RPM, or servo positions.
pub struct AccelMap {
    /// AKA A
    pub square: f32,
    /// AKA B
    pub lin: f32,
    /// AKA C
    pub constant: f32,
}

impl AccelMap {
    /// Pts are (angular accel, RPM or servo posit delta) // todo: Do you want it the other way?
    pub fn new(pt0: (f32, f32), pt1: (f32, f32), pt2: (f32, f32)) -> Self {
        let (square, lin, constant) = util::create_polynomial_terms(pt0, pt1, pt2);
        Self {
            square,
            lin,
            constant,
        }
    }

    pub fn update_coeffs(&mut self, pt0: (f32, f32), pt1: (f32, f32), pt2: (f32, f32)) {
        let (square, lin, constant) = util::create_polynomial_terms(pt0, pt1, pt2);

        self.square = square;
        self.lin = lin;
        self.constant = constant;
    }

    /// Given a target angular acceleration, calculate a polynomial-fit RPM or servo position.
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

/// Map RPM to angular acceleration (thrust proxy). Average over time, and over all props.
/// Note that this relationship may be exponential, or something similar, with RPM increases
/// at higher ranges providing a bigger change in thrust.
/// For fixed wing, we use servo position instead of RPM.
// #[derive(Default)] // todo temp to get it to compille
// #[cfg(feature = "quad")]
// pub struct RpmAccelMap {
//     // // Value are in (RPM, acceleration (m/s^2))
//     // // todo: What is the max expected RPM? Adjust this A/R.
//     // // todo: An internet search implies 4-6k is normal.
//     //
//     // // Lower power should probably be from idle, not 0. So we include p_0 here.
//     // r_0: (f32, f32),
//     // r_1k: (f32, f32),
//     // r_2k: (f32, f32),
//     // r_3k: (f32, f32),
//     // r_4k: (f32, f32),
//     // r_5k: (f32, f32),
//     // r_6k: (f32, f32),
//     // r_7k: (f32, f32),
//     // r_8k: (f32, f32),
//     // r_9k: (f32, f32),
//     // r_10k: (f32, f32),
// }
//
// #[cfg(feature = "quad")]
// impl RpmAccelMap {
//     // todo: DRY with pwr to rpm MAP
//     /// Interpolate, to get power from this LUT.
//     pub fn pitch_rpm_to_accel(&self, rpm: f32) -> f32 {
//         // Ideally, this slope isn't used, since our map range exceeds motor RPM capability
//         // under normal circumstances.
//
//         // todo: This probably isn't the same for all axis! You most likely want
//         // todo to split this by axis long-term. Probalby by more fields on this struct.
//
//         // todo: QC order on this.
//         let end_slope = (self.r_10k.1 - self.r_9k.1) / (self.r_10k.0 - self.r_9k.0);
//         //
//
//         // todo: You want the opposite!
//
//         // todo: This isn't quite right. You need to take into account that the first tuple
//         // todo value of the fields is the actual RPM!
//         match rpm {
//             (0.0..=1_000.) => map_linear(rpm, (0.0, 1_000.), (self.r_0.1, self.r_1k.1)),
//             (1_000.0..=2_000.) => map_linear(rpm, (1_000., 2_000.), (self.r_1k.1, self.r_2k.1)),
//             (2_000.0..=3_000.) => map_linear(rpm, (2_000., 3_000.), (self.r_2k.1, self.r_3k.1)),
//             (3_000.0..=4_000.) => map_linear(rpm, (3_000., 4_000.), (self.r_3k.1, self.r_4k.1)),
//             (4_000.0..=5_000.) => map_linear(rpm, (4_000., 5_000.), (self.r_4k.1, self.r_5k.1)),
//             (5_000.0..=6_000.) => map_linear(rpm, (5_000., 6_000.), (self.r_5k.1, self.r_6k.1)),
//             (6_000.0..=7_000.) => map_linear(rpm, (6_000., 7_000.), (self.r_6k.1, self.r_7k.1)),
//             (7_000.0..=8_000.) => map_linear(rpm, (7_000., 8_000.), (self.r_7k.1, self.r_8k.1)),
//             (8_000.0..=9_000.) => map_linear(rpm, (8_000., 9_000.), (self.r_8k.1, self.r_9k.1)),
//             (9_000.0..=10_000.) => map_linear(rpm, (9_000., 10_000.), (self.r_9k.1, self.r_10k.1)),
//             // If above 10k, extrapolate from the prev range.
//             _ => rpm * end_slope,
//         }
//     }
//
//     pub fn roll_rpm_to_accel(&self, rpm: f32) -> f32 {
//         1. // todo
//     }
//
//     pub fn yaw_rpm_to_accel(&self, rpm: f32) -> f32 {
//         1. // todo
//     }
//
//     pub fn pitch_accel_to_rpm(&self, α: f32) -> f32 {
//         0. // todo
//     }
//
//     pub fn roll_accel_to_rpm(&self, α: f32) -> f32 {
//         0. // todo
//     }
//
//     pub fn yaw_accel_to_rpm(&self, α: f32) -> f32 {
//         0. // todo
//     }
//
//     /// Log a power, and rpm.
//     pub fn log_val(&mut self, rpm: f32, accel: f32) {
//         // todo: Allow for spin-up time.
//
//         // todo: filtering! But how, given the pwr these are logged at changes?
//         // todo: Maybe filter a an interpolation to the actual values, and store those?
//
//         if rpm < 0.1 {
//             self.r_0 = (rpm, accel);
//         } else if rpm < 0.2 {
//             self.r_1k = (rpm, accel);
//         } else if rpm < 0.3 {
//             self.r_2k = (rpm, accel);
//         } else if rpm < 0.4 {
//             self.r_3k = (rpm, accel);
//         } else if rpm < 0.5 {
//             self.r_4k = (rpm, accel);
//         } else if rpm < 0.6 {
//             self.r_5k = (rpm, accel);
//         } else if rpm < 0.7 {
//             self.r_6k = (rpm, accel);
//         } else if rpm < 0.8 {
//             self.r_7k = (rpm, accel);
//         } else if rpm < 0.9 {
//             self.r_8k = (rpm, accel);
//         } else if rpm < 1.0 {
//             self.r_9k = (rpm, accel);
//         } else {
//             self.r_10k = (rpm, accel);
//         }
//     }
// }
//
// // todo: Set up these defaults with something that should be safe during
// // todo initialization?
// #[cfg(feature = "quad")]
// impl Default for RpmAccelMap {
//     fn default() -> Self {
//         Self {
//
//         }
//     }
// }
//
// #[cfg(feature = "fixed-wing")]
// impl Default for ServoCmdAccelMap {
//     fn default() -> Self {
//         Self {}
//     }
// }

/// A logged mapping point, in one direction.
/// todo: timestamp?
#[derive(Default, Clone, Copy)]
pub struct AccelMapPt {
    pub rpm_or_servo_delta: f32,
    pub angular_accel: f32,
    // pub timestamp: usize, // todo: maybe
}

#[derive(Default)]
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
