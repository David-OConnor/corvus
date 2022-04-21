//! Contains misc and utility functions.

use cmsis_dsp_sys as dsp_sys;

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: dsp_sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

// todo: Move Quaternion and RotorMath elsewhere as appropriate.

/// A quaternion. Used for attitude state
struct Quaternion {
    i: f32,
    j: f32,
    k: f32,
    l: f32,
}

// impl Sub for Quaternion {
//     type Output = Self;
//
//     fn sub(self, other: Self) -> Self::Output {
//         Self {
//             x: self.x - other.x,
//             y: self.y - other.y,
//             z: self.z - other.z,
//         }
//     }
// }

impl Quaternion {
    pub fn new(i: f32, j: f32, k: f32, l: f32) -> Self {
        Self { i, j, k, l }
    }
}

/// A generalized quaternion
struct RotorMath {}

/// Utility fn to make up for `core::cmp::max` requiring f32 to impl `Ord`, which it doesn't.
/// todo: Move elsewhere?
pub fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

pub fn abs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
}