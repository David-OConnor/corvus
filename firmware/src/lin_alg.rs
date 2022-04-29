//! Linear algebra operations, for use with sensor fusion.
//! Similar to https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionMath.h,
//! and 3d graphics math libs.
//! Note: We could replace much of this with CMSIS-DSP.

use core::ops::{Add, Mul, Sub};

use num_traits::float::Float;

/// 3D vector.
#[derive(Clone, Copy, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Add<Self> for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub<Self> for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Vector of zeros.
    pub fn zero() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }

    /// Vector of ones.
    pub fn ones() -> Self {
        Self {
            x: 1.,
            y: 1.,
            z: 1.,
        }
    }

    /// Returns true if the vector is zero.
    pub fn is_zero(&self) -> bool {
        let eps = 0.00001;
        self.x.abs() < eps && self.y.abs() == eps && self.z.abs() < eps
    }

    /// Returns a sum of all elements
    pub fn sum(&self) -> f32 {
        self.x + self.y + self.z
    }

    /// Calculate the cross product.
    pub fn cross(&self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    /// Calculates the Hadamard product (element-wise multiplication).
    pub fn hadamard_product(self, rhs: Self) -> Self {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }

    /// Returns the vector magnitude squared
    pub fn magnitude_squared(self) -> f32 {
        (self.hadamard_product(self)).sum()
    }

    /// Returns the vector magnitude.
    pub fn magnitude(&self) -> f32 {
        (self.magnitude_squared()).sqrt()
    }

    /// Returns the normalised version of the vector
    pub fn to_normalized(&self) -> Self {
        let mag_recip = 1. / self.magnitude();
        *self * mag_recip
    }
}

#[derive(Clone, Copy, Default)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Add<Self> for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Mul<Self> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl Mul<Vec3> for Quaternion {
    type Output = Self;

    /// Returns the multiplication of a Quaternion with a vector.  This is a
    /// normal Quaternion multiplication where the vector is treated a
    /// Quaternion with a W element value of zero.  The Quaternion is post-
    /// multiplied by the vector.
    fn mul(self, rhs: Vec3) -> Self::Output {
        Self {
            w: -self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x,
        }
    }
}

impl Quaternion {
    pub fn new_identity() -> Self {
        Self {
            w: 1.,
            x: 0.,
            y: 0.,
            z: 0.,
        }
    }

    /// Converts a Quaternion to Euler angles, in radians.
    pub fn to_euler(&self) -> EulerAngle {
        let qwqw_minus_half = self.w * self.w - 0.5; // calculate common terms to avoid repeated operations

        EulerAngle {
            roll: (self.y * self.z - self.w * self.x).atan2(qwqw_minus_half + self.z * self.z),
            pitch: -1.0 * (2.0 * (self.x * self.z + self.w * self.y)).asin(),
            yaw: (self.x * self.y - self.w * self.z).atan2(qwqw_minus_half + self.x * self.x),
        }
    }

    /// Converts a Quaternion to a rotation matrix
    #[rustfmt::skip]
    pub fn to_matrix(&self) -> Mat3 {
        let qwqw = self.w * self.w; // calculate common terms to avoid repeated operations
        let qwqx = self.w * self.x;
        let qwqy = self.w * self.y;
        let qwqz = self.w * self.z;
        let qxqy = self.x * self.y;
        let qxqz = self.x * self.z;
        let qyqz = self.y * self.z;

        Mat3 {
            data: [
                2.0 * (qwqw - 0.5 + self.x * self.x),
                2.0 * (qxqy + qwqz),
                2.0 * (qxqz - qwqy),
                2.0 * (qxqy - qwqz),
                2.0 * (qwqw - 0.5 + self.y * self.y),
                2.0 * (qyqz + qwqx),
                2.0 * (qxqz + qwqy),
                2.0 * (qyqz - qwqx),
                2.0 * (qwqw - 0.5 + self.z * self.z),
            ]
        }
    }

    pub fn magnitude(&self) -> f32 {
        (self.w.powi(2) + self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    /// Returns the normalised quaternion
    pub fn to_normalized(&self) -> Self {
        let mag_recip = 1. / self.magnitude();

        Self {
            w: self.w * mag_recip,
            x: self.x * mag_recip,
            y: self.y * mag_recip,
            z: self.z * mag_recip,
        }
    }
}

/// 3x3 matrix in row-major order.
/// See http://en.wikipedia.org/wiki/Row-major_order
#[derive(Clone)]
pub struct Mat3 {
    pub data: [f32; 9],
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3 {
            x: rhs.x * self.data[0] + rhs.y * self.data[1] + rhs.z * self.data[2],
            y: rhs.x * self.data[3] + rhs.y * self.data[4] + rhs.z * self.data[5],
            z: rhs.x * self.data[6] + rhs.y * self.data[7] + rhs.z * self.data[8],
        }
    }
}

impl Mat3 {
    #[rustfmt::skip]
    pub fn new_identity() -> Self {
        Self {
            data: [
                1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.,
            ]
        }
    }
}

/// Euler angles.
pub struct EulerAngle {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl EulerAngle {
    /// Euler angles of zero.
    pub fn zero() -> Self {
        Self {
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
        }
    }
}
