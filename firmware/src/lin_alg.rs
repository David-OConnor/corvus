//! Linear algebra operations, for use with sensor fusion.
//! Similar to https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionMath.h,
//! and 3d graphics math libs.
//! Note: We could replace much of this with CMSIS-DSP.

use core::ops::{Add, Sub, Mul};

/// 3D vector.
#[derive(Clone, Copy)]
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
    /// Vector of zeros.
    pub fn zero() -> Self {
        Self { x: 0., y: 0., z: 0. }
    }

    /// Vector of ones.
    pub fn one() -> Self {
        Self { x: 1., y: 1., z: 1. }
    }

    /// Returns true if the vector is zero.
    pub fn is_zero(&self) -> bool {
        self.x == 0.0 && self.y == 0.0 && self.z == 0.0
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
    fn hadamard_product(self, rhs: Self) -> Self {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }
}

pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Add<Self> for Quaternion{
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

    /// Returns the multiplication of a quaternion with a vector.  This is a
    /// normal quaternion multiplication where the vector is treated a
    /// quaternion with a W element value of zero.  The quaternion is post-
    /// multiplied by the vector.
    fn mul(&self, rhs: Vec3) -> Self::Output {
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
        Self { w: 1., x: 0., y: 0., z: 0. }
    }


    /// Converts a quaternion to Euler angles, in radians.
    pub fn to_euler(&self) -> EulerAngle {
        let qwqw_minus_half = self.w * self.w - 0.5; // calculate common terms to avoid repeated operations

        EulerAngle {
            roll: atan2f(self.y * self.z - self.w * self.x, qwqw_minus_half + self.z * self.z),
            pitch: -1.0 * asinf(2.0 * (self.x * self.z + self.w * self.y)),
            yaw: atan2f(self.x * self.y - self.w * self.z, qwqw_minus_half + self.x * self.x),
        }
    }

    /// Converts a quaternion to a rotation matrix
    #[rustfmt::skip]
    pub fn to_matrix(&self) -> Mat3 {
        let qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
        let qwqx = Q.w * Q.x;
        let qwqy = Q.w * Q.y;
        let qwqz = Q.w * Q.z;
        let qxqy = Q.x * Q.y;
        let qxqz = Q.x * Q.z;
        let qyqz = Q.y * Q.z;

        Mat3 {
            data: [
                2.0 * (qwqw - 0.5 + Q.x * Q.x),
                2.0 * (qxqy + qwqz),
                2.0 * (qxqz - qwqy),
                2.0 * (qxqy - qwqz),
                2.0 * (qwqw - 0.5 + Q.y * Q.y),
                2.0 * (qyqz + qwqx),
                2.0 * (qxqz + qwqy),
                2.0 * (qyqz - qwqx),
                2.0 * (qwqw - 0.5 + Q.z * Q.z),
            ]
        }
    }


}

/// 3x3 matrix in row-major order.
/// See http://en.wikipedia.org/wiki/Row-major_order
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
    roll: f32,
    pitch: f32,
    yaw: f32,
}

impl EulerAngle {
    /// Euler angles of zero.
    pub fn zero() -> Self {
        Self { roll: 0., pitch: 0., yaw: 0. }
    }
}


/// Calculates the reciprocal of the square root.
/// See https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
/// Returns reciprocal of the square root of x.
fn FastInverseSqrt(x: f32) -> f32 {

    typedef union {
        float f;
        int32_t i;
    } Union32;

    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231 - 0.714158168 * x * union32.f * union32.f);
}

/**
 * @brief Returns the vector magnitude squared.
 * @param vector Vector.
 * @return Vector magnitude squared.
 */
static inline float FusionVectorMagnitudeSquared(const FusionVector vector) {
return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

/**
 * @brief Returns the vector magnitude.
 * @param vector Vector.
 * @return Vector magnitude.
 */
static inline float FusionVectorMagnitude(const FusionVector vector) {
return sqrtf(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief Returns the normalised vector.
 * @param vector Vector.
 * @return Normalised vector.
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
#ifdef FUSION_USE_NORMAL_SQRT
const float magnitudeReciprocal = 1.0 / sqrtf(FusionVectorMagnitudeSquared(vector));
#else
const float magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
#endif
return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}


/**
 * @brief Returns the quaternion conjugate.
 * @param quaternion Quaternion.
 * @return Quaternion conjugate.
 */
static inline FusionQuaternion FusionQuaternionConjugate(const FusionQuaternion quaternion) {
FusionQuaternion conjugate;
conjugate.element.w = quaternion.element.w;
conjugate.element.x = -1.0 * quaternion.element.x;
conjugate.element.y = -1.0 * quaternion.element.y;
conjugate.element.z = -1.0 * quaternion.element.z;
return conjugate;
}

/**
 * @brief Returns the normalised quaternion.
 * @param quaternion Quaternion.
 * @return Normalised quaternion.
 */
static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion) {
const Q quaternion.element
#ifdef FUSION_USE_NORMAL_SQRT
const float magnitudeReciprocal = 1.0 / sqrtf(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#else
const float magnitudeReciprocal = FusionFastInverseSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#endif
FusionQuaternion normalisedQuaternion;
normalisedQuaternion.element.w = Q.w * magnitudeReciprocal;
normalisedQuaternion.element.x = Q.x * magnitudeReciprocal;
normalisedQuaternion.element.y = Q.y * magnitudeReciprocal;
normalisedQuaternion.element.z = Q.z * magnitudeReciprocal;
return normalisedQuaternion;
#undef Q
}

//------------------------------------------------------------------------------
// Inline functions - Matrix operations

