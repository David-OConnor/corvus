//! Madgwick filter, for attaining an attitude platform from a 3-axis acceleratometer, gyro, and optionally
//! magnetometer.
//! https://ahrs.readthedocs.io/en/latest/filters/madgwick.html:
//!
//! "This is an orientation filter applicable to IMUs consisting of tri-axial gyroscopes and accelerometers,
//! and MARG arrays, which also include tri-axial magnetometers, proposed by Sebastian Madgwick [Madgwick].
//! The filter employs a quaternion representation of orientation to describe the nature of orientations
//! in three-dimensions and is not subject to the singularities associated with an Euler
//! angle representation, allowing accelerometer and magnetometer data to be used in an analytically
//! derived and optimised gradient-descent algorithm to compute the direction of the gyroscope
//! measurement error as a quaternion derivative.
//!
//! [Original algorithm, by Seb Madgwick, in C](https://github.com/xioTechnologies/Fusion)
//! [Translated into python[(https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/madgwick.py)
//!
//! https://github.com/chris1seto/OzarkRiver/tree/4channel/FlightComputerFirmware/Src/Madgwick
//! https://github.com/bjohnsonfl/Madgwick_Filter
//! https://github.com/chris1seto/OzarkRiver/blob/main/FlightComputerFirmware/Src/ImuAhrs.c


use crate::sensor_fusion::ImuReadings;

use core::{
    f32::consts::TAU,
};

use super::lin_alg::{Mat3, Vec3, Quaternion, EulerAngle};


// FusionAhrs.h:

/**
 * @brief AHRS algorithm settings.
 */
struct AhrsSettings {
    pub gain: f32,
    pub accelerationRejection: f32,
    pub magneticRejection: f32,
    pub rejectionTimeout: u32, // todo type?
}

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * must not be accessed by the application.
 */
struct Ahrs {
    pub settings: AhrsSettings,
    pub quaternion: FusionQuaternion,
    pub accelerometer: FusionVector,
    pub initialising: bool,
    pub rampedGain: f32,
    pub rampedGainStep: f32,
    pub halfAccelerometerFeedback: FusionVector,
    pub halfMagnetometerFeedback: FusionVector,
    pub accelerometerIgnored: bool,
    pub accelerationRejectionTimer: u32, // todo type?
    pub accelerationRejectionTimeout: bool,
    pub magnetometerIgnored: bool,
    pub magneticRejectionTimer: u32, // todo type?
    pub magneticRejectionTimeout: bool,
}

/**
 * @brief AHRS algorithm internal states.
 */
struct AhrsInternalStates {
    pub accelerationError: f32,
    pub accelerometerIgnored: bool,
    pub accelerationRejectionTimer: f32,
    pub magneticError: f32,
    pub magnetometerIgnored: bool,
    pub magneticRejectionTimer: f32,
}

/**
 * @brief AHRS algorithm flags.
 */
struct AhrsFlags {
    pub initialising: bool,
    pub accelerationRejectionWarning: bool,
    pub accelerationRejectionTimeout: bool,
    pub magneticRejectionWarning: bool,
    pub magneticRejectionTimeout: bool,
}

// FusionAxes.h

/// Axes alignment describing the sensor axes relative to the body axes.
/// For example, if the body X axis is aligned with the sensor Y axis and the
/// body Y axis is aligned with sensor X axis but pointing the opposite direction
/// then alignment is +Y-X+Z.
#[derive(Clone, Copy)]
enum AxesAlignment {
    PXPYPZ, /* +X+Y+Z */
    PXNZPY, /* +X-Z+Y */
    PXNYNZ, /* +X-Y-Z */
    PXPZNY, /* +X+Z-Y */
    NXPYNZ, /* -X+Y-Z */
    NXPZPY, /* -X+Z+Y */
    NXNYPZ, /* -X-Y+Z */
    NXNZNY, /* -X-Z-Y */
    PYNXPZ, /* +Y-X+Z */
    PYNZNX, /* +Y-Z-X */
    PYPXNZ, /* +Y+X-Z */
    PYPZPX, /* +Y+Z+X */
    NYPXPZ, /* -Y+X+Z */
    NYNZPX, /* -Y-Z+X */
    NYNXNZ, /* -Y-X-Z */
    NYPZNX, /* -Y+Z-X */
    PZPYNX, /* +Z+Y-X */
    PZPXPY, /* +Z+X+Y */
    PZNYPX, /* +Z-Y+X */
    PZNXNY, /* +Z-X-Y */
    NZPYPX, /* -Z+Y+X */
    NZNXPY, /* -Z-X+Y */
    NZNYNX, /* -Z-Y-X */
    NZPXNY, /* -Z+X-Y */
}


/// Swaps sensor axes for alignment with the body axes.
/// @param sensor Sensor axes.
/// @param alignment Axes alignment.
/// return Sensor axes aligned with the body axes.
fn FusionAxesSwap(sensor: Vec3, alignment: AxesAlignment) -> Vec3 {
    match alignment {
        AxesAlignment::PXPYPZ => {
            sensor
        }
        AxesAlignment::PXNZPY => {
            Vec3 {
                x: sensor.x,
                y: -sensor.z,
                z: sensor.y,
            }
        }
        AxesAlignment::PXNYNZ => {
            Vec3 {

                x: sensor.x,
                y: -sensor.y,
                z: -sensor.z,
            }
        }
        AxesAlignment::PXPZNY => {
            Vec3 {
                x: sensor.x,
                y: sensor.z,
                z: -sensor.y,
            }
        }
        AxesAlignment::NXPYNZ => {
            Vec3 {
                x: -sensor.x,
                y: sensor.y,
                z: -sensor.z,
            }
        }
        AxesAlignment::NXPZPY => {
            Vec3 {
                x: -sensor.x,
                y: sensor.z,
                z: sensor.y,
            }
        }
        AxesAlignment::NXNYPZ => {
            Vec3 {
                x: -sensor.x,
                y: -sensor.y,
                z: sensor.z,
            }
        }
        AxesAlignment::NXNZNY => {
            Vec3 {
                x: -sensor.x,
                y: -sensor.z,
                z: -sensor.y,
            }
        }
        AxesAlignment::PYNXPZ => {
            Vec3 {
                x: sensor.y,
                y: -sensor.x,
                z: sensor.z,
            }
        }
        AxesAlignment::PYNZNX => {
            Vec3 {
                x: sensor.y,
                y: -sensor.z,
                z: -sensor.x,
            }
        }
        AxesAlignment::PYPXNZ => {
            Vec3 {
                x: sensor.y,
                y: sensor.x,
                z: -sensor.z,
            }
        }
        AxesAlignment::PYPZPX => {
            Vec3 {
                x: sensor.y,
                y: sensor.z,
                z: sensor.x,
            }
        }
        AxesAlignment::NYPXPZ => {
            Vec3 {
                x: -sensor.y,
                y: sensor.x,
                z: sensor.z,
            }
        }
        AxesAlignment::NYNZPX => {
            Vec3 {
                x: -sensor.y,
                y: -sensor.z,
                z: sensor.x,
            }
        }
        AxesAlignment::NYNXNZ => {
            Vec3 {
                x: -sensor.y,
                y: -sensor.x,
                z: -sensor.z,
            }
        }
        AxesAlignment::NYPZNX => {
            Vec3 {
                x: -sensor.y,
                y: sensor.z,
                z: -sensor.x,
            }
        }
        AxesAlignment::PZPYNX => {
            Vec3 {
                x: sensor.z,
                y: sensor.y,
                z: -sensor.x,
            }
        }
        AxesAlignment::PZPXPY => {
            Vec3 {
                x: sensor.z,
                y: sensor.x,
                z: sensor.y,
            }
        }
        AxesAlignment::PZNYPX => {
            Vec3 {
                x: sensor.z,
                y: -sensor.y,
                z: sensor.x,
            }
        }
        AxesAlignment::PZNXNY => {
            Vec3 {
                x: sensor.z,
                y: -sensor.x,
                z: -sensor.y,
            }
        }
        AxesAlignment::NZPYPX => {
            Vec3 {
                x: -sensor.z,
                y: sensor.y,
                z: sensor.x,
            }
        }
        AxesAlignment::NZNXPY => {
            Vec3 {
                x: -sensor.z,
                y: -sensor.x,
                z: sensor.y,
            }
        }
        AxesAlignment::NZNYNX => {
            Vec3 {
                x: -sensor.z,
                y: -sensor.y,
                z: -sensor.x,
            }
        }
        AxesAlignment::NZPXNY => {
            Vec3 {
                x: -sensor.z,
                y: sensor.x,
                z: -sensor.y,
            }
        }
    }
}

// FusionOffset.h:

/// Gyroscope offset algorithm structure.  Structure members are used
/// internally and must not be accessed by the application.
struct Offset {
    pub filterCoefficient: f32,
    pub timeout: u32,
    pub timer: u32,
    pub gyroscopeOffset: Vec3
}
