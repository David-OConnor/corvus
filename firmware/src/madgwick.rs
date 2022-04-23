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

use cmsis_dsp_sys::{arm_cos_f32 as cos, arm_sin_f32 as sin};

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

/// AHRS algorithm structure.  Structure members are used internally and
/// must not be accessed by the application.
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

impl Ahrs {
    /// Resets the AHRS algorithm.  This is equivalent to reinitialising the
    /// algorithm while maintaining the current settings.
    /// param ahrs AHRS algorithm structure.
    fn AhrsReset(&mut self) {
        self.quaternion = Quaternion::new_identity(),
        self.accelerometer = Vec3::zero();
        self.initialising = true;
        self.rampedGain = INITIAL_GAIN;
        self.halfAccelerometerFeedback = Vec3::zero();
        self.halfMagnetometerFeedback = Vec3::zero();
        self.accelerometerIgnored = false;
        self.accelerationRejectionTimer = 0;
        self.accelerationRejectionTimeout = false;
        self.magnetometerIgnored = false;
        self.magneticRejectionTimer = 0;
        self.magneticRejectionTimeout = false;
    }

    /// brief Sets the AHRS algorithm settings.
    /// param ahrs AHRS algorithm structure.
    /// aram settings Settings.
    fn FusionAhrsSetSettings(&mut self, settings: &AhrsSettings) {
        self.settings.gain = settings.gain;
        if settings.accelerationRejection == 0.0 || settings.rejectionTimeout == 0 {
            self.settings.accelerationRejection = FLT_MAX;
        } else {
            self.settings.accelerationRejection = (0.5 * sin(settings.accelerationRejection)).pow(2);
        }
        if (settings.magneticRejection == 0.0) || (settings.rejectionTimeout == 0) {
            self.settings.magneticRejection = FLT_MAX;
        } else {
            self.settings.magneticRejection = (0.5 * sin(FusionDegreesToRadians(settings.magneticRejection)).pow(2);
        }
        self.settings.rejectionTimeout = settings.rejectionTimeout;
        if self.initialising == false {
            self.rampedGain = self.settings.gain;
        }
        self.rampedGainStep = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD;
    }

    /**
     * @brief Updates the AHRS algorithm using the gyroscope, accelerometer, and
     * magnetometer measurements.
     * @param ahrs AHRS algorithm structure.
     * @param gyroscope Gyroscope measurement in degrees per second.
     * @param accelerometer Accelerometer measurement in g.
     * @param magnetometer Magnetometer measurement in arbitrary units.
     * @param deltaTime Delta time in seconds.
     */
    fn FusionAhrsUpdate(&mut self, gyroscope: Vec3, accelerometer: Vec3, magnetometer: Vec3, dt: f32) {
        let q = ahrs.quaternion;

        // Store accelerometer
        self.accelerometer = accelerometer;

        // Ramp down gain during initialisation
        if self.initialising {
            self.rampedGain -= self.rampedGainStep * dt;
            if self.rampedGain < self.settings.gain {
                self.rampedGain = self.settings.gain;
                self.initialising = false;
                self.accelerationRejectionTimeout = false;
            }
        }

        // Calculate direction of gravity indicated by algorithm
        let half_gravity = Vec3 {
            x: q.x * q.z - q.w * q.y,
            y: q.w * q.x + q.y * q.z,
            z: q.w * q.w - 0.5 + q.z * q.z,
        }; // equal to 3rd column of rotation matrix representation scaled by 0.5

        // Calculate accelerometer feedback
        let mut half_accelerometer_feedback = Vec3::zero();
        self.accelerometerIgnored = true;
        if !accelerometer.is_zero() {

            // Enter acceleration recovery state if acceleration rejection times out
            if self.accelerationRejectionTimer >= self.settings.rejectionTimeout {
                let quaternion = self.quaternion;
                self.FusionAhrsReset();
                self.quaternion = quaternion;
                self.accelerationRejectionTimer = 0;
                self.accelerationRejectionTimeout = true;
            }

            // Calculate accelerometer feedback scaled by 0.5
            self.halfAccelerometerFeedback = FusionVectorCrossProduct(FusionVectorNormalise(accelerometer), half_gravity);

            // Ignore accelerometer if acceleration distortion detected
            if self.initialising == true || FusionVectorMagnitudeSquared(self.halfAccelerometerFeedback) <= self.settings.accelerationRejection {
                half_accelerometer_feedback = self.halfAccelerometerFeedback;
                self.accelerometerIgnored = false;
                self.accelerationRejectionTimer -= if self.accelerationRejectionTimer >= 10 { 10 } else { 0 };
            } else {
                self.accelerationRejectionTimer += 1;
            }
        }

        // Calculate magnetometer feedback
        let mut halfMagnetometerFeedback = Vec3::zero();
        self.magnetometerIgnored = true;
        if !magnetometer.is_zero() {

            // Set to compass heading if magnetic rejection times out
            self.magneticRejectionTimeout = false;
            if self.magneticRejectionTimer >= self.settings.rejectionTimeout {
                self.FusionAhrsSetHeading(FusionCompassCalculateHeading(half_gravity, magnetometer));
                self.magneticRejectionTimer = 0;
                self.magneticRejectionTimeout = true;
            }

            // Compute direction of west indicated by algorithm
            let halfWest = Vec3 {
                x: q.x * q.y + q.w * q.z,
                y: q.w * q.w - 0.5 + q.y * q.y,
                z: q.y * q.z - q.w * q.x
            }; // equal to 2nd column of rotation matrix representation scaled by 0.5

            // Calculate magnetometer feedback scaled by 0.5
            self.halfMagnetometerFeedback = FusionVectorCrossProduct(FusionVectorNormalise(FusionVectorCrossProduct(half_gravity, magnetometer)), halfWest);

            // Ignore magnetometer if magnetic distortion detected
            if self.initialising == true || FusionVectorMagnitudeSquared(self.halfMagnetometerFeedback) <= self.settings.magneticRejection {
                halfMagnetometerFeedback = self.halfMagnetometerFeedback;
                self.magnetometerIgnored = false;
                self.magneticRejectionTimer -= if self.magneticRejectionTimer >= 10 { 10 } else { 0 };
            } else {
                self.magneticRejectionTimer += 1;
            }
        }

        // Convert gyroscope to radians per second scaled by 0.5
        let half_gyro = gyroscope * 0.5;

        // Apply feedback to gyroscope
        let adjusted_half_gyro = half_gyro + (half_accelerometer_feedback + halfMagnetometerFeedback) * self.rampedGain;

        // Integrate rate of change of quaternion
        self.quaternion = self.quaternion + self.quaternion * (adjusted_half_gyro * deltaTime);

        // Normalise quaternion
        self.quaternion = FusionQuaternionNormalise(self.quaternion);
    }

    /**
     * @brief Updates the AHRS algorithm using the gyroscope and accelerometer
     * measurements only.
     * @param ahrs AHRS algorithm structure.
     * @param gyroscope Gyroscope measurement in degrees per second.
     * @param accelerometer Accelerometer measurement in g.
     * @param deltaTime Delta time in seconds.
     */
    fn FusionAhrsUpdateNoMagnetometer(&mut self, gyroscope: Vec3, accelerometer: Vec3, dt: f32) {

        // Update AHRS algorithm
        self.FusionAhrsUpdate(gyroscope, accelerometer, Vec3::zero(), dt);

        // Zero heading during initialisation
        if self.initialising == true && ahrs.accelerationRejectionTimeout == false {
            self.FusionAhrsSetHeading(0.0);
        }
    }

    /**
     * @brief Updates the AHRS algorithm using the gyroscope, accelerometer, and
     * heading measurements.
     * @param ahrs AHRS algorithm structure.
     * @param gyroscope Gyroscope measurement in degrees per second.
     * @param accelerometer Accelerometer measurement in g.
     * @param heading Heading measurement in degrees.
     * @param deltaTime Delta time in seconds.
     */
    fn FusionAhrsUpdateExternalHeading(&self,
    gyroscope: Vec3,  accelerometer: Vec3, heading: f32, deltaTime: f32) {

    // Calculate roll
    let roll = atan2f(Q.y * Q.z + Q.w * Q.x, Q.w * Q.w - 0.5 + Q.z * Q.z);

    // Calculate magnetometer
    let sin_heading = sin(heading);
    let magnetometer = Vec3 {
        x: cos(heading),
        y: - 1.0 * cos(roll) * sin_heading,
        z: sin_heading * sin(roll),
    };

    // Update AHRS algorithm
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
    }

    /**
     * @brief Returns the quaternion describing the sensor relative to the Earth.
     * @param ahrs AHRS algorithm structure.
     * @return Quaternion describing the sensor relative to the Earth.
     */
    fn FusionAhrsGetQuaternion(&self) -> Quaternion {
        FusionQuaternionConjugate(self.quaternion)
    }

    /**
     * @brief Returns the linear acceleration measurement equal to the accelerometer
     * measurement with the 1 g of gravity removed.
     * @param ahrs AHRS algorithm structure.
     * @return Linear acceleration measurement.
     */
    fn FusionAhrsGetLinearAcceleration(&self) -> Vec3 {
        let Q = self.quaternion;
        let gravity = Vec3 {
            x: 2.0 * (Q.x * Q.z - Q.w * Q.y),
            y: 2.0 * (Q.w * Q.x + Q.y * Q.z),
            z: 2.0 * (Q.w * Q.w - 0.5 + Q.z * Q.z),
        }; // equal to 3rd column of rotation matrix representation scaled by the acceleration correction
        self.accelerometer - gravity
    }

    /**
     * @brief Returns the Earth acceleration measurement equal to accelerometer
     * measurement in the Earth coordinate frame with the 1 g of gravity removed.
     * @param ahrs AHRS algorithm structure.
     * @return Earth acceleration measurement.
     */
    fn FusionAhrsGetEarthAcceleration(&self) -> Vec3 {
        let Q = self.quaternion;
        let A = self.accelerometer;

        let qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
        let qwqx = Q.w * Q.x;
        let qwqy = Q.w * Q.y;
        let qwqz = Q.w * Q.z;
        let qxqy = Q.x * Q.y;
        let qxqz = Q.x * Q.z;
        let qyqz = Q.y * Q.z;
        Vec3 {
            x: 2.0 * ((qwqw - 0.5 + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
            y: 2.0 * ((qxqy + qwqz) * A.x + (qwqw - 0.5 + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
            z: (2.0 * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5 + Q.z * Q.z) * A.z)) - 1.0,
        } // transpose of a rotation matrix representation multiplied with the accelerometer, with 1 g subtracted
    }

    /**
     * @brief Returns the AHRS algorithm internal states.
     * @param ahrs AHRS algorithm structure.
     * @return AHRS algorithm internal states.
     */
    fn FusionAhrsGetInternalStates(&self, ) -> AhrsInternalStates {
        let rejectionTimeoutReciprocal = 1.0 / ahrs.settings.rejectionTimeout as f32;

        AhrsInternalStates {
            accelerationError: FusionRadiansToDegrees(asin(2.0 * FusionVectorMagnitude(ahrs.halfAccelerometerFeedback))),
            accelerometerIgnored: ahrs.accelerometerIgnored,
            accelerationRejectionTimer: self.accelerationRejectionTimer * rejectionTimeoutReciprocal,
            magneticError: FusionRadiansToDegrees(asin(2.0 * FusionVectorMagnitude(ahrs.halfMagnetometerFeedback))),
            magnetometerIgnored: ahrs.magnetometerIgnored,
            magneticRejectionTimer: self.magneticRejectionTimer * rejectionTimeoutReciprocal,
        }
    }

    /**
     * @brief Returns the AHRS algorithm flags.
     * @param ahrs AHRS algorithm structure.
     * @return AHRS algorithm flags.
     */
    fn FusionAhrsGetFlags(&self) -> AhrsFlags {
        let warningTimeout = self.settings.rejectionTimeout / 4;
        AhrsFlags {
            initialising: self.initialising,
            accelerationRejectionWarning: self.accelerationRejectionTimer > warningTimeout,
            accelerationRejectionTimeout: self.accelerationRejectionTimeout,
            magneticRejectionWarning: self.magneticRejectionTimer > warningTimeout,
            magneticRejectionTimeout: self.magneticRejectionTimeout,
        }
    }

    /**
     * @brief Sets the heading of the orientation measurement provided by the AHRS
     * algorithm.  This function can be used to reset drift in heading when the AHRS
     * algorithm is being used without a magnetometer.
     * @param ahrs AHRS algorithm structure.
     * @param heading Heading angle in degrees.
     */
    fn FusionAhrsSetHeading(&mut self, heading: f32) {
        let Q = self.quaternion;

        let inverseHeading = atan2f(Q.x * Q.y + Q.w * Q.z, Q.w * Q.w - 0.5 + Q.x * Q.x); // Euler angle of conjugate
        let halfInverseHeadingMinusOffset = 0.5 * (inverseHeading - heading);
        let inverseHeadingQuaternion = Quaternion {
            w: cos(halfInverseHeadingMinusOffset),
            x: 0.0,
            y: 0.0,
            z: -1.0 * sin(halfInverseHeadingMinusOffset),
        };
        self.quaternion = inverseHeadingQuaternion * self.quaternion;
    }

    /// Initialises the AHRS algorithm structure.
    /// ahrs AHRS algorithm structure.
    fn FusionAhrsInitialise(&mut self) {
        let settings = AhrsSettings {
            gain: 0.5,
            accelerationRejection: 90.0,
            magneticRejection: 90.0,
            rejectionTimeout: 0,
        };
        self.FusionAhrsSetSettings(&settings);
        self.FusionAhrsReset();
    }
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

/// AHRS algorithm flags.
struct AhrsFlags {
    pub initialising: bool,
    pub accelerationRejectionWarning: bool,
    pub accelerationRejectionTimeout: bool,
    pub magneticRejectionWarning: bool,
    pub magneticRejectionTimeout: bool,
}

// FusionAhrs.c:


// Initial gain used during the initialisation.
const INITIAL_GAIN: f32 = 10.0;

// Initialisation period in seconds.
const INITIALISATION_PERIOD: f32 = 3.0;



// FusionAxes.h:

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
    pub filter_coefficient: f32,
    pub timeout: u32,
    pub timer: u32,
    pub gyroscope_offset: Vec3
}

// FusionOffset.c

// Cutoff frequency in Hz.
const CUTOFF_FREQUENCY: f32 = 0.02;

// Timeout in seconds.
const TIMEOUT: u32 = 5; // todo type?

// Threshold in radians per second.
const THRESHOLD: f32 = 0.05236;


/// Initialises the gyroscope offset algorithm.
/// offset Gyroscope offset algorithm structure.
/// Sample rate in Hz.
fn FusionOffsetInitialise(offset: &mut Offset, sample_rate: u32) {
    offset.filter_coefficient = TAU * CUTOFF_FREQUENCY * (1. / sample_rate as f32);
    offset.timeout = TIMEOUT * sample_rate;
    offset.timer = 0;
    offset.gyroscope_offset = Vec3.zero();
}


/// Updates the gyroscope offset algorithm and returns the corrected
/// gyroscope measurement.
/// Gyroscope offset algorithm structure.
/// Gyroscope measurement in radians per second.
/// return Corrected gyroscope measurement in radians per second.
fn FusionOffsetUpdate(offset: &mut Offset, gyroscope: Vec3) -> Vec3 {

    // Subtract offset from gyroscope measurement
    let gyroscope = gyroscope - *offset.gyroscope_offset;

    // Reset timer if gyroscope not stationary
    if (gyroscope.axis.x).abs() > THRESHOLD || (gyroscope.axis.y).abs() > THRESHOLD || (gyroscope.axis.z).abs() > THRESHOLD {
        offset.timer = 0;
        return gyroscope;
    }

    // Increment timer while gyroscope stationary
    if offset.timer < offset.timeout {
        offset.timer += 1;
        return gyroscope;
    }

    // Adjust offset if timer has elapsed
    offset.gyroscope_offset = offset.gyroscope_offset + gyroscope * offset.filter_coefficient;
    return gyroscope;
}

// FusionCompass.c

/// Calculates the heading relative to magnetic north.
/// accelerometer Accelerometer measurement in any calibrated units.
/// magnetometer Magnetometer measurement in any calibrated units.
/// return Heading angle in radians
fn FusionCompassCalculateHeading(accelerometer: Vec3, magnetometer: Vec3) -> f32 {

    // Compute direction of magnetic west (Earth's y axis)
    let magnetic_west = FusionVectorNormalise(accelerometer.cross(magnetometer));

    // Compute direction of magnetic north (Earth's x axis)
    let magnetic_north = FusionVectorNormalise(magnetic_west.cross(accelerometer));

    // Calculate angular heading relative to magnetic north
    atan2f(magnetic_west.axis.x, magnetic_north.axis.x)
}