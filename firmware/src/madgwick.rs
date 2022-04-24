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
//! This library is a translation of the original algorithm below:
//! [Original algorithm, by Seb Madgwick, in C](https://github.com/xioTechnologies/Fusion)
//!
//! https://github.com/chris1seto/OzarkRiver/tree/4channel/FlightComputerFirmware/Src/Madgwick
//! https://github.com/bjohnsonfl/Madgwick_Filter
//! https://github.com/chris1seto/OzarkRiver/blob/main/FlightComputerFirmware/Src/ImuAhrs.c

use crate::sensor_fusion::ImuReadings;

use core::f32::consts::TAU;

use num_traits::float::Float; // abs etc

use cmsis_dsp_sys::{arm_cos_f32, arm_sin_f32};

use super::lin_alg::{EulerAngle, Mat3, Quaternion, Vec3};

fn cos(v: f32) -> f32 {
    unsafe { arm_cos_f32(v) }
}

fn sin(v: f32) -> f32 {
    unsafe { arm_sin_f32(v) }
}

#[derive(Default)]
/// AHRS algorithm settings.
pub struct Settings {
    pub gain: f32,
    pub accel_rejection: f32,
    pub magnetic_rejection: f32,
    pub rejection_timeout: u32,
}

pub struct AhrsCalibration {
        pub gyroscopeMisalignment: Mat3,
        pub gyroscopeSensitivity: Vec3,
        pub gyroscopeOffset: Vec3,
        pub accelerometerMisalignment: Mat3,
        pub accelerometerSensitivity: Vec3,
        pub accelerometerOffset: Vec3,
        pub softIronMatrix: Mat3,
        pub hardIronOffset: Vec3,
}

impl Default for AhrsCalibration {
    fn default() -> Self {
        Self {
            //     const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            //     const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
            //     const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
            //     const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            //     const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
            //     const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
            //     const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            //     const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};
        }
    }
}

#[derive(Default)]
/// AHRS algorithm structure.  Structure members are used internally and
/// must not be accessed by the application.
pub struct Ahrs {
    pub settings: Settings,
    /// The quaternion describing the sensor relative to the Earth.
    pub quaternion: Quaternion,
    pub accelerometer: Vec3,
    pub initialising: bool,
    pub ramped_gain: f32,
    pub ramped_gain_step: f32,
    pub half_accelerometer_feedfwd: Vec3,
    pub half_magnetometer_feedback: Vec3,
    pub accelerometer_ignored: bool,
    pub accel_rejection_timer: u32,
    pub accel_rejection_timeout: bool,
    pub magnetometer_ignored: bool,
    pub mag_rejection_timer: u32,
    pub mag_rejection_timeout: bool,
    pub offset: AhrsOffset,
    pub calibration: AhrsCalibration,
}

impl Ahrs {
    /// Resets the AHRS algorithm.  This is equivalent to reinitialising the
    /// algorithm while maintaining the current settings.
    /// param ahrs AHRS algorithm structure.
    fn reset(&mut self) {
        self.quaternion = Quaternion::new_identity();
        self.accelerometer = Vec3::zero();
        self.initialising = true;
        self.ramped_gain = INITIAL_GAIN;
        self.half_accelerometer_feedfwd = Vec3::zero();
        self.half_magnetometer_feedback = Vec3::zero();
        self.accelerometer_ignored = false;
        self.accel_rejection_timer = 0;
        self.accel_rejection_timeout = false;
        self.magnetometer_ignored = false;
        self.mag_rejection_timer = 0;
        self.mag_rejection_timeout = false;
    }

    /// brief Sets the AHRS algorithm settings.
    /// param ahrs AHRS algorithm structure.
    /// aram settings Settings.
    pub fn set_settings(&mut self, settings: &Settings) {
        self.settings.gain = settings.gain;
        if settings.accel_rejection == 0.0 || settings.rejection_timeout == 0 {
            self.settings.accel_rejection = f32::MAX;
        } else {
            self.settings.accel_rejection = (0.5 * sin(settings.accel_rejection)).powi(2);
        }
        if (settings.magnetic_rejection == 0.0) || (settings.rejection_timeout == 0) {
            self.settings.magnetic_rejection = f32::MAX;
        } else {
            self.settings.magnetic_rejection = (0.5 * sin(settings.magnetic_rejection)).powi(2);
        }
        self.settings.rejection_timeout = settings.rejection_timeout;
        if self.initialising == false {
            self.ramped_gain = self.settings.gain;
        }
        self.ramped_gain_step = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD;
    }

    // todo: Does accel need to be in G, or can we use m/s^2

    /// Updates the AHRS algorithm using the gyroscope, accelerometer, and
    /// magnetometer measurements.
    /// Gyroscope measurement in radians per second
    /// accelerometer Accelerometer measurement in -g- m/s^2.
    /// magnetometer Magnetometer measurement in arbitrary units.
    /// dt in seconds.
    pub fn update(&mut self, gyroscope: Vec3, accelerometer: Vec3, magnetometer: Vec3, dt: f32) {
        let q = self.quaternion;

        // Store accelerometer
        self.accelerometer = accelerometer;

        // Ramp down gain during initialisation
        if self.initialising {
            self.ramped_gain -= self.ramped_gain_step * dt;
            if self.ramped_gain < self.settings.gain {
                self.ramped_gain = self.settings.gain;
                self.initialising = false;
                self.accel_rejection_timeout = false;
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
        self.accelerometer_ignored = true;
        if !accelerometer.is_zero() {
            // Enter acceleration recovery state if acceleration rejection times out
            if self.accel_rejection_timer >= self.settings.rejection_timeout {
                let quaternion = self.quaternion;
                self.reset();
                self.quaternion = quaternion;
                self.accel_rejection_timer = 0;
                self.accel_rejection_timeout = true;
            }

            // Calculate accelerometer feedback scaled by 0.5
            self.half_accelerometer_feedfwd =
                accelerometer.to_normalized().cross(half_gravity);

            // Ignore accelerometer if acceleration distortion detected
            if self.initialising == true
                || self.half_accelerometer_feedfwd.magnitude_squared()
                    <= self.settings.accel_rejection
            {
                half_accelerometer_feedback = self.half_accelerometer_feedfwd;
                self.accelerometer_ignored = false;
                self.accel_rejection_timer -= if self.accel_rejection_timer >= 10 {
                    10
                } else {
                    0
                };
            } else {
                self.accel_rejection_timer += 1;
            }
        }

        // Calculate magnetometer feedback
        let mut half_magnetometer_feedback = Vec3::zero();
        self.magnetometer_ignored = true;
        if !magnetometer.is_zero() {
            // Set to compass heading if magnetic rejection times out
            self.mag_rejection_timeout = false;
            if self.mag_rejection_timer >= self.settings.rejection_timeout {
                self.set_heading(compass_calc_heading(half_gravity, magnetometer));
                self.mag_rejection_timer = 0;
                self.mag_rejection_timeout = true;
            }

            // Compute direction of west indicated by algorithm
            let half_west = Vec3 {
                x: q.x * q.y + q.w * q.z,
                y: q.w * q.w - 0.5 + q.y * q.y,
                z: q.y * q.z - q.w * q.x,
            }; // equal to 2nd column of rotation matrix representation scaled by 0.5

            // Calculate magnetometer feedback scaled by 0.5
            self.half_magnetometer_feedback = half_gravity.cross(magnetometer).to_normalized().cross(half_west);

            // Ignore magnetometer if magnetic distortion detected
            if self.initialising == true
                || self.half_magnetometer_feedback.magnitude_squared()
                    <= self.settings.magnetic_rejection
            {
                half_magnetometer_feedback = self.half_magnetometer_feedback;
                self.magnetometer_ignored = false;
                self.mag_rejection_timer -= if self.mag_rejection_timer >= 10 {
                    10
                } else {
                    0
                };
            } else {
                self.mag_rejection_timer += 1;
            }
        }

        // Convert gyroscope to radians per second scaled by 0.5
        let half_gyro = gyroscope * 0.5;

        // Apply feedback to gyroscope
        let adjusted_half_gyro = half_gyro
            + (half_accelerometer_feedback + half_magnetometer_feedback) * self.ramped_gain;

        // Integrate rate of change of quaternion
        self.quaternion = self.quaternion + self.quaternion * (adjusted_half_gyro * dt);

        // Normalise quaternion
        self.quaternion = self.quaternion.to_normalized();
    }

    /// Updates the AHRS algorithm using the gyroscope and accelerometer
    /// measurements only.
    /// ahrs AHRS algorithm structure.
    /// gyroscope Gyroscope measurement in degrees per second.
    /// accelerometer Accelerometer measurement in g.
    /// deltaTime dt in seconds.
    pub fn update_no_magnetometer(&mut self, gyroscope: Vec3, accelerometer: Vec3, dt: f32) {
        // Update AHRS algorithm
        self.update(gyroscope, accelerometer, Vec3::zero(), dt);

        // Zero heading during initialisation
        if self.initialising && !self.accel_rejection_timeout {
            self.set_heading(0.0);
        }
    }

    /// Updates the AHRS algorithm using the gyroscope, accelerometer, and
    /// heading measurements.
    /// ahrs AHRS algorithm structure.
    /// gyroscope Gyroscope measurement in degrees per second.
    /// accelerometer Accelerometer measurement in g.
    /// heading Heading measurement in degrees.
    /// dt in seconds.
    pub fn update_external_heading(
        &mut self,
        gyroscope: Vec3,
        accelerometer: Vec3,
        heading: f32,
        dt: f32,
    ) {
        // Calculate roll
        let roll =(
            self.quaternion.y * self.quaternion.z + self.quaternion.w * self.quaternion.x
        ).atan2(self.quaternion.w * self.quaternion.w - 0.5 + self.quaternion.z * self.quaternion.z);

        // Calculate magnetometer
        let sin_heading = sin(heading);
        let magnetometer = Vec3 {
            x: cos(heading),
            y: -1.0 * cos(roll) * sin_heading,
            z: sin_heading * sin(roll),
        };

        // Update AHRS algorithm
        self.update(gyroscope, accelerometer, magnetometer, dt);
    }

    /// Returns the linear acceleration measurement equal to the accelerometer
    /// measurement with the 1 g of gravity removed.
    /// ahrs AHRS algorithm structure.
    fn get_linear_accel(&self) -> Vec3 {
        let q = self.quaternion;
        let gravity = Vec3 {
            x: 2.0 * (q.x * q.z - q.w * q.y),
            y: 2.0 * (q.w * q.x + q.y * q.z),
            z: 2.0 * (q.w * q.w - 0.5 + q.z * q.z),
        }; // equal to 3rd column of rotation matrix representation scaled by the acceleration correction
        self.accelerometer - gravity
    }

    /// Returns the Earth acceleration measurement equal to accelerometer
    /// measurement in the Earth coordinate frame with the 1 g of gravity removed.
    /// ahrs AHRS algorithm structure.
    fn get_earth_accel(&self) -> Vec3 {
        let q = self.quaternion;
        let a = self.accelerometer;

        let qwqw = q.w * q.w; // calculate common terms to avoid repeated operations
        let qwqx = q.w * q.x;
        let qwqy = q.w * q.y;
        let qwqz = q.w * q.z;
        let qxqy = q.x * q.y;
        let qxqz = q.x * q.z;
        let qyqz = q.y * q.z;
        Vec3 {
            x: 2.0 * ((qwqw - 0.5 + q.x * q.x) * a.x + (qxqy - qwqz) * a.y + (qxqz + qwqy) * a.z),
            y: 2.0 * ((qxqy + qwqz) * a.x + (qwqw - 0.5 + q.y * q.y) * a.y + (qyqz - qwqx) * a.z),
            z: (2.0 * ((qxqz - qwqy) * a.x + (qyqz + qwqx) * a.y + (qwqw - 0.5 + q.z * q.z) * a.z))
                - 1.0,
        } // transpose of a rotation matrix representation multiplied with the accelerometer, with 1 g subtracted
    }

    /// Returns the AHRS algorithm internal states.
    fn get_internal_states(&self) -> InternalStates {
        let rejection_timeout_interval = 1.0 / self.settings.rejection_timeout as f32;

        InternalStates {
            accel_error: (2.0 * self.half_accelerometer_feedfwd.magnitude()).asin(),
            accelerometer_ignored: self.accelerometer_ignored,
            accel_rejection_timer: self.accel_rejection_timer as f32 * rejection_timeout_interval,
            magnetic_error: (2.0 * self.half_magnetometer_feedback.magnitude()).asin(),
            magnetometer_ignored: self.magnetometer_ignored,
            magnetic_rejection_timer: self.mag_rejection_timer as f32 * rejection_timeout_interval,
        }
    }

    /// Returns the AHRS algorithm flags.
    fn get_flags(&self) -> Flags {
        let warning_timeout = self.settings.rejection_timeout / 4;

        Flags {
            initialising: self.initialising,
            accel_rejection_warning: self.accel_rejection_timer > warning_timeout,
            accel_rejection_timeout: self.accel_rejection_timeout,
            mag_rejection_warning: self.mag_rejection_timer > warning_timeout,
            mag_rejection_timeout: self.mag_rejection_timeout,
        }
    }

    /// Sets the heading of the orientation measurement provided by the AHRS
    /// algorithm.  This function can be used to reset drift in heading when the AHRS
    /// algorithm is being used without a magnetometer.
    fn set_heading(&mut self, heading: f32) {
        let q = self.quaternion;

        let inverse_heading = (q.x * q.y + q.w * q.z).atan2(q.w * q.w - 0.5 + q.x * q.x); // Euler angle of conjugate
        let half_inv_hdg_minus_offset = 0.5 * (inverse_heading - heading);
        let inv_hdg_quat = Quaternion {
            w: cos(half_inv_hdg_minus_offset),
            x: 0.0,
            y: 0.0,
            z: -1.0 * sin(half_inv_hdg_minus_offset),
        };
        self.quaternion = inv_hdg_quat * self.quaternion;
    }

    /// Initialises the AHRS algorithm structure.
    /// ahrs AHRS algorithm structure.
    fn initialize(&mut self) {
        let settings = Settings {
            gain: 0.5,
            accel_rejection: 90.0,
            magnetic_rejection: 90.0,
            rejection_timeout: 0,
        };
        self.set_settings(&settings);
        self.reset();
    }
}

/// AHRS algorithm internal states.
struct InternalStates {
    pub accel_error: f32,
    pub accelerometer_ignored: bool,
    pub accel_rejection_timer: f32,
    pub magnetic_error: f32,
    pub magnetometer_ignored: bool,
    pub magnetic_rejection_timer: f32,
}

/// AHRS algorithm flags.
struct Flags {
    pub initialising: bool,
    pub accel_rejection_warning: bool,
    pub accel_rejection_timeout: bool,
    pub mag_rejection_warning: bool,
    pub mag_rejection_timeout: bool,
}

// Initial gain used during the initialisation.
const INITIAL_GAIN: f32 = 10.0;

// Initialisation period in seconds.
const INITIALISATION_PERIOD: f32 = 3.0;

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
fn axes_swap(sensor: Vec3, alignment: AxesAlignment) -> Vec3 {
    match alignment {
        AxesAlignment::PXPYPZ => sensor,
        AxesAlignment::PXNZPY => Vec3 {
            x: sensor.x,
            y: -sensor.z,
            z: sensor.y,
        },
        AxesAlignment::PXNYNZ => Vec3 {
            x: sensor.x,
            y: -sensor.y,
            z: -sensor.z,
        },
        AxesAlignment::PXPZNY => Vec3 {
            x: sensor.x,
            y: sensor.z,
            z: -sensor.y,
        },
        AxesAlignment::NXPYNZ => Vec3 {
            x: -sensor.x,
            y: sensor.y,
            z: -sensor.z,
        },
        AxesAlignment::NXPZPY => Vec3 {
            x: -sensor.x,
            y: sensor.z,
            z: sensor.y,
        },
        AxesAlignment::NXNYPZ => Vec3 {
            x: -sensor.x,
            y: -sensor.y,
            z: sensor.z,
        },
        AxesAlignment::NXNZNY => Vec3 {
            x: -sensor.x,
            y: -sensor.z,
            z: -sensor.y,
        },
        AxesAlignment::PYNXPZ => Vec3 {
            x: sensor.y,
            y: -sensor.x,
            z: sensor.z,
        },
        AxesAlignment::PYNZNX => Vec3 {
            x: sensor.y,
            y: -sensor.z,
            z: -sensor.x,
        },
        AxesAlignment::PYPXNZ => Vec3 {
            x: sensor.y,
            y: sensor.x,
            z: -sensor.z,
        },
        AxesAlignment::PYPZPX => Vec3 {
            x: sensor.y,
            y: sensor.z,
            z: sensor.x,
        },
        AxesAlignment::NYPXPZ => Vec3 {
            x: -sensor.y,
            y: sensor.x,
            z: sensor.z,
        },
        AxesAlignment::NYNZPX => Vec3 {
            x: -sensor.y,
            y: -sensor.z,
            z: sensor.x,
        },
        AxesAlignment::NYNXNZ => Vec3 {
            x: -sensor.y,
            y: -sensor.x,
            z: -sensor.z,
        },
        AxesAlignment::NYPZNX => Vec3 {
            x: -sensor.y,
            y: sensor.z,
            z: -sensor.x,
        },
        AxesAlignment::PZPYNX => Vec3 {
            x: sensor.z,
            y: sensor.y,
            z: -sensor.x,
        },
        AxesAlignment::PZPXPY => Vec3 {
            x: sensor.z,
            y: sensor.x,
            z: sensor.y,
        },
        AxesAlignment::PZNYPX => Vec3 {
            x: sensor.z,
            y: -sensor.y,
            z: sensor.x,
        },
        AxesAlignment::PZNXNY => Vec3 {
            x: sensor.z,
            y: -sensor.x,
            z: -sensor.y,
        },
        AxesAlignment::NZPYPX => Vec3 {
            x: -sensor.z,
            y: sensor.y,
            z: sensor.x,
        },
        AxesAlignment::NZNXPY => Vec3 {
            x: -sensor.z,
            y: -sensor.x,
            z: sensor.y,
        },
        AxesAlignment::NZNYNX => Vec3 {
            x: -sensor.z,
            y: -sensor.y,
            z: -sensor.x,
        },
        AxesAlignment::NZPXNY => Vec3 {
            x: -sensor.z,
            y: sensor.x,
            z: -sensor.y,
        },
    }
}

// FusionOffset.h:

/// Gyroscope offset algorithm structure.  Structure members are used
/// internally and must not be accessed by the application.
#[derive(Default)]
struct Offset {
    pub filter_coefficient: f32,
    pub timeout: u32,
    pub timer: u32,
    pub gyroscope_offset: Vec3,
}

impl Offset {
    /// Initialises the gyroscope offset algorithm.
    /// offset Gyroscope offset algorithm structure.
    /// Sample rate in Hz.
    fn initialize(&mut self, sample_rate: u32) {
        self.filter_coefficient = TAU * CUTOFF_FREQUENCY * (1. / sample_rate as f32);
        self.timeout = TIMEOUT * sample_rate;
        self.timer = 0;
        self.gyroscope_offset = Vec3::zero();
    }

    /// Updates the gyroscope offset algorithm and returns the corrected
    /// gyroscope measurement.
    /// Gyroscope offset algorithm structure.
    /// Gyroscope measurement in radians per second.
    /// return Corrected gyroscope measurement in radians per second.
    fn update(&mut self, gyroscope: Vec3) -> Vec3 {
        // Subtract offset from gyroscope measurement
        let gyroscope = gyroscope - self.gyroscope_offset;

        // Reset timer if gyroscope not stationary
        if (gyroscope.x).abs() > THRESHOLD
            || (gyroscope.y).abs() > THRESHOLD
            || (gyroscope.z).abs() > THRESHOLD
        {
            self.timer = 0;
            return gyroscope;
        }

        // Increment timer while gyroscope stationary
        if self.timer < self.timeout {
            self.timer += 1;
            return gyroscope;
        }

        // Adjust offset if timer has elapsed
        self.gyroscope_offset = self.gyroscope_offset + gyroscope * self.filter_coefficient;
        return gyroscope;
    }
}

// Cutoff frequency in Hz.
const CUTOFF_FREQUENCY: f32 = 0.02;

// Timeout in seconds.
const TIMEOUT: u32 = 5;

// Threshold in radians per second.
const THRESHOLD: f32 = 0.05236;

/// Calculates the heading relative to magnetic north.
/// accelerometer Accelerometer measurement in any calibrated units.
/// magnetometer Magnetometer measurement in any calibrated units.
/// return Heading angle in radians
fn compass_calc_heading(accelerometer: Vec3, magnetometer: Vec3) -> f32 {
    // Compute direction of magnetic west (Earth's y axis)
    let magnetic_west = accelerometer.cross(magnetometer).to_normalized();

    // Compute direction of magnetic north (Earth's x axis)
    let magnetic_north = magnetic_west.cross(accelerometer).to_normalized();

    // Calculate angular heading relative to magnetic north
    magnetic_west.x.atan2(magnetic_north.x)
}
