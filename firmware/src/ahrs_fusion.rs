//! AHRS fusion filter, for attaining an attitude platform from a 3-axis acceleratometer, gyro, and optionally
//! magnetometer. Based on a variant by Madgwick (By, not the?)
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
//! [AHRS](https://github.com/xioTechnologies/Fusion)
//! " The algorithm is based on the revised AHRS algorithm presented in chapter 7 of Madgwick's
//! PhD thesis. This is a different algorithm to the better-known initial AHRS algorithm presented in chapter 3,
//! commonly referred to as the Madgwick algorithm."
//!
//! https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf
//! https://github.com/chris1seto/OzarkRiver/tree/4channel/FlightComputerFirmware/Src/Madgwick
//! https://github.com/bjohnsonfl/Madgwick_Filter
//! https://github.com/chris1seto/OzarkRiver/blob/main/FlightComputerFirmware/Src/ImuAhrs.c

use core::f32::consts::TAU;

use num_traits::float::Float; // abs etc

use cmsis_dsp_sys::{arm_cos_f32, arm_sin_f32};

use super::{
    lin_alg::{Quaternion, Vec3},
    flight_ctrls::common::CtrlInputs,
};

use defmt::println;

// const G: f32 = 9.80665; // Gravity, in m/s^2

// Cutoff frequency in Hz.
const CUTOFF_FREQUENCY: f32 = 0.02;

// Timeout in seconds.
const TIMEOUT: u32 = 5;

// Threshold in radians per second.
const THRESHOLD: f32 = 0.05236;

// Initial gain used during the initialisation.
const INITIAL_GAIN: f32 = 10.0 * 360. / TAU;

// Initialisation period in seconds.
const INITIALISATION_PERIOD: f32 = 3.0;

fn cos(v: f32) -> f32 {
    unsafe { arm_cos_f32(v) }
}

fn sin(v: f32) -> f32 {
    unsafe { arm_sin_f32(v) }
}

/// AHRS algorithm settings. Field doc comments here are from [the readme](https://github.com/xioTechnologies/Fusion).
pub struct Settings {
    /// The algorithm calculates the orientation as the integration of the gyroscope
    /// summed with a feedback term. The feedback term is equal to the error in the current measurement
    /// of orientation as determined by the other sensors, multiplied by a gain. The algorithm therefore
    /// functions as a complementary filter that combines high-pass filtered gyroscope measurements
    /// with low-pass filtered measurements from other sensors with a corner frequency determined by
    /// the gain. A low gain will 'trust' the gyroscope more and so be more susceptible to drift. A
    /// high gain will increase the influence of other sensors and the errors that result from
    /// accelerations and magnetic distortions. A gain of zero will ignore the other sensors so that
    /// the measurement of orientation is determined by only the gyroscope.
    pub gain: f32,
    /// The acceleration rejection feature reduces the errors that result from the accelerations
    /// of linear and rotational motion. Acceleration rejection works by comparing the
    /// instantaneous measurement of inclination provided by the accelerometer with the
    /// current measurement of inclination of the algorithm output. If the angular difference
    /// between these two inclinations is greater than a threshold then the accelerometer will
    /// be ignored for this algorithm update. This is equivalent to a dynamic gain that
    /// deceases as accelerations increase.
    /// In radians.
    pub accel_rejection: f32,
    /// The magnetic rejection feature reduces the errors that result from temporary magnetic
    /// distortions. Magnetic rejection works using the same principle as acceleration
    /// rejection operating on the magnetometer instead of the accelerometer and by comparing
    /// the measurements of heading instead of inclination. A magnetic rejection timeout will
    /// not cause the algorithm to reinitialise. If a magnetic rejection timeout occurs then
    /// the heading of the algorithm output will be set to the instantaneous measurement of heading
    /// provided by the magnetometer.
    /// In radians.
    pub magnetic_rejection: f32,
    /// In seconds.
    pub rejection_timeout: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // todo: I'm not sure why the default gain of 0.5 seemed too low. Maybe it also assumes
            // todo degrees? This modified default seems to work.
            gain: 0.5 * 360. / TAU,
            accel_rejection: TAU / 4.,
            magnetic_rejection: TAU / 4.,
            // rejection_timeout: (5. * crate::IMU_UPDATE_RATE) as u32,
            rejection_timeout: 0,
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
    pub half_accelerometer_feedback: Vec3,
    pub half_magnetometer_feedback: Vec3,
    pub accelerometer_ignored: bool,
    pub accel_rejection_timer: u32,
    pub accel_rejection_timeout: bool,
    pub magnetometer_ignored: bool,
    pub mag_rejection_timer: u32,
    pub mag_rejection_timeout: bool,
    pub offset: Offset,
}

impl Ahrs {
    /// Initialises the AHRS algorithm structure.
    pub fn new(settings: &Settings, sample_rate: u32) -> Self {
        let mut result = Self::default();

        result.set_settings(settings);
        result.reset();
        result.offset = Offset::new(sample_rate);

        result
    }

    /// Resets the AHRS algorithm.  This is equivalent to reinitialising the
    /// algorithm while maintaining the current settings.
    /// param ahrs AHRS algorithm structure.
    fn reset(&mut self) {
        self.quaternion = Quaternion::new_identity();
        self.accelerometer = Vec3::zero();
        self.initialising = true;
        self.ramped_gain = INITIAL_GAIN;
        self.half_accelerometer_feedback = Vec3::zero();
        self.half_magnetometer_feedback = Vec3::zero();
        self.accelerometer_ignored = false;
        self.accel_rejection_timer = 0;
        self.accel_rejection_timeout = false;
        self.magnetometer_ignored = false;
        self.mag_rejection_timer = 0;
        self.mag_rejection_timeout = false;
    }

    /// Sets the AHRS algorithm settings.
    fn set_settings(&mut self, settings: &Settings) {
        self.settings.gain = settings.gain;

        if settings.accel_rejection < 0.0001 || settings.rejection_timeout == 0 {
            self.settings.accel_rejection = f32::MAX;
        } else {
            self.settings.accel_rejection = (0.5 * sin(settings.accel_rejection)).powi(2);
        }
        if settings.magnetic_rejection < 0.0001 || settings.rejection_timeout == 0 {
            self.settings.magnetic_rejection = f32::MAX;
        } else {
            self.settings.magnetic_rejection = (0.5 * sin(settings.magnetic_rejection)).powi(2);
        }

        self.settings.rejection_timeout = settings.rejection_timeout;
        if !self.initialising {
            self.ramped_gain = self.settings.gain;
        }
        self.ramped_gain_step = (INITIAL_GAIN - self.settings.gain) / INITIALISATION_PERIOD;
    }

    /// Updates the AHRS algorithm using the gyroscope, accelerometer, and
    /// magnetometer measurements.
    /// Gyroscope measurement in radians per second
    /// accelerometer Accelerometer measurement is in -g- m/s^2.
    /// magnetometer Magnetometer measurement is in arbitrary units.
    /// dt is in seconds.
    pub fn update(&mut self, gyro_data: Vec3, accel_data: Vec3, mag_data: Vec3, dt: f32) {
        let q = self.quaternion;

        // Store accelerometer
        self.accelerometer = accel_data;

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
            y: q.y * q.z + q.w * q.x,
            z: q.w * q.w - 0.5 + q.z * q.z,
        }; // third column of transposed rotation matrix scaled by 0.5

        // Calculate accelerometer feedback
        let mut half_accelerometer_feedback = Vec3::zero();
        self.accelerometer_ignored = true;
        if !accel_data.is_zero() {
            // Enter acceleration recovery state if acceleration rejection times out
            if self.accel_rejection_timer > self.settings.rejection_timeout {
                let quaternion = self.quaternion;
                self.reset();
                self.quaternion = quaternion;
                self.accel_rejection_timer = 0;
                self.accel_rejection_timeout = true;
            }

            // Calculate accelerometer feedback scaled by 0.5
            self.half_accelerometer_feedback = accel_data.to_normalized().cross(half_gravity);

            // Ignore accelerometer if acceleration distortion detected
            if self.initialising
                || self.half_accelerometer_feedback.magnitude_squared()
                    <= self.settings.accel_rejection
            {
                half_accelerometer_feedback = self.half_accelerometer_feedback;
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
        if !mag_data.is_zero() {
            // Set to compass heading if magnetic rejection times out
            self.mag_rejection_timeout = false;
            if self.mag_rejection_timer > self.settings.rejection_timeout {
                self.set_heading(compass_calc_heading(half_gravity, mag_data));
                self.mag_rejection_timer = 0;
                self.mag_rejection_timeout = true;
            }

            // Compute direction of west indicated by algorithm
            let half_west = Vec3 {
                x: q.x * q.y + q.w * q.z,
                y: q.w * q.w - 0.5 + q.y * q.y,
                z: q.y * q.z - q.w * q.x,
            }; // second column of transposed rotation matrix scaled by 0.5

            // Calculate magnetometer feedback scaled by 0.5
            self.half_magnetometer_feedback = half_gravity
                .cross(mag_data)
                .to_normalized()
                .cross(half_west);

            // Ignore magnetometer if magnetic distortion detected
            if self.initialising
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

        // Apply feedback to gyroscope
        let adjusted_half_gyro = gyro_data * 0.5
            + (half_accelerometer_feedback + half_magnetometer_feedback) * self.ramped_gain;

        // Integrate rate of change of quaternion
        self.quaternion = self.quaternion + (self.quaternion * (adjusted_half_gyro * dt));

        // Normalise quaternion
        self.quaternion = self.quaternion.to_normalized();
    }

    /// Updates the AHRS algorithm using the gyroscope and accelerometer
    /// measurements only.
    /// yroscope measurement is in radians per second.
    /// Accelerometer measurement is in m/s^2
    /// dt is in seconds.
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
    /// Gyroscope measurement is in radians per second.
    /// Accelerometer is measured in m/s^2
    /// Heading measurement is in radians per second.
    /// dt is in seconds.
    pub fn update_external_heading(
        &mut self,
        gyroscope: Vec3,
        accelerometer: Vec3,
        heading: f32,
        dt: f32,
    ) {
        // Calculate roll
        let q = self.quaternion;
        let roll = (q.w * q.x + q.y * q.z).atan2(0.5 - q.y * q.y - q.x * q.x);

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
    /// measurement with the 9.8m/s^2 of gravity removed.
    fn get_linear_accel(&self) -> Vec3 {
        let q = self.quaternion;

        // todo: Update this for m/s^2!!
        let gravity = Vec3 {
            x: 2.0 * (q.x * q.z - q.w * q.y),
            y: 2.0 * (q.y * q.z + q.w * q.x),
            z: 2.0 * (q.w * q.w - 0.5 + q.z * q.z),
        }; // third column of transposed rotation matrix

        self.accelerometer - gravity
    }

    /// Returns the Earth acceleration measurement equal to accelerometer
    /// measurement in the Earth coordinate frame with the 1 g of gravity removed.
    /// ahrs AHRS algorithm structure.
    pub fn get_earth_accel(&self) -> Vec3 {
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
        }
    }

    /// Returns the AHRS algorithm internal states.
    fn get_internal_states(&self) -> InternalStates {
        let (accel_rejection_timer, magnetic_rejection_timer) =
            if self.settings.rejection_timeout == 0 {
                (0., 0.)
            } else {
                (
                    (self.accel_rejection_timer / self.settings.rejection_timeout) as f32,
                    (self.mag_rejection_timer / self.settings.rejection_timeout) as f32,
                )
            };

        InternalStates {
            accel_error: fusion_asin(2.0 * self.half_accelerometer_feedback.magnitude()),
            accelerometer_ignored: self.accelerometer_ignored,
            accel_rejection_timer,
            magnetic_error: fusion_asin(2.0 * self.half_magnetometer_feedback.magnitude()),
            magnetometer_ignored: self.magnetometer_ignored,
            magnetic_rejection_timer,
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
    /// algorithm. This function can be used to reset drift in heading when the AHRS
    /// algorithm is being used without a magnetometer.
    fn set_heading(&mut self, heading: f32) {
        let q = self.quaternion;

        let yaw = (q.w * q.z + q.x * q.y).atan2(0.5 - q.y * q.y - q.z * q.z);
        let half_iyaw_minus_heading = 0.5 * (yaw - heading);
        let rotation = Quaternion {
            w: cos(half_iyaw_minus_heading),
            x: 0.0,
            y: 0.0,
            z: -1.0 * sin(half_iyaw_minus_heading),
        };
        self.quaternion = rotation * self.quaternion;
    }

    /// todo WIP
    /// Not part of official AHRS fusion algorithm
    fn apply_controls_quad(&mut self, controls: CtrlInputs) {

    }

    /// todo WIP
    /// Not part of official AHRS fusion algorithm
    fn apply_controls_fixed_wing(&mut self, controls: CtrlInputs) {

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

/// Axes alignment describing the sensor axes relative to the body axes.
/// For example, if the body X axis is aligned with the sensor Y axis and the
/// body Y axis is aligned with sensor X axis but pointing the opposite direction
/// then alignment is +Y-X+Z.
#[derive(Clone, Copy)]
enum AxesAlignment {
    PxPyPz, /* +X+Y+Z */
    PxNzPy, /* +X-Z+Y */
    PxNyNz, /* +X-Y-Z */
    PxPzNz, /* +X+Z-Y */
    NxPyNz, /* -X+Y-Z */
    NxPzPy, /* -X+Z+Y */
    NxNyPz, /* -X-Y+Z */
    NxNzNy, /* -X-Z-Y */
    PyNxPz, /* +Y-X+Z */
    PyNzNs, /* +Y-Z-X */
    PyPxNz, /* +Y+X-Z */
    PyPzPx, /* +Y+Z+X */
    NyPxPz, /* -Y+X+Z */
    NyNzPx, /* -Y-Z+X */
    NyNxNz, /* -Y-X-Z */
    NyPzNx, /* -Y+Z-X */
    PzPyNx, /* +Z+Y-X */
    PzPxPy, /* +Z+X+Y */
    PzNyPx, /* +Z-Y+X */
    PzNxNy, /* +Z-X-Y */
    NzPyPx, /* -Z+Y+X */
    NzNsPy, /* -Z-X+Y */
    NzNyNx, /* -Z-Y-X */
    NzPxNy, /* -Z+X-Y */
}

impl AxesAlignment {
    /// Swaps sensor axes for alignment with the body axes.
    /// Return Sensor axes aligned with the body axes.
    pub fn axes_swap(&mut self, sensor: Vec3) -> Vec3 {
        match self {
            Self::PxPyPz => sensor,
            Self::PxNzPy => Vec3 {
                x: sensor.x,
                y: -sensor.z,
                z: sensor.y,
            },
            Self::PxNyNz => Vec3 {
                x: sensor.x,
                y: -sensor.y,
                z: -sensor.z,
            },
            Self::PxPzNz => Vec3 {
                x: sensor.x,
                y: sensor.z,
                z: -sensor.y,
            },
            Self::NxPyNz => Vec3 {
                x: -sensor.x,
                y: sensor.y,
                z: -sensor.z,
            },
            Self::NxPzPy => Vec3 {
                x: -sensor.x,
                y: sensor.z,
                z: sensor.y,
            },
            Self::NxNyPz => Vec3 {
                x: -sensor.x,
                y: -sensor.y,
                z: sensor.z,
            },
            Self::NxNzNy => Vec3 {
                x: -sensor.x,
                y: -sensor.z,
                z: -sensor.y,
            },
            Self::PyNxPz => Vec3 {
                x: sensor.y,
                y: -sensor.x,
                z: sensor.z,
            },
            Self::PyNzNs => Vec3 {
                x: sensor.y,
                y: -sensor.z,
                z: -sensor.x,
            },
            Self::PyPxNz => Vec3 {
                x: sensor.y,
                y: sensor.x,
                z: -sensor.z,
            },
            Self::PyPzPx => Vec3 {
                x: sensor.y,
                y: sensor.z,
                z: sensor.x,
            },
            Self::NyPxPz => Vec3 {
                x: -sensor.y,
                y: sensor.x,
                z: sensor.z,
            },
            Self::NyNzPx => Vec3 {
                x: -sensor.y,
                y: -sensor.z,
                z: sensor.x,
            },
            Self::NyNxNz => Vec3 {
                x: -sensor.y,
                y: -sensor.x,
                z: -sensor.z,
            },
            Self::NyPzNx => Vec3 {
                x: -sensor.y,
                y: sensor.z,
                z: -sensor.x,
            },
            Self::PzPyNx => Vec3 {
                x: sensor.z,
                y: sensor.y,
                z: -sensor.x,
            },
            Self::PzPxPy => Vec3 {
                x: sensor.z,
                y: sensor.x,
                z: sensor.y,
            },
            Self::PzNyPx => Vec3 {
                x: sensor.z,
                y: -sensor.y,
                z: sensor.x,
            },
            Self::PzNxNy => Vec3 {
                x: sensor.z,
                y: -sensor.x,
                z: -sensor.y,
            },
            Self::NzPyPx => Vec3 {
                x: -sensor.z,
                y: sensor.y,
                z: sensor.x,
            },
            Self::NzNsPy => Vec3 {
                x: -sensor.z,
                y: -sensor.x,
                z: sensor.y,
            },
            Self::NzNyNx => Vec3 {
                x: -sensor.z,
                y: -sensor.y,
                z: -sensor.x,
            },
            Self::NzPxNy => Vec3 {
                x: -sensor.z,
                y: sensor.x,
                z: -sensor.y,
            },
        }
    }
}

// todo: Should this be with the calibration code in sensor_fusion?
// todo: Should you have a separate imu_calibration module?
/// Gyroscope offset algorithm structure. Structure members are used
/// internally and must not be accessed by the application.
#[derive(Default)]
pub struct Offset {
    filter_coefficient: f32,
    timeout: u32,
    timer: u32,
    gyroscope_offset: Vec3,
}

impl Offset {
    /// Initialises the gyroscope offset algorithm.
    /// Sample rate is in Hz.
    pub fn new(sample_rate: u32) -> Self {
        Self {
            filter_coefficient: TAU * CUTOFF_FREQUENCY * (1. / sample_rate as f32),
            timeout: TIMEOUT * sample_rate,
            timer: 0,
            gyroscope_offset: Vec3::zero(),
        }
    }

    /// Updates the gyroscope offset algorithm and returns the corrected
    /// gyroscope measurement.
    /// Gyroscope measurement is in radians per second.
    /// return Corrected gyroscope measurement in radians per second.
    pub fn update(&mut self, gyroscope: Vec3) -> Vec3 {
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
        gyroscope
    }
}

/// Calculates the heading relative to magnetic north.
/// Accelerometer measurement is in any calibrated units.
/// Magnetometer measurement is in any calibrated units.
/// return Heading angle in radians
pub fn compass_calc_heading(accelerometer: Vec3, magnetometer: Vec3) -> f32 {
    // Compute direction of magnetic west (Earth's y axis)
    let magnetic_west = accelerometer.cross(magnetometer).to_normalized();

    // Compute direction of magnetic north (Earth's x axis)
    let magnetic_north = magnetic_west.cross(accelerometer).to_normalized();

    // Calculate angular heading relative to magnetic north
    magnetic_west.x.atan2(magnetic_north.x)
}

/// Returns the arc sine of the value. Presumably used as a workaround for certain cases.
/// (pub for use in lin_alg)
pub fn fusion_asin(value: f32) -> f32 {
    if value <= -1.0 {
        return TAU / -4.0;
    }
    if value >= 1.0 {
        return TAU / 4.0;
    }
    value.asin()
}
