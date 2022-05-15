// #![allow(uncommon_codepoints)] // eg ϕ

//! This module contains code related to sensor fusion, eg using an extended kalman filter
//! to combine inputs from multiple sensors. It also includes code for interpreting, integrating,
//! and taking the derivatives of sensor readings. Code here is device-agnostic.
//!
//! Some IMUs can integrate with a magnetometer and do some sensor fusion; we use
//! software since it's more general, flexible, and device-agnostic.
//!
//! From HyperShield: "Unscented will not offer any improvement over EKF. The reason for this is
//! that your quadrotor will mainly stay near hover (because of the assumption that gravity
//! will be the dominant acceleration) so the system will appear quasi-linear. If you want to go
//! agile flight then UKF might offer improvements, but then you can't rely on the gravity assumption.
//! There are attitude estimators like the hua filter that circumvents this (it estimates the
//! acceleration in the inertial frame and uses that for prediction instead assuming it's equal
//! to the gravity vector)."
//!
//! Python guide: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
//!
//! [Youtube video: Phil's Lab](https://www.youtube.com/watch?v=hQUkiC5o0JI)

// todo: Try this : https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py

use cmsis_dsp_sys::{arm_cos_f32 as cos, arm_sin_f32 as sin};

use defmt::println;

use stm32_hal2::{
    dma::{Dma, DmaChannel},
    gpio::Pin,
    pac::{DMA1, SPI1},
    spi::Spi,
};

use crate::{
    flight_ctrls::Params,
    imu,
    lin_alg::{Mat3, Vec3},
    madgwick::{self, Ahrs},
};

// C file with impl of EKF for quaternion rotation:
// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/EKF.h
// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/updateEKFQuatAtt.m

// Rotors: Quaternion generalization?
// https://marctenbosch.com/quaternions/

// Re quaternions, euler angles, and error state;
// Euler angles are mostly used to synthesize controllers such as PIDs.
// When it comes to state/attitude estimation, quaternions are more common.
// However you can't use a quaternion directly as a state in your EKF (technically you can with
// some care) since the the unit constraint will be violated. You can instead use an
// error-state EKF
// http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf

const G: f32 = 9.8; // m/s

// IMU readings buffer. 3 accelerometer, and 3 gyro measurements; 2 bytes each. 0-padded on the left,
// since that's where we pass the register
// in the write buffer.
pub static mut IMU_READINGS: [u8; 13] = [0; 13];

fn tan(val: f32) -> f32 {
    unsafe { sin(val) / cos(val) }
}

// todo: Consider an `imu_shared` module.
/// Represents sensor readings from a 6-axis accelerometer + gyro. Similar to
/// `ParamsInst`.
#[derive(Default)]
pub struct ImuReadings {
    pub a_x: f32,
    pub a_y: f32,
    pub a_z: f32,
    pub v_pitch: f32,
    pub v_roll: f32,
    pub v_yaw: f32,
}

impl ImuReadings {
    /// We use this to assemble readings from the DMA buffer.
    pub fn from_buffer(buf: &[u8]) -> Self {
        // todo: Note: this mapping may be different for diff IMUs, eg if they use a different reading register ordering.
        // todo: Currently hard-set for ICM426xx.

        // Ignore byte 0; it's for the first reg passed during the `write` transfer.
        Self {
            a_x: imu::interpret_accel(i16::from_be_bytes([buf[1], buf[2]])),
            a_y: imu::interpret_accel(i16::from_be_bytes([buf[3], buf[4]])),
            a_z: imu::interpret_accel(i16::from_be_bytes([buf[5], buf[6]])),
            // Positive pitch nose down
            v_pitch: -imu::interpret_gyro(i16::from_be_bytes([buf[7], buf[8]])),
            // Positive roll: left wing up
            v_roll: imu::interpret_gyro(i16::from_be_bytes([buf[9], buf[10]])),
            // Positive yaw: CW rotation.
            v_yaw: -imu::interpret_gyro(i16::from_be_bytes([buf[11], buf[12]])),
        }
    }
}

/// Calibrate the IMU, by taking a series of series while on a level surface.
pub fn calibrate() -> AhrsCalibration {
    // todo: average? lowpass?
    Default::default()
}

/// Update and get the attitude from the AHRS.
pub fn update_get_attitude(ahrs: &mut Ahrs, params: &mut Params) {
    /// Gyro measurements - not really a vector.
    let mut gyro_data = Vec3 {
        x: params.v_pitch,
        y: params.v_roll,
        z: params.v_yaw,
    };
    let mut accel_data = Vec3 {
        x: params.a_x,
        y: params.a_y,
        z: params.a_z,
    };

    // todo: Separate calibration from AHRS/madgwick.

    // Apply calibration
    // todo: Come back to this.
    // gyro_data = madgwick::apply_cal_inertial(
    //     gyro_data,
    //     ahrs.calibration.gyro_misalignment.clone(),
    //     ahrs.calibration.gyro_sensitivity,
    //     ahrs.calibration.gyro_offset,
    // );
    // accel_data = madgwick::apply_cal_inertial(
    //     accel_data,
    //     ahrs.calibration.accel_misalignment.clone(),
    //     ahrs.calibration.accel_sensitivity,
    //     ahrs.calibration.accel_offset,
    // );

    // todo: Once you add mag.
    // let magnetometer = madgwick::apply_cal_magnetic(magnetometer, softIronMatrix, hardIronOffset);

    // Update gyroscope offset correction algorithm
    let gyro_data = ahrs.offset.update(gyro_data);

    // todo: Can we use the hard-set 8kHz IMU-spec DT, or do we need to measure?
    ahrs.update_no_magnetometer(gyro_data, accel_data, crate::DT_IMU);

    let att_euler = ahrs.quaternion.to_euler();
    // let att_earth = ahrs.get_earth_accel();

    // Update params with our calibrated gryo and accel data, in addition to attitude.

    params.v_roll = gyro_data.x;
    params.v_pitch = gyro_data.y;
    params.v_yaw = gyro_data.z;

    params.a_x = accel_data.x;
    params.a_y = accel_data.y;
    params.a_z = accel_data.z;

    // Note: Swapped pitch/roll swapped due to how the madgwick filter or quaternion -> euler angle
    // is calculated.
    params.s_roll = att_euler.pitch;
    params.s_pitch = att_euler.roll;
    params.s_yaw = att_euler.yaw;

    // params.s_roll = att_earth.x;
    // params.s_pitch = att_earth.y;
    // params.s_yaw = att_earth.z;
}

// This calibration functionality is from [AHRS](https://github.com/xioTechnologies/Fusion)

pub struct AhrsCalibration {
    pub gyro_misalignment: Mat3,
    pub gyro_sensitivity: Vec3,
    pub gyro_offset: Vec3,
    pub accel_misalignment: Mat3,
    pub accel_sensitivity: Vec3,
    pub accel_offset: Vec3,
    pub soft_iron_matrix: Mat3,
    pub hard_iron_offset: Vec3,
}

impl Default for AhrsCalibration {
    fn default() -> Self {
        Self {
            gyro_misalignment: Mat3 {
                data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            gyro_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            gyro_offset: Vec3::new(0.0, 0.0, 0.0),
            accel_misalignment: Mat3 {
                data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            accel_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            accel_offset: Vec3::new(0.0, 0.0, 0.0),
            soft_iron_matrix: Mat3 {
                data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            hard_iron_offset: Vec3::new(0.0, 0.0, 0.0),
        }
    }
}

/// Gyroscope and accelerometer calibration model. Returns calibrated measurement.
pub fn apply_cal_inertial(
    uncalibrated: Vec3,
    misalignment: Mat3,
    sensitivity: Vec3,
    offset: Vec3,
) -> Vec3 {
    misalignment * (uncalibrated - offset).hadamard_product(sensitivity)
}

/// Magnetometer calibration model. Returns calibrated measurement.
pub fn apply_cal_magnetic(
    uncalibrated: Vec3,
    soft_iron_matrix: Mat3,
    hard_iron_offset: Vec3,
) -> Vec3 {
    soft_iron_matrix * uncalibrated - hard_iron_offset
}

/// Read all 3 measurements, by commanding a DMA transfer. The transfer is closed, and readings
/// are processed in the Transfer Complete ISR.
pub fn read_imu_dma(starting_addr: u8, spi: &mut Spi<SPI1>, cs: &mut Pin, dma: &mut Dma<DMA1>) {
    // First byte is the first data reg, per this IMU's. Remaining bytes are empty, while
    // the MISO line transmits readings.
    let write_buf = [starting_addr, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    cs.set_low();

    unsafe {
        spi.transfer_dma(
            &write_buf,
            &mut IMU_READINGS,
            DmaChannel::C1,
            DmaChannel::C2,
            Default::default(),
            Default::default(),
            dma,
        );
    }
}

/// Kalman filter predict step
/// - Use dynamical model to update state estimates.  x^_n+1 = x^_n + T f(x^_n, u)
/// - Update covaraince (P) P will always increase with this step.
fn _predict() {
    // todo: Switch away from euler angles, eg to quaternions.

    // Update nonlinear state-transition fn.
    // x^_n+1 = x^_n + T_gyr dot f(x^, u)
    // X_n = [[ϕ_n], [θ_n]] (state estimate at sample n). f(x^, u) = [[ϕdot], [θdot]] (euler rates)
}

/// Kalman filter update step
/// - Use measurements from sensors to correct predictions: x^ = x^ + K(y - h(x^, u))
/// - Update error covariance P (P should decrease in this step)
fn _update() {}

/// Estimate attitude, based on IMU data of accelerations and roll rates.
pub fn _estimate_attitude(readings: &ImuReadings) -> Params {
    // Euler angle conventions: θ = pitch. phi = roll. ^ indicates an estimate

    // todo: Put useful params here.
    // filter_ = signal.iirfilter(1, 60, btype="lowpass", ftype="bessel", output="sos", fs=32_000)
    // coeffs = []
    // for row in filter_:
    //     coeffs.extend([row[0] / row[3], row[1] / row[3], row[2] / row[3], -row[4] / row[3], -row[5] / row[3]])
    // let coeffs = [0.00585605892206321, 0.00585605892206321, 0.0, 0.9882878821558736, -0.0];

    // Values at rest:
    // a_x = g * sin(θ)
    // a_y = -g * cos(θ) * sin(ϕ)
    // a_z = -g * cos(θ) * cos(ϕ)

    // todo: Apply your lowpass, or confirm hardware lowpass is set up.

    // todo: time-varying bias?

    // Pitch and roll from accel readings alone, using gravity.
    // let ϕ_est = (ay / az).atan();
    // let θ_est = (ax / G).asin();

    // hat = est

    // https://www.youtube.com/watch?v=RZd6XDx5VXo

    // Pitch and roll angles from gyroscopes alone.
    let mut θ_est = 0.;
    let mut ϕ_est = 0.;

    // Transform body rates to Euler angles. (Matrix multiplication)
    // todo: rename from dot etc
    // v_ϕ = readings.p + tan(θ_est) * sin(ϕ_est) * readings.q + cos(ϕ_est) * readings.r;
    // v_θ = cos(ϕ_est) * readings.q - sin(ϕ_est) * readings.r;

    // Integrate Euler rates to get estimate of roll and pitch angles.
    // ϕ_est = ϕ_hat + DT * v_ϕ;
    // θ_est = θ_hat + DT * v_θ;

    Params {
        ..Default::default()
    }
}

// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/EKF.h
// https://github.com/gaochq/IMU_Attitude_Estimator/blob/master/src/EKF_Attitude.cpp

/* Variable Definitions */
static mut p: f32 = 0.;
static mut q: f32 = 0.;
static mut r: f32 = 0.;
static mut ax: f32 = 0.;
static mut ay: f32 = 0.;
static mut az: f32 = 0.;
static mut mx: f32 = 0.;
static mut my: f32 = 0.;
static mut mz: f32 = 0.;
static mut Va: f32 = 0.;
static mut lpfGyr: f32 = 0.;
static mut lpfAcc: f32 = 0.;
static mut lpfMag: f32 = 0.;
static mut lpfVa: f32 = 0.;
static mut x: [f32; 7] = [0.; 7];
static mut P: [f32; 49] = [0.; 49];
static mut Q: [f32; 49] = [0.; 49];
static mut R: [f32; 36] = [0.; 36];
static mut g: f32 = 0.;

/* Function Definitions */
fn EKF_init(nGyro: f32, nAcc: f32, nMag: f32) {
    let mut i = 0;

    // todo: These might need to be statics/globals
    let dv4 = [1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-8, 1.0E-8, 1.0E-8];

    let dv5 = [
        nGyro * nGyro,
        nGyro * nGyro,
        nGyro * nGyro,
        nGyro * nGyro,
        1.0E-9,
        1.0E-9,
        1.0E-9,
    ];

    let dv6 = [nAcc * nAcc, nAcc * nAcc, nAcc * nAcc, nMag, nMag, nMag];

    unsafe {
        g = 9.81;

        /*  Low-pass filtered measurements */
        p = 0.0;
        q = 0.0;
        r = 0.0;
        ax = 0.0;
        ay = 0.0;
        az = 0.0;
        mx = 0.0;
        my = 0.0;
        mz = 0.0;
        Va = 0.0;

        /*  Low-pass filter coefficients */
        lpfGyr = 0.7;
        lpfAcc = 0.9;
        lpfMag = 0.4;
        lpfVa = 0.7;

        /*  Initialise state estimate vector */
        for i in 0..7 {
            x[i] = 0.0;
        }

        x[0] = 1.0;

        let sizeofdouble = 8; // 8 bytes.

        /*  Initialise covariance matrix */
        P = [0.; 49];
        // memset(&P[0], 0, 49 * sizeofdouble);
        for i in 0..7 {
            P[i + 7 * i] = dv4[i];
        }

        /*  Process noise matrix */
        Q = [0.; 49];
        // memset(&Q[0], 0, 49 * sizeofdouble);
        for i in 0..7 {
            Q[i + 7 * i] = dv5[i];
        }

        /*  Measurement noise matrix */
        R = [0.; 36];
        // memset(&R[0], 0, 36 * sizeofdouble);
        for i in 0..6 {
            R[i + 6 * i] = dv6[i];
        }
    }
}

// todo: Put back
// unsafe fn updateEKFQuatAtt(gyr_rps: [f32; 3], acc_mps2: [f32; 3], mag_unit: [f32; 3], Va_mps: f32,
//                            magDecRad: f32,
//                            T: f32, NdivT: f32, roll_deg: &mut f32, pitch_deg: &mut f32, yaw_deg: &mut f32)
// {
//     let mut mnorm = 0.;
//     let mut i0 = 0;
//     let mut iy = 0;
//     let mut a = 0.;
//     let mut dv0 = [0.; 12];
//     let mut unnamed_idx_2 = 0.;
//     let mut A = [0.; 49];
//
//     let mut s = 0.;
//     let mut A_tmp = 0.;
//     let mut k = 0;
//     let mut b_a = [0.; 7];
//     let mut b_A = [0.; 49];
//     let mut dv1 = [0.; 49];
//     let mut smag = 0.;
//     let mut cmag = 0.;
//     let mut kBcol = 0;
//     let mut jA = 0;
//     let mut C = [0.; 42];
//     let mut C_tmp = 0.;
//     let mut b_tmp = [0.; 42];
//     let mut j =0;
//     let mut jj = 0;
//     let mut K = [0.; 42];
//     let mut jp1j = 0;
//     let mut ipiv = [0; 6];
//     let mut n = 0;
//     let mut b_C = [0.; 42];
//     let mut ix = 0;
//     let mut B = [0.; 36];
//     let mut dv2 = [0.; 6];
//     let mut dv3 = [0.; 6];
//
//     /*  Get measurements and low-pass filter them */
//     p = lpfGyr * p + (1.0 - lpfGyr) * gyr_rps[0];
//     q = lpfGyr * q + (1.0 - lpfGyr) * gyr_rps[1];
//     r = lpfGyr * r + (1.0 - lpfGyr) * gyr_rps[2];
//     ax = lpfAcc * ax + (1.0 - lpfAcc) * acc_mps2[0];
//     ay = lpfAcc * ay + (1.0 - lpfAcc) * acc_mps2[1];
//     az = lpfAcc * az + (1.0 - lpfAcc) * acc_mps2[2];
//     mx = lpfMag * mx + (1.0 - lpfMag) * mag_unit[0];
//     my = lpfMag * my + (1.0 - lpfMag) * mag_unit[1];
//     mz = lpfMag * mz + (1.0 - lpfMag) * mag_unit[2];
//     mnorm = sqrt((mx * mx + my * my) + mz * mz);
//     mx /= mnorm;
//     my /= mnorm;
//     mz /= mnorm;
//     Va = lpfVa * Va + (1.0 - lpfVa) * Va_mps;
//     i0 = NdivT as u32;
//     for iy in 0..i0 {
//         /*  Extract states */
//         /*  State transition function, xdot = f(x, u) */
//         /*  Update state estimate */
//         a = T / NdivT;
//         mnorm = 0.5 * -x[1];
//         dv0[0] = mnorm;
//         unnamed_idx_2 = 0.5 * -x[2];
//         dv0[4] = unnamed_idx_2;
//         s = 0.5 * -x[3];
//         dv0[8] = s;
//         dv0[1] = 0.5 * x[0];
//         dv0[5] = s;
//         dv0[9] = 0.5 * x[2];
//         dv0[2] = 0.5 * x[3];
//         dv0[6] = 0.5 * x[0];
//         dv0[10] = mnorm;
//         dv0[3] = unnamed_idx_2;
//         dv0[7] = 0.5 * x[1];
//         dv0[11] = 0.5 * x[0];
//         mnorm = p - x[4];
//         s = q - x[5];
//         unnamed_idx_2 = r - x[6];
//
//         for k in 0..4 {
//             b_a[k] = a * ((dv0[k] * mnorm + dv0[k + 4] * s) + dv0[k + 8] *
//                 unnamed_idx_2);
//         }
//
//         b_a[4] = 0.0;
//         b_a[5] = 0.0;
//         b_a[6] = 0.0;
//         for k in 0..7 {
//             x[k] += b_a[k];
//         }
//     }
//
//     /*  Normalise quaternion */
//     mnorm = sqrt(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
//     x[0] /= mnorm;
//     x[1] /= mnorm;
//     x[2] /= mnorm;
//     x[3] /= mnorm;
//
//     /*  Re-extract states */
//     /*  Compute Jacobian of f, A(x, u) */
//     A[0] = 0.0;
//     mnorm = p - x[4];
//     A[7] = -0.5 * mnorm;
//     unnamed_idx_2 = q - x[5];
//     s = -0.5 * unnamed_idx_2;
//     A[14] = s;
//     a = r - x[6];
//     A_tmp = -0.5 * a;
//     A[21] = A_tmp;
//     A[28] = 0.5 * x[1];
//     A[35] = 0.5 * x[2];
//     A[42] = 0.5 * x[3];
//     mnorm *= 0.5;
//     A[1] = mnorm;
//     A[8] = 0.0;
//     a *= 0.5;
//     A[15] = a;
//     A[22] = s;
//     A[29] = -0.5 * x[0];
//     A[36] = 0.5 * x[3];
//     A[43] = -0.5 * x[2];
//     s = 0.5 * unnamed_idx_2;
//     A[2] = s;
//     A[9] = A_tmp;
//     A[16] = 0.0;
//     A[23] = mnorm;
//     A[30] = -0.5 * x[3];
//     A[37] = -0.5 * x[0];
//     A[44] = 0.5 * x[1];
//     A[3] = a;
//     A[10] = s;
//     A[17] = -0.5 * (p - x[4]);
//     A[24] = 0.0;
//     A[31] = 0.5 * x[2];
//     A[38] = -0.5 * x[1];
//     A[45] = -0.5 * x[0];
//
//     for i0 in 0..7 {
//         A[4 + 7 * i0] = 0.0;
//         A[5 + 7 * i0] = 0.0;
//         A[6 + 7 * i0] = 0.0;
//     }
//
//     /*  Update error covariance matrix */
//     for i0 in 0..7 {
//         for k in 0..7 {
//             iy = i0 + 7 * k;
//             dv1[iy] = 0.0;
//             mnorm = 0.0;
//             unnamed_idx_2 = 0.0;
//             for kBcol in 0..7 {
//                 jA = i0 + 7 * kBcol;
//                 mnorm += A[jA] * P[kBcol + 7 * k];
//                 unnamed_idx_2 += P[jA] * A[k + 7 * kBcol];
//             }
//
//             dv1[iy] = unnamed_idx_2;
//             b_A[iy] = mnorm;
//         }
//     }
//
//     for i0 in 0..49 {
//         P[i0] += T * ((b_A[i0] + dv1[i0]) + Q[i0]);
//     }
//
//     /*  Compute magnetic field unit vector estimate in body coordinates from */
//     /*  quaternion estimates */
//     smag = sin(magDecRad);
//     cmag = cos(magDecRad);
//
//     /*  Compute accelerometer output estimates */
//     /*  Note: assuming here that u = Va, v = 0, w = 0 */
//     /*  Would be good to set u = Va * cos(theta), v = 0, w = Va * sin(theta) */
//     /*  But need expressions for cos(theta) as quaternions... */
//     /*  Output function z(x, u) = [axhat, ayhat, azhat, mxhat, myhat, mzhat] */
//     /*  Jacobian of z, C(x, u)  */
//     C[0] = 2.0 * g * x[2];
//     mnorm = -2.0 * g * x[3];
//     C[6] = mnorm;
//     C[12] = 2.0 * g * x[0];
//     unnamed_idx_2 = -2.0 * g * x[1];
//     C[18] = unnamed_idx_2;
//     C[24] = 0.0;
//     C[30] = 0.0;
//     C[36] = 0.0;
//     C[1] = unnamed_idx_2;
//     C[7] = -2.0 * g * x[0];
//     C[13] = mnorm;
//     C[19] = -2.0 * g * x[2];
//     C[25] = 0.0;
//     C[31] = 0.0;
//     C[37] = -Va;
//     C[2] = 0.0;
//     C[8] = 4.0 * g * x[1];
//     C[14] = 4.0 * g * x[2];
//     C[20] = 0.0;
//     C[26] = 0.0;
//     C[32] = Va;
//     C[38] = 0.0;
//     mnorm = 2.0 * x[3] * smag;
//     C[3] = mnorm;
//     unnamed_idx_2 = 2.0 * x[2] * smag;
//     C[9] = unnamed_idx_2;
//     s = 2.0 * x[1] * smag;
//     C[15] = s - 4.0 * x[2] * cmag;
//     A_tmp = 2.0 * x[0] * smag;
//     C[21] = A_tmp - 4.0 * x[3] * cmag;
//     C[27] = 0.0;
//     C[33] = 0.0;
//     C[39] = 0.0;
//     C[4] = -2.0 * x[3] * cmag;
//     a = 2.0 * x[2] * cmag;
//     C[10] = a - 4.0 * x[1] * smag;
//     C_tmp = 2.0 * x[1] * cmag;
//     C[16] = C_tmp;
//     C[22] = -2.0 * x[0] * cmag - 4.0 * x[3] * smag;
//     C[28] = 0.0;
//     C[34] = 0.0;
//     C[40] = 0.0;
//     C[5] = a - s;
//     C[11] = 2.0 * x[3] * cmag - A_tmp;
//     C[17] = 2.0 * x[0] * cmag + mnorm;
//     C[23] = C_tmp + unnamed_idx_2;
//     C[29] = 0.0;
//     C[35] = 0.0;
//     C[41] = 0.0;
//
//     /*  Kalman gain */
//     for i0 in 0..6 {
//         for k in 0..7 {
//             b_tmp[k + 7 * i0] = C[i0 + 6 * k];
//         }
//     }
//
//     for i0 in 0..7 {
//         for k in 0..6 {
//             mnorm = 0.0;
//             for iy in 0..7 {
//                 mnorm += P[i0 + 7 * iy] * b_tmp[iy + 7 * k];
//             }
//
//             K[i0 + 7 * k] = mnorm;
//         }
//     }
//
//     for i0 in 0..6 {
//         for k in 0..7 {
//             mnorm = 0.0;
//             for iy in 0..7 {
//                 mnorm += C[i0 + 6 * iy] * P[iy + 7 * k];
//             }
//
//             b_C[i0 + 6 * k] = mnorm;
//         }
//
//         for k in 0..6 {
//             mnorm = 0.0;
//             for iy in 0..7 {
//                 mnorm += b_C[i0 + 6 * iy] * b_tmp[iy + 7 * k];
//             }
//
//             iy = i0 + 6 * k;
//             B[iy] = mnorm + R[iy];
//         }
//
//         ipiv[i0] = (1 + i0);
//     }
//
//     for j in 0..5 {
//         jA = j * 7;
//         jj = j * 7;
//         jp1j = jA + 2;
//         n = 6 - j;
//         kBcol = 0;
//         ix = jA;
//         mnorm = fabs(B[jA]);
//         for k in 2..=n {
//             ix += 1;
//             s = fabs(B[ix]);
//             if s > mnorm {
//                 kBcol = k - 1;
//                 mnorm = s;
//             }
//         }
//
//         if B[jj + kBcol] != 0.0 {
//             if kBcol != 0 {
//                 iy = j + kBcol;
//                 ipiv[j] = iy + 1;
//                 ix = j;
//                 for k in 0..6 {
//                     mnorm = B[ix];
//                     B[ix] = B[iy];
//                     B[iy] = mnorm;
//                     ix += 6;
//                     iy += 6;
//                 }
//             }
//
//             i0 = (jj - j) as u32 + 6;
//             for n in jp1j..=i0 {
//                 B[n - 1] /= B[jj];
//             }
//         }
//
//         n = 4 - j;
//         iy = jA + 6;
//         jA = jj;
//         for kBcol in 0..=n {
//             mnorm = B[iy];
//             if B[iy] != 0.0 {
//                 ix = jj + 1;
//                 i0 = jA as u32 + 8;
//                 k = (jA - j) + 12;
//                 for jp1j in i0..=k {
//
//                     B[jp1j - 1] += B[ix] * -mnorm;
//                     ix += 1;
//                 }
//             }
//
//             iy += 6;
//             jA += 6;
//         }
//     }
//
//     for j in 0..6 {
//         jA = 7 * j - 1;
//         iy = 6 * j;
//         for k in 0..j {
//             kBcol = 7 * k;
//             mnorm = B[k + iy];
//             if mnorm != 0.0 {
//                 for n in 0..7 {
//                     jp1j = (n + jA) + 1;
//                     K[jp1j] -= mnorm * K[n + kBcol];
//                 }
//             }
//         }
//
//         mnorm = 1.0 / B[j + iy];
//         for n in 0..7 {
//             jp1j = (n + jA) + 1;
//             K[jp1j] *= mnorm;
//         }
//     }
//
//     for j in 5..=0 {
//         jA = 7 * j - 1;
//         iy = 6 * j - 1;
//         i0 = j as u32 + 2;
//         for k in 0..7 {
//             kBcol = 7 * (k - 1);
//             mnorm = B[k + iy];
//             if mnorm != 0.0 {
//                 for n in 0..7 {
//                     jp1j = (n + jA) + 1;
//                     K[jp1j] -= mnorm * K[n + kBcol];
//                 }
//             }
//         }
//     }
//
//     for iy in 4..=0 {
//         if ipiv[iy] != iy + 1 {
//             for kBcol in 0..7 {
//                 jA = kBcol + 7 * iy;
//                 mnorm = K[jA];
//                 jp1j = kBcol + 7 * (ipiv[iy] - 1);
//                 K[jA] = K[jp1j];
//                 K[jp1j] = mnorm;
//             }
//         }
//     }
//
//     /*  Update error covariance matrix */
//     memset(&A[0], 0, 49 * sizeof(double));
//     for k in 0..7 {
//         A[k + 7 * k] = 1.0;
//     }
//
//     for i0 in 0..7 {
//         for k in 0..7 {
//             mnorm = 0.0;
//             for iy in 0..6 {
//                 mnorm += K[i0 + 7 * iy] * C[iy + 6 * k];
//             }
//
//             iy = i0 + 7 * k;
//             b_A[iy] = A[iy] - mnorm;
//         }
//
//         for k in 0..7 {
//             mnorm = 0.0;
//             for iy in 0..7 {
//                 mnorm += b_A[i0 + 7 * iy] * P[iy + 7 * k];
//             }
//
//             A[i0 + 7 * k] = mnorm;
//         }
//     }
//
//     memcpy(&P[0], &A[0], 49 * sizeof(double));
//
//     /*  Update state estimate using measurements (accelerometer and */
//     /*  magnetometer) */
//     dv2[0] = ax;
//     dv2[1] = ay;
//     dv2[2] = az;
//     dv2[3] = mx;
//     dv2[4] = my;
//     dv2[5] = mz;
//     dv3[0] = -2.0 * g * (x[1] * x[3] - x[2] * x[0]);
//     dv3[1] = Va * (r - x[6]) - 2.0 * g * (x[2] * x[3] + x[1] * x[0]);
//     dv3[2] = -Va * (q - x[5]) - g * (1.0 - 2.0 * (x[1] * x[1] + x[2] * x[2]));
//     mnorm = 2.0 * x[0] * x[3];
//     unnamed_idx_2 = 2.0 * x[1] * x[2];
//     s = 2.0 * x[3] * x[3];
//     dv3[3] = smag * (mnorm + unnamed_idx_2) - cmag * ((2.0 * x[2] * x[2] + s) -
//         1.0);
//     dv3[4] = -cmag * (mnorm - unnamed_idx_2) - smag * ((2.0 * x[1] * x[1] + s) -
//         1.0);
//     dv3[5] = cmag * (2.0 * x[0] * x[2] + 2.0 * x[1] * x[3]) - smag * (2.0 * x[0] *
//         x[1] - 2.0 * x[2] * x[3]);
//     for i0 in 0..6 {
//         dv2[i0] -= dv3[i0];
//     }
//
//     for i0 in 0..7 {
//         mnorm = 0.0;
//         for k in 0..6 {
//             mnorm += K[i0 + 7 * k] * dv2[k];
//         }
//
//         x[i0] += mnorm;
//     }
//
//     /*  Normalise quaternion */
//     mnorm = sqrt(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
//     x[0] /= mnorm;
//     x[1] /= mnorm;
//     x[2] /= mnorm;
//     x[3] /= mnorm;
//
//     /*  Re-extract states */
//     /*  Store state estimates */
//     mnorm = x[0] * x[0];
//     unnamed_idx_2 = x[1] * x[1];
//     s = x[2] * x[2];
//     a = x[3] * x[3];
//
//     *roll_deg = atan2(2.0 * (x[0] * x[1] + x[2] * x[3]), ((mnorm + a) -
//         unnamed_idx_2) - s) * 180.0 / PI;
//     *pitch_deg = (2.0 * (x[0] * x[2] - x[1] * x[3])).arcsin() * 180.0 /
//        PI;
//     *yaw_deg = (2.0 * (x[0] * x[3] + x[1] * x[2]), ((mnorm + unnamed_idx_2) -
//         s) - a).arctan() * 180.0 / PI;
// }

// https://github.com/pms67/EKF-Quaternion-Attitude-Estimation/blob/master/updateEKFQuatAtt.m
//
// fn updateEKFQuatAtt_ml(gyr_rps, acc_mps2, mag_unit, Va_mps, magDecRad, T, NdivT) -> (f32, f32, f32) {
//
//     persistent flag
//     persistent p
//     persistent q
//     persistent r
//     persistent ax
//     persistent ay
//     persistent az
//     persistent mx
//     persistent my
//     persistent mz
//     persistent Va
//
//     persistent lpfGyr
//     persistent lpfAcc
//     persistent lpfMag
//     persistent lpfVa
//
//     persistent x
//     persistent P
//     persistent Q
//     persistent R
//
//     persistent g
//
//     if isempty(flag) {
//
//         flag = 1;
//
//         g = 9.81;
//
//         // Low-pass filtered measurements
//         p = 0.0; q = 0.0; r = 0.0;
//         ax = 0.0; ay = 0.0; az = 0.0;
//         mx = 0.0; my = 0.0; mz = 0.0;
//         Va = 0.0;
//
//         // Low-pass filter coefficients
//         lpfGyr = 0.7;
//         lpfAcc = 0.9;
//         lpfMag = 0.4;
//         lpfVa  = 0.7;
//
//         // Initialise state estimate vector
//         x = zeros(7, 1);
//         x(1) = 1.0;
//
//         // Initialise covariance matrix
//         cAtt0 = 0.001;
//         cBias0 = 0.0001;
//         P = diag([[1 1 1 1] * cAtt0, [1 1 1] * cBias0] .^ 2);
//
//         // Process noise matrix
//         nProcAtt  = 0.00005;
//         nProcBias = 0.000001;
//         Q = diag([[1 1 1 1] * nProcAtt, [1 1 1] * nProcBias] .^ 2);
//
//         // Measurement noise matrix
//         nMeasAcc = 0.05;
//         nMeasMag = 0.02;
//         R = diag([[1 1 1] * nMeasAcc, [1 1 1] * nMeasMag] .^ 2);
//
//     }
//
//     // Get measurements and low-pass filter them
//     p = lpfGyr * p + (1 - lpfGyr) * gyr_rps(1);
//     q = lpfGyr * q + (1 - lpfGyr) * gyr_rps(2);
//     r = lpfGyr * r + (1 - lpfGyr) * gyr_rps(3);
//
//     ax = lpfAcc * ax + (1 - lpfAcc) * acc_mps2(1);
//     ay = lpfAcc * ay + (1 - lpfAcc) * acc_mps2(2);
//     az = lpfAcc * az + (1 - lpfAcc) * acc_mps2(3);
//
//     mx = lpfMag * mx + (1 - lpfMag) * mag_unit(1);
//     my = lpfMag * my + (1 - lpfMag) * mag_unit(2);
//     mz = lpfMag * mz + (1 - lpfMag) * mag_unit(3);
//
//     mnorm = sqrt(mx * mx + my * my + mz * mz);
//     mx = mx / mnorm;
//     my = my / mnorm;
//     mz = mz / mnorm;
//
//     Va = lpfVa * Va + (1 - lpfVa) * Va_mps;
//
//     for it = 1 : NdivT
//
//         % Extract states
//     q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
//     bp = x(5); bq = x(6); br = x(7);
//
//     % State transition function, xdot = f(x, u)
//     C_body2qdot = 0.5 * [-q1, -q2, -q3; ...
//         q0, -q3,  q2; ...
//         q3,  q0, -q1; ...
//         -q2,  q1,  q0];
//
//     f = [ C_body2qdot * [p - bp; q - bq; r - br]; 0; 0; 0];
//
//     // Update state estimate
//     x = x + (T / NdivT) * f;
//
// }
//
// // Normalise quaternion
// qNorm = sqrt(x(1) * x(1) + x(2) * x(2) + x(3) * x(3) + x(4) * x(4));
// x(1) = x(1) / qNorm;
// x(2) = x(2) / qNorm;
// x(3) = x(3) / qNorm;
// x(4) = x(4) / qNorm;
//
// // Re-extract states
// q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
// bp = x(5); bq = x(6); br = x(7);
//
//
// // Compute Jacobian of f, A(x, u)
// A = [0, -0.5 * (p - bp), -0.5 * (q - bq), -0.5 * (r - br), 0.5 * q1, 0.5 * q2, 0.5 * q3; ...
// 0.5 * (p - bp), 0, 0.5 * (r - br), -0.5 * (q - bq), -0.5 * q0, 0.5 * q3, -0.5 * q2; ...
// 0.5 * (q - bq), -0.5 * (r - br), 0, 0.5 * (p - bp), -0.5 * q3, -0.5 * q0, 0.5 * q1; ...
// 0.5 * (r - br), 0.5 * (q - bq), -0.5 * (p - bp), 0, 0.5 * q2, -0.5 * q1, -0.5 * q0; ...
// 0, 0, 0, 0, 0, 0, 0; ...
// 0, 0, 0, 0, 0, 0, 0; ...
// 0, 0, 0, 0, 0, 0, 0];
//
// % Update error covariance matrix
// P = P + T * (A * P + P * A' + Q);
//
// // Compute magnetic field unit vector estimate in body coordinates from
// // quaternion estimates
// smag = sin(magDecRad);
// cmag = cos(magDecRad);
//
// magUnitEstimate = [ smag * (2 * q0 * q3 + 2 * q1 * q2) - cmag * (2 * q2 * q2 + 2 * q3 * q3 - 1); ...
// -cmag * (2 * q0 * q3 - 2 * q1 * q2) - smag * (2 * q1 * q1 + 2 * q3 * q3 - 1); ...
// cmag * (2 * q0 * q2 + 2 * q1 * q3) - smag * (2 * q0 * q1 - 2 * q2 * q3)];
//
// // Compute accelerometer output estimates
// // Note: assuming here that u = Va, v = 0, w = 0
// // Would be good to set u = Va * cos(theta), v = 0, w = Va * sin(theta)
// // But need expressions for cos(theta) as quaternions...
//
// accEstimate = [-2 * g * (q1 * q3 - q2 * q0); ...
// Va * (r - br) - 2 * g * (q2 * q3 + q1 * q0); ...
// -Va * (q - bq) - g * (1 - 2 * (q1 * q1 + q2 * q2))];
//
// // Output function z(x, u) = [axhat, ayhat, azhat, mxhat, myhat, mzhat]
// z = [accEstimate; ...
// magUnitEstimate];
//
// // Jacobian of z, C(x, u)
// C = zeros(6, 7);
//
// C = [ 2 * g * q2, -2 * g * q3,  2 * g * q0, -2 * g * q1, 0, 0, 0; ...
// -2 * g * q1, -2 * g * q0, -2 * g * q3, -2 * g * q2, 0, 0, -Va; ...
// 0, 4 * g * q1, 4 * g * q2, 0, 0, Va, 0; ...
// 2 * q3 * smag, 2 * q2 * smag, 2 * q1 * smag - 4 * q2 * cmag, 2 * q0 * smag - 4 * q3 * cmag, 0, 0, 0; ...
// -2 * q3 * cmag, 2 * q2 * cmag - 4 * q1 * smag, 2 * q1 * cmag, -2 * q0 * cmag - 4 * q3 * smag, 0, 0, 0; ...
// 2 * q2 * cmag - 2 * q1 * smag, 2 * q3 * cmag - 2 * q0 * smag, 2 * q0 * cmag + 2 * q3 * smag, 2 * q1 * cmag + 2 * q2 * smag, 0, 0, 0];
//
// // Kalman gain
// K = P * C' / (C * P * C' + R);
//
// // Update error covariance matrix
// P = (eye(length(x)) - K * C) * P;
//
// // Update state estimate using measurements (accelerometer and
// // magnetometer)
// x = x + K * ([ax; ay; az; mx; my; mz] - z);
//
// // Normalise quaternion
// qNorm = sqrt(x(1) * x(1) + x(2) * x(2) + x(3) * x(3) + x(4) * x(4));
// x(1) = x(1) / qNorm;
// x(2) = x(2) / qNorm;
// x(3) = x(3) / qNorm;
// x(4) = x(4) / qNorm;
//
// // Re-extract states
// q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
//
// // Store state estimates
// roll_deg   = atan2(2 * (q0 * q1 + q2 * q3), q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2) * 180 / pi;
// pitch_deg  = asin (2 * (q0 * q2 - q1 * q3)) * 180 / pi;
// yaw_deg    = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / pi;
//
// }

// See thsi approach as well: https://github.com/copterust/dcmimu/blob/master/src/lib.rs
