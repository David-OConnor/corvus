//! This module contains code related to sensor fusion, eg using an extended kalman filter
//! to combine inputs from multiple sensors. It also includes code for interpreting, integrating,
//! and taking the derivatives of sensor readings. Code here is device-agnostic.

const G: f32 = 9.8; // m/s

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

/// Estimate attitude, based on IMU data of accelerations and roll rates.
pub fn estimate_attitude(readings: ImuReadings) {
    // Euler angle conventions: θ = pitch. phi = roll.

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

    // pitch and roll from accel readings alone, using gravity.
    let ϕ_est = (ay / az).atan();
    let θ_est = (ax / G).asin();

    // https://www.youtube.com/watch?v=RZd6XDx5VXo

    let mut θ_est = 0.;
    let mut ϕ_est = 0.;

    // Transform body rates to Euler angles
    // todo: rename from dot etc
    ϕ_dot = readings.p + θ_est.tan() * (ϕ_est).sin() * readings.q + ϕ_est.cos() * readings.r;
    θ_dot = (ϕ_est).cos() * readings.q - ϕ_est.sin() * readings.r;
}