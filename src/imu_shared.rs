//! This module contains device-agnostic IMU code, including parsing IMU readings from a static
//! DMA buffer.

use stm32_hal2::{
    dma::{ChannelCfg, DmaPeriph, Priority},
    gpio::{self, Port},
};

use lin_alg2::f32::{Mat3, Vec3};

use crate::setup::{SpiImu, IMU_RX_CH, IMU_TX_CH};

const G: f32 = 9.8; // m/s

const GYRO_FULLSCALE: f32 = 34.90659; // In radians per second; equals 2,000 degrees/sec
const ACCEL_FULLSCALE: f32 = 156.9056; // 16 G

// static mut WRITE_BUF: [u8; 13] = [0; 13];

// In order to let this fill multiple times per processing, we need to send the register
// requests once per reading.
static mut WRITE_BUF: [u8; 13] = [0; 13];
// static mut WRITE_BUF: [u8; 13 * FLIGHT_CTRL_IMU_RATIO] = [0; 13 * FLIGHT_CTRL_IMU_RATIO];

// IMU readings buffer. 3 accelerometer, and 3 gyro measurements; 2 bytes each. 0-padded on the left,
// since that's where we pass the register in the write buffer.
// We use this buffer for DMA transfers of IMU readings. Note that reading order is different
// between different IMUs, due to their reg layout, and consecutive reg reads. In both cases, 6 readings,
// each with 2 bytes each.
pub static mut IMU_READINGS: [u8; 13] = [0; 13];
// pub static mut IMU_READINGS: [u8; 13 * FLIGHT_CTRL_IMU_RATIO] = [0; 13 * FLIGHT_CTRL_IMU_RATIO];

/// Represents sensor readings from a 6-axis accelerometer + gyro.
/// Accelerometer readings are in m/2^2. Gyroscope readings are in radians/s.
#[derive(Default)]
pub struct ImuReadings {
    /// Positive X: Accel towards right wing
    pub a_x: f32,
    /// Positive Y: Accel forwards
    pub a_y: f32,
    /// Positive X: Accel up
    pub a_z: f32,
    /// Positive pitch: nose up
    pub v_pitch: f32,
    /// Positive roll: left wing up
    pub v_roll: f32,
    /// Positive yaw: CW rotation.
    pub v_yaw: f32,
}

/// These are raw register readings. Intended for our deprecated, non-DMA API.
#[derive(Default)]
pub struct _ImuReadingsRaw {
    pub a_x: i16,
    pub a_y: i16,
    pub a_z: i16,
    pub v_pitch: i16,
    pub v_roll: i16,
    pub v_yaw: i16,
}

impl ImuReadings {
    /// We use this to assemble readings from the DMA buffer.
    pub fn from_buffer(buf: &[u8]) -> Self {
        // todo: Note: this mapping may be different for diff IMUs, eg if they use a different reading register ordering.
        // todo: Currently hard-set for ICM426xx.

        // Ignore byte 0; it's for the first reg passed during the `write` transfer.
        Self {
            a_x: interpret_accel(i16::from_be_bytes([buf[1], buf[2]])),
            a_y: interpret_accel(i16::from_be_bytes([buf[3], buf[4]])),
            a_z: interpret_accel(i16::from_be_bytes([buf[5], buf[6]])),
            v_pitch: interpret_gyro(i16::from_be_bytes([buf[7], buf[8]])),
            v_roll: interpret_gyro(i16::from_be_bytes([buf[9], buf[10]])),
            v_yaw: -interpret_gyro(i16::from_be_bytes([buf[11], buf[12]])),
        }
    }
}

/// Read all 3 measurements, by commanding a DMA transfer. The transfer is closed, and readings
/// are processed in the Transfer Complete ISR.
pub fn read_imu(starting_addr: u8, spi: &mut SpiImu, periph: DmaPeriph) {
    // First byte is the first data reg, per this IMU's. Remaining bytes are empty, while
    // the MISO line transmits readings.
    unsafe {
        WRITE_BUF[0] = starting_addr;
    }

    #[cfg(feature = "h7")]
    gpio::set_low(Port::C, 4);
    #[cfg(feature = "g4")]
    gpio::set_low(Port::B, 12);

    unsafe {
        spi.transfer_dma(
            &WRITE_BUF,
            &mut IMU_READINGS,
            IMU_TX_CH,
            IMU_RX_CH,
            ChannelCfg {
                priority: Priority::Low,
                ..Default::default()
            },
            ChannelCfg {
                priority: Priority::Low,
                ..Default::default()
            },
            periph,
        );
    }
}

/// Output: m/s^2
pub fn interpret_accel(val: i16) -> f32 {
    (val as f32 / i16::MAX as f32) * ACCEL_FULLSCALE
}

/// Output: rad/s
pub fn interpret_gyro(val: i16) -> f32 {
    (val as f32 / i16::MAX as f32) * GYRO_FULLSCALE
}

// This calibration functionality is from [AHRS](https://github.com/xioTechnologies/Fusion)

pub struct ImuCalibration {
    pub gyro_misalignment: Mat3,
    pub gyro_sensitivity: Vec3,
    pub gyro_offset: Vec3,
    pub accel_misalignment: Mat3,
    pub accel_sensitivity: Vec3,
    pub accel_offset: Vec3,
    pub soft_iron_matrix: Mat3,
    pub hard_iron_offset: Vec3,
}

impl Default for ImuCalibration {
    #[rustfmt::skip]
    fn default() -> Self {
        Self {
            gyro_misalignment: Mat3 {
                data: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
            },
            gyro_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            gyro_offset: Vec3::new(0.0, 0.0, 0.0),
            accel_misalignment: Mat3 {
                data: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
            },
            accel_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            accel_offset: Vec3::new(0.0, 0.0, 0.0),
            soft_iron_matrix: Mat3 {
                data: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
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

/// Calibrate the IMU, by taking a series of series while on a level surface.
pub fn calibrate() -> ImuCalibration {
    // todo: average? lowpass?
    Default::default()
}
