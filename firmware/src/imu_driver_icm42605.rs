//! This module contains code for the ICM42605 inertial measuring unit.
//! This IMU has a 8kHz maximum update rate.
//! SPI speed max is 24Mhz.
//!
//! Note that both this and the DPS310 barometer read temperature.

use stm32_hal2::{gpio::Pin, pac::SPI4, spi::Spi};

use crate::sensor_fusion::ImuReadings;

const TEMP_DATA1: u8 = 0x1D;
const TEMP_DATA0: u8 = 0x1E;

const ACCEL_DATA_X1: u8 = 0x1F;
const ACCEL_DATA_X0: u8 = 0x20;
const ACCEL_DATA_Y1: u8 = 0x21;
const ACCEL_DATA_Y0: u8 = 0x22;
const ACCEL_DATA_Z1: u8 = 0x23;
const ACCEL_DATA_Z0: u8 = 0x24;

const GYRO_DATA_X1: u8 = 0x25;
const GYRO_DATA_X0: u8 = 0x26;
const GYRO_DATA_Y1: u8 = 0x27;
const GYRO_DATA_Y0: u8 = 0x28;
const GYRO_DATA_Z1: u8 = 0x29;
const GYRO_DATA_Z0: u8 = 0x2a;

const PWR_MGMT0: u8 = 0x4E;
const GYRO_CONFIG0: u8 = 0x4F;
const ACCEL_CONFIG0: u8 = 0x50;
const GYRO_CONFIG1: u8 = 0x51;
const GYRO_ACCEL_CONFIG0: u8 = 0x52;

const INST_SOURCE0: u8 = 0x65;
const INST_SOURCE1: u8 = 0x66;
const INST_SOURCE3: u8 = 0x68;
const INST_SOURCE4: u8 = 0x69;

// todo: Read via DMA at a very high rate, then apply a lowpass filter?

// https://github.com/pms67/Attitude-Estimation

/// Utility function to read a single byte.
fn read_one(reg: u8, spi: &mut Spi<SPI4>, cs: &mut Pin) -> u8 {
    let mut buf = [reg, 0];

    cs.set_low();
    spi.transfer(&mut buf);
    cs.set_high();

    buf[0]
}

/// Configure the device.
pub fn setup(spi: &mut Spi<SPI4>, cs: &mut Pin) {
    // Leave default of SPI mode 0 and 3.

    // Enable gyros and accelerometers in low noise mode.
    cs.set_low();
    spi.write(&[PWR_MGMT0, 0b0000_0011]);
    cs.set_high();

    // Set gyros and accelerometers to 8kHz update rate.
    // todo: What do we want full scale select to be for these?
    // todo: Currently leaving default of 2000dps for gyro, and setting +-8g for accel.
    cs.set_low();
    spi.write(&[GYRO_CONFIG0, 0b0000_1111]);
    cs.set_high();

    cs.set_low();
    spi.write(&[ACCEL_CONFIG0, 0b0010_1111]);
    cs.set_high();

    // Enable UI data ready interrupt routed to INT1
    cs.set_low();
    spi.write(&[INT_SOURCE0, 0b0000_1000]);
    cs.set_high();

    // todo: Set filters?
}

// todo: Low power fn

/// Read temperature.
pub fn read_temp(spi: &mut Spi<SPI4>, cs: &mut Pin) -> f32 {
    let upper_byte = read_one(TEMP_DATA1, spi, cs);
    let lower_byte = read_one(TEMP_DATA0, spi, cs);

    // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
    // todo: Also temp exists from FIFO in 8 bits?
    let temp_data = ((upper_byte as u16) << 8) | (lower_byte as u16);
    temp_data as f32 / 132.48 + 25.
}

// todo: Do we want to use FIFO over DMA?

/// Read all data
pub fn read_all(spi: &mut Spi<SPI4>, cs: &mut Pin) -> ImuReadings {
    let accel_x_upper = read_one(ACCEL_DATA_X1, spi, cs);
    let accel_x_lower = read_one(ACCEL_DATA_X0, spi, cs);
    let accel_y_upper = read_one(ACCEL_DATA_Y1, spi, cs);
    let accel_y_lower = read_one(ACCEL_DATA_Y0, spi, cs);
    let accel_z_upper = read_one(ACCEL_DATA_Z1, spi, cs);
    let accel_z_lower = read_one(ACCEL_DATA_Z0, spi, cs);

    let gyro_x_upper = read_one(GYRO_DATA_X1, spi, cs);
    let gyro_x_lower = read_one(GYRO_DATA_X0, spi, cs);
    let gyro_y_upper = read_one(GYRO_DATA_Y1, spi, cs);
    let gyro_y_lower = read_one(GYRO_DATA_Y0, spi, cs);
    let gyro_z_upper = read_one(GYRO_DATA_Z1, spi, cs);
    let gyro_z_lower = read_one(GYRO_DATA_Z0, spi, cs);

    let a_x = u16::from_be_bytes([accel_x_upper, accel_x_lower]) as f32;
    let a_y = u16::from_be_bytes([accel_y_upper, accel_y_lower]) as f32;
    let a_z = u16::from_be_bytes([accel_z_upper, accel_z_lower]) as f32;

    let v_roll = u16::from_be_bytes([gyro_x_upper, gyro_x_lower]) as f32;
    let v_pitch = u16::from_be_bytes([gyro_y_upper, gyro_y_lower]) as f32;
    let v_yaw = u16::from_be_bytes([gyro_z_upper, gyro_z_lower]) as f32;

    // todo: How do we map these to radians per second and m/s^2?

    ImuReadings {
        a_x,
        a_y,
        a_z,
        v_pitch,
        v_roll,
        v_yaw,
    }
}
