//! This module contains code for the ICM42605 inertial measuring unit.
//! This IMU has a 8kHz maximum update rate.
//! SPI speed max is 24Mhz.
//!
//! Note that both this and the DPS310 barometer read temperature.
//!
//!
// todo: ICM-42688 instead? Compare features and availability

use stm32_hal2::{dma::{Dma, DmaChannel}, gpio::Pin, pac::{DMA1, SPI1}, spi::Spi};

use crate::sensor_fusion::ImuReadings;

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
#[repr(u8)]
enum Reg {
    TEMP_DATA1 = 0x1D,
    TEMP_DATA0 = 0x1E,

    ACCEL_DATA_X1 = 0x1F,
    ACCEL_DATA_X0 = 0x20,
    ACCEL_DATA_Y1 = 0x21,
    ACCEL_DATA_Y0 = 0x22,
    ACCEL_DATA_Z1 = 0x23,
    ACCEL_DATA_Z0 = 0x24,

    GYRO_DATA_X1 = 0x25,
    GYRO_DATA_X0 = 0x26,
    GYRO_DATA_Y1 = 0x27,
    GYRO_DATA_Y0 = 0x28,
    GYRO_DATA_Z1 = 0x29,
    GYRO_DATA_Z0 = 0x2a,

    PWR_MGMT0 = 0x4E,
    GYRO_CONFIG0 = 0x4F,
    ACCEL_CONFIG0 = 0x50,
    GYRO_CONFIG1 = 0x51,
    GYRO_ACCEL_CONFIG0 = 0x52,

    INT_SOURCE0 = 0x65,
    INT_SOURCE1 = 0x66,
    INT_SOURCE3 = 0x68,
    INT_SOURCE4 = 0x69,
}

// todo: Read via DMA at a very high rate, then apply a lowpass filter?

// https://github.com/pms67/Attitude-Estimation

/// Utility function to read a single byte.
fn read_one(reg: u8, spi: &mut Spi<SPI4>, cs: &mut Pin) -> u8 {
    let mut buf = [reg, 0];

    cs.set_low();
    spi.transfer(&mut buf).ok();
    cs.set_high();

    buf[0]
}

/// Configure the device.
pub fn setup(spi: &mut Spi<SPI4>, cs: &mut Pin) {
    // Leave default of SPI mode 0 and 3.

    // Enable gyros and accelerometers in low noise mode.
    cs.set_low();
    spi.write(&[Reg::PWR_MGMT0 as u8, 0b0000_0011]).ok();
    cs.set_high();

    // Set gyros and accelerometers to 8kHz update rate, 2000 DPS gyro full scale range,
    // and +-16g accelerometer full scale range.
    cs.set_low();
    spi.write(&[Reg::GYRO_CONFIG0 as u8, 0b0000_1111]).ok();
    cs.set_high();

    cs.set_low();
    spi.write(&[Reg::ACCEL_CONFIG0 as u8, 0b0000_1111]).ok();
    cs.set_high();

    // Enable UI data ready interrupt routed to INT1
    cs.set_low();
    spi.write(&[Reg::INT_SOURCE0 as u8, 0b0000_1000]).ok();
    cs.set_high();

    // todo: Set filters?
}

// todo: Low power fn

/// Read temperature.
pub fn read_temp(spi: &mut Spi<SPI4>, cs: &mut Pin) -> f32 {
    let upper_byte = read_one(Reg::TEMP_DATA1 as u8, spi, cs);
    let lower_byte = read_one(Reg::TEMP_DATA0 as u8, spi, cs);

    // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
    // todo: Also temp exists from FIFO in 8 bits?
    let temp_data = ((upper_byte as u16) << 8) | (lower_byte as u16);
    temp_data as f32 / 132.48 + 25.
}

// todo: Do we want to use FIFO over DMA?

/// Read all data
pub fn read_all(spi: &mut Spi<SPI4>, cs: &mut Pin) -> ImuReadings {
    let accel_x_upper = read_one(Reg::ACCEL_DATA_X1 as u8, spi, cs);
    let accel_x_lower = read_one(Reg::ACCEL_DATA_X0 as u8, spi, cs);
    let accel_y_upper = read_one(Reg::ACCEL_DATA_Y1 as u8, spi, cs);
    let accel_y_lower = read_one(Reg::ACCEL_DATA_Y0 as u8, spi, cs);
    let accel_z_upper = read_one(Reg::ACCEL_DATA_Z1 as u8, spi, cs);
    let accel_z_lower = read_one(Reg::ACCEL_DATA_Z0 as u8, spi, cs);

    let gyro_x_upper = read_one(Reg::GYRO_DATA_X1 as u8, spi, cs);
    let gyro_x_lower = read_one(Reg::GYRO_DATA_X0 as u8, spi, cs);
    let gyro_y_upper = read_one(Reg::GYRO_DATA_Y1 as u8, spi, cs);
    let gyro_y_lower = read_one(Reg::GYRO_DATA_Y0 as u8, spi, cs);
    let gyro_z_upper = read_one(Reg::GYRO_DATA_Z1 as u8, spi, cs);
    let gyro_z_lower = read_one(Reg::GYRO_DATA_Z0 as u8, spi, cs);

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


pub fn read_all_dma(spi: &mut Spi<SPI1>, cs: &mut Pin, dma: &mut Dma<DMA1>) {

    // todo: Is this right? What should it be?
    let buf = [
        Reg::ACCEL_DATA_X1 as u8,
        Reg::ACCEL_DATA_X0 as u8,
        Reg::ACCEL_DATA_Y1 as u8,
        Reg::ACCEL_DATA_Y0 as u8,
        Reg::ACCEL_DATA_Z1 as u8,
        Reg::ACCEL_DATA_Z0 as u8,
        Reg::GYRO_DATA_X1 as u8,
        Reg::GYRO_DATA_X0 as u8,
        Reg::GYRO_DATA_Y1 as u8,
        Reg::GYRO_DATA_Y0 as u8,
        Reg::GYRO_DATA_Z1 as u8,
        Reg::GYRO_DATA_Z0 as u8,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    ]; // todo

    cs.set_low();
    // imu::read_all(spi, cx.local.imu_cs));

    unsafe {
        spi.write_dma(
            &buf,
            dma::ChannelCfg {
                circular: dma::Circular::Enabled,
                ..Default::default()
            },
            dma,
        );
    }

    // todo: Do we need to enable SPI here, or can we leave it enabled the whole time?
}