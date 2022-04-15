//! This module contains code for the ICM42605, and ICM42688 inertial measuring units.
//! This IMU has a 8kHz maximum update rate (42605), or 32kHz for ICM42688.
//! SPI speed max is 24Mhz.
//!
//! Note that both this and the DPS310 barometer read temperature.
//!
//! 24 MHz max SPI frequency

use stm32_hal2::{
    dma::{Dma, DmaChannel},
    gpio::Pin,
    pac::{DMA1, SPI1},
    spi::Spi,
};

use cortex_m::delay::Delay;

use crate::sensor_fusion::ImuReadings;

const GRYO_FULLSCALE: f32 = 34.90659; // 2,000 degrees/sec
const ACCEL_FULLSCALE: f32 = 156.9056; // 16 G

// todo: Consider hardware notch filter.

// todo: Use WHOAMI etc to determine if we have this, or the ST IMU

// todo: Calibration

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
#[derive(Clone, Copy)]
#[repr(u8)]
enum Reg {
    DeviceConfig = 0x11,
    DriveConfig = 0x13,
    IntConfig = 0x14,
    FifoConfig = 0x16,

    TempData1 = 0x1D,
    TempData0 = 0x1E,

    AccelDataX1 = 0x1F,
    AccelDataX0 = 0x20,
    AccelDataY1 = 0x21,
    AccelDataY0 = 0x22,
    AccelDataZ1 = 0x23,
    AccelDataZ0 = 0x24,

    GyroDataX1 = 0x25,
    GyroDataX0 = 0x26,
    GyroDataY1 = 0x27,
    GyroDataY0 = 0x28,
    GyroDataZ1 = 0x29,
    GyroDataZ0 = 0x2a,

    IntStatus = 0x2d,
    PwrMgmt0 = 0x4E,
    GyroConfig0 = 0x4F,
    AccelConfig0 = 0x50,
    GyroConfig1 = 0x51,
    GyroAccelConfig0 = 0x52,

    IntSource0 = 0x65,
    IntSource2 = 0x66,
    IntSource3 = 0x68,
    IntSource4 = 0x69,

    IntfConfig4 = 0x7a,
    IntfConfig6 = 0x7c,
}

impl Reg {
    /// Get the read address, which has the MSB = 1. Use the `u8` repr for writes.
    pub fn read_addr(&self) -> u8 {
        0x80 | (*self as u8)
    }
}

// todo: Read via DMA at a very high rate, then apply a lowpass filter?

// https://github.com/pms67/Attitude-Estimation

/// Utility function to read a single byte.
fn read_one(reg: Reg, spi: &mut Spi<SPI1>, cs: &mut Pin) -> u8 {
    let mut buf = [reg.read_addr(), 0];
    // let mut buf = [reg as u8, 0];

    cs.set_low();
    spi.transfer(&mut buf).ok();
    cs.set_high();

    // buf[0]
    buf[1]
}

/// Utility function to write a single byte.
fn write_one(reg: Reg, word: u8, spi: &mut Spi<SPI1>, cs: &mut Pin) {
    cs.set_low();
    spi.write(&[reg as u8, word]).ok();
    cs.set_high();
}

/// Configure the device.
pub fn setup(spi: &mut Spi<SPI1>, cs: &mut Pin, delay: &mut Delay) {
    // Leave default of SPI mode 0 and 3.

    // Enable gyros and accelerometers in low noise mode.
    write_one(Reg::PwrMgmt0, 0b0000_1111, spi, cs);

    // Set gyros and accelerometers to 8kHz update rate, 2000 DPS gyro full scale range,
    // and +-16g accelerometer full scale range.
    write_one(Reg::GyroConfig0, 0b0000_0011, spi, cs);
    // "When transitioning from OFF to any of the other modes, do not issue any
    // register writes for 200µs." (Gyro and accel)
    delay.delay_us(200);

    write_one(Reg::AccelConfig0, 0b0000_0011, spi, cs);
    delay.delay_us(200);

    // (Leave default interrupt settings of active low, push pull, pulsed.)
    // write_one(Reg::IntConfig, 0b0000_0000, spi, cs);

    // Enable UI data ready interrupt routed to INT1
    write_one(Reg::IntSource0, 0b0000_1000, spi, cs);

    // todo: Set filters?
}

// todo: Low power fn

/// Read temperature.
pub fn read_temp(spi: &mut Spi<SPI1>, cs: &mut Pin) -> f32 {
    let upper_byte = read_one(Reg::TempData1, spi, cs);
    let lower_byte = read_one(Reg::TempData0, spi, cs);

    // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
    let temp_data = ((upper_byte as u16) << 8) | (lower_byte as u16);
    temp_data as f32 / 132.48 + 25.
}

/// Output: m/s^2
pub fn interpret_accel(accel: i16) -> f32 {
    (accel as f32 / i16::MAX as f32) * ACCEL_FULLSCALE
}

/// Output: rad/s
pub fn interpret_gyro(accel: i16) -> f32 {
    (accel as f32 / i16::MAX as f32) * ACCEL_FULLSCALE
}

/// Read all data
pub fn read_all(spi: &mut Spi<SPI1>, cs: &mut Pin) -> ImuReadings {
    let accel_x_upper = read_one(Reg::AccelDataX1, spi, cs);
    let accel_x_lower = read_one(Reg::AccelDataX0, spi, cs);
    let accel_y_upper = read_one(Reg::AccelDataY1, spi, cs);
    let accel_y_lower = read_one(Reg::AccelDataY0, spi, cs);
    let accel_z_upper = read_one(Reg::AccelDataZ1, spi, cs);
    let accel_z_lower = read_one(Reg::AccelDataZ0, spi, cs);

    let gyro_x_upper = read_one(Reg::GyroDataX1, spi, cs);
    let gyro_x_lower = read_one(Reg::GyroDataX0, spi, cs);
    let gyro_y_upper = read_one(Reg::GyroDataY1, spi, cs);
    let gyro_y_lower = read_one(Reg::GyroDataY0, spi, cs);
    let gyro_z_upper = read_one(Reg::GyroDataZ1, spi, cs);
    let gyro_z_lower = read_one(Reg::GyroDataZ0, spi, cs);

    let a_x = interpret_accel(i16::from_be_bytes([accel_x_upper, accel_x_lower]));
    let a_y = interpret_accel(i16::from_be_bytes([accel_y_upper, accel_y_lower]));
    let a_z = interpret_accel(i16::from_be_bytes([accel_z_upper, accel_z_lower]));

    let v_pitch = interpret_gyro(i16::from_be_bytes([gyro_x_upper, gyro_x_lower]));
    let v_roll = interpret_gyro(i16::from_be_bytes([gyro_y_upper, gyro_y_lower]));
    let v_yaw = interpret_gyro(i16::from_be_bytes([gyro_z_upper, gyro_z_lower]));

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
        Reg::AccelDataX1.read_addr(),
        Reg::AccelDataX0.read_addr(),
        Reg::AccelDataY1.read_addr(),
        Reg::AccelDataY0.read_addr(),
        Reg::AccelDataZ1.read_addr(),
        Reg::AccelDataZ0.read_addr(),
        Reg::GyroDataX1.read_addr(),
        Reg::GyroDataX0.read_addr(),
        Reg::GyroDataY1.read_addr(),
        Reg::GyroDataY0.read_addr(),
        Reg::GyroDataZ1.read_addr(),
        Reg::GyroDataZ0.read_addr(),
    ];

    cs.set_low();
    // imu::read_all(spi, cx.local.imu_cs));

    // unsafe {
    //     spi.write_dma(&buf, DmaChannel::C1, Default::default(), dma);
    // }

    // (The read command is in the Tx DMA xfer-complete ISR.)
    unsafe {
        // spi.read_dma(
        //     &mut crate::IMU_READINGS,
        //     DmaChannel::C2,
        //     Default::default(),
        //     dma,
        // );

        spi.transfer_dma(
            &buf,
            &mut crate::IMU_READINGS,
            DmaChannel::C1,
            DmaChannel::C2,
            Default::default(),
            Default::default(),
            dma,
        );
    }

    // todo: Do we need to enable SPI here, or can we leave it enabled the whole time?
}
