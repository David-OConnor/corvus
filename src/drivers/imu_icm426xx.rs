//! This module contains code for the ICM42605, and ICM42688 inertial measuring units.
//! This IMU has a 8kHz maximum update rate (42605), or 32kHz for ICM42688.
//! SPI speed max is 24Mhz.
//!
//! Note that both this and the DPS310 barometer read temperature.
//!
//! 24 MHz max SPI frequency

use stm32_hal2::{
    gpio::Pin,
    pac::SPI1,
    spi::{self, Spi},
};

use cortex_m::delay::Delay;

use crate::imu_shared::_ImuReadingsRaw;

// todo: Consider hardware notch filter.

// todo: Use WHOAMI etc to determine if we have this, or the ST IMU

pub struct ImuNotConnectedError {}

impl From<spi::Error> for ImuNotConnectedError {
    fn from(e: spi::Error) -> Self {
        Self {}
    }
}

const ADDR: u8 = 0x69; // todo

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
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

    IntConfig0 = 0x63,
    IntConfig1 = 0x64,

    IntSource0 = 0x65,
    IntSource2 = 0x66,
    IntSource3 = 0x68,
    IntSource4 = 0x69,

    IntfConfig4 = 0x7a,
    IntfConfig5 = 0x7b,
    IntfConfig6 = 0x7c,
}

impl Reg {
    /// Get the read address, which has the MSB = 1. Use the `u8` repr for writes.
    pub fn read_addr(&self) -> u8 {
        0x80 | (*self as u8)
    }
}

// We use this to determine which reg to start DMA reads
pub const READINGS_START_ADDR: u8 = 0x80 | 0x1F; // (AccelDataX1)

// https://github.com/pms67/Attitude-Estimation

/// Utility function to read a single byte.
fn read_one(reg: Reg, spi: &mut Spi<SPI1>, cs: &mut Pin) -> Result<u8, ImuNotConnectedError> {
    let mut buf = [reg.read_addr(), 0];

    cs.set_low();
    spi.transfer(&mut buf)?;
    cs.set_high();

    Ok(buf[1])
}

/// Utility function to write a single byte.
fn write_one(
    reg: Reg,
    word: u8,
    spi: &mut Spi<SPI1>,
    cs: &mut Pin,
) -> Result<(), ImuNotConnectedError> {
    cs.set_low();
    spi.write(&[reg as u8, word])?;
    cs.set_high();

    Ok(())
}

/// Configure the device.
pub fn setup(
    spi: &mut Spi<SPI1>,
    cs: &mut Pin,
    delay: &mut Delay,
) -> Result<(), ImuNotConnectedError> {
    // Leave default of SPI mode 0 and 3.

    // An external cyrstal is connected on othe H7 FC, but not the G4.
    #[cfg(feature = "h7")]
    write_one(Reg::IntfConfig5, 0b0000_0100, spi, cs)?;

    // Enable gyros and accelerometers in low noise mode.
    write_one(Reg::PwrMgmt0, 0b0000_1111, spi, cs)?;

    // Set gyros and accelerometers to 8kHz update rate, 2000 DPS gyro full scale range,
    // and +-16g accelerometer full scale range.
    write_one(Reg::GyroConfig0, 0b0000_0011, spi, cs)?;

    // "When transitioning from OFF to any of the other modes, do not issue any
    // register writes for 200µs." (Gyro and accel)
    delay.delay_us(200);

    write_one(Reg::AccelConfig0, 0b0000_0011, spi, cs)?;
    delay.delay_us(200);

    // Set both the accelerator and gyro filtesr to the low latency option.
    // todo: This is what BF does. Do we want this?
    write_one(Reg::GyroAccelConfig0, 14 << 4 | 14, spi, cs)?;

    // (Leave default INT_CONFIG settings of active low, push pull, pulsed.)

    //  "Interrupt pulse duration is 8 µs. Required if ODR ≥ 4kHz, optional for ODR
    // < 4kHz."
    // "Disables de-assert duration. Required if ODR ≥ 4kHz, optional for ODR <
    // 4kHz."
    // "For register INT_CONFIG1 (bank 0 register 0x64) bit 4 INT_ASYNC_RESET, user should change
    // setting to 0 from default setting of 1 for proper INT1 and INT2 pin operation."
    write_one(Reg::IntConfig1, 0b0110_0000, spi, cs)?;

    // todo Temp TS. BF uses this, but I think that only applies to latched from DS.
    // write_one(Reg::IntConfig0, 0b0011_0000, spi, cs);

    // Enable UI data ready interrupt routed to the INT1 pin.
    write_one(Reg::IntSource0, 0b0000_1000, spi, cs)?;

    // todo: Set filters?

    Ok(())
}

// todo: Low power fn

/// Read temperature.
pub fn _read_temp(spi: &mut Spi<SPI1>, cs: &mut Pin) -> Result<f32, ImuNotConnectedError> {
    let upper_byte = read_one(Reg::TempData1, spi, cs)?;
    let lower_byte = read_one(Reg::TempData0, spi, cs)?;

    // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
    let temp_data = ((upper_byte as u16) << 8) | (lower_byte as u16);
    Ok(temp_data as f32 / 132.48 + 25.)
}

/// Read all data, in blocking fashion. Deprecated in favor of DMA.
pub fn _read_all(
    spi: &mut Spi<SPI1>,
    cs: &mut Pin,
) -> Result<_ImuReadingsRaw, ImuNotConnectedError> {
    let accel_x_upper = read_one(Reg::AccelDataX1, spi, cs)?;
    let accel_x_lower = read_one(Reg::AccelDataX0, spi, cs)?;
    let accel_y_upper = read_one(Reg::AccelDataY1, spi, cs)?;
    let accel_y_lower = read_one(Reg::AccelDataY0, spi, cs)?;
    let accel_z_upper = read_one(Reg::AccelDataZ1, spi, cs)?;
    let accel_z_lower = read_one(Reg::AccelDataZ0, spi, cs)?;

    let gyro_x_upper = read_one(Reg::GyroDataX1, spi, cs)?;
    let gyro_x_lower = read_one(Reg::GyroDataX0, spi, cs)?;
    let gyro_y_upper = read_one(Reg::GyroDataY1, spi, cs)?;
    let gyro_y_lower = read_one(Reg::GyroDataY0, spi, cs)?;
    let gyro_z_upper = read_one(Reg::GyroDataZ1, spi, cs)?;
    let gyro_z_lower = read_one(Reg::GyroDataZ0, spi, cs)?;

    let a_x = i16::from_be_bytes([accel_x_upper, accel_x_lower]);
    let a_y = i16::from_be_bytes([accel_y_upper, accel_y_lower]);
    let a_z = i16::from_be_bytes([accel_z_upper, accel_z_lower]);

    // Positive yaw: CW rotation. Positive pitch: Nose down.
    let v_pitch = i16::from_be_bytes([gyro_x_upper, gyro_x_lower]);
    let v_roll = i16::from_be_bytes([gyro_y_upper, gyro_y_lower]);
    let v_yaw = i16::from_be_bytes([gyro_z_upper, gyro_z_lower]);

    Ok(_ImuReadingsRaw {
        a_x,
        a_y,
        a_z,
        v_pitch,
        v_roll,
        v_yaw,
    })
}
