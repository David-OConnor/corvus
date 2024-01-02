//! This module contains code for the ICM42605, and ICM42688 inertial measuring units.
//! This IMU has a 8kHz maximum update rate (42605), or 32kHz for ICM42688.
//! SPI speed max is 24Mhz.
//!
//! Note that both this and the DPS310 barometer read temperature.
//!
//! 24 MHz max SPI frequency

// todo: Robust fault detection: regularly check IMU's fault registers, and put that in the init
// todo script. Use the `Fault` status etc as required.

use stm32_hal2::{delay_us, gpio::Pin, spi};

use crate::setup::{SpiImu, AHB_FREQ};

const DEVICE_ID: u8 = 0x47;

// todo: Check this out:
// https://github.com/betaflight/betaflight/pull/12444/files

#[derive(Clone, Copy)]
pub enum ImuError {
    NotConnected,
    SelfTestFail,
}

impl From<spi::SpiError> for ImuError {
    fn from(_e: spi::SpiError) -> Self {
        Self::NotConnected
    }
}

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
/// Note that registers are divided into banks.
#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum RegBank0 {
    // Bank 0
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

    SelfTestConfig = 0x70,
    WhoAmI = 0x75,
    BankSel = 0x76,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum RegBank1 {
    GyroConfigStatic3 = 0x0c,
    GyroConfigStatic4 = 0x0d,
    GyroConfigStatic5 = 0x0e,
    IntfConfig4 = 0x7a,
    IntfConfig5 = 0x7b,
    IntfConfig6 = 0x7c,
}

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
/// Note that registers are divided into banks.
#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum RegBank2 {
    AccelConfigStatic2 = 0x03,
    AccelConfigStatic3 = 0x04,
    AccelConfigStatic4 = 0x05,
}

#[derive(Clone, Copy)]
pub enum Reg {
    Bank0(RegBank0),
    Bank1(RegBank1),
    Bank2(RegBank2),
    // Bank4(RegBank4),
}

impl Reg {
    pub fn addr(&self) -> u8 {
        match self {
            Self::Bank0(r) => *r as u8,
            Self::Bank1(r) => *r as u8,
            Self::Bank2(r) => *r as u8,
        }
    }
}

impl Reg {
    /// Get the read address, which has the MSB = 1. Use the `u8` repr for writes.
    pub fn read_addr(&self) -> u8 {
        0x80 | self.addr()
    }
}

// We use this to determine which reg to start DMA reads
pub const READINGS_START_ADDR: u8 = 0x80 | 0x1F; // (AccelDataX1)

// https://github.com/pms67/Attitude-Estimation

/// Utility function to read a single byte.
pub fn read_one(reg: Reg, spi: &mut SpiImu, cs: &mut Pin) -> Result<u8, ImuError> {
    let mut buf = [reg.read_addr(), 0];

    cs.set_low();
    spi.transfer(&mut buf)?;
    cs.set_high();

    Ok(buf[1])
}

/// Utility function to write a single byte.
fn write_one(reg: Reg, word: u8, spi: &mut SpiImu, cs: &mut Pin) -> Result<(), ImuError> {
    cs.set_low();
    spi.write(&[reg.addr(), word])?;
    cs.set_high();

    Ok(())
}

/// Select the bank we are writing to or reading from.
fn set_bank(bank: Reg, spi: &mut SpiImu, cs: &mut Pin) -> Result<(), ImuError> {
    let val = match bank {
        Reg::Bank0(_) => 0,
        Reg::Bank1(_) => 1,
        Reg::Bank2(_) => 2,
        // Reg::Bank4(_) => 4,
    };

    write_one(Reg::Bank0(RegBank0::BankSel), val, spi, cs)
}

/// Set up anti-alias filters. See DS, section 5.3. Reselect Bank 0 once complete.
fn setup_aa_filters(spi: &mut SpiImu, cs: &mut Pin) -> Result<(), ImuError> {
    // todo: What should these be?? Currently set to 997Hz.
    let aaf_delt = 21_u16;
    let aaf_delt_sqr = 440_u16;
    let aaf_bitshift = 6_u16;
    set_bank(Reg::Bank1(RegBank1::GyroConfigStatic3), spi, cs)?; // todo dummy val

    write_one(
        Reg::Bank1(RegBank1::GyroConfigStatic3),
        aaf_delt as u8,
        spi,
        cs,
    )?;

    write_one(
        Reg::Bank1(RegBank1::GyroConfigStatic4),
        (aaf_delt_sqr & 0xff) as u8,
        spi,
        cs,
    )?;

    write_one(
        Reg::Bank1(RegBank1::GyroConfigStatic5),
        ((aaf_delt_sqr >> 8) | (aaf_bitshift << 4)) as u8,
        spi,
        cs,
    )?;

    set_bank(Reg::Bank2(RegBank2::AccelConfigStatic2), spi, cs)?; // todo dummy val

    write_one(
        Reg::Bank2(RegBank2::AccelConfigStatic2),
        aaf_delt as u8,
        spi,
        cs,
    )?;

    write_one(
        Reg::Bank2(RegBank2::AccelConfigStatic3),
        (aaf_delt_sqr & 0xff) as u8,
        spi,
        cs,
    )?;

    write_one(
        Reg::Bank2(RegBank2::AccelConfigStatic4),
        ((aaf_delt_sqr >> 8) | (aaf_bitshift << 4)) as u8,
        spi,
        cs,
    )?;

    set_bank(Reg::Bank0(RegBank0::WhoAmI), spi, cs)?; // todo dummy val

    Ok(())
}

/// Configure the device.
pub fn setup(spi: &mut SpiImu, cs: &mut Pin) -> Result<(), ImuError> {
    // Leave default of SPI mode 0 and 3.

    // todo: Without self-test, we'll use a WHOAMI read to verify if the IMU is connected. Note that
    // todo the SPI bus will still not fail if the IMU isn't present. HAL error?
    // todo: Better sanity check than WHOAMI.

    // loop {
    //     write_one(Reg::Bank0(RegBank0::PwrMgmt0), 0b0111_0101, spi, cs).ok();
    //     delay_ms(10, AHB_FREQ);
    // }

    let device_id = read_one(Reg::Bank0(RegBank0::WhoAmI), spi, cs)?;

    if device_id != DEVICE_ID {
        return Err(ImuError::NotConnected);
    }

    // An external crystal is connected on the H7 FC, but not the G4.
    set_bank(Reg::Bank1(RegBank1::GyroConfigStatic3), spi, cs)?; // todo dummy val
    #[cfg(feature = "h7")]
    write_one(Reg::Bank1(RegBank1::IntfConfig5), 0b0000_0100, spi, cs)?;
    // (Bank 0 set by AA filter setup fn);

    setup_aa_filters(spi, cs)?;

    // Enable gyros and accelerometers in low noise mode.
    // Do this after setting up the AA filters.
    write_one(Reg::Bank0(RegBank0::PwrMgmt0), 0b0000_1111, spi, cs)?;

    // Set gyros and accelerometers to 8kHz update rate, 2000 DPS gyro full scale range,
    // and +-16g accelerometer full scale range.
    write_one(Reg::Bank0(RegBank0::GyroConfig0), 0b0000_0011, spi, cs)?;

    // "When transitioning from OFF to any of the other modes, do not issue any
    // register writes for 200µs." (Gyro and accel)
    delay_us(200, AHB_FREQ);

    write_one(Reg::Bank0(RegBank0::AccelConfig0), 0b0000_0011, spi, cs)?;
    delay_us(200, AHB_FREQ);

    // Set both the accelerator and gyro filters to the low latency option.
    // todo: This is what BF does. Do we want this?
    write_one(
        Reg::Bank0(RegBank0::GyroAccelConfig0),
        14 << 4 | 14,
        spi,
        cs,
    )?;

    // (Leave default INT_CONFIG settings of active low, push pull, pulsed.)

    //  "Interrupt pulse duration is 8 µs. Required if ODR ≥ 4kHz, optional for ODR
    // < 4kHz."
    // "Disables de-assert duration. Required if ODR ≥ 4kHz, optional for ODR <
    // 4kHz."
    // "For register INT_CONFIG1 (bank 0 register 0x64) bit 4 INT_ASYNC_RESET, user should change
    // setting to 0 from default setting of 1 for proper INT1 and INT2 pin operation."
    write_one(Reg::Bank0(RegBank0::IntConfig1), 0b0110_0000, spi, cs)?;

    // todo Temp TS. BF uses this, but I think that only applies to latched from DS.
    // write_one(Reg::IntConfig0, 0b0011_0000, spi, cs);

    // Enable UI data ready interrupt routed to the INT1 pin.
    write_one(Reg::Bank0(RegBank0::IntSource0), 0b0000_1000, spi, cs)?;

    // todo: Set filters?

    // Start a self test on all 6 channels, and accel power self test.
    // write_one(Reg::SelfTestConfig, 0b0111_1111, spi, cs)?;

    // todo: Get self-test working. DS is unclear on how this works. If fail, return SelfTest error.

    Ok(())
}

/// Read temperature.
pub fn _read_temp(spi: &mut SpiImu, cs: &mut Pin) -> Result<f32, ImuError> {
    let upper_byte = read_one(Reg::Bank0(RegBank0::TempData1), spi, cs)?;
    let lower_byte = read_one(Reg::Bank0(RegBank0::TempData0), spi, cs)?;

    // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
    let temp_data = ((upper_byte as u16) << 8) | (lower_byte as u16);
    Ok(temp_data as f32 / 132.48 + 25.)
}

// /// Read all data, in blocking fashion. Deprecated in favor of DMA.
// pub fn _read_all(spi: &mut SpiImu, cs: &mut Pin) -> Result<_ImuReadingsRaw, ImuError> {
//     let accel_x_upper = read_one(Reg::Bank0(RegBank0::AccelDataX1), spi, cs)?;
//     let accel_x_lower = read_one(Reg::Bank0(RegBank0::AccelDataX0), spi, cs)?;
//     let accel_y_upper = read_one(Reg::Bank0(RegBank0::AccelDataY1), spi, cs)?;
//     let accel_y_lower = read_one(Reg::Bank0(RegBank0::AccelDataY0), spi, cs)?;
//     let accel_z_upper = read_one(Reg::Bank0(RegBank0::AccelDataZ1), spi, cs)?;
//     let accel_z_lower = read_one(Reg::Bank0(RegBank0::AccelDataZ0), spi, cs)?;
//
//     let gyro_x_upper = read_one(Reg::Bank0(RegBank0::GyroDataX1), spi, cs)?;
//     let gyro_x_lower = read_one(Reg::Bank0(RegBank0::GyroDataX0), spi, cs)?;
//     let gyro_y_upper = read_one(Reg::Bank0(RegBank0::GyroDataY1), spi, cs)?;
//     let gyro_y_lower = read_one(Reg::Bank0(RegBank0::GyroDataY0), spi, cs)?;
//     let gyro_z_upper = read_one(Reg::Bank0(RegBank0::GyroDataZ1), spi, cs)?;
//     let gyro_z_lower = read_one(Reg::Bank0(RegBank0::GyroDataZ0), spi, cs)?;
//
//     let a_x = i16::from_be_bytes([accel_x_upper, accel_x_lower]);
//     let a_y = i16::from_be_bytes([accel_y_upper, accel_y_lower]);
//     let a_z = i16::from_be_bytes([accel_z_upper, accel_z_lower]);
//
//     // Positive yaw: CW rotation. Positive pitch: Nose down.
//     let v_pitch = i16::from_be_bytes([gyro_x_upper, gyro_x_lower]);
//     let v_roll = i16::from_be_bytes([gyro_y_upper, gyro_y_lower]);
//     let v_yaw = i16::from_be_bytes([gyro_z_upper, gyro_z_lower]);
//
//     Ok(_ImuReadingsRaw {
//         a_x,
//         a_y,
//         a_z,
//         v_pitch,
//         v_roll,
//         v_yaw,
//     })
// // }
