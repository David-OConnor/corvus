//! This module contains code for the ICM42605 inertial measuring unit.
//! This IMU has a 8kHz maximum update rate. Also compatible with ICM42688
//! SPI speed max is 24Mhz.
//!
//! Note that both this and the DPS310 barometer read temperature.
//!
//!

// todo: Consider hardware notch filter.

use stm32_hal2::{gpio::Pin, pac::SPI1, spi::Spi};

use crate::sensor_fusion::ImuReadings;

const GYRO_FULLSCALE: f32 = 34.90659; // 2,000 degrees/sec
const ACCEL_FULLSCALE: f32 = 156.9056; // 16 G

/// See Datasheet, Table 19.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    FuncCfgAccess = 0x01,
    PinCtrl = 0x02,
    FifoCtrl1 = 0x07,
    FifoCtrl2 = 0x08,
    FifoCtrl3 = 0x09,
    FifoCtrl4 = 0x0A,
    CounterBdrReg1 = 0x0B,
    CounterBdrReg2 = 0x0C,
    Int1Ctrl = 0x0D,
    Int2Ctrl = 0x0E,
    WhoAmI = 0x0F,
    Ctrl1Xl = 0x10,
    Ctrl2G = 0x11,
    Ctrl3C = 0x12,
    Ctrl4C = 0x13,
    Ctrl5C = 0x14,
    Ctrl6C = 0x15,
    Ctrl7G = 0x16,
    Ctrl8Xl = 0x17,
    Ctrl9Xl = 0x18,
    Ctrl10C = 0x19,
    AllIntSrc = 0x1A,
    WakeUpSrc = 0x1B,
    TapSrc = 0x1C,
    D6DSrc = 0x1D,
    StatusReg = 0x1E,
    OutTempL = 0x20,
    OutTempH = 0x21,
    OutxLG = 0x22,
    OutxHG = 0x23,
    OutyLG = 0x24,
    OutyHG = 0x25,
    OutzLG = 0x26,
    OutzHG = 0x27,
    OutxLA = 0x28,
    OutxHA = 0x29,
    OutyLA = 0x2A,
    OutyHA = 0x2B,
    OutzLA = 0x2C,
    OutzHA = 0x2D,
    EmbFuncStatusMainpage = 0x35,
    FsmStatusAMainpage = 0x36,
    FsmStatusBMainpage = 0x37,
    MlcStatusMainpage = 0x38,
    StatusMasterMainpage = 0x39,
    FifoStatus1 = 0x3A,
    FifoStatus2 = 0x3B,
    // todo missing a lot here. Add as required or jsut add all
    FifoDataOutTag = 0x78,
    FifoDataOutXL = 0x79,
    FifoDataOutXH = 0x7A,
    FifoDataOutYL = 0x7B,
    FifoDataOutYH = 0x7C,
    FifoDataOutZL = 0x7D,
    FifoDataOutZH = 0x7E,
}

// todo: Copy `read_reg` method from icm426xx` too, if required.
impl Reg {
    /// Get the read address, which has the MSB = 1. Use the `u8` repr for writes.
    pub fn read_addr(&self) -> u8 {
        0x80 | (*self as u8)
    }
}
// We use this to determine which reg to start DMA reads
pub const READINGS_START_ADDR: u8 = 0x80 | 0x22; // (OutxLG)

// todo: Read via DMA at a very high rate, then apply a lowpass filter?

// https://github.com/pms67/Attitude-Estimation

/// Utility function to read a single byte.
fn read_one(reg: Reg, spi: &mut Spi<SPI1>, cs: &mut Pin) -> u8 {
    let mut buf = [reg.read_addr(), 0];

    cs.set_low();
    spi.transfer(&mut buf).ok();
    cs.set_high();

    buf[1]
}

/// Utility function to write a single byte.
fn write_one(reg: Reg, word: u8, spi: &mut Spi<SPI1>, cs: &mut Pin) {
    cs.set_low();
    spi.write(&[reg as u8, word]).ok();
    cs.set_high();
}

/// Configure the device.
pub fn setup(spi: &mut Spi<SPI1>, cs: &mut Pin) {
    // Leave default of SPI mode 0 and 3.

    // "The accelerometer is activated from power-down by writing ODR_XL[3:0] in CTRL1_XL (10h) while the gyroscope
    // is activated from power-down by writing ODR_G[3:0] in CTRL2_G (11h). For combo-mode the ODRs are totally
    // independent."

    // Set accelerometer to ODR = 6.66kHz update rate, +-16G full scale range, first state digital filtering
    // todo: Currently set to output from first state digital filtering. Do we want this, or second?

    write_one(Reg::Ctrl1Xl, 0b1010_0100, spi, cs);

    // Set gyro ODR to 6.66kHz update rate, 2000dps full scale range
    write_one(Reg::Ctrl2G, 0b1010_1100, spi, cs);

    // Disable I2C interface. Enable Gyro LPF1.
    write_one(Reg::Ctrl4C, 0b0000_0110, spi, cs);

    // Enable high performance mode on the accelerometer
    write_one(Reg::Ctrl6C, 0b1000_0000, spi, cs);

    // Enable high performance mode on the gyro
    write_one(Reg::Ctrl7G, 0b1000_0000, spi, cs);

    // Enable data ready on INT1 pin.
    // todo: What do we want? Both accel and gyro? DEN? What is DEN? Will both gryo and accel
    // todo together cause problems?
    write_one(Reg::Int1Ctrl, 0b0000_0011, spi, cs);

    // todo: Finish this; check what other settings are avail.
}

// todo: Low power fn

// todo: Combine read_temp and read_all with ICM driver?, but pass different reg values

/// Read temperature.
pub fn _read_temp(spi: &mut Spi<SPI1>, cs: &mut Pin) -> f32 {
    let upper_byte = read_one(Reg::OutTempH, spi, cs);
    let lower_byte = read_one(Reg::OutTempL, spi, cs);

    // todo: What's the conversion? This comment and code is for the ICM chip.
    // Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
    let temp_data = ((upper_byte as u16) << 8) | (lower_byte as u16);
    temp_data as f32 / 132.48 + 25.
}

/// Output: m/s^2
pub fn interpret_accel(val: i16) -> f32 {
    (val as f32 / i16::MAX as f32) * ACCEL_FULLSCALE
}

/// Output: rad/s
pub fn interpret_gyro(val: i16) -> f32 {
    (val as f32 / i16::MAX as f32) * GYRO_FULLSCALE
}

/// Read all data
pub fn read_all(spi: &mut Spi<SPI1>, cs: &mut Pin) -> ImuReadings {
    let accel_x_upper = read_one(Reg::OutxHA, spi, cs);
    let accel_x_lower = read_one(Reg::OutxLA, spi, cs);
    let accel_y_upper = read_one(Reg::OutyHA, spi, cs);
    let accel_y_lower = read_one(Reg::OutyLA, spi, cs);
    let accel_z_upper = read_one(Reg::OutzHA, spi, cs);
    let accel_z_lower = read_one(Reg::OutzLA, spi, cs);

    let gyro_x_upper = read_one(Reg::OutxHG, spi, cs);
    let gyro_x_lower = read_one(Reg::OutxLG, spi, cs);
    let gyro_y_upper = read_one(Reg::OutyHG, spi, cs);
    let gyro_y_lower = read_one(Reg::OutyLG, spi, cs);
    let gyro_z_upper = read_one(Reg::OutzHG, spi, cs);
    let gyro_z_lower = read_one(Reg::OutzLG, spi, cs);

    let a_x = interpret_accel(i16::from_be_bytes([accel_x_upper, accel_x_lower]));
    let a_y = interpret_accel(i16::from_be_bytes([accel_y_upper, accel_y_lower]));
    let a_z = interpret_accel(i16::from_be_bytes([accel_z_upper, accel_z_lower]));

    // Positive yaw: CW rotation. Positive pitch: Nose down.
    let v_pitch = -interpret_gyro(i16::from_be_bytes([gyro_x_upper, gyro_x_lower]));
    let v_roll = interpret_gyro(i16::from_be_bytes([gyro_y_upper, gyro_y_lower]));
    let v_yaw = -interpret_gyro(i16::from_be_bytes([gyro_z_upper, gyro_z_lower]));

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
