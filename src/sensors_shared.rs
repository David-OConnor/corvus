//! This module contains code shared between sensors. Currently this is
//! regarding DMA operations on the barometer and external sensors I2C lines.

use stm32_hal2::{dma, i2c::I2c};

use crate::{
    baro, gnss, mag,
    setup::{
        self, I2cBaro, I2cMag, BARO_DMA_PERIPH, BARO_RX_CH, BARO_TX_CH, EXT_SENSORS_TX_CH,
        IMU_DMA_PERIPH,
    },
    tof,
};

// Each of these values is register, value to write to register.
// todo: Populate these.
// We sequence these using TC ISRs.
pub static mut WRITE_BUF_BARO: [u8; 1] = [baro::Reg::PsrB2 as u8];
pub static mut WRITE_BUF_MAG: [u8; 1] = [mag::Reg::OutXL as u8];
pub static mut WRITE_BUF_TOF: [u8; 2] = [0; 2];

pub static mut READ_BUF_GNSS: [u8; gnss::MAX_BUF_LEN] = [0; gnss::MAX_BUF_LEN];
pub static mut READ_BUF_BARO: [u8; 6] = [0; 6]; // 3x pressure, 3x temperature.
pub static mut READ_BUF_MAG: [u8; 6] = [0; 6]; // 2 mag for each dimension.
pub static mut READ_BUF_TOF: [u8; 2] = [0; 2];

pub static mut V_A_ADC_READ_BUF: [u16; 2] = [0; 2];

// These values correspond to how much the voltage divider on these ADC pins reduces the input
// voltage. Multiply by these values to get the true readings.
// V batt / V read
pub const ADC_BATT_V_DIV: f32 = 11.;

// mA/V
pub const ADC_CURR_DIV: f32 = 400.; // todo

pub const ADC_SAMPLE_FREQ: f32 = 50.; // todo: what should this be?

/// We use this to sequence DMA writes and reads among the extenral sensors.
#[derive(Clone, Copy)]
pub enum ExtSensor {
    Mag,
    Gps,
    Tof,
}

// 3 sensors; each 16 bits.
// pub static mut EXT_SENSORS_READINGS: [u8; 3 * 2] = [0; 3 * 2];

/// Start continous transfers for all sensors controlled by this module.
pub fn start_transfers(i2c_ext_sensors: &mut I2cMag, i2c_baro: &mut I2cBaro) {
    unsafe {
        // todo: Does this continuous/circular approach work for I2C/multiple sensors
        // todo/these specific sensors?
        // i2c2.write_read_dma(
        //     baro::Addr,
        //     &write_buf_baro,
        //     &mut BARO_READINGS,
        //     BARO_TX_CH,
        //     BARO_RX_CH,
        //     // ChannelCfg {
        //     //     circular: dma::Circular::Enabled,
        //     // ..Default::default()
        //     // ..Default::default()
        //     // },
        //     Default::default(),
        //     Default::default(),
        // );
        //
        // i2c1.write_read_dma(
        //     baro::ADDR, // todo??
        //     &write_buf_ext_sensors,
        //     &mut EXT_SENSORS_READINGS,
        //     EXT_SENSORS_TX_CH,
        //     EXT_SENSORS_RX_CH,
        //     // ChannelCfg {
        //     //     circular: dma::Circular::Enabled,
        //     // ..Default::default()
        //     // ..Default::default()
        //     // },
        //     Default::default(),
        //     Default::default(),
        // );

        // todo: Or, maybe you have to sequence all reads and writes?

        // Read seq for ext sensors: Mag, GPS,

        // In DMA TC ISRs, sequence read and writes; These are the transfers that start
        // the sequence of writes and reads for each bus.

        // This stop appears to be required in some cases.
        dma::stop(BARO_DMA_PERIPH, BARO_TX_CH);
        dma::stop(BARO_DMA_PERIPH, BARO_RX_CH);

        i2c_baro.write_dma(
            // i2c_ext_sensors.write_dma(
            // todo temp since baro is currently wired to i2c1.
            baro::ADDR,
            &WRITE_BUF_BARO,
            false,
            BARO_TX_CH,
            Default::default(),
            setup::BARO_DMA_PERIPH,
        );

        // todo: Put back; temp removed while testing Baro
        // i2c_ext_sensors.write_dma(
        //     mag::ADDR,
        //     &write_buf_mag,
        //     false,
        //     EXT_SENSORS_TX_CH,
        //     Default::default(),
        //     setup::EXT_SENSORS_DMA_PERIPH,
        // );
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum BattCellCount {
    S2 = 2,
    S3 = 3,
    S4 = 4,
    S6 = 6,
    S8 = 8,
}

impl Default for BattCellCount {
    fn default() -> Self {
        Self::S4
    }
}

impl BattCellCount {
    pub fn num_cells(&self) -> f32 {
        // float since it interacts with floats.
        match self {
            Self::S2 => 2.,
            Self::S3 => 3.,
            Self::S4 => 4.,
            Self::S6 => 6.,
            Self::S8 => 8.,
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            Self::S2 => "2S",
            Self::S3 => "3S",
            Self::S4 => "4S",
            Self::S6 => "6S",
            Self::S8 => "8S",
        }
    }
}
