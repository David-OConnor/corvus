//! This module contains code shared between sensors. Currently this is
//! regarding DMA operations on the barometer and external sensors I2C lines.

use stm32_hal2::{
    dma::{ChannelCfg, Dma},
    i2c::I2c,
    pac::{DMA1, DMA2, I2C1, I2C2},
};

use crate::{
    baro, gps, mag,
    setup::{BARO_RX_CH, BARO_TX_CH, EXT_SENSORS_RX_CH, EXT_SENSORS_TX_CH},
    tof,
};

// Each of these values is register, value to write to register.
// todo: Populate these.
// We sequence these using TC ISRs.
pub static mut WRITE_BUF_GPS: [u8; 2] = [0; 2];
pub static mut WRITE_BUF_TOF: [u8; 2] = [0; 2];

/// We use this to sequence DMA writes and reads among the extenral sensors.
#[derive(Clone, Copy)]
pub enum ExtSensor {
    Mag,
    Gps,
    Tof,
}

// todo: Sizes on these, and fns to interp them.
pub static mut BARO_READINGS: [u8; 2] = [0; 2];

pub static mut MAG_READINGS: [u8; 8] = [0; 8];
pub static mut GPS_READINGS: [u8; 12] = [0; 12];
pub static mut TOF_READINGS: [u8; 2] = [0; 2];

// 3 sensors; each 16 bits.
// pub static mut EXT_SENSORS_READINGS: [u8; 3 * 2] = [0; 3 * 2];

/// Start continous transfers for all sensors controlled by this module.
pub fn start_transfers(
    i2c_ext_sensors: &mut I2c<I2C1>,
    i2c_baro: &mut I2c<I2C2>,
    dma2: &mut Dma<DMA2>,
) {
    // let write_buf_ext_sensors = [starting_addr, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let write_buf_mag = [0, 0];
    let write_buf_baro = [0, 0];
    // todo: Hal-level write_read_dma fn?

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

        i2c_baro.write_dma(
            baro::ADDR,
            &write_buf_baro,
            false,
            BARO_TX_CH,
            Default::default(),
            dma2,
        );

        i2c_ext_sensors.write_dma(
            mag::ADDR,
            &write_buf_mag,
            false,
            EXT_SENSORS_TX_CH,
            Default::default(),
            dma2,
        );
    }
}
