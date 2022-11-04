//! This module contains code shared between sensors. Currently this is
//! regarding DMA operations on the barometer and external sensors I2C lines.

use stm32_hal2::{
    dma::Dma,
    i2c::I2c,
    pac::{DMA2, I2C1, I2C2},
};

use crate::{
    baro, gps, mag,
    setup::{BARO_TX_CH, EXT_SENSORS_TX_CH},
    tof,
};

use defmt::println;

// Each of these values is register, value to write to register.
// todo: Populate these.
// We sequence these using TC ISRs.
pub static mut WRITE_BUF_GPS: [u8; 1] = [0; 1];
pub static mut WRITE_BUF_TOF: [u8; 2] = [0; 2];

pub static mut V_A_ADC_READ_BUF: [u16; 2] = [0; 2];

// These values correspond to how much the voltage divider on these ADC pins reduces the input
// voltage. Multiply by these values to get the true readings.
// V batt / V read
pub const ADC_BATT_V_DIV: f32 = 11.;

// mA/V
pub const ADC_CURR_DIV: f32 = 400.; // todo

/// We use this to sequence DMA writes and reads among the extenral sensors.
#[derive(Clone, Copy)]
pub enum ExtSensor {
    Mag,
    Gps,
    Tof,
}

// todo: Sizes on these, and fns to interp them.
// 3 pressure bytes, followed by 3 temp bytes.
pub static mut BARO_READINGS: [u8; 6] = [0; 6];

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
    println!("Starting transfers");
    // let write_buf_ext_sensors = [starting_addr, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let write_buf_mag = [0, 0];
    let write_buf_baro = [baro::Reg::PsrB2 as u8];
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