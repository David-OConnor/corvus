//! This module contains device-agnostic IMU code, including parsing IMU readings from a static
//! DMA buffer.

use hal::{
    dma::{ChannelCfg, DmaPeriph, Priority},
    gpio::{self, Port},
};

use crate::setup::{SpiImu, IMU_RX_CH, IMU_TX_CH};

const G: f32 = 9.8; // m/s

pub const GYRO_FULLSCALE: f32 = 34.90659; // In radians per second; equals 2,000 degrees/sec
pub const ACCEL_FULLSCALE: f32 = 156.9056; // 16 G

// In order to let this fill multiple times per processing, we need to send the register
// requests once per reading.
static mut WRITE_BUF: [u8; 13] = [0; 13];

// IMU readings buffer. 3 accelerometer, and 3 gyro measurements; 2 bytes each. 0-padded on the left,
// since that's where we pass the register in the write buffer.
// We use this buffer for DMA transfers of IMU readings. Note that reading order is different
// between different IMUs, due to their reg layout, and consecutive reg reads. In both cases, 6 readings,
// each with 2 bytes each.
pub static mut IMU_READINGS: [u8; 13] = [0; 13];

/// Read all 3 measurements, by commanding a DMA transfer. The transfer is closed, and readings
/// are processed in the Transfer Complete ISR.
pub fn read_imu(starting_addr: u8, spi: &mut SpiImu, periph: DmaPeriph) {
    // First byte is the first data reg, per this IMU's. Remaining bytes are empty, while
    // the MISO line transmits readings.
    unsafe {
        WRITE_BUF[0] = starting_addr;
    }

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
