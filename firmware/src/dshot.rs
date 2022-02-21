//! This module contains code for the DSHOT digital protocol, using to control motor speed.
//! [Implementation in C++ we use code from](https://github.com/korken89/crect_dshot_stm32/blob/master/tim_dma.hpp)
//!
//! [Some information on the protocol](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/):
//! Every digital protocol has a structure, also called a frame. It defines which information is at which position in the data stream. And the frame structure of DSHOT is pretty straight forward:
//!
//! 11 bit throttle: 2048 possible values. 0 is reserved for disarmed. 1-47 are reserved for special commands. Leaving 48 to 2047 (2000 steps) for the actual throttle value
//! 1 bit telemetry request - if this is set, telemetry data is sent back via a separate channel
//! 4 bit CRC: (Cyclic Redundancy) Check to validate data (throttle and telemetry request bit)
//! 1 and 0 in the DSHOT frame are distinguished by their high time. This means that every bit has a certain (constant) length,
//! and the length of the high part of the bit dictates if a 1 or 0 is being received.

use stm32_hal2::{
    dma::{ChannelCfg, Dma},
    timer::{Timer, TimChannel},
    pac::{DMA1, TIM3, TIM5},
};

use crate::flight_ctrls::Rotor;


// Configure the DSHOT protocol for a given timer rate.

pub const TIM_FREQ: f32 = 600_000.; // DSHOT 600.

const TIM_RATE: u16 = 100_000_000; // todo!

// todo: what type for these?
const BIT_PERIOD: u16 = (TIM_RATE + 1_200_000/2) / 1_200_000;
const BIT_0: u16 = (BIT_PERIOD * 1 + 1) / 3 - 1;
const BIT_1: u16 = (BIT_PERIOD * 3 + 2) / 4 - 1;

static mut PAYLOAD_R1: [u32; 17] = [0; 17];

fn prepare_packet(val: u16, request_telemetry: bool) -> u16 {
    let mut packet = (val << 1) | (if request_telemetry_ { 1 } else { 0 });

    // Compute checksum
    let crc = (val ^ (val >> 4) ^ (val >> 8)) & 0x0F;

    (packet << 4) | crc
}


/// Set an individual rotor's power. Power ranges from 0. to 1.
pub fn set_power(rotor: Rotor, power: f32, timer: &mut Timer<TIM3>) {

    // todo: Convert power to u16.
    //

    // First 11 (0:10) bits are the throttle settings. 0 means disarmed. 1-47 are reserved
    // for special commands. 48 - 2_047 are throttle value (2_000 possible values)

    // Bit 11 is 1 to request telemetry; 0 otherwise.
    // Bits 12:15 are CRC, to validate data.

    // Dshot 300.
    let val = (power * 1_999.) as u16 + 48;

    let mut packet = prepare_packet(val, true);

    for i in 0..16 {
        // Dshot is MSB first
        unsafe {
            payload_[i] = if packet & 0x8000 { BIT_1 } else { BIT_0 };
        }
        packet <<= 1;
    }

    // todo: Use a LUT or something for performance.
    // let arr_portion = power * PWM_ARR as f32;

    // timer.set_duty(rotor.tim_channel(), arr_portion as u32);
}

fn send_payload(timer: &mut Timer<TIM3>, dma: &mut Dma<DMA1>){

    let dma_cfg = ChannelCfg ::Default();

    unsafe {
        timer.write_dma_burst(
            &PAYLOAD_R1, // todo
            TimChannel::C3, // todo
            1, // todo
            DmaChannel::C2, // todo
            dma_cfg,
            &mut dma,
        );
    }

    // Reset DMA payload size, flags and enable it
    // DMA2_Stream5->NDTR = payload_.size();
    // DMA2->HIFCR = DMA2->HISR;
    // DMA2_Stream5->CR |= DMA_SxCR_EN;

    // Reset and update Timer registers
    timer.regs.egr.write(|w| w.ug().set_bit());
  }