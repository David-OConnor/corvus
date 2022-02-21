//! This module contains code for the DSHOT digital protocol, using to control motor speed.
//! [Implementation in C++ we use code from](https://github.com/korken89/crect_dshot_stm32/blob/master/tim_dma.hpp)

use stm32_hal2::{
    timer::Timer,
    pac::{TIM3, TIM5},
};

use crate::flight_ctrls::Rotor;

// Configure the DSHOT protocol for a given timer rate.

const TIM_RATE: u16 = 100_000_000; // todo!

// todo: what type for these?
const BIT_PERIOD: u16 = (TIM_RATE + 1200000/2) / 1200000;
const BIT_0: u16 = (BIT_PERIOD * 1 + 1) / 3 - 1;
const BIT_1: u16 = (BIT_PERIOD * 3 + 2) / 4 - 1;

static mut PAYLOAD_R1: [u32; 17] = [0; 17];

fn prepare_packet(val: u16, requrest_telemetry: bool) -> u16 {
    let mut packet = (val << 1) | (if request_telemetry_ { 1 } else { 0 });

    // Compute checksum
    let mut csum = 0;
    let mut csum_data = packet;

    for i in 0..3 {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    (packet << 4) | (csum & 0xf)
  }


/// Set an individual rotor's power. Power ranges from 0. to 1.
pub fn set_power(rotor: Rotor, power: f32, timer: &mut Timer<TIM3>) {

    // todo: Convert power to u16.
    let val = (power * u16::MAX as f32) as u16;

    let mut packet = prepare_packet(val);

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

fn send_payload(timer: &mut Timer<TIM3>){
    // Reset DMA payload size, flags and enable it
    // DMA2_Stream5->NDTR = payload_.size();
    // DMA2->HIFCR = DMA2->HISR;
    // DMA2_Stream5->CR |= DMA_SxCR_EN;

    // Reset and update Timer registers
    timer.regs.egr.write(|w| w.ug().set_bit());
  }