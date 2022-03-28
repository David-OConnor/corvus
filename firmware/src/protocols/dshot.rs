//! This module contains code for the DSHOT digital protocol, using to control motor speed.
//! [Implementation in C++ we use code from](https://github.com/korken89/crect_dshot_stm32/blob/master/tim_dma.hpp)
//!
//! [Some information on the protocol](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/):
//! Every digital protocol has a structure, also called a frame. It defines which information is at
//! which position in the data stream. And the frame structure of DSHOT is pretty straight forward:
//!
//! 11 bit throttle: 2048 possible values. 0 is reserved for disarmed. 1-47 are reserved for special commands.
//! Leaving 48 to 2047 (2000 steps) for the actual throttle value
//! 1 bit telemetry request - if this is set, telemetry data is sent back via a separate channel
//! 4 bit CRC: (Cyclic Redundancy) Check to validate data (throttle and telemetry request bit)
//! 1 and 0 in the DSHOT frame are distinguished by their high time. This means that every bit has a certain (constant) length,
//! and the length of the high part of the bit dictates if a 1 or 0 is being received.
//!
//! The DSHOT protocol (DSHOT-300, DSHOT-600 etc) is determined by the `DSHOT_ARR` and `DSHOT_PSC` settings in the
//! main crate; ie set a 600kHz countdown for DSHOT-600.

use stm32_hal2::{
    dma::{ChannelCfg, Dma, DmaChannel},
    pac::{DMA1, TIM2, TIM3},
    timer::{TimChannel, Timer},
};

use crate::Rotor;

// Duty cycle values (to be written to CCMRx), based on our ARR value. 0. = 0%. ARR = 100%.
const DUTY_HIGH: u32 = crate::DSHOT_ARR * 3 / 4;
const DUTY_LOW: u32 = crate::DSHOT_ARR * 3 / 8;

// DSHOT-600
pub const TIM_FREQ: f32 = 600.;

// DMA buffers for each rotor. 16-bit data, but using a 32-bit API. Note that
// rotors 1/2 and 3/4 share a timer, so we can use the same DMA stream with them. Data for the 2
// channels are interleaved.
static mut PAYLOAD_R1_2: [u32; 32] = [0; 32];
// static mut PAYLOAD_R2: [u32; 16] = [0; 16];
static mut PAYLOAD_R3_4: [u32; 32] = [0; 32];
// static mut PAYLOAD_R4: [u32; 16] = [0; 16];

/// Possible DSHOT commands (ie, DSHOT values 0 - 47). Does not include power settings.
#[derive(Copy, Clone)]
#[repr(u16)]
pub enum Command {
    MotorStop = 0,
    Beacon1 = 1,
    Beacon2 = 2,
    Beacon3 = 3,
    Beacon4 = 4,
    Beacon5 = 5,
    EscInfo = 6,
    SpinDir1 = 7,
    SpinDir2 = 8,
    _3dModeOff = 9,
    _3dModeOn = 10,
    SettingsRequest = 11,
    SaveSettings = 12,
    SpinDirNormal = 20,
    SpinDirReversed = 21,
    Led0On = 22,               // BLHeli32 only
    Led1On = 23,               // BLHeli32 only
    Led2On = 24,               // BLHeli32 only
    Led3On = 25,               // BLHeli32 only
    Led0Off = 26,              // BLHeli32 only
    Led1Off = 27,              // BLHeli32 only
    Led2Off = 28,              // BLHeli32 only
    Led3Off = 29,              // BLHeli32 only
    AudioStreamModeOnOff = 30, // KISS audio Stream mode on/Off
    SilendModeOnOff = 31,      // KISS silent Mode on/Off
    Max = 47,
}

enum CmdType {
    Command(Command),
    Power(f32),
}

// todo: Do we need compiler fences between payload seup, and starting the DMA writes? Consider adding if you
// todo run into trouble.

pub fn stop_all(timer_a: &mut Timer<TIM2>, timer_b: &mut Timer<TIM3>, dma: &mut Dma<DMA1>) {
    setup_payload(Rotor::R1, CmdType::Command(Command::MotorStop));
    setup_payload(Rotor::R2, CmdType::Command(Command::MotorStop));
    setup_payload(Rotor::R3, CmdType::Command(Command::MotorStop));
    setup_payload(Rotor::R4, CmdType::Command(Command::MotorStop));

    // todo: Make sure you have the right motors here.
    send_payload_a(timer_a, dma);
    // send_payload_a(Rotor::R2, timer_a, dma);
    send_payload_b(timer_b, dma);
    // send_payload_b(Rotor::R4, timer_b, dma);
}

/// Set up the direction for each motor, in accordance with user config.
pub fn setup_motor_dir(
    motors_reversed: (bool, bool, bool, bool),
    timer_a: &mut Timer<TIM2>,
    timer_b: &mut Timer<TIM3>,
    dma: &mut Dma<DMA1>,
) {
    // todo: DRY
    if motors_reversed.0 {
        setup_payload(Rotor::R1, CmdType::Command(Command::SpinDirReversed));
    } else {
        setup_payload(Rotor::R1, CmdType::Command(Command::SpinDirNormal));
    }

    if motors_reversed.1 {
        setup_payload(Rotor::R2, CmdType::Command(Command::SpinDirReversed));
    } else {
        setup_payload(Rotor::R2, CmdType::Command(Command::SpinDirNormal));
    }

    if motors_reversed.2 {
        setup_payload(Rotor::R3, CmdType::Command(Command::SpinDirReversed));
    } else {
        setup_payload(Rotor::R3, CmdType::Command(Command::SpinDirNormal));
    }

    if motors_reversed.3 {
        setup_payload(Rotor::R4, CmdType::Command(Command::SpinDirReversed));
    } else {
        setup_payload(Rotor::R4, CmdType::Command(Command::SpinDirNormal));
    }

    send_payload_a(timer_a, dma);
    send_payload_b(timer_b, dma);
}

/// Update our DSHOT payload for a given rotor, with a given power level.
pub fn setup_payload(rotor: Rotor, cmd: CmdType) {
    // First 11 (0:10) bits are the throttle settings. 0 means disarmed. 1-47 are reserved
    // for special commands. 48 - 2_047 are throttle value (2_000 possible values)

    // Bit 11 is 1 to request telemetry; 0 otherwise.
    // Bits 12:15 are CRC, to validate data.

    let data_word = match cmd {
        CmdType::Command(c) => c as u16,
        CmdType::Power(pwr) => (pwr * 1_999.) as u16 + 48,
    };

    let telemetry_bit = 1; // todo temp
    let packet = (data_word << 1) | telemetry_bit;

    // Compute the checksum
    let crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    let mut packet = (packet << 4) | crc;

    // todo: method on rotor for payload?
    let (payload, offset) = unsafe {
        match rotor {
            Rotor::R1 => (&mut PAYLOAD_R1_2, 0),
            Rotor::R2 => (&mut PAYLOAD_R1_2, 1),
            Rotor::R3 => (&mut PAYLOAD_R3_4, 0),
            Rotor::R4 => (&mut PAYLOAD_R3_4, 1),
        }
    };

    // Create a DMA payload of 16 timer CCR (duty) settings, each for one bit of our data word.
    for i in 0..16 {
        let bit = (packet >> i) & 1;
        let val = if bit == 1 { DUTY_HIGH } else { DUTY_LOW };
        // DSHOT uses MSB first alignment.
        // Values alternate in the buffer between the 2 registers we're editing, so
        // we interleave values here. (Each timer and DMA stream is associated with 2 channels).
        payload[(15 - i) * 2 + offset] = val;
    }
}

/// Set an individual rotor's power, using a 16-bit DHOT word, transmitted over DMA via timer CCR (duty)
/// settings. `power` ranges from 0. to 1.
pub fn set_power_a(
    rotor1: Rotor,
    rotor2: Rotor,
    power1: f32,
    power2: f32,
    timer: &mut Timer<TIM2>,
    dma: &mut Dma<DMA1>,
) {
    setup_payload(rotor1, CmdType::Power(power1));
    setup_payload(rotor2, CmdType::Power(power2));

    send_payload_a(timer, dma)
}

// todo: DRY due to type issue. Use a trait?
pub fn set_power_b(
    rotor1: Rotor,
    rotor2: Rotor,
    power1: f32,
    power2: f32,
    timer: &mut Timer<TIM3>,
    dma: &mut Dma<DMA1>,
) {
    setup_payload(rotor1, CmdType::Power(power1));
    setup_payload(rotor2, CmdType::Power(power2));

    send_payload_b(timer, dma)
}

/// Send the stored payload for timer A. (2 channels).
/// Make sure, in the timer's ISR, you disable the timer.
fn send_payload_a(timer: &mut Timer<TIM2>, dma: &mut Dma<DMA1>) {
    // let payload = &unsafe {
    //     match rotor {
    //         Rotor::R1 => PAYLOAD_R1_2,
    //         Rotor::R2 => PAYLOAD_R1_2,
    //         Rotor::R3 => PAYLOAD_R3_4,
    //         Rotor::R4 => PAYLOAD_R3_4,
    //     }
    // };

    let payload = unsafe { &PAYLOAD_R1_2 };

    unsafe {
        timer.write_dma_burst(
            payload,
            Rotor::R1.base_addr_offset(),
            2, // Burst len of 2, since we're updating 2 channels.
            Rotor::R1.dma_channel(),
            Default::default(),
            dma,
        );
    }

    // Note that timer enabling is handled by `write_dma_burst`.

    // todo: Do we need to enable DMA here, or should it already be enabled?

    // todo: Do we want this?
    // Reset and update Timer registers
    // timer.regs.egr.write(|w| w.ug().set_bit());
}

// todo: DRY again. Trait?
/// Send the stored payload for timer B. (2 channels)
fn send_payload_b(timer: &mut Timer<TIM3>, dma: &mut Dma<DMA1>) {
    // let payload = &unsafe {
    //     match rotor {
    //         Rotor::R1 => PAYLOAD_R1_2,
    //         Rotor::R2 => PAYLOAD_R1_2,
    //         Rotor::R3 => PAYLOAD_R3_4,
    //         Rotor::R4 => PAYLOAD_R3_4,
    //     }
    // };
    //
    let payload = unsafe { &PAYLOAD_R3_4 };
    let base_addr_offset = Rotor::R3.base_addr_offset();

    unsafe {
        timer.write_dma_burst(
            payload,
            Rotor::R3.base_addr_offset(),
            2, // Burst len of 2, since we're updating 2 channels.
            Rotor::R3.dma_channel(),
            Default::default(),
            dma,
        );
    }

    // Reset and update Timer registers
    // timer.regs.egr.write(|w| w.ug().set_bit());
}
