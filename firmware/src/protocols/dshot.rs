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
    gpio,
};

use defmt::println;

use cfg_if::cfg_if;

use crate::Rotor;

// The frequency our motor-driving PWM operates at, in Hz.
// todo: Make this higher (eg 96kHz) after making sure the ESC
// const PWM_FREQ: f32 = 12_000.;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately for DSHOT.
// These are set for a 200MHz timer frequency.
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period.

// todo: On H7, will we get more precision with 400mhz tim clock, vice 200? Is that possible?

// Set up for DSHOT-600. (600k bits/second) So, timer frequency = 600kHz.
// todo: (PSC = 0, AAR = 332 results in a 600.6kHz update freq; not 600kHz exactly. Is that ok?)
// todo: Is this even what we want?

cfg_if! {
    if #[cfg(feature = "mercury-h7")] {
        pub const DSHOT_PSC: u32 = 0;
        pub const DSHOT_ARR: u32 = 332;
    } else if #[cfg(feature = "mercury-g4")] {
        // 170Mhz tim clock. Results in 600.707kHz.
        pub const DSHOT_PSC: u16 = 0;
        pub const DSHOT_ARR: u16 = 282;
    }
}

// Duty cycle values (to be written to CCMRx), based on our ARR value. 0. = 0%. ARR = 100%.
const DUTY_HIGH: u16 = DSHOT_ARR * 3 / 4;
const DUTY_LOW: u16 = DSHOT_ARR * 3 / 8;

// DSHOT-600
pub const DSHOT_FREQ: f32 = 600_000.;

// DMA buffers for each rotor. 16-bit data. Note that
// rotors 1/2 and 3/4 share a timer, so we can use the same DMA stream with them. Data for the 2
// channels are interleaved.
static mut PAYLOAD_R1_2: [u16; 32] = [0; 32];
static mut PAYLOAD_R3_4: [u16; 32] = [0; 32];

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

pub enum CmdType {
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

static TEST_PAYLOAD: [u16; 4] = [DUTY_LOW, DUTY_LOW, DUTY_LOW, DUTY_LOW]; // todo temp
static TEST_PAYLOAD2: [u16; 4] = [DUTY_LOW, DUTY_LOW, DUTY_LOW, DUTY_LOW]; // todo temp

/// Send the stored payload for timer A. (2 channels).
fn send_payload_a(timer: &mut Timer<TIM2>, dma: &mut Dma<DMA1>) {
    let payload = unsafe { &PAYLOAD_R1_2 };

    // println!("payload: {}", payload);

    dma.stop(Rotor::R1.dma_channel()); // todo: Shouldn't be required?

    // todo: TSing G4's ISR vs mux etc.


    unsafe {
        timer.write_dma_burst(
            // payload,
            &TEST_PAYLOAD,
            Rotor::R1.base_addr_offset(),
            2, // Burst len of 2, since we're updating 2 channels.
            Rotor::R1.dma_channel(),
            Default::default(),
            dma,
        );
    }

    // Note that timer enabling is handled by `write_dma_burst`.

    // Reset and update Timer registers // todo: Do we want this?
    // timer.regs.egr.write(|w| w.ug().set_bit());
}

// todo: DRY again. Trait?
/// Send the stored payload for timer B. (2 channels)
fn send_payload_b(timer: &mut Timer<TIM3>, dma: &mut Dma<DMA1>) {
    let payload = unsafe { &PAYLOAD_R3_4 };

    dma.stop(Rotor::R3.dma_channel()); // todo: Shouldn't be required?

    unsafe {
        timer.write_dma_burst(
            // payload,
            &TEST_PAYLOAD2,
            Rotor::R3.base_addr_offset(),
            2,
            Rotor::R3.dma_channel(),
            Default::default(),
            dma,
        );
    }
}

/// An alternative approach to sending payloads, using GPIO DMA.
pub fn _send_payloads_bitbang(dma: &mut Dma<DMA1>) {

    // let payload = unsafe { &PAYLOAD_R1_2 };

    // todo test, using PA0 pin:
    let payload = &[1<<0, 1<<16, 1<<0, 1<<16, 1<<0, 1<<16, 1<<0, 1<<16];

    unsafe {
        gpio::write_dma(
            payload,
            gpio::Port::A,
            Rotor::R1.dma_channel(),
            Default::default(),
            dma,
        );
    }

    // Note that timer enabling is handled by `write_dma_burst`.

    // Reset and update Timer registers // todo: Do we want this?
    // timer.regs.egr.write(|w| w.ug().set_bit());
}
