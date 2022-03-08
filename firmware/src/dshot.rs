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
//!
//! The DSHOT protocol (DSHOT-300, DSHOT-600 etc) is determined by the `DSHOT_ARR` and `DSHOT_PSC` settings in the
//! main crate; ie set a 600kHz countdown for DSHOT-600.

use stm32_hal2::{
    dma::{ChannelCfg, Dma, DmaChannel},
    pac::{DMA1, TIM2, TIM3},
    timer::{TimChannel, Timer},
};

use crate::flight_ctrls::Rotor;

// Duty cycle values (to be written to CCMRx), based on our ARR value. 0. = 0%. ARR = 100%.
const DUTY_HIGH: u32 = crate::DSHOT_ARR * 3 / 4;
const DUTY_LOW: u32 = crate::DSHOT_ARR * 3 / 8;

// DMA buffers for each rotor. 16-bit data, but using a 32-bit API.
static mut PAYLOAD_R1: [u32; 16] = [0; 16];
static mut PAYLOAD_R2: [u32; 16] = [0; 16];
static mut PAYLOAD_R3: [u32; 16] = [0; 16];
static mut PAYLOAD_R4: [u32; 16] = [0; 16];

/// Possible DSHOT commands (ie, DSHOT values 0 - 47
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
    Led0On = 22, // BLHeli32 only
    Led1On = 23, // BLHeli32 only
    Led2On = 24, // BLHeli32 only
    Led3On = 25, // BLHeli32 only
    Led0Off = 26, // BLHeli32 only
    Led1Off = 27, // BLHeli32 only
    Led2Off = 28, // BLHeli32 only
    Led3Off = 29, // BLHeli32 only
    AudioStreamModeOnOff = 30, // KISS audio Stream mode on/Off
    SilendModeOnOff = 31, // KISS silent Mode on/Off
    Max = 47
}

enum CmdType {
    Command(Command),
    Power(f32),
}

pub fn stop_all(timer_a: &mut Timer<TIM2>, timer_b: &mut Timer<TIM3>) {
    setup_payload(Rotor::R1, CmdType::Command(Command::MotorStop));
    setup_payload(Rotor::R2, CmdType::Command(Command::MotorStop));
    setup_payload(Rotor::R3, CmdType::Command(Command::MotorStop));
    setup_payload(Rotor::R4, CmdType::Command(Command::MotorStop));

    // todo: Make sure you have the right motors here.
    send_payload_a(Rotor::R1, timer_a, dma: &mut Dma<DMA1>);
    send_payload_a(Rotor::R2, timer_a, dma: &mut Dma<DMA1>);
    send_payload_b(Rotor::R3, timer_b, dma: &mut Dma<DMA1>);
    send_payload_b(Rotor::R4, timer_b, dma: &mut Dma<DMA1>);
}

/// Update our DSHOT payload for a given rotor, with a given power level.
pub fn setup_payload(rotor: Rotor, cmd: CmdType) {
    // First 11 (0:10) bits are the throttle settings. 0 means disarmed. 1-47 are reserved
    // for special commands. 48 - 2_047 are throttle value (2_000 possible values)

    // Bit 11 is 1 to request telemetry; 0 otherwise.
    // Bits 12:15 are CRC, to validate data.

    let data_word = match cmd {
        CmdType::Command(c) => c,
        CmdType::Power(pwr) => (pwr * 1_999.) as u16 + 48,
    };

    let telemetry_bit = 1; // todo temp
    let packet = (data_word << 1) | telemetry_bit;

    // Compute the checksum
    let crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    let mut packet = (packet << 4) | crc;

    // todo: method on rotor for payload?
    let payload = &mut unsafe {
        match rotor {
            Rotor::R1 => PAYLOAD_R1,
            Rotor::R2 => PAYLOAD_R2,
            Rotor::R3 => PAYLOAD_R3,
            Rotor::R4 => PAYLOAD_R4,
        }
    };

    // Create a DMA payload of 16 timer CCR (duty) settings, each for one bit of our data word.
    for i in 0..16 {
        //
        let bit = (packet >> i) & 1;
        unsafe {
            // DSHOT uses MSB first alignment.
            payload[15 - i] = if bit == 1 { DUTY_HIGH } else { DUTY_LOW };
        }
    }
}

/// Set an individual rotor's power, using a 16-bit DHOT word, transmitted over DMA via timer CCR (duty)
/// settings. `power` ranges from 0. to 1.
pub fn set_power_a(rotor: Rotor, power: f32, timer: &mut Timer<TIM2>, dma: &mut Dma<DMA1>) {
    setup_payload(rotor, CmdType::Power(power));

    send_payload_a(rotor, timer, dma: &mut Dma<DMA1>)
}

// todo: DRY due to type issue. Use a trait?
pub fn set_power_b(rotor: Rotor, power: f32, timer: &mut Timer<TIM3>, dma: &mut Dma<DMA1>) {
    setup_payload(rotor, CmdType::Power(power));

    send_payload_b(rotor, timer, dma: &mut Dma<DMA1>)
}

fn send_payload_a(rotor: Rotor, timer: &mut Timer<TIM2>, dma: &mut Dma<DMA1>) {
    let dma_cfg = ChannelCfg::Default();

    let payload = &unsafe {
        match rotor {
            Rotor::R1 => PAYLOAD_R1,
            Rotor::R2 => PAYLOAD_R2,
            Rotor::R3 => PAYLOAD_R3,
            Rotor::R4 => PAYLOAD_R4,
        }
    };

    unsafe {
        timer.write_dma_burst(
            payload,
            rotor.base_addr_offset(),
            1, // Burst len of 1
            rotor.dma_channel(),
            dma_cfg,
            dma,
        );
    }

    // Reset and update Timer registers
    timer.regs.egr.write(|w| w.ug().set_bit());
    // todo: DMA enable?
}

// todo: DRY again. Trait?
fn send_payload_b(rotor: Rotor, timer: &mut Timer<TIM3>, dma: &mut Dma<DMA1>) {
    let dma_cfg = ChannelCfg::Default();

    let payload = &unsafe {
        match rotor {
            Rotor::R1 => PAYLOAD_R1,
            Rotor::R2 => PAYLOAD_R2,
            Rotor::R3 => PAYLOAD_R3,
            Rotor::R4 => PAYLOAD_R4,
        }
    };

    unsafe {
        timer.write_dma_burst(
            payload,
            rotor.base_addr_offset(),
            1, // Burst len of 1
            rotor.dma_channel(),
            dma_cfg,
            dma,
        );
    }

    // Reset and update Timer registers
    timer.regs.egr.write(|w| w.ug().set_bit());
    // todo: DMA enable?
}

// void init(unsigned bitrate)
//   {
//     //auto bit_period_ = (TIM_RATE + bitrate/2) / bitrate;
//     //bit_0_ = (bit_period_ * 1 + 1) / 3 - 1;
//     //bit_1_ = (bit_period_ * 3 + 2) / 3 - 1;
//
//     // Set the payload end
//     payload_[16] = 0;
//
//     //
//     // Setup GPIO Port A, Pin 8, AF01 (Pulse output)
//     //
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                // Enable GPIOA clock
//     GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);       // AF01
//     GPIOA->MODER |= (2 << GPIO_MODER_MODE8_Pos);        // Alternate function
//     GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED8_Pos);  // Fast mode
//
//
//     //
//     // Setup DMA
//     //
//
//     // DMA2 clock enable
//     RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
//
//     // Configure DMA2 Stream5 CR register
//     // Set CHSEL bits according to DMA Channel 6
//     // Set DIR bits according to Memory to peripheral direction
//     // Set PINC bit according to DMA Peripheral Increment Disable
//     // Set MINC bit according to DMA Memory Increment Enable
//     // Set PSIZE bits according to Peripheral DataSize Word
//     // Set MSIZE bits according to Memory DataSize Word
//     // Set CIRC bit according to disable circular mode
//     // Set PL bits according to very high priority
//     // Set MBURST bits according to single memory burst
//     // Set PBURST bits according to single peripheral burst
//     DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) |
//                        (1 << DMA_SxCR_DIR_Pos) |
//                        (1 << DMA_SxCR_MINC_Pos) |
//                        (0 << DMA_SxCR_PINC_Pos) |
//                        (2 << DMA_SxCR_MSIZE_Pos) |
//                        (2 << DMA_SxCR_PSIZE_Pos) |
//                        (3 << DMA_SxCR_PL_Pos);
//
//     // Write Timer DMAR address
//     DMA2_Stream5->PAR = reinterpret_cast<uintptr_t>(&TIM1->DMAR);
//
//     // Set the address to the memory buffer
//     DMA2_Stream5->M0AR = reinterpret_cast<uintptr_t>(payload_.data());
//
//     //
//     // Setup timer PWM mode
//     //
//
//     // Enable clock
//     RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//
//     // Reset settings
//     TIM1->CR1 = 0;
//     TIM1->CR2 = 0;
//
//     // Configure the period
//     constexpr const auto bit_period_ = (TIM_RATE + 1200000/2) / 1200000;
//     TIM1->ARR = bit_period_ - 1;
//
//     // Configure the Timer prescaler
//     const auto div = 1200000 / bitrate - 1;
//     TIM1->PSC = div;
//
//     // Configure pulse width
//     TIM1->CCR1 = 0;
//
//     // Enable auto-reload Preload
//     TIM1->CR1 |= TIM_CR1_ARPE;
//
//     //
//     // Set PWM mode
//     //
//
//     // Select the output compare mode 1
//     // Enable output compare 1 Preload
//     TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) |
//                   (1 << TIM_CCMR1_OC1PE_Pos);
//
//     // Enable the TIM1 Main Output
//     TIM1->BDTR = TIM_BDTR_MOE;
//
//     // Enable CC1 output
//     TIM1->CCER = TIM_CCER_CC1E;
//
//     //
//     // Setup Timer DMA settings
//     //
//
//     // Configure of the DMA Base register to CCR1 and the DMA Burst Length to 1
//     TIM1->DCR = (0 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);
//
//     // TIM1 DMA Update enable
//     TIM1->DIER |= TIM_DIER_UDE;
//
//     // Enable the TIM Counter
//     TIM1->CR1 |= TIM_CR1_CEN;
//   }
// };
