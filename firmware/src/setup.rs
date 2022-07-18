//! This module contains setup code, including hardware-specific details like pin numbers,
//! and timer and DMA assigments. Makes use of feature-gating as required.

use cfg_if::cfg_if;

use crate::flight_ctrls::{flying_wing::ServoWing, quad::Motor};

use stm32_hal2::{
    dma::{self, Dma, DmaChannel, DmaInput, DmaInterrupt},
    gpio::{Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    pac::DMA1,
    timer::TimChannel,
};

#[cfg(feature = "g4")]
use stm32_hal2::pac::DMAMUX;
#[cfg(feature = "h7")]
use stm32_hal2::pac::DMAMUX1 as DMAMUX;

// Keep all DMA channel number bindings in this code block, to make sure we don't use duplicates.
pub const IMU_TX_CH: DmaChannel = DmaChannel::C1;
pub const IMU_RX_CH: DmaChannel = DmaChannel::C2;
#[cfg(feature = "g4")]
pub const MOTOR_CH_A: DmaChannel = DmaChannel::C3;
pub const MOTOR_CH_B: DmaChannel = DmaChannel::C4;
pub const CRSF_RX_CH: DmaChannel = DmaChannel::C5;
pub const CRSF_TX_CH: DmaChannel = DmaChannel::C6;

pub const BATT_CURR_CH: DmaChannel = DmaChannel::C7;

pub const BATT_ADC_CH: u8 = 17;
pub const CURR_ADC_CH: u8 = 12;

impl Motor {
    // todo: Feature gate these methods based on board, as required.
    pub fn tim_channel(&self) -> TimChannel {
        match self {
            Self::M1 => TimChannel::C1,
            Self::M2 => TimChannel::C2,
            Self::M3 => TimChannel::C3,
            Self::M4 => TimChannel::C4,
        }
    }

    /// Dma input channel. This should be in line with `tim_channel`.
    pub fn dma_input(&self) -> DmaInput {
        cfg_if! {
            if #[cfg(feature = "h7")] {
                match self {
                    Self::M1 => DmaInput::Tim3Up,
                    Self::M2 => DmaInput::Tim3Up,
                    Self::M3 => DmaInput::Tim3Up,
                    Self::M4 => DmaInput::Tim3Up,
                }
            } else {
                match self {
                    // The DMA write isn't associated with a channel; using the Update even seems to work.
                    Self::M1 => DmaInput::Tim2Up,
                    Self::M2 => DmaInput::Tim2Up,
                    Self::M3 => DmaInput::Tim3Up,
                    Self::M4 => DmaInput::Tim3Up,
                }
            }
        }
    }

    /// Used for commanding timer DMA, for DSHOT protocol. Maps to CCR1, 2, 3, or 4.
    pub fn dma_channel(&self) -> DmaChannel {
        #[cfg(feature = "h7")]
        return MOTOR_CH_B;

        #[cfg(feature = "g4")]
        match self {
            Self::M1 | Self::M2 => MOTOR_CH_A,
            Self::M3 | Self::M4 => MOTOR_CH_B,
        }
    }

    /// Used for commanding timer DMA, for DSHOT protocol. Maps to CCR1, 2, 3, or 4.
    /// Calculate by taking the Adddress Offset for the associated CCR channel in the
    /// RM register table, and dividing by 4.
    pub fn base_addr_offset(&self) -> u8 {
        #[cfg(feature = "h7")]
        return 13;

        #[cfg(feature = "g4")]
        match self.tim_channel() {
            TimChannel::C1 => 13, // CCR1
            TimChannel::C2 => 13, // CCR2 (starting with CCR1, burst len 2)
            TimChannel::C3 => 15, // CCR3
            TimChannel::C4 => 15, // CCR4 (starting with CCR3, burst len 2)
        }
    }
}

impl ServoWing {
    pub fn tim_channel(&self) -> TimChannel {
        match self {
            Self::S1 => TimChannel::C3,
            Self::S2 => TimChannel::C4,
        }
    }
}

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // Rotors connected to Tim2 CH3, 4; Tim3 ch3, 4
    cfg_if! {
        if #[cfg(feature = "mercury-h7")] {
            // todo: If flying wing on H7, set rotors 3 and 4 to Alt3, for TIM8 (same channels)
            let mut rotor1 = Pin::new(Port::C, 6, PinMode::Alt(2)); // Tim3 ch1
            let mut rotor2 = Pin::new(Port::C, 7, PinMode::Alt(2)); // Tim3 ch2
            let mut rotor3 = Pin::new(Port::C, 8, PinMode::Alt(2)); // Tim3 ch3
            let mut rotor4 = Pin::new(Port::C, 9, PinMode::Alt(2)); // Tim3 ch4
        } else {
            let mut rotor1 = Pin::new(Port::A, 0, PinMode::Alt(1)); // Tim2 ch1
            let mut rotor2 = Pin::new(Port::A, 1, PinMode::Alt(1)); // Tim2 ch2
            let mut rotor3 = Pin::new(Port::B, 0, PinMode::Alt(2)); // Tim3 ch3
            let mut rotor4 = Pin::new(Port::B, 1, PinMode::Alt(2)); // Tim3 ch4
        }
    }

    rotor1.output_speed(OutputSpeed::High);
    rotor2.output_speed(OutputSpeed::High);
    rotor3.output_speed(OutputSpeed::High);
    rotor4.output_speed(OutputSpeed::High);

    let _buzzer = Pin::new(Port::A, 10, PinMode::Alt(6)); // Tim1 ch3

    // todo: USB? How do we set them up (no alt fn) PA11(DN) and PA12 (DP).
    // let _usb_dm = Pin::new(Port::A, 11, PinMode::Output);
    // let _usb_dp = Pin::new(Port::A, 12, PinMode::Output);

    let batt_v_adc_ = Pin::new(Port::A, 4, PinMode::Analog); // ADC2, channel 17
    let current_sense_adc_ = Pin::new(Port::B, 2, PinMode::Analog); // ADC2, channel 12

    // SPI1 for the IMU. Nothing else on the bus, since we use it with DMA
    let mut sck1 = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut miso1 = Pin::new(Port::A, 6, PinMode::Alt(5));
    let mut mosi1 = Pin::new(Port::A, 7, PinMode::Alt(5));

    // todo: Output speed on SPI pins?
    sck1.output_speed(OutputSpeed::High);
    miso1.output_speed(OutputSpeed::High);
    mosi1.output_speed(OutputSpeed::High);

    // SPI2 for the LoRa chip
    let mut sck2 = Pin::new(Port::B, 13, PinMode::Alt(5));
    let mut miso2 = Pin::new(Port::B, 14, PinMode::Alt(5));
    let mut mosi2 = Pin::new(Port::B, 15, PinMode::Alt(5));

    // todo: Output speed on SPI pins?
    sck2.output_speed(OutputSpeed::High);
    miso2.output_speed(OutputSpeed::High);
    mosi2.output_speed(OutputSpeed::High);

    // SPI3 for flash
    cfg_if! {
        if #[cfg(feature = "h7")] {
            // Use use Uart 7 for the onboard ELRS MCU.
            let _uart7_tx = Pin::new(Port::B, 3, PinMode::Alt(11));
            let _uart7_rx = Pin::new(Port::B, 4, PinMode::Alt(11));
        } else {
            // G4 board uses onboard ELRS, on SPI3
            let _sck3 = Pin::new(Port::B, 3, PinMode::Alt(6));
            let _miso3 = Pin::new(Port::B, 4, PinMode::Alt(6));
            let _mosi3 = Pin::new(Port::B, 5, PinMode::Alt(6));
        }
    }

    // We use UARTs for misc external devices, including ESC telemetry,
    // and VTX OSD.

    // let _uart1_tx = Pin::new(Port::B, 6, PinMode::Alt(7));
    // let _uart1_rx = Pin::new(Port::B, 7, PinMode::Alt(7));
    // let _uart2_tx = Pin::new(Port::A, 2, PinMode::Alt(7));
    // let _uart2_rx = Pin::new(Port::A, 3, PinMode::Alt(7));
    // let _uart3_tx = Pin::new(Port::B, 10, PinMode::Alt(7));
    // let _uart3_rx = Pin::new(Port::B, 11, PinMode::Alt(7));
    // let _uart4_tx = Pin::new(Port::C, 10, PinMode::Alt(7));
    // let _uart4_rx = Pin::new(Port::C, 11, PinMode::Alt(7));

    // Used to trigger a PID update based on new IMU data.
    // We assume here the interrupt config uses default settings active low, push pull, pulsed.
    #[cfg(feature = "mercury-h7")]
    let mut imu_interrupt = Pin::new(Port::B, 12, PinMode::Input);
    #[cfg(feature = "mercury-g4")]
    let mut imu_interrupt = Pin::new(Port::C, 4, PinMode::Input);

    imu_interrupt.output_type(OutputType::OpenDrain);
    imu_interrupt.pull(Pull::Up);
    imu_interrupt.enable_interrupt(Edge::Falling);

    // ELRS busy and DIO currently in the main fn.
    //
    // // Used to trigger a a control-data-received update based on new ELRS data.
    // let mut elrs_busy = Pin::new(Port::C, 14, PinMode::Input);
    // elrs_busy.output_type(OutputType::OpenDrain);
    // elrs_busy.pull(Pull::Up);
    // elrs_busy.enable_interrupt(Edge::Falling);
    //
    // let mut elrs_dio = Pin::new(Port::C, 14, PinMode::Input);
    // elrs_busy.output_type(OutputType::OpenDrain);
    // elrs_busy.pull(Pull::Up);
    // elrs_busy.enable_interrupt(Edge::Falling);

    // I2C1 for external sensors, via pads
    let mut scl1 = Pin::new(Port::A, 15, PinMode::Alt(4));
    scl1.output_type(OutputType::OpenDrain);
    scl1.pull(Pull::Up);

    let mut sda1 = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda1.output_type(OutputType::OpenDrain);
    sda1.pull(Pull::Up);

    // I2C2 for the DPS310 barometer, and pads.
    let mut scl2 = Pin::new(Port::A, 9, PinMode::Alt(4));
    scl2.output_type(OutputType::OpenDrain);
    scl2.pull(Pull::Up);

    let mut sda2 = Pin::new(Port::A, 8, PinMode::Alt(4));
    sda2.output_type(OutputType::OpenDrain);
    sda2.pull(Pull::Up);
}

/// Assign DMA channels to peripherals.
pub fn setup_dma(dma: &mut Dma<DMA1>, mux: &mut DMAMUX) {
    // IMU
    dma::mux(IMU_TX_CH, DmaInput::Spi1Tx, mux);
    dma::mux(IMU_RX_CH, DmaInput::Spi1Rx, mux);

    // DSHOT, motors 1 and 2 (all 4 for H7)
    #[cfg(feature = "g4")]
    dma::mux(Motor::M1.dma_channel(), Motor::M1.dma_input(), mux);

    // DSHOT, motors 3 and 4 (not used on H7)
    #[cfg(not(feature = "h7"))]
    dma::mux(Motor::M3.dma_channel(), Motor::M3.dma_input(), mux);

    // CRSF (onboard ELRS)
    #[cfg(feature = "h7")]
    let elrs_dma_ch = DmaInput::Usart7Rx;
    #[cfg(feature = "g4")]
    let elrs_dma_ch = DmaInput::Usart3Rx;
    dma::mux(CRSF_RX_CH, elrs_dma_ch, mux);
    // Note: If we run out of DMA channels, consider removing the CRSF transmit channel;
    // we only have it set up to respond to pings, and that's probably unecessary.
    // dma::mux(DmaChannel::C8, DmaInput::Usart3Tx, mux);

    dma::mux(BATT_CURR_CH, DmaInput::Adc2, mux);

    // TOF sensor
    // dma::mux(DmaChannel::C4, dma::DmaInput::I2c2Tx, &mut dp.DMAMUX);
    // dma::mux(DmaChannel::C5, dma::DmaInput::I2c2Rx, &mut dp.DMAMUX);

    // We use Spi transfer complete to know when our readings are ready - in its ISR,
    // we trigger the attitude-rates PID loop.
    dma.enable_interrupt(IMU_RX_CH, DmaInterrupt::TransferComplete);

    // We use Dshot transfer-complete interrupts to disable the timer.
    dma.enable_interrupt(Motor::M1.dma_channel(), DmaInterrupt::TransferComplete);
    #[cfg(not(feature = "h7"))]
    dma.enable_interrupt(Motor::M3.dma_channel(), DmaInterrupt::TransferComplete);
}
