//! This module contains setup code, including hardware-specific details like pin numbers,
//! and timer and DMA assigments. Makes use of feature-gating as required.

use cfg_if::cfg_if;
use defmt::println;

use crate::Rotor;

use stm32_hal2::{
    dma::{self, Dma, DmaChannel, DmaInput, DmaInterrupt},
    gpio::{Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    pac::{DMA1, DMAMUX},
    timer::TimChannel,
};

impl Rotor {
    // todo: Feature gate these methods based on board, as required.
    pub fn tim_channel(&self) -> TimChannel {
        match self {
            Self::R1 => TimChannel::C1,
            Self::R2 => TimChannel::C2,
            Self::R3 => TimChannel::C3,
            Self::R4 => TimChannel::C4,
        }
    }

    /// Dma input channel. This should be in line with `tim_channel`.
    pub fn dma_input(&self) -> DmaInput {
        match self {
            // The DMA write isn't associated with a channel; using the Update even seems to work.
            Self::R1 => DmaInput::Tim2Up,
            Self::R2 => DmaInput::Tim2Up,
            Self::R3 => DmaInput::Tim3Up,
            Self::R4 => DmaInput::Tim3Up,
        }
    }

    /// Used for commanding timer DMA, for DSHOT protocol. Maps to CCR1, 2, 3, or 4.
    pub fn dma_channel(&self) -> DmaChannel {
        match self {
            Self::R1 => DmaChannel::C3,
            Self::R2 => DmaChannel::C3,
            Self::R3 => DmaChannel::C4,
            Self::R4 => DmaChannel::C4,
        }
    }

    /// Used for commanding timer DMA, for DSHOT protocol. Maps to CCR1, 2, 3, or 4.
    /// Calculate by taking the Adddress Offset for the associated CCR channel in the
    /// RM register table, and dividing by 4.
    pub fn base_addr_offset(&self) -> u8 {
        match self.tim_channel() {
            TimChannel::C1 => 13, // CCR1
            TimChannel::C2 => 13, // CCR2 (starting with CCR1, burst len 2)
            TimChannel::C3 => 15, // CCR3
            TimChannel::C4 => 15, // CCR4 (starting with CCR3, burst len 2)
        }
    }
}

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // SAI pins to accept input from the 4 PDM ICs, using SAI1, and 4, both blocks.
    // We use the same SCK and FS clocks for all 4 ICs.

    cfg_if! {
        if #[cfg(feature = "mercury-h7")] {
            // todo
        } else if #[cfg(feature = "mercury-g4")] {
            // Rotors connected to Tim2 CH3, 4; Tim3 ch3, 4
            let mut rotor1 = Pin::new(Port::A, 0, PinMode::Alt(1)); // Tim2 ch1
            let mut rotor2 = Pin::new(Port::A, 1, PinMode::Alt(1)); // Tim2 ch2
            let mut rotor3 = Pin::new(Port::B, 0, PinMode::Alt(2)); // Tim3 ch3
            let mut rotor4 = Pin::new(Port::B, 1, PinMode::Alt(2)); // Tim3 ch4

            rotor1.output_speed(OutputSpeed::High);
            rotor2.output_speed(OutputSpeed::High);
            rotor3.output_speed(OutputSpeed::High);
            rotor4.output_speed(OutputSpeed::High);

            let _buzzer = Pin::new(Port::A, 10, PinMode::Alt(6)); // Tim1 ch3

            // todo: USB? How do we set them up (no alt fn) PA11(DN) and PA12 (DP).
            let _usb_dm = Pin::new(Port::A, 11, PinMode::Output);
            let _usb_dp = Pin::new(Port::A, 12, PinMode::Output);

            let batt_v_adc_ = Pin::new(Port::A, 4, PinMode::Analog);  // ADC2, channel 17
            let current_sense_adc_ = Pin::new(Port::B, 2, PinMode::Analog);  // ADC2, channel 12

            // SPI1 for the IMU. Nothing else on the bus, since we use it with DMA
            let _sck1 = Pin::new(Port::A, 5, PinMode::Alt(5));
            let _miso1 = Pin::new(Port::A, 6, PinMode::Alt(5));
            let _mosi1 = Pin::new(Port::A, 7, PinMode::Alt(5));

            // todo: Output speed on SPI pins?

            // SPI2 for the LoRa chip
            let _sck2 = Pin::new(Port::B, 13, PinMode::Alt(5));
            let _miso2 = Pin::new(Port::B, 14, PinMode::Alt(5));
            let _mosi2 = Pin::new(Port::B, 15, PinMode::Alt(5));

            // SPI3 for flash
            let _sck3 = Pin::new(Port::B, 3, PinMode::Alt(6));
            let _miso3 = Pin::new(Port::B, 4, PinMode::Alt(6));
            let _mosi3 = Pin::new(Port::B, 5, PinMode::Alt(6));


            // We use UARTs for misc external devices, including ESC telemetry,
            // and VTX OSD.

            let _uart1_tx = Pin::new(Port::B, 6, PinMode::Alt(7));
            let _uart1_rx = Pin::new(Port::B, 7, PinMode::Alt(7));
            let _uart2_tx = Pin::new(Port::A, 2, PinMode::Alt(7));
            let _uart2_rx = Pin::new(Port::A, 3, PinMode::Alt(7));
            let _uart3_tx = Pin::new(Port::B, 10, PinMode::Alt(7));
            let _uart3_rx = Pin::new(Port::B, 11, PinMode::Alt(7));
            let _uart4_tx = Pin::new(Port::C, 10, PinMode::Alt(7));
            let _uart4_rx = Pin::new(Port::C, 11, PinMode::Alt(7));

            // Used to trigger a PID update based on new IMU data.
            // We assume here the interrupt config uses default settings active low, push pull, pulsed.
            let mut imu_interrupt = Pin::new(Port::C, 4, PinMode::Input); // PA4 for IMU interrupt.
            imu_interrupt.output_type(OutputType::OpenDrain);
            imu_interrupt.pull(Pull::Up);
            imu_interrupt.enable_interrupt(Edge::Falling);

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
    }
}

/// Assign DMA channels to peripherals.
pub fn setup_dma(dma: &mut Dma<DMA1>, mux: &mut DMAMUX) {
    // IMU
    dma::mux(DmaChannel::C1, DmaInput::Spi1Tx, mux);
    dma::mux(DmaChannel::C2, DmaInput::Spi1Rx, mux);
    // dma::mux(DmaChannel::C2, DmaInput::Dac1Ch1, mux); // todo temp TS DMA!

    // todo: Give you're using burst DMA here, one channel per timer, how does this work?

    // DSHOT, motors 1 and 2
    dma::mux(Rotor::R1.dma_channel(), Rotor::R1.dma_input(), mux);

    // DSHOT, motors 3 and 4
    dma::mux(Rotor::R3.dma_channel(), Rotor::R3.dma_input(), mux);

    // TOF sensor
    // dma::mux(DmaChannel::C4, dma::DmaInput::I2c2Tx, &mut dp.DMAMUX);
    // dma::mux(DmaChannel::C5, dma::DmaInput::I2c2Rx, &mut dp.DMAMUX);

    // We use Spi transfer complete to know when our readings are ready - in its ISR,
    // we trigger the attitude-rates PID loop.
    dma.enable_interrupt(DmaChannel::C2, DmaInterrupt::TransferComplete);

    // We use Dshot transfer-complete interrupts to disable the timer.
    dma.enable_interrupt(Rotor::R1.dma_channel(), DmaInterrupt::TransferComplete);
    // dma.enable_interrupt(DmaChannel::C5, DmaInterrupt::TransferComplete);
    dma.enable_interrupt(Rotor::R3.dma_channel(), DmaInterrupt::TransferComplete);
}
