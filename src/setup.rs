//! This module contains setup code, including hardware-specific details like pin numbers,
//! and timer and DMA assigments. Makes use of feature-gating as required.

use cfg_if::cfg_if;

use cortex_m::delay::Delay;

use stm32_hal2::{
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaInput, DmaInterrupt, DmaPeriph},
    gpio::{Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, DMA1, DMA2, I2C1, I2C2, SPI1, USART2},
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::TimChannel,
    usart::Usart,
};

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::{ServoWing, ServoWingPosition};
    } else {
    }
}
use crate::{
    drivers::{
        baro_dps310 as baro, gps_ublox as gps, imu_icm426xx as imu, mag_lis3mdl as mag,
        tof_vl53l1 as tof,
    },
    flight_ctrls::common::{Motor, Params},
    ppks::{Location, LocationType},
    state::{SensorStatus, SystemStatus},
};

// Keep all DMA channel number bindings in this code block, to make sure we don't use duplicates.

// DMA 1
pub const IMU_TX_CH: DmaChannel = DmaChannel::C1;
pub const IMU_RX_CH: DmaChannel = DmaChannel::C2;

#[cfg(feature = "g4")]
pub const MOTOR_CH_A: DmaChannel = DmaChannel::C3;
pub const MOTOR_CH_B: DmaChannel = DmaChannel::C4;

pub const CRSF_RX_CH: DmaChannel = DmaChannel::C5;
pub const CRSF_TX_CH: DmaChannel = DmaChannel::C6;

pub const BATT_CURR_DMA_CH: DmaChannel = DmaChannel::C7;

// We still have CH8 (or CH0) avail on DMA1)

// todo: You can possibly share channels here for Tx and Rx for each fo baro, ext,
// todo since the xfers are not active at once. This would free up channels,
// but require more logic to determine wheather read or write fired for ext sensors.

// DMA 2
pub const BARO_TX_CH: DmaChannel = DmaChannel::C1;
pub const BARO_RX_CH: DmaChannel = DmaChannel::C2;

// Channels for GPS, magnetometer, and TOF sensor.
pub const EXT_SENSORS_TX_CH: DmaChannel = DmaChannel::C3;
pub const EXT_SENSORS_RX_CH: DmaChannel = DmaChannel::C4;

/// Run on startup, or when desired. Run on the ground. Gets an initial GPS fix,
/// and other initialization functions. We currently use the sensor initialization
/// bus communication results here to detemrine how to set system status flags.
/// todo: Periodically check these sensor statuses after init.
pub fn init_sensors(
    params: &mut Params,
    base_pt: &mut Location,
    spi1: &mut Spi<SPI1>,
    i2c1: &mut I2c<I2C1>,
    i2c2: &mut I2c<I2C2>,
    cs_imu: &mut Pin,
    delay: &mut Delay,
) -> (SystemStatus, baro::Altimeter) {
    let mut system_status = SystemStatus::default();

    let eps = 0.001;

    match gps::setup(i2c1) {
        Ok(_) => system_status.gps = SensorStatus::Pass,
        Err(_) => system_status.gps = SensorStatus::NotConnected,
    }

    match mag::setup(i2c1) {
        Ok(_) => system_status.magnetometer = SensorStatus::Pass,
        Err(_) => system_status.magnetometer = SensorStatus::NotConnected,
    }

    match tof::setup(i2c1) {
        Ok(_) => system_status.tof = SensorStatus::Pass,
        Err(_) => system_status.tof = SensorStatus::NotConnected,
    }

    match imu::setup(spi1, cs_imu, delay) {
        Ok(_) => system_status.imu = SensorStatus::Pass,
        Err(_) => system_status.imu = SensorStatus::NotConnected,
    };

    // if let Some(agl) = tof::read(params.quaternion, i2c1) {
    //     if agl > 0.01 {
    //         return result;
    //     }
    // }

    let mut altimeter = match baro::Altimeter::new(i2c2) {
        Ok(a) => {
            system_status.baro = SensorStatus::Pass;
            a
        }
        Err(_) => {
            system_status.baro = SensorStatus::NotConnected;
            Default::default()
        }
    };

    let fix = gps::get_fix(i2c1);

    match fix {
        Ok(f) => {
            params.lon = f.lon;
            params.lat = f.lat;
            params.baro_alt_msl = f.alt_msl;

            *base_pt = Location::new(LocationType::LatLon, f.lat, f.lon, f.alt_msl);

            altimeter.calibrate_from_gps(Some(f.alt_msl), i2c2);
        }
        Err(_) => {
            altimeter.calibrate_from_gps(None, i2c2);
        }
    }
    // todo: Use Rel0 location type if unable to get fix.

    (system_status, altimeter)
}

cfg_if! {
    if #[cfg(feature = "h7")] {
        pub const OSD_CH: DmaChannel = DmaChannel::C0;
        pub const BATT_ADC_CH: u8 = 18;
        pub const CURR_ADC_CH: u8 = 16;
    } else {
        pub const OSD_CH: DmaChannel = DmaChannel::C8;
        pub const BATT_ADC_CH: u8 = 17;
        pub const CURR_ADC_CH: u8 = 12;
    }
}

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

#[cfg(feature = "fixed-wing")]
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
    // G4: Rotors connected to Tim2 CH3, 4; Tim3 ch3, 4
    // H7: Rotors connected to Tim3 CH1-4, or Tim8 ch 1-4
    cfg_if! {
        if #[cfg(feature = "mercury-h7")] {
            if #[cfg(feature = "fixed-wing")] {
                let alt = 3; // TIM8
            } else {
                let alt = 2; // TIM3
            }
            let mut rotor1 = Pin::new(Port::C, 6, PinMode::Alt(alt)); // Tim3/8 ch1
            let mut rotor2 = Pin::new(Port::C, 7, PinMode::Alt(alt)); // Tim3/8 ch2
            let mut rotor3 = Pin::new(Port::C, 8, PinMode::Alt(alt)); // Tim3/8 ch3
            let mut rotor4 = Pin::new(Port::C, 9, PinMode::Alt(alt)); // Tim3/8 ch4
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

    // SPI1 for the IMU. Nothing else on the bus, since we use it with DMA
    let mut sck1 = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut miso1 = Pin::new(Port::A, 6, PinMode::Alt(5));
    let mut mosi1 = Pin::new(Port::A, 7, PinMode::Alt(5));

    // todo: Output speed on SPI pins?
    sck1.output_speed(OutputSpeed::High);
    miso1.output_speed(OutputSpeed::High);
    mosi1.output_speed(OutputSpeed::High);

    // SPI2 for the LoRa chip on G4; OctoSPI1 (in Quad mode) on H7.
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let _batt_v_adc = Pin::new(Port::A, 4, PinMode::Analog); // ADC12, channel 18
            let _current_sense_adc = Pin::new(Port::A, 0, PinMode::Analog); // ADC1, channel 16

            let qspi_sck = Pin::new(Port::B, 2, PinMode::Alt(9));
            let qspi_nss = Pin::new(Port::E, 11, PinMode::Alt(11));
            let io0 = Pin::new(Port::D, 11, PinMode::Alt(9));
            let io1 = Pin::new(Port::D, 12, PinMode::Alt(9));
            let io2 = Pin::new(Port::B, 13, PinMode::Alt(4));
            let io3 = Pin::new(Port::D, 13, PinMode::Alt(9));
        } else {
            let _batt_v_adc = Pin::new(Port::A, 4, PinMode::Analog); // ADC2, channel 17
            let _current_sense_adc = Pin::new(Port::B, 2, PinMode::Analog); // ADC2, channel 12

            let sck2 = Pin::new(Port::B, 13, PinMode::Alt(5));
            let miso2 = Pin::new(Port::B, 14, PinMode::Alt(5));
            let mosi2 = Pin::new(Port::B, 15, PinMode::Alt(5));
        }
    }

    // SPI3 for flash
    cfg_if! {
        if #[cfg(feature = "h7")] {
            // Use use Uart 7 for the onboard ELRS MCU.
            let _uart7_tx = Pin::new(Port::B, 3, PinMode::Alt(11));
            let _uart7_rx = Pin::new(Port::B, 4, PinMode::Alt(11));
        } else {
            // // G4 board uses onboard ELRS, on SPI3
            // let _sck3 = Pin::new(Port::B, 3, PinMode::Alt(6));
            // let _miso3 = Pin::new(Port::B, 4, PinMode::Alt(6));
            // let _mosi3 = Pin::new(Port::B, 5, PinMode::Alt(6));
        }
    }

    // We use UARTs for misc external devices, including ESC telemetry,
    // and VTX OSD.

    // let _uart1_tx = Pin::new(Port::B, 6, PinMode::Alt(7));
    // let _uart1_rx = Pin::new(Port::B, 7, PinMode::Alt(7));
    // let _uart2_tx = Pin::new(Port::A, 2, PinMode::Alt(7));
    // let _uart2_rx = Pin::new(Port::A, 3, PinMode::Alt(7));
    // let _uart3_tx = Pin::new(Port::D, 8, PinMode::Alt(7));
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
pub fn setup_dma(dma: &mut Dma<DMA1>, dma2: &mut Dma<DMA2>) {
    // IMU
    dma::mux(DmaPeriph::Dma1, IMU_TX_CH, DmaInput::Spi1Tx);
    dma::mux(DmaPeriph::Dma1, IMU_RX_CH, DmaInput::Spi1Rx);

    // DSHOT, motors 1 and 2 (all 4 for H7)
    #[cfg(feature = "g4")]
    dma::mux(
        DmaPeriph::Dma1,
        Motor::M1.dma_channel(),
        Motor::M1.dma_input(),
    );

    // DSHOT, motors 3 and 4 (not used on H7)
    #[cfg(not(feature = "h7"))]
    dma::mux(
        DmaPeriph::Dma1,
        Motor::M3.dma_channel(),
        Motor::M3.dma_input(),
    );

    // CRSF (onboard ELRS)
    #[cfg(feature = "h7")]
    let elrs_dma_ch = DmaInput::Uart7Rx;
    #[cfg(feature = "g4")]
    let elrs_dma_ch = DmaInput::Usart3Rx;
    dma::mux(DmaPeriph::Dma1, CRSF_RX_CH, elrs_dma_ch);

    // Note: If we run out of DMA channels, consider removing the CRSF transmit channel;
    // we only have it set up to respond to pings, and that's probably unecessary.
    // dma::mux(DmaChannel::C8, DmaInput::Usart3Tx);

    #[cfg(feature = "h7")]
    dma::mux(DmaPeriph::Dma1, BATT_CURR_DMA_CH, DmaInput::Adc1);
    #[cfg(feature = "g4")]
    dma::mux(DmaPeriph::Dma1, BATT_CURR_DMA_CH, DmaInput::Adc2);

    dma::mux(DmaPeriph::Dma1, OSD_CH, DmaInput::Usart2Tx);

    dma::mux(DmaPeriph::Dma2, BARO_TX_CH, DmaInput::I2c2Tx);
    dma::mux(DmaPeriph::Dma2, BARO_RX_CH, DmaInput::I2c2Rx);

    dma::mux(DmaPeriph::Dma2, EXT_SENSORS_TX_CH, DmaInput::I2c1Tx);
    dma::mux(DmaPeriph::Dma2, EXT_SENSORS_RX_CH, DmaInput::I2c1Rx);

    // TOF sensor
    // dma::mux(DmaChannel::C4, dma::DmaInput::I2c2Tx);
    // dma::mux(DmaChannel::C5, dma::DmaInput::I2c2Rx);

    // We use Spi transfer complete to know when our readings are ready - in its ISR,
    // we trigger the attitude-rates PID loop.
    dma.enable_interrupt(IMU_RX_CH, DmaInterrupt::TransferComplete);

    // We use Dshot transfer-complete interrupts to disable the timer.
    dma.enable_interrupt(Motor::M1.dma_channel(), DmaInterrupt::TransferComplete);
    #[cfg(not(feature = "h7"))]
    dma.enable_interrupt(Motor::M3.dma_channel(), DmaInterrupt::TransferComplete);

    // Enable TC interrupts for all I2C sections; we use this to sequence readings,
    // and store reading data.
    dma.enable_interrupt(BARO_TX_CH, DmaInterrupt::TransferComplete);
    dma.enable_interrupt(BARO_RX_CH, DmaInterrupt::TransferComplete);
    dma.enable_interrupt(EXT_SENSORS_TX_CH, DmaInterrupt::TransferComplete);
    dma.enable_interrupt(EXT_SENSORS_RX_CH, DmaInterrupt::TransferComplete);
}

#[cfg(feature = "h7")]
type UART_ELRS_REGS = pac::UART7;
#[cfg(feature = "g4")]
type UART_ELRS_REGS = pac::USART3;

/// Configure the SPI and I2C busses.
pub fn setup_busses(
    spi1_pac: SPI1,
    i2c1_pac: I2C1,
    i2c2_pac: I2C2,
    uart2_pac: USART2,
    uart_elrs_pac: UART_ELRS_REGS,
    clock_cfg: &Clocks,
) -> (
    Spi<SPI1>,
    Pin,
    Pin,
    I2c<I2C1>,
    I2c<I2C2>,
    Usart<USART2>,
    Usart<crate::UART_ELRS>,
) {
    // We use SPI1 for the IMU
    // SPI input clock is 400MHz for H7, and 170Mhz for G4. 400MHz / 32 = 12.5 MHz. 170Mhz / 8 = 21.25Mhz.
    // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
    // 426xx can use any SPI mode. Maybe St is only mode 3? Not sure.
    #[cfg(feature = "g4")]
    // todo: Switch to higher speed.
    let imu_baud_div = BaudRate::Div8; // for ICM426xx, for 24Mhz limit
                                       // todo: 10 MHz max SPI frequency for intialisation? Found in BF; confirm in RM.
                                       // let imu_baud_div = BaudRate::Div32; // 5.3125 Mhz, for ISM330 10Mhz limit
    #[cfg(feature = "h7")]
    let imu_baud_div = BaudRate::Div32; // for ICM426xx, for 24Mhz limit
                                        // let imu_baud_div = BaudRate::Div64; // 6.25Mhz for ISM330 10Mhz limit

    let imu_spi_cfg = SpiConfig {
        // Per ICM42688 and ISM330 DSs, only mode 3 is valid.
        mode: SpiMode::mode3(),
        ..Default::default()
    };

    let spi1 = Spi::new(spi1_pac, imu_spi_cfg, imu_baud_div);

    #[cfg(feature = "mercury-h7")]
    let mut cs_imu = Pin::new(Port::C, 4, PinMode::Output);
    #[cfg(feature = "mercury-g4")]
    let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);

    cs_imu.set_high();

    // We use I2C1 for the GPS, magnetometer, and TOF sensor. Some details:
    // The U-BLOX GPS' max speed is 400kHz.
    // The LIS3MDL altimeter's max speed is 400kHz.
    // todo: DO you want 100kHz due to the connection being external? (Lower freqs
    // todo may have fewer issues)
    let i2c_external_sensors_cfg = I2cConfig {
        // Lower speeds may work better on external runs, hence not 400khz here.
        speed: I2cSpeed::Standard100K,
        ..Default::default()
    };

    // We use I2C for the TOF sensor.(?)
    let i2c_baro_cfg = I2cConfig {
        speed: I2cSpeed::Fast400K,
        ..Default::default()
    };

    // We use I2C1 for offboard sensors: Magnetometer, GPS, and TOF sensor.
    let i2c1 = I2c::new(i2c1_pac, i2c_external_sensors_cfg, clock_cfg);

    // We use I2C2 for the onboard barometer (altimeter).
    let i2c2 = I2c::new(i2c2_pac, i2c_baro_cfg, clock_cfg);

    // We use SPI3 for SPI flash on G4. On H7, we use octospi instead.
    // todo: Find max speed and supported modes.
    // todo: Commented this out due to a HAL/PAC limitation.
    // cfg_if! {
    //     if #[cfg(feature = "h7")] {
    //         let spi_flash = Octospi::new(dp.OCTOSPI, Default::default(), BaudRate::Div32);
    //
    //     } else {
    //         // todo: HAL issue where SPI needs to cast as SPI1
    //         let spi_flash = Spi::new(dp.SPI3 as SPI1, Default::default(), BaudRate::Div32);
    //         // let spi_flash = Spi::new(dp.SPI3, Default::default(), BaudRate::Div32);
    //     }
    // }

    #[cfg(feature = "h7")]
    let flash_pin = 10;
    #[cfg(not(feature = "h7"))]
    let flash_pin = 6;

    let mut cs_flash = Pin::new(Port::C, flash_pin, PinMode::Output);
    cs_flash.set_high();

    // We use UART2 for the OSD, for DJI, via the MSP protocol.
    // todo: QC baud.
    let uart_osd = Usart::new(uart2_pac, 115_200, Default::default(), clock_cfg);

    // We use `uart1` for the radio controller receiver, via CRSF protocol.
    // CRSF protocol uses a single wire half duplex uart connection.
    //  * The master sends one frame every 4ms and the slave replies between two frames from the master.
    //  *
    //  * 420000 baud
    //  * not inverted
    //  * 8 Bit
    //  * 1 Stop bit
    //  * Big endian
    //  * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
    //  * Max frame size is 64 bytes
    //  * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.

    // The STM32-HAL default UART config includes stop bits = 1, parity disabled, and 8-bit words,
    // which is what we want.

    let uart_elrs = Usart::new(uart_elrs_pac, 420_000, Default::default(), clock_cfg);

    (spi1, cs_imu, cs_flash, i2c1, i2c2, uart_osd, uart_elrs)
}
