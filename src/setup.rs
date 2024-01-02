//! This module contains setup code, including hardware-specific details like pin numbers,
//! and timer and DMA assigments. Makes use of feature-gating as required to support both the G4
//! and H7 flight controller.
//!
//! This module is the source of definitions of Buses, binding busses named after use cases to
//! specific hardware STM32 peripherals.

use ahrs::{ppks::PositVelEarthUnits, Params};
use cfg_if::cfg_if;
use fdcan::{FdCan, NormalOperationMode};
// #[cfg(feature = "h7")]
// use stm32_hal2::qspi::{Qspi, QspiConfig};
#[cfg(feature = "fixed-wing")]
use stm32_hal2::timer::OutputCompare;
use stm32_hal2::{
    can::Can,
    clocks::Clocks,
    delay_ms,
    dma::{self, DmaChannel, DmaInput, DmaInterrupt, DmaPeriph},
    gpio::{Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, I2C1, I2C2, SPI1, SPI2, USART1, USART2},
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::{BasicTimer, MasterModeSelection, TimChannel, Timer, TimerConfig, TimerInterrupt},
    usart::{Usart, UsartConfig},
};

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::protocols::servo::ServoWing;
    }
}
use defmt::println;
use stm32_hal2::usart::UsartInterrupt;

#[cfg(feature = "g4")]
use crate::drivers::{spi2_kludge::Spi2, uart4_kludge::Usart4};
use crate::{
    atmos_model::AltitudeCalPt,
    crsf,
    drivers::{
        baro_dps310 as baro,
        flash_spi,
        // tof_vl53l1 as tof,
        imu_icm426xx as imu,
    },
    // flight_ctrls::common::Motor,
    protocols::{
        dshot::{self, Motor},
        msp, servo,
    },
    safety,
    sensors_shared,
    system_status::{SensorStatus, SystemStatus},
};

// Keep all DMA channel number bindings in this code block, to make sure we don't use duplicates.

pub const IMU_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
pub const MOTORS_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
pub const CRSF_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
pub const BATT_CURR_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;

pub const BARO_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma2;
pub const OSD_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma2;
pub const EXT_SENSORS_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma2;

// DMA 1
pub const IMU_TX_CH: DmaChannel = DmaChannel::C1;
pub const IMU_RX_CH: DmaChannel = DmaChannel::C2;

// Note: DMA1, C4 is unused.
pub const MOTOR_CH: DmaChannel = DmaChannel::C3;

pub const CRSF_RX_CH: DmaChannel = DmaChannel::C5;
// pub const CRSF_TX_CH: DmaChannel = DmaChannel::C6; // Note: Unused

pub const BATT_CURR_DMA_CH: DmaChannel = DmaChannel::C7;

// DMA 2
pub const BARO_TX_CH: DmaChannel = DmaChannel::C1;
pub const BARO_RX_CH: DmaChannel = DmaChannel::C2;

pub const OSD_TX_CH: DmaChannel = DmaChannel::C3;
// pub const OSD_RX_CH: DmaChannel = DmaChannel::C4;

pub const MOTORS_DMA_INPUT: DmaInput = DmaInput::Tim3Up;

// Used for commanding timer DMA, for DSHOT protocol. Maps to CCR1, and is incremented
// automatically when we set burst len = 4 in the DMA write and read.
// Calculate by taking the Adddress Offset for the associated CCR channel in the
// RM register table, and dividing by 4.
pub const DSHOT_BASE_DIR_OFFSET: u8 = 0x34 / 4;

#[cfg(feature = "g4")]
pub const AHB_FREQ: u32 = 170_000_000;
#[cfg(feature = "h7")]
pub const AHB_FREQ: u32 = 240_000_000; // todo: Check this!

// Update frequency: 600kHz
// 170Mhz tim clock on G4.
// 240Mhz tim clock on H743
// 260Mhz tim clock on H723 @ 520Mhz. 275Mhz @ 550Mhz

cfg_if! {
    if #[cfg(feature = "h7")] {
        // pub const TIM_CLK: u32 = 260_000_000; // Hz. H723 @ 550Mhz
        // pub const TIM_CLK: u32 = 275_000_000; // Hz.  H723 @ 520Mhz
        pub const TIM_CLK_SPEED: u32 = 240_000_000; // Hz.  H743
        pub const DSHOT_SPEED: u32 = 600_000; // Hz.
        // todo: What should this be on H7?
        pub const DSHOT_ARR_READ: u32 = 17_000; // 17k for DSHOT300

    } else if #[cfg(feature = "g4")] {
        pub const TIM_CLK_SPEED: u32 = 170_000_000;
        pub const DSHOT_SPEED: u32 = 300_000; // Hz.
        // todo: How should thsi be set up?
        // todo: Uhoh - getting a weird stagger past 14.5k or so where starts jittering
        // todo between increase and decrease?
        pub const DSHOT_ARR_READ: u32 = 17_000; // 17k for DSHOT300
    }
}

cfg_if! {
    if #[cfg(feature = "h7")] {
        // todo: USB2 on H743; USB1 on H723.
        // use stm32_hal2::usb_otg::Usb1BusType as UsbBusType;
        pub use stm32_hal2::usb_otg::Usb2BusType as UsbBusType;
    } else {
        pub use stm32_hal2::usb::UsbBusType;
    }
}

// Code shortener to isolate typestate syntax.
pub type Can_ = FdCan<Can, NormalOperationMode>;

// Define types for peripheral buses here; call these types from driver modules.
pub type MotorTimer = Timer<pac::TIM3>;
pub type ServoTimer = Timer<pac::TIM8>; // Valid for H7 on all channels. Valid for G4 on Ch 1, 3, 4.
pub type SpiImu = Spi<SPI1>;
pub type I2cBaro = I2c<I2C2>;
pub type I2cMag = I2c<I2C1>; // Currently unused.
pub type SpiPacFlash = pac::SPI2;

cfg_if! {
    if #[cfg(feature = "h7")] {
        type UartCrsfRegs = pac::UART7;
        type UartOsdRegs = pac::USART2;
        // type SpiPacFlash = pac::OCTOSPI1;
        // pub type SpiPacFlash = pac::QUADSPI;
        // pub type SpiFlash = Qspi;
        pub type SpiFlash = Spi<SpiPacFlash>;
        pub type UartCrsf = Usart<pac::UART7>;
        pub type UartOsd = Usart<pac::USART2>;
    } else {
        type UartCrsfRegs = pac::USART2;
        type UartOsdRegs = pac::UART4;
        // pub type SpiPacFlash = pac::SPI2;
        pub type SpiFlash = Spi2<SpiPacFlash>;
        pub type UartCrsf = Usart<pac::USART2>;
        pub type UartOsd = Usart4<pac::UART4>;
    }
}

cfg_if! {
    if #[cfg(feature = "h7")] {
        pub const BATT_ADC_CH: u8 = 18;
        pub const CURR_ADC_CH: u8 = 16;
    } else {
        pub const BATT_ADC_CH: u8 = 2;
        pub const CURR_ADC_CH: u8 = 12;
    }
}

/// Run on startup, or when desired. Run on the ground. Gets an initial GPS fix,
/// and other initialization functions. We currently use the sensor initialization
/// bus communication results here to detemrine how to set system status flags.
/// todo: Periodically check these sensor statuses after init.
pub fn init_sensors(
    params: &mut Params,
    base_pt: &mut PositVelEarthUnits,
    spi1: &mut Spi<SPI1>,
    spi_flash: &mut SpiFlash,
    i2c_mag: &mut I2cMag,
    i2c_baro: &mut I2cBaro,
    cs_imu: &mut Pin,
    cs_flash: &mut Pin,
    clock_cfg: &Clocks,
) -> (SystemStatus, baro::Altimeter) {
    let mut system_status = SystemStatus::default();

    match imu::setup(spi1, cs_imu) {
        Ok(_) => system_status.imu = SensorStatus::Pass,
        Err(_) => system_status.imu = SensorStatus::NotConnected,
    };

    // match mag::setup(i2c1) {
    //     Ok(_) => system_status.magnetometer = SensorStatus::Pass,
    //     Err(_) => system_status.magnetometer = SensorStatus::NotConnected,
    // }

    match flash_spi::setup(spi_flash, cs_flash) {
        Ok(_) => system_status.flash_spi = SensorStatus::Pass,
        Err(_) => system_status.flash_spi = SensorStatus::NotConnected,
    }

    let altimeter = match baro::Altimeter::new(i2c_baro) {
        Ok(mut alt) => {
            system_status.baro = SensorStatus::Pass;
            alt
        }
        Err(_) => {
            system_status.baro = SensorStatus::NotConnected;
            Default::default()
        }
    };

    // let fix = gps::get_fix(uart1);
    // match fix {
    //     Ok(f) => {
    //         params.lon = f.lon;
    //         params.lat = f.lat;
    //         params.alt_msl_baro = f.alt_msl;
    //
    //         *base_pt = Location::new(LocationType::LatLon, f.lat, f.lon, f.alt_msl);
    //
    //         if system_status.baro == SensorStatus::Pass {
    //             altimeter.calibrate_from_gps(Some(f.alt_msl), i2c2).ok();
    //         }
    //     }
    //     Err(_) => {
    //         if system_status.baro == SensorStatus::Pass {
    //             altimeter.calibrate_from_gps(None, i2c2).ok();
    //         }
    //     }
    // }

    println!("Setup compl");
    // todo: Use Rel0 location type if unable to get fix.
    (system_status, altimeter)
}

impl Motor {
    /// This same channel arrangement applies for H7 and G4.
    pub fn tim_channel(&self) -> TimChannel {
        match self {
            Self::M1 => TimChannel::C1,
            Self::M2 => TimChannel::C2,
            Self::M3 => TimChannel::C3,
            Self::M4 => TimChannel::C4,
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
    // Rotors connected to Tim3 CH1-4, or Tim8 (ch 1-4 on H7)

    // todo: For configuring H7 fixed wing with a third servo and 1 motor, you need

    // FOr the breakdown of these timers by MCU and aircraft type, see the `MotorTimers` struct.

    let alt_motors = 2; // TIM3

    let mut motor1 = Pin::new(Port::C, 6, PinMode::Alt(alt_motors)); // Ch1

    cfg_if! {
        if #[cfg(feature = "h7")] {
            // On H7, we TIM3 and TIM8 have full overlap as ch 1-4 for our timer pins.
            let alt_servos = 3; // TIM8 (Avail on all channels)

            // todo: Let us customize; set motor2 as `alt_servos` if equipped with a rudder etc.
            let mut motor2 = Pin::new(Port::C, 7, PinMode::Alt(alt_motors)); // Ch2
            let mut motor3 = Pin::new(Port::C, 8, PinMode::Alt(alt_servos)); // Ch3
            let mut motor4 = Pin::new(Port::C, 9, PinMode::Alt(alt_servos)); // Ch4
        } else {
            // Servos: CH1 for Motor 1 pin, and CH2N/3N for Motor 3 and 4 pin.
            let alt_servos = 4; //  TIM8 (Avail on channels 1, 3, and 4.(TIM1 also avail on channels 3 and 4: AF6)

            let mut motor2 = Pin::new(Port::A, 4, PinMode::Alt(alt_motors)); // Ch2
            let mut motor3 = Pin::new(Port::B, 0, PinMode::Alt(alt_motors)); // Ch3
            let mut motor4 = Pin::new(Port::B, 1, PinMode::Alt(alt_motors)); // Ch4
        }
    }

    // Enable interrupts on both edges for the pins, for use with reading RPM. Then mask the
    // interrupt. This performs some extra setup, then lets us enable and disable the interrupt
    // by masking and unmasking using imr1.

    if dshot::BIDIR_EN {
        motor1.enable_interrupt(Edge::Either);
        motor2.enable_interrupt(Edge::Either);
        motor3.enable_interrupt(Edge::Either);
        motor4.enable_interrupt(Edge::Either);

        let exti = unsafe { &(*pac::EXTI::ptr()) };
        cfg_if! {
            if #[cfg(feature = "h7")] {
                exti.cpuimr1.modify(|_, w| {
                    w.mr6().clear_bit();
                    w.mr7().clear_bit();
                    w.mr8().clear_bit();
                    w.mr9().clear_bit()
                });
            } else {
                exti.imr1.modify(|_, w| {
                    w.im6().clear_bit();
                    w.im4().clear_bit();
                    w.im0().clear_bit();
                    w.im1().clear_bit()
                });
            }
        }
    }

    // todo: What should this be?: Low is good up to the Mhz range, which is good enough?
    let dshot_gpiospeed = OutputSpeed::Low; // Note: Low is the default value.

    motor1.output_speed(dshot_gpiospeed);
    motor2.output_speed(dshot_gpiospeed);
    motor3.output_speed(dshot_gpiospeed);
    motor4.output_speed(dshot_gpiospeed);

    // SPI1 for the IMU. Nothing else on the bus, since we use it with DMA
    let mut sck1 = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut miso1 = Pin::new(Port::A, 6, PinMode::Alt(5));
    let mut mosi1 = Pin::new(Port::A, 7, PinMode::Alt(5));

    // Depending on capacitance, med or high should be appropriate for SPI speeds.
    // High means sharper edges, which also may mean more interference.
    let spi_gpiospeed = OutputSpeed::Medium;

    sck1.output_speed(spi_gpiospeed);
    miso1.output_speed(spi_gpiospeed);
    mosi1.output_speed(spi_gpiospeed);

    // todo TS batt ADC on H7 board.
    // let batt_v_adc = Pin::new(Port::A, 4, PinMode::Input); // ADC12, channel 18
    // loop {
    //     println!("High: {:?}", batt_v_adc.is_high());
    //     delay_ms(500, 200_000_000);
    // }

    cfg_if! {
        if #[cfg(feature = "h7")] {
            let _batt_v_adc = Pin::new(Port::A, 4, PinMode::Analog); // ADC12, channel 18
            let _current_sense_adc = Pin::new(Port::A, 0, PinMode::Analog); // ADC1, channel 16

            // let qspi_sck = Pin::new(Port::B, 2, PinMode::Alt(9));
            // let qspi_nss = Pin::new(Port::E, 11, PinMode::Alt(11));
            // let io0 = Pin::new(Port::D, 11, PinMode::Alt(9));
            // let io1 = Pin::new(Port::D, 12, PinMode::Alt(9));
            // let io2 = Pin::new(Port::E, 2, PinMode::Alt(9));
            // let io3 = Pin::new(Port::D, 13, PinMode::Alt(9));

            let mut sck2 = Pin::new(Port::A, 9, PinMode::Alt(5));
            let mut miso2 = Pin::new(Port::B, 14, PinMode::Alt(5));
            let mut mosi2 = Pin::new(Port::B, 15, PinMode::Alt(5));

            sck2.output_speed(spi_gpiospeed);
            miso2.output_speed(spi_gpiospeed);
            mosi2.output_speed(spi_gpiospeed);
        } else {
            let _batt_v_adc = Pin::new(Port::A, 1, PinMode::Analog); // ADC12, channel 1
            let _current_sense_adc = Pin::new(Port::B, 2, PinMode::Analog); // ADC2, channel 12

            let mut sck2 = Pin::new(Port::B, 13, PinMode::Alt(5));
            let mut miso2 = Pin::new(Port::B, 14, PinMode::Alt(5));
            let mut mosi2 = Pin::new(Port::B, 15, PinMode::Alt(5));

            sck2.output_speed(spi_gpiospeed);
            miso2.output_speed(spi_gpiospeed);
            mosi2.output_speed(spi_gpiospeed);
        }
    }

    cfg_if! {
        if #[cfg(feature = "h7")] {
            // We use Uart 7 for the onboard ELRS MCU.
            let _uart_crsf_tx = Pin::new(Port::B, 4, PinMode::Alt(11));
            let mut uart_crsf_rx = Pin::new(Port::B, 3, PinMode::Alt(11));
        } else {
            let pwr = unsafe { &(*pac::PWR::ptr()) };
            let rcc = unsafe { &(*pac::RCC::ptr()) };

            // Some or all of these lines are required to prevent strange
            // behavior of the ELRS MCU; likely due to JTAG behavior of PB3 and 4.
            rcc.apb1enr2.modify(|_, w| w.ucpd1en().set_bit());
            rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());
            rcc.apb1enr2.modify(|_, w| w.ucpd1en().set_bit());
            pwr.cr3.modify(|_, w| w.ucpd1_dbdis().set_bit());

            // We use UART 2 for ELRS on G4.
            let mut uart_crsf_tx = Pin::new(Port::B, 3, PinMode::Alt(7));
            let mut uart_crsf_rx = Pin::new(Port::B, 4, PinMode::Alt(7));

            // Undo settings on PB3 and PB4 that are due to initial debug-pin config.
            uart_crsf_tx.pull(Pull::Floating);
            uart_crsf_rx.pull(Pull::Floating);
            uart_crsf_tx.output_speed(OutputSpeed::Low);
            uart_crsf_rx.output_speed(OutputSpeed::Low);
        }
    }

    // Pull up the CRSF RX line. Without this, our idle interrupt fires spuratically in
    // some conditions, like when touching the (even outside) of the wires if an Rx
    // module isn't connected.
    uart_crsf_rx.pull(Pull::Up);

    let _uart_osd_tx = Pin::new(Port::C, 10, PinMode::Alt(5));
    let mut uart_osd_rx = Pin::new(Port::C, 11, PinMode::Alt(5));

    uart_osd_rx.pull(Pull::Up);

    // We use UARTs for misc external devices, including ESC telemetry,
    // and VTX OSD.

    // Used to trigger a PID update based on new IMU data.
    // We assume here the interrupt config uses default settings active low, push pull, pulsed.
    #[cfg(feature = "h7")]
    let mut imu_exti_pin = Pin::new(Port::B, 12, PinMode::Input);
    #[cfg(feature = "g4")]
    let mut imu_exti_pin = Pin::new(Port::C, 13, PinMode::Input);

    imu_exti_pin.output_type(OutputType::OpenDrain);
    imu_exti_pin.pull(Pull::Up);

    let imu_exti_edge = Edge::Falling;
    imu_exti_pin.enable_interrupt(imu_exti_edge);

    let i2c_alt = PinMode::Alt(4);

    cfg_if! {
        if #[cfg(feature = "h7")] {
            // I2C1 for external sensors, via pads
            let mut scl1 = Pin::new(Port::B, 8, i2c_alt);
            let mut sda1 = Pin::new(Port::B, 9, i2c_alt);

            // I2C2 for the DPS310 barometer, and pads.
            let mut scl2 = Pin::new(Port::B, 10, i2c_alt);
            let mut sda2 = Pin::new(Port::B, 11, i2c_alt);
        } else {
            let mut scl1 = Pin::new(Port::A, 15, i2c_alt);
            let mut sda1 = Pin::new(Port::B, 9, i2c_alt);

            let mut scl2 = Pin::new(Port::A, 9, i2c_alt);
            let mut sda2 = Pin::new(Port::A, 8, i2c_alt);
        }
    }

    scl2.pull(Pull::Up);
    sda2.pull(Pull::Up);

    // Note that we use hardware pullups, so don't enable the firmware pullups.
    sda1.output_type(OutputType::OpenDrain);
    scl1.output_type(OutputType::OpenDrain);
    sda2.output_type(OutputType::OpenDrain);
    scl2.output_type(OutputType::OpenDrain);

    // Configure CAN pins. Currently on the H7 board uses CAN.
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let mut can_rx = Pin::new(Port::D, 0, PinMode::Alt(9));
            let mut can_tx = Pin::new(Port::D, 1, PinMode::Alt(9));

            can_tx.output_speed(OutputSpeed::VeryHigh);
            can_rx.output_speed(OutputSpeed::VeryHigh);
        }
    }
}

/// Assign DMA channels to peripherals.
pub fn setup_dma() {
    #[cfg(feature = "g4")]
    dma::enable_mux1();

    // IMU
    dma::mux(IMU_DMA_PERIPH, IMU_TX_CH, DmaInput::Spi1Tx);
    dma::mux(IMU_DMA_PERIPH, IMU_RX_CH, DmaInput::Spi1Rx);

    // DSHOT, all 4 motors.
    dma::mux(MOTORS_DMA_PERIPH, MOTOR_CH, MOTORS_DMA_INPUT);

    // todo: This matrix is perhaps better suited to go with the consts at the top of this module.
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let adc_dma_ip = DmaInput::Adc1;
            let crsf_dma_ip = DmaInput::Uart7Rx;
            let osd_dma_ip = DmaInput::Usart2Tx;
            let osd_dma_rx_ip = DmaInput::Usart2Rx;
        } else {
            let crsf_dma_ip = DmaInput::Usart2Rx;
            let adc_dma_ip = DmaInput::Adc2;
            let osd_dma_ip = DmaInput::Uart4Tx;
            let osd_dma_rx_ip = DmaInput::Uart4Rx;
        }
    }

    dma::mux(CRSF_DMA_PERIPH, CRSF_RX_CH, crsf_dma_ip);
    // CRSF TX is currently unused.
    // dma::mux(CRSF_DMA_PERIPH, CRSF_TX_CH, DmaInput::Usart2Tx);
    dma::mux(BATT_CURR_DMA_PERIPH, BATT_CURR_DMA_CH, adc_dma_ip);
    dma::mux(OSD_DMA_PERIPH, OSD_TX_CH, osd_dma_ip);
    // dma::mux(OSD_DMA_PERIPH, OSD_RX_CH, osd_dma_rx_ip);

    dma::mux(BARO_DMA_PERIPH, BARO_TX_CH, DmaInput::I2c2Tx);
    dma::mux(BARO_DMA_PERIPH, BARO_RX_CH, DmaInput::I2c2Rx);

    // We use Spi transfer complete to know when our readings are ready - in its ISR,
    // we trigger the attitude-rates PID loop.
    dma::enable_interrupt(IMU_DMA_PERIPH, IMU_RX_CH, DmaInterrupt::TransferComplete);

    // It appears the timer `DmaUpdate`, interrupt, enabled in DSHOT setup code, will
    // auto-enable the transfer complete interrupts; that interrupt is required for
    // timer burst DMA to work. There's therefore no purpose in enabling TC explicitly here.

    // Enable TC interrupts for all I2C sections; we use this to sequence readings,
    // and store reading data.
    dma::enable_interrupt(BARO_DMA_PERIPH, BARO_TX_CH, DmaInterrupt::TransferComplete);
    dma::enable_interrupt(BARO_DMA_PERIPH, BARO_RX_CH, DmaInterrupt::TransferComplete);

    dma::enable_interrupt(OSD_DMA_PERIPH, OSD_TX_CH, DmaInterrupt::TransferComplete);
    // dma::enable_interrupt(OSD_DMA_PERIPH, OSD_RX_CH, DmaInterrupt::TransferComplete);
}

/// Configure the SPI and I2C busses.
pub fn setup_busses(
    spi1_pac: SPI1,
    spi_flash_pac: SpiPacFlash,
    i2c1_pac: I2C1,
    i2c2_pac: I2C2,
    uart_osd_pac: UartOsdRegs,
    uart_crsf_pac: UartCrsfRegs,
    clock_cfg: &Clocks,
) -> (
    Spi<SPI1>,
    SpiFlash,
    Pin,
    Pin,
    I2c<I2C1>,
    I2c<I2C2>,
    UartOsd,
    UartCrsf,
) {
    // We use SPI1 for the IMU
    // SPI input clock is 400MHz for H7, and 170Mhz for G4. 400MHz / 32 = 12.5 MHz. 170Mhz / 8 = 21.25Mhz.
    // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
    // 426xx can use any SPI mode. Maybe St is only mode 3? Not sure.

    cfg_if! {
        if #[cfg(feature = "h7")] {
            let mut cs_imu = Pin::new(Port::C, 4, PinMode::Output);

            // todo: Temp config of USB pins on H743. We don't need this on G4 or H723
            let _usb_dm = Pin::new(Port::A, 11, PinMode::Alt(10));
            let _usb_dp = Pin::new(Port::A, 12, PinMode::Alt(10));

            // On H7, we run PLLQ1 as the SPI1 clock (default).
            // Configured with Divq=8. H743: 120Mhz SPI clock if core clock is 480Mhz. 100Mhz if @400Mhz.
            // 120Mhz/8 = 15Mhz. 100Mhz / 8 = 12.5Mhz
            // For H723, using PLLQ1 with PLLQ = DIV8: (And DIVN = 260; we lower DIVP in this case)
            // 68.75Mhz at 550Mhz, and 65Mhz SPI at 520Mhz core. /16 = 17Mhz and 16Mhz respectively.
            // If H723 @ 400Mhz, use same settings as H743.
            // 120Mhz/8 = 15Mhz. 65Mhz/4 = 16.25Mhz. 68.75/4 = 17.2Mhz.
            let imu_baud_div = BaudRate::Div8; // H743
            // let imu_baud_div = BaudRate::Div4;  // H723 (Assuming not @ 400Mhz; if that, use Div8)
        } else {
            let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);

            // for ICM426xx, for 24Mhz limit. 170Mhz / 8 = ~21Mhz.
            let imu_baud_div = BaudRate::Div8;
        }
    }

    cs_imu.set_high();

    let imu_spi_cfg = SpiConfig {
        // Per ICM42688 and ISM330 DSs, only mode 3 is valid.
        mode: SpiMode::mode3(),
        ..Default::default()
    };

    let spi_imu = Spi::new(spi1_pac, imu_spi_cfg, imu_baud_div);

    // We use I2C1 for the GPS, magnetometer, and TOF sensor. Some details:
    // The U-BLOX GPS' max speed is 400kHz.
    // The LIS3MDL magnetometer's max speed is 400kHz.
    // 100kHz, vice 400k, due to the connection being external. (Lower freqs
    // may have fewer line-interference issues)
    let i2c_external_sensors_cfg = I2cConfig {
        // Lower speeds may work better on external runs, hence not 400khz here.
        speed: I2cSpeed::Standard100K,
        ..Default::default()
    };

    // We use I2C2 for the baro.
    let i2c_baro_cfg = I2cConfig {
        // We could go 1M (3.4Mhz is the DPS310 limit), but perhaps keeping signal speed low
        // will reduce RF interference.
        speed: I2cSpeed::Fast400K,
        ..Default::default()
    };

    // We use I2C1 for offboard sensors: Magnetometer, GPS, and TOF sensor.
    let i2c1 = I2c::new(i2c1_pac, i2c_external_sensors_cfg, clock_cfg);

    // We use I2C2 for the onboard barometer (altimeter).
    let i2c2 = I2c::new(i2c2_pac, i2c_baro_cfg, clock_cfg);

    // We use SPI2 for SPI flash on G4. On H7, we use octospi instead.
    // Max speed: 104Mhz (single, dual, or quad) for 8M variant, and 133Mhz for
    // 16M variant.
    // W25 flash chips use SPI mode 0 or 3.
    // cfg_if! {
    //     if #[cfg(feature = "h7")] {
    //         let flash_config = QspiConfig {
    //             // 240Mhz / 4 = 60Mhz.
    //             // clock_division: 4,
    //             clock_division: 16, // todo: TS by using a slower speed
    //             ..Default::default()
    //         };
    //         let mut spi_flash = Qspi::new(spi_flash_pac, flash_config, clock_cfg);
    //
    //         let mut cs_flash = Pin::new(Port::E, 11, PinMode::Output); // todo temp
    //         // Enable QSPI, in status regiver 2.
    //         cs_flash.set_high();
    //
    //         cs_flash.set_low();
    //         spi_flash.write_indirect(0x31, &[0b0000_0010]);
    //         cs_flash.set_high();
    //
    //         let mut read_buf = [0x9f, 0, 0, 0];
    //         // let mut read_buf = [0, 0, 0, 0];
    //         cs_flash.set_low();
    //         spi_flash.read_indirect(0x9f, &mut read_buf);
    //         // spi_flash.read_indirect(0x35, &mut read_buf);
    //         cs_flash.set_high();
    //
    //         println!("Read buf from QSPI flash: {:?}", read_buf);
    //
    //     } else {
    #[cfg(feature = "g4")]
    let spi_flash = Spi2::new(spi_flash_pac, Default::default(), BaudRate::Div2);
    #[cfg(feature = "h7")]
    let spi_flash = Spi::new(spi_flash_pac, Default::default(), BaudRate::Div2);
    // }
    // }

    #[cfg(feature = "h7")]
    let mut cs_flash = Pin::new(Port::E, 11, PinMode::Output);
    #[cfg(feature = "g4")]
    let mut cs_flash = Pin::new(Port::A, 0, PinMode::Output);

    cs_flash.set_high();

    let uart_cfg = UsartConfig {
        overrun_disabled: true, // Seems to be required
        ..Default::default()
    };

    // We use UART4 for the OSD, for DJI, via the MSP protocol.
    // todo: QC baud.
    #[cfg(feature = "h7")]
    let mut uart_osd = Usart::new(
        uart_osd_pac,
        crate::osd::BAUD,
        UsartConfig {
            overrun_disabled: true,
            ..UsartConfig::default()
        },
        clock_cfg,
    );

    #[cfg(feature = "g4")]
    let mut uart_osd = Usart4::new(
        uart_osd_pac,
        crate::osd::BAUD,
        UsartConfig {
            overrun_disabled: true,
            ..UsartConfig::default()
        },
        clock_cfg,
    );

    // uart_osd.enable_interrupt(UsartInterrupt::CharDetect(Some(msp::PREAMBLE_0)));
    // We only use this read to identify a status message, so we can reply with arming status
    // to enable the OSD to enter high power mode.
    uart_osd.enable_interrupt(UsartInterrupt::CharDetect(Some(msp::MSG_ID_STATUS)));
    // uart_osd.enable_interrupt(UsartInterrupt::Idle);

    // We use UART for the radio controller receiver, via CRSF protocol.

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

    //
    // •
    // When oversampling by 16, the baud rate ranges from usart_ker_ck_pres/65535 and
    // usart_ker_ck_pres/16.
    // •
    // When oversampling by 8, the baud rate ranges from usart_ker_ck_pres/65535 and
    // usart_ker_ck_pres/8.

    let uart_crsf = Usart::new(
        uart_crsf_pac,
        crsf::BAUD,
        UsartConfig {
            // We're having a struggle with overruns. Must leave disabled.
            overrun_disabled: true,
            ..Default::default()
        },
        clock_cfg,
    );

    (
        spi_imu, spi_flash, cs_imu, cs_flash, i2c1, i2c2, uart_osd, uart_crsf,
    )
}

/// Set up misc timers.
pub fn setup_timers(
    tim1_pac: pac::TIM1,
    // tim17_pac: pac::TIM17,
    tim5_pac: pac::TIM5,
    tim6_pac: pac::TIM6,
    clock_cfg: &Clocks,
) -> (
    Timer<pac::TIM1>,
    // Timer<pac::TIM17>,
    Timer<pac::TIM5>,
    BasicTimer<pac::TIM6>,
) {
    let ctrl_coeff_adj_timer = Timer::new_tim1(
        tim1_pac,
        1. / crate::CTRL_COEFF_ADJ_TIMEOUT,
        TimerConfig {
            one_pulse_mode: true,
            ..Default::default()
        },
        &clock_cfg,
    );

    // let mut lost_link_timer = Timer::new_tim17(
    //     tim17_pac,
    //     1. / safety::LOST_LINK_TIMEOUT,
    //     TimerConfig {
    //         one_pulse_mode: true,
    //         ..Default::default()
    //     },
    //     &clock_cfg,
    // );
    // lost_link_timer.enable_interrupt(TimerInterrupt::Update);

    // We use this timer to maintain a time since bootup.
    // A shorter timeout period will allow higher resolution measurements, while a longer one
    // will command an interrupt less often. (The interrupt only increments an atomic overflow counter).
    let mut tick_timer = Timer::new_tim5(
        tim5_pac,
        1. / crate::TICK_TIMER_PERIOD,
        Default::default(),
        &clock_cfg,
    );
    // todo: Maybe manually set ARR and PSC, since ARR is what controls precision?
    tick_timer.enable_interrupt(TimerInterrupt::Update);

    // Set up a basic timer that will trigger our ADC reads, at a fixed rate.
    // If you wish to sample at a fixed rate, consider using a basic timer (TIM6 or TIM7)
    let mut adc_timer = BasicTimer::new(tim6_pac, sensors_shared::ADC_SAMPLE_FREQ, &clock_cfg);

    // The update event is selected as a trigger output (TRGO). For instance a
    // master timer can then be used as a prescaler for a slave timer.
    adc_timer.set_mastermode(MasterModeSelection::Update);

    (ctrl_coeff_adj_timer, tick_timer, adc_timer)
}

/// Configures all 4 motor timers for quadcopters, or combinations of motors and servos
/// for fixed-wing
pub fn setup_motor_timers(motor_timer: &mut MotorTimer, servo_timer: &mut ServoTimer) {
    motor_timer.set_prescaler(dshot::PSC_DSHOT);
    motor_timer.set_auto_reload(dshot::ARR_DSHOT as u32);

    motor_timer.enable_interrupt(TimerInterrupt::UpdateDma);

    cfg_if! {
        if #[cfg(feature = "quad")] {
            dshot::set_to_output(motor_timer);
            dshot::set_bidirectional(dshot::BIDIR_EN, motor_timer);
        } else {
            servo_timer.set_prescaler(servo::PSC_SERVOS);
            servo_timer.set_auto_reload(servo::ARR_SERVOS);

            // Arbitrary duty cycle set, since we'll override it with DMA bursts for the motor, and
            // position settings for the servos.
            motor_timer.enable_pwm_output(Motor::M1.tim_channel(), OutputCompare::Pwm1, 0.);
            servo_timer.enable_pwm_output(ServoWing::S1.tim_channel(), OutputCompare::Pwm1, 0.);
            servo_timer.enable_pwm_output(ServoWing::S2.tim_channel(), OutputCompare::Pwm1, 0.);

            // PAC, since our HAL currently only sets this on `new`.
            servo_timer.regs.cr1.modify(|_, w| w.opm().set_bit()); // todo: Does this work?

            // Set servo pins to pull-up, to make sure they don't shorten a pulse on a MCU reset
            // or similar condition.
            // todo: #1: Don't hard-code these pins. #2: Consider if this is helping and/or sufficient.
            #[cfg(feature = "h7")]
            unsafe {
                (*pac::GPIOC::ptr()).pupdr.modify(|_, w| {
                    w.pupdr8().bits(0b01);
                    w.pupdr9().bits(0b01)
                });
            }
            #[cfg(feature = "g4")]
            unsafe {
                (*pac::GPIOB::ptr()).pupdr.modify(|_, w| {
                    w.pupdr0().bits(0b01);
                    w.pupdr1().bits(0b01)
                });
            }

            // Motor timer is enabled in Timer burst DMA. We enable the servo timer here.
            servo_timer.enable();
        }
    }
}
