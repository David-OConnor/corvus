//! This module contains setup code, including hardware-specific details like pin numbers,
//! and timer and DMA assigments. Makes use of feature-gating as required.

use cfg_if::cfg_if;

use cortex_m::delay::Delay;

use stm32_hal2::{
    clocks::Clocks,
    dma::{self, Dma, DmaChannel, DmaInput, DmaInterrupt, DmaPeriph},
    gpio::{Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, DMA1, DMA2, I2C1, I2C2, SPI1, SPI2, USART2},
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::{OutputCompare, TimChannel, Timer, TimerInterrupt},
    usart::{OverSampling, Usart, UsartConfig},
};

#[cfg(feature = "h7")]
use stm32_hal2::qspi::Qspi;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::{ServoWing, ServoWingPosition};
    }
}
use crate::{
    atmos_model::AltitudeCalPt,
    crsf,
    drivers::{
        baro_dps310 as baro, flash_spi, gps_ublox as gps, imu_icm426xx as imu, mag_lis3mdl as mag,
        tof_vl53l1 as tof,
    },
    dshot,
    flight_ctrls::common::Motor,
    params::Params,
    ppks::{Location, LocationType},
    system_status::{SensorStatus, SystemStatus},
};

#[cfg(feature = "g4")]
use crate::drivers::spi2_kludge::Spi2;

use defmt::println;

// Keep all DMA channel number bindings in this code block, to make sure we don't use duplicates.

pub const IMU_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
pub const MOTORS_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
pub const CRSF_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;
pub const BATT_CURR_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1;

pub const BARO_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma2;
pub const EXT_SENSORS_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma2;
pub const OSD_DMA_PERIPH: DmaPeriph = DmaPeriph::Dma2;

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

// Channels for GPS, magnetometer, and TOF sensor.
pub const EXT_SENSORS_TX_CH: DmaChannel = DmaChannel::C3;
pub const EXT_SENSORS_RX_CH: DmaChannel = DmaChannel::C4;
pub const OSD_CH: DmaChannel = DmaChannel::C5;

pub const MOTORS_DMA_INPUT: DmaInput = DmaInput::Tim3Up;

/// Used for commanding timer DMA, for DSHOT protocol. Maps to CCR1, and is incremented
/// automatically when we set burst len = 4 in the DMA write and read.
/// Calculate by taking the Adddress Offset for the associated CCR channel in the
/// RM register table, and dividing by 4.
/// todo: Is this valid for H7 as well?
pub const DSHOT_BASE_DIR_OFFSET: u8 = 0x34 / 4;

pub type MotorTimer = Timer<pac::TIM3>;
pub type ServoTimer = Timer<pac::TIM8>; // todo: Valid for G4 too?

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
    base_pt: &mut Location,
    spi1: &mut Spi<SPI1>,
    spi_flash: &mut flash_spi::SpiFlashType,
    i2c1: &mut I2c<I2C1>,
    i2c2: &mut I2c<I2C2>,
    cs_imu: &mut Pin,
    cs_flash: &mut Pin,
    delay: &mut Delay,
) -> (SystemStatus, baro::Altimeter) {
    let mut system_status = SystemStatus::default();

    // let eps = 0.001;

    match imu::setup(spi1, cs_imu, delay) {
        Ok(_) => system_status.imu = SensorStatus::Pass,
        Err(_) => system_status.imu = SensorStatus::NotConnected,
    };

    // todo: Put these external sensor setups back once complete with baro TS
    // match gps::setup(i2c1) {
    //     Ok(_) => system_status.gps = SensorStatus::Pass,
    //     Err(_) => system_status.gps = SensorStatus::NotConnected,
    // }

    // match mag::setup(i2c1) {
    //     Ok(_) => system_status.magnetometer = SensorStatus::Pass,
    //     Err(_) => system_status.magnetometer = SensorStatus::NotConnected,
    // }
    //
    // match tof::setup(i2c1) {
    //     Ok(_) => system_status.tof = SensorStatus::Pass,
    //     Err(_) => system_status.tof = SensorStatus::NotConnected,
    // }

    match flash_spi::setup(spi_flash, cs_flash) {
        Ok(_) => system_status.flash_spi = SensorStatus::Pass,
        Err(_) => system_status.flash_spi = SensorStatus::NotConnected,
    }

    // if let Some(agl) = tof::read(params.quaternion, i2c1) {
    //     if agl > 0.01 {
    //         return result;
    //     }
    // }

    println!("B");

    let mut altimeter = match baro::Altimeter::new(i2c2) {
        Ok(mut alt) => {
            system_status.baro = SensorStatus::Pass;
            alt
        }
        Err(_) => {
            system_status.baro = SensorStatus::NotConnected;
            Default::default()
        }
    };

    let mut altimeter = baro::Altimeter::default(); // todo tmep!!!

    println!("C");

    let fix = gps::get_fix(i2c1);
    match fix {
        Ok(f) => {
            params.lon = f.lon;
            params.lat = f.lat;
            params.baro_alt_msl = f.alt_msl;

            *base_pt = Location::new(LocationType::LatLon, f.lat, f.lon, f.alt_msl);

            if system_status.baro == SensorStatus::Pass {
                altimeter.calibrate_from_gps(Some(f.alt_msl), i2c2); // todo temp 1
            }
        }
        Err(_) => {
            if system_status.baro == SensorStatus::Pass {
                altimeter.calibrate_from_gps(None, i2c2); // todo temp 1
            }
        }
    }
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
            let mut motor2 = Pin::new(Port::C, 7, PinMode::Alt(alt_motors)); // Ch
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
    motor1.enable_interrupt(Edge::Either);
    // todo: On G4, this interrupt is also used by the IMU. Sort this out; especially given you use both
    // todo edges on RPM reception, but only one on IMU.
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
            // todo: IM4 causing trouble??
            exti.imr1.modify(|_, w| {
                w.im6().clear_bit();
                // w.im4().clear_bit();  // EXTI 4 is shared with the IMU; leave enabled.
                w.im0().clear_bit();
                w.im1().clear_bit()
            });
        }
    }

    // todo: What should this be?: Low is good up to the Mhz range, which is good enough?
    let dshot_gpiospeed = OutputSpeed::Low; // Note: Low is the default value.

    motor1.output_speed(dshot_gpiospeed);
    motor2.output_speed(dshot_gpiospeed);
    motor3.output_speed(dshot_gpiospeed);
    motor4.output_speed(dshot_gpiospeed);

    // #[cfg(feature = "g4")]
    // Not used
    // let _buzzer = Pin::new(Port::A, 10, PinMode::Alt(6)); // Tim1 ch3
    // let _buzzer = Pin::new(Port::E, 9, PinMode::Alt(1)); // Tim1 ch1

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
            let _batt_v_adc = Pin::new(Port::A, 1, PinMode::Analog); // ADC12, channel 1
            let _current_sense_adc = Pin::new(Port::B, 2, PinMode::Analog); // ADC2, channel 12

            let mut sck2 = Pin::new(Port::B, 13, PinMode::Alt(5));
            let mut miso2 = Pin::new(Port::B, 14, PinMode::Alt(5));
            let mut mosi2 = Pin::new(Port::B, 15, PinMode::Alt(5));

            // todo: Temp config to check radio status
            // let mut _sck3 = Pin::new(Port::B, 3, PinMode::Alt(6));
            // let mut _miso3 = Pin::new(Port::B, 4, PinMode::Alt(6));
            // let mut _mosi3 = Pin::new(Port::B, 5, PinMode::Alt(6));

            sck2.output_speed(spi_gpiospeed);
            miso2.output_speed(spi_gpiospeed);
            mosi2.output_speed(spi_gpiospeed);
        }
    }

    cfg_if! {
        if #[cfg(feature = "h7")] {
            // Use use Uart 7 for the onboard ELRS MCU.
            let _uart_crsf_tx = Pin::new(Port::B, 3, PinMode::Alt(11));
            let mut uart_crsf_rx = Pin::new(Port::B, 4, PinMode::Alt(11));
        } else {
            let _uart_crsf_tx = Pin::new(Port::B, 10, PinMode::Alt(7));
            let mut uart_crsf_rx = Pin::new(Port::B, 11, PinMode::Alt(7));
        }
    }

    // Pull up the CRSF RX line. Without this, our idle interrupt fires spuratically in
    // some conditions, like when touching the (even outside) of the wires if an Rx
    // module isn't connected.
    uart_crsf_rx.pull(Pull::Up);

    let _uart_osd_tx = Pin::new(Port::A, 2, PinMode::Alt(7));
    let mut uart_osd_rx = Pin::new(Port::A, 3, PinMode::Alt(7));

    uart_osd_rx.pull(Pull::Up);

    // We use UARTs for misc external devices, including ESC telemetry,
    // and VTX OSD.

    // Used to trigger a PID update based on new IMU data.
    // We assume here the interrupt config uses default settings active low, push pull, pulsed.
    #[cfg(feature = "h7")]
    let mut imu_exti_pin = Pin::new(Port::B, 12, PinMode::Input);
    #[cfg(feature = "g4")]
    let mut imu_exti_pin = Pin::new(Port::C, 4, PinMode::Input);

    imu_exti_pin.output_type(OutputType::OpenDrain);
    imu_exti_pin.pull(Pull::Up);

    // todo: Moved to `idle` for setup.
    // todo: On G4, sort out if you need to set Rising and falling, for M2 RPM reception.
    // todo: Once back from Argentina, if you have trouble, check this.
    #[cfg(feature = "h7")]
    let imu_exti_edge = Edge::Falling;
    #[cfg(feature = "g4")]
    // let imu_exti_edge = Edge::Either;
    let imu_exti_edge = Edge::Falling;

    // todo: Will this still work? Does a given port need control of the exti line, eg
    // todo from the SYSCFG setup?
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
            sda2.output_type(OutputType::OpenDrain);
        } else {
            let mut scl1 = Pin::new(Port::A, 15, i2c_alt);
            let mut sda1 = Pin::new(Port::B, 9, i2c_alt);

            let mut scl2 = Pin::new(Port::A, 9, i2c_alt);
            let mut sda2 = Pin::new(Port::A, 10, i2c_alt);

        }
    }

    // Note that we use hardware pullups, so don't enable the firmware pullups.
    sda1.output_type(OutputType::OpenDrain);
    scl1.output_type(OutputType::OpenDrain);
    sda2.output_type(OutputType::OpenDrain);
    scl2.output_type(OutputType::OpenDrain);
}

/// Assign DMA channels to peripherals.
pub fn setup_dma(dma: &mut Dma<DMA1>, dma2: &mut Dma<DMA2>) {
    // IMU
    dma::mux(IMU_DMA_PERIPH, IMU_TX_CH, DmaInput::Spi1Tx);
    dma::mux(IMU_DMA_PERIPH, IMU_RX_CH, DmaInput::Spi1Rx);

    // DSHOT, motors 1 and 2 (all 4 for H7)
    dma::mux(MOTORS_DMA_PERIPH, MOTOR_CH, MOTORS_DMA_INPUT);

    // CRSF (onboard ELRS)
    #[cfg(feature = "h7")]
    let crsf_dma_ch = DmaInput::Uart7Rx;
    #[cfg(feature = "g4")]
    let crsf_dma_ch = DmaInput::Usart3Rx;

    dma::mux(CRSF_DMA_PERIPH, CRSF_RX_CH, crsf_dma_ch);

    // CRSF TX is currently unused.
    // dma::mux(CRSF_DMA_PERIPH, CRSF_TX_CH, DmaInput::Usart3Tx);

    #[cfg(feature = "h7")]
    dma::mux(BATT_CURR_DMA_PERIPH, BATT_CURR_DMA_CH, DmaInput::Adc1);
    #[cfg(feature = "g4")]
    dma::mux(BATT_CURR_DMA_PERIPH, BATT_CURR_DMA_CH, DmaInput::Adc2);

    dma::mux(OSD_DMA_PERIPH, OSD_CH, DmaInput::Usart2Tx);

    dma::mux(BARO_DMA_PERIPH, BARO_TX_CH, DmaInput::I2c2Tx);
    // dma::mux(BARO_DMA_PERIPH, BARO_TX_CH, DmaInput::I2c1Tx); // todo temp i2c1.
    dma::mux(BARO_DMA_PERIPH, BARO_RX_CH, DmaInput::I2c1Rx);

    // todo: Put back!
    // dma::mux(EXT_SENSORS_DMA_PERIPH, EXT_SENSORS_TX_CH, DmaInput::I2c1Tx);
    // dma::mux(EXT_SENSORS_DMA_PERIPH, EXT_SENSORS_RX_CH, DmaInput::I2c1Rx);

    // We use Spi transfer complete to know when our readings are ready - in its ISR,
    // we trigger the attitude-rates PID loop.
    dma.enable_interrupt(IMU_RX_CH, DmaInterrupt::TransferComplete);

    // todo: It appears the timer `DmaUpdate`, interrupt, enabled in DSHOT setup code, will
    // todo auto-enable the transfer complete interrupts; that interrupt is required for
    // todo timer burst DMA to work. There's therefore no purpose in enabling TC explicitly here.
    // It seems that enabling these is not explicitly required; teh TC ISr still fires
    // once per burst command, even without enabling these!
    // We use Dshot transfer-complete interrupts to disable the timer.
    // dma.enable_interrupt(Motor::M1.dma_channel(), DmaInterrupt::TransferComplete);
    // #[cfg(not(feature = "h7"))]
    // dma.enable_interrupt(Motor::M3.dma_channel(), DmaInterrupt::TransferComplete);

    // Enable TC interrupts for all I2C sections; we use this to sequence readings,
    // and store reading data.
    dma::enable_interrupt(BARO_DMA_PERIPH, BARO_TX_CH, DmaInterrupt::TransferComplete);
    dma::enable_interrupt(BARO_DMA_PERIPH, BARO_RX_CH, DmaInterrupt::TransferComplete);
    // dma.enable_interrupt(EXT_SENSORS_TX_CH, DmaInterrupt::TransferComplete);
    // dma.enable_interrupt(EXT_SENSORS_RX_CH, DmaInterrupt::TransferComplete);
}

cfg_if! {
    if #[cfg(feature = "h7")] {
        type UartCrsfRegs = pac::UART7;
        type SpiPacFlash = pac::OCTOSPI1;
        type SpiFlash = Qspi;
    } else {
        type UartCrsfRegs = pac::USART3;
        type SpiPacFlash = pac::SPI2;
        type SpiFlash = Spi2<SpiPacFlash>;
    }
}

/// Configure the SPI and I2C busses.
pub fn setup_busses(
    spi1_pac: SPI1,
    spi_flash_pac: SpiPacFlash,
    i2c1_pac: I2C1,
    i2c2_pac: I2C2,
    uart2_pac: USART2,
    uart_crsf_pac: UartCrsfRegs,
    clock_cfg: &Clocks,
) -> (
    Spi<SPI1>,
    SpiFlash,
    Pin,
    Pin,
    I2c<I2C1>,
    I2c<I2C2>,
    Usart<USART2>,
    Usart<crate::UART_CRSF>,
) {
    // We use SPI1 for the IMU
    // SPI input clock is 400MHz for H7, and 170Mhz for G4. 400MHz / 32 = 12.5 MHz. 170Mhz / 8 = 21.25Mhz.
    // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
    // 426xx can use any SPI mode. Maybe St is only mode 3? Not sure.
    #[cfg(feature = "g4")]
    let imu_baud_div = BaudRate::Div8; // for ICM426xx, for 24Mhz limit
    #[cfg(feature = "h7")]
    let imu_baud_div = BaudRate::Div32; // for ICM426xx, for 24Mhz limit

    let imu_spi_cfg = SpiConfig {
        // Per ICM42688 and ISM330 DSs, only mode 3 is valid.
        mode: SpiMode::mode3(),
        ..Default::default()
    };

    let spi_imu = Spi::new(spi1_pac, imu_spi_cfg, imu_baud_div);

    #[cfg(feature = "h7")]
    let mut cs_imu = Pin::new(Port::C, 4, PinMode::Output);
    #[cfg(feature = "g4")]
    let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);

    cs_imu.set_high();

    // We use I2C1 for the GPS, magnetometer, and TOF sensor. Some details:
    // The U-BLOX GPS' max speed is 400kHz.
    // The LIS3MDL altimeter's max speed is 400kHz.
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
    let mut i2c2 = I2c::new(i2c2_pac, i2c_baro_cfg, clock_cfg);

    println!("Entering I2C2 test loop");
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = Delay::new(cp.SYST, 170_000_000);

    // unsafe {
    //     (*pac::PWR::ptr()).cr3.modify(|_, w| w.ucpd1_dbdis().set_bit()); // todo TS PA9 not working.
    // }

    let mut scl2 = Pin::new(Port::A, 9, PinMode::Output);
    let mut sda2 = Pin::new(Port::A, 10, PinMode::Output);
    sda2.output_type(OutputType::PushPull);
    scl2.output_type(OutputType::PushPull);

    // todo on Output pin test: SDA2 (PA10 is good)
    // todo: SCL2 (PA9) shows no signs of life... WTF???

    for i in 0..1_000_000_000 {
        if i % 2 == 0 {
            scl2.set_low();
            sda2.set_high()
        } else {
            // scl2.set_high();
            sda2.set_low()
        }
        delay.delay_ms(100);
    }

    // We use SPI2 for SPI flash on G4. On H7, we use octospi instead.
    // Max speed: 104Mhz (single, dual, or quad) for 8M variant, and 133Mhz for
    // 16M variant.
    // W25 flash chips use SPI mode 0 or 3.
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let spi_flash = Qspi::new(spi_flash_pac, Default::default(), BaudRate::Div8);

        } else {
            let spi_flash = Spi2::new(spi_flash_pac, Default::default(), BaudRate::Div2);
        }
    }

    #[cfg(feature = "h7")]
    let mut cs_flash = Pin::new(Port::E, 11, PinMode::Output);
    #[cfg(feature = "g4")]
    let mut cs_flash = Pin::new(Port::A, 0, PinMode::Output);

    cs_flash.set_high();

    // We use UART2 for the OSD, for DJI, via the MSP protocol.
    // todo: QC baud.
    let uart_osd = Usart::new(uart2_pac, crate::osd::BAUD, Default::default(), clock_cfg);

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

    // todo TS
    // let uart_elrs = Usart::new(uart_elrs_pac, crsf::BAUD, Default::default(), clock_cfg);
    let mut uart_crsf = Usart::new(
        uart_crsf_pac,
        crsf::BAUD,
        UsartConfig {
            // todo: We're having a struggle with overruns.
            overrun_disabled: true,
            ..Default::default()
        },
        clock_cfg,
    );

    (
        spi_imu, spi_flash, cs_imu, cs_flash, i2c1, i2c2, uart_osd, uart_crsf,
    )
}

/// Configures all 4 motor timers for quadcopters, or combinations of motors and servos
/// for fixed-wing
pub fn setup_motor_timers(motor_timer: &mut MotorTimer, servo_timer: &mut ServoTimer) {
    motor_timer.set_prescaler(dshot::DSHOT_PSC);
    motor_timer.set_auto_reload(dshot::DSHOT_ARR as u32);

    motor_timer.enable_interrupt(TimerInterrupt::UpdateDma);

    cfg_if! {
        if #[cfg(feature = "quad")] {
            dshot::set_to_output(motor_timer);
            dshot::set_bidirectional(dshot::BIDIR_EN, motor_timer);
        } else {
            servo_timer.set_prescaler(PSC_SERVOS);
            servo_timer.set_auto_reload(ARR_SERVOS);

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
