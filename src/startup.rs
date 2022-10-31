//! Handles hardware and software initialization. Run with interrupts disabled. Eg run within `Rtic`'s `init` fn.
//!
//! // todo: Not used due to RTIC limitations. Also, outdated/not compatible with current code structure (oct 29, 2022)

use stm32_hal2::{
    self,
    adc::{self, Adc, AdcConfig, AdcDevice},
    clocks::{self, Clocks, CrsSyncSrc, InputSrc, PllSrc},
    dma::{self, ChannelCfg, Dma, DmaChannel},
    flash::{Bank, Flash},
    gpio::{self, Edge, Pin, PinMode, Port},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{
        self, ADC2, DMA1, I2C1, I2C2, SPI1, SPI2, SPI3, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4,
        USART3,
    },
    rtc::Rtc,
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::{OutputCompare, TimChannel, Timer, TimerConfig, TimerInterrupt},
    usart::{Usart, UsartInterrupt},
};

use crate::{
    app::{Shared, Local, Monotomic},
    lin_alg::{Mat3, Vec3},
    // pid::{CtrlCoeffGroup, PidDerivFilters, PidGroup},
    ppks::{Location, LocationType},
    protocols::{crsf, dshot, usb_cfg},
    safety::{LOST_LINK_TIMEOUT, ArmStatus},
    state::{AircraftType, OperationMode, StateVolatile, UserCfg},
    madgwick::{self, Ahrs},
};


use defmt::println;
use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "g4")] {
        use stm32_hal2::usb::{Peripheral, UsbBus, UsbBusType};

        use usb_device::bus::UsbBusAllocator;
        use usb_device::prelude::*;
        use usbd_serial::{SerialPort, USB_CLASS_CDC};


        // Due to the way the USB serial lib is set up, the USB bus must have a static lifetime.
        // In practice, we only mutate it at initialization.
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
    }
}

#[cfg(feature = "h7")]
use crate::clocks::VosRange;

pub fn init() -> (Shared, Local, Monotomic) {
    // Cortex-M peripherals
    let mut cp = cx.core;
    // Set up microcontroller peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks {
        // Config for 550Mhz full speed:
        #[cfg(feature = "h7")]
        pll_src: PllSrc::Hse(16_000_000),
        #[cfg(feature = "g4")]
        input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
        #[cfg(feature = "h7")]
        pll1: PllCfg {
            divm: 4, // To compensate with 16Mhz HSE instead of 64Mhz HSI // todo
            ..Default::default()
        },
        hsi48_on: true,
        #[cfg(feature = "h7")]
        usb_src: clocks::UsbSrc::Hsi48,
        #[cfg(feature = "g4")]
        clk48_src: clocks::Clk48Src::Hsi48,
        #[cfg(feature = "g4")]
        boost_mode: true, // Required for speeds > 150Mhz.
        ..Default::default()
    };

    // todo: Make sur eyou set up vos0 etc as requaired for 550Mhz H7.

    clock_cfg.setup().unwrap();

    // Enable the Clock Recovery System, which improves HSI48 accuracy.
    #[cfg(feature = "h7")]
        clocks::enable_crs(CrsSyncSrc::OtgHs);
    #[cfg(feature = "g4")]
        clocks::enable_crs(CrsSyncSrc::Usb);

    // Improves performance, at a cost of slightly increased power use.
    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    // cp.SCB.clean_invalidate_dcache(); // todo?
    // cp.SCB.enable_dcache(); // todo?

    // Set up pins with appropriate modes.
    setup::setup_pins();

    let mut dma = Dma::new(dp.DMA1);
    #[cfg(feature = "g4")]
        dma::enable_mux1();

    #[cfg(feature = "h7")]
        setup::setup_dma(&mut dma, &mut dp.DMAMUX1);
    #[cfg(feature = "g4")]
        setup::setup_dma(&mut dma, &mut dp.DMAMUX);

    // We use SPI1 for the IMU
    // SPI input clock is 400MHz for H7, and 172Mhz for G4. 400MHz / 32 = 12.5 MHz. 170Mhz / 8 = 21.25Mhz.
    // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
    // 426xx can use any SPI mode. Maybe St is only mode 3? Not sure.
    #[cfg(feature = "g4")]
        // todo: Switch to higher speed.
        // let imu_baud_div = BaudRate::Div8;  // for ICM426xx, for 24Mhz limit
        // todo: 10 MHz max SPI frequency for intialisation? Found in BF; confirm in RM.
        let imu_baud_div = BaudRate::Div32; // 5.3125 Mhz, for ISM330 10Mhz limit
    #[cfg(feature = "h7")]
        // let imu_baud_div = BaudRate::Div32; // for ICM426xx, for 24Mhz limit
        let imu_baud_div = BaudRate::Div64; // 6.25Mhz for ISM330 10Mhz limit

    let imu_spi_cfg = SpiConfig {
        // Per ICM42688 and ISM330 DSs, only mode 3 is valid.
        mode: SpiMode::mode3(),
        ..Default::default()
    };

    let mut spi1 = Spi::new(dp.SPI1, imu_spi_cfg, imu_baud_div);

    #[cfg(feature = "mercury-h7")]
        let mut cs_imu = Pin::new(Port::C, 4, PinMode::Output);
    #[cfg(feature = "g4")]
        let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);

    cs_imu.set_high();

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    imu::setup(&mut spi1, &mut cs_imu, &mut delay);

    // We use SPI2 for the LoRa ELRS chip. Uses mode0.  // todo: Find max speed.
    // Note that ELRS uses 9Mhz, although we could go higher if wanted. Div32 = 5.3125.
    let spi2 = Spi::new(dp.SPI2, Default::default(), BaudRate::Div32);

    // See ELRS `SX1280_hal.cpp`, `init` fn for pin setup.
    // todo: Move this as approp.
    // Used to trigger a a control-data-received update based on new ELRS data.
    let elrs_busy = Pin::new(Port::C, 14, PinMode::Input);

    // todo: Put elrs_dio back. Currently disabled since we have accidentally shorted it to SCL2.
    // let mut elrs_dio = Pin::new(Port::C, 13, PinMode::Input);
    // elrs_dio.enable_interrupt(Edge::Rising);
    let mut cs_elrs = Pin::new(Port::C, 15, PinMode::Input);
    cs_elrs.set_high();

    // We use SPI3 for flash. // todo: Find max speed and supported modes.
    let spi3 = Spi::new(dp.SPI3, Default::default(), BaudRate::Div32);

    #[cfg(feature = "h7")]
        let flash_pin = 10;
    #[cfg(not(feature = "h7"))]
        let flash_pin = 6;
    let mut cs_flash = Pin::new(Port::C, flash_pin, PinMode::Output);
    cs_flash.set_high();

    // We use I2C for the TOF sensor.(?)
    let i2c_tof_cfg = I2cConfig {
        speed: I2cSpeed::FastPlus1M,
        // speed: I2cSpeed::Fast400k,
        ..Default::default()
    };

    // We use I2C1 for offboard sensors: Magnetometer, GPS, and TOF sensor.
    let mut i2c1 = I2c::new(dp.I2C1, i2c_tof_cfg, &clock_cfg);

    // We use I2C for the TOF sensor.(?)
    let i2c_baro_cfg = I2cConfig {
        speed: I2cSpeed::Fast400K,
        ..Default::default()
    };

    // We use UART2 for the OSD, for DJI, via the MSP protocol.
    let mut uart2 = Usart::new(dp.USART2, 115_200, Default::default(), &clock_cfg);
    // todo: DMA for OSD?

    // todo: Put back
    // osd::setup(&mut uart2); // Keep this channel in sync with `setup.rs`.

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

    // todo note: We'd like to move to ELRS long term, but use this for now.
    // The STM32-HAL default UART config includes stop bits = 1, parity disabled, and 8-bit words,
    // which is what we want.
    let mut uart3 = Usart::new(dp.USART3, 420_000, Default::default(), &clock_cfg);
    crsf::setup(&mut uart3, setup::CRSF_RX_CH, &mut dma); // Keep this channel in sync with `setup.rs`.

    // We use the RTC to assist with power use measurement.
    let rtc = Rtc::new(dp.RTC, Default::default());

    // We use the ADC to measure battery voltage and ESC current.
    let adc_cfg = AdcConfig {
        operation_mode: adc::OperationMode::Continuous,
        ..Default::default()
    };

    let mut batt_curr_adc = Adc::new_adc2(dp.ADC2, AdcDevice::Two, adc_cfg, &clock_cfg);

    // With non-timing-critical continuous reads, we can set a long sample time.
    batt_curr_adc.set_sample_time(setup::BATT_ADC_CH, adc::SampleTime::T601);
    batt_curr_adc.set_sample_time(setup::CURR_ADC_CH, adc::SampleTime::T601);

    // Note: With this circular DMA approach, we discard many readings,
    // but shouldn't have consequences other than higher power use, compared to commanding
    // conversions when needed.
    unsafe {
        batt_curr_adc.read_dma(
            &mut ADC_READ_BUF,
            &[setup::BATT_ADC_CH, setup::CURR_ADC_CH],
            setup::BATT_CURR_ADC_CH,
            ChannelCfg {
                circular: dma::Circular::Enabled,
                ..Default::default()
            },
            &mut dma,
        );
    }

    // Set rate to UPDATE_RATE_ATTITUDE; the slower velocity-update timer will run once
    // every few of these updates.
    let mut update_timer = Timer::new_tim15(
        dp.TIM15,
        UPDATE_RATE_ATTITUDE,
        Default::default(),
        &clock_cfg,
    );
    update_timer.enable_interrupt(TimerInterrupt::Update);

    let rf_limiter_timer = Timer::new_tim16(
        dp.TIM16,
        MAX_RF_UPDATE_RATE,
        TimerConfig {
            one_pulse_mode: true,
            ..Default::default()
        },
        &clock_cfg,
    );

    let rotor_timer_cfg = TimerConfig {
        // We use ARPE since we change duty with the timer running.
        auto_reload_preload: true,
        ..Default::default()
    };

    // We use multiple timers instead of a single one based on pin availability; a single
    // timer with 4 channels would be ideal.
    // Frequency here can be arbitrary; we set manually using PSC and ARR below.
    cfg_if! {
            if #[cfg(feature = "mercury-h7")] {
                let mut rotor_timer_a =
                    Timer::new_tim2(dp.TIM2, 1., rotor_timer_cfg.clone(), &clock_cfg);

                let mut rotor_timer_b = Timer::new_tim3(dp.TIM3, 1., rotor_timer_cfg, &clock_cfg);

                // For fixed wing on H7; need a separate timer from the 4 used for DSHOT.
                // todo: PAC ommission??
                // let mut servo_timer = Timer::new_tim8(dp.TIM8, 1., Default::default(), &clock_cfg);

            } else if #[cfg(feature = "g4")] {

                let mut rotor_timer_a =
                    Timer::new_tim2(dp.TIM2, 1., rotor_timer_cfg.clone(), &clock_cfg);

                let mut rotor_timer_b = Timer::new_tim3(dp.TIM3, 1., rotor_timer_cfg, &clock_cfg);
            }
        }

    let mut lost_link_timer = Timer::new_tim17(
        dp.TIM17,
        1. / LOST_LINK_TIMEOUT,
        TimerConfig {
            one_pulse_mode: true,
            ..Default::default()
        },
        &clock_cfg,
    );
    lost_link_timer.enable_interrupt(TimerInterrupt::Update);

    let elrs_timer = Timer::new_tim4(
        dp.TIM4,
        1.,
        TimerConfig {
            auto_reload_preload: true,
            ..Default::default()
        },
        &clock_cfg,
    );

    let mut user_cfg = UserCfg::default();

    user_cfg.aircraft_type = AircraftType::FlyingWing; // todo temp


    let mut ctrl_coeffs = Default::default();
    match user_cfg.aircraft_type {
        AircraftType::Quadcopter => {
            #[cfg(feature = "h7")]
                dshot::setup_timers(&mut rotor_timer_b);
            #[cfg(feature = "g4")]
                dshot::setup_timers(&mut rotor_timer_a, &mut rotor_timer_b);
        }
        AircraftType::FlyingWing => {
            flying_wing::setup_timers(&mut rotor_timer_a, &mut rotor_timer_b);
            // ctrl_coeffs = CtrlCoeffGroup::default_flying_wing();
        }
    }

    // todo temp
    // loop {
    //     flying_wing::set_elevon_posit(
    //         flying_wing::ServoWing::S1,
    //         -0.5,
    //         &user_cfg.servo_wing_mapping,
    //         &mut rotor_timer_b,
    //     );
    //     flying_wing::set_elevon_posit(
    //         flying_wing::ServoWing::S2,
    //         -0.5,
    //         &user_cfg.servo_wing_mapping,
    //         &mut rotor_timer_b,
    //     );
    //
    //     delay.delay_ms(2000);
    //     flying_wing::set_elevon_posit(
    //         flying_wing::ServoWing::S1,
    //         0.5,
    //         &user_cfg.servo_wing_mapping,
    //         &mut rotor_timer_b,
    //     );
    //     flying_wing::set_elevon_posit(
    //         flying_wing::ServoWing::S2,
    //         0.5,
    //         &user_cfg.servo_wing_mapping,
    //         &mut rotor_timer_b,
    //     );
    //     delay.delay_ms(2000);
    // }

    // We use I2C2 for the barometer / altimeter.
    let mut i2c2 = I2c::new(dp.I2C2, i2c_baro_cfg, &clock_cfg);

    println!("Setting up alt");
    let mut altimeter = baro::Altimeter::new(&mut i2c2);

    println!("Altimeter setup complete");

    // loop {
    //     let reading = altimeter.read_pressure(&mut i2c2);
    //     println!("Alt: {:?}", reading);
    //     delay.delay_ms(500);
    // }

    // todo: ID connected sensors etc by checking their device ID etc.
    let mut state_volatile = StateVolatile::default();

    cfg_if! {
            if #[cfg(feature = "g4")] {
                let usb = Peripheral { regs: dp.USB };

                unsafe { USB_BUS = Some(UsbBus::new(usb)) };

                let usb_serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

                let usb_dev = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(0x16c0, 0x27dd))
                    .manufacturer("Anyleaf")
                    // .product("MercuryG4")
                    .product("G4")
                    // We use `serial_number` to identify the device to the PC. If it's too long,
                    // we get permissions errors on the PC.
                    .serial_number("AN") // todo: Try 2 letter only if causing trouble?
                    .device_class(USB_CLASS_CDC)
                    .build();
            }
        }
    usb_cfg::init_crc();

    // todo: DMA for voltage ADC (?)

    // todo: Note that you may need to either increment the flash page offset, or cycle flash pages, to
    // todo avoid wear on a given sector from erasing each time. Note that you can still probably get 10k
    // todo erase/write cycles per sector, so this is low priority.
    let mut flash_onboard = Flash::new(dp.FLASH);

    // todo: Testing flash
    let mut flash_buf = [0; 8];
    // let cfg_data =
    flash_onboard.read(Bank::B1, crate::FLASH_CFG_PAGE, 0, &mut flash_buf);

    // println!(
    //     "mem val: {}",
    //     flash_onboard.read(ONBOARD_FLASH_START_PAGE, 0)
    // );

    println!("Flash Buf ( should be 1, 2, 3, 0, 0): {:?}", flash_buf);
    // flash_onboard.erase_write_page(Bank::B1, ONBOARD_FLASH_START_PAGE, &[10, 11, 12, 13, 14, 15, 16, 17]).ok();
    println!("Flash write complete");

    let mut params = Default::default();

    let mut base_point = Location::default();

    // todo: For params, consider raw readings without DMA. Currently you're just passign in the
    // todo default; not going to cut it.?
    crate::init_sensors(
        &mut params,
        &mut altimeter,
        &mut base_point,
        &mut i2c1,
        &mut i2c2,
    );

    // loop {
    //     let pressure = altimeter.read_pressure(&mut i2c2);
    //     println!("Pressure: {}", pressure);
    //
    //     let temp = altimeter.read_temp(&mut i2c2);
    //     println!("Temp: {}", temp);
    //
    //     delay.delay_ms(500);
    // }

    // todo: Calibation proecedure, either in air or on ground.
    let ahrs_settings = madgwick::Settings::default();

    // Note: Calibration and offsets ares handled handled by their defaults currently.
    let imu_calibration = imu_calibration::ImuCalibration {
        gyro_misalignment: Mat3 {
            data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        gyro_sensitivity: Vec3::new(1.0, 1.0, 1.0),
        gyro_offset: Vec3::new(0.0, 0.0, 0.0),
        accel_misalignment: Mat3 {
            data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        accel_sensitivity: Vec3::new(1.0, 1.0, 1.0),
        accel_offset: Vec3::new(0.0, 0.0, 0.0),
        ..Default::default()
    }; // todo - load from flash

    let ahrs = Ahrs::new(&ahrs_settings, crate::IMU_UPDATE_RATE as u32);

    update_timer.enable();

    println!("Entering main loop...");
    (
        // todo: Make these local as able.
        Shared {
            user_cfg,
            state_volatile,
            input_map: Default::default(),
            input_mode: InputMode::Acro,
            autopilot_status: Default::default(),
            ctrl_coeffs,
            current_params: params,
            velocities_commanded: Default::default(),
            attitudes_commanded: Default::default(),
            rates_commanded: Default::default(),
            control_mix: Default::default(),
            pid_velocity: Default::default(),
            pid_attitude: Default::default(),
            pid_rate: Default::default(),
            control_channel_data: Default::default(),
            // manual_inputs: Default::default(),
            dma,
            spi1,
            spi2,
            cs_imu,
            cs_elrs,
            i2c1,
            i2c2,
            altimeter,
            uart3,
            batt_curr_adc,
            // rtc,
            update_timer,
            rf_limiter_timer,
            lost_link_timer,
            rotor_timer_a,
            rotor_timer_b,
            elrs_timer,
            // delay_timer,
            #[cfg(feature = "g4")]
            usb_dev,
            #[cfg(feature = "g4")]
            usb_serial,
            flash_onboard,
            power_used: 0.,
            pid_deriv_filters: Default::default(),
            imu_filters: Default::default(),
            base_point,
            command_state: Default::default(),
            ahrs,
            imu_calibration,
        },
        Local {
            spi3,
            arm_signals_received: 0,
            disarm_signals_received: 0,
            update_loop_i: 0,
            fixed_wing_rate_loop_i: 0,
        },
        init::Monotonics(),
    )
}