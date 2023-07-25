//! This module contains intialization code, run at power-up.

use stm32_hal2::{
	adc::{Adc, AdcDevice, AdcConfig},
    clocks,
    dma::{self, Dma, ChannelCfg},
    flash::Flash,
    timer::{Timer, TimerInterrupt},
    usart::Usart2,
};

use ahrs::Ahrs;

use crate::{
    app::{self, Local, init::Monotonics, Shared},
    protocols::{crsf, dshot},
    setup,
    main_loop,
    system_status::{SensorStatus},
};

use usbd_serial::SerialPort;

use defmt::println;

pub fn run(mut cx: app::init::Context) -> (Shared, Local, Monotonics) {
    let mut cp = cx.core;
    let mut dp = pac::Peripherals::take().unwrap();

    // Improves performance, at a cost of slightly increased power use.
    // Note that these enable fns should automatically invalidate prior.

    // Todo: Enable these caches once the program basicallyw orks on H7; don't want
    // todo subtle concurrency bugs from the caches confusing things.
    // #[cfg(feature = "h7")]
    // cp.SCB.enable_icache();
    // #[cfg(feature = "h7")]
    // cp.SCB.enable_dcache(&mut cp.CPUID);

    let pll_src = PllSrc::Hse(16_000_000);
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let clock_cfg = Clocks {
                // Config for H723 at 520Mhz, or H743 at 480Mhz.
                // Note: to get H723 working at 550Mhz. You'll need to increase divn to 275,
                // and set CPU_FREQ_BOOST: "The CPU frequency boost can be enabled through the
                // CPUFREQ_BOOST option byte in the FLASH_OPTSR2_PRG register."
                pll_src,
                pll1: PllCfg {
                    divm: 8, // To compensate with 16Mhz HSE instead of 64Mhz HSI
                    pllq_en: true, // PLLQ for Spi1 and CAN clocks. Its default div of 8 is fine.
                    ..Default::default()
                },
                hsi48_on: true,
                // todo: Why is full speed not always working?
                // ..Clocks::full_speed()
                ..Default::default()
            };
        } else {
            let clock_cfg = Clocks {
                input_src: InputSrc::Pll(pll_src),
                hsi48_on: true,
                ..Default::default()
            };
        }
    }

    println!("Pre clock setup");
    clock_cfg.setup().unwrap();

    println!("Clock config success");

    // Enable the Clock Recovery System, which improves HSI48 accuracy.

    // todo: Put this back once done USB TS.
    #[cfg(feature = "h7")]
    clocks::enable_crs(CrsSyncSrc::OtgHs);
    #[cfg(feature = "g4")]
    clocks::enable_crs(CrsSyncSrc::Usb);

    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Set up pins with appropriate modes.
    setup::setup_pins();

    let dma_ = Dma::new(dp.DMA1);
    let dma2_ = Dma::new(dp.DMA2);

    // Note that the HAL currently won't enable DMA2's RCC wihtout using a struct like this.
    let _dma2_ch1 = dma::Dma2Ch1::new();

    setup::setup_dma();

    cfg_if! {
        if #[cfg(feature = "h7")] {
            let uart_crsf_pac = dp.UART7;
            let uart_osd_pac = dp.USART2;
        } else {
            // let uart_crsf_pac = dp.USART2;
            let uart_crsf_pac = dp.USART3;
            let uart_osd_pac = dp.UART4;
        }
    }
    #[cfg(feature = "h7")]
    // let spi_flash_pac = dp.OCTOSPI1;
    let spi_flash_pac = dp.QUADSPI;
    #[cfg(feature = "g4")]
    let spi_flash_pac = dp.SPI2;

    #[cfg(feature = "h7")]
    can::set_message_ram_layout(); // Must be called explicitly; for H7.

    #[cfg(feature = "h7")]
    let can_clock = dronecan::hardware::CanClock::Mhz120;

    #[cfg(feature = "g4")]
    let can_clock = dronecan::hardware::CanClock::Mhz160;

    let mut can = dronecan::hardware::setup_can(dp.FDCAN1, can_clock, dronecan::CanBitrate::B1m);

    // todo: Configure acceptance filters for Fix2, AHRS, IMU, baro, mag, node status, and possibly others.
    // todo: on G4, you may need to be clever to avoid running out of filters.

    let (
        mut spi1,
        mut flash_spi,
        mut cs_imu,
        mut cs_flash,
        mut i2c1,
        mut i2c2,
        uart_osd,
        mut uart_crsf,
    ) = setup::setup_busses(
        dp.SPI1,
        spi_flash_pac,
        dp.I2C1,
        dp.I2C2,
        uart_osd_pac,
        uart_crsf_pac,
        &clock_cfg,
    );

    // todo start I2c test
    // println!("Starting I2c/SPI test loop");
    //
    // // let x = unsafe {
    // //     (*pac::RCC::ptr()).apb2enr.read().spi1en().bit_is_set()
    // // };
    //
    // let x = spi1.regs.cr1.read().spe().bit_is_set();
    //
    // println!("RCC TEST: {}", x);
    // //
    // loop {
    //     // let mut buf = [0];
    //     // i2c1.write_read(0x77, &[0x0d], &mut buf).ok();
    //
    //     let mut buf = [0x75, 0, 0];
    //
    //     cs_imu.set_low();
    //     spi1.transfer(&mut buf);
    //
    //     cs_imu.set_high();
    //     println!("Buf: {:?}", buf[0]);
    //     delay.delay_ms(500);
    // }
    // todo end I2c test

    // We use the RTC to assist with power use measurement.
    // let rtc = Rtc::new(dp.RTC, Default::default());

    // We use the ADC to measure battery voltage and ESC current.
    let adc_cfg = AdcConfig {
        operation_mode: adc::OperationMode::Continuous,
        ..Default::default()
    };

    #[cfg(feature = "h7")]
    let mut batt_curr_adc = Adc::new_adc1(dp.ADC1, AdcDevice::One, adc_cfg, &clock_cfg);

    #[cfg(feature = "g4")]
    let mut batt_curr_adc = Adc::new_adc2(dp.ADC2, AdcDevice::Two, adc_cfg, &clock_cfg);

    // With non-timing-critical continuous reads, we can set a long sample time.
    batt_curr_adc.set_sample_time(setup::BATT_ADC_CH, adc::SampleTime::T601);
    // todo: Which edge should it be?
    batt_curr_adc.set_trigger(adc::Trigger::Tim6Trgo, adc::TriggerEdge::HardwareRising);

    // todo temp while we sort out HAL. We've fudged this to make the number come out correctly.
    batt_curr_adc.vdda_calibrated = 3.6;

    // let mut update_timer = Timer::new_tim15(
    //     dp.TIM15,
    //     UPDATE_RATE_MAIN_LOOP,
    //     Default::default(),
    //     &clock_cfg,
    // );
    // update_timer.enable_interrupt(TimerInterrupt::Update);

    // We use this PID adjustment timer to indicate the interval for updating PID from a controller
    // while the switch or button is held. (Equivalently, the min allowed time between actuations)

    let motor_timer_cfg = TimerConfig {
        // We use ARPE since we change duty with the timer running.
        auto_reload_preload: true,
        ..Default::default()
    };

    // Frequency here can be arbitrary; we set manually using PSC and ARR below.
    let mut motor_timer = Timer::new_tim3(dp.TIM3, 1., motor_timer_cfg.clone(), &clock_cfg);

    // For fixed wing on H7; need a separate timer from the 4 used for DSHOT.
    let mut servo_timer = Timer::new_tim8(dp.TIM8, 1., motor_timer_cfg, &clock_cfg);

    setup::setup_motor_timers(&mut motor_timer, &mut servo_timer);

    // This timer periodically fire. When it does, we read the value of each of the 4 motor lines
    // in its ISR.
    let mut dshot_read_timer = Timer::new_tim2(dp.TIM2, 1., Default::default(), &clock_cfg);

    dshot_read_timer.set_prescaler(dshot::PSC_DSHOT);
    dshot_read_timer.set_auto_reload(setup::DSHOT_ARR_READ);
    dshot_read_timer.enable_interrupt(TimerInterrupt::Update);

    let (ctrl_coeff_adj_timer, lost_link_timer, mut tick_timer, mut adc_timer) =
        setup::setup_timers(dp.TIM1, dp.TIM17, dp.TIM5, dp.TIM6, &clock_cfg);

    let mut user_cfg = UserCfg::default();

    // Note: With this circular DMA approach, we discard many readings,
    // but shouldn't have consequences other than higher power use, compared to commanding
    // conversions when needed.

    unsafe {
        batt_curr_adc.read_dma(
            &mut V_A_ADC_READ_BUF,
            &[setup::BATT_ADC_CH, setup::CURR_ADC_CH],
            setup::BATT_CURR_DMA_CH,
            ChannelCfg {
                circular: dma::Circular::Enabled,
                ..Default::default()
            },
            setup::BATT_CURR_DMA_PERIPH,
        );
    }

    // todo: ID connected sensors etc by checking their device ID etc.
    let mut state_volatile = StateVolatile::default();

    cfg_if! {
        // todo: Probably OTG1 on H723.
        // On H743, PA11 and PA12 are connected to OTG2 AKA OTG_FS. There are naming inconsistencies
        // on ST's docs.
        if #[cfg(feature = "h7")] {
            let usb = Usb2::new(
                dp.OTG2_HS_GLOBAL,
                dp.OTG2_HS_DEVICE,
                dp.OTG2_HS_PWRCLK,
                clock_cfg.hclk(),
                // 240_000_000, // todo temp hardcoded
            );

            unsafe { USB_BUS = Some(UsbBus::new(usb, unsafe { &mut USB_EP_MEMORY })) };
        } else {
            let usb = usb::Peripheral { regs: dp.USB };

            unsafe { USB_BUS = Some(UsbBus::new(usb)) };
        }
    }

    let usb_serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

    // todo: Note that you may need to either increment the flash page offset, or cycle flash pages, to
    // todo avoid wear on a given sector from erasing each time. Note that you can still probably get 10k
    // todo erase/write cycles per sector, so this is low priority.
    let mut flash_onboard = Flash::new(dp.FLASH);

    // todo: Testing flash
    let mut flash_buf = [0; 8];
    // let cfg_data =
    #[cfg(feature = "h7")]
    flash_onboard.read(Bank::B1, crate::FLASH_CFG_SECTOR, 0, &mut flash_buf);
    #[cfg(feature = "g4")]
    flash_onboard.read(Bank::B1, crate::FLASH_CFG_PAGE, 0, &mut flash_buf);

    let mut params = Default::default();

    // todo: For params, consider raw readings without DMA. Currently you're just passign in the
    // todo default; not going to cut it.?
    let (system_status, altimeter) = setup::init_sensors(
        &mut params,
        &mut state_volatile.base_point,
        &mut spi1,
        &mut flash_spi,
        &mut i2c1,
        &mut i2c2,
        &mut cs_imu,
        &mut cs_flash,
        &mut delay,
    );

    println!(
        "System status:\n IMU: {}, Baro: {}, Mag: {}, GPS: {}, TOF: {}",
        system_status.imu == SensorStatus::Pass,
        system_status.baro == SensorStatus::Pass,
        system_status.magnetometer == SensorStatus::Pass,
        system_status.gnss == SensorStatus::Pass,
        system_status.tof == SensorStatus::Pass,
    );

    // todo: If you only update attitude on the fligth-control loop, change DT here accordinly.
    let mut ahrs = Ahrs::new(main_loop::DT_IMU, DeviceOrientation::default());
    // todo: Store AHRS IMU cal in config; see GNSS for ref.

    // Allow ESC to warm up and the radio to connect before starting the main loop.
    // delay.delay_ms(WARMUP_TIME);

    // We must set up the USB device after the warmup delay, since its long blocking delay
    // leads the host (PC) to terminate the connection. The (short, repeated) blocking delays
    // in the motor dir config appear to be acceptable.
    let usb_dev = UsbDeviceBuilder::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Anyleaf")
    .product("Mercury")
    // We use `serial_number` to identify the device to the PC. If it's too long,
    // we get permissions errors on the PC.
    .serial_number("AN") // todo: Try 2 letter only if causing trouble?
    .device_class(usbd_serial::USB_CLASS_CDC)
    .build();

    // Set up the main loop, the IMU loop, the CRSF reception after the (ESC and radio-connection)
    // warmpup time.

    // Set up motor direction; do this once the warmup time has elapsed.
    #[cfg(feature = "quad")]
    // todo: Wrong. You need to do this by number; apply your pin mapping.
    let motors_reversed = (
        state_volatile.motor_servo_state.rotor_aft_right.reversed,
        state_volatile.motor_servo_state.rotor_front_right.reversed,
        state_volatile.motor_servo_state.rotor_aft_left.reversed,
        state_volatile.motor_servo_state.rotor_front_left.reversed,
        // user_cfg.control_mapping.m1_reversed,
        // user_cfg.control_mapping.m2_reversed,
        // user_cfg.control_mapping.m3_reversed,
        // user_cfg.control_mapping.m4_reversed,
    );

    #[cfg(feature = "quad")]
    dshot::setup_motor_dir(motors_reversed, &mut motor_timer);

    crsf::setup(&mut uart_crsf);

    // Start our main loop
    // update_timer.enable();
    adc_timer.enable();
    tick_timer.enable();

    println!("Init complete; starting main loops");

    // Unmask the Systick interrupt here; doesn't appear to be handled by RTIC the same was
    // as for STM32 peripherals. (Systick is a Cortex-M peripheral)
    // unsafe { cortex_m::peripheral::NVIC::unmask(cortex_m::peripheral::syst); }

    (
        // todo: Make these local as able.
        Shared {
            user_cfg,
            state_volatile,
            system_status,
            autopilot_status: Default::default(),
            current_params: params.clone(),
            control_channel_data: Default::default(),
            link_stats: Default::default(),
            // dma,
            // dma2,
            spi1,
            i2c1,
            i2c2,
            altimeter,
            // rtc,
            lost_link_timer,
            motor_timer,
            servo_timer,
            usb_dev,
            usb_serial,
            flash_onboard,
            power_used: 0.,
            imu_filters: Default::default(),
            flight_ctrl_filters: Default::default(),
            ext_sensor_active: ExtSensor::Mag,
            pwr_maps: Default::default(),
            motor_pid_state: Default::default(),
            motor_pid_coeffs: Default::default(),
            // rpm_readings: Default::default(),
            // rpms_commanded: Default::default(),
            tick_timer,
            can,
        },
        Local {
            // update_timer,
            uart_crsf,
            // spi_flash, // todo: Fix flash in HAL, then do this.
            arm_signals_received: 0,
            disarm_signals_received: 0,
            // update_isr_loop_i: 0,
            imu_isr_loop_i: 0,
            // aux_loop_i: 0, // todo t
            ctrl_coeff_adj_timer,
            uart_osd,
            time_with_high_throttle: 0.,
            ahrs,
            dshot_read_timer,
            cs_imu,
            params_prev: params,
            batt_curr_adc,
        },
        Monotonics(),
    )
}
