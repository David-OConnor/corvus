//! This module contains intialization code, run at power-up. Calls functions in the
//! `setup` module as required.

use ahrs::{Ahrs, DeviceOrientation};
use hal::{
    adc::{self, Adc, AdcConfig, AdcDevice},
    clocks::{self, Clocks, PllSrc},
    dma::{self, ChannelCfg, Dma},
    flash::Flash,
    iwdg, pac,
    timer::{Timer, TimerConfig, TimerInterrupt},
};
use lin_alg2::f32::Vec3;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{self, SerialPort};

use crate::{
    app::{self, Local, Shared},
    board_config::{BATT_ADC_CH, CAN_CLOCK, CRS_SYNC_SRC, CURR_ADC_CH, DSHOT_ARR_READ},
    main_loop::DT_IMU,
    protocols::{crsf, dshot},
    sensors_shared::{ExtSensor, V_A_ADC_READ_BUF},
    setup,
    state::{StateVolatile, UserConfig},
    system_status::SensorStatus,
};

cfg_if! {
    if #[cfg(feature = "h7")] {
        use hal::{
            can,
            clocks::{PllCfg, VosRange},
            // USB1 on H723; USB2 on H743.
            usb::{Usb2, UsbBus, Usb2BusType as UsbBusType},
        };
    } else if #[cfg(feature = "g4")] {
        use hal::{
            usb::{self, UsbBus, UsbBusType},
            clocks::InputSrc,
        };
    }
}

use cfg_if::cfg_if;
use defmt::println;

// Due to the way the USB serial lib is set up, the USB bus must have a static lifetime.
// In practice, we only mutate it at initialization.
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

use crate::board_config::AHB_FREQ;

pub fn run(mut cx: app::init::Context) -> (Shared, Local) {
    let cp = cx.core;
    let dp = pac::Peripherals::take().unwrap();

    // Improves performance, at a cost of slightly increased power use.
    // Note that these enable fns should automatically invalidate prior.

    // Todo: Enable these caches once the program basically works on H7; don't want
    // todo subtle concurrency bugs from the caches confusing things.
    // #[cfg(feature = "h7")]
    // cp.SCB.enable_icache();
    // #[cfg(feature = "h7")]
    // cp.SCB.enable_dcache(&mut cp.CPUID);

    let pll_src = PllSrc::Hse(16_000_000);
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let clock_cfg = Clocks {
                // Config for H723 at 520Mhz, or H743 at 400Mhz.
                pll_src,
                pll1: PllCfg {
                    divm: 8, // To compensate with 16Mhz HSE instead of 64Mhz HSI
                    // PLLQ for Spi1 and CAN clocks. We set it to 80Mhz, which is convenient for
                    // CAN timings. For example, setting to 100Mhz or 120Mhz doesn't allow 5Mbps,
                    // among other speeds.
                    pllq_en: true,
                    divq: 10, // Sets to 80Mhz, assuming divn = 400.
                    ..Default::default()
                },
                // We use PLL2P as the (default) ADC clock. Keep the speed under 80Mhz.
                pll2: PllCfg {
                    divm: 8, // To compensate with 16Mhz HSE instead of 64Mhz HSI
                    divn: 80,
                    divp: 2, // Sets ADC clock to 80Mhz (400Mhz sysclock)
                    ..Default::default()
                },
                // todo: When you get a chance: Why is Cube showing DIVP3EN, DIVQ3en and r all = 1...
                hsi48_on: true,
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

    clock_cfg.setup().unwrap();

    // Enable the Clock Recovery System, which improves HSI48 accuracy.
    clocks::enable_crs(CRS_SYNC_SRC);

    // Set up pins with appropriate modes.
    setup::setup_pins();

    let _dma = Dma::new(dp.DMA1);
    let _dma = Dma::new(dp.DMA2);

    // Note that the HAL currently won't enable DMA2's RCC wihtout using a struct like this.
    let _dma2_ch1 = dma::Dma2Ch1::new();

    setup::setup_dma();

    cfg_if! {
        if #[cfg(feature = "h7")] {
            let uart_crsf_pac = dp.UART7;
            let uart_osd_pac = dp.USART2;
        } else {
            let uart_crsf_pac = dp.USART2;
            // let uart_crsf_pac = dp.USART3;
            let uart_osd_pac = dp.UART4;
        }
    }

    let spi_flash_pac = dp.SPI2;

    let can = dronecan::hardware::setup_can(dp.FDCAN1, CAN_CLOCK, dronecan::CanBitrate::B5m);

    #[cfg(feature = "h7")]
    can::set_message_ram_layout(); // Must be called explicitly; for H7.

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

    // We use the ADC to measure battery voltage and ESC current.
    let adc_cfg = AdcConfig {
        // With non-timing-critical continuous reads, we can set a long sample time.
        sample_time: adc::SampleTime::T601,
        // operation_mode: adc::OperationMode::Continuous, // todo: Put back
        ..Default::default()
    };

    #[cfg(feature = "h7")]
    let mut batt_curr_adc = Adc::new_adc1(dp.ADC1, AdcDevice::One, adc_cfg, AHB_FREQ);

    #[cfg(feature = "g4")]
    let mut batt_curr_adc = Adc::new_adc2(dp.ADC2, AdcDevice::Two, adc_cfg, AHB_FREQ);

    // todo temp while we sort out HAL. We've fudged this to make the number come out correctly.
    batt_curr_adc.vdda_calibrated = 3.6;

    // todo: Which edge should it be?
    batt_curr_adc.set_trigger(adc::Trigger::Tim6Trgo, adc::TriggerEdge::HardwareRising);

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
    dshot_read_timer.set_auto_reload(DSHOT_ARR_READ);
    dshot_read_timer.enable_interrupt(TimerInterrupt::Update);

    let (ctrl_coeff_adj_timer, mut tick_timer, mut adc_timer) =
        setup::setup_timers(dp.TIM1, dp.TIM5, dp.TIM6, &clock_cfg);

    // Note: With this circular DMA approach, we discard many readings,
    // but shouldn't have consequences other than higher power use, compared to commanding
    // conversions when needed.

    unsafe {
        batt_curr_adc.read_dma(
            &mut V_A_ADC_READ_BUF,
            &[BATT_ADC_CH, CURR_ADC_CH],
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

            // todo: C+P from h7xx-hal. Is this right?
            static mut USB_EP_MEMORY: [u32; 1024] = [0; 1024];
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

    let mut user_cfg = UserConfig::load(&mut flash_onboard);

    println!(
        "Loaded acc cal: x{} y{} z{}",
        user_cfg.acc_cal_bias.0, user_cfg.acc_cal_bias.1, user_cfg.acc_cal_bias.2
    );

    // For the initial config (Eg on a new device), 0xff in flash indicates the config
    // hasn't been saved yet.
    if user_cfg.pid_coeffs.p.is_nan() | user_cfg.pid_coeffs.i.is_nan() {
        println!("Loading default cfg");
        user_cfg = Default::default();
        user_cfg.save(&mut flash_onboard);
    }

    user_cfg.save(&mut flash_onboard);

    let mut ahrs = Ahrs::new(DT_IMU, DeviceOrientation::default());
    // let mut ahrs = Ahrs::new(DT_IMU, user_cfg.orientation); // todo

    ahrs.cal.acc_bias = Vec3::new(
        user_cfg.acc_cal_bias.0,
        user_cfg.acc_cal_bias.1,
        user_cfg.acc_cal_bias.2,
    );

    let mut params = Default::default();

    let (system_status, altimeter) = setup::init_sensors(
        &mut params,
        &mut state_volatile.base_point,
        &mut spi1,
        &mut flash_spi,
        &mut i2c1,
        &mut i2c2,
        &mut cs_imu,
        &mut cs_flash,
        &clock_cfg,
    );

    println!(
        "System status:\n IMU: {}, Baro: {}, Mag: {}, GPS: {}, TOF: {}, OSD: {}",
        system_status.imu == SensorStatus::Pass,
        system_status.baro == SensorStatus::Pass,
        system_status.magnetometer_can == SensorStatus::Pass,
        system_status.gnss_can == SensorStatus::Pass,
        system_status.tof == SensorStatus::Pass,
        system_status.osd == SensorStatus::Pass,
    );

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

    iwdg::setup(0.1);

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
            params: params.clone(),
            control_channel_data: Default::default(),
            link_stats: Default::default(),
            // dma,
            // dma2,
            spi1,
            i2c1,
            i2c2,
            uart_osd,
            altimeter,
            // rtc,
            // lost_link_timer,
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
            // motor_pid_state: Default::default(),
            motor_pid_coeffs: Default::default(),
            // rpm_readings: Default::default(),
            // rpms_commanded: Default::default(),
            tick_timer,
            can,
            fix: Default::default(),
            posit_inertial: Default::default(),
            ahrs,
            calibrating_accel: false,
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
            time_with_high_throttle: 0.,
            time_with_low_throttle: 0.,
            dshot_read_timer,
            cs_imu,
            params_prev: params,
            batt_curr_adc,
            task_durations: Default::default(),
        },
    )
}
