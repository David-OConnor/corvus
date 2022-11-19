#![no_main]
#![no_std]
// Used on USB protocol. Allows adding to the const param buff size
// to make packet size.

// Potential markets:
// - Hobby / racing (duh)
// Tower inspections (Maybe market disruption by using fixed-wing + tpod?)
//
// https://www.youtube.com/watch?v=zOByx3Izf5U
// For state estimation
// https://www.youtube.com/watch?v=RZd6XDx5VXo (Series)
// https://www.youtube.com/watch?v=whSw42XddsU
// https://www.youtube.com/playlist?list=PLn8PRpmsu08ryYoBpEKzoMOveSTyS-h4a
// For quadrotor PID control
// https://www.youtube.com/playlist?list=PLn8PRpmsu08oOLBVYYIwwN_nvuyUqEjrj
// https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y
// https://www.youtube.com/playlist?list=PLn8PRpmsu08pFBqgd_6Bi7msgkWFKL33b
//
// todo: Movable camera that moves with head motion.
// - Ir cam to find or avoid people

use core::sync::atomic::Ordering;

use ahrs_fusion::Ahrs;
use cfg_if::cfg_if;
use control_interface::{ChannelData, LinkStats, PidTuneActuation};
use cortex_m::{self, asm, delay::Delay};
use defmt::println;
use defmt_rtt as _;
use drivers::{
    baro_dps310 as baro, gps_ublox as gps, imu_icm426xx as imu, mag_lis3mdl as mag,
    osd::{self, AutopilotData, OsdData},
    tof_vl53l1 as tof,
};
use filter_imu::ImuFilters;
use flight_ctrls::{
    autopilot::AutopilotStatus,
    common::MotorRpm,
    // pid::{
    //     self, CtrlCoeffGroup, PidDerivFilters, PidGroup, PID_CONTROL_ADJ_AMT,
    //     PID_CONTROL_ADJ_TIMEOUT,
    // },
    ctrl_logic::{self, PowerMaps},
    filters::FlightCtrlFilters,
    pid::{MotorCoeffs, MotorPidGroup},
    ControlMapping,
};
use lin_alg2::f32::Quaternion;
use panic_probe as _;
use ppks::{Location, LocationType};
use protocols::{crsf, dshot, usb_preflight};
use safety::ArmStatus;
use sensors_shared::{ExtSensor, V_A_ADC_READ_BUF};
use state::{OperationMode, StateVolatile, UserCfg};

use params::Params;
use stm32_hal2::{
    self,
    adc::{self, Adc, AdcConfig, AdcDevice},
    clocks::{self, Clocks, CrsSyncSrc, InputSrc, PllSrc},
    dma::{self, ChannelCfg, Dma, DmaInterrupt, DmaPeriph},
    flash::{Bank, Flash},
    gpio::{self, Edge, Pin, Port},
    i2c::I2c,
    pac::{self, DMA1, DMA2, I2C1, I2C2, SPI1, TIM1, TIM15, TIM16, TIM17, TIM2, TIM5, USART2},
    spi::Spi,
    timer::{Timer, TimerConfig, TimerInterrupt},
    usart::{Usart, UsartInterrupt},
};
use system_status::{SensorStatus, SystemStatus};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{self, SerialPort};

mod ahrs_fusion;
mod atmos_model;
mod attitude_platform;
mod cfg_storage;
mod control_interface;
mod drivers;
mod filter_imu;
mod flight_ctrls;
mod imu_calibration;
mod imu_shared;
mod params;
mod ppks;
mod protocols;
mod safety;
mod sensors_shared;
mod setup;
mod state;
mod system_status;
mod util;

cfg_if! {
    if #[cfg(feature = "h7")] {
        use stm32_hal2::{
            clocks::{PllCfg, VosRange},
            usb_otg::{Usb1, UsbBus, Usb1BusType as UsbBusType},
            pac::OCTOSPI1,
            qspi::{Qspi},
        };
        // This USART alias is made pub here, so we don't repeat this line in other modules.
        pub use stm32_hal2::pac::{UART7 as UART_CRSF, ADC1 as ADC};

        // type SpiFlash = Qspi<OCTOSPI>;
        type SpiFlash = Qspi;
    } else if #[cfg(feature = "g4")] {
        use stm32_hal2::{
            usb::{self, UsbBus, UsbBusType},
            pac::SPI3,
        };

        pub use stm32_hal2::pac::{USART3 as UART_CRSF, ADC2 as ADC};

        type SpiFlash = Spi<SPI3>;
    }
}

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use flight_ctrls::{autopilot::Orbit, ControlPositions};
    } else {
        use flight_ctrls::RotationDir;
    }
}

// Due to the way the USB serial lib is set up, the USB bus must have a static lifetime.
// In practice, we only mutate it at initialization.
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

// todo: Can't get startup code working separately since Shared and Local must be private per an RTIC restriction.
// todo: See this GH issue: https://github.com/rtic-rs/cortex-m-rtic/issues/505
// mod startup;

// todo: Cycle flash pages for even wear. Can postpone this.

// If IMU updates at 8kHz and ratio is 4, the flight control loop operates at 2kHz.
const FLIGHT_CTRL_IMU_RATIO: usize = 4; // Likely values: 1, 2, 4.

cfg_if! {
    if #[cfg(feature = "h7")] {
        // H723: 1Mb of flash, in one bank.
        // 8 sectors of 128kb each.
        // (H743 is similar, but may have 2 banks, each with those properties)
        const FLASH_CFG_SECTOR: usize = 6;
        const FLASH_WAYPOINT_SECTOR: usize = 7;

        // The rate our main program updates, in Hz.
        // todo note that we will have to scale up values slightly on teh H7 board with 32.768kHz oscillator:
        // ICM-42688 DS: The ODR values shown in the
        // datasheet are supported with external clock input frequency of 32kHz. For any other external
        // clock input frequency, these ODR values will scale by a factor of (External clock value in kHz / 32).
        // For example, if an external clock frequency of 32.768kHz is used,
        // instead of ODR value of 500Hz, it will be 500 * (32.768 / 32) = 512Hz.
        const UPDATE_RATE_FLIGHT_CTRLS: f32 = 8_192. / FLIGHT_CTRL_IMU_RATIO as f32;
    } else {
        // G47x/G48x: 512k flash.
        // Assumes configured as a single bank: 128 pages of 4kb each.
        // (If using G4 dual bank mode: 128 pages of pages of 2kb each, per bank)
        const FLASH_CFG_PAGE: usize = 126;
        const FLASH_WAYPOINT_PAGE: usize = 127;

        // Todo: Measured: 8.042kHz (2022-10-26)
        const UPDATE_RATE_FLIGHT_CTRLS: f32 = 8_000. / FLIGHT_CTRL_IMU_RATIO as f32;
    }
}

const UPDATE_RATE_MAIN_LOOP: f32 = 600.; // todo: Experiment with this.

const DT_FLIGHT_CTRLS: f32 = 1. / UPDATE_RATE_FLIGHT_CTRLS;
const DT_MAIN_LOOP: f32 = 1. / UPDATE_RATE_MAIN_LOOP;

// Every x main update loops, log parameters etc to flash.
const LOGGING_UPDATE_RATIO: usize = 100;

// Every x main update loops, print system status and sensor readings to console,
// if enabled with the `print-status` feature.
const PRINT_STATUS_RATIO: usize = 2_000;

// Every x main loops, log RPM (or servo posit) to angular accel (thrust) data.
const THRUST_LOG_RATIO: usize = 20;

#[cfg(feature = "h7")]
static mut USB_EP_MEMORY: [u32; 1024] = [0; 1024];

// We use `LOOP_I` to manage sequencing the velocity-update PID from within the attitude-update PID
// loop.
// const VELOCITY_ATTITUDE_UPDATE_RATIO: usize = 4;
// todo: Currently unused.
const FIXED_WING_RATE_UPDATE_RATIO: usize = 16; // 8k IMU update rate / 16 / 500Hz; apt for servos.

// todo: Temp as we switch from PID to other controls; we still will have
// todo params that can be adjusetd in flight.
const CTRL_COEFF_ADJ_TIMEOUT: f32 = 0.3; // seconds
const CTRL_COEFF_ADJ_AMT: f32 = 0.01; // seconds

// The time, in ms, to wait during initializing to allow the ESC and RX to power up and initialize.
const WARMUP_TIME: u32 = 2_000;

// todo: Bit flags that display as diff colored LEDs, and OSD items

// todo: dispatchers are temp
#[rtic::app(device = pac, peripherals = false, dispatchers= [])]
mod app {
    use super::*;
    // use core::time::Duration; // todo temp

    #[monotonic(binds = TIM5, default = true)]
    // type MyMono = Timer<TIM5>; // todo temp
    #[shared]
    struct Shared {
        user_cfg: UserCfg,
        state_volatile: StateVolatile,
        system_status: SystemStatus,
        autopilot_status: AutopilotStatus,
        current_params: Params,
        // todo: `params_prev` is an experimental var used in our alternative/experimental
        // todo flight controls code as a derivative.
        params_prev: Params,
        // None if the data is stale. eg lost link, no link established.
        control_channel_data: Option<ChannelData>,
        /// Link statistics, including Received Signal Strength Indicator (RSSI) from the controller's radio.
        link_stats: LinkStats,
        spi1: Spi<SPI1>,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        altimeter: baro::Altimeter,
        flash_onboard: Flash,
        batt_curr_adc: Adc<ADC>,
        rf_limiter_timer: Timer<TIM16>,
        lost_link_timer: Timer<TIM17>,
        link_lost: bool, // todo: atomic bool? Separate froms StateVolatile due to how it's used.
        motor_timer: setup::MotorTimer,
        servo_timer: setup::ServoTimer,
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_serial: SerialPort<'static, UsbBusType>,
        /// `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        imu_filters: ImuFilters,
        flight_ctrl_filters: FlightCtrlFilters,
        // Note: We don't currently haveh PID filters, since we're not using a D term for the
        // RPM PID.
        imu_calibration: imu_calibration::ImuCalibration,
        ext_sensor_active: ExtSensor,
        pwr_maps: PowerMaps,
        /// Store rotor RPM: (M1, M2, M3, M4). Quad only, but we can't feature gate
        /// shared fields.
        rotor_rpms: MotorRpm,
        motor_pid_state: MotorPidGroup,
        /// PID motor coefficients
        motor_pid_coeffs: MotorCoeffs,
    }

    #[local]
    struct Local {
        update_timer: Timer<TIM15>,
        uart_crsf: Usart<UART_CRSF>, // for ELRS over CRSF.
        // spi_flash: SpiFlash,  // todo: Fix flash in HAL, then do this.
        arm_signals_received: u8, // todo: Put sharedin state volatile.
        disarm_signals_received: u8,
        /// We use this counter to subdivide the main loop into longer intervals,
        /// for various tasks like logging, and outer loops.
        update_isr_loop_i: usize,
        imu_isr_loop_i: usize,
        aux_loop_i: usize, // todo temp
        ctrl_coeff_adj_timer: Timer<TIM1>,
        uart_osd: Usart<USART2>, // for our DJI OSD, via MSP protocol
        time_with_high_throttle: f32,
        measurement_timer: Timer<TIM5>,
        ahrs: Ahrs,
        dshot_read_timer: Timer<TIM2>,
        cs_imu: Pin,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // See note above about an RTIC limit preventing us from initing this way.
        // startup::init(&cx.core)

        // Cortex-M peripherals
        let mut cp = cx.core;
        // Set up microcontroller peripherals
        let mut dp = pac::Peripherals::take().unwrap();

        // Improves performance, at a cost of slightly increased power use.
        // Note that these enable fns should automatically invalidate prior.
        #[cfg(feature = "h7")]
        cp.SCB.enable_icache();
        #[cfg(feature = "h7")]
        cp.SCB.enable_dcache(&mut cp.CPUID);

        cfg_if! {
            if #[cfg(feature = "h7")] {
                let clock_cfg = Clocks {
                    // Config for H723 at 520Mhz, or H743 at 480Mhz.
                    // Note: to get H723 working at 550Mhz. You'll need to increase divn to 275,
                    // and set CPU_FREQ_BOOST: "The CPU frequency boost can be enabled through the
                    // CPUFREQ_BOOST option byte in the FLASH_OPTSR2_PRG register."
                    pll_src: PllSrc::Hse(16_000_000),
                    pll1: PllCfg {
                        // todo: Do we want a 64Mhz HSE for H7?
                        divm: 8, // To compensate with 16Mhz HSE instead of 64Mhz HSI
                        // divn: 275, // For 550Mhz with freq boost enabled.
                        ..Default::default()
                    },
                    hsi48_on: true,
                    usb_src: clocks::UsbSrc::Hsi48,

                    ..Clocks::full_speed()
                };
            } else {
                let clock_cfg = Clocks {
                    input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
                    hsi48_on: true,
                    clk48_src: clocks::Clk48Src::Hsi48,
                    ..Default::default()
                };
            }
        }

        clock_cfg.setup().unwrap();

        // Enable the Clock Recovery System, which improves HSI48 accuracy.
        #[cfg(feature = "h7")]
        clocks::enable_crs(CrsSyncSrc::OtgHs);
        #[cfg(feature = "g4")]
        clocks::enable_crs(CrsSyncSrc::Usb);

        let flash = unsafe { &(*pac::FLASH::ptr()) };

        // Set up pins with appropriate modes.
        setup::setup_pins();

        let mut dma = Dma::new(dp.DMA1);
        let mut dma2 = Dma::new(dp.DMA2);
        #[cfg(feature = "g4")]
        dma::enable_mux1();

        setup::setup_dma(&mut dma, &mut dma2);

        #[cfg(feature = "h7")]
        let uart_crsf = dp.UART7;
        #[cfg(feature = "g4")]
        let uart_crsf = dp.USART3;

        let (mut spi1, mut cs_imu, mut cs_flash, mut i2c1, mut i2c2, uart_osd, mut uart_crsf) =
            setup::setup_busses(dp.SPI1, dp.I2C1, dp.I2C2, dp.USART2, uart_crsf, &clock_cfg);

        let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

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

        // todo temp while we sort out HAL. We've fudged this to make the number come out correctly.
        batt_curr_adc.vdda_calibrated = 3.6;

        let mut update_timer = Timer::new_tim15(
            dp.TIM15,
            UPDATE_RATE_MAIN_LOOP,
            Default::default(),
            &clock_cfg,
        );
        update_timer.enable_interrupt(TimerInterrupt::Update);

        // This timer is used to prevent a stream of continuous RF control signals, should they arrive
        // at any reason, from crashing the FC. Ie, limit damage that can be done from external sources.
        let mut rf_limiter_timer = Timer::new_tim16(
            dp.TIM16,
            safety::MAX_RF_UPDATE_RATE,
            Default::default(),
            &clock_cfg,
        );

        rf_limiter_timer.enable_interrupt(TimerInterrupt::Update);

        // We use this measurement timer to count things, like time between IMU updates.
        // The effective period (1/freq) must be greater than the time we wish to measure.
        let mut measurement_timer =
            Timer::new_tim5(dp.TIM5, 2_000., Default::default(), &clock_cfg);

        // We use this PID adjustment timer to indicate the interval for updating PID from a controller
        // while the switch or button is held. (Equivalently, the min allowed time between actuations)

        let ctrl_coeff_adj_timer = Timer::new_tim1(
            dp.TIM1,
            1. / CTRL_COEFF_ADJ_TIMEOUT,
            TimerConfig {
                one_pulse_mode: true,
                ..Default::default()
            },
            &clock_cfg,
        );

        let motor_timer_cfg = TimerConfig {
            // We use ARPE since we change duty with the timer running.
            auto_reload_preload: true,
            ..Default::default()
        };

        // Frequency here can be arbitrary; we set manually using PSC and ARR below.
        let mut motor_timer = Timer::new_tim3(dp.TIM3, 1., motor_timer_cfg.clone(), &clock_cfg);

        // This timer periodically fire. When it does, we read the value of each of the 4 motor lines
        // in its ISR.
        // todo: Adjust frequency A/R. It should be 5/4 * dshot frequency.
        let mut dshot_read_timer = Timer::new_tim2(
            dp.TIM2,
            // // 25us period, for time between transmission and reception. Modifying during runtime.
            // 1. / 0.000025,
            300_000. * 5. / 4., // todo: Adjust A/R per DSHOT freq.
            Default::default(),
            &clock_cfg,
        );
        dshot_read_timer.enable_interrupt(TimerInterrupt::Update);

        // For fixed wing on H7; need a separate timer from the 4 used for DSHOT.
        let mut servo_timer = Timer::new_tim8(dp.TIM8, 1., motor_timer_cfg, &clock_cfg);

        let mut lost_link_timer = Timer::new_tim17(
            dp.TIM17,
            1. / safety::LOST_LINK_TIMEOUT,
            TimerConfig {
                one_pulse_mode: true,
                ..Default::default()
            },
            &clock_cfg,
        );
        lost_link_timer.enable_interrupt(TimerInterrupt::Update);

        let mut user_cfg = UserCfg::default();
        // todo temp
        user_cfg.waypoints[0] = Some(Location {
            type_: LocationType::LatLon,
            name: [0, 1, 2, 3, 4, 5, 6],
            lon: 1.,
            lat: 2.,
            alt_msl: 3.,
        });

        #[cfg(feature = "quad")]
        flight_ctrls::setup_timers(&mut motor_timer);
        #[cfg(feature = "fixed-wing")]
        flight_ctrls::setup_timers(&mut motor_timer, &servo_timer);

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
            if #[cfg(feature = "h7")] {
                let usb = Usb1::new(
                    dp.OTG1_HS_GLOBAL,
                    dp.OTG1_HS_DEVICE,
                    dp.OTG1_HS_PWRCLK,
                    clock_cfg.hclk(),
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

        // println!(
        //     "mem val: {}",
        //     flash_onboard.read(FLASH_CFG_PAGE, 0)
        // );

        println!("Flash Buf ( should be 1, 2, 3, 0, 0): {:?}", flash_buf);
        // flash_onboard.erase_write_page(Bank::B1, FLASH_CFG_PAGE, &[10, 11, 12, 13, 14, 15, 16, 17]).ok();
        println!("Flash write complete");

        let mut params = Default::default();

        // todo: For params, consider raw readings without DMA. Currently you're just passign in the
        // todo default; not going to cut it.?
        let (system_status, altimeter) = setup::init_sensors(
            &mut params,
            &mut state_volatile.base_point,
            &mut spi1,
            &mut i2c1,
            &mut i2c2,
            &mut cs_imu,
            &mut delay,
        );

        println!(
            "System status:\n IMU: {}, Baro: {}, Mag: {}, GPS: {}, TOF: {}",
            system_status.imu == SensorStatus::Pass,
            system_status.baro == SensorStatus::Pass,
            system_status.magnetometer == SensorStatus::Pass,
            system_status.gps == SensorStatus::Pass,
            system_status.tof == SensorStatus::Pass,
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
        let ahrs_settings = ahrs_fusion::Settings::default();

        // Note: Calibration and offsets ares handled handled by their defaults currently.
        let imu_calibration = imu_calibration::ImuCalibration {
            // gyro_misalignment: Mat3 {
            //     data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            // },
            // gyro_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            // gyro_offset: Vec3::new(0.0, 0.0, 0.0),
            // accel_misalignment: Mat3 {
            //     data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            // },
            // accel_sensitivity: Vec3::new(1.0, 1.0, 1.0),
            // accel_offset: Vec3::new(0.0, 0.0, 0.0),
            ..Default::default()
        }; // todo - load from flash

        let ahrs = Ahrs::new(&ahrs_settings, UPDATE_RATE_FLIGHT_CTRLS as u32);

        // Allow ESC to warm up and the radio to connect before starting the main loop.
        delay.delay_ms(WARMUP_TIME);

        // We must set up the USB device after the warmup delay, since its long blocking delay
        // leads teh host (PC) to terminate the connection. The (short, repeated) blocking delays
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
        let motors_reversed = (
            user_cfg.control_mapping.m1_reversed,
            user_cfg.control_mapping.m2_reversed,
            user_cfg.control_mapping.m3_reversed,
            user_cfg.control_mapping.m4_reversed,
        );

        // todo: temp removed to test bidir
        dshot::setup_motor_dir(motors_reversed, &mut motor_timer);

        crsf::setup(&mut uart_crsf);

        // Start our main loop
        update_timer.enable();

        // Start out IMU-driven loop
        // todo: This is an awk way; Already set up /configured like this in `setup`, albeit with
        // todo opendrain and pullup set, and without enabling interrupt.
        #[cfg(feature = "h7")]
        let mut imu_exti_pin = Pin::new(Port::B, 12, gpio::PinMode::Input);
        #[cfg(feature = "g4")]
        let mut imu_exti_pin = Pin::new(Port::C, 4, gpio::PinMode::Input);
        imu_exti_pin.enable_interrupt(Edge::Falling);

        println!("Init complete; starting main loops");

        unsafe {
            uart_crsf.read_dma(
                &mut crsf::RX_BUFFER,
                setup::CRSF_RX_CH,
                ChannelCfg {
                    // Take precedence over the ADC, but not motors.
                    priority: dma::Priority::Medium,
                    circular: dma::Circular::Enabled, //todo temp
                    ..Default::default()
                },
                setup::CRSF_DMA_PERIPH,
            );
        } //todo temp

        (
            // todo: Make these local as able.
            Shared {
                user_cfg,
                state_volatile,
                system_status,
                autopilot_status: Default::default(),
                current_params: params.clone(),
                params_prev: params,
                control_channel_data: Default::default(),
                link_stats: Default::default(),
                // dma,
                // dma2,
                spi1,
                i2c1,
                i2c2,
                altimeter,
                batt_curr_adc,
                // rtc,
                rf_limiter_timer,
                lost_link_timer,
                link_lost: true, // Initialize to not being on the link
                motor_timer,
                servo_timer,
                usb_dev,
                usb_serial,
                flash_onboard,
                power_used: 0.,
                imu_filters: Default::default(),
                flight_ctrl_filters: Default::default(),
                imu_calibration,
                ext_sensor_active: ExtSensor::Mag,
                pwr_maps: Default::default(),
                motor_pid_state: Default::default(),
                motor_pid_coeffs: Default::default(),
                rotor_rpms: Default::default(),
            },
            Local {
                update_timer,
                uart_crsf,
                // spi_flash, // todo: Fix flash in HAL, then do this.
                arm_signals_received: 0,
                disarm_signals_received: 0,
                update_isr_loop_i: 0,
                imu_isr_loop_i: 0,
                aux_loop_i: 0, // todo t
                ctrl_coeff_adj_timer,
                uart_osd,
                time_with_high_throttle: 0.,
                measurement_timer,
                ahrs,
                dshot_read_timer,
                cs_imu,
            },
            init::Monotonics(),
            // init::Monotonics(measurement_timer)
        )
    }

    #[idle(shared = [], local = [])]
    /// In this function, we perform setup code that must occur with interrupts enabled.
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    // // todo temp
    // #[task(priority = 10)]
    // fn foo(_: foo::Context) {
    //     println!("Mono!");
    //
    //     // Schedule `bar` to run 2 seconds in the future (1 second after foo runs)
    //     // bar::spawn_after(1.secs()).unwrap();
    // }

    // todo: Go through these tasks, and make sure they're not reserving unneded shared params.

    // binds = TIM15,
    // todo: Remove rotor timers, spi, and dma from this ISR; we only use it for tresting DSHOT
    #[task(
    binds = TIM1_BRK_TIM15,
    shared = [current_params,
    power_used, autopilot_status, user_cfg, flight_ctrl_filters,
    control_channel_data, rotor_rpms,
    lost_link_timer, link_lost, altimeter, i2c1, i2c2, state_volatile, system_status,
    batt_curr_adc,
    ],
    local = [arm_signals_received, disarm_signals_received, update_isr_loop_i, uart_osd,
    time_with_high_throttle],

    priority = 5
    )]
    /// This runs periodically, on a ~1kHz timer. It's used to trigger the attitude and velocity PID loops, ie for
    /// sending commands to the attitude and rate PID loop based on things like autopilot, command-mode etc.
    /// We give it a relatively high priority, to ensure it gets run despite faster processes ocurring.
    fn update_isr(mut cx: update_isr::Context) {
        unsafe { (*pac::TIM15::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }
        *cx.local.update_isr_loop_i += 1;

        let mut batt_v = 0.;
        let mut curr_v = 0.;
        cx.shared.batt_curr_adc.lock(|adc| {
            batt_v = adc.reading_to_voltage(unsafe { V_A_ADC_READ_BUF }[0])
                * sensors_shared::ADC_BATT_V_DIV;
            curr_v = adc.reading_to_voltage(unsafe { V_A_ADC_READ_BUF }[1])
                * sensors_shared::ADC_CURR_DIV;
        });

        (
            cx.shared.current_params,
            cx.shared.control_channel_data,
            cx.shared.power_used,
            cx.shared.autopilot_status,
            cx.shared.user_cfg,
            cx.shared.lost_link_timer,
            cx.shared.link_lost,
            cx.shared.altimeter,
            cx.shared.state_volatile,
            cx.shared.system_status,
            cx.shared.rotor_rpms,
            cx.shared.flight_ctrl_filters,
        )
            .lock(
                |params,
                 control_channel_data,
                 power_used,
                 autopilot_status,
                 cfg,
                 lost_link_timer,
                 link_lost,
                 altimeter,
                 state_volatile,
                 system_status,
                 rpms,
                 flight_ctrl_filters| {
                    #[cfg(feature = "print-status")]
                    if *cx.local.update_isr_loop_i % PRINT_STATUS_RATIO == 0 {
                        // todo: Flesh this out, and perhaps make it more like Preflight.

                        // println!("DSHOT: {:?}", unsafe { dshot::PAYLOAD_REC });
                        println!("DSHOT3: {:?}", unsafe { dshot::PAYLOAD_REC_BB_3 });
                        println!("DSHOT4: {:?}", unsafe { dshot::PAYLOAD_REC_BB_4 });

                        println!(
                            "\n\nFaults. Rx: {}. RPM: {}",
                            system_status::RX_FAULT.load(Ordering::Acquire),
                            system_status::RPM_FAULT.load(Ordering::Acquire),
                        );

                        match control_channel_data {
                            Some(ch_data) => {
                                println!(
                                    "\nControl data:\nPitch: {} Roll: {}, Yaw: {}, Throttle: {}, Arm switch: {}",
                                    ch_data.pitch, ch_data.roll,
                                    ch_data.yaw, ch_data.throttle,
                                    ch_data.arm_status == ArmStatus::Armed, // todo fixed-wing
                                );
                            }
                            None => {
                                println!("(No current control channel data)")
                            }
                        }

                        #[cfg(feature = "quad")]
                        println!("Motors armed: {:?}", state_volatile.arm_status == ArmStatus::Armed);

                        #[cfg(feature = "fixed-wing")]
                        println!("Motors armed: {:?}", state_volatile.arm_status == ArmStatus::MotorsArmed ||
                            state_volatile.arm_status == ArmStatus::MotorsControlsArmed);

                        #[cfg(feature = "fixed-wing")]
                        println!("Controls armed: {:?}", state_volatile.arm_status == ArmStatus::ControlsArmed ||
                            state_volatile.arm_status == ArmStatus::MotorsControlsArmed);

                        #[cfg(feature = "quad")]
                        println!(
                            "Autopilot_status: Alt hold: {} Heading hold: {}, Yaw assist: {}, Direct to point: {}, \
                            sequence: {}, takeoff: {}, land: {}, recover: {}, loiter: {}",
                            autopilot_status.alt_hold.is_some(), autopilot_status.hdg_hold.is_some(),
                            autopilot_status.yaw_assist != flight_ctrls::autopilot::YawAssist::Disabled,
                            autopilot_status.direct_to_point.is_some(),
                            autopilot_status.sequence, autopilot_status.takeoff, autopilot_status.land.is_some(),
                            autopilot_status.recover.is_some(),
                            autopilot_status.loiter.is_some(),
                        );

                        #[cfg(feature = "fixed-wing")]
                        println!(
                            "Autopilot_status: Alt hold: {} Heading hold: {}, Direct to point: {}, \
                            sequence: {}, takeoff: {}, land: {}, recover: {}, loiter/orbit: {}",
                            autopilot_status.alt_hold.is_some(), autopilot_status.hdg_hold.is_some(),
                            autopilot_status.direct_to_point.is_some(),
                            autopilot_status.sequence, autopilot_status.takeoff, autopilot_status.land.is_some(),
                            autopilot_status.recover.is_some(),
                            autopilot_status.orbit.is_some(),
                        );

                        println!("Batt V: {} ESC current: {}", state_volatile.batt_v, state_volatile.esc_current);
                        //
                        // println!(
                        //     "Accel: Ax {}, Ay: {}, Az: {}",
                        //     params.a_x, params.a_y, params.a_z
                        // );
                        // //
                        // println!(
                        //     "Gyro: roll {}, pitch: {}, yaw: {}",
                        //     params.v_roll, params.v_pitch, params.v_yaw
                        // );
                        // // //
                        println!("Attitude quat: {} {} {} {}",
                                 params.attitude_quat.w,
                                 params.attitude_quat.x,
                                 params.attitude_quat.y,
                                 params.attitude_quat.z
                        );
                        println!(
                            "Attitude: pitch: {}, roll: {}, yaw: {}\n",
                            params.s_pitch, params.s_roll, params.s_yaw_heading
                        );

                        if let Some(q) = state_volatile.attitude_commanded.quat {
                            println!("Commanded attitude quat: {} {} {} {}",
                                     q.w,
                                     q.x,
                                     q.y,
                                     q.z
                            );

                            let (p, r, y) = q.to_euler();
                            println!("Commanded attitude: pitch: {}, roll: {}, yaw: {}\n", p, r, y);
                        }


                        println!(
                            "RPMs: FL {}, FR: {}, AL: {}, AR: {}\n",
                            rpms.front_left, rpms.front_right, rpms.aft_left, rpms.aft_right
                        );

                        // println!("In acro mode: {:?}", *input_mode == InputMode::Acro);
                        // println!(
                        //     "Input mode sw: {:?}",
                        //     control_channel_data.input_mode == InputModeSwitch::Acro
                        // );

                    }
                    // if *cx.local.update_isr_loop_i % LOGGING_UPDATE_RATIO == 0 {
                    // todo: Eg log params to flash etc.
                    // }

                    let (arm_status, throttle) = match control_channel_data {
                        Some(ch_data) => (ch_data.arm_status, ch_data.throttle),
                        None => (ArmStatus::Disarmed, 0.),
                    };
                    safety::handle_arm_status(
                        cx.local.arm_signals_received,
                        cx.local.disarm_signals_received,
                        arm_status,
                        &mut state_volatile.arm_status,
                        throttle,
                    );

                    if !state_volatile.has_taken_off {
                        safety::handle_takeoff_attitude_lock(
                            throttle,
                            &mut cx.local.time_with_high_throttle,
                            &mut state_volatile.has_taken_off,
                            DT_MAIN_LOOP,
                        );
                    }

                    #[cfg(feature = "quad")]
                    if let Some(ch_data) = control_channel_data {
                        flight_ctrls::set_input_mode(ch_data.input_mode, state_volatile, system_status);
                    }

                    // todo: Support UART telemetry from ESC.

                    // todo: Determine timing for OSD update, and if it should be in this loop,
                    // todo, or slower.

                    // todo: Find the current conversion factor. Probably slope + y int
                    let esc_current = curr_v;

                    state_volatile.batt_v = batt_v;
                    state_volatile.esc_current = esc_current;

                    // todo: Put back A/R
                    // This difference in approach between quad and fixed-wing for the
                    // control deltas is due to using an intermediate step between control settings
                    // and accel for quads (RPM), but doing it directly on fixed-wing,being unable
                    // to measure servo posit directly (which would be the equiv intermediate step)
                    // cfg_if! {
                    //     if #[cfg(feature = "quad")] {
                    //         let pitch_delta = rpms.pitch_delta();
                    //         let roll_delta = rpms.roll_delta();
                    //         let yaw_delta = rpms.yaw_delta(cfg.control_mapping.frontleft_aftright_dir);
                    //
                    //         let pitch_accel = state_volatile.accel_map.pitch_rpm_to_accel(pitch_delta);
                    //         let roll_accel = state_volatile.accel_map.roll_rpm_to_accel(roll_delta);
                    //         let yaw_accel = state_volatile.accel_map.yaw_rpm_to_accel(yaw_delta);
                    //     } else {
                    //         let pitch_delta = state_volatile.ctrl_positions.pitch_delta();
                    //         let roll_delta = state_volatile.ctrl_positions.roll_delta();
                    //         let yaw_delta = state_volatile.ctrl_positions.yaw_delta();
                    //
                    //         let pitch_accel = state_volatile.accel_map.pitch_cmd_to_accel(pitch_delta);
                    //         let roll_accel = state_volatile.accel_map.roll_cmd_to_accel(roll_delta);
                    //         let yaw_accel = state_volatile.accel_map.yaw_cmd_to_accel(yaw_delta);
                    //     }
                    // }
                    //
                    // // Estimate the angular drag coefficient for the current flight regime.
                    // let drag_coeff_pitch = flight_ctrls::ctrl_logic::calc_drag_coeff(
                    //     params.v_pitch,
                    //     params.a_pitch,
                    //     pitch_accel,
                    // );
                    //
                    // let drag_coeff_roll = flight_ctrls::ctrl_logic::calc_drag_coeff(
                    //     params.v_roll,
                    //     params.a_roll,
                    //     state_volatile.accel_map.roll_rpm_to_accel(roll_delta),
                    // );
                    //
                    // #[cfg(feature = "quad")]
                    //     let drag_coeff_yaw = flight_ctrls::ctrl_logic::calc_drag_coeff(
                    //     params.v_yaw,
                    //     params.a_yaw,
                    //     state_volatile.accel_map.yaw_rpm_to_accel(yaw_delta),
                    // );
                    //
                    // let (dcp, dcr, dcy) = flight_ctrl_filters.apply(drag_coeff_pitch, drag_coeff_roll, drag_coeff_yaw);
                    // state_volatile.drag_coeffs.pitch = dcp;
                    // state_volatile.drag_coeffs.roll = dcr;
                    // state_volatile.drag_coeffs.yaw = dcy;

                    let osd_data = OsdData {
                        arm_status: state_volatile.arm_status,
                        battery_voltage: batt_v,
                        current_draw: esc_current,
                        alt_msl_baro: params.baro_alt_msl,
                        gps_fix: Location::default(),
                        pitch: params.s_pitch,
                        roll: params.s_roll,
                        yaw: params.s_yaw_heading,
                        // pid_p: coeffs.roll.k_p_rate,
                        // pid_i: coeffs.roll.k_i_rate,
                        // pid_d: coeffs.roll.k_d_rate,
                        autopilot: AutopilotData {
                            takeoff: autopilot_status.takeoff,
                            land: autopilot_status.land.is_some(),
                            direct_to_point: autopilot_status.direct_to_point.is_some(),
                            #[cfg(feature = "fixed-wing")]
                            orbit: autopilot_status.orbit.is_some(),
                            alt_hold: autopilot_status.alt_hold.is_some(),
                            #[cfg(feature = "quad")]
                            loiter: autopilot_status.loiter.is_some(),
                        },
                        base_dist_bearing: (
                            0., 0., // todo: Fill these out
                        ),
                    };

                    // todo: put back
                    // osd::send_osd_data(cx.local.uart_osd, setup::OSD_CH,&osd_data);

                    if let Some(ch_data) = control_channel_data {
                        autopilot_status.set_modes_from_ctrls(ch_data, &params);
                    }

                    #[cfg(feature = "quad")]
                        let ap_cmds = autopilot_status.apply(
                        params,
                        // filters,
                        // coeffs,
                        system_status,
                    );

                    #[cfg(feature = "fixed-wing")]
                        let ap_cmds = autopilot_status.apply(
                        params,
                        // pid_attitude,
                        // filters,
                        // coeffs,
                        system_status,
                    );

                    // Don't apply autopilot modes if on the ground.
                    if !state_volatile.has_taken_off {
                        // The intermediate variable is due to a attribute binding
                        // issue with teh direct approach.
                        state_volatile.autopilot_commands = ap_cmds;
                    }

                    // if let OperationMode::Preflight = state_volatile.op_mode {
                    //     // exit this fn during preflight *after* measuring voltages using ADCs.
                    //     return;
                    // }

                    // todo: This should probably be delegatd to a fn; get it
                    // todo out here
                    if *cx.local.update_isr_loop_i % THRUST_LOG_RATIO == 0 {
                        cfg_if! {
                            if #[cfg(feature = "quad")] {
                                state_volatile.power_maps.rpm_to_accel_pitch.log_val(
                                // todo: Populate this, and consider if you want rpms to be by motor or rotor posit
                                //     pwr.front_left + pwr.front_right - pwr.aft_left - pwr.aft_right,
                                    // rpms.m1 + rpms.m2 + rpms.m3 + rpms.m4
                                    // todo: Motors. Map Motor num to rotor position here.
                                    // todo: Possibly with helper methods.
                                    0.,
                                    0.,
                                );

                                state_volatile.power_maps.rpm_to_accel_roll.log_val(
                                    0.,
                                    0.,
                                );

                                let mut yaw_pwr = 0.;
                                if cfg.control_mapping.frontleft_aftright_dir == RotationDir::Clockwise {
                                    yaw_pwr *= -1.;
                                }
                                state_volatile.power_maps.rpm_to_accel_yaw.log_val(
                                    yaw_pwr,
                                    0.,
                                );
                            }
                        }
                        // Note: We currently don't have a way to measure servo position,
                        // so we leave the default 1:1 mapping here.
                    }
                });

        (cx.shared.i2c1, cx.shared.i2c2).lock(|i2c1, i2c2| {
            // Start DMA sequences for I2C sensors, ie baro, mag, GPS, TOF.
            // DMA TC isrs are sequenced.
            // todo: Put back; Troubleshooting control and motors issues.
            sensors_shared::start_transfers(i2c1, i2c2);
        });
    }

    /// Runs when new IMU data is ready. Trigger a DMA read.
    /// High priority since it's important, and quick-to-execute
    #[task(binds = EXTI4, shared = [spi1], local = [], priority = 8)]
    fn imu_data_isr(mut cx: imu_data_isr::Context) {
        gpio::clear_exti_interrupt(4);

        cx.shared.spi1.lock(|spi| {
            imu_shared::read_imu(imu::READINGS_START_ADDR, spi, setup::IMU_DMA_PERIPH);
        });
    }

    // binds = DMA1_STR2,
    #[task(binds = DMA1_CH2, shared = [spi1, current_params, params_prev, control_channel_data,
    autopilot_status, imu_filters, flight_ctrl_filters, user_cfg, motor_pid_state, motor_pid_coeffs,
    motor_timer, servo_timer, state_volatile], local = [ahrs, imu_isr_loop_i, cs_imu], priority = 4)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it nominally (and according to our measurements so far) runs at 8kHz.
    /// Note that on the H7 FC with the dedicated IMU LSE, it may run slightly faster.
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        dma::clear_interrupt(
            setup::IMU_DMA_PERIPH,
            setup::IMU_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        cx.local.cs_imu.set_high();

        // todo: Try to get the stop DMA thing here not sharing SPI1, so we can have the `imu_data_isr`
        // todo not use any locks.
        // The commented out code below will do it, but in this case, probably not required.

        // dma::stop(setup::IMU_DMA_PERIPH, setup::IMU_TX_CH,);
        // dma::stop(setup::IMU_DMA_PERIPH, setup::IMU_RX_CH,);
        //
        // #[cfg(feature = "h7")]
        // (*pac::SPI1::ptr()).modify(|_, w| {
        //     w.txdmaen().clear_bit();
        //     w.rxdmaen().clear_bit()
        // });
        // #[cfg(feature = "g4")]
        // (*pac::SPI1::ptr()).modify(|_, w| {
        //     w.txdmaen().clear_bit();
        //     w.rxdmaen().clear_bit()
        // });
        //
        cx.shared.spi1.lock(|spi1| {
            // Note that this step is mandatory, per STM32 RM.
            spi1.stop_dma(setup::IMU_TX_CH, Some(setup::IMU_RX_CH), DmaPeriph::Dma1);
        });

        *cx.local.imu_isr_loop_i += 1;

        // // Note that our filter block size
        // if *cx.local.imu_isr_loop_i % FLIGHT_CTRL_IMU_RATIO != 0 {
        //     return
        // }

        (
            cx.shared.current_params,
            cx.shared.params_prev,
            cx.shared.control_channel_data,
            cx.shared.autopilot_status,
            cx.shared.motor_timer,
            cx.shared.servo_timer,
            cx.shared.motor_pid_state,
            cx.shared.motor_pid_coeffs,
            cx.shared.user_cfg,
            cx.shared.state_volatile,
            cx.shared.flight_ctrl_filters,
        )
            .lock(
                |params,
                 params_prev,
                 control_channel_data,
                 autopilot_status,
                 motor_timer,
                 servo_timer,
                 pid_state,
                 pid_coeffs,
                 cfg,
                 state_volatile,
                 flight_ctrl_filters| {
                    {
                        // if *cx.local.imu_isr_loop_i % 700 == 0 {
                        // let period = cx.local.measurement_timer.time_elapsed().as_secs();
                        // println!("IMU time: {:?}. Freq: {:?}", period, 1. / period);
                        // }

                        // cx.local.measurement_timer.disable();
                        // cx.local.measurement_timer.reset_count();
                        // cx.local.measurement_timer.enable();
                    }

                    let mut imu_data =
                        imu_shared::ImuReadings::from_buffer(unsafe { &imu_shared::IMU_READINGS });

                    cx.shared.imu_filters.lock(|imu_filters| {
                        imu_filters.apply(&mut imu_data);
                    });

                    // Update `params_prev` with past-update data prior to updating params
                    *params_prev = params.clone();
                    params.update_from_imu_readings(imu_data);

                    // Update our flight control logic and motors a fraction of IMU updates, but
                    // apply filter data to all.
                    // todo: Consider a different approach; all you need to do each time is
                    // todo read and filter.
                    // todo: Be wary of how you use params_prev if it's above this break line

                    if *cx.local.imu_isr_loop_i % FLIGHT_CTRL_IMU_RATIO != 0 {
                        return
                    }

                    // Note: Consider if you want to update the attitude using the primary update loop,
                    // vice each IMU update.
                    attitude_platform::update_attitude(cx.local.ahrs, params);

                    // todo: Temp debug code.
                    match control_channel_data {
                        Some(ch_data) => {
                            let mut p = ch_data.throttle;
                            if p < 0.025 {
                                p = 0.025;
                            }
                            if state_volatile.arm_status == ArmStatus::Armed {
                                dshot::set_power(p, p, p, p, motor_timer);
                            } else {
                                dshot::stop_all(motor_timer);
                            }
                        }
                        None => {
                            dshot::stop_all(motor_timer);
                        }
                    };

                    // todo: Impl once you've sorted out your control logic.
                    // todo: Delegate this to another module, eg `attitude_ctrls`.
                    // Update the target attitude based on control inputs
                    // todo: Deconflict this with autopilot; probably by checking commanded
                    // todo pitch, roll, yaw etc!

                    // todo: You probably need to examine your types `RatesCommanded` and `CtrlInputs`.
                    // todo: When are `RatesCommadned` values None?

                    // todo: This whole section between here and the `ctrl_logic` calls is janky!
                    // todo you need to properly blend manual controls and autopilot, and handle
                    // lost-link procedures properly (Which may have been encoded in autopilot elsewhere)

                    // Update our commanded attitude
                    match control_channel_data {
                        Some(ch_data) => {
                            // let rates_commanded = RatesCommanded {
                            //     pitch: Some(cfg.input_map.calc_pitch_rate(ch_data.pitch)),
                            //     roll: Some(cfg.input_map.calc_roll_rate(ch_data.roll)),
                            //     yaw: Some(cfg.input_map.calc_yaw_rate(ch_data.yaw)),
                            // };

                            let pitch = cfg.input_map.calc_pitch_rate(ch_data.pitch);
                            let roll = cfg.input_map.calc_roll_rate(ch_data.roll);
                            let yaw = cfg.input_map.calc_yaw_rate(ch_data.yaw);


                            // If we haven't taken off, apply the attitude lock.
                            if state_volatile.has_taken_off {
                                state_volatile.attitude_commanded.quat = Some(ctrl_logic::modify_att_target(
                                    state_volatile.attitude_commanded.quat.unwrap_or(Quaternion::new_identity()),
                                    pitch, roll, yaw,
                                    DT_FLIGHT_CTRLS,
                                ));
                            } else {
                                state_volatile.attitude_commanded.quat = Some(cfg.takeoff_attitude);
                            }
                        }
                        None => {

                        }
                    };

                    // todo: Are the attitude_commanded fields other than quat used? Should we remove them?

                    // todo: Here, or in a subfunction, blend in autopiot commands! Currently not applied,
                    // todo other than throttle.

                    let throttle = match state_volatile.autopilot_commands.throttle {
                        Some(t) => t,
                        None => {
                            match control_channel_data {
                                Some(ch_data) => ch_data.throttle,
                                None => 0.,
                            }
                        },
                    };

                    if let OperationMode::Preflight = state_volatile.op_mode {
                        return;
                    }

                    return; // todo TS

                    cfg_if! {
                        if #[cfg(feature = "quad")] {
                            let (ctrl_mix, rpms) = ctrl_logic::rotor_rpms_from_att(
                                state_volatile.attitude_commanded.quat.unwrap(),
                                params.attitude_quat,
                                throttle,
                                cfg.control_mapping.frontleft_aftright_dir,
                                params,
                                params_prev,
                                &cfg.ctrl_coeffs,
                                &state_volatile.drag_coeffs,
                                &state_volatile.accel_map,
                                flight_ctrl_filters,
                                DT_FLIGHT_CTRLS,
                            );

                            // todo: We're not accessing the RotorRpms sturct here...
                            // todo: What is that used for etc?

                            rpms.send_to_motors(
                                pid_coeffs,
                                pid_state,
                                &rpms,
                                &cfg.control_mapping,
                                motor_timer,
                                state_volatile.arm_status,
                            );

                            state_volatile.ctrl_mix = ctrl_mix;
                            // todo: Dynamics of `current_pwr` for quads, and `ctrl_posits`
                            // todo for fixed-wing. You're saving it, but when is it used?
                            // state_volatile.current_pwr = motor_power;
                        } else {
                            let (ctrl_mix, control_posits) = ctrl_logic::control_posits_from_att(
                                state_volatile.attitude_commanded.quat.unwrap(),
                                params.attitude_quat,
                                throttle,
                                params,
                                params_prev,
                                // &state_volatile.ctrl_mix,
                                &cfg.ctrl_coeffs,
                                &state_volatile.drag_coeffs,
                                &state_volatile.accel_map,
                                flight_ctrl_filters,
                                DT_FLIGHT_CTRLS,
                            );

                            control_posits.set(&cfg.control_mapping, motor_timers, state_volatile.arm_status,  dma);

                            state_volatile.ctrl_mix = ctrl_mix;
                            state_volatile.ctrl_positions = control_posits;
                        }
                    }
                },
            );
    }

    // binds = OTG_HS
    // todo H735 issue on GH: https://github.com/stm32-rs/stm32-rs/issues/743 (works on H743)
    // todo: NVIC interrupts missing here for H723 etc!
    #[task(binds = USB_LP, shared = [usb_dev, usb_serial, current_params, control_channel_data,
    link_stats, user_cfg, state_volatile, system_status, motor_timer, servo_timer, batt_curr_adc], local = [], priority = 2)]
    /// This ISR handles interaction over the USB serial port, eg for configuring using a desktop
    /// application.
    fn usb_isr(cx: usb_isr::Context) {
        // todo: Do we want to use an approach where we push stats, or this approach where
        // todo respond only?
        (
            cx.shared.usb_dev,
            cx.shared.usb_serial,
            cx.shared.current_params,
            cx.shared.control_channel_data,
            cx.shared.link_stats,
            cx.shared.user_cfg,
            cx.shared.state_volatile,
            cx.shared.system_status,
            cx.shared.motor_timer,
            cx.shared.servo_timer,
            cx.shared.batt_curr_adc,
        )
            .lock(
                |usb_dev,
                 usb_serial,
                 params,
                 ch_data,
                 link_stats,
                 user_cfg,
                 state_volatile,
                 system_status,
                 motor_timer,
                 servo_timer,
                 adc| {
                    if !usb_dev.poll(&mut [usb_serial]) {
                        return;
                    }

                    let mut buf = [0u8; 8];
                    match usb_serial.read(&mut buf) {
                        Ok(_count) => {
                            usb_preflight::handle_rx(
                                usb_serial,
                                &buf,
                                params.attitude_quat,
                                &state_volatile.attitude_commanded,
                                params.baro_alt_msl,
                                params.tof_alt,
                                state_volatile.batt_v,
                                state_volatile.esc_current,
                                ch_data,
                                &link_stats,
                                &user_cfg.waypoints,
                                system_status,
                                &mut state_volatile.arm_status,
                                &mut user_cfg.control_mapping,
                                &mut state_volatile.op_mode,
                                motor_timer,
                                servo_timer,
                                adc,
                            );
                        }
                        Err(_) => {
                            // println!("Error reading USB signal from PC");
                        }
                    }
                },
            )
    }

    // #[task(binds = DMA1_STR3, shared = [], priority = 6)]
    #[task(binds = DMA1_CH3, shared = [], priority = 6)]
    /// We use this ISR to initialize the RPM reception procedures upon completion of the dshot
    /// power setting transmission to the ESC.
    fn dshot_isr(_cx: dshot_isr::Context) {
        dma::clear_interrupt(
            setup::MOTORS_DMA_PERIPH,
            setup::MOTOR_CH,
            DmaInterrupt::TransferComplete,
        );

        // (From testing) We must stop this transaction manually efore future transactions will work.
        dma::stop(setup::MOTORS_DMA_PERIPH, setup::MOTOR_CH);

        if dshot::BIDIR_EN {
            dshot::receive_payload();
        }

        // Shared resource here seems to cause scheduling jitter.
        unsafe {
            (*pac::TIM2::ptr()).cr1.modify(|_, w| w.cen().clear_bit());
        }

        // cx.shared.motor_timer.lock(|motor_timer| {
        //     motor_timer.disable();

        // if dshot::BIDIR_EN {
        //     dshot::receive_payload(motor_timer);
        // }

        // todo: you might  need a slightly delay etc to make sure you're reading towards
        // todo: the middle of a pulse vice the start?

        // if dshot::BIDIR_EN {
        //     if dshot::DSHOT_REC_MODE.load(Ordering::Relaxed) {
        //         // todo: We currently have it set to output mode in `dshot::send_payload`.
        //         // dshot::set_to_output(motor_timer);
        //
        //         // dshot::DSHOT_REC_MODE.store(false, Ordering::Relaxed);
        //     } else {
        //         dshot::receive_payload(motor_timer);
        //
        //         // // todo: Move to `receive_payload` fn on this atomic op?
        //         // dshot::DSHOT_REC_MODE.store(true, Ordering::Relaxed);
        //     }
        //
        // }
        // });
    }

    #[task(binds = EXTI1, shared = [], local = [], priority = 7)]
    /// Start the DSHOT RPM read timer on the first low edge of this pin. (Currently Motor 3.)
    fn dshot_read_start_isr(_cx: dshot_read_start_isr::Context) {
        gpio::clear_exti_interrupt(1);

        // Shared resource here seems to cause scheduling jitter and missed bits.
        unsafe {
            (*pac::TIM2::ptr()).cr1.modify(|_, w| w.cen().set_bit());
        }

        // todo: The setting at the buf complete isn't working. wtf
        dshot::READ_I.store(0, Ordering::Release);

        // todo: Investiate if this is firing twice etc.

        let exti = unsafe { &(*pac::EXTI::ptr()) };
        #[cfg(feature = "h7")]
        exti.cpuimr1.modify(|_, w| w.mr6().clear_bit());
        #[cfg(feature = "g4")]
        exti.ftsr1.modify(|_, w| w.ft1().clear_bit());
    }

    #[task(binds = TIM2, shared = [rotor_rpms], local = [dshot_read_timer], priority = 7)]
    /// We use this ISR to, bit-by-bit, fill the buffer when receiving RPM data. It also handles
    /// termination of the reception timer. Started on the first low edge of the Motor 1 pin.
    fn dshot_read_isr(mut cx: dshot_read_isr::Context) {
        cx.local
            .dshot_read_timer
            .clear_interrupt(TimerInterrupt::Update);

        // todo: Figure out a way to use DMA instead of this?

        cfg_if! {
            if #[cfg(feature = "h7")] {
                let m1_val = gpio::is_high(gpio::Port::C, 6);
                let m2_val = gpio::is_high(gpio::Port::C, 7);
                let m3_val = gpio::is_high(gpio::Port::C, 8);
                let m4_val = gpio::is_high(gpio::Port::C, 9);
            } else {
                // let m1_val = gpio::is_high(gpio::Port::C, 6);
                // let m2_val = gpio::is_high(gpio::Port::A, 4);
                let m3_val = gpio::is_high(gpio::Port::B, 0);
                let m4_val = gpio::is_high(gpio::Port::B, 1);
            }
        }

        let i = dshot::READ_I.fetch_add(1, Ordering::Acquire);

        // We have read the entire payload.
        if i == dshot::REC_BUF_LEN - 1 {
            cx.local.dshot_read_timer.disable();

            // todo: The setting at the buf complete isn't working. wtf
            // dshot::READ_I.store(0, Ordering::Release);
        } else if i > dshot::REC_BUF_LEN - 1 {
            return; // todo: Why is this required?
        }

        unsafe {
            // dshot::PAYLOAD_REC_BB_1[i] = m1_val;
            // dshot::PAYLOAD_REC_BB_2[i] = m2_val;
            dshot::PAYLOAD_REC_BB_3[i] = m3_val;
            dshot::PAYLOAD_REC_BB_4[i] = m4_val;
        }

        let mut rpm_fault = false;

        cx.shared.rotor_rpms.lock(|rotor_rpms| {
            dshot::update_rpms(rotor_rpms, &mut rpm_fault);
        });

        if rpm_fault {
            system_status::RPM_FAULT.store(true, Ordering::Release);
        }
    }

    // todo: Evaluate priority.
    // #[task(binds = USART7,
    #[task(binds = USART3,
    shared = [control_channel_data, link_stats, rf_limiter_timer, link_lost,
    lost_link_timer, system_status], local = [uart_crsf], priority = 4)]
    /// This ISR handles CRSF reception. It handles, in an alternating fashion, message starts,
    /// and message ends. For message starts, it begins a DMA transfer. For message ends, it
    /// processes the radio data, passing it into shared resources for control channel data,
    /// and link stats.
    fn crsf_isr(mut cx: crsf_isr::Context) {
        let uart = &mut cx.local.uart_crsf; // Code shortener

        let mut recieved_ch_data = false; // Lets us split up the lock a bit more.
        let mut rx_fault = false;

        uart.clear_interrupt(UsartInterrupt::CharDetect(None));
        uart.clear_interrupt(UsartInterrupt::Idle);

        (
            cx.shared.control_channel_data,
            cx.shared.link_stats,
            cx.shared.rf_limiter_timer,
        )
            .lock(|ch_data, link_stats, limiter_timer| {
                // todo: Attempting a software flag vice using interrupt flags, to TS CRSF
                // todo anomolies.
                if !crsf::TRANSFER_IN_PROG.load(Ordering::Relaxed) {
                    crsf::TRANSFER_IN_PROG.store(true, Ordering::Relaxed);

                    // Don't allow the starting char, as used in the middle of a message,
                    // to trigger an interrupt.
                    uart.disable_interrupt(UsartInterrupt::CharDetect(None));

                    // todo: Deal with this later.
                    // if limiter_timer.is_enabled() {
                    //     // todo: This is triggering off link stats. Find a way to accept that, but still
                    //     // todo cancel immediately. (?)
                    //     // println!("Time since last req: {}", limiter_timer.time_elapsed().as_secs());
                    //     println!("RF limiter triggered.");
                    //     // return; // todo
                    // } else {
                    //     limiter_timer.disable();
                    //     limiter_timer.reset_count();
                    //     limiter_timer.enable();
                    // }

                    // todo?
                    // dma::stop(setup::CRSF_DMA_PERIPH, setup::CRSF_RX_CH);

                    unsafe {
                        uart.read_dma(
                            &mut crsf::RX_BUFFER,
                            setup::CRSF_RX_CH,
                            ChannelCfg {
                                // Take precedence over the ADC, but not motors.
                                priority: dma::Priority::Medium,
                                ..Default::default()
                            },
                            setup::CRSF_DMA_PERIPH,
                        );
                    }
                } else {
                    crsf::TRANSFER_IN_PROG.store(false, Ordering::Relaxed);
                    // Line is idle.

                    // Stop the DMA read, since it will likely not have filled the buffer, due
                    // to the variable message sizes.
                    dma::stop(setup::CRSF_DMA_PERIPH, setup::CRSF_RX_CH);

                    // A `None` value here re-enables the interrupt without changing the char to match.
                    uart.enable_interrupt(UsartInterrupt::CharDetect(None));

                    if let Some(crsf_data) =
                        crsf::handle_packet(uart, setup::CRSF_RX_CH, &mut rx_fault)
                    {
                        match crsf_data {
                            crsf::PacketData::ChannelData(data) => {
                                *ch_data = Some(data);
                                recieved_ch_data = true;

                                // We have this PID adjustment here, since they're one-off actuations.
                                // We handle other things like autopilot mode entry in the update fn.
                                // if cx.local.ctrl_coeff_adj_timer.is_enabled() {
                                //     println!("PID timer is still running.");
                                // } else {
                                //     let pid_adjustment = match ch_data.pid_tune_actuation {
                                //         PidTuneActuation::Increase => CTRL_COEFF_ADJ_AMT,
                                //         PidTuneActuation::Decrease => -CTRL_COEFF_ADJ_AMT,
                                //         PidTuneActuation::Neutral => 0.,
                                //     };
                                //
                                //     match ch_data.pid_tune_actuation {
                                //         PidTuneActuation::Neutral => (),
                                //         _ => {
                                //             println!("Adjusting PID");
                                //             // match ch_data.pid_tune_mode {
                                //             //     PidTuneMode::Disabled => (),
                                //             //     PidTuneMode::P => {
                                //             //         // todo: for now or forever, adjust pitch, roll, yaw
                                //             //         // todo at once to keep UI simple
                                //             //         ctrl_coeffs.pitch.k_p_rate += pid_adjustment;
                                //             //         ctrl_coeffs.roll.k_p_rate += pid_adjustment;
                                //             //         // todo: Maybe skip yaw here?
                                //             //         ctrl_coeffs.yaw.k_p_rate += pid_adjustment;
                                //             //     }
                                //             //     PidTuneMode::I => {
                                //             //         ctrl_coeffs.pitch.k_i_rate += pid_adjustment;
                                //             //         ctrl_coeffs.roll.k_i_rate += pid_adjustment;
                                //             //         ctrl_coeffs.yaw.k_i_rate += pid_adjustment;
                                //             //     }
                                //             //     PidTuneMode::D => {
                                //             //         ctrl_coeffs.pitch.k_d_rate += pid_adjustment;
                                //             //         ctrl_coeffs.roll.k_d_rate += pid_adjustment;
                                //             //         ctrl_coeffs.yaw.k_d_rate += pid_adjustment;
                                //             //     }
                                //             // }
                                //         }
                                //     }
                                //     cx.local.ctrl_coeff_adj_timer.reset_count();
                                //     cx.local.ctrl_coeff_adj_timer.enable();
                                // }
                            }
                            crsf::PacketData::LinkStats(stats) => {
                                *link_stats = stats;
                            }
                        }
                    }
                }
            });

        (cx.shared.link_lost, cx.shared.lost_link_timer, cx.shared.system_status).lock(|link_lost, lost_link_timer, system_status| {
            if recieved_ch_data {
                // We've received a packet successfully - reset the lost-link timer.
                lost_link_timer.disable();
                lost_link_timer.reset_count();
                lost_link_timer.enable();

                if *link_lost {
                    println!("Link re-aquired");
                    *link_lost = false;
                    // todo: Execute re-acq procedure
                }
                system_status.rf_control_link = SensorStatus::Pass;
            }

            if rx_fault {
                system_status::RX_FAULT.store(true, Ordering::Release);
            }
        });
    }

    /// If this triggers, it means we've received no radio control signals for a significant
    ///period of time; we treat this as a lost-link situation.
    /// (Note that this is for TIM17 on both variants)
    // #[task(binds = TIM17,
    #[task(binds = TIM1_TRG_COM,
    shared = [lost_link_timer, link_lost, state_volatile, autopilot_status,
    current_params, system_status, control_channel_data], priority = 1)]
    fn lost_link_isr(mut cx: lost_link_isr::Context) {
        println!("Lost the link!");

        cx.shared.lost_link_timer.lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
            timer.reset_count();
        });

        cx.shared.link_lost.lock(|link_lost| {
            *link_lost = true;
        });

        cx.shared.control_channel_data.lock(|ch_data| {
            *ch_data = None;
        });

        (
            cx.shared.state_volatile,
            cx.shared.autopilot_status,
            cx.shared.current_params,
            cx.shared.system_status,
        )
            .lock(|state_volatile, autopilot_status, params, system_status| {
                // We run this during the main loop, but here the `entering` flag is set to true,
                // to initialize setup steps.
                safety::link_lost(
                    system_status,
                    autopilot_status,
                    params,
                    &state_volatile.base_point,
                );
            });
    }

    #[task(binds = TIM1_UP_TIM16, shared = [rf_limiter_timer], priority = 1)]
    fn rf_limiter_isr(mut cx: rf_limiter_isr::Context) {
        // println!("RF limiter ISR");
        cx.shared.rf_limiter_timer.lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
            timer.reset_count();
        });
    }

    // binds = DMA2_STR1,
    #[task(binds = DMA2_CH1,
    shared = [i2c2], priority = 1)]
    /// Baro write complete; start baro read.
    fn baro_write_tc_isr(mut cx: baro_write_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_TX_CH);

        println!("Ext sensors D");
        cx.shared.i2c2.lock(|i2c2| unsafe {
            i2c2.read_dma(
                baro::ADDR,
                &mut sensors_shared::BARO_READINGS,
                setup::BARO_RX_CH,
                Default::default(),
                setup::BARO_DMA_PERIPH,
            );
        });
    }

    // todo: For now, we start new transfers in the main loop.

    // binds = DMA2_STR2,
    #[task(binds = DMA2_CH2,
    shared = [altimeter, current_params], priority = 1)]
    /// Baro read complete; handle data, and start next write.
    fn baro_read_tc_isr(cx: baro_read_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_RX_CH);

        println!("Ext sensors C");
        (cx.shared.altimeter, cx.shared.current_params).lock(|altimeter, params| {
            // code shortener.
            let buf = unsafe { &sensors_shared::BARO_READINGS };
            // todo: Process your baro reading here.
            let pressure =
                altimeter.pressure_from_readings(buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

            // todo: Altitude from pressure! Maybe in a diff module? (which?)
            params.baro_alt_msl = pressure;
        });
    }

    // binds = DMA2_STR3,
    #[task(binds = DMA2_CH3,
    shared = [i2c1, ext_sensor_active], priority = 1)]
    /// External sensors write complete; start external sensors read.
    fn ext_sensors_write_tc_isr(cx: ext_sensors_write_tc_isr::Context) {
        dma::clear_interrupt(
            setup::EXT_SENSORS_DMA_PERIPH,
            setup::EXT_SENSORS_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::EXT_SENSORS_DMA_PERIPH, setup::EXT_SENSORS_TX_CH);

        println!("Ext sensors B");
        (cx.shared.i2c1, cx.shared.ext_sensor_active).lock(|i2c1, ext_sensor_active| {
            // todo: Skip sensors if marked as not connected?

            unsafe {
                match ext_sensor_active {
                    ExtSensor::Mag => {
                        i2c1.read_dma(
                            mag::ADDR,
                            &mut sensors_shared::MAG_READINGS,
                            setup::EXT_SENSORS_RX_CH,
                            Default::default(),
                            setup::EXT_SENSORS_DMA_PERIPH,
                        );
                    }
                    ExtSensor::Gps => {
                        i2c1.read_dma(
                            gps::ADDR,
                            &mut sensors_shared::GPS_READINGS,
                            setup::EXT_SENSORS_RX_CH,
                            Default::default(),
                            setup::EXT_SENSORS_DMA_PERIPH,
                        );
                    }
                    ExtSensor::Tof => {
                        i2c1.read_dma(
                            tof::ADDR,
                            &mut sensors_shared::TOF_READINGS,
                            setup::EXT_SENSORS_RX_CH,
                            Default::default(),
                            setup::EXT_SENSORS_DMA_PERIPH,
                        );
                    }
                }
            }
        });
    }

    // binds = DMA2_STR4,
    #[task(binds = DMA2_CH4,
    shared = [i2c1, ext_sensor_active], priority = 1)]
    /// Baro write complete; start baro read.
    fn ext_sensors_read_tc_isr(cx: ext_sensors_read_tc_isr::Context) {
        dma::clear_interrupt(
            setup::EXT_SENSORS_DMA_PERIPH,
            setup::EXT_SENSORS_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::EXT_SENSORS_DMA_PERIPH, setup::EXT_SENSORS_RX_CH);

        println!("Ext sensors A");
        (cx.shared.i2c1, cx.shared.ext_sensor_active).lock(|i2c1, ext_sensor_active| {
            // todo: Skip sensors if marked as not connected?

            // todo: Interp data, and place data into its apt struct here.

            unsafe {
                match ext_sensor_active {
                    ExtSensor::Mag => {
                        i2c1.write_dma(
                            gps::ADDR,
                            &mut sensors_shared::WRITE_BUF_GPS,
                            false,
                            setup::EXT_SENSORS_RX_CH,
                            Default::default(),
                            setup::EXT_SENSORS_DMA_PERIPH,
                        );
                        *ext_sensor_active = ExtSensor::Gps;
                    }
                    ExtSensor::Gps => {
                        i2c1.write_dma(
                            tof::ADDR,
                            &mut sensors_shared::WRITE_BUF_TOF,
                            false,
                            setup::EXT_SENSORS_RX_CH,
                            Default::default(),
                            setup::EXT_SENSORS_DMA_PERIPH,
                        );
                        *ext_sensor_active = ExtSensor::Tof;
                    }
                    ExtSensor::Tof => {
                        *ext_sensor_active = ExtSensor::Mag;
                        // End of sequence; don't start a new transfer.
                    }
                }
            }
        });
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
