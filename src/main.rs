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
use control_interface::{ChannelData, LinkStats};
use cortex_m::{self, asm, delay::Delay};

use defmt::println;
use defmt_rtt as _;
use panic_probe as _;

use drivers::{
    baro_dps310 as baro, gps_ublox as gps, imu_icm426xx as imu, mag_lis3mdl as mag,
    osd::{self, AutopilotData, OsdData},
    tof_vl53l1 as tof,
};
use filter_imu::ImuFilters;
use flight_ctrls::{
    autopilot::AutopilotStatus,
    // common::{MotorRpm, RpmReadings},
    // pid::{
    //     self, CtrlCoeffGroup, PidDerivFilters, PidGroup, PID_CONTROL_ADJ_AMT,
    //     PID_CONTROL_ADJ_TIMEOUT,
    // },
    ctrl_logic::{self, PowerMaps},
    filters::FlightCtrlFilters,
    motor_servo::{MotorRpm, RpmReadings},
    pid::{MotorCoeffs, MotorPidGroup},
    // ControlMapping,
};
use lin_alg2::f32::Quaternion;

#[cfg(feature = "quad")]
use flight_ctrls::motor_servo::MotorPower; // todo Temp

use ppks::{Location, LocationType};
use protocols::{crsf, dshot, rpm_reception, usb_preflight};
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
    gpio::{self, Pin},
    i2c::I2c,
    pac::{self, I2C1, I2C2, SPI1, TIM1, TIM16, TIM17, TIM2, TIM5},
    spi::Spi,
    timer::{BasicTimer, MasterModeSelection, Timer, TimerConfig, TimerInterrupt},
    usart::UsartInterrupt,
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
            // todo: USB1 on H723; USB2 on H743.
            // usb::{Usb1, UsbBus, Usb1BusType as UsbBusType},
            usb::{Usb2, UsbBus, Usb2BusType as UsbBusType},
            // pac::OCTOSPI1,
            pac::QUADSPI,
            qspi::{Qspi},
        };
        // This USART alias is made pub here, so we don't repeat this line in other modules.
        pub use stm32_hal2::pac::{UART7, ADC1 as ADC};
    } else if #[cfg(feature = "g4")] {
        use stm32_hal2::{
            usb::{self, UsbBus, UsbBusType},
            pac::SPI3,
        };

        pub use stm32_hal2::pac::{UART4, ADC2 as ADC};
    }
}

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        // use flight_ctrls::{autopilot::Orbit, ControlPositions, };
    } else {
        use flight_ctrls::{motor_servo::{RotationDir}};
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

#[cfg(feature = "quad")]
const FLIGHT_CTRL_IMU_RATIO: usize = 4; // Likely values: 1, 2, 4, 8.

#[cfg(feature = "fixed-wing")]
const FLIGHT_CTRL_IMU_RATIO: usize = 8; // Likely values: 1, 2, 4.

const UPDATE_RATE_IMU: f32 = 8_000.;
const DT_IMU: f32 = 1. / UPDATE_RATE_IMU;
const NUM_IMU_LOOP_TASKS: usize = 6; // We cycle through lower-priority tasks in the main loop.

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
        const UPDATE_RATE_FLIGHT_CTRLS: f32 = UPDATE_RATE_IMU / FLIGHT_CTRL_IMU_RATIO as f32;
    }
}

const UPDATE_RATE_MAIN_LOOP: f32 = 600.; // todo: Experiment with this.

const DT_FLIGHT_CTRLS: f32 = 1. / UPDATE_RATE_FLIGHT_CTRLS;
const DT_MAIN_LOOP: f32 = 1. / UPDATE_RATE_MAIN_LOOP;

// Every x main update loops, log parameters etc to flash.
const LOGGING_UPDATE_RATIO: usize = 100;

// Every x IMU loops, print system status and sensor readings to console,
// if enabled with the `print-status` feature.
const PRINT_STATUS_RATIO: usize = 16_000;

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

//// The time, in ms, to wait during initializing to allow the ESC and RX to power up and initialize.
// const WARMUP_TIME: u32 = 100;

// todo: Bit flags that display as diff colored LEDs, and OSD items

// todo: dispatchers are temp
#[rtic::app(device = pac, peripherals = false, dispatchers= [])]
mod app {
    use super::*;
    use crate::flight_ctrls::pid;
    use stm32_hal2::dma::DmaChannel;

    #[shared]
    struct Shared {
        user_cfg: UserCfg,
        state_volatile: StateVolatile,
        system_status: SystemStatus,
        autopilot_status: AutopilotStatus,
        current_params: Params,
        // None if the data is stale. eg lost link, no link established.
        control_channel_data: Option<ChannelData>,
        /// Link statistics, including Received Signal Strength Indicator (RSSI) from the controller's radio.
        link_stats: LinkStats,
        spi1: Spi<SPI1>,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        altimeter: baro::Altimeter,
        flash_onboard: Flash,
        rf_limiter_timer: Timer<TIM16>,
        lost_link_timer: Timer<TIM17>,
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
        rpm_readings: RpmReadings,
        // rpms_commanded: MotorRpm,
        motor_pid_state: MotorPidGroup,
        /// PID motor coefficients
        motor_pid_coeffs: MotorCoeffs,
    }

    #[local]
    struct Local {
        // update_timer: Timer<TIM15>,
        uart_crsf: setup::UartCrsf, // for ELRS over CRSF.
        // spi_flash: SpiFlash,  // todo: Fix flash in HAL, then do this.
        arm_signals_received: u8, // todo: Put sharedin state volatile.
        disarm_signals_received: u8,
        /// We use this counter to subdivide the main loop into longer intervals,
        /// for various tasks like logging, and outer loops.
        // update_isr_loop_i: usize,
        imu_isr_loop_i: usize,
        // aux_loop_i: usize, // todo temp
        ctrl_coeff_adj_timer: Timer<TIM1>,
        uart_osd: setup::UartOsd, // for our DJI OSD, via MSP protocol
        time_with_high_throttle: f32,
        measurement_timer: Timer<TIM5>,
        ahrs: Ahrs,
        dshot_read_timer: Timer<TIM2>,
        cs_imu: Pin,
        // todo: `params_prev` is an experimental var used in our alternative/experimental
        // todo flight controls code as a derivative.
        params_prev: Params,
        batt_curr_adc: Adc<ADC>,
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

        // Todo: Enable these caches once the program basicallyw orks on H7; don't want
        // todo subtle concurrency bugs from the caches confusing things.
        // #[cfg(feature = "h7")]
        // cp.SCB.enable_icache();
        // #[cfg(feature = "h7")]
        // cp.SCB.enable_dcache(&mut cp.CPUID);

        cfg_if! {
            if #[cfg(feature = "h7")] {
                let clock_cfg = Clocks {
                    // Config for H723 at 520Mhz, or H743 at 480Mhz.
                    // Note: to get H723 working at 550Mhz. You'll need to increase divn to 275,
                    // and set CPU_FREQ_BOOST: "The CPU frequency boost can be enabled through the
                    // CPUFREQ_BOOST option byte in the FLASH_OPTSR2_PRG register."

                    pll_src: PllSrc::Hse(16_000_000),
                    pll1: PllCfg {
                        divm: 8, // To compensate with 16Mhz HSE instead of 64Mhz HSI
                        pllq_en: true, // PLLQ for Spi1 clock. Its default div of 8 is fine.
                        // divn: 275, // For 550Mhz H723 with freq boost enabled.
                        // divn: 275, // For 550Mhz H723 with freq boost enabled.
                        ..Default::default()
                    },
                    hsi48_on: true,
                    // todo: Trouble with full speed on H743
                    // ..Clocks::full_speed()
                    ..Default::default()
                };
            } else {
                let clock_cfg = Clocks {
                    input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
                    hsi48_on: true,
                    clk48_src: clocks::Clk48Src::Hsi48, // todo: Should be in be default
                    ..Default::default()
                };
            }
        }

        println!("Pre clocks setup");
        clock_cfg.setup().unwrap();

        println!("Clocks setup ");

        // Enable the Clock Recovery System, which improves HSI48 accuracy.

        // todo: Put this back once done USB TS.
        #[cfg(feature = "h7")]
        clocks::enable_crs(CrsSyncSrc::OtgHs);
        #[cfg(feature = "g4")]
        clocks::enable_crs(CrsSyncSrc::Usb);

        let flash = unsafe { &(*pac::FLASH::ptr()) };

        // Set up pins with appropriate modes.
        setup::setup_pins();

        let mut dma = Dma::new(dp.DMA1);
        let mut dma2 = Dma::new(dp.DMA2);

        // todo: Note that the HAL currently won't enable DMA2's RCC wihtout using a struct like this.
        let dma2_ch1 = dma::Dma2Ch1::new();

        #[cfg(feature = "g4")]
        dma::enable_mux1();

        setup::setup_dma(&mut dma, &mut dma2);

        cfg_if! {
            if #[cfg(feature = "h7")] {
                let uart_crsf_pac = dp.UART7;
                let uart_osd_pac = dp.USART2;
            } else {
                let uart_crsf_pac = dp.USART2;
                let uart_osd_pac = dp.UART4;
            }
        }
        let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

        // todo: End SPI3/ELRs rad test

        #[cfg(feature = "h7")]
        // let spi_flash_pac = dp.OCTOSPI1;
        let spi_flash_pac = dp.QUADSPI;
        #[cfg(feature = "g4")]
        let spi_flash_pac = dp.SPI2;

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

        // todo end SPI test loop.

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

        // Set up a basic timer that will trigger our ADC reads, at a fixed rate.
        // If you wish to sample at a fixed rate, consider using a basic timer (TIM6 or TIM7)
        let mut adc_timer = BasicTimer::new(dp.TIM6, sensors_shared::ADC_SAMPLE_FREQ, &clock_cfg);

        // The update event is selected as a trigger output (TRGO). For instance a
        // master timer can then be used as a prescaler for a slave timer.
        adc_timer.set_mastermode(MasterModeSelection::Update);

        // let mut update_timer = Timer::new_tim15(
        //     dp.TIM15,
        //     UPDATE_RATE_MAIN_LOOP,
        //     Default::default(),
        //     &clock_cfg,
        // );
        // update_timer.enable_interrupt(TimerInterrupt::Update);

        // This timer is used to prevent a stream of continuous RF control signals, should they arrive
        // at any reason, from crashing the FC. Ie, limit damage that can be done from external sources.
        let mut rf_limiter_timer = Timer::new_tim16(
            dp.TIM16,
            safety::MAX_RF_UPDATE_RATE,
            Default::default(),
            &clock_cfg,
        );

        // todo: Testing ELRS MCU flashing
        // Pull up the CRSF RX line. Without this, our idle interrupt fires spuratically in
        // some conditions, like when touching the (even outside) of the wires if an Rx
        // module isn't connected.
        // let mut uart_crsf_tx = Pin::new(Port::B, 3, gpio::PinMode::Input);
        // let mut uart_crsf_rx = Pin::new(Port::B, 4, gpio::PinMode::Input);
        // // uart_crsf_rx.pull(gpio::Pull::Up);
        // // uart_crsf_tx.pull(gpio::Pull::Up);
        //
        // println!("ELRS test loop");
        // loop {}

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

        // For fixed wing on H7; need a separate timer from the 4 used for DSHOT.
        let mut servo_timer = Timer::new_tim8(dp.TIM8, 1., motor_timer_cfg, &clock_cfg);

        setup::setup_motor_timers(&mut motor_timer, &mut servo_timer);

        // This timer periodically fire. When it does, we read the value of each of the 4 motor lines
        // in its ISR.
        let mut dshot_read_timer = Timer::new_tim2(dp.TIM2, 1., Default::default(), &clock_cfg);

        dshot_read_timer.set_prescaler(dshot::PSC_DSHOT);
        dshot_read_timer.set_auto_reload(dshot::DSHOT_ARR_READ);
        dshot_read_timer.enable_interrupt(TimerInterrupt::Update);

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
            system_status.gps == SensorStatus::Pass,
            system_status.tof == SensorStatus::Pass,
        );

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

        // todo: Once back from Argentina, if you have trouble, check this.
        // Start out IMU-driven loop
        // todo: This is an awk way; Already set up /configured like this in `setup`, albeit with
        // todo opendrain and pullup set, and without enabling interrupt.
        // #[cfg(feature = "h7")]
        // let mut imu_exti_pin = Pin::new(Port::B, 12, gpio::PinMode::Input);
        // #[cfg(feature = "g4")]
        // let mut imu_exti_pin = Pin::new(Port::C, 4, gpio::PinMode::Input);

        // // todo: On G4, sort out if you need to set Rising and falling, for M2 RPM reception.
        // imu_exti_pin.enable_interrupt(Edge::Falling);

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
        }

        println!("Init complete; starting main loops");

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
                rf_limiter_timer,
                lost_link_timer,
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
                rpm_readings: Default::default(),
                // rpms_commanded: Default::default(),
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
                measurement_timer,
                ahrs,
                dshot_read_timer,
                cs_imu,
                params_prev: params,
                batt_curr_adc,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [], local = [])]
    /// In this function, we perform setup code that must occur with interrupts enabled.
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    /// Runs when new IMU data is ready. Trigger a DMA read.
    /// High priority since it's important, and quick-to-execute
    #[task(binds = EXTI4,
    // #[task(binds = EXTI15_10,
    shared = [spi1], local = [], priority = 7)]
    fn imu_data_isr(mut cx: imu_data_isr::Context) {
        #[cfg(feature = "h7")]
        gpio::clear_exti_interrupt(12); // PB12
        #[cfg(feature = "g4")]
        gpio::clear_exti_interrupt(4); // todo: Line 13 next revision.

        cx.shared.spi1.lock(|spi| {
            imu_shared::read_imu(imu::READINGS_START_ADDR, spi, setup::IMU_DMA_PERIPH);
        });
    }

    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it nominally (and according to our measurements so far) runs at 8kHz.
    /// Note that on the H7 FC with the dedicated IMU LSE, it may run slightly faster.
    ///
    /// Certain tasks, like reading IMU measurements and filtering are run each time this function runs.
    /// Flight control logic is run once every several runs. Other tasks are run even less,
    /// sequenced among each other.
    // #[task(binds = DMA1_STR2,
    #[task(binds = DMA1_CH2,
    shared = [spi1, i2c1, i2c2, current_params, control_channel_data,
    autopilot_status, imu_filters, flight_ctrl_filters, user_cfg, motor_pid_state, motor_pid_coeffs,
    motor_timer, servo_timer, state_volatile, system_status, rpm_readings],
    local = [ahrs, imu_isr_loop_i, cs_imu, params_prev, time_with_high_throttle,
    arm_signals_received, disarm_signals_received, batt_curr_adc, measurement_timer], priority = 3)]
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        dma::clear_interrupt(
            setup::IMU_DMA_PERIPH,
            setup::IMU_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        cx.local.cs_imu.set_high();

        cx.shared.spi1.lock(|spi1| {
            // Note that this step is mandatory, per STM32 RM.
            spi1.stop_dma(setup::IMU_TX_CH, Some(setup::IMU_RX_CH), DmaPeriph::Dma1);
        });

        *cx.local.imu_isr_loop_i += 1;
        let i = *cx.local.imu_isr_loop_i; // code shortener.

        (
            cx.shared.current_params,
            cx.shared.control_channel_data,
            cx.shared.autopilot_status,
            cx.shared.motor_timer,
            cx.shared.servo_timer,
            cx.shared.motor_pid_state,
            cx.shared.motor_pid_coeffs,
            cx.shared.user_cfg,
            cx.shared.state_volatile,
            cx.shared.flight_ctrl_filters,
            cx.shared.system_status,
            cx.shared.rpm_readings,
            // cx.shared.rpms_commanded,
        )
            .lock(
                |params,
                 control_channel_data,
                 autopilot_status,
                 motor_timer,
                 servo_timer,
                 pid_state,
                 pid_coeffs,
                 cfg,
                 state_volatile,
                 flight_ctrl_filters,
                 system_status,
                 rpm_readings,
                 // rpms_commanded
                | {
                    let mut imu_data =
                        imu_shared::ImuReadings::from_buffer(unsafe { &imu_shared::IMU_READINGS });

                    cx.shared.imu_filters.lock(|imu_filters| {
                        imu_filters.apply(&mut imu_data);
                    });

                    // Update our flight control logic and motors a fraction of IMU updates, but
                    // apply filter data to all.
                    // todo: Consider a different approach; all you need to do each time is
                    // todo read and filter.

                    // Don't execute any other code if not at our determined radio of the IMU's
                    // 8kHz update rate. The code above this excecutes at the full IMU rate; this is
                    // mainly DMA cleanup, and applying IMU data filters.
                    if i % FLIGHT_CTRL_IMU_RATIO != 0 {
                        return
                    }

                    // Update `params_prev` with past-update data prior to updating params
                    // todo: Update params each IMU update, or at FC interval?
                    *cx.local.params_prev = params.clone();
                    params.update_from_imu_readings(imu_data);

                    // todo: Update attitude each IMU update, or at FC interval?
                    attitude_platform::update_attitude(cx.local.ahrs, params);

                    let mut rpm_fault = false;

                    // todo: Clean up the Optionalble Status vs the non-optioned Rpms.
                    // todo: Consider using only the former.

                    // Update RPMs here, so we don't have to lock the read ISR.
                    // cx.shared.rotor_rpms.lock(|rotor_rpms| {
                    // let (rpm1_status, rpm2_status, rpm3_status, rpm4_status) = rpm_reception::update_rpms(rpms, &mut rpm_fault, cfg.pole_count);
                    *rpm_readings = rpm_reception::rpm_readings_from_bufs(&mut rpm_fault, cfg.motor_pole_count);

                    // match rpm_readings.to_rpms() {
                    //     Ok(r) => {
                    //         *rpms = r;
                    //     }
                    //     // todo: Now what?
                    //     Err(_e) => {
                    //     }
                    // }

                    // });
                    if rpm_fault {
                        system_status::RPM_FAULT.store(true, Ordering::Release);
                    }

                    // todo: Sotr out rpms vs rpm status!!

                    // todo: Temp debug code.

                    match control_channel_data {
                        Some(ch_data) => {
                            #[cfg(feature = "quad")]
                            {
                                let mut p = ch_data.throttle;

                                if p < 0.025 {
                                    p = 0.025;
                                }

                                // todo: Temp mapping of throttle settings to RPM
                                let target_rpm = util::map_linear(p, (0., 1.), (300., 6_000.));

                                let rpms_commanded = MotorRpm {
                                    front_left: target_rpm,
                                    aft_left: target_rpm,
                                    front_right: target_rpm,
                                    aft_right: target_rpm,
                                };
                                //
                                // if i % 8000 == 0 {
                                //     println!("Rpms- FL: {:?} FR: {}, AL: {}, AR: {}", rpm_readings.front_left, rpm_readings.front_right,
                                //              rpm_readings.aft_left, rpm_readings.aft_right);
                                // }

                                #[cfg(feature = "quad")]
                                if state_volatile.arm_status == ArmStatus::Armed {
                                    // dshot::set_power(p, p, p, p, motor_timer);

                                    state_volatile.motor_servo_state.set_cmds_from_rpms(
                                        &rpms_commanded,
                                        rpm_readings,
                                        pid_state,
                                        pid_coeffs,
                                    );

                                    state_volatile.motor_servo_state.send_to_rotors(ArmStatus::Armed, motor_timer);
                                } else {
                                    dshot::stop_all(motor_timer);
                                }
                            }
                        }
                        None => {

                            #[cfg(feature = "quad")]
                            {
                                // todo testing
                                if i < 25_000 {
                                    dshot::stop_all(motor_timer);
                                } else {
                                    let target_rpm = 800.;

                                    let rpms_commanded = MotorRpm {
                                        front_left: target_rpm,
                                        aft_left: target_rpm,
                                        front_right: target_rpm,
                                        aft_right: target_rpm,
                                    };

                                    // state_volatile.motor_servo_state.set_cmds_from_rpms(
                                    //     &rpms_commanded,
                                    //     rpm_readings,
                                    //     pid_state,
                                    //     pid_coeffs,
                                    // );

                                    state_volatile.motor_servo_state.set_cmds_from_power(
                                        &MotorPower {
                                            front_left: 0.025,
                                            front_right: 0.025,
                                            aft_left: 0.025,
                                            aft_right: 0.025,
                                        }
                                    );

                                    state_volatile.motor_servo_state.send_to_rotors(ArmStatus::Armed, motor_timer);
                                }
                                if i % 8000 == 0 {
                                    println!("Rpms- FL: {:?} FR: {}, AL: {}, AR: {}", rpm_readings.front_left, rpm_readings.front_right,
                                             rpm_readings.aft_left, rpm_readings.aft_right);
                                }
                            }
                            // todo testing
                            // dshot::stop_all(motor_timer);
                        }
                    };

                    // return; // todo temp!

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
                        None => {}
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

                    // todo: These staggered tasks. should probably be after the flight control logic

                    // todo: Global const
                    //
                    // todo: Time these tasks so that they're roughly even.

                    // Perform various lower priority tasks like updating altimeter data etc. Space
                    // these out between updates to keep loop time relatively consistent, and
                    // avoid desynchronizing these tasks. This creates slots; one slot runs
                    // during each main update loop. Our flow is thus like this:
                    //
                    //- Each IMU update, cleanup DMA, and apply filters to IMU readings.
                    //- Each Every few updates (determined in IMU update ratio), perform the above
                    // logic, which includes applying flight control data
                    // - After applying flight control data, perform a lower priority task.
                    // - Next update, apply a different lower pri task, etc.
                    //  Compared to performing the tasks asynchronously, this is probably better
                    // determined. Compared to performing them all together, this prevents
                    // the update loop from becoming too long.

                    // We're tracking tasks as ones that make it past the initial flight]
                    // control ratio filter, so factor that out.
                    let i_compensated = i / FLIGHT_CTRL_IMU_RATIO;

                    if (i_compensated - 0) % NUM_IMU_LOOP_TASKS == 0 {
                        let mut batt_v = 0.;
                        let mut curr_v = 0.;
                        batt_v = cx.local.batt_curr_adc.reading_to_voltage(unsafe { V_A_ADC_READ_BUF }[0])
                            * sensors_shared::ADC_BATT_V_DIV;
                        curr_v = cx.local.batt_curr_adc.reading_to_voltage(unsafe { V_A_ADC_READ_BUF }[1])
                            * sensors_shared::ADC_CURR_DIV;

                        // todo: Find the current conversion factor. Probably slope + y int
                        let esc_current = curr_v;

                        state_volatile.batt_v = batt_v;
                        state_volatile.esc_current = esc_current;
                    } else if (i_compensated - 1) % NUM_IMU_LOOP_TASKS == 0 {
                        if let OperationMode::Preflight = state_volatile.op_mode {
                            return;
                        }

                        let (controller_arm_status, throttle) = match control_channel_data {
                            Some(ch_data) => (ch_data.arm_status, ch_data.throttle),
                            None => (ArmStatus::Disarmed, 0.),
                        };

                        safety::handle_arm_status(
                            cx.local.arm_signals_received,
                            cx.local.disarm_signals_received,
                            controller_arm_status,
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
                    } else if (i_compensated - 2) % NUM_IMU_LOOP_TASKS == 0 {
                        let osd_data = OsdData {
                            arm_status: state_volatile.arm_status,
                            battery_voltage: state_volatile.batt_v,
                            current_draw: state_volatile.esc_current,
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
                    } else if (i_compensated - 3) % NUM_IMU_LOOP_TASKS == 0 {
                        if let OperationMode::Preflight = state_volatile.op_mode {
                            return;
                        }
                        // println!("D");
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
                    } else if (i_compensated - 4) % NUM_IMU_LOOP_TASKS == 0 {
                        // if *cx.local.update_isr_loop_i % LOGGING_UPDATE_RATIO == 0 {
                        // todo: Eg log params to flash etc.
                        // }

                        // todo: Determine timing for OSD update, and if it should be in this loop,
                        // todo, or slower.


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


                        // todo: This should probably be delegatd to a fn; get it
                        // todo out here
                        if i % THRUST_LOG_RATIO == 0 {
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
                                if state_volatile.motor_servo_state.frontleft_aftright_dir == RotationDir::Clockwise {
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
                    } else if (i_compensated - 5) % NUM_IMU_LOOP_TASKS == 0 {
                        // println!("F");
                        (cx.shared.i2c1, cx.shared.i2c2).lock(|i2c1, i2c2| {
                            // Start DMA sequences for I2C sensors, ie baro, mag, GPS, TOF.
                            // DMA TC isrs are sequenced.
                            sensors_shared::start_transfers(i2c1, i2c2);
                        });
                    } else {
                        println!("No task");
                    }

                    #[cfg(feature = "print-status")]
                    if i % PRINT_STATUS_RATIO == 0 {
                        util::print_status(params, system_status, control_channel_data, state_volatile, autopilot_status, rpm_readings);
                    }

                    return; // todo TS

                    if let OperationMode::Preflight = state_volatile.op_mode {
                        return;
                    }

                    cfg_if! {
                        if #[cfg(feature = "quad")] {
                            let (ctrl_mix, rpms_commanded) = ctrl_logic::rotor_rpms_from_att(
                                state_volatile.attitude_commanded.quat.unwrap(),
                                params.attitude_quat,
                                throttle,
                                state_volatile.motor_servo_state.frontleft_aftright_dir,
                                params,
                                cx.local.params_prev,
                                &cfg.ctrl_coeffs,
                                &state_volatile.drag_coeffs,
                                &state_volatile.accel_map,
                                flight_ctrl_filters,
                                // The DT passed is the IMU rate, since we update params_prev each IMU update.
                                DT_IMU,
                            );

                            state_volatile.motor_servo_state.set_cmds_from_rpms(
                                &rpms_commanded,
                                rpm_readings,
                                pid_state,
                                pid_coeffs,
                            );

                            state_volatile.motor_servo_state.send_to_rotors(ArmStatus::Armed, motor_timer);

                            state_volatile.ctrl_mix = ctrl_mix;

                            // todo: Dynamics of `current_pwr` for quads, and `ctrl_posits`
                            // todo for fixed-wing. You're saving it, but when is it used?
                            // state_volatile.current_pwr = motor_power;
                        } else {
                            let (ctrl_mix, control_posits_commanded) = ctrl_logic::control_posits_from_att(
                                state_volatile.attitude_commanded.quat.unwrap(),
                                params.attitude_quat,
                                throttle,
                                params,
                                cx.local.params_prev,
                                // &state_volatile.ctrl_mix,
                                &cfg.ctrl_coeffs,
                                &state_volatile.drag_coeffs,
                                &state_volatile.accel_map,
                                flight_ctrl_filters,
                                DT_IMU,
                            );

                            state_volatile.motor_servo_state.send_to_motors(ArmStatus::MotorsControlsArmed, motor_timer);
                            state_volatile.motor_servo_state.send_to_servos(ArmStatus::MotorsControlsArmed, servo_timer);

                            state_volatile.ctrl_mix = ctrl_mix;
                        }
                    }
                });
    }

    // todo H735 issue on GH: https://github.com/stm32-rs/stm32-rs/issues/743 (works on H743)
    // todo: NVIC interrupts missing here for H723 etc!
    // #[task(binds = OTG_HS,
    // #[task(binds = OTG_FS,
    #[task(binds = USB_LP,
    shared = [usb_dev, usb_serial, current_params, control_channel_data,
    link_stats, user_cfg, state_volatile, system_status, motor_timer, servo_timer, rpm_readings],
    local = [], priority = 1)]
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
            cx.shared.rpm_readings,
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
                 rpm_readings| {
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
                                state_volatile.pressure_static,
                                state_volatile.temp_baro,
                                params.tof_alt,
                                state_volatile.batt_v,
                                state_volatile.esc_current,
                                ch_data,
                                &link_stats,
                                &user_cfg.waypoints,
                                system_status,
                                &mut state_volatile.arm_status,
                                // &mut user_cfg.control_mapping,
                                &mut state_volatile.op_mode,
                                motor_timer,
                                servo_timer,
                                rpm_readings,
                            );
                        }
                        Err(_) => {
                            // println!("Error reading USB signal from PC");
                        }
                    }
                },
            )
    }

    // #[task(binds = DMA1_STR3,
    #[task(binds = DMA1_CH3,
    shared = [motor_timer], priority = 6)]
    /// We use this ISR to initialize the RPM reception procedures upon completion of the dshot
    /// power setting transmission to the ESC.
    fn dshot_isr(mut _cx: dshot_isr::Context) {
        dma::clear_interrupt(
            setup::MOTORS_DMA_PERIPH,
            setup::MOTOR_CH,
            DmaInterrupt::TransferComplete,
        );

        // (From testing) We must stop this transaction manually before future transactions will work.
        dma::stop(setup::MOTORS_DMA_PERIPH, setup::MOTOR_CH);

        _cx.shared.motor_timer.lock(|motor_timer| {
            motor_timer.disable();

            if dshot::BIDIR_EN {
                dshot::M1_RPM_I.store(0, Ordering::Release);
                dshot::M2_RPM_I.store(0, Ordering::Release);
                dshot::M3_RPM_I.store(0, Ordering::Release);
                dshot::M4_RPM_I.store(0, Ordering::Release);

                // Make sure to clear these buffers at reception start, not after completion; if we do it after,
                // they will be blanked before we can process them.
                unsafe {
                    dshot::PAYLOAD_REC_1 = [0; dshot::REC_BUF_LEN];
                    dshot::PAYLOAD_REC_2 = [0; dshot::REC_BUF_LEN];
                    dshot::PAYLOAD_REC_3 = [0; dshot::REC_BUF_LEN];
                    dshot::PAYLOAD_REC_4 = [0; dshot::REC_BUF_LEN];
                }

                dshot::receive_payload();
            }
        });
    }

    #[task(binds = EXTI9_5, priority = 10)]
    /// We use this to read RPM status on motor 1 on G4, or any motor on H7.
    /// Triggers on rising and falling edges.
    /// High priority since this is time-sensitive, and fast-to-execute.
    fn rpm_read_m1(_cx: rpm_read_m1::Context) {
        let exti = unsafe { &(*pac::EXTI::ptr()) };

        // Determine which line fired. I'm not sure if it's going to be generally (always?)
        // a single line, or multiple ones.
        cfg_if! {
            if #[cfg(feature = "h7")] {
                let pr = exti.cpupr1.read();
                // Don't use if/else, in case multiple fire simultaneously, which seems likely.
                if pr.pr6().bit_is_set() {
                    gpio::clear_exti_interrupt(6);
                    // dshot::update_rec_buf(&dshot::M1_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_1 });
                    dshot::update_rec_buf_1(&dshot::M1_RPM_I);
                }

                if pr.pr7().bit_is_set() {
                    gpio::clear_exti_interrupt(7);
                    // dshot::update_rec_buf(&dshot::M2_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_2 });
                    dshot::update_rec_buf_2(&dshot::M2_RPM_I);
                }

                if pr.pr8().bit_is_set() {
                    gpio::clear_exti_interrupt(8);
                    // dshot::update_rec_buf(&dshot::M3_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_3 });
                    dshot::update_rec_buf_3(&dshot::M3_RPM_I);
                }

                if pr.pr9().bit_is_set() {
                    gpio::clear_exti_interrupt(9);
                    // dshot::update_rec_buf(&dshot::M4_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_4 });
                    dshot::update_rec_buf_4(&dshot::M4_RPM_I);
                }
            } else {
                // On G4, this is only for Motor 1.
                gpio::clear_exti_interrupt(6);
                // dshot::update_rec_buf(&dshot::M1_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_1 });
                dshot::update_rec_buf_1(&dshot::M1_RPM_I);
            }
        }
    }

    // todo: Can we feature-gate the rpm_read_m3[4] ISRs for G4 only?
    // todo: Probably not with RTIC, but they shouldn't fire on H7, so
    // todo: not required.

    // todo: Use this once you have the new (6-layer) revision.
    // #[task(binds = EXTI4, priority = 10)]
    // /// Similar to `rpm_read_m1`, but for M2, on G4 only.
    // fn rpm_read_m2(_cx: rpm_read_m4::Context) {
    //     gpio::clear_exti_interrupt(4);
    //
    //     // dshot::update_rec_buf(&dshot::M2_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_2 });
    //     dshot::update_rec_buf_2(&dshot::M2_RPM_I);
    // }

    #[task(binds = EXTI0, priority = 10)]
    /// Similar to `rpm_read_m1`, but for M3, on G4 only.
    fn rpm_read_m3(_cx: rpm_read_m3::Context) {
        gpio::clear_exti_interrupt(0);
        // dshot::update_rec_buf(&dshot::M3_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_3 });
        dshot::update_rec_buf_3(&dshot::M3_RPM_I);
    }

    #[task(binds = EXTI1, priority = 10)]
    /// Similar to `rpm_read_m1`, but for M4, on G4 only.
    fn rpm_read_m4(_cx: rpm_read_m4::Context) {
        gpio::clear_exti_interrupt(1);

        // dshot::update_rec_buf(&dshot::M4_RPM_I, &mut unsafe { dshot::PAYLOAD_REC_4 });
        dshot::update_rec_buf_4(&dshot::M4_RPM_I);
    }

    #[task(binds = TIM2, shared = [], local = [dshot_read_timer], priority = 9)]
    /// This interrupt fires slightly after the last bit of RPM data is received.
    /// Its timer is started once power setting is transmitted.
    /// In this ISR, we disable reception, and return the DSHOT lines to an output
    /// state.
    fn dshot_read_isr(mut cx: dshot_read_isr::Context) {
        let timer = &mut cx.local.dshot_read_timer; // code shortener.
        timer.clear_interrupt(TimerInterrupt::Update);
        timer.disable();

        // Disable interrupts on the motor pins.
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
                    // EXTI 4 is shared with the IMU; leave enabled.
                    w.im0().clear_bit();
                    w.im1().clear_bit()
                });
            }
        }

        // Set motor pins back to their timer alt fn.
        let alt_mode = 0b10;
        unsafe {
            cfg_if! {
                if #[cfg(feature = "h7")] {
                    (*pac::GPIOC::ptr())
                        .moder
                        .modify(|_, w| {
                            w.moder6().bits(alt_mode);
                            w.moder7().bits(alt_mode);
                            w.moder8().bits(alt_mode);
                            w.moder9().bits(alt_mode)
                        });

                } else {
                    (*pac::GPIOC::ptr())
                        .moder
                        .modify(|_, w| w.moder6().bits(alt_mode));
                    (*pac::GPIOA::ptr())
                        .moder
                        .modify(|_, w| w.moder4().bits(alt_mode));
                    (*pac::GPIOB::ptr())
                        .moder
                        .modify(|_, w| {
                        w.moder0().bits(alt_mode);
                        w.moder1().bits(alt_mode)
                    });
                }
            }
        }
        // todo: Interpret data here? Or maybe in main loop.
    }

    // todo: Evaluate priority.
    // #[task(binds = UART7,
    #[task(binds = USART2,
    shared = [control_channel_data, link_stats, rf_limiter_timer,
    lost_link_timer], local = [uart_crsf], priority = 4)]
    /// This ISR handles CRSF reception. It handles, in an alternating fashion, message starts,
    /// and message ends. For message starts, it begins a DMA transfer. For message ends, it
    /// processes the radio data, passing it into shared resources for control channel data,
    /// and link stats.
    ///
    /// Ideally, the only locks we have here are things used in lower-priority ISRs,
    /// like link timer etc.
    ///
    /// Must be a higher priority than the IMU TC isr.
    fn crsf_isr(mut cx: crsf_isr::Context) {
        let uart = &mut cx.local.uart_crsf; // Code shortener

        println!("CRSF");

        let mut recieved_ch_data = false; // Lets us split up the lock a bit more.
        let mut rx_fault = false;

        uart.clear_interrupt(UsartInterrupt::CharDetect(None));
        uart.clear_interrupt(UsartInterrupt::Idle);

        // todo: Store link stats and control channel data in an intermediate variable.
        // todo: Don't lock it. At least, you don't want any delay when starting the read,
        // todo although a delay on finishing the read is fine.

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

            if let Some(crsf_data) = crsf::handle_packet(uart, setup::CRSF_RX_CH, &mut rx_fault) {
                match crsf_data {
                    crsf::PacketData::ChannelData(data) => {
                        cx.shared.control_channel_data.lock(|ch_data| {
                            *ch_data = Some(data);
                        });

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
                    crsf::PacketData::LinkStats(stats) => cx.shared.link_stats.lock(|link_stats| {
                        *link_stats = stats;
                    }),
                }
            }
        }

        if rx_fault {
            system_status::RX_FAULT.store(true, Ordering::Release);
        }

        cx.shared.lost_link_timer.lock(|lost_link_timer| {
            if recieved_ch_data {
                // We've received a packet successfully - reset the lost-link timer.
                lost_link_timer.disable();
                lost_link_timer.reset_count();
                lost_link_timer.enable();

                if safety::LINK_LOST
                    .compare_exchange(true, false, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
                {
                    println!("Link re-acquired");
                    // todo: Execute re-acq procedure
                }
            }
        });
    }

    /// If this triggers, it means we've received no radio control signals for a significant
    ///period of time; we treat this as a lost-link situation.
    /// (Note that this is for TIM17 on both variants)
    // #[task(binds = TIM17,
    #[task(binds = TIM1_TRG_COM,
    shared = [lost_link_timer, state_volatile, autopilot_status,
    current_params, system_status, control_channel_data], priority = 1)]
    fn lost_link_isr(mut cx: lost_link_isr::Context) {
        println!("Lost the link!");

        cx.shared.lost_link_timer.lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
            timer.reset_count();
        });

        safety::LINK_LOST.store(true, Ordering::Release);

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

    // #[task(binds = TIM16,
    #[task(binds = TIM1_UP_TIM16,
    shared = [rf_limiter_timer], priority = 1)]
    fn rf_limiter_isr(mut cx: rf_limiter_isr::Context) {
        // println!("RF limiter ISR");
        cx.shared.rf_limiter_timer.lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
            timer.reset_count();
        });
    }

    // #[task(binds = DMA2_STR1,
    #[task(binds = DMA2_CH1,
    shared = [i2c2], priority = 2)]
    /// Baro write complete; start baro read.
    fn baro_write_tc_isr(mut cx: baro_write_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_TX_CH);

        cx.shared.i2c2.lock(|i2c| unsafe {
            i2c.read_dma(
                baro::ADDR,
                &mut sensors_shared::BARO_READINGS,
                setup::BARO_RX_CH,
                Default::default(),
                setup::BARO_DMA_PERIPH,
            );
        });
    }

    // todo: For now, we start new transfers in the main loop.

    // #[task(binds = DMA2_STR2,
    #[task(binds = DMA2_CH2,
    shared = [altimeter, current_params, state_volatile], priority = 2)]
    /// Baro read complete; handle data, and start next write.
    fn baro_read_tc_isr(cx: baro_read_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_RX_CH,
            DmaInterrupt::TransferComplete,
        );
        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_RX_CH);

        let buf = unsafe { &sensors_shared::BARO_READINGS };

        (
            cx.shared.altimeter,
            cx.shared.current_params,
            cx.shared.state_volatile,
        )
            .lock(|altimeter, params, state_volatile| {
                // code shortener.

                // todo: Process your baro reading here.
                let (pressure, temp) = altimeter.pressure_temp_from_readings(buf);

                state_volatile.pressure_static = pressure;
                state_volatile.temp_baro = temp;
                params.baro_alt_msl =
                    atmos_model::estimate_altitude_msl(pressure, temp, &altimeter.ground_cal)
            });
    }

    // #[task(binds = DMA2_STR3,
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

    // #[task(binds = DMA2_STR4,
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
