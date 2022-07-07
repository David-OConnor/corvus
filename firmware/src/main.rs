#![no_main]
#![no_std]
#![allow(mixed_script_confusables)] // eg variable names that include greek letters.

use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::{self, asm, delay::Delay};

use defmt::println;
use defmt_rtt as _;
// global logger
use panic_probe as _;

#[cfg(feature = "h7")]
use stm32_hal2::clocks::PllCfg;
use stm32_hal2::{
    self,
    adc::{self, Adc, AdcConfig, AdcDevice},
    clocks::{self, Clocks, CrsSyncSrc, InputSrc, PllSrc},
    dma::{self, ChannelCfg, Dma},
    flash::{Bank, Flash},
    gpio::{self, Pin, PinMode, Port},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, ADC2, DMA1, I2C1, I2C2, SPI1, SPI2, SPI3, TIM15, TIM16, TIM17, USART3},
    rtc::Rtc,
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::{Timer, TimerConfig, TimerInterrupt},
    usart::{Usart, UsartInterrupt},
};
use usb_device::{bus::UsbBusAllocator, prelude::*};

use usbd_serial::{self, SerialPort};

use control_interface::{ChannelData, LinkStats};
use drivers::baro_dps310 as baro;
use drivers::gps_x as gps;
// pub use, so we can use this rename in `sensor_fusion` to interpret the DMA buf.
pub use drivers::imu_icm426xx as imu;
use ahrs_fusion::Ahrs;
use drivers::tof_vl53l1 as tof;
use filter_imu::ImuFilters;
use flight_ctrls::{
    common::{AutopilotStatus, CommandState, CtrlInputs, InputMap, MotorTimers, Params},
    flying_wing::{self, ControlPositions, ServoWingMapping},
    quad::{AxisLocks, InputMode, MotorPower, RotationDir, RotorMapping, RotorPosition},
};

use pid::{CtrlCoeffGroup, PidDerivFilters, PidGroup};
use ppks::{Location, LocationType};
use protocols::{crsf, dshot, usb_cfg};
use safety::ArmStatus;
use state::{AircraftType, OperationMode, StateVolatile, UserCfg};

#[cfg(feature = "h7")]
use crate::clocks::VosRange;

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "h7")] {
        use stm32_hal2::{
            usb_otg::{Usb1, UsbBus, Usb1BusType as UsbBusType},
            // pac::USB
            // OTG1_HS_DEVICE
        };
    } else if #[cfg(feature = "g4")] {
        use stm32_hal2::usb::{self, UsbBus, UsbBusType};
    }
}

// Due to the way the USB serial lib is set up, the USB bus must have a static lifetime.
// In practice, we only mutate it at initialization.
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

mod ahrs_fusion;
mod atmos_model;
mod control_interface;
mod drivers;
mod filter_imu;
mod flight_ctrls;
mod imu_shared;
mod lin_alg;
// mod osd;
mod cfg_storage;
mod imu_calibration;
mod pid;
// mod pid_tuning;
mod attitude_platform;
mod ppks;
mod protocols;
mod safety;
mod setup;
mod state;
mod util;
// todo: Can't get startup code working separately since Shared and Local must be private per an RTIC restriction.
// todo: See this GH issue: https://github.com/rtic-rs/cortex-m-rtic/issues/505
// mod startup;

// todo: Cycle flash pages for even wear. Can postpone this.

// H723: 1Mb of flash, in one bank.
// 8 sectors of 128kb each.
// (H743 is similar, but may have 2 banks, each with those properties)
#[cfg(feature = "h7")]
const FLASH_CFG_SECTOR: usize = 6;
const FLASH_WAYPOINT_SECTOR: usize = 7;

// G47x/G48x: 512k flash.
// Assumes configured as a single bank: 128 pages of 4kb each.
// (If using G4 dual bank mode: 128 pages of pages of 2kb each, per bank)
#[cfg(feature = "g4")]
const FLASH_CFG_PAGE: usize = 254; // todo: Set to 126 etc when on G474!
const FLASH_WAYPOINT_PAGE: usize = 255; // todo: Set to 126 etc when on G474!

// The rate our main program updates, in Hz.
// todo: Currently have it set to what we're measuring. Not sure why we don't get 8khz.
// todo note that we will have to scale up values slightly on teh H7 board with 32.768kHz oscillator:
// ICM-42688 DS: The ODR values shown in the
// datasheet are supported with external clock input frequency of 32kHz. For any other external clock input frequency, these ODR
// values will scale by a factor of (External clock value in kHz / 32). For example, if an external clock frequency of 32.768kHz is used,
// instead of ODR value of 500Hz, it will be 500 * (32.768 / 32) = 512Hz.
#[cfg(feature = "mercury-h7")]
const IMU_UPDATE_RATE: f32 = 6_922.;
#[cfg(feature = "mercury-g4")]
const IMU_UPDATE_RATE: f32 = 6_760.;

// todo: Set update rate attitude back to 1600 etc. Slower rate now since we're using this loop to TS.
const UPDATE_RATE_ATTITUDE: f32 = 1_600.; // IMU rate / 5.
const UPDATE_RATE_VELOCITY: f32 = 400.; // IMU rate / 20.


// Every _ main update loops, log parameters etc to flash.
const LOGGING_UPDATE_RATIO: usize = 100;

const DT_IMU: f32 = 1. / IMU_UPDATE_RATE;
const DT_ATTITUDE: f32 = 1. / UPDATE_RATE_ATTITUDE;
const DT_VELOCITY: f32 = 1. / UPDATE_RATE_VELOCITY;

static mut ADC_READ_BUF: [u16; 2] = [0; 2];

#[cfg(feature = "h7")]
static mut USB_EP_MEMORY: [u32; 1024] = [0; 1024];

// These values correspond to how much the voltage divider on these ADC pins reduces the input
// voltage. Multiply by these values to get the true readings.
pub const ADC_BATT_DIVISION: f32 = 11.;
pub const ADC_CURR_DIVISION: f32 = 11.;

/// Block RX reception of packets coming in at a faster rate then this. This prevents external
/// sources from interfering with other parts of the application by taking too much time.
const MAX_RF_UPDATE_RATE: f32 = 800.; // Hz

// Max distance from curent location, to point, then base a
// direct-to point can be, in meters. A sanity check
// todo: Take into account flight time left.
const DIRECT_AUTOPILOT_MAX_RNG: f32 = 500.;

// We use `LOOP_I` to manage sequencing the velocity-update PID from within the attitude-update PID
// loop.
const VELOCITY_ATTITUDE_UPDATE_RATIO: usize = 4;
const FIXED_WING_RATE_UPDATE_RATIO: usize = 16; // 8k IMU update rate / 16 / 500Hz; apt for servos.

// todo: Course set mode. Eg, point at thing using controls, activate a button,
// todo then the thing flies directly at the target.

// With this in mind: Store params in a global array. Maybe [ParamsInst; N], or maybe a numerical array for each param.

// todo: Consider a nested loop, where inner manages fine-tuning of angle, and outer
// manages directions etc. (?) look up prior art re quads and inner/outer loops.

// todo: Panic button that recovers the aircraft if you get spacial D etc.

// todo: For fixed wing: make inner loop 500Hz. Figure out the best way to do it

// todo: Bit flags that display as diff colored LEDs, and OSD items

/// Data dump from Hypershield on Matrix:
/// You typically don't change the PID gains during flight. They are often tuned experimentally by
///  first tuning the attitude gains, then altitude pid and then the horizontal pids. If you want your
///  pids to cover a large flight envelope (agile flight, hover) then you can use different flight modes
///  that switch between the different gains or use gain scheduling. I don't see a reason to use pitot
/// tubes for a quadrotor. Pitot tubes are used to get the airspeed for fixed-wing cause it has a large
/// influence on the aerodynamics. For a quadrotor it's less important and if you want your velocity
/// it's more common to use a down ward facing camera that uses optical flow. If you want LIDAR
/// (velodyne puck for instance) then we are talking about a very large quadrotor, similar to
///
/// the ones used for the darpa subterranean challenge. Focus rather on something smaller first.
///  The STM32F4 series is common for small quadrotors but if you want to do any sort of SLAM or
/// VIO processing you'll need a companion computer since that type of processing is very demanding.
///  You don't need a state estimator if you are manually flying it and the stick is provided desired
/// angular velocities (similar to what emuflight does). For autonomous flight you need a state estimator
///  where the Extended Kalman Filter is the most commonly used one. A state estimator does not
/// estimate flight parameters, but it estimates the state of the quadrotor (typically position,
/// velocity, orientation). Flight parameters would need to be obtained experimentally for
/// instance through system identification methods (an EKF can actually be used for this purpose
/// by pretending the unknown parameters are states). When it comes to the I term for a PID you
/// would typically create a PID struct or class where the I term is a member, then whenever
///  you compute the output of the PID you also update this variable. See here for instance:
// https://www.youtube.com/watch?v=zOByx3Izf5U
// For state estimation
// https://www.youtube.com/watch?v=RZd6XDx5VXo (Series)
// https://www.youtube.com/watch?v=whSw42XddsU
// https://www.youtube.com/playlist?list=PLn8PRpmsu08ryYoBpEKzoMOveSTyS-h4a
// For quadrotor PID control
// https://www.youtube.com/playlist?list=PLn8PRpmsu08oOLBVYYIwwN_nvuyUqEjrj
// https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y
// https://www.youtube.com/playlist?list=PLn8PRpmsu08pFBqgd_6Bi7msgkWFKL33b
///
///
///
/// todo: Movable camera that moves with head motion.
/// - Ir cam to find or avoid people

// pub enum FlightProfile {
//     /// On ground, with rotors at 0. to 1., with 0. being off, and 1. being 1:1 lift-to-weight
//     OnGround,
//     /// Maintain a hover
//     Hover,
//     /// In transit to a given location, in Location and speed
//     Transit(Location, f32),
//     /// Landing
//     Landing,
// }

/// Run on startup, or when desired. Run on the ground. Gets an initial GPS fix,
/// and other initialization functions.
fn init_sensors(
    params: &mut Params,
    altimeter: &mut baro::Altimeter,
    base_pt: &mut Location,
    i2c1: &mut I2c<I2C1>,
    i2c2: &mut I2c<I2C2>,
) {
    let eps = 0.001;

    // Don't init if in motion.
    if params.v_x > eps
        || params.v_y > eps
        || params.v_z > eps
        || params.v_pitch > eps
        || params.v_roll > eps
    {
        return;
    }

    if let Some(agl) = tof::read(params.s_pitch, params.s_roll, i2c1) {
        if agl > 0.01 {
            return;
        }
    }

    let fix = gps::get_fix(i2c1);

    match fix {
        Ok(f) => {
            params.s_x = f.x;
            params.s_y = f.y;
            params.s_z_msl = f.z;

            *base_pt = Location::new(LocationType::LatLon, f.y, f.x, f.z);

            altimeter.calibrate_from_gps(Some(f.z), i2c2);
        }
        Err(_) => {
            altimeter.calibrate_from_gps(None, i2c2);
        }
    }
    // todo: Use Rel0 location type if unable to get fix.
}

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        // profile: FlightProfile,
        user_cfg: UserCfg,
        state_volatile: StateVolatile,
        input_map: InputMap,
        input_mode: InputMode,
        autopilot_status: AutopilotStatus,
        ctrl_coeffs: CtrlCoeffGroup,
        current_params: Params,
        velocities_commanded: CtrlInputs,
        attitudes_commanded: CtrlInputs,
        rates_commanded: CtrlInputs,
        // control_mix: ControlMix,
        pid_velocity: PidGroup,
        pid_attitude: PidGroup,
        pid_rate: PidGroup,
        control_channel_data: ChannelData,
        // manual_inputs: CtrlInputs,
        dma: Dma<DMA1>,
        spi1: Spi<SPI1>,
        spi2: Spi<SPI2>,
        cs_imu: Pin,
        cs_elrs: Pin,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        altimeter: baro::Altimeter,
        uart3: Usart<USART3>, // for ELRS over CRSF.
        flash_onboard: Flash,
        batt_curr_adc: Adc<ADC2>,
        // rtc: Rtc,
        update_timer: Timer<TIM15>,
        rf_limiter_timer: Timer<TIM16>,
        lost_link_timer: Timer<TIM17>,
        motor_timers: MotorTimers,
        // elrs_timer: Timer<TIM4>,
        // delay_timer: Timer<TIM5>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_serial: SerialPort<'static, UsbBusType>,
        // `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        // Store filter instances for the PID loop derivatives. One for each param used.
        pid_deriv_filters: PidDerivFilters,
        imu_filters: ImuFilters,
        base_point: Location,
        command_state: CommandState,
        ahrs: Ahrs,
        imu_calibration: imu_calibration::ImuCalibration,
    }

    #[local]
    struct Local {
        spi3: Spi<SPI3>,
        arm_signals_received: u8, // todo: Put in state volatile.
        disarm_signals_received: u8,
        /// We use this counter to subdivide the main loop into longer intervals,
        /// for various tasks like logging, and outer loops.
        update_loop_i: usize,
        update_loop_i2: usize, // todo d
        fixed_wing_rate_loop_i: usize,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // startup::init(&cx.core)

        // Cortex-M peripherals
        let mut cp = cx.core;
        // Set up microcontroller peripherals
        let mut dp = pac::Peripherals::take().unwrap();

        // todo: H743 clocks if you end up using that to test with BF
        cfg_if! {
            if #[cfg(feature = "h7")] {
                // todo: Make sure eyou set up vos0 etc as required for 550Mhz H7.
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
                        ..Clocks::full_speed(),
                    },
                    hsi48_on: true,
                    usb_src: clocks::UsbSrc::Hsi48,
                    ..Default::default()
                };
            } else {
                let clock_cfg = Clocks {
                    input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
                    hsi48_on: true,
                    clk48_src: clocks::Clk48Src::Hsi48,
                    boost_mode: true, // Required for speeds > 150Mhz.
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
        // SPI input clock is 400MHz for H7, and 170Mhz for G4. 400MHz / 32 = 12.5 MHz. 170Mhz / 8 = 21.25Mhz.
        // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
        // 426xx can use any SPI mode. Maybe St is only mode 3? Not sure.
        #[cfg(feature = "g4")]
        // todo: Switch to higher speed.
        let imu_baud_div = BaudRate::Div8;  // for ICM426xx, for 24Mhz limit
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

        let mut spi1 = Spi::new(dp.SPI1, imu_spi_cfg, imu_baud_div);

        #[cfg(feature = "mercury-h7")]
        let mut cs_imu = Pin::new(Port::C, 4, PinMode::Output);
        #[cfg(feature = "mercury-g4")]
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
                setup::BATT_CURR_CH,
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

        // This timer is used to prevent a stream of continuous RF control signals, should they arrive
        // at any reason, from crashing the FC. Ie, limit damage that can be done from external sources.
        let rf_limiter_timer = Timer::new_tim16(
            dp.TIM16,
            // MAX_RF_UPDATE_RATE,
            1_000., // todo temp while we coop this timer
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
            if #[cfg(feature = "h7")] {
                let r1234 = Timer::new_tim3(dp.TIM3, 1., rotor_timer_cfg, &clock_cfg);

                // For fixed wing on H7; need a separate timer from the 4 used for DSHOT.
                // todo: PAC ommission??
                let mut servos = Timer::new_tim8(dp.TIM8, 1., Default::default(), &clock_cfg);

                let mut motor_timers = MotorTimers { r1234, servos };

            } else if #[cfg(feature = "mercury-g4")] {

                let mut r12 =
                    Timer::new_tim2(dp.TIM2, 1., rotor_timer_cfg.clone(), &clock_cfg);

                let mut r34_servos = Timer::new_tim3(dp.TIM3, 1., rotor_timer_cfg, &clock_cfg);

                let mut motor_timers = MotorTimers { r12, r34_servos };
            }
        }

        let mut lost_link_timer = Timer::new_tim17(
            dp.TIM17,
            1. / flight_ctrls::common::LOST_LINK_TIMEOUT,
            TimerConfig {
                one_pulse_mode: true,
                ..Default::default()
            },
            &clock_cfg,
        );
        lost_link_timer.enable_interrupt(TimerInterrupt::Update);

        // let elrs_timer = Timer::new_tim4(
        //     dp.TIM4,
        //     1.,
        //     TimerConfig {
        //         auto_reload_preload: true,
        //         ..Default::default()
        //     },
        //     &clock_cfg,
        // );

        let mut user_cfg = UserCfg::default();
        // todo temp
        user_cfg.waypoints[0] = Some(Location {
            type_: LocationType::LatLon,
            name: [0, 1, 2, 3, 4, 5, 6],
            x: 1.,
            y: 2.,
            z: 3.,
        });

        // user_cfg.aircraft_type = AircraftType::FlyingWing; // todo temp

        let mut ctrl_coeffs = Default::default();
        match user_cfg.aircraft_type {
            AircraftType::Quadcopter => {
                dshot::setup_timers(&mut motor_timers);
            }
            AircraftType::FlyingWing => {
                flying_wing::setup_timers(&mut motor_timers);
                ctrl_coeffs = CtrlCoeffGroup::default_flying_wing();
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
        usb_cfg::init_crc();

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
                // control_mix: Default::default(),
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
                motor_timers,
                // elrs_timer,
                // delay_timer,
                usb_dev,
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
                update_loop_i2: 0, // todo
                fixed_wing_rate_loop_i: 0,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [user_cfg, motor_timers, dma])]
    fn idle(cx: idle::Context) -> ! {
        // Set up DSHOT here, since we need interrupts enabled.

        (cx.shared.user_cfg, cx.shared.motor_timers, cx.shared.dma).lock(
            |user_cfg, motor_timers, dma| {
                // Indicate to the ESC we've started with 0 throttle. Not sure if delay is strictly required.

                let cp = unsafe { cortex_m::Peripherals::steal() };
                let mut delay = Delay::new(cp.SYST, 170_000_000);

                // Allow ESC to warm up and radio to connect before starting the main loop.
                delay.delay_ms(2_000);

                dshot::stop_all(motor_timers, dma);

                delay.delay_ms(1);

                // todo still not working
                // dshot::setup_motor_dir(
                //     user_cfg.motors_reversed,
                //     motor_timers,
                //     dma,
                // );
            },
        );

        loop {
            asm::nop();
        }
    }

    // todo: Go through these tasks, and make sure they're not reserving unneded shared params.

    // binds = TIM15,
    // todo: Remove rotor timers, spi, and dma from this ISR; we only use it for tresting DSHOT
    #[task(
    binds = TIM1_BRK_TIM15,
    shared = [current_params, input_map,
    velocities_commanded, attitudes_commanded, rates_commanded, pid_velocity, pid_attitude, pid_rate,
    pid_deriv_filters, power_used, input_mode, autopilot_status, user_cfg, command_state, ctrl_coeffs,
    ahrs, control_channel_data,
    lost_link_timer, altimeter, i2c2, state_volatile, batt_curr_adc,
    ],
    local = [arm_signals_received, disarm_signals_received, update_loop_i],

    priority = 2
    )]
    /// This runs periodically, on a ~1kHz timer. It's used to trigger the attitude and velocity PID loops, ie for
    /// sending commands to the attitude and rate PID loop based on things like autopilot, command-mode etc.
    fn update_isr(mut cx: update_isr::Context) {
        unsafe { (*pac::TIM15::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }

        (
            cx.shared.current_params,
            cx.shared.ahrs,
            cx.shared.control_channel_data,
            cx.shared.input_map,
            cx.shared.velocities_commanded,
            cx.shared.attitudes_commanded,
            cx.shared.rates_commanded,
            cx.shared.pid_rate,
            cx.shared.pid_attitude,
            cx.shared.pid_velocity,
            cx.shared.pid_deriv_filters,
            cx.shared.power_used,
            cx.shared.input_mode,
            cx.shared.autopilot_status,
            cx.shared.user_cfg,
            cx.shared.command_state,
            cx.shared.ctrl_coeffs,
            cx.shared.lost_link_timer,
            cx.shared.altimeter,
            cx.shared.i2c2,
            cx.shared.state_volatile,
            cx.shared.batt_curr_adc,
        )
            .lock(
                |params,
                 ahrs,
                 control_channel_data,
                 input_map,
                 velocities_commanded,
                 attitudes_commanded,
                 rates_commanded,
                 pid_rate,
                 pid_attitude,
                 pid_velocity,
                 filters,
                 power_used,
                 input_mode,
                 autopilot_status,
                 cfg,
                 command_state,
                 coeffs,
                 lost_link_timer,
                 altimeter,
                 i2c2,
                 state_volatile,
                 adc| {
                    // Update barometric altitude
                    // todo: Put back.
                    // let pressure = altimeter.read_pressure(i2c2);
                    // let temp = altimeter.read_temp(i2c2);
                    // params.s_z_msl = altimeter.estimate_altitude_msl(pressure, temp);

                    // todo: For these ADC readings, consider A: DMA. B: Not getting this data every update.

                    // Debug loop.
                    if *cx.local.update_loop_i % 700 == 0 {
                        let batt_v =
                            adc.reading_to_voltage(unsafe { ADC_READ_BUF }[0]) * ADC_BATT_DIVISION;
                        let curr_v =
                            adc.reading_to_voltage(unsafe { ADC_READ_BUF }[1]) * ADC_CURR_DIVISION;

                        // println!("Batt V: {} Curr V: {}", batt_v, curr_v);
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
                        println!(
                            "Attitude: roll {}, pitch: {}, yaw: {}\n",
                            params.s_roll, params.s_pitch, params.s_yaw
                        );

                        // println!("In acro mode: {:?}", *input_mode == InputMode::Acro);
                        // println!(
                        //     "Input mode sw: {:?}",
                        //     control_channel_data.input_mode == InputModeSwitch::Acro
                        // );

                        // println!("AHRS Q: {} {} {} {}",
                        //      ahrs.quaternion.w,
                        //      ahrs.quaternion.x,
                        //      ahrs.quaternion.y,
                        //      ahrs.quaternion.z
                        // );
                        //
                        //
                        //
                        // println!(
                        //     "RSSI1: {}, RSSI2: {}, Uplink qual: {} SNR: {}",
                        //     state_volatile.link_stats.uplink_rssi_1,
                        //     state_volatile.link_stats.uplink_rssi_2,
                        //     state_volatile.link_stats.uplink_link_quality,
                        //     state_volatile.link_stats.uplink_snr,
                        // );
                    }

                    if let OperationMode::Preflight = state_volatile.op_mode {
                        // exit this fn during preflight *after* measuring voltages using ADCs.
                        return;
                    }

                    if *cx.local.update_loop_i % LOGGING_UPDATE_RATIO == 0 {
                        // todo: Eg log rparamsto flash etc.
                    }

                    *cx.local.update_loop_i += 1;

                    safety::handle_arm_status(
                        cx.local.arm_signals_received,
                        cx.local.disarm_signals_received,
                        control_channel_data.arm_status,
                        &mut command_state.arm_status,
                        control_channel_data.throttle,
                        pid_rate,
                        pid_attitude,
                        pid_velocity,
                    );

                    if safety::LINK_LOST.load(Ordering::Acquire) {
                        // todo: For now, works by commanding attitude mode and level flight.
                        safety::link_lost_steady_state(
                            input_mode,
                            control_channel_data,
                            &mut command_state.arm_status,
                            cx.local.arm_signals_received,
                        );
                        return;
                    }

                    flight_ctrls::quad::handle_control_mode(
                        control_channel_data.input_mode,
                        input_mode,
                        state_volatile,
                    );

                    // todo: Do you want to update attitude here, or on each IMU data received?
                    // todo note that the DT you're passing in is currently the IMU update one I think?
                    // sensor_fusion::update_get_attitude(ahrs, params);

                    // todo: Support both UART telemetry from ESC, and analog current sense pin.
                    // todo: Read from an ADC or something, from teh ESC.
                    // let current_current = None;
                    // if let Some(current) = current_current {
                    // *power_used += current * DT;

                    // }
                    // else {
                    // *power_used += current_pwr.total() * DT;
                    // }

                    // Note: Arm status primary handler is in the `set_power` fn, but there's no reason
                    // to run the PIDs if not armed.
                    if command_state.arm_status != ArmStatus::Armed {
                        return;
                    }

                    match input_mode {
                        InputMode::Acro => {}
                        _ => {
                            pid::run_attitude(
                                params,
                                control_channel_data,
                                input_map,
                                attitudes_commanded,
                                rates_commanded,
                                pid_attitude,
                                filters,
                                input_mode,
                                autopilot_status,
                                cfg,
                                command_state,
                                coeffs,
                            );

                            if input_mode == &InputMode::Command {
                                if *cx.local.update_loop_i % VELOCITY_ATTITUDE_UPDATE_RATIO == 0 {
                                    pid::run_velocity(
                                        params,
                                        control_channel_data,
                                        input_map,
                                        velocities_commanded,
                                        attitudes_commanded,
                                        pid_velocity,
                                        filters,
                                        input_mode,
                                        autopilot_status,
                                        cfg,
                                        command_state,
                                        coeffs,
                                    );
                                }
                            }
                        }
                    }
                },
            )
    }

    /// Runs when new IMU data is ready. Trigger a DMA read.
    #[task(binds = EXTI4, shared = [cs_imu, dma, spi1], priority = 4)]
    fn imu_data_isr(cx: imu_data_isr::Context) {
        gpio::clear_exti_interrupt(4);

        (cx.shared.dma, cx.shared.cs_imu, cx.shared.spi1).lock(|dma, cs_imu, spi| {
            imu_shared::read_imu_dma(imu::READINGS_START_ADDR, spi, cs_imu, dma);
        });
    }

    // binds = DMA1_STR2,
    #[task(binds = DMA1_CH2, shared = [dma, spi1, current_params, input_mode, control_channel_data, input_map, autopilot_status,
    rates_commanded, pid_rate, pid_deriv_filters, imu_filters, ctrl_coeffs, command_state, cs_imu, user_cfg,
    motor_timers, ahrs, state_volatile, rf_limiter_timer], local = [fixed_wing_rate_loop_i, update_loop_i2], priority = 5)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it triggers the inner PID loop.
    /// Note: `rf_limiter_timer` is there, co-opted for use measuring this loop.
    /// todo: Currently getting 6.67kHz instead of 8kHz.
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        // Clear DMA interrupt this way due to RTIC conflict.
        #[cfg(feature = "h7")]
        unsafe {
            (*DMA1::ptr()).lifcr.write(|w| w.ctcif2().set_bit())
        }
        #[cfg(feature = "g4")]
        unsafe {
            (*DMA1::ptr()).ifcr.write(|w| w.tcif2().set_bit())
        }

        cx.shared.cs_imu.lock(|cs| {
            cs.set_high();
        });

        (
            cx.shared.current_params,
            cx.shared.ahrs,
            cx.shared.control_channel_data,
            cx.shared.input_mode,
            cx.shared.autopilot_status,
            cx.shared.rates_commanded,
            // cx.shared.control_mix,
            cx.shared.pid_rate,
            cx.shared.pid_deriv_filters,
            cx.shared.motor_timers,
            cx.shared.dma,
            cx.shared.ctrl_coeffs,
            cx.shared.user_cfg,
            cx.shared.input_map,
            cx.shared.command_state,
            cx.shared.spi1,
            // cx.shared.rf_limiter_timer, // todo temp
            cx.shared.state_volatile,
        )
            .lock(
                |params,
                 ahrs,
                 control_channel_data,
                 input_mode,
                 autopilot_status,
                 rates_commanded,
                 // control_mix,
                 pid_inner,
                 filters,
                 motor_timers,
                 dma,
                 coeffs,
                 cfg,
                 input_map,
                 command_state,
                 spi1,
                 // rf_limiter_timer, // todo temp
                 state_volatile| {
                    // Note that these steps are mandatory, per STM32 RM.
                    // spi.stop_dma only can stop a single channel atm, hence the 2 calls here.
                    dma.stop(setup::IMU_TX_CH);

                    spi1.stop_dma(setup::IMU_RX_CH, dma);

                    // // todo: TSing geting wrong freq. (3.5khz instead of 8)
                    // if *cx.local.update_loop_i2 % 700 == 0 {
                    //     let arr = rf_limiter_timer.get_max_duty();
                    //     let count = rf_limiter_timer.read_count();
                    //     let time = (count as f32 / arr as f32) / 1_000.;
                    //
                    //     defmt::println!("Time elapsed: {:?} Freq: {:?}", time, 1./time);
                    // }
                    // *cx.local.update_loop_i2 += 1;

                    // rf_limiter_timer.disable();
                    // rf_limiter_timer.reset_countdown();
                    // rf_limiter_timer.enable();

                    // todo: Temp TS code to verify rotordirection.
                    // if command_state.arm_status == ArmStatus::Armed {
                    //     // dshot::set_power(Rotor::R1, Rotor::R2, 0.0, 0.00, motor_timers, dma);
                    //     dshot::set_power(Rotor::R3, Rotor::R4, 0.00, 0.00, motor_timers, dma);
                    // } else {
                    //     dshot::set_power(Rotor::R1, Rotor::R2, 0., 0., motor_timers, dma);
                    // }
                    // return;

                    let mut imu_data =
                        imu_shared::ImuReadings::from_buffer(unsafe { &imu_shared::IMU_READINGS });

                    // todo: This is a good place to apply IMU calibration.

                    cx.shared.imu_filters.lock(|imu_filters| {
                        imu_filters.apply(&mut imu_data);
                    });

                    // Apply filtered gyro and accel readings directly to params.
                    params.v_roll = imu_data.v_roll;
                    params.v_pitch = imu_data.v_pitch;
                    params.v_yaw = imu_data.v_yaw;

                    params.a_x = imu_data.a_x;
                    params.a_y = imu_data.a_y;
                    params.a_z = imu_data.a_z;

                    // Note: Consider if you want to update the attitude using the primary update loop,
                    // vice each IMU update.
                    attitude_platform::update_get_attitude(ahrs, params);

                    // Note: There is an arm status primary handler is in the `set_power` fn, but if we abort
                    // here without it being set, power will remain at whatever state was set at time
                    // of disarm. Alternatively, we could neither return nor stop motors here, and
                    // let `set_power` handle it.
                    if command_state.arm_status != ArmStatus::Armed {
                        dshot::stop_all(motor_timers, dma);
                        return;
                    }

                    // Our fixed-wing update rate is limited by the servos, so we only run this
                    // 1 out of 16 IMU updates.
                    match cfg.aircraft_type {
                        AircraftType::Quadcopter => {
                            pid::run_rate(
                                params,
                                *input_mode,
                                autopilot_status,
                                // cfg,
                                control_channel_data,
                                rates_commanded,
                                // control_mix,
                                pid_inner,
                                filters,
                                &mut state_volatile.current_pwr,
                                &cfg.motor_mapping,
                                motor_timers,
                                dma,
                                coeffs,
                                cfg.max_speed_ver,
                                input_map,
                                command_state.arm_status,
                                DT_IMU,
                            );
                        }
                        AircraftType::FlyingWing => {
                            *cx.local.fixed_wing_rate_loop_i += 1;
                            if *cx.local.fixed_wing_rate_loop_i % FIXED_WING_RATE_UPDATE_RATIO == 0
                            {
                                pid::run_rate_flying_wing(
                                    params,
                                    *input_mode,
                                    autopilot_status,
                                    // cfg,
                                    control_channel_data,
                                    rates_commanded,
                                    // control_mix,
                                    pid_inner,
                                    filters,
                                    &mut state_volatile.ctrl_positions,
                                    &cfg.servo_wing_mapping,
                                    motor_timers,
                                    dma,
                                    coeffs,
                                    input_map,
                                    command_state.arm_status,
                                    DT_IMU * FIXED_WING_RATE_UPDATE_RATIO as f32,
                                );
                            }
                        }
                    }
                },
            );
    }

    // todo: Commented out USB ISR temporarily

    // binds = OTG_HS
    // todo H735 issue on GH: https://github.com/stm32-rs/stm32-rs/issues/743 (works on H743)
    // todo: NVIC interrupts missing here for H723 etc!
    #[task(binds = USB_LP, shared = [usb_dev, usb_serial, current_params, control_channel_data, command_state,
    user_cfg, state_volatile, motor_timers, batt_curr_adc, dma], local = [], priority = 7)]
    /// This ISR handles interaction over the USB serial port, eg for configuring using a desktop
    /// application.
    fn usb_isr(mut cx: usb_isr::Context) {
        (
            cx.shared.usb_dev,
            cx.shared.usb_serial,
            cx.shared.current_params,
            cx.shared.control_channel_data,
            cx.shared.command_state,
            cx.shared.user_cfg,
            cx.shared.state_volatile,
            cx.shared.motor_timers,
            cx.shared.batt_curr_adc,
            cx.shared.dma,
        )
            .lock(
                |usb_dev,
                 usb_serial,
                 params,
                 ch_data,
                 command_state,
                 user_cfg,
                 state_volatile,
                 motor_timers,
                 adc,
                 dma| {
                    if !usb_dev.poll(&mut [usb_serial]) {
                        return;
                    }

                    let mut buf = [0u8; 8];
                    match usb_serial.read(&mut buf) {
                        Ok(count) => {
                            usb_cfg::handle_rx(
                                usb_serial,
                                &buf,
                                count,
                                // params,
                                params.quaternion,
                                params.s_z_msl,
                                ch_data,
                                &state_volatile.link_stats,
                                &user_cfg.waypoints,
                                &mut command_state.arm_status,
                                &mut user_cfg.motor_mapping,
                                &mut state_volatile.op_mode,
                                motor_timers,
                                adc,
                                dma,
                            );
                        }
                        Err(_) => {
                            // println!("Error reading USB signal from PC");
                        }
                    }
                },
            )
    }

    // Note: We don't use `dshot_isr_r12` on H7; this is associated with timer 2.
    // These should be high priority, so they can shut off before the next 600kHz etc tick.
    #[cfg(not(feature = "h7"))]
    #[task(binds = DMA1_CH3, shared = [motor_timers], priority = 6)]
    /// We use this ISR to disable the DSHOT timer upon completion of a packet send.
    fn dshot_isr_r12(mut cx: dshot_isr_r12::Context) {
        // Feature-gating this fn isn't working,
        #[cfg(not(feature = "h7"))]
        // todo: Why is this gate required when we have feature-gated the fn?
        unsafe {
            (*DMA1::ptr()).ifcr.write(|w| w.tcif3().set_bit())
        }

        cx.shared.motor_timers.lock(|timers| {
            timers.r12.disable();
        });

        // Set to Output pin, low.
        unsafe {
            (*pac::GPIOA::ptr()).moder.modify(|_, w| {
                w.moder0().bits(0b01);
                w.moder1().bits(0b01)
            });
        }
        gpio::set_low(Port::A, 0);
        gpio::set_low(Port::A, 1);
    }

    #[task(binds = DMA1_CH4, shared = [motor_timers], priority = 6)]
    // #[task(binds = DMA1_STR4, shared = [motor_timers], priority = 6)]
    /// We use this ISR to disable the DSHOT t
    /// imer upon completion of a packet send.
    fn dshot_isr_r34(mut cx: dshot_isr_r34::Context) {
        // todo: Feature-gate this out on H7 or something? Not used.
        #[cfg(feature = "h7")]
        unsafe {
            (*DMA1::ptr()).hifcr.write(|w| w.ctcif4().set_bit())
        }
        #[cfg(feature = "g4")]
        unsafe {
            (*DMA1::ptr()).ifcr.write(|w| w.tcif4().set_bit())
        }

        cx.shared.motor_timers.lock(|timers| {
            #[cfg(feature = "h7")]
            timers.r1234.disable();
            #[cfg(feature = "g4")]
            timers.r34_servos.disable();
        });

        // Set to Output pin, low.
        cfg_if! {
            if #[cfg(feature = "h7")] {
                unsafe {
                     (*pac::GPIOC::ptr()).moder.modify(|_, w| {
                        w.moder6().bits(0b01);
                        w.moder7().bits(0b01);
                        w.moder8().bits(0b01);
                        w.moder9().bits(0b01)
                    });
                }

                gpio::set_low(Port::C, 6);
                gpio::set_low(Port::C, 7);
                gpio::set_low(Port::C, 8);
                gpio::set_low(Port::C, 9);

            } else {
                unsafe {
                    (*pac::GPIOB::ptr()).moder.modify(|_, w| {
                        w.moder0().bits(0b01);
                        w.moder1().bits(0b01)
                    });
                }

                gpio::set_low(Port::B, 0);
                gpio::set_low(Port::B, 1);
            }
        }
    }

    // #[task(binds = TIM8, shared = [motor_timers], priority = 6)] // H7
    #[task(binds = TIM3, shared = [motor_timers], priority = 6)] // G4
    /// We use this for fixed wing, to disable the timer after each pulse. We don't enable this interrupt
    /// on quadcopters.
    fn servo_isr(mut cx: servo_isr::Context) {
        cx.shared.motor_timers.lock(|timers| {
            #[cfg(feature = "h7")]
            let mut timer = &mut timers.servos;
            #[cfg(feature = "g4")]
            let mut timer = &mut timers.r34_servos;

            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
        });
    }

    // #[task(binds = TIM4, shared = [elrs_timer], priority = 4)]
    // /// ELRS timer.
    // fn elrs_timer_isr(mut cx: elrs_timer_isr::Context) {
    //     cx.shared.elrs_timer.lock(|timer| {
    //         timer.clear_interrupt(TimerInterrupt::Update);
    //         // elrs::HWtimerCallbackTick(timer);
    //         // elrs::HWtimerCallbackTock(timer);
    //     });
    // }

    #[task(binds = EXTI15_10, shared = [user_cfg, control_channel_data], local = [spi3], priority = 5)]
    /// We use this ISR when receiving data from the radio, via ELRS
    fn radio_data_isr(mut cx: radio_data_isr::Context) {
        gpio::clear_exti_interrupt(14);
        // todo: for when you impl native ELRS
        // (cx.shared.user_cfg, cx.shared.control_channel_data).lock(|cfg, ch_data| {
        //     // *manual_inputs = elrs::get_inputs(cx.local.spi3);
        //     // *ch_data = CtrlInputs::get_manual_inputs(cfg); // todo: this?
        // })
    }

    // #[task(binds = DMA1_CH6, shared = [dma, spi2, cs_elrs], priority = 3)]
    // /// Handle SPI ELRS data received.
    // fn elrs_rx_isr(mut cx: elrs_rx_isr::Context) {
    //     // Clear DMA interrupt this way due to RTIC conflict.
    //     unsafe { (*DMA1::ptr()).ifcr.write(|w| w.tcif6().set_bit()) }
    //
    //     (cx.shared.spi2, cx.shared.cs_elrs, cx.shared.dma).lock(|spi, cs, dma| {
    //         cs.set_high();
    //         dma.stop(setup::ELRS_TX_CH); // spi.stop_dma only can stop a single channel atm.
    //         spi.stop_dma(setup::ELRS_RX_CH, dma);
    //     });
    // }

    /// If this triggers, it means we've lost the link. (Note that this is for TIM17)
    // #[task(binds = TIM17, shared = [lost_link_timer], priority = 2)]
    #[task(binds = TIM1_TRG_COM, shared = [lost_link_timer], priority = 2)]
    fn lost_link_isr(mut cx: lost_link_isr::Context) {
        println!("Lost the link!");

        (cx.shared.lost_link_timer).lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
        });

        safety::LINK_LOST.store(true, Ordering::Release);
    }

    #[task(binds = USART3, shared = [uart3, dma, control_channel_data,
    lost_link_timer, rf_limiter_timer, state_volatile], priority = 5)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it triggers the inner PID loop. This is a high priority interrupt, since we need
    /// to start capturing immediately, or we'll miss part of the packet.
    fn crsf_isr(cx: crsf_isr::Context) {
        (
            cx.shared.uart3,
            cx.shared.dma,
            cx.shared.control_channel_data,
            cx.shared.lost_link_timer,
            cx.shared.rf_limiter_timer,
            cx.shared.state_volatile,
        )
            .lock(
                |uart, dma, ch_data, lost_link_timer, rf_limiter_timer, state_volatile| {
                    uart.clear_interrupt(UsartInterrupt::Idle);

                    // println!("CRSF");

                    if rf_limiter_timer.is_enabled() {
                        // todo: Put this back and figure out why this keeps happening.
                        // return;
                    } else {
                        rf_limiter_timer.reset_countdown();
                        rf_limiter_timer.enable();
                    }

                    if let Some(crsf_data) =
                        crsf::handle_packet(uart, dma, setup::CRSF_RX_CH, setup::CRSF_TX_CH)
                    {
                        match crsf_data {
                            crsf::PacketData::ChannelData(data) => {
                                *ch_data = data;

                                lost_link_timer.reset_countdown();
                                lost_link_timer.enable();

                                if safety::LINK_LOST
                                    .compare_exchange(
                                        true,
                                        false,
                                        Ordering::Relaxed,
                                        Ordering::Relaxed,
                                    )
                                    .is_ok()
                                {
                                    println!("Link re-aquired");
                                    // todo: Execute re-acq procedure
                                }
                            }
                            crsf::PacketData::LinkStats(stats) => {
                                state_volatile.link_stats = stats;
                            }
                        }
                    }
                },
            );
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
