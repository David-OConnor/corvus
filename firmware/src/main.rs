#![no_main]
#![no_std]
#![allow(mixed_script_confusables)] // eg variable names that include greek letters.

use core::{
    f32::consts::TAU,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use cfg_if::cfg_if;

use cortex_m::{self, asm, delay::Delay};

use stm32_hal2::{
    self,
    adc::{Adc, AdcDevice},
    clocks::{self, Clk48Src, Clocks, CrsSyncSrc, InputSrc, PllSrc},
    dma::{self, ChannelCfg, Circular, Dma, DmaChannel},
    flash::Flash,
    gpio::{self, Edge, Pin, PinMode, Port},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{
        self, DMA1, I2C1, I2C2, SPI1, SPI2, SPI3, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, USART3,
    },
    rtc::Rtc,
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::{Timer, TimerConfig, TimerInterrupt},
    usart::{Usart, UsartInterrupt},
};

#[cfg(feature = "h7")]
use stm32_hal2::clocks::PllCfg;

use defmt::println;

// todo: Low power mode when idle to save battery (minor), and perhaps improve lifetime.

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
use crate::{
    clocks::VosRange,
    power::{SupplyConfig, VoltageLevel},
};

use defmt_rtt as _; // global logger
use panic_probe as _;

mod control_interface;
mod drivers;
mod filter_imu;
mod flight_ctrls;
mod lin_alg;
mod madgwick;
// mod osd;
mod cfg_storage;
mod pid;
mod pid_tuning;
mod ppks;
mod protocols;
mod sensor_fusion;
mod setup;
mod util;

use drivers::baro_dps310 as baro;
use drivers::gps_x as gps;
// pub use, so we can use this rename in `sensor_fusion` to interpret the DMA buf.
pub use drivers::imu_icm426xx as imu;
// use drivers::imu_ism330dhcx as imu;
use drivers::tof_vl53l1 as tof;

use control_interface::{ChannelData, LinkStats};

use protocols::{crsf, dshot, usb_cfg};

use flight_ctrls::{
    ArmStatus, AutopilotStatus, AxisLocks, CommandState, CtrlInputs, InputMap, InputMode, Params,
    Rotor, RotorMapping, RotorPosition, RotorPower, POWER_LUT,
};

use filter_imu::ImuFilters;
use pid::{CtrlCoeffGroup, PidDerivFilters, PidGroup};

use madgwick::Ahrs;

use ppks::{Location, LocationType};

// 512k flash. Page size 2kbyte. 72-bit data read (64 bits plus 8 ECC bits)
// Each page is 8 rows of 256 bytes. 255 pages in main memory.
const ONBOARD_FLASH_START_PAGE: usize = 160;

// Our DT timer speed, in Hz.
const DT_TIM_FREQ: u32 = 200_000_000;

// The rate our main program updates, in Hz.
// Currently set to
const IMU_UPDATE_RATE: f32 = 8_000.;

// todo: Set update rate attitude back to 1600 etc. Slower rate now since we're using this loop to TS.
const UPDATE_RATE_ATTITUDE: f32 = 1_600.; // IMU rate / 5.
const UPDATE_RATE_VELOCITY: f32 = 400.; // IMU rate / 20.

// How many inner loop ticks occur between mid and outer loop.
// const OUTER_LOOP_RATIO: usize = 10;

const DT_IMU: f32 = 1. / IMU_UPDATE_RATE;
const DT_ATTITUDE: f32 = 1. / UPDATE_RATE_ATTITUDE;
const DT_VELOCITY: f32 = 1. / UPDATE_RATE_VELOCITY;

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
static LOOP_I: AtomicUsize = AtomicUsize::new(0);

// We use this to make sure a received char equal to the FC destination value (eg as part
// of channel data) doesn't interrupt our data receive.
static CRSF_RX_IN_PROG: AtomicBool = AtomicBool::new(false);

// Enable this to print parameters (eg location, altitude, attitude, angular rates etc) to the console.
// const DEBUG_PARAMS: bool = true;

// todo: Course set mode. Eg, point at thing using controls, activate a button,
// todo then the thing flies directly at the target.

// With this in mind: Store params in a global array. Maybe [ParamsInst; N], or maybe a numerical array for each param.

// todo: Consider a nested loop, where inner manages fine-tuning of angle, and outer
// manages directions etc. (?) look up prior art re quads and inner/outer loops.

// todo: Panic button that recovers the aircraft if you get spacial D etc.

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

#[derive(Clone, Copy)]
pub enum OperationMode {
    /// Eg flying
    Normal,
    /// Plugged into a PC to verify motors, IMU readings, control readings etc, and adjust settings
    Preflight,
}

/// State that doesn't get saved to flash.
pub struct StateVolatile {
    op_mode: OperationMode,
    /// The GPS module is connected. Detected on init.
    gps_attached: bool,
    /// The time-of-flight sensor module is connected. Detected on init.
    tof_attached: bool,
    // FOr now, we use "link lost" to include never having been connected.
    // connected_to_controller: bool,
}

impl Default for StateVolatile {
    fn default() -> Self {
        Self {
            op_mode: OperationMode::Normal,
            gps_attached: false,
            tof_attached: false,
            // connected_to_controller: false,
        }
    }
}

/// User-configurable settings. These get saved to and loaded from internal flash.
pub struct UserCfg {
    /// Set a ceiling the aircraft won't exceed. Defaults to 400' (Legal limit in US for drones).
    /// In meters.
    ceiling: Option<f32>,
    /// In Attitude and related control modes, max pitch angle (from straight up), ie
    /// full speed, without going horizontal or further.
    max_angle: f32, // radians
    max_velocity: f32, // m/s
    idle_pwr: f32,
    /// These input ranges map raw output from a manual controller to full scale range of our control scheme.
    /// (min, max). Set using an initial calibration / setup procedure.
    pitch_input_range: (f32, f32),
    roll_input_range: (f32, f32),
    yaw_input_range: (f32, f32),
    throttle_input_range: (f32, f32),
    /// Is the aircraft continuously collecting data on obstacles, and storing it to external flash?
    mapping_obstacles: bool,
    max_speed_hor: f32,
    max_speed_ver: f32,
    /// It's common to arbitrarily wire motors to the ESC. Reverse each from its
    /// default direction, as required.
    motors_reversed: (bool, bool, bool, bool),
    /// Map motor connection number to position.
    motor_mapping: RotorMapping,
    baro_cal: baro::BaroCalPt,
    // Note that this inst includes idle power.
    // todo: We want to store this inst, but RTIC doesn't like it not being sync. Maybe static mut.
    // todo. For now, lives in the acro PID fn lol.
    // power_interp_inst: dsp_sys::arm_linear_interp_instance_f32,
}

impl Default for UserCfg {
    fn default() -> Self {
        Self {
            ceiling: Some(122.),
            // todo: Do we want max angle and vel here? Do we use them, vice settings in InpuMap?
            max_angle: TAU * 0.22,
            max_velocity: 30., // todo: raise?
            // Note: Idle power now handled in `power_interp_inst`
            idle_pwr: 0.02, // scale of 0 to 1.
            // todo: Find apt value for these
            pitch_input_range: (0., 1.),
            roll_input_range: (0., 1.),
            yaw_input_range: (0., 1.),
            throttle_input_range: (0., 1.),
            mapping_obstacles: false,
            max_speed_hor: 20.,
            max_speed_ver: 20.,
            motors_reversed: (false, false, false, false),
            motor_mapping: RotorMapping {
                r1: RotorPosition::AftLeft,
                r2: RotorPosition::FrontLeft,
                r3: RotorPosition::AftRight,
                r4: RotorPosition::FrontRight,
            },
            baro_cal: Default::default(),
            // Make sure to update this interp table if you change idle power.
            // todo: This LUT setup is backwards! You need to put thrust on a fixed spacing,
            // todo, and throttle as dynamic (y)!
            // power_interp_inst: dsp_sys::arm_linear_interp_instance_f32 {
            //     nValues: 11,
            //     x1: 0.,
            //     xSpacing: 0.1,
            //     pYData: [
            //         // Idle power.
            //         0.02, // Make sure this matches the above.
            //         POWER_LUT[0],
            //         POWER_LUT[1],
            //         POWER_LUT[2],
            //         POWER_LUT[3],
            //         POWER_LUT[4],
            //         POWER_LUT[5],
            //         POWER_LUT[6],
            //         POWER_LUT[7],
            //         POWER_LUT[8],
            //         POWER_LUT[9],
            //         POWER_LUT[10],
            //     ].as_mut_ptr()
            // },
        }
    }
}

#[derive(Clone, Copy)]
/// Role in a swarm of drones
pub enum SwarmRole {
    Queen,
    Worker(u16),    // id
    PersonFollower, // When your queen is human.
}

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

            let temp = 0.; // todo: Which sensor reads temp? The IMU?
            baro::calibrate(f.z, temp, i2c2);
        }
        Err(_) => (), // todo
    }

    // todo: Use Rel0 location type if unable to get fix.
}

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;
    use cortex_m::peripheral::NVIC;
    use stm32_hal2::dma::DmaInterrupt;

    // todo: Move vars from here to `local` as required.
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
        pid_velocity: PidGroup,
        pid_attitude: PidGroup,
        pid_rate: PidGroup,
        control_channel_data: ChannelData,
        control_link_stats: LinkStats,
        // manual_inputs: CtrlInputs,
        axis_locks: AxisLocks,
        current_pwr: RotorPower,
        dma: Dma<DMA1>,
        spi1: Spi<SPI1>,
        spi2: Spi<SPI2>,
        cs_imu: Pin,
        cs_elrs: Pin,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        uart3: Usart<USART3>, // for ELRS over CRSF.
        flash_onboard: Flash,
        // rtc: Rtc,
        update_timer: Timer<TIM15>,
        rf_limiter_timer: Timer<TIM16>,
        lost_link_timer: Timer<TIM17>,
        rotor_timer_a: Timer<TIM2>,
        rotor_timer_b: Timer<TIM3>,
        elrs_timer: Timer<TIM4>,
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
    }

    #[local]
    struct Local {
        spi3: Spi<SPI3>,
        arm_signals_received: u8, // todo: Put in state volatile.
        disarm_signals_received: u8,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let mut cp = cx.core;
        // Set up microcontroller peripherals
        let mut dp = pac::Peripherals::take().unwrap();

        #[cfg(feature = "h7")]
        SupplyConfig::DirectSmps.setup(&mut dp.PWR, VoltageLevel::V2_5);

        // todo: Range 1 boost mode on G4!!
        let clock_cfg = Clocks {
            // Config for 480Mhz full speed:
            #[cfg(feature = "h7")]
            pll_src: PllSrc::Hse(16_000_000),
            #[cfg(feature = "g4")]
            input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
            // vos_range: VosRange::VOS0, // Note: This may use extra power. todo: Put back!
            #[cfg(feature = "h7")]
            pll1: PllCfg {
                divm: 4, // To compensate with 8Mhz HSE instead of 64Mhz HSI
                // divn: 480,// todo: Put back! No longer working??
                ..Default::default()
            },
            hsi48_on: true,
            clk48_src: Clk48Src::Hsi48,
            #[cfg(feature = "g4")]
            boost_mode: true, // Required for speeds > 150Mhz.
            ..Default::default()
        };

        clock_cfg.setup().unwrap();

        // Enable the Clock Recovery System, which improves HSI48 accuracy.
        clocks::enable_crs(CrsSyncSrc::Usb);

        // (We don't need the debug workaround, since we don't use low power modes.)
        // debug_workaround();

        // Improves performance, at a cost of slightly increased power use.
        cp.SCB.invalidate_icache();
        cp.SCB.enable_icache();
        // cp.SCB.clean_invalidate_dcache(); // todo?
        // cp.SCB.enable_dcache(); // todo?

        // Set up pins with appropriate modes.
        setup::setup_pins();
        // loop {} // todo temp!

        let mut dma = Dma::new(dp.DMA1);
        #[cfg(feature = "g4")]
        dma::enable_mux1();

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
        let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);
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
        let mut elrs_dio = Pin::new(Port::C, 13, PinMode::Input);
        elrs_dio.enable_interrupt(Edge::Rising);
        let mut cs_elrs = Pin::new(Port::C, 15, PinMode::Input);
        cs_elrs.set_high();

        // We use SPI3 for flash. // todo: Find max speed and supported modes.
        let spi3 = Spi::new(dp.SPI3, Default::default(), BaudRate::Div32);

        let mut cs_flash = Pin::new(Port::C, 6, PinMode::Output);
        cs_flash.set_high();

        // We use I2C for the TOF sensor.(?)
        let i2c_tof_cfg = I2cConfig {
            speed: I2cSpeed::FastPlus1M,
            // speed: I2cSpeed::Fast400k,
            ..Default::default()
        };

        // We use I2C1 for offboard sensors: Magnetometer, GPS, and TOF sensor.
        let mut i2c1 = I2c::new(dp.I2C1, i2c_tof_cfg.clone(), &clock_cfg);

        // We use I2C for the TOF sensor.(?)
        let i2c_baro_cfg = I2cConfig {
            speed: I2cSpeed::Fast400K,
            ..Default::default()
        };

        // We use I2C2 for the barometer.
        let mut i2c2 = I2c::new(dp.I2C2, i2c_baro_cfg, &clock_cfg);

        // println!("Pre altimeter setup");
        baro::setup(&mut i2c2);
        // println!("Altimeter setup complete");
        let pressure = baro::read(&mut i2c2);
        println!("Pressure: {}", pressure);

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

        let mut buf = [0_u8; 400];

        // We use the RTC to assist with power use measurement.
        let rtc = Rtc::new(dp.RTC, Default::default());

        // We use the ADC to measure battery voltage.
        let batt_adc = Adc::new_adc1(dp.ADC1, AdcDevice::One, Default::default(), &clock_cfg);

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
            } else if #[cfg(feature = "mercury-g4")] {

                let mut rotor_timer_a =
                    Timer::new_tim2(dp.TIM2, 1., rotor_timer_cfg.clone(), &clock_cfg);

                let mut rotor_timer_b = Timer::new_tim3(dp.TIM3, 1., rotor_timer_cfg, &clock_cfg);
            }
        }

        dshot::setup_timers(&mut rotor_timer_a, &mut rotor_timer_b);

        let mut lost_link_timer = Timer::new_tim17(
            dp.TIM17,
            1. / flight_ctrls::LOST_LINK_TIMEOUT,
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

        // let delay_timer = Timer::new_tim5(
        //     dp.TIM5,
        //     1.,
        //      Default::default(),
        // &clock_cfg);

        let mut user_cfg = UserCfg::default();

        // todo: ID connected sensors etc by checking their device ID etc.
        let mut state_volatile = StateVolatile::default();

        // Hard-coded for our test setup
        // user_cfg.motors_reversed = (false, true, true, true);
        // user_cfg.motors_reversed = (true, true, true, true);

        cfg_if! {
            if #[cfg(feature = "g4")] {
                let usb = Peripheral { regs: dp.USB };

                unsafe { USB_BUS = Some(UsbBus::new(usb)) };

                let usb_serial = SerialPort::new(unsafe { &USB_BUS.as_ref().unwrap() });

                let usb_dev = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(0x16c0, 0x27dd))
                    .manufacturer("Anyleaf")
                    .product("Serial port")
                    // We use `serial_number` to identify the device to the PC. If it's too long,
                    // we get permissions errors on the PC.
                    .serial_number("g4") // todo: Try 2 letter only if causing trouble?
                    .device_class(USB_CLASS_CDC)
                    .build();
            }
        }

        // todo: DMA for voltage ADC (?)

        let mut flash_onboard = Flash::new(dp.FLASH);

        // todo: Testing flash
        // flash_onboard.unlock();
        let mut flash_buf = [0; 5];
        // let cfg_data = flash_onboard.read_to_buffer(ONBOARD_FLASH_START_PAGE, 0, &mut flash_buf);

        println!("Flash Buf ( should be 1, 2, 3, 0, 0): {:?}", flash_buf);
        // flash_onboard.erase_page(ONBOARD_FLASH_START_PAGE);
        // flash_onboard.write_page(ONBOARD_FLASH_START_PAGE, &[1, 2, 3, 0, 0]);

        let mut params = Default::default();

        // todo: Instead of a `Barometer` struct, perhaps store baro calibration elsewhere.
        baro::setup(&mut i2c2);

        let mut base_point = Location::default();
        init_sensors(&mut params, &mut base_point, &mut i2c1, &mut i2c2);

        // todo: Calibation proecedure, either in air or on ground.
        let ahrs_settings = madgwick::Settings {
            gain: 0.5,
            accel_rejection: 10.,
            magnetic_rejection: 20.,
            rejection_timeout: (5. * crate::IMU_UPDATE_RATE) as u32,
        };

        // Note: Calibration and offsets ares handled handled by their defaults currently.
        let ahrs_cal = madgwick::AhrsCalibration::default(); // todo - load from flash

        let ahrs = Ahrs::new(&ahrs_settings, ahrs_cal, crate::IMU_UPDATE_RATE as u32);

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
                ctrl_coeffs: Default::default(),
                current_params: params,
                velocities_commanded: Default::default(),
                attitudes_commanded: Default::default(),
                rates_commanded: Default::default(),
                pid_velocity: Default::default(),
                pid_attitude: Default::default(),
                pid_rate: Default::default(),
                control_channel_data: Default::default(),
                control_link_stats: Default::default(),
                // manual_inputs: Default::default(),
                axis_locks: Default::default(),
                current_pwr: Default::default(),
                dma,
                spi1,
                spi2,
                cs_imu,
                cs_elrs,
                i2c1,
                i2c2,
                uart3,
                // rtc,
                update_timer,
                rf_limiter_timer,
                lost_link_timer,
                rotor_timer_a,
                rotor_timer_b,
                elrs_timer,
                // delay_timer,
                usb_dev,
                usb_serial,
                flash_onboard,
                power_used: 0.,
                pid_deriv_filters: PidDerivFilters::new(),
                imu_filters: ImuFilters::new(),
                base_point,
                command_state: Default::default(),
                ahrs,
            },
            Local {
                spi3,
                arm_signals_received: 0,
                disarm_signals_received: 0,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [user_cfg, rotor_timer_a, rotor_timer_b, dma])]
    fn idle(cx: idle::Context) -> ! {
        // Set up DSHOT here, since we need interrupts enabled.

        (
            cx.shared.user_cfg,
            cx.shared.rotor_timer_a,
            cx.shared.rotor_timer_b,
            cx.shared.dma,
        )
            .lock(|user_cfg, rotor_timer_a, rotor_timer_b, dma| {
                // Indicate to the ESC we've started with 0 throttle. Not sure if delay is strictly required.

                let cp = unsafe { cortex_m::Peripherals::steal() };
                let mut delay = Delay::new(cp.SYST, 170_000_000);

                // todo: Figure out which delays you need. Intent is to allow ESC to warm up before issuing
                // todo commands.
                delay.delay_ms(1_000);

                dshot::stop_all(rotor_timer_a, rotor_timer_b, dma);
                delay.delay_ms(1);
                //
                // dshot::setup_motor_dir(
                //     user_cfg.motors_reversed,
                //     rotor_timer_a,
                //     rotor_timer_b,
                //     dma,
                // );
            });

        loop {
            asm::nop();
        }
    }

    // todo: Go through these tasks, and make sure they're not reserving unneded shared params.

    // todo: Remove rotor timers, spi, and dma from this ISR; we only use it for tresting DSHOT
    #[task(
    binds = TIM1_BRK_TIM15,
    shared = [current_params, input_map, current_pwr,
    velocities_commanded, attitudes_commanded, rates_commanded, pid_velocity, pid_attitude, pid_rate,
    pid_deriv_filters, power_used, input_mode, autopilot_status, user_cfg, command_state, ctrl_coeffs,
    dma, rotor_timer_a, rotor_timer_b, ahrs, axis_locks, control_channel_data, control_link_stats,
    lost_link_timer, state_volatile,
    ],
    local = [arm_signals_received, disarm_signals_received],

    priority = 2
    )]
    /// This runs periodically, on a ~1kHz timer. It's used to trigger the attitude and velocity PID loops, ie for
    /// sending commands to the attitude and rate PID loop based on things like autopilot, command-mode etc.
    fn update_isr(mut cx: update_isr::Context) {
        unsafe { (*pac::TIM15::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }

        static mut i: usize = 0; // For debugging.

        (
            cx.shared.current_params,
            cx.shared.ahrs,
            cx.shared.control_channel_data,
            cx.shared.input_map,
            cx.shared.current_pwr,
            cx.shared.velocities_commanded,
            cx.shared.attitudes_commanded,
            cx.shared.rates_commanded,
            cx.shared.pid_velocity,
            cx.shared.pid_attitude,
            cx.shared.pid_deriv_filters,
            cx.shared.power_used,
            cx.shared.input_mode,
            cx.shared.autopilot_status,
            cx.shared.user_cfg,
            cx.shared.command_state,
            cx.shared.rotor_timer_a,
            cx.shared.rotor_timer_b,
            cx.shared.dma,
            cx.shared.ctrl_coeffs,
            cx.shared.lost_link_timer,
            cx.shared.state_volatile,
        )
            .lock(
                |params,
                 ahrs,
                 control_channel_data,
                 input_map,
                 current_pwr,
                 velocities_commanded,
                 attitudes_commanded,
                 rates_commanded,
                 pid_velocity,
                 pid_attitude,
                 filters,
                 power_used,
                 input_mode,
                 autopilot_status,
                 cfg,
                 command_state,
                 rotor_timer_a,
                 rotor_timer_b,
                 dma,
                 coeffs,
                 lost_link_timer,
                 state_volatile| {
                    if let OperationMode::Preflight = state_volatile.op_mode {
                        return;
                    }

                    // Debug loop.
                    if unsafe { i } % 1_000 == 0 {
                        // todo temp
                        println!(
                            "IMU Data: Ax {}, Ay: {}, Az: {}",
                            params.a_x, params.a_y, params.a_z
                        );

                        println!(
                            "IMU Data: roll {}, pitch: {}, yaw: {}",
                            params.v_roll, params.v_pitch, params.v_yaw
                        );

                        println!(
                            "Attitude: roll {}, pitch: {}, yaw: {}\n",
                            params.s_roll, params.s_pitch, params.s_yaw
                        );

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
                        //     "up RSSI: {}, Uplink qual: {} SNR: {}, tx pwr: {}, down RSSI: {}, down qual: {}, down snr: {}",
                        //     link_stats.uplink_rssi_1,
                        //     link_stats.uplink_link_quality,
                        //     link_stats.uplink_snr,
                        //     link_stats.uplink_tx_power,
                        //     link_stats.downlink_rssi,
                        //     link_stats.downlink_link_quality,
                        //     link_stats.downlink_snr,
                        // );
                    }

                    unsafe {
                        i += 1;
                    };

                    flight_ctrls::handle_arm_status(
                        cx.local.arm_signals_received,
                        cx.local.disarm_signals_received,
                        control_channel_data.arm_status,
                        &mut command_state.arm_status,
                        control_channel_data.throttle,
                    );

                    if flight_ctrls::LINK_LOST.load(Ordering::Acquire) {
                        // todo: For now, works by commanding attitude mode and level flight.
                        flight_ctrls::link_lost_steady_state(
                            input_mode,
                            control_channel_data,
                            &mut command_state.arm_status,
                            cx.local.arm_signals_received,
                        );
                        return;
                    }

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
                        InputMode::Acro => {
                            return;
                        }
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
                                if LOOP_I.fetch_add(1, Ordering::Relaxed)
                                    % VELOCITY_ATTITUDE_UPDATE_RATIO
                                    == 0
                                {
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
            sensor_fusion::read_imu_dma(imu::READINGS_START_ADDR, spi, cs_imu, dma);
        });
    }

    #[task(binds = DMA1_CH2, shared = [dma, spi1, current_params, input_mode, control_channel_data, input_map, autopilot_status,
    rates_commanded, pid_rate, pid_deriv_filters, imu_filters, current_pwr, ctrl_coeffs, command_state, cs_imu, user_cfg,
    rotor_timer_a, rotor_timer_b, ahrs, state_volatile], priority = 5)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it triggers the inner PID loop.
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        // Clear DMA interrupt this way due to RTIC conflict.
        unsafe { (*DMA1::ptr()).ifcr.write(|w| w.tcif2().set_bit()) }

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
            cx.shared.pid_rate,
            cx.shared.pid_deriv_filters,
            cx.shared.current_pwr,
            cx.shared.rotor_timer_a,
            cx.shared.rotor_timer_b,
            cx.shared.dma,
            cx.shared.ctrl_coeffs,
            cx.shared.user_cfg,
            cx.shared.input_map,
            cx.shared.command_state,
            cx.shared.spi1,
            cx.shared.state_volatile,
        )
            .lock(
                |params,
                 ahrs,
                 control_channel_data,
                 input_mode,
                 autopilot_status,
                 rates_commanded,
                 pid_inner,
                 filters,
                 current_pwr,
                 rotor_timer_a,
                 rotor_timer_b,
                 dma,
                 coeffs,
                 cfg,
                 input_map,
                 command_state,
                 spi1,
                 state_volatile| {
                    // Note that these steps are mandatory, per STM32 RM.
                    // spi.stop_dma only can stop a single channel atm, hence the 2 calls here.
                    dma.stop(setup::IMU_TX_CH);
                    spi1.stop_dma(setup::IMU_RX_CH, dma);

                    // todo: Temp TS code to verify rotordirection.
                    // if command_state.arm_status == ArmStatus::Armed {
                    //     dshot::set_power_a(Rotor::R1, Rotor::R2, 0.05, 0.05, rotor_timer_a, dma);
                    //     dshot::set_power_b(Rotor::R3, Rotor::R4, 0.05, 0.05, rotor_timer_b, dma);
                    // } else {
                    //     dshot::set_power_a(Rotor::R1, Rotor::R2, 0., 0., rotor_timer_a, dma);
                    //     dshot::set_power_b(Rotor::R3, Rotor::R4, 0., 0., rotor_timer_b, dma);
                    // }
                    // return;

                    let mut imu_data = sensor_fusion::ImuReadings::from_buffer(unsafe {
                        &sensor_fusion::IMU_READINGS
                    });

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
                    sensor_fusion::update_get_attitude(ahrs, params);

                    if let OperationMode::Preflight = state_volatile.op_mode {
                        return;
                    }

                    // Note: There is an arm status primary handler is in the `set_power` fn, but if we abort
                    // here without it being set, power will remain at whatever state was set at time
                    // of disarm. Alternatively, we could neither return nor stop motors here, and
                    // let `set_power` handle it.
                    if command_state.arm_status != ArmStatus::Armed {
                        dshot::stop_all(rotor_timer_a, rotor_timer_b, dma);
                        return;
                    }

                    pid::run_rate(
                        params,
                        *input_mode,
                        autopilot_status,
                        cfg,
                        control_channel_data,
                        rates_commanded,
                        pid_inner,
                        filters,
                        current_pwr,
                        &cfg.motor_mapping,
                        rotor_timer_a,
                        rotor_timer_b,
                        dma,
                        coeffs,
                        cfg.max_speed_ver,
                        input_map,
                        command_state.arm_status,
                        DT_IMU,
                    );
                },
            );
    }

    // todo: Commented out USB ISR so we don't get the annoying beeps from PC on conn/dc

    #[task(binds = USB_LP, shared = [usb_dev, usb_serial, current_params], local = [], priority = 3)]
    /// This ISR handles interaction over the USB serial port, eg for configuring using a desktop
    /// application.
    fn usb_isr(mut cx: usb_isr::Context) {
        (cx.shared.usb_dev, cx.shared.usb_serial, cx.shared.current_params).lock(
            |usb_dev, usb_serial, params| {
                if !usb_dev.poll(&mut [usb_serial]) {
                    return;
                }

                let mut buf = [0u8; 8];
                match usb_serial.read(&mut buf) {
                    Ok(count) => {
                        // usb_serial.write(&[1, 2, 3]).ok();
                        // todo: Only pass params if needed?
                        usb_cfg::handle_rx(usb_serial, &buf, count, params);
                    }
                    Err(_) => {
                        println!("Error reading USB signal from PC");
                    }
                }
            },
        )
    }

    // These should be high priority, so they can shut off before the next 600kHz etc tick.
    #[task(binds = DMA1_CH3, shared = [rotor_timer_a], priority = 6)]
    /// We use this ISR to disable the DSHOT timer upon completion of a packet send.
    fn dshot_isr_a(mut cx: dshot_isr_a::Context) {
        unsafe { (*DMA1::ptr()).ifcr.write(|w| w.tcif3().set_bit()) }

        cx.shared.rotor_timer_a.lock(|timer| {
            timer.disable();
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

    #[task(binds = DMA1_CH4, shared = [rotor_timer_b], priority = 6)]
    /// We use this ISR to disable the DSHOT timer upon completion of a packet send.
    fn dshot_isr_b(mut cx: dshot_isr_b::Context) {
        unsafe { (*DMA1::ptr()).ifcr.write(|w| w.tcif4().set_bit()) }

        cx.shared.rotor_timer_b.lock(|timer| {
            timer.disable();
        });

        // Set to Output pin, low.
        unsafe {
            (*pac::GPIOB::ptr()).moder.modify(|_, w| {
                w.moder0().bits(0b01);
                w.moder1().bits(0b01)
            });
        }
        gpio::set_low(Port::B, 0);
        gpio::set_low(Port::B, 1);
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
    //         dma.stop(DmaChannel::C1); // spi.stop_dma only can stop a single channel atm.
    //         spi.stop_dma(DmaChannel::C2, dma);
    //     });
    // }

    /// If this triggers, it means we've lost the link. (Note that this is for TIM17)
    #[task(binds = TIM1_TRG_COM, shared = [lost_link_timer], priority = 2)]
    fn lost_link_isr(mut cx: lost_link_isr::Context) {
        println!("Lost the link!");

        (cx.shared.lost_link_timer).lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
        });

        flight_ctrls::LINK_LOST.store(true, Ordering::Release);
    }

    #[task(binds = USART3, shared = [uart3, dma, control_channel_data, control_link_stats,
    lost_link_timer, rf_limiter_timer, state_volatile], priority = 5)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it triggers the inner PID loop. This is a high priority interrupt, since we need
    /// to start capturing immediately, or we'll miss part of the packet.
    fn crsf_isr(mut cx: crsf_isr::Context) {
        (
            cx.shared.uart3,
            cx.shared.dma,
            cx.shared.control_channel_data,
            cx.shared.control_link_stats,
            cx.shared.lost_link_timer,
            cx.shared.rf_limiter_timer,
            cx.shared.state_volatile,
        )
            .lock(
                |uart, dma, ch_data, link_stats, lost_link_timer, rf_limiter_timer, state_volatile| {
                    uart.clear_interrupt(UsartInterrupt::Idle);

                    if rf_limiter_timer.is_enabled() {
                        // todo: Put this back and figure out why this keeps happening.
                        // return;
                    } else {
                        rf_limiter_timer.reset_countdown();
                        rf_limiter_timer.enable();
                    }

                    if let Some(crsf_data) =
                        crsf::handle_packet(uart, dma, setup::CRSF_RX_CH, DmaChannel::C8)
                    {
                        match crsf_data {
                            crsf::PacketData::ChannelData(data) => {
                                *ch_data = data;

                                lost_link_timer.reset_countdown();
                                lost_link_timer.enable();
                                // state_volatile.connected_to_controller = true;


                                if flight_ctrls::LINK_LOST
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
                                *link_stats = stats;
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

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
