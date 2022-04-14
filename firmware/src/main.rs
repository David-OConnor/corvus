#![no_main]
#![no_std]
// #![allow(non_ascii_idents)] // todo: May no longer be required
#![allow(mixed_script_confusables)] // eg variable names that include greek letters.

use core::{
    f32::consts::TAU,
    sync::atomic::{AtomicUsize, Ordering},
};

use cfg_if::cfg_if;

use cortex_m::{self, asm};

use stm32_hal2::{
    self,
    adc::{Adc, AdcDevice},
    clocks::{self, Clk48Src, Clocks, CrsSyncSrc, InputSrc, PllSrc},
    dac, // todo temp
    dma::{self, ChannelCfg, Dma, DmaChannel, DmaInterrupt},
    flash::Flash,
    gpio::{Edge, Pin, PinMode, Port},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, DMA1, I2C1, I2C2, SPI1, SPI2, SPI3, TIM15, TIM2, TIM3},
    rtc::Rtc,
    spi::{BaudRate, Spi, SpiConfig, SpiMode},
    timer::{
        self, Alignment, MasterModeSelection, OutputCompare, TimChannel, Timer, TimerConfig,
        TimerInterrupt,
    },
    usart::Usart,
    usb,
};

#[cfg(feature = "mercury-h7")]
use stm32_hal2::clocks::PllCfg;

use cmsis_dsp_sys as dsp_sys;

use defmt::println;

cfg_if! {
    if #[cfg(feature = "mercury-g4")] {
        use stm32_hal2::usb::{Peripheral, UsbBus, UsbBusType};

        use usb_device::bus::UsbBusAllocator;
        use usb_device::prelude::*;
        use usbd_serial::{SerialPort, USB_CLASS_CDC};


        // Due to the way the USB serial lib is set up, the USB bus must have a static lifetime.
        // In practice, we only mutate it at initialization.
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
    }
}

#[cfg(feature = "mercury-h7")]
use crate::{
    clocks::VosRange,
    power::{SupplyConfig, VoltageLevel},
};

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32_hal2::dma::DmaInput;

mod control_interface;
mod drivers;
mod flight_ctrls;
mod osd;
mod pid;
mod pid_tuning;
mod protocols;
mod sensor_fusion;
mod setup;

// cfg_if! {
// if #[cfg(feature = "mercury-h7")] {
use drivers::baro_dps310 as baro;
use drivers::gps_x as gps;
use drivers::imu_icm426xx as imu;
// use drivers::imu_ism330dhcx as imu;
use drivers::tof_vl53l1 as tof;

// use protocols::{dshot, elrs};
use protocols::dshot;

use flight_ctrls::{
    ArmStatus, AutopilotStatus, CommandState, CtrlInputs, InputMap, InputMode, Params, RotorPower,
    POWER_LUT,
};

use pid::{CtrlCoeffGroup, PidDerivFilters, PidGroup};

// Our DT timer speed, in Hz.
const DT_TIM_FREQ: u32 = 200_000_000;

// 6 readings; 2 bytes each.
static mut IMU_READINGS: [u8; 6 * 2] = [0; 6 * 2];

// The rate our main program updates, in Hz.
// Currently set to
const IMU_UPDATE_RATE: f32 = 8_000.;

// todo: Set update rate attitude back to 1600 etc. Slower rate now since we're using this loop to TS.
const UPDATE_RATE_ATTITUDE: f32 = 2.; // IMU rate / 5.
const UPDATE_RATE_VELOCITY: f32 = 400.; // IMU rate / 20.

// How many inner loop ticks occur between mid and outer loop.
// const OUTER_LOOP_RATIO: usize = 10;

const DT_IMU: f32 = 1. / IMU_UPDATE_RATE;
const DT_ATTITUDE: f32 = 1. / UPDATE_RATE_ATTITUDE;
const DT_VELOCITY: f32 = 1. / UPDATE_RATE_VELOCITY;

// Max distance from curent location, to point, then base a
// direct-to point can be, in meters. A sanity check
// todo: Take into account flight time left.
const DIRECT_AUTOPILOT_MAX_RNG: f32 = 500.;

// We use `LOOP_I` to manage sequencing the velocity-update PID from within the attitude-update PID
// loop.
const VELOCITY_ATTITUDE_UPDATE_RATIO: usize = 4;
static LOOP_I: AtomicUsize = AtomicUsize::new(0);

// Enable this to print parameters (eg location, altitude, attitude, angular rates etc) to the console.
const DEBUG_PARAMS: bool = true;

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
/// ///
/// 3 level loop? S, v, angle?? Or just 2? (position cmds in outer loop)

/// Utility fn to make up for `core::cmp::max` requiring f32 to impl `Ord`, which it doesn't.
/// todo: Move elsewhere?
fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

fn abs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
}

pub enum LocationType {
    /// Lattitude and longitude. Available after a GPS fix
    LatLon,
    /// Start at 0, and count in meters away from it.
    Rel0,
}

impl Default for LocationType {
    fn default() -> Self {
        Self::Rel0
    }
}

/// If type is LatLon, `x` and `y` are in degrees. If Rel0, in meters. `z` is in m MSL.
#[derive(Default)]
pub struct Location {
    pub type_: LocationType,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Location {
    pub fn new(type_: LocationType, x: f32, y: f32, z: f32) -> Self {
        Self { type_, x, y, z }
    }
}

/// User-configurable settings
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
    /// The GPS module is connected
    gps_attached: bool,
    /// The time-of-flight sensor module is connected
    tof_attached: bool,
    /// It's common to arbitrarily wire motors to the ESC. Reverse each from its
    /// default direction, as required.
    motors_reversed: (bool, bool, bool, bool),
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
            gps_attached: false,
            tof_attached: false,
            motors_reversed: (false, false, false, false),
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

/// A quaternion. Used for attitude state
struct Quaternion {
    i: f32,
    j: f32,
    k: f32,
    l: f32,
}

// impl Sub for Quaternion {
//     type Output = Self;
//
//     fn sub(self, other: Self) -> Self::Output {
//         Self {
//             x: self.x - other.x,
//             y: self.y - other.y,
//             z: self.z - other.z,
//         }
//     }
// }

impl Quaternion {
    pub fn new(i: f32, j: f32, k: f32, l: f32) -> Self {
        Self { i, j, k, l }
    }
}

/// A generalized quaternion
struct RotorMath {}

/// Represents a complete quadcopter. Used for setting control parameters.
struct AircraftProperties {
    mass: f32,               // grams
    arm_len: f32,            // meters
    drag_coeff: f32,         // unitless
    thrust_coeff: f32,       // N/m^2
    moment_of_intertia: f32, // kg x m^2
    rotor_inertia: f32,      // kg x m^2
}

impl AircraftProperties {
    /// Calculate the power level required, applied to each rotor, to maintain level flight
    /// at a given MSL altitude. (Alt is in meters)
    pub fn level_pwr(&self, alt: f32) -> f32 {
        return 0.1; // todo
    }
}

/// Specify the rotor. Includdes methods that get information regarding timer and DMA, per
/// specific board setups.
#[derive(Clone, Copy)]
pub enum Rotor {
    R1,
    R2,
    R3,
    R4,
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

// todo temp!
use cortex_m::delay::Delay;

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;

    // todo: Move vars from here to `local` as required.
    #[shared]
    struct Shared {
        // profile: FlightProfile,
        user_cfg: UserCfg,
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
        manual_inputs: CtrlInputs,
        current_pwr: RotorPower,
        dma: Dma<DMA1>,
        spi2: Spi<SPI2>,
        cs_imu: Pin,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        // rtc: Rtc,
        update_timer: Timer<TIM15>,
        rotor_timer_a: Timer<TIM2>,
        rotor_timer_b: Timer<TIM3>,
        // todo: Figure out how to store usb_dev and usb_serial as resources. Getting errors atm.
        // usb_dev: UsbDevice<UsbBusType>,
        // usb_serial: SerialPort<UsbBusType>,
        // `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        // Store filter instances for the PID loop derivatives. One for each param used.
        pid_deriv_filters: PidDerivFilters,
        base_point: Location,
        command_state: CommandState,
    }

    #[local]
    struct Local {
        spi1: Spi<SPI1>,
        spi3: Spi<SPI3>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let mut cp = cx.core;
        // Set up microcontroller peripherals
        let mut dp = pac::Peripherals::take().unwrap();

        unsafe {
            (*pac::RCC::ptr())
                .ahb2enr
                .modify(|_, w| w.gpioaen().set_bit());
        }

        #[cfg(feature = "mercury-h7")]
        SupplyConfig::DirectSmps.setup(&mut dp.PWR, VoltageLevel::V2_5);

        // Set up clocks
        let clock_cfg = Clocks {
            // Config for 480Mhz full speed:
            #[cfg(feature = "mercury-h7")]
            pll_src: PllSrc::Hse(8_000_000),
            #[cfg(feature = "mercury-g4")]
            input_src: InputSrc::Pll(PllSrc::Hse(16_000_000)),
            // vos_range: VosRange::VOS0, // Note: This may use extra power. todo: Put back!
            #[cfg(feature = "mercury-h7")]
            pll1: PllCfg {
                divm: 4, // To compensate with 8Mhz HSE instead of 64Mhz HSI
                // divn: 480,// todo: Put back! No longer working??
                ..Default::default()
            },
            hsi48_on: true,
            clk48_src: Clk48Src::Hsi48,
            ..Default::default()
        };

        clock_cfg.setup().unwrap();

        // Enable the Clock Recovery System, which improves HSI48 accuracy.
        clocks::enable_crs(CrsSyncSrc::Usb);

        println!("Clocks setup successfully");

        // (We don't need the debug workaround, since we don't use low power modes.)
        // debug_workaround();

        // Improves performance, at a cost of slightly increased power use.
        cp.SCB.invalidate_icache();
        cp.SCB.enable_icache();
        // cp.SCB.clean_invalidate_dcache(); // todo?
        // cp.SCB.enable_dcache(); // todo?

        // Set up pins with appropriate modes.
        setup::setup_pins();

        // We use SPI1 for the IMU
        // SPI input clock is 400MHz for H7, and 172Mhz for G4. 400MHz / 32 = 12.5 MHz. 170Mhz / 8 = 21.25Mhz.
        // The limit is the max SPI speed of the ICM-42605 IMU of 24 MHz. The Limit for the St Inemo ISM330  is 10Mhz.
        // 426xx can use any SPI mode. Maybe St is only mode 3? Not sure.
        #[cfg(feature = "mercury-g4")]
        // let imu_baud_div = BaudRate::Div8;  // for ICM426xx, for 24Mhz limit
        // todo: 10 MHz max SPI frequency for intialisation? Found in BF; confirm in RM.
        let imu_baud_div = BaudRate::Div32; // 5.3125 Mhz, for ISM330 10Mhz limit
        let imu_baud_div = BaudRate::Div64; // todo: Lower clock speed to TS IMU not working.
        #[cfg(feature = "mercury-h7")]
        // let imu_baud_div = BaudRate::Div32; // for ICM426xx, for 24Mhz limit
        let imu_baud_div = BaudRate::Div64; // 6.25Mhz for ISM330 10Mhz limit

        let imu_spi_cfg = SpiConfig {
            // Per ICM42688 and ISM330 DSs, we need either mode 2 or mode 3, but I'm not sure which.
            // todo: Leaning mode 2.
            mode: SpiMode::mode3(),
            ..Default::default()
        };

        let mut spi1 = Spi::new(dp.SPI1, imu_spi_cfg, imu_baud_div);

        #[cfg(feature = "mercury-h7")]
        let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);
        #[cfg(feature = "mercury-g4")]
        let mut cs_imu = Pin::new(Port::B, 12, PinMode::Output);

        let mut delay = Delay::new(cp.SYST, clock_cfg.systick());
        delay.delay_ms(100);

        println!("Pre IMU setup");
        imu::setup(&mut spi1, &mut cs_imu, &mut delay);
        println!("IMU setup complete");

        // todo: Testing IMU

        loop {
            let readings = imu::read_all(&mut spi1, &mut cs_imu);
            println!(
                "Ax: {}, Ay: {}, Az: {}, pitch: {}, roll: {}, yaw: {}",
                readings.a_x,
                readings.a_y,
                readings.a_z,
                readings.v_pitch,
                readings.v_roll,
                readings.v_yaw
            );
            delay.delay_ms(500);
        }

        // todo IMU  test complete.

        // We use SPI2 for the LoRa ELRS chip.  // todo: Find max speed and supported modes.
        let spi2 = Spi::new(dp.SPI2, Default::default(), BaudRate::Div32);

        let elrs_dio = Pin::new(Port::C, 13, PinMode::Output); // todo: input or output?

        // We use SPI3 for flash. // todo: Find max speed and supported modes.
        let spi3 = Spi::new(dp.SPI3, Default::default(), BaudRate::Div32);

        let cs_flash = Pin::new(Port::C, 6, PinMode::Output);

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
        // baro::setup(&mut i2c2);
        // println!("Altimeter setup complete");
        // let pressure = baro::read(&mut i2c2);
        // println!("Pressure: {}", pressure);

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
        let uart1 = Usart::new(dp.USART1, 420_000, Default::default(), &clock_cfg);
        // let uart2 = Usart::new(dp.USART1, 9_600., Default::default(), &clock_cfg);

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

        let rotor_timer_cfg = TimerConfig {
            // We use ARPE since we change duty with the timer running.
            auto_reload_preload: true,
            // alignment: Alignment::Center1, // todo temp
            // direction: timer::CountDir::Down, // todo TS
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

        let mut dma = Dma::new(dp.DMA1);
        #[cfg(feature = "mercury-g4")]
        dma::enable_mux1();

        setup::setup_dma(&mut dma, &mut dp.DMAMUX);

        let mut user_cfg = UserCfg::default();

        dshot::setup_motor_dir(
            user_cfg.motors_reversed,
            &mut rotor_timer_a,
            &mut rotor_timer_b,
            &mut dma,
        );

        // We use `dt_timer` to count the time between IMU updates, for use in the PID loop
        // integral, derivative, and filters. If set to 1Mhz, the CNT value is the number of
        // µs elapsed.

        // let mut dt_timer = Timer::new_tim15(dp.TIM3, 1_., Default::default(), &clock_cfg);
        // We expect the IMU to update every 8kHz. Set 1kHz as the freq here, which is low,
        // by a good margin. Not too low, as to keep resolution high.
        // We use manual PSC and ARR, as to maximize resolution through a high ARR.
        // dt_timer.set_prescaler(DT_PSC);
        // dt_timer.set_auto_reload(DT_ARR);

        cfg_if! {
            if #[cfg(feature = "mercury-g4")] {
                let usb = Peripheral { regs: dp.USB };
                // let usb_bus = UsbBus::new(usb);

                unsafe { USB_BUS = Some(UsbBus::new(usb)) };

                let usb_serial = SerialPort::new(unsafe { &USB_BUS.as_ref().unwrap() });

                let usb_dev = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(0x16c0, 0x27dd))
                    .manufacturer("Anyleaf")
                    .product("Serial port")
                    // We use `serial_number` to identify the device to the PC. If it's too long,
                    // we get permissions errors on the PC.
                    .serial_number("Mercury-G4") // todo: Try 2 letter only if causing trouble?
                    .device_class(USB_CLASS_CDC)
                    .build();
            }
        }

        // Used to update the input data from the ELRS radio
        let mut elrs_dio = Pin::new(Port::C, 13, PinMode::Output); // todo: In or out?
        let mut elrs_cs = Pin::new(Port::C, 15, PinMode::Output);
        let mut elrs_busy = Pin::new(Port::C, 14, PinMode::Input);

        // todo: But back A/R.
        // elrs_busy.enable_interrupt(Edge::Falling); // todo: Rising or falling?

        // todo: DMA for voltage ADC (?)

        let mut flash = Flash::new(dp.FLASH); // todo temp mut to test

        let mut params = Default::default();

        // todo: Instead of a `Barometer` struct, perhaps store baro calibration elsewhere.
        baro::setup(&mut i2c2);

        let mut base_point = Location::default();
        init_sensors(&mut params, &mut base_point, &mut i2c1, &mut i2c2);

        update_timer.enable();

        println!("Entering main loop...");
        (
            // todo: Make these local as able.
            Shared {
                user_cfg,
                input_map: Default::default(),
                input_mode: InputMode::Attitude,
                autopilot_status: Default::default(),
                ctrl_coeffs: Default::default(),
                current_params: params,
                velocities_commanded: Default::default(),
                attitudes_commanded: Default::default(),
                rates_commanded: Default::default(),
                pid_velocity: Default::default(),
                pid_attitude: Default::default(),
                pid_rate: Default::default(),
                manual_inputs: Default::default(),
                current_pwr: Default::default(),
                dma,
                spi2,
                cs_imu,
                i2c1,
                i2c2,
                // rtc,
                update_timer,
                rotor_timer_a,
                rotor_timer_b,
                // todo: Put these back. If you have to, use mutex<refcell.
                // usb_dev,
                // usb_serial,
                power_used: 0.,
                pid_deriv_filters: PidDerivFilters::new(),
                base_point,
                command_state: Default::default(),
            },
            Local { spi1, spi3 },
            init::Monotonics(),
        )
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    #[task(binds = TIM4, shared = [current_params], priority = 1)]
    /// Used to periodically print things to the console.
    fn debug_timer_isr(mut cx: debug_timer_isr::Context) {
        // todo: Set up this timer.
        cx.shared.current_params.lock(|params| {
            defmt::println!(
                "Pitch rate: {}
                Roll rate: {}
                Yaw rate: {}",
                params.v_roll,
                params.v_pitch,
                params.v_yaw,
                // todo etc
            );
        });
    }

    // todo: Remove rotor timers, spi, and dma from this ISR; we only use it for tresting DSHOT
    #[task(
    binds = TIM1_BRK_TIM15,
    shared = [current_params, manual_inputs, input_map, current_pwr,
    velocities_commanded, attitudes_commanded, rates_commanded, pid_velocity, pid_attitude, pid_rate,
    pid_deriv_filters, power_used, input_mode, autopilot_status, user_cfg, command_state, ctrl_coeffs,
    dma, rotor_timer_a, rotor_timer_b
    ],
    priority = 2
    )]
    /// This runs periodically, on a ~1kHz timer. It's used to trigger the attitude and velocity PID loops, ie for
    /// sending commands to the attitude and rate PID loop based on things like autopilot, command-mode etc.
    fn update_isr(cx: update_isr::Context) {
        unsafe { (*pac::TIM15::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }

        // todo: Dshot test
        (
            cx.shared.rotor_timer_a,
            cx.shared.rotor_timer_b,
            cx.shared.dma,
        )
            .lock(|rotor_timer_a, rotor_timer_b, dma| {
                unsafe {
                    // println!("DMA ISR {}", dma.regs.isr.read().bits());

                    dshot::set_power_a(Rotor::R1, Rotor::R2, 0.8, 0.2, rotor_timer_a, dma);

                    dshot::set_power_b(Rotor::R3, Rotor::R4, 1., 0.3, rotor_timer_b, dma);
                }
            });
        return; // todo temp for test

        (
            cx.shared.current_params,
            cx.shared.manual_inputs,
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
            cx.shared.ctrl_coeffs,
        )
            .lock(
                |params,
                 manual_inputs,
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
                 coeffs| {
                    // todo: Support both UART telemetry from ESC, and analog current sense pin.
                    // todo: Read from an ADC or something, from teh ESC.
                    // let current_current = None;
                    // if let Some(current) = current_current {
                    // *power_used += current * DT;

                    // }
                    // else {
                    // *power_used += current_pwr.total() * DT;
                    // }

                    match input_mode {
                        InputMode::Acro => {
                            return;
                        }
                        _ => {
                            pid::run_attitude(
                                params,
                                manual_inputs,
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
                                        manual_inputs,
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

    /// Runs when new IMU data is recieved. This functions as our PID inner loop, and updates
    /// pitch and roll. We use this ISR with an interrupt from the IMU, since we wish to
    /// update rotor power settings as soon as data is available.
    #[task(binds = EXTI15_10, shared = [
    cs_imu, dma, rotor_timer_a, rotor_timer_b, command_state
    ], local = [spi1], priority = 4)]
    fn imu_data_isr(mut cx: imu_data_isr::Context) {
        println!("IMU DATA ISR");

        unsafe {
            // Clear the interrupt flag.
            #[cfg(feature = "mercury-h7")]
            (*pac::EXTI::ptr()).c1pr1.modify(|_, w| w.pr15().set_bit());
            #[cfg(feature = "mercury-g4")]
            (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pif15().set_bit());
        }

        (
            cx.shared.command_state,
            cx.shared.rotor_timer_a,
            cx.shared.rotor_timer_b,
            cx.shared.dma,
            cx.shared.cs_imu,
        )
            .lock(|state, rotor_timer_a, rotor_timer_b, dma, cs_imu| {
                if state.armed != ArmStatus::Armed {
                    dshot::stop_all(rotor_timer_a, rotor_timer_b, dma);
                }

                imu::read_all_dma(cx.local.spi1, cs_imu, dma);
            });
    }

    // todo SPI tx for testing DMA
    #[task(binds = DMA1_CH1, priority = 7)]
    fn test_isr(cx: test_isr::Context) {
        println!("DMA CH1 Interrupt!");
    }

    #[task(binds = DMA1_CH2, shared = [dma, current_params, input_mode, manual_inputs, input_map, autopilot_status,
    rates_commanded, pid_rate, pid_deriv_filters, current_pwr, ctrl_coeffs, command_state, cs_imu, user_cfg,
    rotor_timer_a, rotor_timer_b], local = [], priority = 5)]
    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it triggers the inner PID loop.
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        println!("DMA Ch2 transfer complete.");
        // Clear DMA interrupt this way due to RTIC conflict.
        unsafe { (*DMA1::ptr()).ifcr.write(|w| w.tcif2().set_bit()) }

        cx.shared.cs_imu.lock(|cs| {
            cs.set_high();
        });

        let imu_data = sensor_fusion::ImuReadings::from_buffer(unsafe { &IMU_READINGS });

        // todo: In this inner, ~8kHz update whwere we call sensor fusion? Or should we (perhaps with)
        // todo a lowpass filter applied, just set the angular rates here, and perform fusion/
        // todo attitude platform etc only in the `update` isr?
        let mut sensor_data_fused = sensor_fusion::estimate_attitude(&imu_data);

        // todo: Temp replacing back in imu data while we sort out the fusion.
        sensor_data_fused.v_pitch = imu_data.v_pitch;
        sensor_data_fused.v_roll = imu_data.v_roll;
        sensor_data_fused.v_yaw = imu_data.v_yaw;

        sensor_data_fused.a_x = imu_data.a_x;
        sensor_data_fused.a_y = imu_data.a_y;
        sensor_data_fused.a_z = imu_data.a_z;

        (
            cx.shared.current_params,
            cx.shared.manual_inputs,
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
        )
            .lock(
                |params,
                 manual_inputs,
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
                 input_map| {
                    *params = sensor_data_fused;

                    pid::run_rate(
                        params,
                        *input_mode,
                        autopilot_status,
                        cfg,
                        manual_inputs,
                        rates_commanded,
                        pid_inner,
                        filters,
                        current_pwr,
                        rotor_timer_a,
                        rotor_timer_b,
                        dma,
                        coeffs,
                        cfg.max_speed_ver,
                        input_map,
                        DT_IMU,
                    );
                },
            );
    }

    #[task(binds = EXTI3, shared = [user_cfg, manual_inputs], local = [spi3], priority = 4)]
    /// We use this ISR when receiving data from the radio, via ELRS
    fn radio_data_isr(mut cx: radio_data_isr::Context) {
        (cx.shared.user_cfg, cx.shared.manual_inputs).lock(|cfg, manual_inputs| {
            // *manual_inputs = elrs::get_inputs(cx.local.spi3);
            *manual_inputs = CtrlInputs::get_manual_inputs(cfg); // todo: this?
        })
    }

    // todo: Put this USB ISR back in once you've sorted your RTIC resource for usb_dev and usb_serial.
    // #[task(binds = USB_LP, shared = [usb_dev, usb_serial, current_params], local = [], priority = 3)]
    // /// This ISR handles interaction over the USB serial port, eg for configuring using a desktop
    // /// application.
    // fn usb_isr(mut cx: usb_isr::Context) {
    //     (cx.shared.usb_dev, cx.shared.usb_serial, cx.shared.params).lock(
    //         |usb_dev, usb_serial, params| {
    //             if !usb_dev.poll(&mut [usb_serial]) {
    //                 return;
    //             }
    //
    //             let mut buf = [0u8; 8];
    //             match usb_serial.read(&mut buf) {
    //                 // todo: match all start bits and end bits. Running into an error using the naive approach.
    //                 Ok(count) => {
    //                     usb_serial.write(&[1, 2, 3]).ok();
    //                 }
    //                 Err(_) => {
    //                     //...
    //                 }
    //             }
    //         },
    //     )
    // }

    // #[task(binds = TIM2, shared = [rotor_timer_a], priority = 3)]
    // /// We use this ISR to disable the DSHOT timer upon completion of a packet send.
    // fn ts_a(mut cx: ts_a::Context) {
    //     println!("Rotor 1 TO");
    //
    //     cx.shared.rotor_timer_a.lock(|timer| {
    //         unsafe {
    //             println!("Timer SR {}", timer.regs.sr.read().bits());
    //         }
    //
    //         timer.clear_interrupt(TimerInterrupt::Update);
    //     });
    // }

    // Note: This is actually Channel 3, but there's an issue with G4 streams being off
    // from their interrupt handler by 1.
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
            (*pac::GPIOA::ptr())
                .moder
                .modify(|_, w| w.moder0().bits(0b01));
            (*pac::GPIOA::ptr()).bsrr.write(|w| w.br0().set_bit());
            (*pac::GPIOA::ptr())
                .moder
                .modify(|_, w| w.moder1().bits(0b01));
            (*pac::GPIOA::ptr()).bsrr.write(|w| w.br1().set_bit());
        }
    }

    // See note about about DMA channels being off by 1.
    #[task(binds = DMA1_CH4, shared = [rotor_timer_b], priority = 6)]
    /// We use this ISR to disable the DSHOT timer upon completion of a packet send.
    fn dshot_isr_b(mut cx: dshot_isr_b::Context) {
        unsafe { (*DMA1::ptr()).ifcr.write(|w| w.tcif4().set_bit()) }

        cx.shared.rotor_timer_b.lock(|timer| {
            timer.disable();
        });

        // Set to Output pin, low.
        unsafe {
            (*pac::GPIOB::ptr())
                .moder
                .modify(|_, w| w.moder0().bits(0b01));
            (*pac::GPIOB::ptr()).bsrr.write(|w| w.br0().set_bit());
            (*pac::GPIOB::ptr())
                .moder
                .modify(|_, w| w.moder1().bits(0b01));
            (*pac::GPIOB::ptr()).bsrr.write(|w| w.br1().set_bit());
        }
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
