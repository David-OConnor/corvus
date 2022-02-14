#![no_main]
#![no_std]

// #![allow(non_ascii_idents)] // todo: May no longer be required

use core::{
    f32::consts::TAU,
    ops::Sub,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use cortex_m::{self, asm, delay::Delay};

use stm32_hal2::{
    self,
    adc::{Adc, AdcConfig, AdcDevice},
    clocks::{Clocks, InputSrc, PllCfg, PllSrc, VosRange},
    debug_workaround,
    dma::{self, Dma, DmaChannel, DmaInterrupt},
    flash::Flash,
    gpio::{self, Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, DMA1, I2C1, I2C2, SPI2, SPI4, TIM15, TIM2, TIM3, TIM5},
    power::{SupplyConfig, VoltageLevel},
    rtc::Rtc,
    spi::{BaudRate, Spi, SpiConfig},
    timer::{OutputCompare, TimChannel, Timer, TimerConfig, TimerInterrupt},
};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as sys;

use defmt_rtt as _; // global logger
use panic_probe as _;

// mod atmos_model;
mod baro_driver_dps310;
mod flight_ctrls;
mod imu_driver_icm42605;

// `tof_driver` uses partially-translated C code that doesn't conform to Rust naming conventions.
mod sensor_fusion;
mod osd_driver_max7456;

#[allow(non_snake_case)]
// mod tof_driver;

use baro_driver_dps310 as baro;
use imu_driver_icm42605 as imu;
use osd_driver_max7456 as osd;

use flight_ctrls::{
    AutopilotMode, CommandState,  CtrlCoeffGroup, CtrlConstraint, CtrlInputs, FlightCmd, InputMode, ParamType, Params,
    ParamsInst, PidDerivFilters, PidState, PidState2, PidStateGroup, RotorPower,
};

// Our DT timer speed, in Hz.
const DT_TIM_FREQ: u32 = 200_000_000;

// The frequency our motor-driving PWM operates at, in Hz.
// todo: Make this higher (eg 96kHz) after making sure the ESC
const PWM_FREQ: f32 = 12_000.;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately.
// These are set for a 200MHz timer frequency.
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period.
// Keep this in sync with `PWM_FREQ`!
const PWM_PSC: u32 = 0;
const PWM_ARR: u32 = 7_395;

// For 200Mhz, 32-bit timer:
// For CNT = us:
// PSC = (200Mhz / 1Mhz) - 1 = 199. ARR = (1<<32) - 1
const DT_PSC: u32 = 199;
const DT_ARR: u32 = u32::MAX - 1;

// The rate our main program updates, in Hz.
// Currently set to
const IMU_UPDATE_RATE: f32 = 8_000.;
const UPDATE_RATE: f32 = 1_600.; // IMU rate / 5.

// How many inner loop ticks occur between mid and outer loop.
const OUTER_LOOP_RATIO: usize = 10;

const DT_IMU: f32 = 1. / IMU_UPDATE_RATE;
const DT: f32 = 1. / UPDATE_RATE;

// Speed in meters per second commanded by full power.
// todo: This may not be what you want; could be unachievable, or maybe you really want
// full speed.
// const V_FULL_DEFLECTION: f32 = 20.;

// Max distance from curent location, to point, then base a
// direct-to point can be, in meters. A sanity check
// todo: Take into account flight time left.
const DIRECT_AUTOPILOT_MAX_RNG: f32 = 500.;

const IDLE_POWR: f32 = 0.01; // Eg when on the ground, and armed.

// We use `LOOP_I` to manage inner vs outer loops.
static LOOP_I: AtomicU32 = AtomicU32::new(0);

static ARMED: AtomicBool = AtomicBool::new(false);

// todo: Course set mode. Eg, point at thing using controls, activate a button,
// todo then the thing flies directly at the target.

// With this in mind: Store params in a global array. Maybe [ParamsInst; N], or maybe a numerical array for each param.

// todo: Consider a nested loop, where inner manages fine-tuning of angle, and outer
// manages directions etc. (?) look up prior art re quads and inner/outer loops.

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

/// A vector in 3 dimensions
struct Vector {
    x: f32,
    y: f32,
    z: f32,
}

impl Sub for Vector {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Vector {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

enum LocationType {
    /// Lattitude and longitude. Available after a GPS fix
    LatLon,
    /// Start at 0, and count in meters away from it.
    Rel0,
}

/// If type is LatLon, `x` and `y` are in degrees. If Rel0, in meters. `z` is in m MSL.
struct Location {
    type_: LocationType,
    x: f32,
    y: f32,
    z: f32,
}

impl Location {
    pub fn new(type_: LocationType, x: f32, y: f32, z: f32) -> Self {
        Self { type_, x, y, z }
    }
}

/// User-configurable settings
struct UserCfg {
    /// Set a ceiling the aircraft won't exceed. Defaults to 400' (Legal limit in US for drones).
    /// In meters.
    ceiling: f32,
    /// In Attitude and related control modes, max pitch angle (from straight up), ie
    /// full speed, without going horizontal or further.
    max_angle: f32,  // radians
    max_velocity: f32,  // m/s
    /// These input ranges map raw output from a manual controller to full scale range of our control scheme.
    /// (min, max). Set using an initial calibration / setup procedure.
    pitch_input_range: (f32, f32),
    roll_input_range: (f32, f32),
    yaw_input_range: (f32, f32),
    throttle_input_range: (f32, f32),
    /// Is the aircraft continuously collecting data on obstacles, and storing it to external flash?
    mapping_obstacles: bool,
}

impl Default for UserCfg {
    fn default() -> Self {
        Self {
            ceiling: 122.,
            max_angle: TAU * 0.22,
            max_velocity: 30., // todo: raise?
            // todo: Find apt value for these
            pitch_input_range: (0., 1.),
            roll_input_range: (0., 1.),
            yaw_input_range: (0., 1.),
            throttle_input_range: (0., 1.),
            mapping_obstacles: false,
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

/// Specify the rotor
#[derive(Clone, Copy)]
pub enum Rotor {
    R1,
    R2,
    R3,
    R4,
}

impl Rotor {
    pub fn tim_channel(&self) -> TimChannel {
        match self {
            Self::R1 => TimChannel::C1,
            Self::R2 => TimChannel::C2,
            Self::R3 => TimChannel::C3,
            Self::R4 => TimChannel::C4,
        }
    }
}

#[derive(Clone, Copy)]
/// Role in a swarm of drones
pub enum SwarmRole {
    Queen,
    Worker(u16), // id
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

/// Set up the pins that have structs that don't need to be accessed after.
pub fn setup_pins() {
    // SAI pins to accept input from the 4 PDM ICs, using SAI1, and 4, both blocks.
    // We use the same SCK and FS clocks for all 4 ICs.

    // For use with Matek H7 board:
    // http://www.mateksys.com/?portfolio=h743-slim#tab-id-7

    // todo: Determine what output speeds to use.

    // Connected to TIM3 CH3, 4; TIM5 CH1, 1; Matek
    let mut rotor1_pwm_ = Pin::new(Port::B, 0, PinMode::Alt(2));
    let mut rotor2_pwm_ = Pin::new(Port::B, 1, PinMode::Alt(2));
    let mut rotor3_pwm_ = Pin::new(Port::A, 0, PinMode::Alt(2));
    let mut rotor3_pwm_ = Pin::new(Port::A, 1, PinMode::Alt(2));

    // SPI4 for Matek IMU. AAnd/or
    let mosi4_ = Pin::new(Port::E, 14, PinMode::Alt(5));
    let miso4_ = Pin::new(Port::E, 13, PinMode::Alt(5));
    let sck4_ = Pin::new(Port::E, 12, PinMode::Alt(5));

    // SPI2 for Matek OSD (MAX7456?)
    let mosi4_ = Pin::new(Port::B, 15, PinMode::Alt(5));
    let miso4_ = Pin::new(Port::B, 14, PinMode::Alt(5));
    let sck4_ = Pin::new(Port::B, 13, PinMode::Alt(5));

    // Used to trigger a PID update based on new IMU data.
    let mut imu_interrupt = Pin::new(Port::E, 15, PinMode::Input);
    imu_interrupt.enable_interrupt(Edge::Falling); // todo: Rising or falling? Configurable on IMU I think.

    // I2C1 for Matek digital airspeed and compass
    let mut scl1 = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl1.output_type(OutputType::OpenDrain);
    scl1.pull(Pull::Up);

    let mut sda1 = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda1.output_type(OutputType::OpenDrain);
    sda1.pull(Pull::Up);

    // I2C2 for Matek barometer
    let mut scl2 = Pin::new(Port::B, 10, PinMode::Alt(4));
    scl2.output_type(OutputType::OpenDrain);
    scl2.pull(Pull::Up);

    let mut sda2 = Pin::new(Port::B, 11, PinMode::Alt(4));
    sda2.output_type(OutputType::OpenDrain);
    sda2.pull(Pull::Up);

    // todo: Use one of these buses for TOF sensor, or its own?

    let bat_adc_ = Pin::new(Port::C, 0, PinMode::Analog);
}

/// Run on startup, or when desired. Run on the ground. Gets an initial GPS fix,
/// and other initialization functions.
fn init(params: &mut Params, baro: Barometer, base_pt: &mut Location, i2c: &mut I2c<I2C1>) {
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

    if let Some(agl) = tof_driver::read(params.s_pitch, params.s_roll, i2c) {
        if agl > 0.01 {
            return;
        }
    }

    let fix = gps::get_fix(i2c);
    params.s_x = fix.lon;
    params.s_y = fix.lat;
    params.s_z_msl = fix.alt;

    *base_pt = Location::new(LocationType::LatLon, fix.lon, fix.lat, fix.alt);

    // todo: Use Rel0 location type if unable to get fix.

    let temp = 0.; // todo: Which sensor reads temp? The IMU?

    baro.calibrate(fix.alt, temp);
}

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;

    // todo: Move vars from here to `local` as required.
    #[shared]
    struct Shared {
        // profile: FlightProfile,
        user_cfg: UserCfg,
        input_mode: InputMode,
        autopilot_mode: AutopilotMode,
        ctrl_coeffs: CtrlCoeffGroup,
        // todo: current_params vs
        current_params: Params,
        // mid_flt_cmd: FlightCmd,
        inner_flt_cmd: FlightCmd,
        /// Proportional, Integral, Differential error
        pid_outer: PidStateGroup,
        pid_mid: PidStateGroup,
        pid_inner: PidStateGroup,
        manual_inputs: CtrlInputs,
        current_pwr: RotorPower,
        dma: Dma<DMA1>,
        spi2: Spi<SPI2>,
        spi4: Spi<SPI4>,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        // rtc: Rtc,
        update_timer: Timer<TIM15>,
        rotor_timer_a: Timer<TIM3>,
        rotor_timer_b: Timer<TIM5>,
        // `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        // Store filter instances for the PID loop derivatives. One for each param used.
        pid_deriv_filters: PidDerivFilters,
        base_point: Location,
        commands: CommandState,
    }

    #[local]
    struct Local {
        // dt_timer: Timer<TIM3>,
        // last_rtc_time: (u8, u8), // seconds, minutes
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let mut cp = cx.core;
        // Set up microcontroller peripherals
        let mut dp = pac::Peripherals::take().unwrap();

        let power = SupplyConfig::DirectSmps;
        power.setup(&mut dp.PWR, VoltageLevel::V2_5);

        // Set up clocks
        let clock_cfg = Clocks {
            // Config for 480Mhz full speed:
            pll_src: PllSrc::Hse(8_000_000),
            // vos_range: VosRange::VOS0, // Note: This may use extra power. todo: Put back!
            pll1: PllCfg {
                divm: 4, // To compensate with 8Mhz HSE instead of 64Mhz HSI
                // divn: 480,// todo: Put back! No longer working??
                ..Default::default()
            },
            ..Default::default()
        };

        clock_cfg.setup().unwrap();
        defmt::println!("Clocks setup successfully");
        debug_workaround();

        // Improves performance, at a cost of slightly increased power use.
        // May be required to prevent sound problems.
        cp.SCB.invalidate_icache();
        cp.SCB.enable_icache();

        // Set up pins with appropriate modes.
        setup_pins();

        // We use SPI for the IMU
        // SPI input clock is 400MHz. 400MHz / 32 = 12.5 MHz. The limit is the max SPI speed
        // of the ICM-42605 IMU of 24 MHz. This IMU can use any SPI mode, with the correct config on it.
        let mut spi = Spi::new(dp.SPI4, Default::default(), BaudRate::Div32);

        // We use I2C for the TOF sensor.(?) and for Matek digital airspeed and compass
        let i2c_cfg = I2cConfig {
            speed: I2cSpeed::Fast1M,
            // speed: I2cSpeed::Fast400k,
            ..Default::default()
        };
        let mut i2c1 = I2c::new(dp.I2C1, i2c_cfg.clone(), &clock_cfg);

        // We use (Matek) I2C2 for the barometer.
        let mut i2c2 = I2c::new(dp.I2C2, i2c_cfg, &clock_cfg);

        // We use the RTC to assist with power use measurement.
        let rtc = Rtc::new(dp.RTC, Default::default());

        // We use the ADC to measure battery voltage.
        let batt_adc = Adc::new_adc1(dp.ADC1, AdcDevice::One, Default::default(), &clock_cfg);

        // todo: Do we want these to be a single timer, with 4 channels?

        let mut update_timer =
            Timer::new_tim15(dp.TIM15, UPDATE_RATE, Default::default(), &clock_cfg);
        update_timer.enable_interrupt(TimerInterrupt::Update);

        let rotor_timer_cfg = TimerConfig {
            // We use ARPE since we change duty with the timer running.
            auto_reload_preload: true,
            ..Default::default()
        };

        // Timer that periodically triggers the noise-cancelling filter to update its coefficients.

        // todo: Why does the Matek board use 2 separate rotor timers, when each has 4 channels.
        let mut rotor_timer_a = Timer::new_tim2(dp.TIM3, PWM_FREQ, rotor_timer_cfg.clone(), &clock_cfg);

        rotor_timer_a.enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.);
        rotor_timer_a.enable_pwm_output(TimChannel::C2, OutputCompare::Pwm1, 0.);
        rotor_timer_a.enable_pwm_output(TimChannel::C3, OutputCompare::Pwm1, 0.);
        rotor_timer_a.enable_pwm_output(TimChannel::C4, OutputCompare::Pwm1, 0.);

        let mut rotor_timer_b = Timer::new_tim2(dp.TIM5, PWM_FREQ, rotor_timer_cfg, &clock_cfg);

        rotor_timer_b.enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.);
        rotor_timer_b.enable_pwm_output(TimChannel::C2, OutputCompare::Pwm1, 0.);
        rotor_timer_b.enable_pwm_output(TimChannel::C3, OutputCompare::Pwm1, 0.);
        rotor_timer_b.enable_pwm_output(TimChannel::C4, OutputCompare::Pwm1, 0.);

        // We use `dt_timer` to count the time between IMU updates, for use in the PID loop
        // integral, derivative, and filters. If set to 1Mhz, the CNT value is the number of
        // µs elapsed.

        // let mut dt_timer = Timer::new_tim15(dp.TIM3, 1_., Default::default(), &clock_cfg);
        // We expect the IMU to update every 8kHz. Set 1kHz as the freq here, which is low,
        // by a good margin. Not too low, as to keep resolution high.
        // We use manual PSC and ARR, as to maximize resolution through a high ARR.
        // dt_timer.set_prescaler(DT_PSC);
        // dt_timer.set_auto_reload(DT_ARR);

        // From Matek
        let mut imu_cs = Pin::new(Port::E, 11, PinMode::Output);
        imu::setup(&mut spi, &mut imu_cs);

        let mut osd_cs = Pin::new(Port::B, 12, PinMode::Output);

        let mut dma = Dma::new(dp.DMA1);

        // todo: COnsider how you use DMA, and bus splitting.
        // IMU
        dma::mux(DmaChannel::C0, dma::DmaInput::SPI1, &mut dp.DMAMUX1);
        // TOF sensor
        dma::mux(DmaChannel::C1, dma::DmaInput::I2C1, &mut dp.DMAMUX1);
        // Baro
        dma::mux(DmaChannel::C2, dma::DmaInput::I2C2, &mut dp.DMAMUX1);

        let mut flash = Flash::new(dp.FLASH); // todo temp mut to test

        rotor_timer.enable();
        update_timer.enable();
        // dt_timer.enable();

        (
            // todo: Make these local as able.
            Shared {
                user_cfg: Default::default(),
                input_mode: InputMode::Angle,
                autopilot_mode: AutopilotMode::None,
                ctrl_coeffs: Default::default(),
                current_params: Default::default(),
                // mid_flt_cmd: Default::default(),
                inner_flt_cmd: Default::default(),
                pid_outer: Default::default(),
                pid_mid: Default::default(),
                pid_inner: Default::default(),
                manual_inputs: Default::default(),
                current_pwr: Default::default(),
                dma,
                spi2,
                spi4,
                i2c1,
                i2c2,
                // rtc,
                update_timer,
                rotor_timer_a,
                rotor_timer_b,
                power_used: 0.,
                pid_deriv_filters: PidDerivFilters::new(),
                base_point: Location::new(LocationType::Rel0, 0., 0., 0.),
                commands: Default::default(),
            },
            Local {
                // dt_timer,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [], local = [])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    #[task(
    binds = TIM15,
    shared = [current_params, manual_inputs, current_pwr,
    inner_flt_cmd, pid_outer, pid_mid, pid_inner, pid_error_outer, pid_deriv_filters,
    power_used, input_mode, autopilot_mode, user_cfg, commands
    ],
    local = [],
    priority = 2
    )]
    fn update_isr(cx: update_isr::Context) {
        // todo: Loop index is to dtermine if we need to run the outer (position-tier)
        // todo loop.
        let loop_i = LOOP_I.fetch_add(1, Ordering::Relaxed);

        (
            cx.shared.current_params,
            cx.shared.manual_inputs,
            cx.shared.current_pwr,
            cx.shared.mid_flt_cmd,
            cx.shared.inner_flt_cmd,
            cx.shared.pid_outer,
            cx.shared.pid_mid,
            cx.shared.pid_inner,
            cx.shared.spi,
            cx.shared.power_used,
            cx.shared.input_mode,
            cx.shared.autopilot_mode,
            cx.shared.cfg,
            cx.shared.commands,

        )
            .lock(
                |params,
                 inputs,
                 current_pwr,
                 mid_flt_cmd,
                 inner_flt_cmd,
                 pid_outer,
                 pid_mid,
                 pid_inner,
                 power_used,
                 input_mode,
                 autopilot_mode,
                 cfg,
                 commands| {

                    *power_used += current_pwr.total() * DT;

                    // todo: Placeholder for sensor inputs/fusion.

                    if loop_i % OUTER_LOOP_RATIO == 0 {
                        match input_mode {
                            InputMode::Command => {
                                // let flt_cmd = FlightCmd::from_inputs(inputs, input_mode);

                                // todo: impl

                                *mid_flt_cmd = FlightCmd {
                                    y_pitch: Some((CtrlConstraint::Xy, ParamType::S, inputs.pitch)),
                                    x_roll: Some((CtrlConstraint::Xy, ParamType::S, inputs.roll)),
                                    // throttle could control altitude set point; make sure the throttle input
                                    // in this case is altitude.
                                    thrust: Some((ParamType::S, inputs.pitch)),
                                    // todo: S or V for yaw? Subjective. Go with existing drone schemes?
                                    yaw: Some((ParamType::V, inputs.yaw)),
                                };
                            }
                            _ => ()
                        }
                    }

                    flight_ctrls::run_pid_mid(
                        params,
                        inputs,
                        mid_flt_cmd,
                        inner_flt_cmd,
                        pid_mid,
                        pid_inner,
                        input_mode,
                        autopilot_mode,
                        cfg,
                        commands
                    );
                })


    }
}

/// Runs when new IMU data is recieved. This functions as our PID inner loop, and updates
/// pitch and roll. We use this ISR with an interrupt from the IMU, since we wish to
/// update rotor power settings as soon as data is available.
#[task(binds=EXTI9_15, shared=[params, inner_flt_cmd, pid_inner, pid_deriv_filters, current_pwr,
rotor_timer_a, rotor_timer_b], local=[], priority = 2)]
fn imu_data_isr(cx: imu_data_isr::Context) {
    unsafe {
        // Clear the interrupt flag.
        (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr1().set_bit());
    }

    // let timer_val = cx.local.dt_timer.read_count();
    // cx.local.dt_timer.disable();
    // cx.local.dt_timer.reset_countdown();
    // cx.local.tim_timer.enable();

    // the DT timer is set to tick at 1Mhz. This means each tick is a microsecond. We want
    // seconds.
    // todo: Can we get away with a fixed rate, like 8kHz?
    // todo: If you use this
    let dt = IMU_UPDATE_RATE;
    // let dt = timer_val / 1_000_000;

    // todo: Don't just use IMU; use sensor fusion. Starting with this by default
    let imu_data = imu::read_all(spi);
    let sensor_data_fused = sensor_fusion::estimate_attitude(imu_data);

    // todo: Move these into a `Params` method?
    // We have acceleration data for x, y, z: Integrate to get velocity and position.
    let v_x = sensor_data_fused.v_x + sensor_data_fused.a_x * DT;
    let v_y = sensor_data_fused.v_y + sensor_data_fused.a_y * DT;
    let v_z = sensor_data_fused.v_z + sensor_data_fused.a_z * DT;

    let s_x = sensor_data_fused.s_x + v_x * DT;
    let s_y = sensor_data_fused.s_y + v_y * DT;
    let s_z_msl = sensor_data_fused.s_z_msl + v_z * DT;
    let s_z_agl = sensor_data_fused.s_z_agl + v_z * DT;

    // We have position data for pitch, roll, yaw: Take derivative to get velocity and acceleration
    let s_pitch = sensor_data_fused.s_pitch + sensor_data_fused.v_pitch * DT;
    let s_roll = sensor_data_fused.s_roll + sensor_data_fused.v_roll * DT;
    let s_yaw = sensor_data_fused.s_yaw + sensor_data_fused.v_yaw * DT;

    let a_pitch = (v_pitch - sensor_data_fused.v_pitch) / DT;
    let a_roll = (v_roll - sensor_data_fused.v_roll) * DT;
    let a_yaw = (v_yaw - sensor_data_fused.v_yaw) / DT;

    cx.shared.params.lock(|params| {
        *params = Params {
            s_x,
            s_y,
            s_z_msl,
            s_z_agl,

            s_pitch,
            s_roll,
            s_yaw,

            v_x,
            v_y,
            v_z,
            v_pitch: sensor_data_fused.v_pitch,
            v_roll: sensor_data_fused.v_roll,
            v_yaw: sensor_data_fused.v_yaw,

            a_x: sensor_data_fused.a_x,
            a_y: sensor_data_fused.a_y,
            a_z: sensor_data_fused.a_z,
            a_pitch,
            a_roll,
            a_yaw,
        };
    })

        (
            cx.shared.params,
            cx.shared.inner_flt_cmd,
            cx.shared.pid_inner,
            cx.shared.pid_deriv_filters,
            cx.shared.rotor_timer_a,
            cx.shared.rotor_timer_b,
        )
        .lock(|params, inner_flt_cmd, pid_inner, filters, current_pwr,
               rotor_timer_a, rotor_timer_b| {
            flight_ctrls::run_pid_inner(
                params, inner_flt_cmd, pid_inner, filters, current_pwr,
                rotor_timer_a, rotor_timer_b, dt
            );
        })
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
