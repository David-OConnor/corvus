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
    clocks::{Clocks, InputSrc, PllCfg, PllSrc, VosRange},
    debug_workaround,
    dma::{self, Dma, DmaChannel, DmaInterrupt},
    flash::Flash,
    gpio::{self, Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::{I2c, I2cConfig, I2cSpeed},
    pac::{self, DMA1, I2C1, SPI1, TIM15, TIM2},
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
#[allow(non_snake_case)]
// mod tof_driver;

use baro_driver_dps310 as baro;
use imu_driver_icm42605 as imu;

use flight_ctrls::{
    AutopilotMode, CtrlCoeffs, CtrlConstraint, CtrlInputs, FlightCmd, InputMode, ParamType, Params,
    ParamsInst, PidDerivFilters, PidState, PidState2, PidStateGroup, RotorPower,
};

// The frequency our motor-driving PWM operates at, in Hz.
// todo: Make this higher (eg 96kHz) after making sure the ESC
const PWM_FREQ: f32 = 12_000.;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately.
// These are set for a 200MHz timer frequency.
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period.
// Keep this in sync with `PWM_FREQ`!
const PWM_PSC: u32 = 20; // todo set properly
const PWM_ARR: u32 = 7_395; // todo set properly

// The rate our main program updates, in Hz.
const UPDATE_RATE: f32 = 1_000.;

// todo: put CEILING in a user-customizable cfg struct

const CEILING: f32 = 122.; // meters

// How many inner loop ticks occur between outer loop ones.
// const OUTER_LOOP_RATIO: usize = 20;

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
    max_angle: f32,
    /// These input ranges map raw output from a manual controller to full scale range of our control scheme.
    /// (min, max). Set using an initial calibration / setup procedure.
    pitch_input_range: (f32, f32),
    roll_input_range: (f32, f32),
    yaw_input_range: (f32, f32),
    throttle_input_range: (f32, f32),
}

impl Default for UserCfg {
    fn default() -> Self {
        Self {
            ceiling: 122.,
            max_angle: TAU * 0.22,
            // todo: Find apt value for these
            pitch_input_range: (0., 1.),
            roll_input_range: (0., 1.),
            yaw_input_range: (0., 1.),
            throttle_input_range: (0., 1.),
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

    // todo: Determine what output speeds to use.

    let mut rotor1_pwm = Pin::new(Port::A, 0, PinMode::Alt(0));
    let mut rotor1_pwm = Pin::new(Port::A, 0, PinMode::Alt(0));
    let mut rotor1_pwm = Pin::new(Port::A, 0, PinMode::Alt(0));
    let mut rotor1_pwm = Pin::new(Port::A, 0, PinMode::Alt(0));

    let mosi = Pin::new(Port::A, 0, PinMode::Alt(0));
    let miso = Pin::new(Port::A, 0, PinMode::Alt(0));
    let sck = Pin::new(Port::A, 0, PinMode::Alt(0));
    let imu1_cs = Pin::new(Port::A, 0, PinMode::Alt(0));

    // I2C pins for the TOF sensor.
    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);
    scl.pull(Pull::Up);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);
    sda.pull(Pull::Up);

    // Used to trigger a PID update based on new IMU data.
    let imu_interrupt = Pin::new(Port::B, 1, PinMode::Input);

    // Temp pin for initial TS.
    let mut debug_killswitch = Pin::new(Port::B, 0, PinMode::Input);
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
        ctrl_coeffs: CtrlCoeffs,
        // todo: current_params vs
        current_params: Params,
        inner_flt_cmd: FlightCmd,
        /// Proportional, Integral, Differential error
        // todo: Re-think how you store and manage PID error.
        // pid_error_s: PidState,
        // pid_error_v: PidState,
        // pid_error_a: PidState,
        pid_error_inner: PidStateGroup,
        pid_error_outer: PidStateGroup,
        manual_inputs: CtrlInputs,
        current_pwr: RotorPower,
        dma: Dma<DMA1>,
        spi: Spi<SPI1>,
        update_timer: Timer<TIM15>,
        rotor_timer: Timer<TIM2>,
        // `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        // Store filter instances for the PID loop derivatives. One for each param used.
        pid_deriv_filters: PidDerivFilters,
        base_point: Location,
    }

    #[local]
    struct Local {}

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
        // of the IMU of __ MHz.
        let mut spi = Spi::new(dp.SPI1, Default::default(), BaudRate::Div32);

        // We use I2C for the TOF sensor.
        let i2c_cfg = I2cConfig {
            speed: I2cSpeed::Fast1M,
            // speed: I2cSpeed::Fast400k,
            ..Default::default()
        };
        let mut i2c = I2c::new(dp.I2C1, i2c_cfg, &clock_cfg);

        // todo: Do we want these to be a single timer, with 4 channels?

        let timer_cfg = TimerConfig {
            // We use ARPE since we change duty with the timer running.
            auto_reload_preload: true,
            ..Default::default()
        };

        let mut update_timer =
            Timer::new_tim15(dp.TIM15, UPDATE_RATE, Default::default(), &clock_cfg);
        update_timer.enable_interrupt(TimerInterrupt::Update);

        // Timer that periodically triggers the noise-cancelling filter to update its coefficients.
        let mut rotor_timer = Timer::new_tim2(dp.TIM2, PWM_FREQ, Default::default(), &clock_cfg);

        rotor_timer.enable_pwm_output(TimChannel::C1, OutputCompare::Pwm1, 0.);
        rotor_timer.enable_pwm_output(TimChannel::C2, OutputCompare::Pwm1, 0.);
        rotor_timer.enable_pwm_output(TimChannel::C3, OutputCompare::Pwm1, 0.);
        rotor_timer.enable_pwm_output(TimChannel::C4, OutputCompare::Pwm1, 0.);

        // We use the RTC to assist with power use measurement.
        let rtc = Rtc::new(dp.RTC, Default::default());

        let imu_cs = Pin::new(Port::A, 0, PinMode::Alt(0));

        let mut dma = Dma::new(dp.DMA1);

        // todo: COnsider how you use DMA, and bus splitting.
        // IMU
        dma::mux(DmaChannel::C0, dma::DmaInput::SPI1, &mut dp.DMAMUX1);
        // TOF sensor
        dma::mux(DmaChannel::C1, dma::DmaInput::I2C1, &mut dp.DMAMUX1);
        // Baro
        dma::mux(DmaChannel::C2, dma::DmaInput::I2C2, &mut dp.DMAMUX1);

        let mut flash = Flash::new(dp.FLASH); // todo temp mut to test

        (
            // todo: Make these local as able.
            Shared {
                user_cfg: Default::default(),
                input_mode: InputMode::Angle,
                autopilot_mode: AutopilotMode::None,
                ctrl_coeffs: Default::default(),
                current_params: Default::default(),
                inner_flt_cmd: Default::default(),
                // todo: Probably use a struct for PID error, with the 3 fields, as applicable
                // pid_error_s: Default::default(),
                // pid_error_v: Default::default(),
                // pid_error_a: Default::default(),
                pid_error_inner: Default::default(),
                pid_error_outer: Default::default(),
                manual_inputs: Default::default(),
                current_pwr: Default::default(),
                dma,
                spi,
                update_timer,
                rotor_timer,
                power_used: 0.,
                pid_deriv_filters: PidDerivFilters::new(),
                base_point: Location::new(LocationType::Rel0, 0., 0., 0.),
            },
            Local {},
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
    shared = [current_params, manual_inputs, rotor_timer, current_pwr, pid_error_inner,
    pid_error_outer, spi, power_used, input_mode, autopilot_mode, inner_flt_cmd, pid_deriv_filters],
    local = [],
    priority = 2
    )]
    fn update_isr(cx: update_isr::Context) {
        // let update_isr::SharedResources { params, inputs, rotor_timer, current_pwr, pid_error_v, pid_error_s, spi, power_used, input_mode } = cx.shared;

        (
            cx.shared.current_params,
            cx.shared.manual_inputs,
            cx.shared.rotor_timer,
            cx.shared.current_pwr,
            // cx.shared.pid_error_v,
            // cx.shared.pid_error_s,
            cx.shared.pid_error_inner,
            cx.shared.pid_error_outer,
            cx.shared.spi,
            cx.shared.power_used,
            cx.shared.input_mode,
            cx.shared.autopilot_mode,
            cx.shared.inner_flt_cmd,
        )
            .lock(
                |params,
                 inputs,
                 rotor_timer,
                 current_pwr,
                 pid_error_inner,
                 pid_error_outer,
                 // pid_error_v,
                 // pid_error_s,
                 spi,
                 power_used,
                 input_mode,
                 autopilot_mode,
                 inner_flt_cmd| {
                    // todo: Placeholder for sensor inputs/fusion.

                    // todo: Loop index is to dtermine if we need to run outer. Currently,
                    // todo, this ISR only runs outer, and inner is run on update from IMU.
                    // let loop_i = LOOP_I.fetch_add(1, Ordering::Relaxed);
                    *power_used += current_pwr.total() * DT;

                    if gpio::is_high(Port::B, 0) {
                        // Deadman switch; temporary for debugging.
                        return;
                    }

                    // todo: DRY. Maybe you just need to use the match to set flight_cmd?
                    match autopilot_mode {
                        AutopilotMode::None => {
                            // Determine inputs to appclassly
                            // let mut flight_cmd = FlightCmd::default();
                            // let mut flight_cmd = FlightCmd::level();
                            // flight_cmd.add_inputs(inputs);

                            // todo: Over contraint, ie matching flight mode here, and in `from_inputs`.
                            let mut flt_cmd = FlightCmd::from_inputs(inputs, input_mode);

                            match input_mode {
                                InputMode::Acro => {
                                    // In rate mode, simply update the inner command; don't do anything
                                    // in the outer PID loop.
                                    inner_flt_cmd = flt_cmd;
                                }

                                InputMode::Attitude => {
                                    // In attitude, our outer loop (eg controls specifying pitch and roll
                                    // position) determines how to set our inner loop, which sets the pitch
                                    // and roll. The throttle command sets altitude. Yaw behaves the
                                    // same as in Rate mode.

                                    // Note that a naive approach would be to do nothing here, as in
                                    // Rate mode, and let the inner loop manage pitch instead of pitch rate.
                                    // Here, we compensate for wind etc, but this will still drift,
                                    // since we don't attempt to maintain a position.

                                    // todo: For now, keep the naive approach. Test xy velocity-based
                                    // todo approaches later, eg after this is working.
                                    inner_flt_cmd = flt_cmd;
                                    //
                                    // pid_error_outer.pitch = flight_ctrls::calc_pid_error(
                                    //     inner_flight_cmd,
                                    //     params.v_x,
                                    //     pid_error_outer.pitch,
                                    //     coeffs,
                                    //     filters.s_pitch,
                                    //     DT
                                    // );
                                }
                                InputMode::Loiter => {
                                    // todo: This is a cheap version using V. It will drift!
                                    // todo: A candidate for attitude mode as well. Here, you might
                                    // todo need a third outer loop that handles position??

                                    // todo: THis is actually overwriting the x_s etc based approach in
                                    // todo the input parser. YOu need to find a way to make this cleaner.
                                    // inner_flt_cmd.x_roll = Some((CtrlConstraint::Xy, ParamType::V, inputs.roll));
                                    // inner_flt_cmd.y_pitch = Some((CtrlConstraint::Xy, ParamType::V, inputs.pitch));

                                    inner_flt_cmd.x_roll = Some((
                                        CtrlConstraint::PitchRoll,
                                        ParamType::S,
                                        inputs.roll,
                                    ));
                                    inner_flt_cmd.y_pitch = Some((
                                        CtrlConstraint::PitchRoll,
                                        ParamType::S,
                                        inputs.pitch,
                                    ));

                                    pid_error_outer.pitch = flight_ctrls::calc_pid_error(
                                        inner_flight_cmd,
                                        params.v_y,
                                        pid_error_outer.pitch,
                                        coeffs,
                                        filters.s_pitch,
                                        DT,
                                    );

                                    pid_error_outer.roll = flight_ctrls::calc_pid_error(
                                        inner_flight_cmd,
                                        params.v_x,
                                        pid_error_outer.pitch,
                                        coeffs,
                                        filters.s_pitch,
                                        DT,
                                    );
                                }
                            }
                        }
                        AutopilotMode::Takeoff => {
                            let flight_cmd = FlightCmd {
                                y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                                x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                                yaw: Some((ParamType::V, 0.)),
                                z: Some((
                                    ParamType::V,
                                    flight_ctrls::takeoff_speed(params.s_z_agl),
                                )),
                            };
                        }
                        AutopilotMode::Land => {
                            let flight_cmd = FlightCmd {
                                y_pitch: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                                x_roll: Some((CtrlConstraint::PitchRoll, ParamType::S, 0.)),
                                yaw: Some((ParamType::V, 0.)),
                                z: Some((
                                    ParamType::V,
                                    -flight_ctrls::landing_speed(params.s_z_agl),
                                )),
                            };
                        }
                        _ => (),
                    }
                },
            )
    }

    /// Runs when new IMU data is recieved. This functions as our PID inner loop, and updates
    /// pitch and roll. We use this ISR with an interrupt from the IMU, since we wish to
    /// update rotor power settings as soon as data is available.
    #[task(binds=EXTI1, shared=[params, inner_flt_cmd, pid_error_inner, pid_deriv_filters], local=[], priority = 2)]
    fn imu_data_isr(cx: imu_data_isr::Context) {
        unsafe {
            // Clear the interrupt flag.
            (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr1().set_bit());
        }

        // todo: Calculate dt!
        let dt = 0.000125; // todo temp at 8kHz. Use a subtraction from RTC etc.

        (
            cx.shared.params,
            cx.shared.inner_flt_cmd,
            cx.shared.pid_error_inner,
            cx.shared.pid_deriv_filters,
        )
            .lock(|params, inner_flt_cmd, pid_error_inner, filters| {
                // todo: Don't just use IMU; use sensor fusion. Starting with this by default
                let imu_data = imu::read_all(spi);

                // todo: Move these into a `Params` method?
                // We have acceleration data for x, y, z: Integrate to get velocity and position.
                let v_x = params.v_x + imu_data.a_x * DT;
                let v_y = params.v_y + imu_data.a_y * DT;
                let v_z = params.v_z + imu_data.a_z * DT;

                let s_x = params.s_x + v_x * DT;
                let s_y = params.s_y + v_y * DT;
                let s_z_msl = params.s_z_msl + v_z * DT;
                let s_z_agl = params.s_z_agl + v_z * DT;

                // We have position data for pitch, roll, yaw: Take derivative to get velocity and acceleration
                let s_pitch = params.s_pitch + imu_data.v_pitch * DT;
                let s_roll = params.s_roll + imu_data.v_roll * DT;
                let s_yaw = params.s_yaw + imu_data.v_yaw * DT;

                let a_pitch = (v_pitch - params.v_pitch) / DT;
                let a_roll = (v_roll - params.v_roll) * DT;
                let a_yaw = (v_yaw - params.v_yaw) / DT;

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
                    v_pitch: imu_data.v_pitch,
                    v_roll: imu_data.v_roll,
                    v_yaw: imu_data.v_yaw,

                    a_x: imu_data.a_x,
                    a_y: imu_data.a_y,
                    a_z: imu_data.a_z,
                    a_pitch,
                    a_roll,
                    a_yaw,
                };

                // // todo: This is a bit overdefined, since `inner_flt_cmd` already does a check
                // // todo for Rate, and uses the approp param.
                // // todo related: We're scoffing the other details provided by flight command with this approach.
                // let (cmd_pitch, param_pitch, cmd_roll, param_roll) = match input_mode {
                //     InputMode::Acro => (
                //         inner_flt_cmd.y_pitch.unwrap().2,
                //         params.v_pitch,
                //         inner_flt_cmd.x_roll.unwrap().2,
                //         params.v_roll,
                //     ),
                //     _ => (
                //         inner_flt_cmd.y_pitch.unwrap().2,
                //         params.s_pitch,
                //         inner_flt_cmd.x_roll.unwrap().2,
                //         params.s_roll,
                //     ),
                // };

                // todo: DRY

                match inner_flt_cmd.y_pitch {
                    Some(cmd) => {
                        let param = match cmd.1 {
                            ParamType::S => params.s_pitch,
                            ParamType::V => params.v_pitch,
                            ParamType::A => params.a_pitch,
                        };

                        pid_error_inner.pitch = flight_ctrls::calc_pid_error(
                            cmd.2,
                            param,
                            pid_error_inner.pitch,
                            coeffs,
                            filters.inner_pitch,
                            dt,
                        );
                    }
                    None => {
                        pid_error_inner.pitch = 0.;
                        // todo. Consider how to handle this re not screwing up DT and the PID.
                        // todo: You don't want to update the controls for this aspect (eg pitch) though.
                    }
                }

                match inner_flt_cmd.y_roll {
                    Some(cmd) => {
                        let param = match cmd.1 {
                            ParamType::S => params.s_roll,
                            ParamType::V => params.v_roll,
                            ParamType::A => params.a_roll,
                        };

                        pid_error_inner.roll = flight_ctrls::calc_pid_error(
                            cmd.2,
                            param,
                            pid_error_inner.roll,
                            coeffs,
                            filters.inner_roll,
                            dt,
                        );
                    }
                    None => {
                        pid_error_inner.roll = 0.;
                    }
                }

                match inner_flt_cmd.y_yaw {
                    Some(cmd) => {
                        let param = match cmd.0 {
                            ParamType::S => params.s_yaw,
                            ParamType::V => params.v_yaw,
                            ParamType::A => params.a_yaw,
                        };

                        pid_error_inner.yaw = flight_ctrls::calc_pid_error(
                            cmd.1,
                            param,
                            pid_error_inner.yaw,
                            coeffs,
                            filters.inner_yaw,
                            dt,
                        );
                    }
                    None => {
                        pid_error_inner.yaw = 0.;
                    }
                }

                match inner_flt_cmd.y_z {
                    Some(cmd) => {
                        let param = match cmd.0 {
                            ParamType::S => params.s_z,
                            ParamType::V => params.v_z,
                            ParamType::A => params.a_z,
                        };

                        pid_error_inner.z = flight_ctrls::calc_pid_error(
                            cmd.1,
                            param,
                            pid_error_inner.z,
                            coeffs,
                            filters.inner_z,
                            dt,
                        );
                    }
                    None => {
                        pid_error_inner.z = 0.;
                    }
                }

                //
                //
                // pid_error_inner.roll = flight_ctrls::calc_pid_error(
                //     cmd_roll,
                //     param_roll,
                //     pid_error_inner.roll,
                //     coeffs,
                //     filters.inner_roll,
                //     dt,
                // );
                //
                // pid_error_inner.yaw = flight_ctrls::calc_pid_error(
                //     inner_flt_cmd.yaw.unwrap().1,
                //     params.v_yaw,
                //     pid_error_inner.yaw,
                //     coeffs,
                //     filters.inner_yaw,
                //     dt,
                // );
                //
                // pid_error_inner.z = flight_ctrls::calc_pid_error(
                //     inner_flt_cmd.z.unwrap().1,
                //     params.v_z,
                //     pid_error_inner.z,
                //     coeffs,
                //     filters.inner_z,
                //     dt,
                // );

                flight_ctrls::adjust_ctrls(
                    pid_error_inner.pitch,
                    pid_error_inner.roll,
                    pid_error_inner.yaw,
                    pid_error_inner.z,
                    current_pwr,
                    rotor_timer,
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
