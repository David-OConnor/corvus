#![no_main]
#![no_std]

// #![allow(non_ascii_idents)] // todo: May no longer be required

use core::{
    f32::consts::TAU,
    ops::Sub,
    sync::atomic::{AtomicU32, AtomicBool, Ordering},
};

use cortex_m::{self, asm, delay::Delay};

use stm32_hal2::{
    self,
    clocks::{Clocks, InputSrc, PllCfg, PllSrc, VosRange},
    debug_workaround,
    dma::{self, Dma, DmaChannel, DmaInterrupt},
    flash::Flash,
    gpio::{self, Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::I2c,
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

mod atmos_model;
mod baro_driver_dps310;
mod flight_ctrls;
mod imu_driver_icm42605;
mod lidar_driver;
mod sensor_fusion;

use baro_driver_dps310 as baro;
use imu_driver_icm42605 as imu;
use lidar_driver as lidar;

use flight_ctrls::{FlightCmd, ManualInputs, Params, ParamsInst, PidState, RotorPower, PidDerivFilters};

// The frequency our motor-driving PWM operates at, in Hz.
const PWM_FREQ: f32 = 96_000.;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately.
const PWM_PSC: u32 = 100; // todo set properly
const PWM_ARR: u32 = 100; // todo set properly

// The rate our main program updates, in Hz.
const UPDATE_RATE: f32 = 32_000.; // todo: increase
const DT: f32 = 1. / UPDATE_RATE;

// Speed in meters per second commanded by full power.
// todo: This may not be what you want; could be unachievable, or maybe you really want
// full speed.
const V_FULL_DEFLECTION: f32 = 20.;

// Outside these thresholds, ignore LIDAR data. // todo: Set these
const LIDAR_THRESH_DIST: f32 = 10.; // meters
const LIDAR_THRESH_ANGLE: f32 = 0.03 * TAU; // radians, from level, in any direction.

// We use `LOOP_I` to manage inner vs outer loops.
static LOOP_I: AtomicU32 = AtomicU32::new(0);

static ARMED: AtomicBool = AtomicBool::new(false);

// todo: Start with a PID loop that dynamically adjusts its
// constants based on conditions and performance of the loop.
// (ie oscillations detected etc. Maybe FFT flight history?)

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
/// ///

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

/// Mode used for control inputs. These are the three "industry-standard" modes.
enum InputMode {
    /// Rate, also know as manual, hard or Acro
    Rate,
    /// Attitude also know as self-level or Auto-level
    Attitude,
    /// GPS-hold, also known as Loiter. Maintains a specific position.
    Loiter,
}

/// Categories of control mode, in regards to which parameters are held fixed.
/// Use this in conjunction with `InputMode`, and control inputs.
enum AutopilotMode {
    /// There are no specific targets to hold
    None,
    /// Altitude is fixed at a given altimeter (AGL)
    AltHold(f32),
    /// Heading is fixed.
    HdgHold(f32), // hdg
    /// Altidude and heading are fixed
    AltHdgHold(f32, f32), // alt, hdg
    /// Continuously fly towards a path. Note that `pitch` and `yaw` for the
    /// parameters here correspond to the flight path; not attitude.
    VelocityVector(f32, f32), // pitch, yaw
    /// The aircraft will fly a fixed profile between sequence points
    Sequence,
    /// Terrain following mode. Similar to TF radar in a jet. Require a forward-pointing sensor.
    TerrainFollowing(f32), // AGL to hold
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

    // I2C pins for _
    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);
    scl.pull(Pull::Up);

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);
    sda.pull(Pull::Up);

    // Temp pin for initial TS.
    let mut debug_killswitch = Pin::new(Port::B, 0, PinMode::Input);
}

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;

    // todo: Move vars from here to `local` as required.
    #[shared]
    struct Shared {
        // profile: FlightProfile,
        input_mode: InputMode,
        // todo: current_params vs
        current_params: Params,
        /// Proportional, Integral, Differential error
        // todo: Re-think how you store and manage PID error.
        pid_error_s: PidState,
        pid_error_v: PidState,
        pid_error_a: PidState,
        manual_inputs: ManualInputs,
        current_pwr: RotorPower,
        dma: Dma<DMA1>,
        spi: Spi<SPI1>,
        update_timer: Timer<TIM15>,
        rotor_timer: Timer<TIM2>,
        // `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        // Store filter instances for the PID loop derivatives. One for each param used.
        pid_deriv_filters: PidDerivFilters,
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
        let mut spi = Spi::new(dp.SPI1, Default::default(), BaudRate::Div32);

        // We use SPI for the _
        let mut i2c = I2c::new(dp.I2C1, Default::default(), &clock_cfg);

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
        // dma::mux(DmaChannel::C0, dma::DmaInput::Sai1A, &mut dp.DMAMUX1);

        let mut flash = Flash::new(dp.FLASH); // todo temp mut to test

        (
            // todo: Make these local as able.
            Shared {
                input_mode: InputMode::Attitude,
                // profile: FlightProfile::OnGround,
                current_params: Default::default(),
                // todo: Probably use a struct for PID error, with the 3 fields, as applicable
                pid_error_s: Default::default(),
                pid_error_v: Default::default(),
                pid_error_a: Default::default(),
                manual_inputs: Default::default(),
                current_pwr: Default::default(),
                dma,
                spi,
                update_timer,
                rotor_timer,
                power_used: 0.,
                pid_deriv_filters: PidDerivFilters::new(),
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
        shared = [current_params, manual_inputs, rotor_timer, current_pwr, pid_error_v, pid_error_s, spi, power_used],
        local = [],
        priority = 1
    )]
    fn update_isr(cx: update_isr::Context) {
        (
            cx.shared.current_params,
            cx.shared.manual_inputs,
            cx.shared.rotor_timer,
            cx.shared.current_pwr,
            cx.shared.pid_error_v,
            cx.shared.pid_error_s,
            cx.shared.spi,
            cx.shared.power_used,
        )
            .lock(
                |params,
                 inputs,
                 rotor_timer,
                 current_pwr,
                 pid_error_v,
                 pid_error_s,
                 spi,
                 power_used| {
                    // todo: Placeholder for sensor inputs/fusion.

                    let loop_i = LOOP_I.fetch_add(1, Ordering::Relaxed);
                    *power_used += current_pwr.total() * DT;

                    if gpio::is_high(Port::B, 0) {
                        // Deadman switch; temporary for debugging.
                        return;
                    }

                    // Sensors:
                    // - IMU 1:
                    // - IMU 2: (Mount perpendicular to IMU1, and avg the result)
                    // - magnetometer/compass
                    // - barometer
                    // - fwd airspeed?
                    // - Side airspeed?
                    // - laser facing down
                    // - ultrasonic facing down?

                    // todo: Don't just use IMU; use sensor fusion. Starting with this by default
                    let imu_data = imu::read_all(spi);

                    // todo: Move these into a `Params` method?
                    // We have acceleration data for x, y, z: Integrate to get velocity and position.
                    let v_x = params.v_x + imu_data.a_x;
                    let v_y = params.v_y + imu_data.a_y;
                    let v_z = params.v_z + imu_data.a_z;

                    let s_x = params.s_x + v_x;
                    let s_y = params.s_y + v_y;
                    let s_z = params.s_z + v_z;

                    // We have position data for pitch, roll, yaw: Take derivative to get velocity and acceleration
                    let v_pitch = imu_data.s_pitch - params.s_pitch;
                    let v_roll = imu_data.s_roll - params.s_roll;
                    let v_yaw = imu_data.s_yaw - params.s_yaw;

                    let a_pitch = v_pitch - params.v_pitch;
                    let a_roll = v_roll - params.v_roll;
                    let a_yaw = v_yaw - params.v_yaw;

                    let updated_params = Params {
                        s_x,
                        s_y,
                        s_z,
                        s_pitch: imu_data.s_pitch,
                        s_roll: imu_data.s_roll,
                        s_yaw: imu_data.s_yaw,

                        v_x,
                        v_y,
                        v_z,
                        v_pitch,
                        v_roll,
                        v_yaw,

                        a_x: imu_data.a_x,
                        a_y: imu_data.a_y,
                        a_z: imu_data.a_z,
                        a_pitch,
                        a_roll,
                        a_yaw,
                    };

                    *params = updated_params;

                    // Determine inputs to apply
                    let mut flight_cmd = FlightCmd::level();
                    // flight_cmd.add_inputs(inputs); todo. For now, make the hover work.

                    let (pid_s, pid_v) = flight_ctrls::calc_pid_error(
                        &updated_params,
                        &flight_cmd,
                        pid_error_s,
                        pid_error_v,
                    );

                    // flight_ctrls::adjust_ctrls(flight_cmd, pid_s, pid_v, current_pwr, rotor_timer);
                    flight_ctrls::adjust_ctrls(pid_s, pid_v, current_pwr, rotor_timer);
                },
            )
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
