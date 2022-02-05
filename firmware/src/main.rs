#![no_main]
#![no_std]

// #![allow(non_snake_case)]
// #![allow(clippy::needless_range_loop)]

use core::{
    f32::consts::TAU,
    ops::Sub,
    sync::atomic::{AtomicBool, AtomicI8, AtomicU32, AtomicUsize, Ordering},
};

use cortex_m::{self, asm, delay::Delay};

use stm32_hal2::{
    self,
    clocks::{Clocks, HsiDiv, InputSrc, PllCfg, PllSrc, SaiSrc, VosRange},
    debug_workaround,
    dma::{self, Dma, DmaChannel, DmaInterrupt},
    flash::Flash,
    gpio::{self, Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::I2c,
    pac::{self, DMA1, EXTI, I2C1, SPI1, TIM15, TIM2, TIM3, TIM4, TIM5},
    power::{SupplyConfig, VoltageLevel},
    rtc::{Rtc, RtcConfig},
    spi::{BaudRate, Spi, SpiConfig},
    timer::{OutputCompare, TimChannel, Timer, TimerConfig, TimerInterrupt},
};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as sys;

use defmt_rtt as _; // global logger
use panic_probe as _;

mod atmos_model;
mod baro_driver_dps310;
mod lidar_driver;
mod imu_driver_icm42605;

use baro_driver_dps310 as baro;
use lidar_driver as lidar;
use imu_driver_icm42605 as imu;

// The frequency our motor-driving PWM operates at, in Hz.
const PWM_FREQ: f32 = 96_000.;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately.
const PWM_PSC: u32 = 100; // todo set properly
const PWM_ARR: u32 = 100; // todo set properly

// The rate our main program updates, in Hz.
const UPDATE_RATE: f32 = 8_000.; // todo: increase
const DT: f32 = 1. / UPDATE_RATE;

// Speed in meters per second commanded by full power.
// todo: This may not be what you want; could be unachievable, or maybe you really want
// full speed.
const V_FULL_DEFLECTION: f32 = 20.;

const GRAVITY: f32 = 9.8; // m/s

// Outside these thresholds, ignore LIDAR data. // todo: Set these
const LIDAR_THRESH_DIST: f32 = 10.; // meters
const LIDAR_THRESH_ANGLE: f32 = 0.03 * TAU; // radians, from level, in any direction.

// `POWER_USED` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
static POWER_USED: AtomicU32 = AtomicU32::new(0);
// We use `LOOP_I` to manage inner vs outer loops.
static LOOP_I: AtomicU32 = AtomicU32::new(0);


// todo: Start with a PID loop that dynamically adjusts its
// constants based on conditions and performance of the loop.
// (ie oscillations detected etc. Maybe FFT flight history?)

// todo: Course set mode. Eg, point at thing using controls, activate a button,
// todo then the thing flies directly at the target.

// With this in mind: Store params in a global array. Maybe [ParamsInst; N], or maybe a numerical array for each param.

// todo: Consider a nested loop, where inner manages fine-tuning of angle, and outer
// manages directions etc. (?) look up prior art re quads and inner/outer loops.

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
struct aircraft_properties {
    mass: f32,               // grams
    arm_len: f32,            // meters
    drag_coeff: f32,         // unitless
    thrust_coeff: f32,       // N/m^2
    moment_of_intertia: f32, // kg x m^2
    rotor_inertia: f32,      // kg x m^2
}

impl aircraft_properties {
    pub fn level_pwr(&self, alt: f32) -> f32 {
        /// Calculate the power level required, applied to each rotor, to maintain level flight
        /// at a given MSL altitude. (Alt is in meters)
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
    pub fn tim_channel(&self) -> TimerChannel {
        match self {
            Self::R1 => TimerChannel::C1,
            Self::R2 => TimerChannel::C2,
            Self::R3 => TimerChannel::C3,
            Self::R4 => TimerChannel::C4,
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
    HdgHold(f32),  // hdg
    /// Altidude and heading are fixed
    AltHdgHold(f32, f32), // alt, hdg
    /// Continuously fly towards a path. Note that `pitch` and `yaw` for the
    /// parameters here correspond to the flight path; not attitude.
    VelocityVector(f32, f32),  // pitch, yaw
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


/// Represents parameters at a fixed instant. Can be position, velocity, or accel.
#[derive(Default)]
pub struct ParamsInst {
    x: f32,
    y: f32,
    /// Altitude
    z: f32,
    pitch: f32,
    roll: f32,
    yaw: f32,
}

impl Sub for ParamsInst {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            pitch: self.pitch - other.pitch,
            roll: self.roll - other.roll,
            yaw: self.yaw - other.yaw,
        }
    }
}

// todo: Quaternions?

/// Represents a first-order status of the drone. todo: What grid/reference are we using?
#[derive(Default)]
pub struct Params {
    // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
    s_x: f32,
    s_y: f32,
    /// Altitude, in AGL. We treat MSL as a varying offset from this.
    s_z: f32,

    s_pitch: f32,
    s_roll: f32,
    s_yaw: f32,

    // Velocity
    v_x: f32,
    v_y: f32,
    v_z: f32,

    v_pitch: f32,
    v_roll: f32,
    v_yaw: f32,

    // Acceleration
    a_x: f32,
    a_y: f32,
    a_z: f32,

    a_pitch: f32,
    a_roll: f32,
    a_yaw: f32,
}

/// A set of flight parameters to achieve and/or maintain. Similar values to `Parameters`,
/// but Options, specifying only the parameters we wish to achieve.
/// todo: Instead of hard-set values, consider a range, etc
/// todo: Some of these are mutually exclusive; consider a more nuanced approach.
#[derive(Default)]
pub struct FlightCmd {
    // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
    s_x: Option<f32>,
    s_y: Option<f32>,
    /// Altitude, in AGL. We treat MSL as a varying offset from this.
    s_z: Option<f32>,

    s_pitch: Option<f32>,
    s_roll: Option<f32>,
    s_yaw: Option<f32>,

    // Velocity
    v_x: Option<f32>,
    v_y: Option<f32>,
    v_z: Option<f32>,

    v_pitch: Option<f32>,
    v_roll: Option<f32>,
    v_yaw: Option<f32>,

    // Acceleration
    a_x: Option<f32>,
    a_y: Option<f32>,
    a_z: Option<f32>,

    a_pitch: Option<f32>,
    a_roll: Option<f32>,
    a_yaw: Option<f32>,
}

impl FlightCmd {
    // Command a basic hover. Maintains an altitude and pitch, and attempts to maintain position,
    // but does revert to a fixed position.
    // Alt is in AGL.
    pub fn hover(alt: f32) -> Self {
        Self {
            // Maintaining attitude isn't enough. We need to compensate for wind etc.
            v_x: Some(0.),
            v_y: Some(0.),
            v_z: Some(0.),
            // todo: Hover at a fixed position, using more advanced logic. Eg command an acceleration
            // todo to reach it, then slow down and alt hold while near it?
            // s_z: Some(alt),
            // Pitch, roll, and yaw probably aren't required here?
            // s_pitch: Some(0.),
            // s_roll: Some(0.),
            // s_yaw: Some(0.),
            ..Default::default()
        }
    }

    /// Keep the device level, with no other inputs. (Currently used for testing; perhaps also a baseline
    /// elsewhere)
    pub fn level() -> Self {
        Self {
            s_pitch: Some(0.),
            s_roll: Some(0.),
            s_yaw: Some(0.),
            ..Default::default()
        }
    }

    /// Maintains a hover in a specific location. lat and lon are in degrees. alt is in MSL.
    pub fn hover_geostationary(lat: f32, lon: f32, alt: f32) {

    }
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

/// Stores the current manual inputs to the system. `pitch`, `yaw`, and `roll` are in range -1. to +1.
/// `throttle` is in range 0. to 1. Corresponds to stick positions on a controller.
/// The interpretation of these depends on the current input mode.
#[derive(Default)]
pub struct ManualInputs {
    pitch: f32,
    roll: f32,
    yaw: f32,
    throttle: f32,
}

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% PWM duty cycle.
// todo: Discrete levels perhaps, eg multiples of the integer PWM ARR values.
#[derive(Default)]
pub struct RotorPower {
    p1: f32,
    p2: f32,
    p3: f32,
    p4: f32,
}

impl RotorPower {
    pub fn total(&self) -> f32 {
        self.p1 + self.p2 + self.p3 + self.p4
    }

    /// Send this power command to the rotors
    pub fn set(&self, pwm_timer: &mut Timer<TIM2>) {
        set_power(Rotor::R1, self.p1, timer);
        set_power(Rotor::R2, self.p2, timer);
        set_power(Rotor::R3, self.p3, timer);
        set_power(Rotor::R4, self.p4, timer);
    }
}

// todo: DMA for timer? How?

/// Set rotor speed for all 4 rotors, based on 6-axis control adjustments. Params here are power levels,
/// from 0. to 1. This translates and applies settings to rotor controls.
/// todo: This needs conceptual/fundamental work
fn set_attitude(
    pitch: f32,
    roll: f32,
    yaw: f32,
    throttle: f32,
    power: &mut RotorPower,
    pwm_timer: &mut Timer<TIM2>,
) {
    // todo: Start with `current_power` instead of zeroing?
    // let mut power = RotorPower::default();
    // let power = current_power;

    power.p1 += pitch / pitch_coeff;
    power.p2 += pitch / pitch_coeff;
    power.p3 -= pitch / pitch_coeff;
    power.p4 -= pitch / pitch_coeff;

    power.p1 += roll / roll_coeff;
    power.p2 -= roll / roll_coeff;
    power.p3 -= roll / roll_coeff;
    power.p4 += roll / roll_coeff;

    power.p1 += yaw / yaw_coeff;
    power.p2 -= yaw / yaw_coeff;
    power.p3 += yaw / yaw_coeff;
    power.p4 -= yaw / yaw_coeff;

    power.p1 *= throttle;
    power.p2 *= throttle;
    power.p3 *= throttle;
    power.p4 *= throttle;

    power.set(pwm_timer);
    // current_power = power;
}

/// Set an individual rotor's power. Power ranges from 0. to 1.
fn set_power(rotor: Rotor, power: f32, timer: &mut Timer<TIM2>) {
    // todo: Use a LUT or something for performance.
    let arr_portion = power * PWM_ARR as f32;

    timer.set_duty(rotor.tim_channel(), arr_portion as u32);
}

/// Calculate the vertical velocity (m/s), for a given height above the ground (m).
fn landing_speed(height: f32) -> f32 {
    // todo: LUT?
    height / 4.
}

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

    #[shared]
    struct Shared {
        profile: FlightProfile,
        input_mode: InputMode,
        // todo: current_params vs
        current_params: Params,
        // v_prev: ParamsInst,
        // v_diff_prev: ParamsInst,
        // v_integral_prev: ParamsInst,
        /// Proportional, Integral, Differential error
        // pid_error: (f32, f32, f32),
        pid_error_v: (ParamsInst, ParamsInst, ParamsInst),
        manual_inputs: ManualInputs,
        current_power: RotorPower,
        dma: Dma<DMA1>,
        spi: Spi<SPI1>,
        update_timer: Timer<TIM15>,
        rotor_timer: Timer<TIM2>,
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
            input_src: InputSrc::Pll1,
            pll_src: PllSrc::Hse(8_000_000),
            // vos_range: VosRange::VOS0, // Note: This may use extra power. todo: Put back!
            pll1: PllCfg {
                divm: 4, // To compensate with 8Mhz HSE instead of 64Mhz HSI
                // divn: 480,// todo: Put back! No longer working??
                ..Default::default()
            },

            // Configure PLL2P as the audio clock, with a speed of 12.286 Mhz.
            // todo: Fractional PLL to get exactly 12.288Mhz SAI clock?
            pll2: PllCfg {
                enabled: true,
                pllp_en: true,

                divm: 4, // To deal with 8Mhz HSE instead of 64Mhz HSI

                divn: 258,
                // We use the 12.288Mhz clock, due to the output codec being inflexible. Consider
                // a 9.216Mhz clock if you switch to a more flexible codec or DAC in the future.
                divp: 42, // 42 for For 12.288Mhz SCK; used for 48kHz, 256x bclk/FS ratio, 32-bit, 8-slot TDM
                // divp = 56 for For 9.216Mhz SCK; used for 48kHz, 192x bclk/FS ratio, 24-bit, 8-slot TDM
                ..PllCfg::disabled()
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
                profile: FlightProfile::OnGround,
                current_params: Default::default(),
                // v_prev: Default::default(),
                // v_diff_prev: Default::default(),
                // v_integ_prev: Default::default(),
                // pid_error: (0., 0., 0.),
                // todo: Probably use a struct for PID error, with the 3 fields, as applicable
                pid_error_v: (Default::default(), Default::default(), Default::default()),
                manual_inputs: Default::default(),
                current_power: Default::default(),
                dma,
                spi,
                update_timer,
                rotor_timer,
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

    #[task(binds = TIM15, shared = [current_params, manual_inputs, rotor_timer, 
        v_diff_prev, current_power, pid_error, pwm_timer, spi], local = [], priority = 1)]
    fn update_isr(cx: update_isr::Context) {
        (
            cx.shared.current_params,
            cx.shared.manual_inputs,
            cx.shared.v_diff_prev,
            cx.shared.current_power,
            cx.shared.pid_error,
            cx.shared.pwm_timer,
            cx.shared.spi,
        )
            .lock(|params, inputs, v_diff_prev, current_power, pid_error, pwm_timer, spi| {
                // todo: Placeholder for sensor inputs/fusion.

                let loop_i = LOOP_I.fetch_add(1, Ordering::Relaxed);
                POWER_USED.fetch_add(current_power.total() / UPDATE_FREQ, Ordering::Relaxed);

                if gpio::is_high(Port::B, 0) {
                    // Deadman switch; temporary for debugging.
                    return
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
                imu_data = imu::read_all(spi);


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

                params = updated_params;
                

                // Determine inputs to apply
                let mut flight_cmd = FlightCmd::level();
                // flight_cmd.mix_inputs(inputs);

                // PID "constants"
                let k_p = 0.1;
                let k_i = 0.05;
                let k_d = 0.;

                // Find appropriate control inputs using PID control.
                let error_p = v_current - flight_cmd;
                let error_i = pid_error.1 + error_p * DT;
                let error_d = (error_p - pid_error.2) / DT;

                // todo: Set pid error directly instead of with intermediate vars?
                pid_error = (error_p, error_i, error_d);

                // todo: DRY, make fn/method etc
                let mut adj_pitch = 0.;
                if let Some(v) = flight_cmd.s_pitch {
                    // todo: Check sign.
                    adj_pitch = k_p * error_p + k_i * error_i + k_d * error_d;
                }

                set_attitude(pitch_adj, roll_adj, yaw_adj, throttle_adj, current_power, pwm_timer);


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
