#![no_main]
#![no_std]

// #![allow(non_snake_case)]
// #![allow(clippy::needless_range_loop)]

use core::{
    ops::Sub,
    sync::atomic::{AtomicBool, AtomicI8, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::{self, asm, delay::Delay};

use stm32_hal2::{
    self,
    clocks::{Clocks, HsiDiv, InputSrc, PllCfg, PllSrc, SaiSrc, VosRange},
    debug_workaround,
    dma::{self, Dma, DmaChannel, DmaInterrupt},
    flash::Flash,
    gpio::{Edge, OutputSpeed, OutputType, Pin, PinMode, Port, Pull},
    i2c::I2c,
    pac::{self, DMA1, EXTI, I2C1, SPI1, TIM2, TIM3, TIM4, TIM5, TIM15},
    power::{SupplyConfig, VoltageLevel},
    rtc::{Rtc, RtcConfig},
    spi::{Spi, SpiConfig, BaudRate},
    timer::{OutputCompare, Timer, TimChannel, TimerConfig, TimerInterrupt},
};

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as sys;

use defmt_rtt as _; // global logger
use panic_probe as _;

mod imu_driver;

// The frequency our motor-driving PWM operates at, in Hz.
const PWM_FREQ: f32 = 96_000.;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately.
const PWM_PSC: u32 = 100; // todo set properly
const PWM_ARR: u32 = 100; // todo set properly

// The rate our main program updates, in Hz.
const UPDATE_RATE: f32 = 500.;

// Speed in meters per second commanded by full scale "left stick" deflection
const V_FULL_DEFLECTION: f32 = 20.;

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

#[derive(Clone, Copy)]
/// Operating mode, selected by the user
pub enum OperatingMode {
    Flying,
}

#[derive(Clone, Copy)]
/// Role in a swarm of drones
pub enum SwarmRole {
    Queen,
    Worker(u16), // id
}

/// Represents positional parameters at a fixed instant.
#[derive(Default)]
pub struct ParamsInst {
    x: f32,
    y: f32,
    /// Altitude
    z: f32,
    /// Heading
    h: f32,
    pitch: f32,
    roll: f32,
}

// todo: Quaternions? 

/// Represents a first-order status of the drone. todo: What grid/reference are we using?
#[derive(Default)]
pub struct Params {
    // todo: Do we want to use this full struct, or store multiple (3+) instantaneous ones?
    x: f32,
    y: f32,
    /// Altitude
    z: f32,
    /// Heading
    h: f32,
    pitch: f32,
    roll: f32,
    // Velocity
    vx: f32,
    vy: f32,
    vz: f32,
    vh: f32,
    vpitch: f32,
    vroll: f32,
    // Acceleration
    ax: f32,
    ay: f32,
    az: f32,
    ah: f32,
    apitch: f32,
    aroll: f32,
}

pub enum FlightProfile {
    /// On ground, with rotors at 0. to 1., with 0. being off, and 1. being 1:1 lift-to-weight
    OnGround,
    /// Maintain a hover
    Hover,
    /// In transit to a given location, in Location and speed
    Transit(Location, f32),
    /// Landing
    Landing,
}

/// Stores the current manual inputs to the system
#[derive(Default)]
pub struct ManualInputs {
    /// xy can be thought of as gamepad "left stick"
    xy_dir: f32, // radians
    xy_mag: f32, // 0. to 1.
    up_dn: f32,  // -1. to 1.
    /// rot can be thought of as gamepad "right stick"
    rot_dir: f32,
    rot_mag: f32,
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

/// Execute a hovering profile
fn hover(timer1: Timer<TIM2>, timer2: Timer<TIM3>, timer3: Timer<TIM4>, timer4: Timer<TIM5>) {}

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

    // I2C pins for controller the output codec
    // let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    // scl.output_type(OutputType::OpenDrain);
    // scl.pull(Pull::Up);

    // let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    // sda.output_type(OutputType::OpenDrain);
    // sda.pull(Pull::Up);
}

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        profile: FlightProfile,
        current_params: Params,
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

        let imu_cs = Pin::new(Port::A, 0, PinMode::Alt(0));

        let mut dma = Dma::new(dp.DMA1);
        // dma::mux(DmaChannel::C0, dma::DmaInput::Sai1A, &mut dp.DMAMUX1);

        let mut flash = Flash::new(dp.FLASH); // todo temp mut to test

        (
            Shared {
                profile: FlightProfile::OnGround,
                current_params: Default::default(),
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

    #[task(binds = TIM15, shared = [current_params, manual_inputs, rotor_timer], local = [], priority = 1)]
    fn update_isr(cx: update_isr::Context) {
        (cx.shared.current_params, cx.shared.manual_inputs).lock(|params, inputs| {
            let dv1 = Vector::new(inputs.xy_dir, inputs.xy_mag, 1.)
                - Vector::new(params.vx, params.vy, params.vz);

            // xy_dir: f32, // radians
            // xy_mag: f32, // 0. to 1.
            // /// rot can be thought of as gamepad "right stick"
            // rot_dir: f32,
            // rot_mag: f32,
            // up_dn: f32, // -1. to 1.
        });
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
