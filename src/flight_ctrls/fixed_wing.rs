//! This module contains flight control code for flying-wing aircraft.
//! Note: We use M1 to connect to the motor, M3 for left elevon, and M4 for right elevon. M2 is unused.
//! On G4, We use Tim2 for the motor, and Tim3 for elevons (equivalent to quads).
//! On H7, since we use a single timer for all 4 motors on quads, but need different periods between servo
//! and motor here, we use Tim2 for the motor (as before), but Tim8 for the elevons (same pins).

// todo: For wing, consider lowering your main loop frequency to whatever the min servo update frequency is.

use stm32_hal2::{
    dma::Dma,
    pac::{self, DMA1},
    timer::{OutputCompare, TimerInterrupt},
};

use crate::{dshot, safety::ArmStatus, util};

use super::common::{CtrlMix, InputMap, Motor, MotorTimers};

use cfg_if::cfg_if;
// use defmt::println;

// todo: We're going to assume the servos operate off pulse width, with frequency between 40 and 200hz.
// todo: Unable to find DS for the specific servos used here.

const MIN_MOTOR_POWER: f32 = 0.02;

// Max power setting for any individual rotor at idle setting.
pub const MAX_MOTOR_POWER: f32 = 1.;

const ELEVON_MIN: f32 = -1.;
const ELEVON_MAX: f32 = 1.;

const RUDDER_MIN: f32 = -1.;
const RUDDER_MAX: f32 = 1.;

const ANGULAR_ACCEL_LOG_RATIO: usize = 20;

// ROLL_COEFF is used to balance pitch and roll input sensitivity, compared to the implied
// pitch coeffecient of 1. A higher coefficient will cause a greater roll response for a given input command,
// while leaving pitch response the same.
// const ROLL_COEFF: f32 = 5.;
// const YAW_COEFF: f32 = 1.; // todo

// Update frequency: 500Hz. See `dshot.rs` for the calculation.
// 170Mhz tim clock on G4.
// 240Mhz tim clock on H743
// 260Mhz tim clock on H723 @ 520Mhz. 275Mhz @ 550Mhz
cfg_if! {
    if #[cfg(feature = "h7")] {
        // 240Mhz tim clock.
        // pub const PSC_SERVOS: u16 = 7;
        // pub const ARR_SERVOS: u32 = 59_999;
        // 260Mhz tim clock.
        pub const PSC_SERVOS: u16 = 7;
        pub const ARR_SERVOS: u32 = 64_999;
        // 275Mhz tim clock.
        // pub const PSC_SERVOS: u16 = 8;
        // pub const ARR_SERVOS: u32 = 61_110;
    } else if #[cfg(feature = "g4")] {
        pub const PSC_SERVOS: u16 = 6;
        pub const ARR_SERVOS: u32 = 48_570;
    }
}

// These values are to set middle, min and max values of 1.5ms, 1ms, and 2ms used
// by common hobby servos.
// Calculations, assuming frequency of 500Hz; 500Hz = 2ms.
// ARR indicates
const ARR_MIN: u32 = ARR_SERVOS / 2;
const ARR_MID: u32 = ARR_SERVOS * 3 / 4;
// Don't us full ARR: there needs to be some low time.
const ARR_MAX: u32 = ARR_SERVOS - 100;

impl Default for InputMap {
    fn default() -> Self {
        Self {
            pitch_rate: (-6., 6.),
            roll_rate: (-6., 6.),
            yaw_rate: (-6., 6.),
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
        }
    }
}

/// Sets the physical position of an elevon; commands a servo movement.
pub fn set_elevon_posit(
    elevon: ServoWing,
    position: f32,
    mapping: &ControlMapping,
    timers: &mut MotorTimers,
) {
    let range_in = match elevon {
        ServoWing::S1 => {
            if mapping.s1_reversed {
                (mapping.servo_high, -mapping.servo_high)
            } else {
                (-mapping.servo_high, mapping.servo_high)
            }
        }
        ServoWing::S2 => {
            if mapping.s2_reversed {
                (mapping.servo_high, -mapping.servo_high)
            } else {
                (-mapping.servo_high, mapping.servo_high)
            }
        }
    };

    // todo: Consider storring ARR_MIN and ARR_MAX as f32.
    let duty_arr = util::map_linear(position, range_in, (ARR_MIN as f32, ARR_MAX as f32)) as u32;

    #[cfg(feature = "h7")]
    timers
        .servos
        .set_duty(elevon.tim_channel(), duty_arr as u16);
    #[cfg(feature = "g4")]
    timers.r34_servos.set_duty(elevon.tim_channel(), duty_arr);
}

/// Similar to `dshot::setup_timers`, but for fixed-wing.
pub fn setup_timers(timers: &mut MotorTimers) {
    cfg_if! {
        if #[cfg(feature = "h7")] {
            let mut motor_tim = &mut timers.r1234;
            let mut servo_tim = &mut timers.servos;
        } else {
            let mut motor_tim = &mut timers.r12;
            let mut servo_tim = &mut timers.r34_servos;
        }
    }

    motor_tim.set_prescaler(dshot::DSHOT_PSC_600);
    motor_tim.set_auto_reload(dshot::DSHOT_ARR_600 as u32);
    servo_tim.set_prescaler(PSC_SERVOS);
    servo_tim.set_auto_reload(ARR_SERVOS);

    motor_tim.enable_interrupt(TimerInterrupt::UpdateDma);
    // servo_timer.enable_interrupt(TimerInterrupt::Update);

    // Arbitrary duty cycle set, since we'll override it with DMA bursts for the motor, and
    // position settings for the servos.
    motor_tim.enable_pwm_output(Motor::M1.tim_channel(), OutputCompare::Pwm1, 0.);
    servo_tim.enable_pwm_output(ServoWing::S1.tim_channel(), OutputCompare::Pwm1, 0.);
    servo_tim.enable_pwm_output(ServoWing::S2.tim_channel(), OutputCompare::Pwm1, 0.);

    // PAC, since our HAL currently only sets this on `new`.
    servo_tim.regs.cr1.modify(|_, w| w.opm().set_bit());

    // Set servo pins to pull-up, to make sure they don't shorten a pulse on a MCU reset
    // or similar condition.
    // todo: #1: Don't hard-code these pins. #2: Consider if this is helping and/or sufficient.
    #[cfg(feature = "h7")]
    unsafe {
        (*pac::GPIOC::ptr()).pupdr.modify(|_, w| {
            w.pupdr8().bits(0b01);
            w.pupdr9().bits(0b01)
        });
    }
    #[cfg(feature = "g4")]
    unsafe {
        (*pac::GPIOB::ptr()).pupdr.modify(|_, w| {
            w.pupdr0().bits(0b01);
            w.pupdr1().bits(0b01)
        });
    }

    // Motor timer is enabled in Timer burst DMA. We enable the servo timer here.
    servo_tim.enable();
}

/// Equivalent of `Motor` for quadcopters.
#[derive(Clone, Copy)]
pub enum ServoWing {
    S1,
    S2,
}

/// Specify the wing associated with a servo. Equivalent of `RotorPosition` for quadcopters.
/// repr(u8) is for use in Preflight.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ServoWingPosition {
    Left = 0,
    Right = 1,
}

/// Maps servos to wing position, and related details.
pub struct ControlMapping {
    pub s1: ServoWingPosition,
    pub s2: ServoWingPosition,
    /// Reverse direction is somewhat arbitrary.
    pub s1_reversed: bool,
    pub s2_reversed: bool,
    /// These represent full scale deflection of the evelons, on a scale of -1 to +1.
    /// We don't use full ARR for max high, since that would be full high the whole time.
    /// Note that the 0 position is fixed; we don't map between these two values; we map between
    /// 0 and each of these.
    /// Note: We currently clamp high and low to be on opposite sides of 0. This may not reflect
    /// control surface reality, but keeps things simple, and should be good enough to start.
    pub servo_high: f32,
    // pub servo_low: f32,
}

impl Default for ControlMapping {
    fn default() -> Self {
        Self {
            s1: ServoWingPosition::Left,
            s2: ServoWingPosition::Right,
            s1_reversed: false,
            s2_reversed: true,
            servo_high: 0.5,
            // servo_low: -0.5,
        }
    }
}

impl ControlMapping {
    pub fn servo_from_position(&self, position: ServoWingPosition) -> ServoWing {
        // todo: This assumes each servo maps to exactly one position. We probably
        // todo should have some constraint to enforce this.
        if self.s1 == position {
            ServoWing::S1
        } else {
            ServoWing::S2
        }
    }
}

/// Represents control settings for the motor, and elevons. Equivalent API to `quad::MotorPower`.
/// Positive elevon value means pointed up relative to its hinge point.
#[derive(Clone, Default)]
pub struct ControlPositions {
    pub motor: f32,
    pub elevon_left: f32,
    pub elevon_right: f32,
    /// Only used if a rudder (or other yaw system) is connected; otherwise eignore.
    pub rudder: f32,
}

impl ControlPositions {
    /// Apply controls based on pitch, roll, yaw, and throttle. Servo average position controls pitch;
    /// servo difference controls roll. We don't have a yaw control.
    /// If a servo exceeds min or max power settings, clamp it.
    ///
    /// Positive pitch means nose up. Positive roll means left wing up.
    ///
    /// Input deltas as on an abitrary scale based on PID output; they're not in real units like radians/s.
    pub fn from_cmds(mix: &CtrlMix) -> Self {
        let mut elevon_left = 0.;
        let mut elevon_right = 0.;
        let mut rudder = 0.;

        elevon_left += mix.pitch;
        elevon_right += mix.pitch;

        // elevon_left += mix.roll * ROLL_COEFF;
        // elevon_right -= mix.roll * ROLL_COEFF;
        //
        // rudder += mix.pitch * YAW_COEFF;

        elevon_left += mix.roll;
        elevon_right -= mix.roll;

        rudder += mix.pitch;

        let mut result = Self {
            motor: mix.throttle,
            elevon_left,
            elevon_right,
            rudder,
        };

        result.clamp();

        result
    }

    /// Send this command to cause power to be applied to the motor and servos.
    pub fn set(
        &self,
        mapping: &ControlMapping,
        motor_timers: &mut MotorTimers,
        arm_status: ArmStatus,
        dma: &mut Dma<DMA1>,
    ) {
        // M2 isn't used here, but keeps our API similar to Quad.
        match arm_status {
            ArmStatus::MotorsControlsArmed => {
                dshot::set_power(self.motor, 0., 0., 0., motor_timers, dma);

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, mapping, motor_timers);
                set_elevon_posit(ServoWing::S2, self.elevon_right, mapping, motor_timers);
            }
            ArmStatus::ControlsArmed => {
                dshot::stop_all(motor_timers, dma);

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, mapping, motor_timers);
                set_elevon_posit(ServoWing::S2, self.elevon_right, mapping, motor_timers);
            }
            ArmStatus::Disarmed => {
                dshot::stop_all(motor_timers, dma);

                set_elevon_posit(ServoWing::S1, 0., mapping, motor_timers);
                set_elevon_posit(ServoWing::S2, 0., mapping, motor_timers);
            }
        }
    }

    /// Clamp motor speed and servo motion. A simple form of dealing with out of limits.
    pub fn clamp(&mut self) {
        if self.motor < MIN_MOTOR_POWER {
            self.motor = MIN_MOTOR_POWER;
        } else if self.motor > MAX_MOTOR_POWER {
            self.motor = MAX_MOTOR_POWER;
        }

        if self.elevon_left < ELEVON_MIN {
            self.elevon_left = ELEVON_MIN;
        } else if self.elevon_left > ELEVON_MAX {
            self.elevon_left = ELEVON_MAX;
        }

        if self.elevon_right < ELEVON_MIN {
            self.elevon_right = ELEVON_MIN;
        } else if self.elevon_right > ELEVON_MAX {
            self.elevon_right = ELEVON_MAX;
        }

        if self.rudder < RUDDER_MIN {
            self.rudder = RUDDER_MIN;
        } else if self.rudder > RUDDER_MAX {
            self.rudder = RUDDER_MAX;
        }
    }
}

// todo: Move PWM code out of this module if it makes sense, ie separate servo; flight-control module

/// For a target pitch and roll rate, estimate the control positions required. Note that `throttle`
/// in `ctrl_positions` output is unused. Rates are in rad/s. Airspeed is indicated AS in m/s. Throttle is on a
/// scale of 0. to 1.
/// todo: Using power setting as a standin for airspeed for now, if we don't have a GPS or pitot.
/// todo: In the future use power as a permanent standin if these aren't equipped.
fn _estimate_ctrl_posits(
    pitch_rate: f32,
    roll_rate: f32,
    airspeed: Option<f32>,
    throttle: f32,
) -> ControlPositions {
    let mut center = 0.;
    let mut diff = 0.; // positive diff = left wing up.
    let mut rudder = 0.;

    // todo: Placeholder
    let pitch_const = 0.1;
    let roll_const = 0.1;
    let yaw_const = 0.1;

    // todo: Clean up DRY once the dust settles on this fn.

    // todo: Use this to modify rudder too.
    match airspeed {
        Some(speed) => {
            center = pitch_const * pitch_rate / speed;
            diff = roll_const * roll_rate / speed;
        }
        None => {
            center = pitch_const * pitch_rate / throttle;
            diff = roll_const * roll_rate / throttle;
        }
    }

    // todo: DRY from apply_ctrls!
    let mut elevon_left = 0.;
    let mut elevon_right = 0.;

    elevon_left += center;
    elevon_right += center;

    elevon_left -= diff * ROLL_COEFF;
    elevon_right += diff * ROLL_COEFF;

    rudder += diff * YAW_COEFF;

    // todo: Clamp both elevons in both directions.

    ControlPositions {
        motor: throttle,
        elevon_left,
        elevon_right,
        rudder,
    }
}
