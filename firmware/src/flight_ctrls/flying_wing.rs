//! This module contains flight control code for flying-wing aircraft.
//! Note: We use M1 to connect to the motor, M3 for left elevon, and M4 for right elevon. M2 is unused.
//! On G4, We use Tim2 for the motor, and Tim3 for elevons (equivalent to quads).
//! On H7, since we use a single timer for all 4 motors on quads, but need different periods between servo
//! and motor here, we use Tim2 for the motor (as before), but Tim8 for the elevons (same pins).

// todo: For wing, consider lowering your main loop frequency to whatever the min servo update frequency is.

use stm32_hal2::{
    dma::Dma,
    pac::{self, DMA1, TIM2, TIM3},
    timer::{OutputCompare, TimChannel, Timer, TimerInterrupt},
};

#[cfg(feature = "h7")]
use stm32_hal2::pac::TIM8;

use crate::{dshot, flight_ctrls::quad::Motor, safety::ArmStatus, util, RotorMapping};

// todo: We're going to assume the servos operate off pulse width, with frequency between 40 and 200hz.
// todo: Unable to find DS for the specific servos used here.

use cfg_if::cfg_if;

use defmt::println;

const MIN_MOTOR_POWER: f32 = 0.03;

// Max power setting for any individual rotor at idle setting.
pub const MAX_MOTOR_POWER: f32 = 1.;

const ELEVON_MIN: f32 = -1.;
const ELEVON_MAX: f32 = 1.;

// ROLL_COEFF is used to balance pitch and roll input sensitivity, compared to the implied
// pitch coeffecient of 1.
const ROLL_COEFF: f32 = 1.;

// These pulse durations are in seconds, and correspond to nuetral, and full scale deflections.
// todo: Probably should be in a user config.
const ELEVON_PULSE_DUR_NEUTRAL: f32 = 0.0015; // In seconds. With this pulse dir, cervo is centered.
const ELEVON_PULSE_DUR_FULL_DOWN: f32 = 0.001; // In seconds. With this pulse dir, cervo is centered.
const ELEVON_PULSE_DUR_FULL_UP: f32 = 0.002; // In seconds. With this pulse dir, cervo is centered.

cfg_if! {
    if #[cfg(feature = "h7")] {
        // 480Mhz tim clock.
        pub const PSC: u16 = 0; // todo
        pub const ARR: u32 = 332; // todo
        // 520Mhz tim clock.
        // pub const PSC: u16 = 0; // todo
        // pub const ARR: u32 = 332; // todo
    } else if #[cfg(feature = "g4")] {
        // 170Mhz tim clock. Results in 500Hz.
        // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
        pub const PSC: u16 = 6;
        pub const ARR: u32 = 48_570;
    }
}

// High and low correspond to 2ms, and 1ms pulse width respsectively. (Period of 500Hz is 2ms)
// const DUTY_HIGH: u32 = ARR / 5 * 2;s
// const DUTY_LOW: u32 = ARR / 5;
// We don't use full ARR for max high, since that would be full high the whole time.
// const SERVO_DUTY_HIGH: f32 = (ARR - 50) as f32; // todo: 2ms
const SERVO_DUTY_HIGH: f32 = ARR as f32 * 0.2;
// const SERVO_DUTY_LOW: f32 = (ARR / 2) as f32; // todo: Thi sis our 1ms=low value.
const SERVO_DUTY_LOW: f32 = ARR as f32 * 0.7;

/// Sets the physical position of an elevon; commands a servo movement.
pub fn set_elevon_posit(
    elevon: ServoWing,
    position: f32,
    mapping: &ServoWingMapping,
    servo_timer: &mut Timer<TIM3>,
) {
    let range_out = match elevon {
        ServoWing::S1 => {
            if mapping.s1_reversed {
                (SERVO_DUTY_HIGH, SERVO_DUTY_LOW)
            } else {
                (SERVO_DUTY_LOW, SERVO_DUTY_HIGH)
            }
        }
        ServoWing::S2 => {
            if mapping.s2_reversed {
                (SERVO_DUTY_HIGH, SERVO_DUTY_LOW)
            } else {
                (SERVO_DUTY_LOW, SERVO_DUTY_HIGH)
            }
        }
    };

    let duty_arr = util::map_linear(position, (ELEVON_MIN, ELEVON_MAX), range_out) as u32;
    servo_timer.set_duty(elevon.tim_channel(), duty_arr);
}

/// See also: `dshot::setup_timers`.
pub fn setup_timers(motor_timer: &mut Timer<TIM2>, servo_timer: &mut Timer<TIM3>) {
    // todo: On H7, use TIM8 for the servos.

    motor_timer.set_prescaler(dshot::DSHOT_PSC_600);
    motor_timer.set_auto_reload(dshot::DSHOT_ARR_600 as u32);
    servo_timer.set_prescaler(PSC);
    servo_timer.set_auto_reload(ARR);

    motor_timer.enable_interrupt(TimerInterrupt::UpdateDma);
    // servo_timer.enable_interrupt(TimerInterrupt::Update);

    // Arbitrary duty cycle set, since we'll override it with DMA bursts.
    motor_timer.enable_pwm_output(Motor::M1.tim_channel(), OutputCompare::Pwm1, 0.);
    servo_timer.enable_pwm_output(ServoWing::S1.tim_channel(), OutputCompare::Pwm1, 0.);
    servo_timer.enable_pwm_output(ServoWing::S2.tim_channel(), OutputCompare::Pwm1, 0.);

    // PAC, since our HAL currently only sets this on `new`.
    servo_timer.regs.cr1.modify(|_, w| w.opm().set_bit());

    // Set servo pins to pull-down, to make sure they don't send an errant pulse that triggers a
    // movement out-of-range of the control surfaces.
    // todo: #1: Don't hard-code these pins. #2: Consider if this is helping and/or sufficient.
    unsafe {
        (*pac::GPIOB::ptr()).pupdr.modify(|_, w| {
            w.pupdr0().bits(0b10);
            w.pupdr1().bits(0b10)
        });
    }

    // Motor timer is enabled in Timer burst DMA. We enable the servo timer here.
    servo_timer.enable();
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

/// Equivalent of `RotorMapping` for quadcopters.
pub struct ServoWingMapping {
    pub s1: ServoWingPosition,
    pub s2: ServoWingPosition,
    // Reverse direction is somewhat arbitrary.
    pub s1_reversed: bool,
    pub s2_reversed: bool,
}

impl Default for ServoWingMapping {
    fn default() -> Self {
        Self {
            s1: ServoWingPosition::Left,
            s2: ServoWingPosition::Right,
            s1_reversed: false,
            s2_reversed: true,
        }
    }
}

impl ServoWingMapping {
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
#[derive(Default)]
pub struct ControlPositions {
    pub motor: f32,
    pub elevon_left: f32,
    pub elevon_right: f32,
}

impl ControlPositions {
    /// Send this command to cause power to be applied to the motor and servos.
    #[cfg(feature = "h7")]
    pub fn set(
        &self,
        motor_tim: &mut Timer<TIM3>,
        servo_tim: &mut Timer<TIM8>,
        arm_status: ArmStatus,
        mapping: &ServoWingMapping,
        dma: &mut Dma<DMA1>,
    ) {
        // M2 isn't used here, but keeps our API similar to Quad.
        // todo: TEMP to deal with lack of CRSF module attached to test rig!!
        let arm_status = ArmStatus::Armed;

        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power(
                    Motor::M1,
                    Motor::M2,
                    Motor::M3,
                    Motor::M4,
                    self.motor,
                    0.,
                    0.,
                    0.,
                    motor_tim,
                    dma,
                );

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, mapping, servo_tim);
                set_elevon_posit(ServoWing::S2, self.elevon_right, mapping, servo_tim);
            }
            ArmStatus::Disarmed => {
                dshot::stop_all(motor_tim, dma);

                set_elevon_posit(ServoWing::S1, 0., mapping, servo_tim);
                set_elevon_posit(ServoWing::S2, 0., mapping, servo_tim);
            }
        }
    }

    // todo: DRY with above!
    #[cfg(feature = "g4")]
    pub fn set(
        &self,
        motor_tim: &mut Timer<TIM2>,
        servo_tim: &mut Timer<TIM3>,
        arm_status: ArmStatus,
        mapping: &ServoWingMapping,
        dma: &mut Dma<DMA1>,
    ) {
        // M2 isn't used here, but keeps our API similar to Quad.
        // todo: TEMP to deal with lack of CRSF module attached to test rig!!
        let arm_status = ArmStatus::Armed;

        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power_a(self.motor, 0., motor_tim, dma);

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, mapping, servo_tim);
                set_elevon_posit(ServoWing::S2, self.elevon_right, mapping, servo_tim);
            }
            ArmStatus::Disarmed => {
                dshot::set_power_a(0., 0., motor_tim, dma);

                set_elevon_posit(ServoWing::S1, 0., mapping, servo_tim);
                set_elevon_posit(ServoWing::S2, 0., mapping, servo_tim);
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
    }
}

// todo: Move PWM code out of this module if it makes sense, ie separate servo; flight-control module

/// Apply controls based on pitch, roll, yaw, and throttle. Servo average position controls pitch;
/// servo difference controls roll. We don't have a yaw control.
/// If a servo exceeds min or max power settings, clamp it.
///
/// Positive pitch means nose up. Positive roll means left wing up.
///
/// Input deltas as on an abitrary scale based on PID output; they're not in real units like radians/s.
pub fn apply_controls(
    pitch_delta: f32,
    roll_delta: f32,
    throttle: f32,
    motor_tim: &mut Timer<TIM2>,
    servo_tim: &mut Timer<TIM3>,
    arm_status: ArmStatus,
    mapping: &ServoWingMapping,
    dma: &mut Dma<DMA1>,
) {
    let mut elevon_left = 0.;
    let mut elevon_right = 0.;

    elevon_left += pitch_delta;
    elevon_right += pitch_delta;

    elevon_left -= roll_delta * ROLL_COEFF;
    elevon_right += roll_delta * ROLL_COEFF;

    let mut posits = ControlPositions {
        motor: throttle,
        elevon_left,
        elevon_right,
    };

    posits.clamp();

    posits.set(motor_tim, servo_tim, arm_status, mapping, dma);
}

/// For a target pitch and roll rate, estimate the control positions required. Note that `throttle`
/// in `ctrl_positions` output is unused. Rates are in rad/s. Airspeed is indicated AS in m/s. Throttle is on a
/// scale of 0. to 1.
/// todo: Using power setting as a standin for airspeed for now, if we don't have a GPS or pitot.
/// todo: In the future use power as a permanent standin if these aren't equipped.
fn estimate_ctrl_posits(
    pitch_rate: f32,
    roll_rate: f32,
    airspeed: Option<f32>,
    throttle: f32,
) -> ControlPositions {
    let mut center = 0.;
    let mut diff = 0.; // positive diff = left wing up.

    // todo: Placeholder
    let pitch_const = 0.1;
    let roll_const = 0.1;

    // todo: Clean up DRY once the dust settles on this fn.

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

    // todo: Clamp both elevons in both directions.

    ControlPositions {
        motor: throttle,
        elevon_left,
        elevon_right,
    }
}
