//! This module contains flight control code for flying-wing aircraft.
//! We use the motor 1-4 pins for a mix of motors and servos.
//! We use M1 for the power motor, M3 for left elevon, and M4 for right elevon.
//! M2 is currently unused. Possibly future uses include second motor, and rudder.

// todo: For wing, consider lowering your main loop frequency to whatever the min servo update frequency is.

use crate::{
    dshot,
    protocols::servo,
    safety::ArmStatus,
    setup::{MotorTimer, ServoTimer},
    util,
};

use super::common::{CtrlMix, InputMap, Motor};

use cfg_if::cfg_if;
// use defmt::println;

const MIN_MOTOR_POWER: f32 = 0.02;

// Max power setting for any individual rotor at idle setting.
pub const MAX_MOTOR_POWER: f32 = 1.;

// Constants that represent min and max position of servos.
const ELEVON_MIN: f32 = -1.;
const ELEVON_MAX: f32 = 1.;

const RUDDER_MIN: f32 = -1.;
const RUDDER_MAX: f32 = 1.;

const ANGULAR_ACCEL_LOG_RATIO: usize = 20;

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
    timer: &mut ServoTimer,
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

    servo::set_posit(position, range_in, timer, elevon.tim_channel());
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

// /// Represents control settings for the motor, and elevons. Equivalent API to `quad::MotorPower`.
// /// Positive elevon value means pointed up relative to its hinge point.
// #[derive(Clone, Default)]
// pub struct ControlPositions {
//     pub motor: f32,
//     pub elevon_left: f32,
//     pub elevon_right: f32,
//     /// Only used if a rudder (or other yaw system) is connected; otherwise eignore.
//     pub rudder: f32,
// }

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

    pub fn set(
        &self,
        mapping: &ControlMapping,
        // motor_timer: &mut MotorTimer,
        servo_timer: &mut ServoTimer,
        arm_status: ArmStatus,
    ) {
        match arm_status {
            ArmStatus::MotorsControlsArmed => {
                // dshot::set_power(self.motor, 0., 0., 0., motor_timer);

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, mapping, servo_timer);
                set_elevon_posit(ServoWing::S2, self.elevon_right, mapping, servo_timer);
            }
            ArmStatus::ControlsArmed => {
                // dshot::stop_all(motor_timer);

                // todo: Apply to left and right wing by mapping etc! Here or upstream.
                set_elevon_posit(ServoWing::S1, self.elevon_left, mapping, servo_timer);
                set_elevon_posit(ServoWing::S2, self.elevon_right, mapping, servo_timer);
            }
            ArmStatus::Disarmed => {
                // dshot::stop_all(motor_timer);

                set_elevon_posit(ServoWing::S1, 0., mapping, servo_timer);
                set_elevon_posit(ServoWing::S2, 0., mapping, servo_timer);
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

    /// Maps to angular accel. Positive means nose-up pitching.
    /// Note: This is located on a non-equiv struct on Quads (RPMs). This is because
    /// on fixed-wing, we map control commands directly to accel, while
    pub fn pitch_delta(&self) -> f32 {
        self.elevon_left + self.elevon_right // todo: QC this
    }

    /// Maps to angular accel. Positive means left-wing-up.
    /// (See note on `pitch_delta)`.
    pub fn roll_delta(&self) -> f32 {
        self.elevon_right - self.elevon_left
    }

    pub fn yaw_delta(&self) -> f32 {
        self.rudder
    }
}
