//! This module contains code for storing state, and applying motor and servo commands.
//! its more basic data structures apply to both quadcopters and fixed-wing, and aren't
//! specific to a specific role. The aggregate structures are more specific.

use crate::{
    protocols::{dshot, servo},
    safety::ArmStatus,
    setup::{MotorTimer, ServoTimer},
    DT_FLIGHT_CTRLS,
};

use super::{common::CtrlMix, pid};

const MOTOR_CMD_MIN: f32 = 0.03; //  An idle.
const MOTOR_CMD_MAX: f32 = 1.;
const MOTOR_RPM_MIN: f32 = 500.;
const MOTOR_RPM_MAX: f32 = 6_000.; // todo: PRobably depends on motors.
const SERVO_CMD_MIN: f32 = -1.;
const SERVO_CMD_MAX: f32 = 1.;

// todo: Do we use these?
// // Min power setting for any individual rotor at idle setting.
// const MIN_ROTOR_POWER: f32 = 0.03;
//
// Min RPM setting for any individual rotor at idle setting.
const MIN_ROTOR_RPM: f32 = 100.; // todo: Finda good value.

// Max power setting for any individual rotor at idle setting.
pub const MAX_ROTOR_POWER: f32 = 1.;

// todo: End of do we use these.

#[derive(Default)]
pub struct RpmCmd {
    /// The RPM commanded.
    pub rpm_cmd: f32,
    /// The instantaneous power level calculated to achieve this RPM.
    pub pwr_calculated: f32,
}

pub enum MotorCmd {
    Power(f32),
    Rpm(RpmCmd),
}

impl Default for MotorCmd {
    fn default() -> Self {
        Self::Power(0.)
    }
}

impl MotorCmd {
    /// Get the power command; both types have it.
    pub fn power(&self) -> f32 {
        match self {
            Self::Power(p) => *p,
            Self::Rpm(r) => r.pwr_calculated,
        }
    }

    /// Clamp power and/or RPM commands.
    pub fn clamp(&mut self) {
        match self {
            Self::Power(c) => {
                if *c < MOTOR_CMD_MIN {
                    *c = MOTOR_CMD_MIN;
                } else if *c > MOTOR_CMD_MAX {
                    *c = MOTOR_CMD_MAX;
                }
            }
            Self::Rpm(c) => {
                if c.rpm_cmd < MOTOR_RPM_MIN {
                    c.rpm_cmd = MOTOR_RPM_MIN;
                } else if c.rpm_cmd > MOTOR_RPM_MAX {
                    c.rpm_cmd = MOTOR_RPM_MAX;
                }

                if c.pwr_calculated < MOTOR_CMD_MIN {
                    c.pwr_calculated = MOTOR_CMD_MIN;
                } else if c.pwr_calculated > MOTOR_CMD_MAX {
                    c.pwr_calculated = MOTOR_CMD_MAX;
                }
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)] // u8 repr for serializing via USB.
pub enum RotationDir {
    Clockwise = 0,
    CounterClockwise = 1,
}

/// State of an individual motor.
#[derive(Default)]
pub struct MotorState {
    /// None indicates no reading.
    pub cmd: MotorCmd,
    pub power_setting: f32,
    // todo: When do we set this??a
    pub rpm_reading: Option<f32>, // todo: This state is repatative with `rpm_readings`.
    // pub dir: RotationDir, // todo: Do we want this?
    /// Reversed is in relation to the 3-wire motor brushless wiring. This software setting
    /// allows the wires to be connected in any order, and compensated for in software. (Eg by
    /// sending a DSHOT command to reverse the direction appropriately)
    /// Note that we set this at firmware init.
    pub reversed: bool,
}

/// State of an individual servo.
#[derive(Clone, Copy, PartialEq, Default)]
pub struct ServoState {
    /// Commanded position
    pub posit_cmd: f32,
    pub reversed: bool,
}

impl ServoState {
    pub fn clamp(&mut self) {
        if self.posit_cmd < SERVO_CMD_MIN {
            self.posit_cmd = SERVO_CMD_MIN;
        } else if self.posit_cmd > SERVO_CMD_MAX {
            self.posit_cmd = SERVO_CMD_MAX;
        }
    }
}

/// A possible function for a given motor/servo pin
/// Interior values for rotors are RPM. Interior values for servos
/// are servos positions. (On a scale of -1. to 1.)
pub enum MotorServoRole {
    Unused,
    RotorFrontLeft(MotorState),
    RotorFrontRight(MotorState),
    RotorAftLeft(MotorState),
    RotorAftRight(MotorState),
    ThrustMotor1(MotorState),
    ThrustMotor2(MotorState),
    ElevonLeft(ServoState),
    ElevonRight(ServoState), // todo
}

impl MotorServoRole {
    /// Clamp commanded settings.
    pub fn clamp_cmd(&mut self) {
        match self {
            Self::Unused => (),
            Self::RotorFrontLeft(m) => {
                m.cmd.clamp();
            }
            Self::RotorFrontRight(m) => {
                m.cmd.clamp();
            }
            Self::RotorAftLeft(m) => {
                m.cmd.clamp();
            }
            Self::RotorAftRight(m) => {
                m.cmd.clamp();
            }
            Self::ThrustMotor1(m) => {
                m.cmd.clamp();
            }
            Self::ThrustMotor2(m) => {
                m.cmd.clamp();
            }
            Self::ElevonLeft(m) => {
                m.clamp();
            }
            Self::ElevonRight(m) => {
                m.clamp();
            }
        }
    }
}

/// Corresponds to pin number. Used to map functions (Such as thrust motor, front-left rotor etc)
/// to hardware pins. The u8 repr is for Preflight.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum MotorServoHardware {
    Pin1 = 1,
    Pin2 = 2,
    Pin3 = 3,
    Pin4 = 4,
    Pin5 = 5,
    Pin6 = 6,
}

/// Describes the function of all motors and servos.  Is based on the pin connections. Each pin
/// can be a motor or servo. Doesn't directly deliniate quadcopter vice fixed-wing.
///
/// Note: there are some restrictions based on timer-pin mappings, that are different between
/// G4 and H7.
/// todo: Rename A/R
#[cfg(feature = "quad")]
pub struct MotorServoState {
    // todo: Consider hardware presets instead of free fields; would allow to better match
    // todo timer pin-mapping constraints.
    pub rotor_front_left_hardware: MotorServoHardware,
    pub rotor_front_right_hardware: MotorServoHardware,
    pub rotor_aft_left_hardware: MotorServoHardware,
    pub rotor_aft_right_hardware: MotorServoHardware,
    pub servo_aux_1_hardware: Option<MotorServoHardware>,
    pub servo_aux_2_hardware: Option<MotorServoHardware>,

    pub rotor_front_left: MotorState,
    pub rotor_front_right: MotorState,
    pub rotor_aft_left: MotorState,
    pub rotor_aft_right: MotorState,
    pub servo_aux_1: Option<ServoState>,
    pub servo_aux_2: Option<ServoState>,

    pub frontleft_aftright_dir: RotationDir,
}

#[cfg(feature = "fixed-wing")]
pub struct MotorServoState {
    pub motor_thrust1_hardware: MotorServoHardware,
    pub motor_thrust2_hardware: Option<MotorServoHardware>,
    pub elevon_left_hardware: MotorServoHardware,
    pub elevon_right_hardware: MotorServoHardware,
    pub rudder_hardware: Option<MotorServoHardware>,
    pub servo_aux_1_hardware: Option<MotorServoHardware>,
    pub servo_aux_2_hardware: Option<MotorServoHardware>,

    pub motor_thrust1: MotorState,
    pub motor_thrust2: Option<MotorState>,
    /// Elevons are similar to stabilators.
    pub elevon_left: ServoState,
    pub elevon_right: ServoState,
    pub rudder: Option<ServoState>,
    pub servo_aux_1: Option<ServoState>,
    pub servo_aux_2: Option<ServoState>,
    // todo: More A/R, eg ailerons, elevator etc.
}

impl Default for MotorServoState {
    fn default() -> Self {
        #[cfg(feature = "quad")]
        return Self {
            rotor_front_left_hardware: MotorServoHardware::Pin4,
            rotor_front_right_hardware: MotorServoHardware::Pin2,
            rotor_aft_left_hardware: MotorServoHardware::Pin3,
            rotor_aft_right_hardware: MotorServoHardware::Pin1,
            servo_aux_1_hardware: None,
            servo_aux_2_hardware: None,

            rotor_front_left: Default::default(),
            rotor_front_right: Default::default(),
            rotor_aft_left: Default::default(),
            rotor_aft_right: Default::default(),
            servo_aux_1: None,
            servo_aux_2: None,

            frontleft_aftright_dir: RotationDir::Clockwise,
        };

        #[cfg(feature = "fixed-wing")]
        return Self {
            motor_thrust1_hardware: MotorServoHardware::Pin1,
            motor_thrust2_hardware: None,
            elevon_left_hardware: MotorServoHardware::Pin3,
            elevon_right_hardware: MotorServoHardware::Pin2,
            rudder_hardware: None,
            servo_aux_1_hardware: None,
            servo_aux_2_hardware: None,

            motor_thrust1: Default::default(),
            motor_thrust2: None,
            elevon_left: Default::default(),
            elevon_right: Default::default(),
            rudder: None,
            servo_aux_1: None,
            servo_aux_2: None,
        };
    }
}

impl MotorServoState {
    //     #[cfg(feature = "quad")]
    //     fn get_front_left(&self) -> &mut MotorServoRole {
    //
    //     }

    /// Update internal state of RPM readings.
    pub fn update_rpm_readings(&mut self, readings: &RpmReadings) {
        self.rotor_front_left.rpm_reading = readings.front_left;
        self.rotor_front_right.rpm_reading = readings.front_right;
        self.rotor_aft_left.rpm_reading = readings.aft_left;
        self.rotor_aft_right.rpm_reading = readings.aft_right;
    }

    /// Populate command state from rotor RPMs. This both marks the target RPM,
    /// and calculates an instantaneous power level to achieve it.
    ///
    /// Note that RPMs must already be updated in this instance.
    #[cfg(feature = "quad")]
    pub fn set_cmds_from_rpms(
        &mut self,
        rpms_commanded: &MotorRpm,
        pid_group: &pid::MotorPidGroup,
        pid_coeffs: &pid::MotorCoeffs,
    ) {
        // todo: DRY
        // Calculate target RPMS, using our PID logic.
        let p_front_left = match self.rotor_front_left.rpm_reading {
            Some(reading) => {
                pid::run(
                    rpms_commanded.front_left,
                    reading,
                    &pid_group.front_left,
                    pid_coeffs.p_front_left,
                    pid_coeffs.i_front_left,
                    0.,
                    None,
                    DT_FLIGHT_CTRLS,
                )
                .out()
                    + self.rotor_front_left.cmd.power()
            }
            None => 0.,
        };

        let p_front_right = match self.rotor_front_right.rpm_reading {
            Some(reading) => {
                pid::run(
                    rpms_commanded.front_right,
                    reading,
                    &pid_group.front_right,
                    pid_coeffs.p_front_right,
                    pid_coeffs.i_front_right,
                    0.,
                    None,
                    DT_FLIGHT_CTRLS,
                )
                .out()
                    + self.rotor_front_right.cmd.power()
            }
            None => 0.,
        };

        let p_aft_left = match self.rotor_aft_left.rpm_reading {
            Some(reading) => {
                pid::run(
                    rpms_commanded.aft_left,
                    reading,
                    &pid_group.aft_left,
                    pid_coeffs.p_aft_left,
                    pid_coeffs.i_aft_left,
                    0.,
                    None,
                    DT_FLIGHT_CTRLS,
                )
                .out()
                    + self.rotor_aft_left.cmd.power()
            }
            None => 0.,
        };

        let p_aft_right = match self.rotor_aft_right.rpm_reading {
            Some(reading) => {
                pid::run(
                    rpms_commanded.aft_right,
                    reading,
                    &pid_group.aft_right,
                    pid_coeffs.p_aft_right,
                    pid_coeffs.i_aft_right,
                    0.,
                    None,
                    DT_FLIGHT_CTRLS,
                )
                .out()
                    + self.rotor_aft_right.cmd.power()
            }
            None => 0.,
        };

        self.rotor_front_left.cmd = MotorCmd::Rpm(RpmCmd {
            rpm_cmd: rpms_commanded.front_left,
            pwr_calculated: p_front_left,
        });
        self.rotor_front_right.cmd = MotorCmd::Rpm(RpmCmd {
            rpm_cmd: rpms_commanded.front_right,
            pwr_calculated: p_front_right,
        });
        self.rotor_aft_left.cmd = MotorCmd::Rpm(RpmCmd {
            rpm_cmd: rpms_commanded.aft_left,
            pwr_calculated: p_aft_left,
        });
        self.rotor_aft_right.cmd = MotorCmd::Rpm(RpmCmd {
            rpm_cmd: rpms_commanded.aft_right,
            pwr_calculated: p_aft_right,
        });

        self.clamp_cmds();
    }

    #[cfg(feature = "quad")]
    pub fn get_rpm_readings(&self) -> MotorRpm {
        MotorRpm {
            front_left: self.rotor_front_left.rpm_reading.unwrap_or(0.),
            front_right: self.rotor_front_right.rpm_reading.unwrap_or(0.),
            aft_left: self.rotor_aft_left.rpm_reading.unwrap_or(0.),
            aft_right: self.rotor_aft_right.rpm_reading.unwrap_or(0.),
        }
    }

    #[cfg(feature = "quad")]
    pub fn get_power_settings(&self) -> MotorPower {
        MotorPower {
            front_left: self.rotor_front_left.power_setting,
            front_right: self.rotor_front_right.power_setting,
            aft_left: self.rotor_aft_left.power_setting,
            aft_right: self.rotor_aft_right.power_setting,
        }
    }

    #[cfg(feature = "fixed-wing")]
    pub fn get_ctrl_positions(&self) -> CtrlSfcPosits {
        CtrlSfcPosits {
            elevon_left: self.elevon_left.posit_cmd,
            elevon_right: self.elevon_right.posit_cmd,
            rudder: None, //todo! And differential thrust between motors A/R.
        }
    }

    /// Populate commands from motor powers.
    #[cfg(feature = "quad")]
    pub fn set_cmds_from_power(&mut self, powers: &MotorPower) {
        self.rotor_front_left.cmd = MotorCmd::Power(powers.front_left);
        self.rotor_front_right.cmd = MotorCmd::Power(powers.front_right);
        self.rotor_aft_left.cmd = MotorCmd::Power(powers.aft_left);
        self.rotor_aft_right.cmd = MotorCmd::Power(powers.aft_right);

        self.clamp_cmds();
    }

    #[cfg(feature = "fixed-wing")]
    pub fn set_cmds_from_control_posits(&mut self, posits: &CtrlSfcPosits) {
        self.elevon_left.posit_cmd = posits.elevon_left;
        self.elevon_right.posit_cmd = posits.elevon_right;

        if let Some(mut r) = self.rudder {
            r.posit_cmd = posits.rudder.unwrap_or(0.);
        }

        self.clamp_cmds();
    }

    #[cfg(feature = "quad")]
    pub fn clamp_cmds(&mut self) {
        self.rotor_front_left.cmd.clamp();
        self.rotor_front_right.cmd.clamp();
        self.rotor_aft_left.cmd.clamp();
        self.rotor_aft_right.cmd.clamp();
        // self.ms5.cmd.clamp();
        // self.ms6.cmd.clamp();
    }

    #[cfg(feature = "fixed-wing")]
    pub fn clamp_cmds(&mut self) {
        self.motor_thrust1.cmd.clamp();
        if let Some(m) = &mut self.motor_thrust2 {
            m.cmd.clamp();
        }

        self.elevon_left.clamp();
        self.elevon_right.clamp();

        if let Some(r) = &mut self.rudder {
            r.clamp();
        }
    }

    /// Send commands to all rotors. This uses a single DSHOT command. Assumes power level
    /// to achieve the target RPM is already applied.
    #[cfg(feature = "quad")]
    pub fn send_to_rotors(&mut self, arm_status: ArmStatus, motor_timer: &mut MotorTimer) {
        let mut p1 = 0.;
        let mut p2 = 0.;
        let mut p3 = 0.;
        let mut p4 = 0.;

        let p_fl = self.rotor_front_left.cmd.power();
        let p_fr = self.rotor_front_right.cmd.power();
        let p_al = self.rotor_aft_left.cmd.power();
        let p_ar = self.rotor_aft_right.cmd.power();

        // Map from rotor position to motor number.
        // todo DRY
        // todo: This process doesn't elegantly handle mismapped pins.
        match self.rotor_front_left_hardware {
            MotorServoHardware::Pin1 => {
                p1 = p_fl;
            }
            MotorServoHardware::Pin2 => {
                p2 = p_fl;
            }
            MotorServoHardware::Pin3 => {
                p3 = p_fl;
            }
            MotorServoHardware::Pin4 => {
                p4 = p_fl;
            }
            _ => (), // todo
        }

        match self.rotor_front_right_hardware {
            MotorServoHardware::Pin1 => {
                p1 = p_fr;
            }
            MotorServoHardware::Pin2 => {
                p2 = p_fr;
            }
            MotorServoHardware::Pin3 => {
                p3 = p_fr;
            }
            MotorServoHardware::Pin4 => {
                p4 = p_fr;
            }
            _ => (), // todo
        }

        match self.rotor_aft_left_hardware {
            MotorServoHardware::Pin1 => {
                p1 = p_al;
            }
            MotorServoHardware::Pin2 => {
                p2 = p_al;
            }
            MotorServoHardware::Pin3 => {
                p3 = p_al;
            }
            MotorServoHardware::Pin4 => {
                p4 = p_al;
            }
            _ => (), // todo
        }

        match self.rotor_aft_right_hardware {
            MotorServoHardware::Pin1 => {
                p1 = p_ar;
            }
            MotorServoHardware::Pin2 => {
                p2 = p_ar;
            }
            MotorServoHardware::Pin3 => {
                p3 = p_ar;
            }
            MotorServoHardware::Pin4 => {
                p4 = p_ar;
            }
            _ => (), // todo
        }

        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power(p1, p2, p3, p4, motor_timer);

                self.rotor_front_left.power_setting = p_fl;
                self.rotor_front_right.power_setting = p_fr;
                self.rotor_aft_left.power_setting = p_al;
                self.rotor_aft_right.power_setting = p_ar;
            }
            ArmStatus::Disarmed => {
                dshot::stop_all(motor_timer);

                self.rotor_front_left.power_setting = 0.;
                self.rotor_front_right.power_setting = 0.;
                self.rotor_aft_left.power_setting = 0.;
                self.rotor_aft_right.power_setting = 0.;
            }
        }
    }

    /// Send commands to all thrust motors. This uses a single DSHOT command. Assumes power level
    /// to achieve the target RPM is already applied.
    #[cfg(feature = "fixed-wing")]
    pub fn send_to_motors(&self, arm_status: ArmStatus, motor_timer: &mut MotorTimer) {
        let mut p1 = 0.;
        let mut p2 = 0.;
        let mut p3 = 0.;
        let mut p4 = 0.;
        let mut p5 = 0.;
        let mut p6 = 0.;

        // todo: p5 and p6 are currently unused in our DSHOT setup.

        let p_thrust1 = self.motor_thrust1.cmd.power();

        // Map from rotor position to motor number.
        // todo DRY
        // todo: This process doesn't elegantly handle mismapped pins.
        match self.motor_thrust1_hardware {
            MotorServoHardware::Pin1 => {
                p1 = p_thrust1;
            }
            MotorServoHardware::Pin2 => {
                p2 = p_thrust1;
            }
            MotorServoHardware::Pin3 => {
                p3 = p_thrust1;
            }
            MotorServoHardware::Pin4 => {
                p4 = p_thrust1;
            }
            MotorServoHardware::Pin5 => {
                p5 = p_thrust1;
            }
            MotorServoHardware::Pin6 => {
                p6 = p_thrust1;
            }
        }

        if let Some(motor_thrust2) = &self.motor_thrust2 {
            let p_thrust2 = motor_thrust2.cmd.power();

            // Note: This unwrap assumes the Some/None is synced between `motor_thrust2` and
            // `motor_thrust2_hardware`.
            match self.motor_thrust2_hardware.unwrap() {
                MotorServoHardware::Pin1 => {
                    p1 = p_thrust2;
                }
                MotorServoHardware::Pin2 => {
                    p2 = p_thrust2;
                }
                MotorServoHardware::Pin3 => {
                    p3 = p_thrust2;
                }
                MotorServoHardware::Pin4 => {
                    p4 = p_thrust2;
                }
                MotorServoHardware::Pin5 => {
                    p5 = p_thrust2;
                }
                MotorServoHardware::Pin6 => {
                    p6 = p_thrust2;
                }
            }
        }

        match arm_status {
            ArmStatus::MotorsControlsArmed => {
                dshot::set_power(p1, p2, p3, p4, motor_timer);
            }
            _ => {
                dshot::stop_all(motor_timer);
            }
        }
    }

    #[cfg(feature = "fixed-wing")]
    pub fn send_to_servos(&self, arm_status: ArmStatus, servo_timer: &mut ServoTimer) {
        // todo: In the future, this may apply to quads as well.

        if arm_status == ArmStatus::Disarmed {
            return;
        }

        let range_in_l = if self.elevon_left.reversed {
            (-SERVO_CMD_MIN, -SERVO_CMD_MAX)
        } else {
            (SERVO_CMD_MIN, SERVO_CMD_MAX)
        };

        let range_in_r = if self.elevon_right.reversed {
            (-SERVO_CMD_MIN, -SERVO_CMD_MAX)
        } else {
            (SERVO_CMD_MIN, SERVO_CMD_MAX)
        };

        servo::set_posit(
            self.elevon_left.posit_cmd,
            range_in_l,
            servo_timer,
            // todo: 1 v 2 and L v 2
            servo::ServoWing::S1.tim_channel(),
        );
        servo::set_posit(
            self.elevon_right.posit_cmd,
            range_in_r,
            servo_timer,
            servo::ServoWing::S2.tim_channel(),
        );
    }
}

/// Measurements of RPM. `None` indicates that no reading is available for a given motor.
#[cfg(feature = "quad")]
#[derive(Default)]
pub struct RpmReadings {
    pub front_left: Option<f32>,
    pub front_right: Option<f32>,
    pub aft_left: Option<f32>,
    pub aft_right: Option<f32>,
}

#[cfg(feature = "fixed-wing")]
#[derive(Default)]
pub struct RpmReadings {
    pub thrust1: Option<f32>,
    pub thrust2: Option<f32>,
}

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% power.
/// Used as a quad-specific output from flight control logic. Passed to the motor state,
/// which handles application.
#[cfg(feature = "quad")]
#[derive(Clone, Default)]
pub struct MotorPower {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

#[cfg(feature = "fixed-wing")]
#[derive(Clone, Default)]
pub struct MotorPower {
    pub thrust1: f32,
    /// `None` if this motor isn't present.
    pub thrust2: Option<f32>,
}

/// Holds all 4 RPMs, by position.
/// Used as a quad-specific output from flight control logic. Passed to the motor state,
/// which handles application.
/// todo: Quad-specific, but in `common` due to how we store it in Shared.
#[cfg(feature = "quad")]
#[derive(Default)]
pub struct MotorRpm {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

#[cfg(feature = "fixed-wing")]
#[derive(Default)]
pub struct MotorRpm {
    pub thrust1: f32,
    /// `None` if this motor isn't present.
    pub thrust2: Option<f32>,
}

#[cfg(feature = "fixed-wing")]
#[derive(Default)]
pub struct CtrlSfcPosits {
    pub elevon_left: f32,
    /// `None` if the rudder isn't present.
    pub elevon_right: f32,
    pub rudder: Option<f32>,
}

#[cfg(feature = "fixed-wing")]
impl CtrlSfcPosits {
    pub fn from_mix(mix: &CtrlMix) -> Self {
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

        rudder += mix.yaw;

        let mut result = Self {
            elevon_left,
            elevon_right,
            rudder: Some(rudder), // todo?
        };

        result
    }

    /// Maps to angular accel. Positive means nose-up pitching.
    /// Note: This is located on a non-equiv struct on Quads (RPMs). This is because
    /// on fixed-wing, we map control commands directly to accel, while
    pub fn pitch_delta(&self) -> f32 {
        self.elevon_left + self.elevon_right
    }

    /// Maps to angular accel. Positive means left-wing-up.
    /// (See note on `pitch_delta)`.
    pub fn roll_delta(&self) -> f32 {
        self.elevon_right - self.elevon_left
    }

    pub fn yaw_delta(&self) -> f32 {
        match self.rudder {
            Some(r) => r,
            None => 0.,
        }
    }
}

#[cfg(feature = "quad")]
impl MotorRpm {
    /// Generate RPMs for each motor, from a control mix.
    /// Pitch, roll, and yaw are in RPM difference between the sum of each pair.
    pub fn from_mix(mix: &CtrlMix, front_left_dir: RotationDir) -> Self {
        // let baseline_rpm = estimate_rpm_from_pwr(mix.throttle);
        let baseline_rpm = mix.throttle; // todo temp

        let mut front_left = baseline_rpm;
        let mut front_right = baseline_rpm;
        let mut aft_left = baseline_rpm;
        let mut aft_right = baseline_rpm;

        // inputs are a differential between opposing rotor pairs.
        let half_pitch = mix.pitch / 2.;
        let half_roll = mix.roll / 2.;
        let half_yaw = mix.yaw / 2.;

        // Nose down for positive pitch.
        front_left -= half_pitch;
        front_right -= half_pitch;
        aft_left += half_pitch;
        aft_right += half_pitch;

        // Left side up for positive roll
        front_left += half_roll;
        front_right -= half_roll;
        aft_left += half_roll;
        aft_right -= half_roll;

        // Assumes positive yaw from the IMU means clockwise.
        // If props rotate in, front-left/aft-right rotors induce a CCW torque on the aircraft.
        // If props rotate out, these same rotors induce a CW torque.
        // This code assumes props rotate inwards towards the front and back ends.
        let yaw = if front_left_dir == RotationDir::Clockwise {
            half_yaw
        } else {
            -half_yaw
        };

        front_left += yaw;
        front_right -= yaw;
        aft_left -= yaw;
        aft_right += yaw;

        Self {
            front_left,
            front_right,
            aft_left,
            aft_right,
        }
    }

    /// Motor pair delta. Maps to angular accel. Positive means nose-up pitching.
    pub fn pitch_delta(&self) -> f32 {
        self.front_left + self.front_right - self.aft_left - self.aft_right
    }

    /// Motor pair delta. Maps to angular accel. Positive means left-wing-up.
    pub fn roll_delta(&self) -> f32 {
        self.front_left + self.aft_left - self.front_right - self.aft_right
    }

    /// Motor pair delta. Maps to angular accel. Positive means rotate clockwise when
    /// looking down from above.
    pub fn yaw_delta(&self, frontleft_aftright_dir: RotationDir) -> f32 {
        match frontleft_aftright_dir {
            RotationDir::Clockwise => {
                self.front_right + self.aft_left - self.front_left - self.aft_right
            }
            RotationDir::CounterClockwise => {
                self.front_left + self.aft_right - self.front_right - self.aft_left
            }
        }
    }
}

#[cfg(feature = "quad")]
impl MotorPower {
    /// Generate power for each motor, from a control mix.
    /// Pitch, roll, and yaw are in RPM difference between the sum of each pair.
    /// todo: DRY with equiv RPM command.
    pub fn from_mix(mix: &CtrlMix, front_left_dir: RotationDir) -> Self {
        let baseline_pwr = mix.throttle;

        let mut front_left = baseline_pwr;
        let mut front_right = baseline_pwr;
        let mut aft_left = baseline_pwr;
        let mut aft_right = baseline_pwr;

        // inputs are a differential between opposing rotor pairs.
        let half_pitch = mix.pitch / 2.;
        let half_roll = mix.roll / 2.;
        let half_yaw = mix.yaw / 2.;

        // Nose down for positive pitch.
        front_left -= half_pitch;
        front_right -= half_pitch;
        aft_left += half_pitch;
        aft_right += half_pitch;

        // Left side up for positive roll
        front_left += half_roll;
        front_right -= half_roll;
        aft_left += half_roll;
        aft_right -= half_roll;

        // Assumes positive yaw from the IMU means clockwise.
        // If props rotate in, front-left/aft-right rotors induce a CCW torque on the aircraft.
        // If props rotate out, these same rotors induce a CW torque.
        // This code assumes props rotate inwards towards the front and back ends.
        let yaw = if front_left_dir == RotationDir::Clockwise {
            half_yaw
        } else {
            -half_yaw
        };

        front_left += yaw;
        front_right -= yaw;
        aft_left -= yaw;
        aft_right += yaw;

        Self {
            front_left,
            front_right,
            aft_left,
            aft_right,
        }
    }
}

#[cfg(feature = "quad")]
impl MotorPower {
    /// Motor pair delta. Maps to angular accel. Positive means nose-up pitching.
    pub fn pitch_delta(&self) -> f32 {
        self.front_left + self.front_right - self.aft_left - self.aft_right
    }

    /// Motor pair delta. Maps to angular accel. Positive means left-wing-up.
    pub fn roll_delta(&self) -> f32 {
        self.front_left + self.aft_left - self.front_right - self.aft_right
    }

    /// Motor pair delta. Maps to angular accel. Positive means rotate clockwise when
    /// looking down from above.
    pub fn yaw_delta(&self, frontleft_aftright_dir: RotationDir) -> f32 {
        match frontleft_aftright_dir {
            RotationDir::Clockwise => {
                self.front_right + self.aft_left - self.front_left - self.aft_right
            }
            RotationDir::CounterClockwise => {
                self.front_left + self.aft_right - self.front_right - self.aft_left
            }
        }
    }

    /// Calculates total power. Used to normalize individual rotor powers when setting total
    /// power, eg from a thrust setting.
    pub fn _total(&self) -> f32 {
        self.front_left + self.front_right + self.aft_left + self.aft_right
    }

    /// Calculates total power. Used to normalize individual rotor powers when setting total
    /// power, eg from a thrust setting.
    pub fn _scale_all(&mut self, scaler: f32) {
        self.front_left *= scaler;
        self.front_right *= scaler;
        self.aft_left *= scaler;
        self.aft_right *= scaler;
    }
}
