//! This module contains flight control code not specific to an aircraft design category.
//! It is mostly types.

use crate::{
    util::map_linear,
    safety::ArmStatus,
    setup::{MotorTimer, ServoTimer},
    protocols::dshot,
    flight_ctrls::pid,
    DT_FLIGHT_CTRLS,
};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "quad")] {
        use crate::flight_ctrls::{MotorRpm, MotorPower}
    } else {

    }
}

use lin_alg2::f32::Quaternion;
use crate::protocols::dshot::Motor;

// Our input ranges for the 4 controls
const PITCH_IN_RNG: (f32, f32) = (-1., 1.);
const ROLL_IN_RNG: (f32, f32) = (-1., 1.);
const YAW_IN_RNG: (f32, f32) = (-1., 1.);
const THROTTLE_IN_RNG: (f32, f32) = (0., 1.);

const MOTOR_CMD_MIN: f32 = 0.;
const MOTOR_CMD_MAX: f32 = 1.;
const MOTOR_RPM_MIN: f32 = 500.;
const MOTOR_RPM_MAX: f32 = 6_000.; // todo: PRobably depends on motors.
const SERVO_MIN: f32 = 0.;
const SERVO_MAX: f32 = 1.;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum MotorCmd {
    Power(f32),
    Rpm(f32),
}

impl Default for MotorCmd {
    fn default() -> Self {
        Self::Power(0.)
    }
}

impl MotorCmd {
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
                if *c < MOTOR_RPM_MIN {
                    *c = MOTOR_RPM_MIN;
                } else if *c > MOTOR_RPM_MAX {
                    *c = MOTOR_RPM_MAX;
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
    pub rpm_reading: Option<f32>,
    // pub dir: RotationDir, // todo: Do we want this?
    /// Reversed is in relation to the 3-wire motor brushless wiring. This software setting
    /// allows the wires to be connected in any order, and compensated for in software. (Eg by
    /// sending a DSHOT command to reverse the direction appropriately)
    pub reversed: bool,
}

/// State of an individual servo.
#[derive(Clone, Copy, PartialEq, Default)]
pub struct ServoState {
    /// Commanded position
    pub posit_cmd: f32,
}

// impl ServoState {
//     pub fn send_to_servo(arm_status: ArmStatus, servo_timer: &mut ServoTimer) {
//
//     }
// }

impl ServoState {
    pub fn clamp(&mut self) {
        if self.posit_cmd < SERVO_MIN {
            self.posit_cmd = SERVO_MIN;
        } else if self.posit_cmd > SERVO_MAX {
            self.posit_cmd = SERVO_MAX;
        }
    }
}

/// A possible function for a given motor/servo pin
/// Interior values for rotors are RPM. Interior values for servos
/// are servos positions. (On a scale of -1. to 1.)
// #[derive(Clone, Copy, PartialEq)]
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
    /// Apply the motor or servo command.
    // pub fn send_to_motor_servo(&mut self, arm_status: ArmStatus, motor_timer: &mut MotorTimer, servo_timer: &mut ServoTimer) {
    //     match Self {
    //         Self::Unused => (),
    //         Self::RotorFrontLeft(m) => {
    //             m.cmd.send_to_motor(motor_timer, arm_status);
    //         }
    //         Self::RotorFrontRight(m) => {
    //             m.cmd.send_to_motor(motor_timer, arm_status);
    //         }
    //         Self::RotorAftLeft(m) => {
    //             m.cmd.send_to_motor(motor_timer, arm_status);
    //         }
    //         Self::RotorAftRight(m) => {
    //             m.cmd.send_to_motor(motor_timer, arm_status);
    //         }
    //         Self::ThrustMotor1(m) => {
    //             m.cmd.send_to_motor(motor_timer, arm_status);
    //         }
    //         Self::ThrustMotor2(m) => {
    //             m.cmd.send_to_motor(motor_timer, arm_status);
    //         }
    //         Self::ElevonLeft(s) => {
    //             s.cmd.send_to_servo(servo_timer, arm_status);
    //         }
    //         Self::ElevonRight(s) => {
    //             s.cmd.send_to_servo(servo_timer, arm_status);
    //         }
    //     }
    // }

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
/// to hardware pins.
#[derive(Clone, Copy)]
pub enum MotorServoHardware {
    Pin1,
    Pin2,
    Pin3,
    Pin4,
    Pin5,
    Pin6,
}

/// Describes the function of all motors and servos.  Is based on the pin connections. Each pin
/// can be a motor or servo. Doesn't directly deliniate quadcopter vice fixed-wing.
///
/// Note: there are some restrictions based on timer-pin mappings, that are different between
/// G4 and H7.
/// todo: Rename A/R
#[cfg(feature = "quad")]
pub struct MotorServoState {
    // pub ms1: MotorServoRole,
    // pub ms2: MotorServoRole,
    // pub ms3: MotorServoRole,
    // pub ms4: MotorServoRole,
    // pub ms5: MotorServoRole,
    // pub ms6: MotorServoRole,

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
    pub motor_thrust1_hardware: MotorState,
    pub motor_thrust2_hardware: Option<MotorState>,
    pub elevon_left_hardware: ServoState,
    pub elevon_right_hardware: ServoState,
    pub rudder_hardware: Option<ServoState>,
    pub servo_aux_1_hardware: Option<ServoState>,
    pub servo_aux_2_hardware: Option<ServoState>,

    pub motor_thrust1: MotorState,
    pub motor_thrust2: Option<MotorState>,
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
            // ms1: MotorServoRole::RotorFrontLeft(Default::default()),
            // ms2: MotorServoRole::RotorFrontRight(Default::default()),
            // ms3: MotorServoRole::RotorAftLeft(Default::default()),
            // ms4: MotorServoRole::RotorAftRight(Default::default()),
            // ms5: MotorServoRole::Unused,
            // ms6: MotorServoRole::Unused,
            // rotor_front_left: MotorState::default(),
            // rotor_front_right: MotorState::default(),
            // rotor_aft_left: MotorState::default(),
            // rotor_aft_right: MotorState::default(),
            // servo

            rotor_front_left_hardware: MotorServoHardware,
            rotor_front_right_hardware: MotorServoHardware,
            rotor_aft_left_hardware: MotorServoHardware,
            rotor_aft_right_hardware: MotorServoHardware,
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
            //     ms1: MotorServoRole::RotorFrontLeft(Default::default()),
            //     ms2: MotorServoRole::Unused,
            //     ms3: MotorServoRole::ElevonLeft(Default::default()),
            //     ms4: MotorServoRole::ElevonRight(Default::default()),
            //     ms5: MotorServoRole::Unused,
            //     ms6: MotorServoRole::Unused,

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

    /// Populate command state from rotor RPMs.
    #[cfg(feature = "quad")]
    pub fn set_cmds_from_rpms(&mut self, rpms: &MotorRpms) {
        self.rotor_front_left.cmd = MotorCmd::Rpm(rpms.front_left);
        self.rotor_front_right.cmd = MotorCmd::Rpm(rpms.front_right);
        self.rotor_aft_left.cmd = MotorCmd::Rpm(rpms.aft_left);
        self.rotor_aft_right.cmd = MotorCmd::Rpm(rpms.aft_right);
    }

    /// Populate commands from motor powers.
    #[cfg(feature = "quad")]
    pub fn set_cmds_from_power(&mut self, powers: &MotorPower) {
        self.rotor_front_left.cmd = MotorCmd::Power(powers.front_left);
        self.rotor_front_right.cmd = MotorCmd::Power(powers.front_right);
        self.rotor_aft_left.cmd = MotorCmd::Power(powers.aft_left);
        self.rotor_aft_right.cmd = MotorCmd::Power(powers.aft_right);
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

    /// Send commands to all rotors. This uses a single DSHOT command.
    /// todo: Note - we don't need PID coeffs adn readings if sending power.
    #[cfg(feature = "quad")]
    pub fn send_to_rotors(
        &self,
        arm_status: ArmStatus,
        rpm_readings: &RpmReadings,
        pid_group: &pid::PidGroup,
        pid_coeffs: &pid::MotorCoeffs,
        motor_timer: &mut MotorTimer,
    ) {
        // todo: Temp hard-coded mappinsg.

        // todo: Lots of DRY.
        let cmd_front_left = match self.rotor_front_left.cmd {
            MotorCmd::Power(p) => p,
            MotorCmd::Rpm(r) => {
                match rpm_readings.front_left {
                    Some(reading) => {
                        pid::run(
                            r,
                            reading,
                            &pid_group.front_left,
                            pid_coeffs.p_front_left,
                            pid_coeffs.i_front_left,
                            0.,
                            None,
                            DT_FLIGHT_CTRLS,
                        )
                            .out()
                            + prev_pwr.front_left
                    }
                    None => 0.,
                }
            }
        };

        let cmd_front_right = match self.rotor_front_right.cmd {
            MotorCmd::Power(p) => p,
            MotorCmd::Rpm(r) => {
                match rpm_readings.front_right {
                    Some(reading) => {
                        pid::run(
                            r,
                            reading,
                            &pid_group.front_right,
                            pid_coeffs.p_front_right,
                            pid_coeffs.i_front_right,
                            0.,
                            None,
                            DT_FLIGHT_CTRLS,
                        )
                            .out()
                            + prev_pwr.front_right
                    }
                    None => 0.,
                }
            }
        };

        let cmd_aft_left = match self.rotor_aft_left.cmd {
            MotorCmd::Power(p) => p,
            MotorCmd::Rpm(r) => {
                match rpm_readings.aft_left {
                    Some(reading) => {
                        pid::run(
                            r,
                            reading,
                            &pid_group.aft_left,
                            pid_coeffs.p_aft_left,
                            pid_coeffs.i_aft_left,
                            0.,
                            None,
                            DT_FLIGHT_CTRLS,
                        )
                            .out()
                            + prev_pwr.aft_left
                    }
                    None => 0.,
                }
            }
        };

        let cmd_aft_right = match self.rotor_aft_right.cmd {
            MotorCmd::Power(p) => p,
            MotorCmd::Rpm(r) => {
                match rpm_readings.aft_right {
                    Some(reading) => {
                        pid::run(
                            r,
                            reading,
                            &pid_group.aft_right,
                            pid_coeffs.p_aft_right,
                            pid_coeffs.i_aft_right,
                            0.,
                            None,
                            DT_FLIGHT_CTRLS,
                        )
                            .out()
                            + prev_pwr.aft_right
                    }
                    None => 0.,
                }
            }
        };

        let mut p1 = 0.;
        let mut p2 = 0.;
        let mut p3 = 0.;
        let mut p4 = 0.;

        // todo DRY
        // todo: This process doesn't elegantly handle mismapped pins.
        match self.rotor_front_left_hardware {
            MotorServoHardware::Pin1 => {
                p1 = cmd_front_left;
            }
            MotorServoHardware::Pin2 => {
                p2 = cmd_front_left;
            }
            MotorServoHardware::Pin3 => {
                p3 = cmd_front_left;
            }
            MotorServoHardware::Pin4 => {
                p4 = cmd_front_left;
            }
            _ => (), // todo
        }

        match self.rotor_front_right_hardware {
            MotorServoHardware::Pin1 => {
                p1 = cmd_front_right;
            }
            MotorServoHardware::Pin2 => {
                p2 = cmd_front_right;
            }
            MotorServoHardware::Pin3 => {
                p3 = cmd_front_right;
            }
            MotorServoHardware::Pin4 => {
                p4 = cmd_front_right;
            }
            _ => (), // todo
        }

        match self.rotor_aft_left_hardware {
            MotorServoHardware::Pin1 => {
                p1 = cmd_aft_left;
            }
            MotorServoHardware::Pin2 => {
                p2 = cmd_aft_left;
            }
            MotorServoHardware::Pin3 => {
                p3 = cmd_aft_left;
            }
            MotorServoHardware::Pin4 => {
                p4 = cmd_aft_left;
            }
            _ => (), // todo
        }

        match self.rotor_aft_right_hardware {
            MotorServoHardware::Pin1 => {
                p1 = cmd_aft_right;
            }
            MotorServoHardware::Pin2 => {
                p2 = cmd_aft_right;
            }
            MotorServoHardware::Pin3 => {
                p3 = cmd_aft_right;
            }
            MotorServoHardware::Pin4 => {
                p4 = cmd_aft_right;
            }
            _ => (), // todo
        }

        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power(p1, p2, p3, p4, motor_timer);
            }
            ArmStatus::Disarmed => {
                dshot::stop_all(timer);
            }
        }
    }
}

/// Measurements of RPM. `None` indicates that no reading is available for a given motor.
/// todo: Quad-specific, but in `common` due to how we store it in Shared.
#[derive(Default)]
pub struct RpmReadings {
    pub front_left: Option<f32>,
    pub front_right: Option<f32>,
    pub aft_left: Option<f32>,
    pub aft_right: Option<f32>,
}

// pub struct RpmMissingError {}

// impl RpmReadings {
//     pub fn to_rpms(&self) -> Result<MotorRpm, RpmMissingError> {
//         let e = Err(RpmMissingError {});
//
//         let front_left = match self.front_left {
//             Some(rpm) => rpm,
//             None => return e,
//         };
//
//         let front_right = match self.front_right {
//             Some(rpm) => rpm,
//             None => return e,
//         };
//
//         let aft_left = match self.aft_left {
//             Some(rpm) => rpm,
//             None => return e,
//         };
//
//         let aft_right = match self.aft_right {
//             Some(rpm) => rpm,
//             None => return e,
//         };
//
//         Ok(MotorRpm {
//             front_left,
//             front_right,
//             aft_left,
//             aft_right,
//         })
//     }
// }

/// Maps manual control inputs (range 0. to 1. or -1. to 1.) to velocities, rotational velocities etc
/// for various flight modes. The values are for full input range.
/// Note that defaults are defined in the `quad` and `fixed-wing` modules.
pub struct InputMap {
    /// Pitch velocity commanded, (Eg Acro mode). radians/sec
    pub pitch_rate: (f32, f32),
    /// Pitch velocity commanded (Eg Acro mode)
    pub roll_rate: (f32, f32),
    /// Yaw velocity commanded (Eg Acro mode)
    pub yaw_rate: (f32, f32),
    #[cfg(feature = "quad")]
    /// Throttle setting, clamped to leave room for maneuvering near the limits.
    pub throttle_clamped: (f32, f32),
    #[cfg(feature = "quad")]
    /// Pitch angle commanded (Eg Attitude mode) // radians from vertical
    pub pitch_angle: (f32, f32),
    #[cfg(feature = "quad")]
    /// Roll angle commanded (Eg Attitude mode)
    pub roll_angle: (f32, f32),
    /// When a stick (eg throttle) is mapped to a commanded baro altitude.
    /// Offset MSL is MSL, but 0 maps to launch alt
    pub alt_commanded_offset_msl: (f32, f32),
    /// When a stick (eg throttle) is mapped to a commanded AGL altitude.
    pub alt_commanded_agl: (f32, f32),
}

impl InputMap {
    /// Convert from control inputs to radians/s.
    pub fn calc_pitch_rate(&self, input: f32) -> f32 {
        map_linear(input, PITCH_IN_RNG, self.pitch_rate)
    }

    pub fn calc_roll_rate(&self, input: f32) -> f32 {
        map_linear(input, ROLL_IN_RNG, self.roll_rate)
    }

    pub fn calc_yaw_rate(&self, input: f32) -> f32 {
        map_linear(input, YAW_IN_RNG, self.yaw_rate)
    }

    #[cfg(feature = "quad")]
    pub fn calc_manual_throttle(&self, input: f32) -> f32 {
        map_linear(input, THROTTLE_IN_RNG, self.throttle_clamped)
    }

    #[cfg(feature = "quad")]
    pub fn calc_pitch_angle(&self, input: f32) -> f32 {
        map_linear(input, PITCH_IN_RNG, self.pitch_angle)
    }

    #[cfg(feature = "quad")]
    pub fn calc_roll_angle(&self, input: f32) -> f32 {
        map_linear(input, ROLL_IN_RNG, self.roll_angle)
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum AltType {
    /// Above ground level (eg from a TOF sensor)
    Agl,
    /// Mean sea level (eg from GPS or baro)
    Msl,
}

#[derive(Clone, Default)]
/// We use this to store control commands, that map directly to motor power (quads), or
/// elevon (and optionally rudder or differential engine/speedbrake power) on fixed-wing. Similar to `CtrlInputs`,
/// but without the options, and for a different use.
///
/// For quads, pitch, roll, and yaw are in RPM difference between the sum of each pair.
/// todo: For now, throttle is an absolute power setting on quads.
/// For fixed-wing, they're in arbitrary units, due to being unable to measure servo position.
pub struct CtrlMix {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub throttle: f32,
}

/// Stores inputs to the system. `pitch`, `yaw`, and `roll` are in range -1. to +1.
/// `thrust` is in range 0. to 1. Corresponds to stick positions on a controller, but can
/// also be used as a model for autonomous flight.
/// The interpretation of these depends on the current input mode.
/// These inputs, (if directly from flight control radio inputs), are translated from raw inputs from the radio
/// to -1. to 1. (0. to 1. for thrust)
///
/// A value of `None` indicates no value is commanded.
#[derive(Clone, Default)]
pub struct CtrlInputs {
    /// Acro mode: Change pitch angle
    /// Attitude mode: Command forward and aft motion
    pub pitch: Option<f32>,
    /// Acro mode: Change roll angle
    /// Attitude mode: Command left and right motion
    pub roll: Option<f32>,
    /// Yaw, in either mode
    pub yaw: Option<f32>,
    /// Acro mode: Change overall power (Altitude, or speed depending on orientation)
    /// Attitude mode: Change altitude
    pub throttle: Option<f32>,
}

/// Command a quaternion attitude, part of all of a Euler angle attitude, or neither.
/// If both a quaternion and euler angle are present, favor the quaternion.
#[derive(Default)]
pub struct AttitudeCommanded {
    pub quat: Option<Quaternion>,
    pub pitch: Option<f32>,
    pub roll: Option<f32>,
    pub yaw: Option<f32>,
}

// /// Command one or more angular rates.
// #[derive(Default)]
// pub struct RatesCommanded {
//     pub pitch: Option<f32>,
//     pub roll: Option<f32>,
//     pub yaw: Option<f32>,
// }
