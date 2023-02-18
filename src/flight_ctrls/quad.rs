//! This module contains flight control code for quadcopters.

// todo: Feed foward - https://drones.stackexchange.com/questions/495/what-does-feed-forward-do-and-how-does-it-work
// todo: Feed foward is an adjustment (to P term?) proportional to (change in?) stick position. It's used
// todo to make maneuver initiation more responsive. Anticapates control. Perhaps something simple like
// todo immediately initiating a power adjustment in response to control actuation? (derivative of ctrl position?)

use core::f32::consts::TAU;

use crate::{
    control_interface::InputModeSwitch,
    dshot,
    safety::ArmStatus,
    setup::MotorTimer,
    state::StateVolatile,
    system_status::{SensorStatus, SystemStatus},
    util, DT_FLIGHT_CTRLS,
};

use super::{
    common::{CtrlMix, InputMap, Motor, MotorRpm, RpmReadings},
    pid,
};

// // Min power setting for any individual rotor at idle setting.
// const MIN_ROTOR_POWER: f32 = 0.03;
//

// Min RPM setting for any individual rotor at idle setting.
const MIN_ROTOR_RPM: f32 = 100.; // todo: Finda good value.

// Max power setting for any individual rotor at idle setting.
pub const MAX_ROTOR_POWER: f32 = 1.;

// todo: Variabel/struct field found from cal routine that is power to hover.

// Our maneuverability clamps are different from normal throttle settings: They're used
// to reduce the risk and severity of individual rotors clamping due to throttle settings that
// are too high or too low. They reduce user throttle authority, but provide more predictable
// responses when maneucvering near min and max throttle
// These are power settings, not RPM.
pub const THROTTLE_MAX_MNVR_CLAMP: f32 = 0.80;
// todo: You should probably disable the min maneuver clamp when on the ground (how to check?)
// and have it higher otherwise.
pub const THROTTLE_MIN_MNVR_CLAMP: f32 = 0.06;

// Don't execute the calibration procedure from below this altitude, in meters AGL, eg for safety.
// todo: Calibration unimplemented
// const MIN_CAL_ALT: f32 = 6.;

impl Default for InputMap {
    // todo: move deafult impls to their respective moudles (quad, flying wing)?
    fn default() -> Self {
        Self {
            pitch_rate: (-10., 10.),
            roll_rate: (-10., 10.),
            yaw_rate: (-10., 10.),
            throttle_clamped: (THROTTLE_MIN_MNVR_CLAMP, THROTTLE_MAX_MNVR_CLAMP),
            pitch_angle: (-TAU / 4., TAU / 4.),
            roll_angle: (-TAU / 4., TAU / 4.),
            alt_commanded_offset_msl: (0., 100.),
            alt_commanded_agl: (0.5, 8.),
        }
    }
}

/// Specify the rotor by position. Used in power application code.
/// repr(u8) is for use in Preflight.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum RotorPosition {
    FrontLeft = 0,
    FrontRight = 1,
    AftLeft = 2,
    AftRight = 3,
}

/// Mode used for control inputs. These are the three "industry-standard" modes.
#[derive(Clone, Copy, PartialEq)]
pub enum InputMode {
    /// Rate, also know as manual, hard or Acro. Attitude and power stay the same after
    /// releasing controls.
    Acro,
    /// Attitude also know as self-level, angle, or Auto-level. Attitude resets to a level
    /// hover after releasing controls.  When moving the
    /// roll/pitch stick to its maximum position, the drone will also reach the maximum angle
    /// it’s allowed to tilt (defined by the user), and it won’t flip over. As you release the
    /// stick back to centre, the aircraft will also return to its level position.
    /// We use attitude mode as a no-GPS fallback.
    Attitude,
    // GPS-hold, also known as Loiter. Maintains a specific position.
    /// In `Command` mode, the device loiters when idle. Otherwise, it flies at specific velocities,
    /// and altitudes commanded by the controller. Allows for precise control, including in confined
    /// spaces.
    Command,
    // /// This mode is easy stable, and designed to make control easy, including in confined spaces.
    // /// Similar to `Command` mode, it loiters when idle. It uses an internal model of
    // /// todo: Same as Command mode? Consolidate?
    // VideoGame,
}

impl Default for InputMode {
    fn default() -> Self {
        Self::Acro
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)] // u8 repr for serializing via USB.
pub enum RotationDir {
    Clockwise = 0,
    CounterClockwise = 1,
}

/// Maps servos to wing position, and related details.
/// Map rotor positions to connection numbers to ESC, and motor directions. Positions are associated with flight controls;
/// numbers are associated with motor control. Also allows configuring rotor direcction, using front-left,
/// and aft-right rotors as our stake.
pub struct ControlMapping {
    pub m1: RotorPosition,
    pub m2: RotorPosition,
    pub m3: RotorPosition,
    pub m4: RotorPosition,
    /// It's common to arbitrarily wire motors to the ESC. Reverse each from its
    /// default direction, as required.
    pub m1_reversed: bool,
    pub m2_reversed: bool,
    pub m3_reversed: bool,
    pub m4_reversed: bool,
    pub frontleft_aftright_dir: RotationDir,
}

impl Default for ControlMapping {
    fn default() -> Self {
        Self {
            m1: RotorPosition::AftRight,
            m2: RotorPosition::FrontRight,
            m3: RotorPosition::AftLeft,
            m4: RotorPosition::FrontLeft,
            m1_reversed: false,
            m2_reversed: false,
            m3_reversed: false,
            m4_reversed: false,
            frontleft_aftright_dir: RotationDir::Clockwise,
        }
    }
}

impl ControlMapping {
    pub fn motor_from_position(&self, position: RotorPosition) -> Motor {
        // todo: This assumes each motor maps to exactly one position. We probably
        // todo should have some constraint to enforce this.
        if self.m1 == position {
            Motor::M1
        } else if self.m2 == position {
            Motor::M2
        } else if self.m3 == position {
            Motor::M3
        } else {
            Motor::M4
        }
    }
}

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% power.
#[derive(Clone, Default)]
pub struct MotorPower {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

impl MotorRpm {
    /// Generate power settings for each motor, from RPM commands.
    /// Pitch, roll, and yaw are in RPM difference between the sum of each pair.
    pub fn from_cmds(mix: &CtrlMix, front_left_dir: RotationDir) -> Self {
        // let baseline_rpm = estimate_rpm_from_pwr(mix.throttle);
        let baseline_rpm = 100.; // todo temp to get to compile!

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

        let mut result = Self {
            front_left,
            front_right,
            aft_left,
            aft_right,
        };

        result.clamp_individual_rotors();

        result
    }

    /// Clamp rotor speeds by an RPM idle.
    fn clamp_individual_rotors(&mut self) {
        if self.front_left < MIN_ROTOR_RPM {
            self.front_left = MIN_ROTOR_RPM;
        }

        if self.front_right < MIN_ROTOR_RPM {
            self.front_right = MIN_ROTOR_RPM;
        }

        if self.aft_left < MIN_ROTOR_RPM {
            self.aft_left = MIN_ROTOR_RPM;
        }

        if self.aft_right < MIN_ROTOR_RPM {
            self.aft_right = MIN_ROTOR_RPM;
        }
    }

    /// Send this power command to the rotors, after converting to `MotorPower`,
    /// via a power-to-RPM PID.
    pub fn send_to_motors(
        &self,
        pid_coeffs: &pid::MotorCoeffs,
        pids: &pid::MotorPidGroup,
        prev_pwr: &mut MotorPower,
        rpm_readings: &RpmReadings,
        mapping: &ControlMapping,
        timer: &mut MotorTimer,
        arm_status: ArmStatus,
    ) {
        let fl = pid::run(
            self.front_left,
            rpm_readings.front_left.unwrap_or(0.),
            &pids.front_left,
            pid_coeffs.p_front_left,
            pid_coeffs.i_front_left,
            0.,
            None,
            DT_FLIGHT_CTRLS,
        );

        let fr = pid::run(
            self.front_right,
            rpm_readings.front_right.unwrap_or(0.),
            &pids.front_right,
            pid_coeffs.p_front_right,
            pid_coeffs.i_front_right,
            0.,
            None,
            DT_FLIGHT_CTRLS,
        );

        let al = pid::run(
            self.aft_left,
            rpm_readings.aft_left.unwrap_or(0.),
            &pids.aft_left,
            pid_coeffs.p_aft_left,
            pid_coeffs.i_aft_left,
            0.,
            None,
            DT_FLIGHT_CTRLS,
        );

        let ar = pid::run(
            self.aft_right,
            rpm_readings.aft_right.unwrap_or(0.),
            &pids.aft_right,
            pid_coeffs.p_aft_right,
            pid_coeffs.i_aft_right,
            0.,
            None,
            DT_FLIGHT_CTRLS,
        );

        let power = MotorPower {
            front_left: prev_pwr.front_left + fl.out(),
            // front_right: prev_pwr.front_right + fr.out(),
            // aft_left: prev_pwr.aft_left + al.out(),
            aft_right: prev_pwr.aft_right + ar.out(),
            // todo temp so it doesn't accidentally take off while experimenting.
            // front_left: 0.,
            front_right: 0.,
            aft_left: 0.,
        };

        power.set(mapping, timer, arm_status);

        *prev_pwr = power;
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

impl MotorPower {
    /// Convert rotor position to its associated power setting.
    fn by_rotor_num(&self, mapping: &ControlMapping) -> (f32, f32, f32, f32) {
        // todo: DRY
        let p1 = match mapping.m1 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        let p2 = match mapping.m2 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        let p3 = match mapping.m3 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        let p4 = match mapping.m4 {
            RotorPosition::FrontLeft => self.front_left,
            RotorPosition::FrontRight => self.front_right,
            RotorPosition::AftLeft => self.aft_left,
            RotorPosition::AftRight => self.aft_right,
        };

        (p1, p2, p3, p4)
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

    /// Clamp rotor speeds to 1.0 max power.
    pub fn clamp_individual_rotors(&mut self) {
        if self.front_left > MAX_ROTOR_POWER {
            self.front_left = MAX_ROTOR_POWER;
        }

        if self.front_right > MAX_ROTOR_POWER {
            self.front_right = MAX_ROTOR_POWER;
        }

        if self.aft_left > MAX_ROTOR_POWER {
            self.aft_left = MAX_ROTOR_POWER;
        }

        if self.aft_right > MAX_ROTOR_POWER {
            self.aft_right = MAX_ROTOR_POWER;
        }
    }

    /// Send this power command to the rotors
    pub fn set(&self, mapping: &ControlMapping, timer: &mut MotorTimer, arm_status: ArmStatus) {
        let (p1, p2, p3, p4) = self.by_rotor_num(mapping);

        match arm_status {
            ArmStatus::Armed => {
                dshot::set_power(p1, p2, p3, p4, timer);
            }
            ArmStatus::Disarmed => {
                dshot::stop_all(timer);
            }
        }
    }
}

/// Calculate the horizontal target velocity (m/s), for a given distance (m) from a point horizontally.
pub fn enroute_speed_hor(dist: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if dist > 20. {
        max_v
    } else if dist > 10. {
        2.0_f32.max(max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    }
}

/// Calculate the vertical target velocity (m/s), for a given distance (m) from a point vertically.
/// `dist` is postive if the aircraft altitude is below the set point; otherwise negative.
pub fn enroute_speed_ver(dist: f32, max_v: f32, z_agl: f32) -> f32 {
    // todo: fill this out. LUT?

    if z_agl < 7. {
        let mut result = 3.0_f32.max(max_v);

        if dist < 0. {
            result *= -1.;
        }
    }

    let dist_abs = util::abs(dist);

    let mut result = if dist_abs > 20. {
        max_v
    } else if dist_abs > 10. {
        2.0_f32.max(max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    };

    if dist < 0. {
        result *= -1.;
    }
    result
}

/// Calculate the landing vertical velocity (m/s), for a given height  (m) above the ground.
pub fn landing_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5;
    }
    (height / 4.).max(max_v)
}

/// Calculate the takeoff vertical velocity (m/s), for a given height (m) above the ground.
pub fn takeoff_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.;
    }
    (height / 4. + 0.01).max(max_v)
}

pub struct UnsuitableParams {}

// /// Execute a profile designed to test PID and motor gain coefficients; update them.
// pub fn calibrate_coeffs(params: &Params) -> Result<(), UnsuitableParams> {
//     if params.tof_alt.unwrap_or(0.) < MIN_CAL_ALT {
//         return Err(UnsuitableParams {});
//     }
//
//     Ok(())
// }

pub fn set_input_mode(
    input_mode_control: InputModeSwitch,
    state_volatile: &mut StateVolatile,
    system_status: &SystemStatus,
) {
    state_volatile.input_mode_switch = input_mode_control; // todo: Do we need or use this field?

    state_volatile.input_mode = match input_mode_control {
        InputModeSwitch::Acro => InputMode::Acro,
        InputModeSwitch::AttitudeCommand => {
            if system_status.gps == SensorStatus::Pass {
                InputMode::Command
            } else {
                InputMode::Attitude
            }
        }
    }
}
