//! This module contains flight control code for quadcopters.

// todo: Feed foward - https://drones.stackexchange.com/questions/495/what-does-feed-forward-do-and-how-does-it-work
// todo: Feed foward is an adjustment (to P term?) proportional to (change in?) stick position. It's used
// todo to make maneuver initiation more responsive. Anticapates control. Perhaps something simple like
// todo immediately initiating a power adjustment in response to control actuation? (derivative of ctrl position?)

use stm32_hal2::{
    dma::Dma,
    pac::{DMA1, TIM2, TIM3},
    timer::Timer,
};

use crate::{control_interface::InputModeSwitch, dshot, safety::ArmStatus, util, StateVolatile};

use super::common::Params;

use defmt::println;

use cmsis_dsp_sys as dsp_sys;

use cfg_if::cfg_if;

// Min power setting for any individual rotor at idle setting.
const MIN_ROTOR_POWER: f32 = 0.03;

// Max power setting for any individual rotor at idle setting.
pub const MAX_ROTOR_POWER: f32 = 1.;

// todo: Variabel/struct field found from cal routine that is power to hover.

// Our maneuverability clamps are different from normal throttle settings: They're used
// to reduce the risk and severity of individual rotors clamping due to throttle settings that
// are too high or too low. They reduce user throttle authority, but provide more predictable
// responses when maneucvering near min and max throttle
pub const THROTTLE_MAX_MNVR_CLAMP: f32 = 0.80;
// todo: You should probably disable the min maneuver clamp when on the ground (how to check?)
// and have it higher otherwise.
pub const THROTTLE_MIN_MNVR_CLAMP: f32 = 0.06;

// Even if PID output for a given axis is higher than this, don't allow more than this
// half-pair-delta between rotor power levels.
const ROTOR_HALF_DELTA_CLAMP: f32 = 0.30;

// Don't execute the calibration procedure from below this altitude, in meters AGL, eg for safety.
const MIN_CAL_ALT: f32 = 6.;

// Minimium speed before auto-yaw will engate. (if we end up setting up auto-yaw to align flight path
// with heading)
// todo: Maybe this could also be used if we end up setting up auto-yaw as sideway-accel cancellation?
// todo, and this would be the min *fwd* velocity?
pub const YAW_ASSIST_MIN_SPEED: f32 = 0.5; // m/s

// if coeff = 0.5, if accel is 1 m/s^2, yaw correction is 1/2 rad/s
// angular velocity / accel: (radians/s) / (m/s^2) = radiants x s / m
pub const YAW_ASSIST_COEFF: f32 = 0.1;

// We use these LUTs to map thrust commanded to throttle position. Note that the starting values will
// include an idle setting.
// todo: Do we need to use these locally, to offset for idle setting?
// const THRUST_LUT: [f32; 10] = [
//     0.027, 0.075, 0.131, 0.225, 0.345, 0.473, 0.613, 0.751, 0.898, 1.0
// ];

pub const POWER_LUT: [f32; 10] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

/// Represents a complete quadcopter. Used for setting control parameters.
struct AircraftProperties {
    mass: f32,               // grams
    arm_len: f32,            // meters. COG to rotor center, horizontally.
    drag_coeff: f32,         // unitless
    thrust_coeff: f32,       // N/m^2
    moment_of_intertia: f32, // kg x m^2
    rotor_inertia: f32,      // kg x m^2
}

impl AircraftProperties {
    /// Calculate the power level required, applied to each rotor, to maintain level flight
    /// at a given MSL altitude. (Alt is in meters)
    pub fn level_pwr(&self, alt: f32) -> f32 {
        0.1 // todo
    }
}

// /// In the event we lose our control link signal
// /// todo: Consider removing this later once you settle on a procedure.
// pub enum LostLinkProcedure {
//     /// turn off all motors. Aircraft will drop to the ground and crash.
//     Disarm,
//     /// Command level attitude and/or louter, and auto land. TOF optional.
//     LoiterAutoLand,
//     /// Return to base, climbing as required, and land. Requires GPS. TOF optional.
//     RtbLand,
//     /// Climb, to attempt to re-acquire the link
//     Climb,
//     /// Hover
//     Hover
// }

/// We use this to freeze an axis in acro mode, when the control for that axis is neutral.
/// this prevents drift from accumulation of errors. Values are uuler angle locked at, in radians. Only used
/// for pitch and roll, since yaw is coupled to those, and can't be maintained independently.
/// todo: Is there a way to do this using portions of a quaternion?
#[derive(Default)]
pub struct AxisLocks {
    pub pitch_locked: Option<f32>,
    pub roll_locked: Option<f32>,
}

/// Specify the rotor by its connection to the ESC. Includdes methods that get information regarding timer
/// and DMA, per specific board setups, in `setup`.
#[derive(Clone, Copy)]
pub enum Motor {
    M1,
    M2,
    M3,
    M4,
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

#[derive(Clone, Copy, PartialEq)]
pub enum RotationDir {
    Clockwise,
    CounterClockwise,
}

/// Map rotor positions to connection numbers to ESC, and motor directions. Positions are associated with flight controls;
/// numbers are associated with motor control. Also allows configuring rotor direcction, using front-left,
/// and aft-right rotors as our stake.
pub struct RotorMapping {
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

impl Default for RotorMapping {
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

impl RotorMapping {
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
#[derive(Default)]
pub struct MotorPower {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

impl MotorPower {
    /// Convert rotor position to its associated power setting.
    fn by_rotor_num(&self, mapping: &RotorMapping) -> (f32, f32, f32, f32) {
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
    pub fn total(&self) -> f32 {
        self.front_left + self.front_right + self.aft_left + self.aft_right
    }

    /// Clamp rotor speeds. A simple form of dealing with a rotor out of limits.
    pub fn clamp_individual_rotors(&mut self) {
        //Note: This loop approach is not working.
        // for rotor in [self.front_left, self.front_right, self.aft_left, self.aft_right].iter_mut() {
        //     if *rotor < THROTTLE_IDLE_POWER {
        //         *rotor = THROTTLE_IDLE_POWER
        //     } else if *rotor > THROTTLE_MAX_POWER {
        //         *rotor = THROTTLE_MAX_POWER;
        //     }
        // }

        if self.front_left < MIN_ROTOR_POWER {
            self.front_left = MIN_ROTOR_POWER;
        } else if self.front_left > MAX_ROTOR_POWER {
            self.front_left = MAX_ROTOR_POWER;
        }

        if self.front_right < MIN_ROTOR_POWER {
            self.front_right = MIN_ROTOR_POWER;
        } else if self.front_right > MAX_ROTOR_POWER {
            self.front_right = MAX_ROTOR_POWER;
        }

        if self.aft_left < MIN_ROTOR_POWER {
            self.aft_left = MIN_ROTOR_POWER;
        } else if self.aft_left > MAX_ROTOR_POWER {
            self.aft_left = MAX_ROTOR_POWER;
        }

        if self.aft_right < MIN_ROTOR_POWER {
            self.aft_right = MIN_ROTOR_POWER;
        } else if self.aft_right > MAX_ROTOR_POWER {
            self.aft_right = MAX_ROTOR_POWER;
        }
    }

    /// Send this power command to the rotors
    pub fn set(
        &self,
        mapping: &RotorMapping,
        rotor_timer_a: &mut Timer<TIM2>,
        rotor_timer_b: &mut Timer<TIM3>,
        arm_status: ArmStatus,
        dma: &mut Dma<DMA1>,
    ) {
        let (p1, p2, p3, p4) = self.by_rotor_num(mapping);

        cfg_if! {
            if #[cfg(feature = "h7")] {
                match arm_status {
                    ArmStatus::Armed => {
                        dshot::set_power(Motor::M1, Motor::M2, Motor::M3, Motor::M4, p1, p2, p3, p4, rotor_timer_b, dma);
                    }
                    ArmStatus::Disarmed => {
                        dshot::stop_all(rotor_timer_b, dma);
                    }
                }
            } else {
                match arm_status {
                    ArmStatus::Armed => {
                        dshot::set_power_a(Motor::M1, Motor::M2, p1, p2, rotor_timer_a, dma);
                        dshot::set_power_b(Motor::M3, Motor::M4, p3, p4, rotor_timer_b, dma);
                    }
                    ArmStatus::Disarmed => {
                        dshot::stop_all(rotor_timer_a, rotor_timer_b, dma);
                    }
                }
            }
        }
    }
}

// todo: DMA for timer? How?

/// Estimate the (single-axis) rotor tilt angle (relative to a level aircraft) to produce
/// a desired amount of acceleration, with a given current velocity.
/// todo: Assume level flight?
/// // todo: come back to this later.
fn estimate_rotor_angle(a_desired: f32, v_current: f32, ac_properties: &AircraftProperties) -> f32 {
    // let drag = ac_properties.drag_coeff * v_current; // todo
    // 1. / ac_properties.thrust_coeff; // todo
    0. // todo
}

/// Helper function for `apply_controls`. Sets specific rotor power based on commanded rates and throttle.
/// We split thihs out so we can call it a second time, in case of a clamp due to min or max power
/// on a rotor.
///
/// The ratios passed in params are on a scale between -1. and +1.
/// Throttle works differently - it's an overall scaler. If ratios are all 0, and power is 1., power for
/// all motors is 100%. No individual power level is allowed to be above 1, or below our idle power
/// setting.
fn calc_rotor_powers(
    mut pitch_half_delta: f32,
    mut roll_half_delta: f32,
    mut yaw_half_delta: f32,
    throttle: f32,
    front_left_dir: RotationDir,
) -> MotorPower {
    // Clamp the output of our PIDs to respect maximum rotor pair power deltas.

    if pitch_half_delta > ROTOR_HALF_DELTA_CLAMP {
        pitch_half_delta = ROTOR_HALF_DELTA_CLAMP
    } else if pitch_half_delta < -ROTOR_HALF_DELTA_CLAMP {
        pitch_half_delta = -ROTOR_HALF_DELTA_CLAMP
    }

    if roll_half_delta > ROTOR_HALF_DELTA_CLAMP {
        roll_half_delta = ROTOR_HALF_DELTA_CLAMP
    } else if roll_half_delta < -ROTOR_HALF_DELTA_CLAMP {
        roll_half_delta = -ROTOR_HALF_DELTA_CLAMP
    }

    if yaw_half_delta > ROTOR_HALF_DELTA_CLAMP {
        yaw_half_delta = ROTOR_HALF_DELTA_CLAMP
    } else if yaw_half_delta < -ROTOR_HALF_DELTA_CLAMP {
        yaw_half_delta = -ROTOR_HALF_DELTA_CLAMP
    }

    // Start by setting all powers equal to the overall throttle setting.
    let mut front_left = throttle;
    let mut front_right = throttle;
    let mut aft_left = throttle;
    let mut aft_right = throttle;

    // println!(
    //     "Pitch {} roll {} yaw {} throttle {}",
    //     pitch_half_delta, roll_half_delta, yaw_half_delta, throttle
    // );

    // Nose down for positive pitch.
    front_left -= pitch_half_delta;
    front_right -= pitch_half_delta;
    aft_left += pitch_half_delta;
    aft_right += pitch_half_delta;

    // Left side up for positive roll
    front_left += roll_half_delta;
    front_right -= roll_half_delta;
    aft_left += roll_half_delta;
    aft_right -= roll_half_delta;

    // Assumes positive yaw from the IMU means clockwise. // todo: Confirm this.
    // If props rotate in, front-left/aft-right rotors induce a CCW torque on the aircraft.
    // If props rotate out, these same rotors induce a CW torque.
    // This code assumes props rotate inwards towards the front and back ends.
    if front_left_dir == RotationDir::Clockwise {
        yaw_half_delta *= -1.;
    }

    front_left += yaw_half_delta;
    front_right -= yaw_half_delta;
    aft_left -= yaw_half_delta;
    aft_right += yaw_half_delta;

    MotorPower {
        front_left,
        front_right,
        aft_left,
        aft_right,
    }
}

/// Set rotor speed for all 4 rotors. We model pitch, roll, and yaw based on target angular rates.
/// We modify power ratio between the appropriate motor pairs to change these
/// parameters.
///
/// Basic model: For each axis, the starting power is throttle. We then increase or decrease opposing
/// pair power up to a limit (Or absolute rotor power limit) based on the input commands.
///
/// If a rotor exceeds min or max power settings, clamp it.
///
/// Input deltas units are half-power-delta.  based on PID output; they're not in real units like radians/s.
pub fn apply_controls(
    pitch_delta: f32,
    roll_delta: f32,
    yaw_delta: f32,
    throttle: f32,
    mapping: &RotorMapping,
    current_pwr: &mut MotorPower,
    rotor_tim_a: &mut Timer<TIM2>,
    rotor_tim_b: &mut Timer<TIM3>,
    arm_status: ArmStatus,
    dma: &mut Dma<DMA1>,
) {
    let mut pwr = calc_rotor_powers(
        pitch_delta,
        roll_delta,
        yaw_delta,
        throttle,
        mapping.frontleft_aftright_dir,
    );

    pwr.clamp_individual_rotors();

    pwr.set(mapping, rotor_tim_a, rotor_tim_b, arm_status, dma);
    *current_pwr = pwr;
}

/// Calculate the horizontal arget velocity (m/s), for a given distance (m) from a point horizontally.
pub fn enroute_speed_hor(dist: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if dist > 20. {
        max_v
    } else if dist > 10. {
        util::max(2., max_v)
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
        let mut result = util::max(3., max_v);

        if dist < 0. {
            result *= -1.;
        }
    }

    let dist_abs = util::abs(dist);

    let mut result = if dist_abs > 20. {
        max_v
    } else if dist_abs > 10. {
        util::max(2., max_v)
    } else {
        // Get close, then the PID loop will handle the final settling.
        0.5
    };

    if dist < 0. {
        result *= -1.;
    }
    result
}

// todo:
// /// Maybe we do map to acceleration at low speeds or power levels, but then go straight V pid?
// fn enroute_accel(desired_v, current_v) -> Option<f32> {
//}

/// Calculate power level to send to the ESC, from throttle setting. This is set up in a way to map
/// linearly between throttle setting and thrust, with an idle floor for power. Both values are on a scale of 0. to
/// 1., but the map isn't linear.
/// [This article](https://innov8tivedesigns.com/images/specs/Prop-Chart-Instructions-B.pdf) has
/// some plots of relevant info. This fn is based on the "Propeller Thrust vs Throttle Position" chart.
// todo: Fn, or LUT+interp? Maybe with CMSIS
pub fn power_from_throttle(
    throttle: f32,
    interp_inst: &dsp_sys::arm_linear_interp_instance_f32,
) -> f32 {
    // todo: We currently have fixed spacing in our LUT between throttle settings,
    // todo, but we need to reverse that!

    // todo: Is the setting we pass to ESC raw power, or is it RPM???
    // todo if RPM, you need to change this.

    // todo: Can't find this fn? Why??
    // dsp_sys::arm_linear_interp_f32(interp_inst, throttle)

    throttle // todo 1:1 mapping, and ignoring idle power.
}

/// Calculate the landing vertical velocity (m/s), for a given height  (m) above the ground.
pub fn landing_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.5;
    }
    util::max(height / 4., max_v)
}

/// Calculate the takeoff vertical velocity (m/s), for a given height (m) above the ground.
pub fn takeoff_speed(height: f32, max_v: f32) -> f32 {
    // todo: LUT?

    if height > 2. {
        return 0.;
    }
    util::max(height / 4. + 0.01, max_v)
}

pub struct UnsuitableParams {}

/// Execute a profile designed to test PID and motor gain coefficients; update them.
pub fn calibrate_coeffs(params: &Params) -> Result<(), UnsuitableParams> {
    if params.s_z_agl < MIN_CAL_ALT {
        return Err(UnsuitableParams {});
    }

    Ok(())
}

pub fn handle_control_mode(
    input_mode_control: InputModeSwitch,
    input_mode: &mut InputMode,
    state_volatile: &mut StateVolatile,
) {
    state_volatile.input_mode_switch = input_mode_control; // todo: Do we need or use this field?

    *input_mode = match input_mode_control {
        InputModeSwitch::Acro => InputMode::Acro,
        InputModeSwitch::AttitudeCommand => {
            if state_volatile.gps_attached {
                InputMode::Command
            } else {
                InputMode::Attitude
            }
        }
    }
}
