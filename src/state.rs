//! This module contains code related to state, both config stored to flash, and volatile data
//! specific to the current flight, and cleared when power is removed.

/// User-configurable settings. These get saved to and loaded from internal flash.
use crate::{
    control_interface::{InputModeSwitch, LinkStats},
    flight_ctrls::{
        autopilot::{AutopilotStatus, LandingCfg},
        common::{AttitudeCommanded, CtrlInputs, CtrlMix, InputMap, RatesCommanded},
        ctrl_logic::{CtrlCoeffs, PowerMaps},
        ControlMapping,
    },
    ppks::Location,
    safety::ArmStatus,
    usb_cfg::WAYPOINT_SIZE,
};

use lin_alg2::f32::{Quaternion, Vec3};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use crate::flight_ctrls::{ControlPositions};
    } else {
        use crate::flight_ctrls::{MotorPower, InputMode};
    }
}

// The maximum number of waypoints available.
pub const MAX_WAYPOINTS: usize = 30; // todo: Consider raising this.

#[derive(Clone, Copy)]
pub enum OperationMode {
    /// Eg flying
    Normal,
    /// Plugged into a PC to verify motors, IMU readings, control readings etc, and adjust settings
    Preflight,
}

impl Default for OperationMode {
    fn default() -> Self {
        Self::Normal
    }
}

// #[derive(Clone, Copy, PartialEq)]
// pub enum AircraftType {
//     /// Angry bumblebee
//     Quadcopter,
//     /// Baby B-2
//     FixedWing,
// }

// #[derive(Clone, Copy)]
// /// Role in a swarm of drones
// pub enum SwarmRole {
//     Queen,
//     Worker(u16),    // id
//     PersonFollower, // When your queen is human.
// }

/// Persistent state; saved to onboard flash memory. Contains user-configurable settings.
pub struct UserCfg {
    /// Set a ceiling the aircraft won't exceed. Defaults to 400' (Legal limit in US for drones).
    /// In meters.
    pub ceiling: Option<f32>,
    /// In Attitude and related control modes, max pitch angle (from straight up), ie
    /// full speed, without going horizontal or further.
    // max_angle: f32, // radians
    pub max_velocity: f32, // m/s
    pub idle_pwr: f32,
    // /// These input ranges map raw output from a manual controller to full scale range of our control scheme.
    // /// (min, max). Set using an initial calibration / setup procedure.
    // pitch_input_range: (f32, f32),
    // roll_input_range: (f32, f32),
    // yaw_input_range: (f32, f32),
    // throttle_input_range: (f32, f32),
    /// Is the aircraft continuously collecting data on obstacles, and storing it to external flash?
    pub mapping_obstacles: bool,
    pub max_speed_hor: f32,
    pub max_speed_ver: f32,
    /// Map motor connection number to position, or servos for fixed wing
    pub control_mapping: ControlMapping,
    // altitude_cal: AltitudeCalPt,
    // Note that this inst includes idle power.
    // todo: We want to store this inst, but RTIC doesn't like it not being sync. Maybe static mut.
    // todo. For now, lives in the acro PID fn lol.
    // power_interp_inst: dsp_sys::arm_linear_interp_instance_f32,
    // Elevation of the launch point, in MSL. Used for our (QFE) altimeter.
    // pub launch_pt_msl: f32,
    // Pressure at the surface at the launch point, in Pa.
    // pub altimeter_setting: f32,
    pub waypoints: [Option<Location>; MAX_WAYPOINTS],
    /// The (index of the) waypoint we are currently steering to.
    pub active_waypoint: usize,
    pub landing_cfg: LandingCfg,
    #[cfg(feature = "fixed-wing")]
    /// (Alternative is no rudder)
    pub rudder_used: bool,
    // ///Modify `rate` mode to command an orientation that changes based on rate control inputs.
    // pub attitude_based_rate_mode: bool,
    pub input_map: InputMap,
    pub ctrl_coeffs: CtrlCoeffs,
    pub takeoff_attitude: Quaternion,
}

impl Default for UserCfg {
    fn default() -> Self {
        let waypoints = [(); MAX_WAYPOINTS].map(|_| Option::<Location>::default());

        Self {
            // aircraft_type: AircraftType::Quadcopter,
            ceiling: Some(122.),
            // todo: Do we want max angle and vel here? Do we use them, vice settings in InpuMap?
            // max_angle: TAU * 0.22,
            max_velocity: 30., // todo: raise?
            // Note: Idle power now handled in `power_interp_inst`
            idle_pwr: 0.02, // scale of 0 to 1.
            mapping_obstacles: false,
            max_speed_hor: 20.,
            max_speed_ver: 20.,
            control_mapping: Default::default(),
            // altitude_cal: Default::default(),
            // Make sure to update this interp table if you change idle power.
            // todo: This LUT setup is backwards! You need to put thrust on a fixed spacing,
            // todo, and throttle as dynamic (y)!
            // power_interp_inst: dsp_sys::arm_linear_interp_instance_f32 {
            //     nValues: 11,
            //     x1: 0.,
            //     xSpacing: 0.1,
            //     pYData: [
            //         // Idle power.
            //         0.02, // Make sure this matches the above.
            //         POWER_LUT[0],
            //         POWER_LUT[1],
            //         POWER_LUT[2],
            //         POWER_LUT[3],
            //         POWER_LUT[4],
            //         POWER_LUT[5],
            //         POWER_LUT[6],
            //         POWER_LUT[7],
            //         POWER_LUT[8],
            //         POWER_LUT[9],
            //         POWER_LUT[10],
            //     ].as_mut_ptr()
            // },
            // launch_pt_msl: 100.,
            // altimeter_setting: 101_325.,
            waypoints,
            active_waypoint: 0,
            landing_cfg: Default::default(),
            #[cfg(feature = "fixed-wing")]
            rudder_used: false,
            // attitude_based_rate_mode: true,
            input_map: Default::default(),
            ctrl_coeffs: Default::default(),
            #[cfg(feature = "quad")]
            takeoff_attitude: Quaternion::new_identity(),
            #[cfg(feature = "fixed-wing")]
            takeoff_attitude: Quaternion::from_axis_angle(Vec3::new(1., 0., 0.), 0.35),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)] // for USB ser
pub enum SensorStatus {
    Pass = 0,
    Fail = 1,
    /// Either an external sensor not plugged in, or a complete failture, werein it's not recognized.
    NotConnected = 2,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self::NotConnected
    }
}

#[derive(Default)]
pub struct SystemStatus {
    pub imu: SensorStatus,
    pub baro: SensorStatus,
    /// The GPS module is connected. Detected on init.
    pub gps: SensorStatus,
    /// The time-of-flight sensor module is connected. Detected on init.
    pub tof: SensorStatus,
    ///  magnetometer is connected. Likely on the same module as GPS. Detected on init.
    pub magnetometer: SensorStatus,
    pub esc_telemetry: SensorStatus,
    pub esc_rpm: SensorStatus,
}

/// State that doesn't get saved to flash.
#[derive(Default)]
pub struct StateVolatile {
    pub arm_status: ArmStatus,
    pub op_mode: OperationMode,
    #[cfg(feature = "quad")]
    pub input_mode: InputMode,
    pub input_mode_switch: InputModeSwitch,
    pub system_status: SystemStatus,
    // FOr now, we use "link lost" to include never having been connected.
    // connected_to_controller: bool,
    pub link_lost: bool,
    #[cfg(feature = "quad")]
    // /// Attitudes to hold for each axis, eg if control input is neutral
    // pub axis_locks: AxisLocks,
    #[cfg(feature = "quad")]
    pub current_pwr: MotorPower,
    #[cfg(feature = "fixed-wing")]
    /// Flight control positions
    pub ctrl_positions: ControlPositions,
    /// Link statistics, including Received Signal Strength Indicator (RSSI) from the controller's radio.
    pub link_stats: LinkStats,
    /// Base point - generally takeoff location.
    pub base_point: Location, // todo: user cfg varianit too?
    /// The commanded attitude. Used in attitude mode, and a variant of rate mode.
    /// For attitude mode, and modified rate mode.
    pub attitude_commanded: AttitudeCommanded,
    pub rates_commanded: RatesCommanded,
    // /// On a scale of 0 to 1.
    // pub throttle_commanded: Option<f32>,
    pub autopilot_commands: CtrlInputs,
    /// We us this to analyze how the current controls are impacting
    /// angular accelerations.
    pub ctrl_mix: CtrlMix,
    /// We use this to determine if we can unlock the attitude controls from the takeoff attitude.
    pub has_taken_off: bool,
    /// We use this to disable normal motor commands until the motor direction has been set.
    pub initializing_motors: bool,
    /// Power to RPM, and RPM to angular acceleration data. (todo: Should this be stored in cfg?)
    pub power_maps: PowerMaps,
    /// Angular drag coefficient, continuously updated.
    pub angular_drag_coeff: f32,
}
