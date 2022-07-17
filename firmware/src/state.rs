//! This module contains code related to state, both config stored to flash, and volatile data
//! specific to the current flight, and cleared when power is removed.

use crate::usb_cfg::WAYPOINT_SIZE;
/// User-configurable settings. These get saved to and loaded from internal flash.
use crate::{
    control_interface::{InputModeSwitch, LinkStats},
    flight_ctrls::{
        flying_wing::{ControlPositions, ServoWingMapping},
        quad::{AxisLocks, MotorPower, RotorMapping},
    },
    ppks::Location,
};

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

#[derive(Clone, Copy, PartialEq)]
pub enum AircraftType {
    /// Angry bumblebee
    Quadcopter,
    /// Baby B-2
    FlyingWing,
}

#[derive(Clone, Copy)]
/// Role in a swarm of drones
pub enum SwarmRole {
    Queen,
    Worker(u16),    // id
    PersonFollower, // When your queen is human.
}

use crate::ppks::Location;

// todo; MOve this ldg config elsewhere
struct LandingCfgFixedWing {
    pub heading: f32, // degrees magnetic
    pub airspeed: f32, // m/s
    pub glideslope: f32, // radians, down from level
    pub touchdown_point: Location,
}

/// Persistent state; saved to onboard flash memory.
pub struct UserCfg {
    pub aircraft_type: AircraftType,
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
    /// Map motor connection number to position. For quadcopters.
    pub motor_mapping: RotorMapping,
    /// For flying wing.
    pub servo_wing_mapping: ServoWingMapping,
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
    pub landing_cfg_fixed_wing: LandingConfigFixedWing,
}

impl Default for UserCfg {
    fn default() -> Self {
        let waypoints = [(); MAX_WAYPOINTS].map(|_| Option::<Location>::default());

        Self {
            aircraft_type: AircraftType::Quadcopter,
            ceiling: Some(122.),
            // todo: Do we want max angle and vel here? Do we use them, vice settings in InpuMap?
            // max_angle: TAU * 0.22,
            max_velocity: 30., // todo: raise?
            // Note: Idle power now handled in `power_interp_inst`
            idle_pwr: 0.02, // scale of 0 to 1.
            // todo: Find apt value for these
            // pitch_input_range: (0., 1.),
            // roll_input_range: (0., 1.),
            // yaw_input_range: (0., 1.),
            // throttle_input_range: (0., 1.),
            mapping_obstacles: false,
            max_speed_hor: 20.,
            max_speed_ver: 20.,
            motor_mapping: Default::default(),
            servo_wing_mapping: Default::default(),
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
        }
    }
}

/// State that doesn't get saved to flash.
#[derive(Default)]
pub struct StateVolatile {
    pub op_mode: OperationMode,
    pub input_mode_switch: InputModeSwitch,
    /// The GPS module is connected. Detected on init.
    pub gps_attached: bool,
    /// The time-of-flight sensor module is connected. Detected on init.
    pub tof_attached: bool,
    // FOr now, we use "link lost" to include never having been connected.
    // connected_to_controller: bool,
    pub axis_locks: AxisLocks,
    /// Quadcopter
    pub current_pwr: MotorPower,
    /// Fixed-wing
    pub ctrl_positions: ControlPositions,
    /// Link statistics, including Received Signal Strength Indicator (RSSI) from the controller's radio.
    pub link_stats: LinkStats,
}
