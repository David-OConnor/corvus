//! This module contains code related to state, both config stored to flash, and volatile data
//! specific to the current flight, and cleared when power is removed.
use ahrs::ppks::PositVelEarthUnits;
use cfg_if::cfg_if;
use lin_alg2::f32::Quaternion;
use stm32_hal2::flash::{Bank, Flash};

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        use lin_alg2::f32::Vec3;
    } else {
        use crate::flight_ctrls::InputMode;
    }
}

use defmt::println;

#[cfg(feature = "fixed-wing")]
use crate::flight_ctrls::{ControlSurfaceConfig, YawControl};
use crate::{
    control_interface::InputModeSwitch,
    flight_ctrls::{
        autopilot::LandingCfg,
        common::{AttitudeCommanded, CtrlInputs, CtrlMix, InputMap},
        ctrl_effect_est::AccelMaps,
        ctrl_logic::{CtrlCoeffs, DragCoeffs},
        // ControlMapping,
        motor_servo::MotorServoState,
        pid::PidCoeffs,
    },
    safety::ArmStatus,
    sensors_shared::BattCellCount,
    usb_preflight::CONFIG_SIZE,
};

// The maximum number of waypoints available.
pub const MAX_WAYPOINTS: usize = 30; // todo: Consider raising this.

#[derive(Clone, Copy, PartialEq)]
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

/// Represents a complete quadcopter. Used for setting control parameters.
/// todo: Currently unimplemented.
// struct AircraftProperties {
//     mass: f32,               // grams
//     arm_len: f32,            // meters. COG to rotor center, horizontally.
//     drag_coeff: f32,         // unitless
//     thrust_coeff: f32,       // N/m^2
//     moment_of_intertia: f32, // kg x m^2
//     rotor_inertia: f32,      // kg x m^2
// }

// impl AircraftProperties {
//     /// Calculate the power level required, applied to each rotor, to maintain level flight
//     /// at a given MSL altitude. (Alt is in meters)
//     pub fn _level_pwr(&self, alt: f32) -> f32 {
//         0.1 // todo
//     }
// }

/// Persistent state; saved to onboard flash memory. Contains user-configurable settings.
pub struct UserConfig {
    #[cfg(feature = "fixed-wing")]
    pub control_surface_config: ControlSurfaceConfig,
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
    // /// Map motor connection number to position, or servos for fixed wing
    // pub control_mapping: ControlMapping,
    // Note that this inst includes idle power.
    // todo: We want to store this inst, but RTIC doesn't like it not being sync. Maybe static mut.
    // todo. For now, lives in the acro PID fn lol.
    // power_interp_inst: dsp_sys::arm_linear_interp_instance_f32,
    // Elevation of the launch point, in MSL. Used for our (QFE) altimeter.
    // pub launch_pt_msl: f32,
    // Pressure at the surface at the launch point, in Pa.
    // pub altimeter_setting: f32,
    pub waypoints: [Option<PositVelEarthUnits>; MAX_WAYPOINTS],
    /// The (index of the) waypoint we are currently steering to.
    pub active_waypoint: usize,
    pub landing_cfg: LandingCfg,
    // ///Modify `rate` mode to command an orientation that changes based on rate control inputs.
    // pub attitude_based_rate_mode: bool,
    pub input_map: InputMap,
    pub ctrl_coeffs: CtrlCoeffs,
    pub takeoff_attitude: Quaternion,
    pub batt_cell_count: BattCellCount,
    /// Number of poles in each motor. Can be counted by hand, or by referencing motor datasheets.
    pub motor_pole_count: u8,
    pub base_pt: PositVelEarthUnits,
    pub pid_coeffs: PidCoeffs,
}

impl Default for UserConfig {
    fn default() -> Self {
        let waypoints = [(); MAX_WAYPOINTS].map(|_| Option::<PositVelEarthUnits>::default());

        Self {
            #[cfg(feature = "fixed-wing")]
            control_surface_config: ControlSurfaceConfig::default(),
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
            // control_mapping: Default::default(),
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
            // #[cfg(feature = "fixed-wing")]
            // attitude_based_rate_mode: true,
            input_map: Default::default(),
            ctrl_coeffs: Default::default(),
            #[cfg(feature = "quad")]
            takeoff_attitude: Quaternion::new_identity(),
            #[cfg(feature = "fixed-wing")]
            takeoff_attitude: Quaternion::from_axis_angle(Vec3::new(1., 0., 0.), 0.35),
            batt_cell_count: Default::default(),
            motor_pole_count: 14,
            base_pt: Default::default(),
            pid_coeffs: Default::default(),
        }
    }
}

impl UserConfig {
    /// For use with Preflight, via USB
    pub fn from_bytes(buf: &[u8]) -> Self {
        let pid_coeffs = PidCoeffs {
            p: f32::from_be_bytes(buf[0..4].try_into().unwrap()),
            i: f32::from_be_bytes(buf[4..8].try_into().unwrap()),
            d: f32::from_be_bytes(buf[8..12].try_into().unwrap()),
            att_ttc: f32::from_be_bytes(buf[12..16].try_into().unwrap()),
        };
        Self {
            pid_coeffs,
            ..Default::default()
        }
    }

    /// For use with Preflight, via USB
    pub fn to_bytes(&self) -> [u8; CONFIG_SIZE] {
        let mut result = [0; CONFIG_SIZE];

        result[..4].clone_from_slice(&self.pid_coeffs.p.to_be_bytes());
        result[4..8].clone_from_slice(&self.pid_coeffs.i.to_be_bytes());
        result[8..12].clone_from_slice(&self.pid_coeffs.d.to_be_bytes());
        result[12..16].clone_from_slice(&self.pid_coeffs.att_ttc.to_be_bytes());

        result
    }

    pub fn save(&self, flash: &mut Flash) {
        flash.erase_page(Bank::B1, crate::FLASH_CFG_PAGE).ok();

        flash
            .write_page(Bank::B1, crate::FLASH_CFG_PAGE, &self.to_bytes())
            .ok();
    }

    pub fn load(flash: &mut Flash) -> Self {
        let mut buf = [0; CONFIG_SIZE];
        flash.read(Bank::B1, crate::FLASH_CFG_PAGE, 0, &mut buf);

        Self::from_bytes(&buf)
    }
}

/// State that doesn't get saved to flash.
#[derive(Default)]
pub struct StateVolatile {
    pub arm_status: ArmStatus,
    pub op_mode: OperationMode,
    #[cfg(feature = "quad")]
    pub input_mode: InputMode,
    pub input_mode_switch: InputModeSwitch,
    // For now, we use "link lost" to include never having been connected.
    // connected_to_controller: bool,
    /// Base point - generally takeoff location.
    pub base_point: PositVelEarthUnits, // todo: user cfg varianit too?
    /// The commanded attitude. Used in attitude mode, and a variant of rate mode.
    /// For attitude mode, and modified rate mode.
    pub attitude_commanded: AttitudeCommanded,
    // pub rates_commanded: RatesCommanded,
    // /// On a scale of 0 to 1.
    pub autopilot_commands: CtrlInputs,
    /// We us this to analyze how the current controls are impacting
    /// angular accelerations.
    pub ctrl_mix: CtrlMix,
    /// We use this to determine if we can unlock the attitude controls from the takeoff attitude.
    pub has_taken_off: bool,
    /// Angular drag coefficient, continuously updated.
    pub angular_drag_coeff: f32,
    pub batt_v: f32,      // volts
    pub esc_current: f32, // amps
    /// Drag calculated drag coefficients from flight params.
    pub drag_coeffs: DragCoeffs,
    /// We log angular acceleration vice control data (RPM deltas, or servo commands/positions) as part
    /// of the control-effect model in our flight-controls system
    /// Relates motor pair delta RPM difference to angular acceleration for quads, or servo settings
    /// to angular accel for fixed-wing.
    pub accel_maps: AccelMaps,
    /// Atmospheric pressure, in Pa.
    pub pressure_static: f32,
    /// Temperature, in K, measured by the barometer
    pub temp_baro: f32,
    /// Holds all motor and servo mappings and state.
    /// todo: Mappings are more of a User Cfg functionality
    pub motor_servo_state: MotorServoState,
    /// Use this, in combination with arm status, and `MotorServoState`.
    pub preflight_motors_running: bool,
    #[cfg(feature = "quad")]
    pub estimated_hover_power: f32,
    #[cfg(feature = "quad")]
    /// Per motor.
    pub estimated_hover_rpm: f32,
    // /// todo - experimental. We use this to avoid numerical precision issues that occur from
    // /// todo tracking the tiny attitude changes each update loop.`
    // /// Todo: Along these lines, you probably don't want to update target attitude each
    // pub att_cmd_history: [Quaternion; crate::TORQUE_CMD_UPDATE_RATIO as usize],
}
