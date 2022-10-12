//! This module contains code related to various autopilot modes.

use core::f32::consts::TAU;

use num_traits::float::Float;

use crate::{
    control_interface::{AltHoldSwitch, AutopilotSwitchA, AutopilotSwitchB, ChannelData},
    flight_ctrls::{
        self,
        common::{AltType, CtrlInputs, InputMap, Params},
    },
    // pid::{self, CtrlCoeffGroup, PidDerivFilters, PidGroup},
    ppks::{Location, LocationType},
    state::{SensorStatus, SystemStatus},
    DT_MAIN_LOOP,
};

// Max distance from curent location, to point, then base a
// direct-to point can be, in meters. A sanity check
// todo: Take into account flight time left.
const DIRECT_AUTOPILOT_MAX_RNG: f32 = 500.;

#[cfg(feature = "fixed-wing")]
const TAKEOFF_PITCH: f32 = 1.1; // radians

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
    } else {
        use crate::flight_ctrls::{InputMode, POWER_LUT, YAW_ASSIST_COEFF, YAW_ASSIST_MIN_SPEED, takeoff_speed};
    }
}

// todo: FOr various autopilot modes, check if variou sensors are connected like GPS, TOF, and MAG!

use cmsis_dsp_sys::{arm_cos_f32, arm_sin_f32};

const R: f32 = 6_371_000.; // Earth's radius in meters. (ellipsoid?)

// Highest bank to use in all autopilot modes.
const MAX_BANK: f32 = TAU / 6.;

const MAX_VER_SPEED: f32 = 10.; // todo? m/s

// Tolerances we use when setting up a glideslope for landing. Compaerd to the landing structs,
// these are independent of the specific landing spot and aircraft.

// todo: Evaluate if you want `START` in these names, and in how you use them.

// The aircraft's heading must be within this range of the landing heading to initiate the descent.
// (Also must have the minimum ground track, as set in the struct.)
const GLIDESLOPE_START_HEADING_TOLERANCE: f32 = 0.25; // ranians. (0.17 = ~10°)

// The angle distance from landing point to aircraft, and landing point to a point abeam
// the aircraft on glideslope
const GLIDESLOPE_START_LATERAL_TOLERANCE: f32 = 0.30; // radians.

// Orbit heading difference in radians from heading on nearest point on obrit track.
const ORBIT_START_HEADING_TOLERANCE: f32 = 0.40; // radians

// Orbit lateral tolerance is in meters. Aircraft dist to nearest point on orbit track
const ORBIT_START_LATERAL_TOLERANCE: f32 = 10.;

pub const ORBIT_DEFAULT_RADIUS: f32 = 20.; // meters.
pub const ORBIT_DEFAULT_GROUNDSPEED: f32 = 10.; // m/s

fn cos(v: f32) -> f32 {
    unsafe { arm_cos_f32(v) }
}

fn sin(v: f32) -> f32 {
    unsafe { arm_sin_f32(v) }
}

/// Calculate the great circle bearing to fly to arrive at a point, given the aircraft's current position.
/// Params and output are in radians. Does not take into account turn radius.
/// https://www.movable-type.co.uk/scripts/latlong.html
/// θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
fn find_bearing(target: (f32, f32), aircraft: (f32, f32)) -> f32 {
    let y = sin(target.1 - aircraft.1) * cos(target.0);
    let x = cos(aircraft.0) * sin(target.0)
        - sin(aircraft.0) * cos(target.0) * cos(target.1 - aircraft.1);
    y.atan2(x) % TAU
}

/// Calculate the distance between two points, in meters.
/// Params are in radians. Uses the 'haversine' formula
/// https://www.movable-type.co.uk/scripts/latlong.html
/// a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
/// c = 2 ⋅ atan2( √a, √(1−a) )
/// d = R ⋅ c
#[allow(non_snake_case)]
fn find_distance(target: (f32, f32), aircraft: (f32, f32)) -> f32 {
    // todo: LatLon struct with named fields.

    let φ1 = aircraft.0; // φ, λ in radians
    let φ2 = target.0;
    let Δφ = target.0 - aircraft.0;
    let Δλ = target.1 - aircraft.1;

    let a = sin(Δφ / 2.) * sin(Δφ / 2.) + cos(φ1) * cos(φ2) * sin(Δλ / 2.) * sin(Δλ / 2.);

    let c = 2. * a.sqrt().atan2((1. - a).sqrt());

    R * c
}

#[cfg(feature = "fixed-wing")]
#[derive(Clone, Copy)]
pub enum OrbitShape {
    Circular,
    Racetrack,
}

#[cfg(feature = "fixed-wing")]
impl Default for OrbitShape {
    fn default() -> Self {
        OrbitShape::Circular
    }
}

#[cfg(feature = "fixed-wing")]
#[derive(Clone, Copy)]
/// Direction from a top-down perspective.
pub enum OrbitDirection {
    Clockwise,
    CounterClockwise,
}

#[cfg(feature = "fixed-wing")]
impl Default for OrbitDirection {
    fn default() -> Self {
        OrbitDirection::Clockwise
    }
}

#[cfg(feature = "fixed-wing")]
/// Represents an autopilot orbit, centered around a point. The point may remain stationary, or
/// move over time.
pub struct Orbit {
    pub shape: OrbitShape,
    pub center_lat: f32,   // radians
    pub center_lon: f32,   // radians
    pub radius: f32,       // m
    pub ground_speed: f32, // m/s
    pub direction: OrbitDirection,
}

#[cfg(feature = "quad")]
#[derive(Default)]
/// A vertical descent.
pub struct LandingCfg {
    // todo: Could also land at an angle.
    pub descent_starting_alt_msl: f32, // altitude to start the descent in QFE msl.
    pub descent_speed: f32,            // m/s
    pub touchdown_point: Location,
}

#[cfg(feature = "quad")]
#[repr(u8)] // for USB serialization
#[derive(Clone, Copy)]
enum YawAssist {
    Disabled = 0,
    YawAssist = 1,
    /// Automatically adjust roll (rate? angle?) to zero out slip, ie based on rudder inputs.
    RollAssist = 2,
}

impl Default for YawAssist {
    fn default() -> Self {
        Self::Disabled
    }
}

#[cfg(feature = "fixed-wing")]
#[derive(Default)]
pub struct LandingCfg {
    /// Radians magnetic.
    pub heading: f32,
    /// radians, down from level
    pub glideslope: f32,
    /// Touchdown location, ideally with GPS (requirement?)
    pub touchdown_point: Location,
    /// Groundspeed in m/s
    /// todo: Remove ground_speed in favor of AOA once you figure out how to measure AOA.
    pub ground_speed: f32,
    /// Angle of attack in radians
    /// todo: Include AOA later once you figure out how to measure it.
    pub angle_of_attack: f32,
    /// Altitude to start the flare in AGL. Requires TOF sensor or similar.
    pub flare_alt_agl: f32,
    /// Minimum ground track distance in meters the craft must fly while aligned on the heading
    pub min_ground_track: f32,
}

/// Categories of control mode, in regards to which parameters are held fixed.
/// Note that some settings are mutually exclusive.
#[derive(Default)]
pub struct AutopilotStatus {
    /// Altitude is fixed. (MSL or AGL)
    pub alt_hold: Option<(AltType, f32)>,
    /// Heading is fixed.
    pub hdg_hold: Option<f32>,
    // todo: Airspeed hold
    #[cfg(feature = "quad")]
    /// Automatically adjust raw to zero out slip.
    pub yaw_assist: YawAssist,
    /// Continuously fly towards a path. Note that `pitch` and `yaw` for the
    /// parameters here correspond to the flight path; not attitude.
    pub velocity_vector: Option<(f32, f32)>, // pitch, yaw
    /// Fly direct to a point
    pub direct_to_point: Option<Location>,
    /// The aircraft will fly a fixed profile between sequence points
    pub sequence: bool,
    /// Terrain following mode. Similar to TF radar in a jet. Require a forward-pointing sensor.
    /// todo: Add a forward (or angled) TOF sensor, identical to the downward-facing one?
    pub terrain_following: Option<f32>, // AGL to hold
    /// Take off automatically
    pub takeoff: bool, // todo: takeoff cfg struct[s].
    /// Land automatically
    pub land: Option<LandingCfg>,
    /// Recover to stable, altitude-holding flight. Generally initiated by a "panic button"-style
    /// switch activation
    pub recover: Option<f32>, // value is MSL alt to hold, eg our alt at time of command.
    #[cfg(feature = "quad")]
    /// Maintain a geographic position and altitude
    pub loiter: Option<Location>,
    #[cfg(feature = "fixed-wing")]
    /// Orbit over a point on the ground
    pub orbit: Option<Orbit>,
}

// todo: Here or PID: If you set something like throttle to some or none via an AP mode etc,
// todo make sure you set it back to none A/R.

impl AutopilotStatus {
    #[cfg(feature = "quad")]
    pub fn apply(
        &self,
        params: &Params,
        // filters: &mut PidDerivFilters,
        // coeffs: &CtrlCoeffGroup,
        system_status: &SystemStatus,
    ) -> CtrlInputs {
        // We use if/else logic on these to indicate they're mutually-exlusive. Modes listed first
        // take precedent.

        let mut autopilot_commands = CtrlInputs::default();

        // todo: sensors check for this fn, and for here and fixed.
        // todo sensor check for alt hold agl

        // todo: add hdg hold here and fixed

        // todo: THis is currently broken; figure out how you command things with it.

        // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
        // either MSL or AGL.
        if self.takeoff {
            let to_speed = match params.tof_alt {
                Some(alt) => alt,
                None => params.baro_alt_msl, // todo temp?
            };

            autopilot_commands = CtrlInputs {
                pitch: Some(0.),
                roll: Some(0.),
                yaw: None,
                throttle: Some(takeoff_speed(to_speed, MAX_VER_SPEED)),
            };
        } else if let Some(ldg_cfg) = &self.land {
            if system_status.gps == SensorStatus::Pass {}
        } else if let Some(pt) = &self.direct_to_point {
            if system_status.gps == SensorStatus::Pass {
                let target_heading = find_bearing((params.lat, params.lon), (pt.lat, pt.lon));

                autopilot_commands.yaw = Some(target_heading);
            }
        } else if let Some(pt) = &self.loiter {
            if system_status.gps == SensorStatus::Pass {
                // todo
            }
        }

        if self.alt_hold.is_none()
            && !self.takeoff
            && self.land.is_none()
            && self.direct_to_point.is_none()
        {
            autopilot_commands.throttle = None;
        }

        if self.alt_hold.is_some() && !self.takeoff && self.land.is_none() {
            let (alt_type, alt_commanded) = self.alt_hold.unwrap();

            if !(alt_type == AltType::Agl && system_status.tof != SensorStatus::Pass) {
                // Set a vertical velocity for the inner loop to maintain, based on distance
                let dist = match alt_type {
                    AltType::Msl => alt_commanded - params.baro_alt_msl,
                    AltType::Agl => alt_commanded - params.tof_alt.unwrap_or(0.),
                };

                // todo: Instead of a PID, consider something simpler.
                // pid.thrust = pid::calc_pid_error(
                //     // If just entering this mode, default to 0. throttle as a starting point.
                //     autopilot_commands.thrust.unwrap_or(0.),
                //     dist,
                //     &pid.thrust,
                //     coeffs.thrust.k_p_attitude,
                //     coeffs.thrust.k_i_attitude,
                //     coeffs.thrust.k_d_attitude,
                //     &mut filters.thrust,
                //     DT_MAIN_LOOP,
                // );
                let scaler = 0.1; // todo: Quick hack.
                autopilot_commands.throttle = Some(dist * scaler);

                // Note that thrust is set using the rate loop.
                autopilot_commands.throttle = None;
            }
        }

        autopilot_commands
    }

    #[cfg(feature = "fixed-wing")]
    pub fn apply(
        &self,
        params: &Params,
        // pid_attitude: &mut PidGroup,
        // filters: &mut PidDerivFilters,
        // coeffs: &CtrlCoeffGroup,
        system_status: &SystemStatus,
    ) -> CtrlInputs {
        let mut autopilot_commands = CtrlInputs::default();

        if self.takeoff {
            autopilot_commands = CtrlInputs {
                pitch: Some(TAKEOFF_PITCH),
                roll: Some(0.),
                yaw: None,
                throttle: Some(1.0), // full thrust.
            };
        } else if let Some(ldg_cfg) = &self.land {
            if system_status.gps == SensorStatus::Pass {
                let dist_to_touchdown = find_distance(
                    (ldg_cfg.touchdown_point.lat, ldg_cfg.touchdown_point.lon),
                    (params.lat, params.lon),
                );

                let heading_diff = 0.; // todo

                let established = dist_to_touchdown < ORBIT_START_LATERAL_TOLERANCE
                    && heading_diff < ORBIT_START_HEADING_TOLERANCE;
            }
            // todo: DRY between quad and FC here, although the diff is power vs pitch.
        } else if let Some(orbit) = &self.orbit {
            if system_status.gps == SensorStatus::Pass {
                // todo: You'll get a smoother entry if you initially calculate, and fly to a point on the radius
                // todo on a heading similar to your own angle to it. For now, fly directly to the point for
                // todo simpler logic and good-enough.

                let dist_to_center = find_distance(
                    (orbit.center_lat, orbit.center_lon),
                    (params.lat, params.lon),
                );

                let heading_diff = 0.; // todo

                let established = dist_to_center < ORBIT_START_LATERAL_TOLERANCE
                    && heading_diff < ORBIT_START_HEADING_TOLERANCE;

                if !established {
                    // If we're not established and outside the radius...
                    if dist_to_center > orbit.radius {

                        // If we're not established and inside the radius...
                    } else {
                    }
                }

                let target_heading = if established {
                    find_bearing(
                        (params.lat, params.lon),
                        (orbit.center_lat, orbit.center_lon),
                    )
                } else {
                    find_bearing(
                        (params.lat, params.lon),
                        (orbit.center_lat, orbit.center_lon),
                    )
                };
            }
        } else if let Some(pt) = &self.direct_to_point {
            if system_status.gps == SensorStatus::Pass {
                let target_heading = find_bearing((params.lat, params.lon), (pt.lat, pt.lon));

                let target_pitch = ((pt.alt_msl - params.baro_alt_msl)
                    / find_distance((pt.lat, pt.lon), (params.lat, params.lon)))
                .atan();

                // todo: Crude algo here. Is this OK? Important distinction: Flight path does'nt mean
                // todo exactly pitch! Might be close enough for good enough.
                let roll_const = 2.; // radians bank / radians heading  todo: Const?
                autopilot_commands.roll =
                    Some(((target_heading - params.s_yaw_heading) * roll_const).max(MAX_BANK));
                autopilot_commands.pitch = Some(target_pitch);
            }
        }

        if self.alt_hold.is_some()
            && !self.takeoff
            && self.land.is_none()
            && self.direct_to_point.is_none()
        {
            let (alt_type, alt_commanded) = self.alt_hold.unwrap();

            if !(alt_type == AltType::Agl && system_status.tof != SensorStatus::Pass) {
                // Set a vertical velocity for the inner loop to maintain, based on distance
                let dist = match alt_type {
                    AltType::Msl => alt_commanded - params.baro_alt_msl,
                    AltType::Agl => alt_commanded - params.tof_alt.unwrap_or(0.),
                };

                // todo replacement for this and quad.
                // pid_attitude.pitch = pid::calc_pid_error(
                //     // If just entering this mode, default to 0. pitch as a starting point.
                //     autopilot_commands.pitch.unwrap_or(0.),
                //     dist,
                //     &pid_attitude.pitch,
                //     coeffs.pitch.k_p_attitude,
                //     coeffs.pitch.k_i_attitude,
                //     coeffs.pitch.k_d_attitude,
                //     &mut filters.pitch_attitude,
                //     DT_MAIN_LOOP,
                // );

                // todo: Set this at rate or attitude level?

                autopilot_commands.pitch = None;
            }
        }

        // If not in an autopilot mode, reset commands that may have been set by the autopilot, and
        // wouldn't have been reset by manual controls. For now, this only applie to throttle.
        if self.alt_hold.is_none()
            && !self.takeoff
            && self.land.is_none()
            && self.direct_to_point.is_none()
        {
            autopilot_commands.pitch = None;
            autopilot_commands.roll = None;
        }

        autopilot_commands
    }

    /// Set auto pilot modes based on control inputs.
    pub fn set_modes_from_ctrls(&mut self, control_channel_data: &ChannelData, params: &Params) {
        match control_channel_data.alt_hold {
            AltHoldSwitch::Disabled => self.alt_hold = None,
            AltHoldSwitch::EnabledAgl => {
                self.alt_hold = Some((AltType::Agl, params.tof_alt.unwrap_or(20.)))
            }
            AltHoldSwitch::EnabledMsl => self.alt_hold = Some((AltType::Msl, params.baro_alt_msl)),
        }

        match control_channel_data.autopilot_a {
            #[cfg(feature = "fixed-wing")]
            AutopilotSwitchA::Disabled => {
                self.orbit = None;
            }
            #[cfg(feature = "quad")]
            AutopilotSwitchA::Disabled => {
                self.loiter = None;
            }
            #[cfg(feature = "fixed-wing")]
            AutopilotSwitchA::LoiterOrbit => {
                self.orbit = Some(Orbit {
                    shape: Default::default(),
                    center_lat: params.lat,
                    center_lon: params.lon,
                    radius: ORBIT_DEFAULT_RADIUS,
                    ground_speed: ORBIT_DEFAULT_GROUNDSPEED,
                    direction: Default::default(),
                });
            }
            #[cfg(feature = "quad")]
            AutopilotSwitchA::LoiterOrbit => {
                self.loiter = Some(Location::new(
                    LocationType::LatLon,
                    params.lat,
                    params.lon,
                    params.baro_alt_msl,
                ));
            }
            AutopilotSwitchA::DirectToPoint => {
                self.alt_hold = Some((AltType::Msl, params.baro_alt_msl))
            }
        }

        match control_channel_data.autopilot_b {
            AutopilotSwitchB::Disabled => {
                self.hdg_hold = None;
                self.land = None;
            }
            AutopilotSwitchB::HdgHold => self.hdg_hold = Some(params.s_yaw_heading),
            AutopilotSwitchB::Land => {
                // todo: impl.
                // self.land = Some(Land);
            }
        }
    }
}
