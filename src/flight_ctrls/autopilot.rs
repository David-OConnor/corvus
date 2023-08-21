//! This module contains code related to various autopilot modes.

use core::f32::consts::TAU;

use num_traits::float::Float;

use crate::{
    control_interface::{AltHoldSwitch, AutopilotSwitchA, AutopilotSwitchB, ChannelData},
    flight_ctrls::common::{AltType, CtrlInputs},
    system_status::{SensorStatus, SystemStatus},
    util,
    // pid::{self, CtrlCoeffGroup, PidDerivFilters, PidGroup},
};

use ahrs::{ppks::PositVelEarthUnits, Fix, Params};

use cfg_if::cfg_if;

// Max distance from curent PositVelEarthUnits, to point, then base a
// direct-to point can be, in meters. A sanity check
// todo: Take into account flight time left.
const DIRECT_AUTOPILOT_MAX_RNG: f32 = 500.;

#[cfg(feature = "fixed-wing")]
const TAKEOFF_PITCH: f32 = 1.1; // radians

use defmt::println;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
    } else {
        use crate::flight_ctrls::takeoff_speed;

        // Minimium speed before auto-yaw will engage. (if we end up setting up auto-yaw to align flight path
        // with heading)
        // todo: Maybe this could also be used if we end up setting up auto-yaw as sideway-accel cancellation?
        // todo, and this would be the min *fwd* velocity?
        const YAW_ASSIST_MIN_SPEED: f32 = 0.5; // m/s

        // if coeff = 0.5, if accel is 1 m/s^2, yaw correction is 1/2 rad/s
        // angular velocity / accel: (radians/s) / (m/s^2) = radiants x s / m
        const YAW_ASSIST_COEFF: f32 = 0.1;
    }
}

// todo: FOr various autopilot modes, check if variou sensors are connected like GPS, TOF, and MAG!

use crate::flight_ctrls::motor_servo::MotorServoState;
use cmsis_dsp_sys::{arm_cos_f32, arm_sin_f32};
use lin_alg2::f32::Vec3;

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

const DEG_SCALE_1E8: f32 = 100_000_000.;

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
fn find_bearing(target: (i64, i64), aircraft: (i64, i64)) -> f32 {
    let tgt_lat = target.0 as f32 / DEG_SCALE_1E8;
    let tgt_lon = target.1 as f32 / DEG_SCALE_1E8;
    let ac_lat = aircraft.0 as f32 / DEG_SCALE_1E8;
    let ac_lon = aircraft.1 as f32 / DEG_SCALE_1E8;

    let y = sin(tgt_lon - tgt_lon) * cos(tgt_lat);
    let x = cos(ac_lat) * sin(tgt_lat) - sin(ac_lat) * cos(tgt_lat) * cos(tgt_lon - ac_lon);
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
    pub touchdown_point: PositVelEarthUnits,
}

#[cfg(feature = "quad")]
#[repr(u8)] // for USB serialization
#[derive(Clone, Copy, PartialEq)]
pub enum YawAssist {
    Disabled = 0,
    YawAssist = 1,
    /// Automatically adjust roll (rate? angle?) to zero out slip, ie based on rudder inputs.
    RollAssist = 2,
}

#[cfg(feature = "quad")]
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
    /// Touchdown PositVelEarthUnits, ideally with GPS (requirement?)
    pub touchdown_point: PositVelEarthUnits,
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
    pub direct_to_point: Option<PositVelEarthUnits>,
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
    pub loiter: Option<PositVelEarthUnits>,
    #[cfg(feature = "fixed-wing")]
    /// Orbit over a point on the ground
    pub orbit: Option<Orbit>,
}

// todo: Here or PID: If you set something like throttle to some or none via an AP mode etc,
// todo make sure you set it back to none A/R.

impl AutopilotStatus {
    #[cfg(feature = "quad")]
    /// The output `CtrlInputs` are in Euler angle attitudes.
    pub fn apply(
        &self,
        autopilot_commands: &mut CtrlInputs,
        params: &Params,
        // filters: &mut PidDerivFilters,
        // coeffs: &CtrlCoeffGroup,
        system_status: &SystemStatus,
        throttle_prev: f32, // ie might be autopilot or ch data.
        dt: f32,
    ) {
        // We use if/else logic on these to indicate they're mutually-exlusive. Modes listed first
        // take precedent.

        // todo: sensors check for this fn, and for here and fixed.
        // todo sensor check for alt hold agl

        // todo: add hdg hold here and fixed

        // todo: THis is currently broken; figure out how you command things with it.

        // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
        // either MSL or AGL.
        if self.takeoff {
            let to_speed = match params.alt_tof {
                Some(alt) => alt,
                None => params.alt_msl_baro, // todo temp?
            };

            *autopilot_commands = CtrlInputs {
                pitch: Some(0.),
                roll: Some(0.),
                yaw: None,
                throttle: Some(takeoff_speed(to_speed, MAX_VER_SPEED)),
            };
        } else if let Some(ldg_cfg) = &self.land {
            if system_status.gnss == SensorStatus::Pass {}
        } else if let Some(pt) = &self.direct_to_point {
            if system_status.gnss == SensorStatus::Pass {
                let target_heading = find_bearing(
                    (params.posit_fused.lat_e8, params.posit_fused.lon_e8),
                    (pt.lat_e8, pt.lon_e8),
                );

                autopilot_commands.yaw = Some(target_heading);
            }
        } else if let Some(pt) = &self.loiter {
            if system_status.gnss == SensorStatus::Pass {
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

        // todo: (Hmm forgot, but it was something I need to add to this!)

        // todo: Take into account attitude! Probalby take angle between earth and AC up,
        // todo, and take into account cos of it. Definitely don't do anything weird if upside-down!

        // Re VV P term: If we are 1 meter too low, what would be a reasonable vertical speed?
        // Think of it that way. Think of this P term as the time, 1 dividied by the seconds, to complete
        // the altitude correction.
        const VERTICAL_VELOCITY_P_TERM: f32 = 2.;

        // The alt hold terms are used to output a throttle adjustment (from its previous value), based
        // on the difference between target and current vertical velocity.
        const ALT_HOLD_P_TERM: f32 = 0.00005;
        // const ALT_HOLD_I_TERM: f32 = 0.00002;
        const ALT_HOLD_I_TERM: f32 = 0.00001;
        // This should be on the order of the error term
        const MAX_I_WINDUP: f32 = 0.1; // todo: What should this be?

        // todo: Integral term for VV?

        static mut integral_vertical_velocity: f32 = 0.;
        static mut alt_msl_baro_prev: f32 = 0.;

        unsafe {
            match self.alt_hold {
                Some((alt_type, alt_commanded)) => {
                    let vertical_velocity = (params.alt_msl_baro - alt_msl_baro_prev) / dt;
                    alt_msl_baro_prev = params.alt_msl_baro;

                    let error_alt = match alt_type {
                        AltType::Msl => alt_commanded - params.alt_msl_baro,
                        AltType::Agl => 0., // todo tmep
                    };

                    integral_vertical_velocity += error_alt * dt;

                    // todo: Use a non-linear setup instead of P loop?
                    let vertical_velocity_commanded = VERTICAL_VELOCITY_P_TERM * error_alt;
                    let error_vertical_velocity = vertical_velocity_commanded - vertical_velocity;

                    let vertical_velocity_correction = ALT_HOLD_P_TERM * error_vertical_velocity
                        + ALT_HOLD_I_TERM * integral_vertical_velocity;

                    autopilot_commands.throttle = {
                        let mut throttle_command =
                            autopilot_commands.throttle.unwrap_or(throttle_prev)
                                + vertical_velocity_correction;

                        // todo: Remove 0.5 limit eventually; it's there for safety currently.
                        // todo: Swithc it to 1, and the lower end to our user_cfg idle.
                        util::clamp(&mut throttle_command, (0.02, 0.5));

                        Some(throttle_command)
                    };

                    util::clamp(
                        &mut integral_vertical_velocity,
                        (-MAX_I_WINDUP, MAX_I_WINDUP),
                    );
                    if autopilot_commands.throttle.unwrap_or(0.) < 0.10 {
                        integral_vertical_velocity = 0.;
                    }

                    // todo tmep
                    static mut I: u32 = 0;
                    I += 1;
                    if I % 400 == 0 {
                        println!(
                            "Alt E: {:?} VV E: {:?} VV Tgt:{} VV Cur: {} T: {:?}",
                            error_alt,
                            error_vertical_velocity,
                            vertical_velocity_commanded,
                            vertical_velocity,
                            autopilot_commands.throttle.unwrap_or(69.)
                        );
                    }
                }
                None => {
                    integral_vertical_velocity = 0.;
                    alt_msl_baro_prev = 0.;
                }
            }
        }

        // todo: Look at the commented out alt hold section below, and mix with the one above A/R

        // todo, aug 2023. Currently have something in flight_ctrls\mod.rs. Move it here.
        // if self.alt_hold.is_some() && !self.takeoff && self.land.is_none() {
        //     let (alt_type, alt_commanded) = self.alt_hold.unwrap();
        //
        //     if !(alt_type == AltType::Agl && system_status.tof != SensorStatus::Pass) {
        //         // Set a vertical velocity for the inner loop to maintain, based on distance
        //         let dist = match alt_type {
        //             AltType::Msl => alt_commanded - params.alt_msl_baro,
        //             AltType::Agl => alt_commanded - params.alt_tof.unwrap_or(0.),
        //         };
        //
        //         // todo: Instead of a PID, consider something simpler.
        //         // pid.thrust = pid::calc_pid_error(
        //         //     // If just entering this mode, default to 0. throttle as a starting point.
        //         //     autopilot_commands.thrust.unwrap_or(0.),
        //         //     dist,
        //         //     &pid.thrust,
        //         //     coeffs.thrust.k_p_attitude,
        //         //     coeffs.thrust.k_i_attitude,
        //         //     coeffs.thrust.k_d_attitude,
        //         //     &mut filters.thrust,
        //         //     DT_MAIN_LOOP,
        //         // );
        //         let scaler = 0.1; // todo: Quick hack.
        //         autopilot_commands.throttle = Some(dist * scaler);
        //
        //         // Note that thrust is set using the rate loop.
        //         autopilot_commands.throttle = None;
        //     }
        // }
    }

    #[cfg(feature = "fixed-wing")]
    pub fn apply(
        &self,
        autopilot_commands: &mut CtrlInputs,
        params: &Params,
        // pid_attitude: &mut PidGroup,
        // filters: &mut PidDerivFilters,
        // coeffs: &CtrlCoeffGroup,
        system_status: &SystemStatus,
        dt: f32,
    ) {
        if self.takeoff {
            autopilot_commands = CtrlInputs {
                pitch: Some(TAKEOFF_PITCH),
                roll: Some(0.),
                yaw: None,
                throttle: Some(1.0), // full thrust.
            };
        } else if let Some(ldg_cfg) = &self.land {
            if system_status.gnss == SensorStatus::Pass {
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
            if system_status.gnss == SensorStatus::Pass {
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
            if system_status.gnss == SensorStatus::Pass {
                let target_heading = find_bearing((params.lat, params.lon), (pt.lat, pt.lon));

                let target_pitch = ((pt.alt_msl - params.alt_msl_baro)
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
                    AltType::Msl => alt_commanded - params.alt_msl_baro,
                    AltType::Agl => alt_commanded - params.alt_tof.unwrap_or(0.),
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
    }

    /// Set auto pilot modes based on control inputs.
    pub fn set_modes_from_ctrls(&mut self, control_channel_data: &ChannelData, params: &Params) {
        match control_channel_data.alt_hold {
            AltHoldSwitch::Disabled => self.alt_hold = None,
            // If just setting this hold mode, use the current altitude. Otherwise, keep
            // the same value.
            AltHoldSwitch::EnabledAgl => {
                let alt_to_hold = match self.alt_hold {
                    Some((alt_type, val)) => match alt_type {
                        AltType::Msl => params.alt_tof.unwrap_or(20.),
                        AltType::Agl => val,
                    },
                    None => params.alt_tof.unwrap_or(20.),
                };
                self.alt_hold = Some((AltType::Agl, alt_to_hold));
            }
            AltHoldSwitch::EnabledMsl => {
                let new_commanded_alt = params.alt_msl_baro;
                // todo temp:
                let new_commanded_alt = 0.3;
                let alt_to_hold = match self.alt_hold {
                    Some((alt_type, val)) => match alt_type {
                        AltType::Msl => val,
                        AltType::Agl => new_commanded_alt,
                    },
                    None => new_commanded_alt,
                };
                self.alt_hold = Some((AltType::Msl, alt_to_hold));
            }
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
                    // todo qc these
                    center_lat: params.posit_fused.lat_e8 as f32 / 10_000_000.,
                    center_lon: params.posit_fused.lat_e8 as f32 / 10_000_000.,
                    radius: ORBIT_DEFAULT_RADIUS,
                    ground_speed: ORBIT_DEFAULT_GROUNDSPEED,
                    direction: Default::default(),
                });
            }
            #[cfg(feature = "quad")]
            AutopilotSwitchA::LoiterOrbit => {
                self.loiter = Some(PositVelEarthUnits {
                    lat_e8: params.posit_fused.lat_e8,
                    lon_e8: params.posit_fused.lon_e8,
                    elevation_msl: params.alt_msl_baro,
                    velocity: Vec3::new(params.v_x, params.v_y, params.v_z),
                });
            }
            AutopilotSwitchA::DirectToPoint => {
                self.alt_hold = Some((AltType::Msl, params.alt_msl_baro))
            }
        }

        match control_channel_data.autopilot_b {
            AutopilotSwitchB::Disabled => {
                self.hdg_hold = None;
                self.land = None;
            }
            AutopilotSwitchB::HdgHold => self.hdg_hold = Some(params.attitude.to_euler().yaw),
            AutopilotSwitchB::Land => {
                // todo: impl.
                // self.land = Some(Land);
            }
        }
    }
}
