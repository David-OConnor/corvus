//! This module contains code related to various autopilot modes.

use crate::{
    flight_ctrls::{
        self,
        common::{AltType, CtrlInputs, InputMap, Params},
        quad::{InputMode, POWER_LUT, YAW_ASSIST_COEFF, YAW_ASSIST_MIN_SPEED},
    },
    pid::{self, CtrlCoeffGroup, PidDerivFilters, PidGroup},
    ppks::Location,
    DT_ATTITUDE,
};

pub enum OrbitShape {
    Circular,
    Racetrack,
}

/// Represents an autopilot orbit, centered around a point. The point may remain stationary, or
/// move over time.
pub struct Orbit {
    shape: OrbitShape,
    center_lat: f32,  // radians
    center_lon: f32,  // radians
    radius: f32,      // m
    groundspeed: f32, // m/s
}

#[derive(Default)]
/// A vertical descent.
pub struct LandingCfgQuad {
    // todo: Could also land at an angle.
    pub descent_starting_alt_msl: f32, // altitude to start the descent in QFE msl.
    pub descent_speed: f32,            // m/s
    pub touchdown_point: Location,
}

#[derive(Default)]
pub struct LandingCfgFixedWing {
    pub heading: f32,    // degrees magnetic
    pub airspeed: f32,   // m/s
    pub glideslope: f32, // radians, down from level
    pub touchdown_point: Location,
}

/// Categories of control mode, in regards to which parameters are held fixed.
/// Note that some settings are mutually exclusive.
#[derive(Default)]
pub struct AutopilotStatus {
    /// Altitude is fixed. (MSL or AGL)
    pub alt_hold: Option<(AltType, f32)>,
    /// Heading is fixed.
    pub hdg_hold: Option<f32>,
    /// Automatically adjust raw to zero out slip. Quad only.
    pub yaw_assist: bool,
    /// Automatically adjust roll (rate? angle?) to zero out slip, ie based on rudder inputs.
    /// Don't enable both yaw assist and roll assist at the same time. Quad only.
    pub roll_assist: bool,
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
    /// todo: We don't need or want both ldg cfgs, but don't have a good way to do one or the other.
    /// todo: Consider a feature gate for fixed vs quad instead of a cfg var.
    pub land_quad: Option<LandingCfgQuad>,
    pub land_fixed_wing: Option<LandingCfgFixedWing>,
    /// Recover to stable, altitude-holding flight. Generally initiated by a "panic button"-style
    /// switch activation
    pub recover: Option<f32>, // value is MSL alt to hold, eg our alt at time of command.
    pub orbit: Option<Orbit>,
}

// todo: Here or PID: If you set something like throttle to some or none via an AP mode etc,
// todo make sure you set it back to none A/R.

impl AutopilotStatus {
    pub fn apply_attitude_quad(
        &self,
        params: &Params,
        attitudes_commanded: &mut CtrlInputs,
        rates_commanded: &mut CtrlInputs,
        pid: &mut PidGroup,
        filters: &mut PidDerivFilters,
        coeffs: &CtrlCoeffGroup,
        input_map: &InputMap,
        max_speed_ver: f32,
    ) {
        // We use if/else logic on these to indicate they're mutually-exlusive. Modes listed first
        // take precedent.

        // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
        // either MSL or AGL.
        if self.takeoff {
            // *attitudes_commanded = CtrlInputs {
            //     pitch: Some(0.),
            //     roll: Some(0.),
            //     yaw: Some(0.),
            //     thrust: Some(flight_ctrls::quad::takeoff_speed(params.tof_alt, max_speed_ver)),
            // };
        } else if let Some(ldg_cfg) = &self.land_quad {
            // *attitudes_commanded = CtrlInputs {
            //     pitch: Some(0.),
            //     roll: Some(0.),
            //     yaw: Some(0.),
            //     thrust: Some(flight_ctrls::quad::landing_speed(params.tof_alt, max_speed_ver)),
            // };
        } else if let Some((alt_type, alt_commanded)) = self.alt_hold {
            // Set a vertical velocity for the inner loop to maintain, based on distance
            let dist = match alt_type {
                AltType::Msl => alt_commanded - params.baro_alt_msl,
                AltType::Agl => alt_commanded - params.tof_alt,
            };

            pid.thrust = pid::calc_pid_error(
                // If just entering this mode, default to 0. throttle as a starting point.
                attitudes_commanded.thrust.unwrap_or(0.),
                dist,
                &pid.thrust,
                coeffs.thrust.k_p_attitude,
                coeffs.thrust.k_i_attitude,
                coeffs.thrust.k_d_attitude,
                &mut filters.thrust,
                DT_ATTITUDE,
            );

            // todo: Set this at rate or attitude level?

            attitudes_commanded.thrust = Some(pid.thrust.out());
        } else {
            // We must reset the commanded pitch if not using one of these modes, so control can
            // be handed back to manual controls etc.
            attitudes_commanded.thrust = None;
        }
    }

    pub fn apply_attitude_fixed_wing(
        &self,
        params: &Params,
        attitudes_commanded: &mut CtrlInputs,
        rates_commanded: &mut CtrlInputs,
        pid_attitude: &mut PidGroup,
        filters: &mut PidDerivFilters,
        coeffs: &CtrlCoeffGroup,
        // input_map: &InputMap,
        // max_speed_ver: f32,
    ) {
        // todo: Consider if you should not have separate autopilot for rate and attitude!
        // todo if so, delete the apply_rate ones, and rename apply_attitude to apply.

        if self.takeoff {
            // *attitudes_commanded = CtrlInputs {
            //     pitch: Some(0.),
            //     roll: Some(0.),
            //     yaw: Some(0.),
            //     thrust: Some(flight_ctrls::quad::takeoff_speed(params.tof_alt, max_speed_ver)),
            // };
        } else if let Some(ldg_cfg) = &self.land_fixed_wing {
            // *attitudes_commanded = CtrlInputs {
            //     pitch: Some(0.),
            //     roll: Some(0.),
            //     yaw: Some(0.),
            //     thrust: Some(flight_ctrls::quad::landing_speed(params.tof_alt, max_speed_ver)),
            // };
            // todo: DRY between quad and FC here, although the diff is power vs pitch.
        } else if let Some((alt_type, alt_commanded)) = self.alt_hold {
            // Set a vertical velocity for the inner loop to maintain, based on distance
            let dist = match alt_type {
                AltType::Msl => alt_commanded - params.baro_alt_msl,
                AltType::Agl => alt_commanded - params.tof_alt,
            };

            pid_attitude.pitch = pid::calc_pid_error(
                // If just entering this mode, default to 0. pitch as a starting point.
                attitudes_commanded.pitch.unwrap_or(0.),
                dist,
                &pid_attitude.pitch,
                coeffs.pitch.k_p_attitude,
                coeffs.pitch.k_i_attitude,
                coeffs.pitch.k_d_attitude,
                &mut filters.pitch_attitude,
                DT_ATTITUDE,
            );

            // todo: Set this at rate or attitude level?

            attitudes_commanded.pitch = Some(pid_attitude.pitch.out());

            // todo: Commented out code below is if we use the velocity loop.
            // // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
            // attitudes_commanded.pitch =
            //     Some(flight_ctrls::quad::enroute_speed_ver(dist, max_speed_ver, params.tof_alt));
        } else {
            // We must reset the commanded pitch if not using one of these modes, so control can
            // be handed back to manual controls etc.
            attitudes_commanded.pitch = None;
        }
    }

    // /// Apply the autopilot controls.
    // pub fn _apply_rate_quad(
    //     &self,
    //     params: &Params,
    //     rates_commanded: &mut CtrlInputs,
    //     max_speed_ver: f32,
    //     pid: &mut PidGroup,
    //     filters: &mut PidDerivFilters,
    //     coeffs: &CtrlCoeffGroup,
    //     dt: f32,
    // ) {
    //     if let Some((alt_type, alt_commanded)) = self.alt_hold {
    //         let dist = match alt_type {
    //             AltType::Msl => alt_commanded - params.baro_alt_msl,
    //             AltType::Agl => alt_commanded - params.tof_alt,
    //         };
    //         // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
    //         rates_commanded.thrust =
    //             flight_ctrls::quad::enroute_speed_ver(dist, max_speed_ver, params.tof_alt);
    //     }
    //
    //     if self.yaw_assist {
    //         // Blend manual inputs with the autocorrection factor. If there are no manual inputs,
    //         // this autopilot mode should neutralize all sideslip.
    //         let hor_dir = 0.; // radians
    //         let hor_speed = 0.; // m/s
    //
    //         let yaw_correction_factor = ((hor_dir - params.s_yaw) / TAU) * YAW_ASSIST_COEFF;
    //
    //         if hor_speed > YAW_ASSIST_MIN_SPEED {
    //             rates_commanded.yaw += yaw_correction_factor;
    //         }
    //     } else if self.roll_assist {
    //         // todo!
    //         let hor_dir = 0.; // radians
    //         let hor_speed = 0.; // m/s
    //
    //         let roll_correction_factor = (-(hor_dir - params.s_yaw) / TAU) * YAW_ASSIST_COEFF;
    //
    //         if hor_speed > YAW_ASSIST_MIN_SPEED {
    //             rates_commanded.yaw += roll_correction_factor;
    //         }
    //     }
    //

    // pub fn _apply_rate_fixed_wing(&self, params: &Params, rates_commanded: &mut CtrlInputs) {
    //     if let Some((alt_type, alt_commanded)) = self.alt_hold {
    //         let dist = match alt_type {
    //             AltType::Msl => alt_commanded - params.baro_alt_msl,
    //             AltType::Agl => alt_commanded - params.tof_alt,
    //         };
    //         // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
    //         rates_commanded.thrust = 0.; // todo
    //     }
    // }
}
