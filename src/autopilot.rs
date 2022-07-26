//! This module contains code related to various autopilot modes.

use core::f32::consts::TAU;

use crate::{
    flight_ctrls::{
        self,
        common::{AltType, CtrlInputs, InputMap, Params},
        quad::{InputMode, POWER_LUT, YAW_ASSIST_COEFF, YAW_ASSIST_MIN_SPEED},
    },
    pid::{self, CtrlCoeffGroup, PidDerivFilters, PidGroup},
    ppks::Location,
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
    pub takeoff: bool,
    /// Land automatically
    pub land: bool,
    /// Recover to stable, altitude-holding flight. Generally initiated by a "panic button"-style
    /// switch activation
    pub recover: Option<f32>, // value is MSL alt to hold, eg our alt at time of command.
    pub orbit: Option<Orbit>,
}

impl AutopilotStatus {
    pub fn apply_attitude_quad(
        &self,
        params: &Params,
        attitudes_commanded: &mut CtrlInputs,
        input_map: &InputMap,
        max_speed_ver: f32,
    ) {
        // todo: Come back to these autopilot modes.
        // Initiate a recovery, regardless of control mode.
        // todo: Set commanded alt to current alt.
        if let Some(alt_msl_commanded) = self.recover {
            let dist_v = alt_msl_commanded - params.baro_alt_msl;

            // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
            let thrust =
                flight_ctrls::quad::enroute_speed_ver(dist_v, max_speed_ver, params.tof_alt);

            // todo: DRY from alt_hold autopilot code.

            // todo: Figure out exactly what you need to pass for the autopilot modes to inner_flt_cmd
            // todo while in acro mode.
            // *attitudes_commanded = CtrlInputs {
            //     pitch: Some(input_map.calc_pitch_angle(0.),
            //     roll: Some(input_map.calc_roll_angle(0.),
            //     yaw: Some(input_map.calc_yaw_rate(0.),
            //     thrust,
            // };
        }

        // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
        // either MSL or AGL.
        if let Some((alt_type, alt_commanded)) = self.alt_hold {
            // Set a vertical velocity for the inner loop to maintain, based on distance
            let dist = match alt_type {
                AltType::Msl => alt_commanded - params.baro_alt_msl,
                AltType::Agl => alt_commanded - params.tof_alt,
            };
            // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
            attitudes_commanded.thrust =
                Some(flight_ctrls::quad::enroute_speed_ver(dist, max_speed_ver, params.tof_alt));
        }

        if self.takeoff {
            *attitudes_commanded = CtrlInputs {
                pitch: Some(0.),
                roll: Some(0.),
                yaw: Some(0.),
                thrust: Some(flight_ctrls::quad::takeoff_speed(params.tof_alt, max_speed_ver)),
            };
        } else if self.land {
            *attitudes_commanded = CtrlInputs {
                pitch: Some(0.),
                roll: Some(0.),
                yaw: Some(0.),
                thrust: Some(flight_ctrls::quad::landing_speed(params.tof_alt, max_speed_ver)),
            };
        }

        // if let Some(alt_msl_commanded) = autopilot_status.recover {
        //     let dist_v = alt_msl_commanded - params.baro_alt_msl;
        //
        //     // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
        //     let thrust =
        //         flight_ctrls::quad::enroute_speed_ver(dist_v, cfg.max_speed_ver, params.tof_alt);
        //
        //     // todo: DRY from alt_hold autopilot code.
        //
        //     // todo: Figure out exactly what you need to pass for the autopilot modes to inner_flt_cmd
        //     // todo while in acro mode.
        //     *velocities_commanded = CtrlInputs {
        //         pitch: Some(input_map.calc_pitch_angle(0.))
        //         roll: (input_map.calc_roll_angle(0.)),
        //         yaw: input_map.calc_yaw_rate(0.)),
        //         thrust,
        //     };
        // }
        //
        // // If in acro or attitude mode, we can adjust the throttle setting to maintain a fixed altitude,
        // // either MSL or AGL.
        // if let Some((alt_type, alt_commanded)) = autopilot_status.alt_hold {
        //     // Set a vertical velocity for the inner loop to maintain, based on distance
        //     let dist = match alt_type {
        //         AltType::Msl => alt_commanded - params.baro_alt_msl,
        //         AltType::Agl => alt_commanded - params.tof_alt,
        //     };
        //     // `enroute_speed_ver` returns a velocity of the appropriate sine for above vs below.
        //     velocities_commanded.thrust =
        //         flight_ctrls::quad::enroute_speed_ver(dist, cfg.max_speed_ver, params.tof_alt);
        // }
    }

    pub fn apply_attitude_fixed_wing(
        &self,
        params: &Params,
        attitudes_commanded: &mut CtrlInputs,
        // input_map: &InputMap,
        // max_speed_ver: f32,
    ) {
    }

    // todo: Consider if you should not have separate autopilot for rate and attitude!
    // todo if so, delete the apply_rate ones, and rename apply_attitude to apply.

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
    //     // We use the rate thrust PID component here; it's not used elsewhere, so this is fine.
    //     if let Some((_, _)) = self.alt_hold {
    //         pid.thrust = pid::calc_pid_error(
    //             rates_commanded.thrust,
    //             params.v_z,
    //             &pid.thrust,
    //             coeffs.thrust.k_p_rate,
    //             coeffs.thrust.k_i_rate,
    //             coeffs.thrust.k_d_rate,
    //             &mut filters.thrust,
    //             dt,
    //         );
    //
    //         rates_commanded.thrust = pid.thrust.out();
    //     }
    // }
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
