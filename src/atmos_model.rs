//! This module contains an atmospheric model, for the purpose of mapping
//! barometer readings to altitude. Agnostic to the specific barometer used.
//!
//! https://en.wikipedia.org/wiki/U.S._Standard_Atmosphere

use num_traits::Float;

// 1976 standard atmospheric model, first stage. We use a linear interpolation between these
// points, and the initial ground point.
pub const POINT_0: AltitudeCalPt = AltitudeCalPt {
    pressure: 101_325.,
    altitude: 0.,
    temp: 288.15,
};

pub const POINT_1: AltitudeCalPt = AltitudeCalPt {
    pressure: 22_632.1,
    altitude: 11_000.,
    temp: 216.65,
};

// K / m
const TEMP_LAPSE_RATE: f32 = (POINT_1.temp - POINT_0.temp) / (POINT_1.altitude - POINT_0.altitude);

// https://en.wikipedia.org/wiki/Barometric_formula
const LAPSE_RATE_COEFF: f32 = -9.80665 * 0.0289644 / 8.3144598; // g_0 * M / R*
const LAPSE_RATE_EXP: f32 = LAPSE_RATE_COEFF / TEMP_LAPSE_RATE;

/// Used to map pressure to altitude. Can use a single point, in conjunction with a standard
/// atmosphere model.
/// The default impl is used when setting up the `Altimeter` struct, pre ground-initialization, where
/// it's replaced with a measurement.
pub struct AltitudeCalPt {
    pub pressure: f32, // Pa
    pub altitude: f32, // MSL, via GPS, in meters
    pub temp: f32,     // K
}

impl Default for AltitudeCalPt {
    /// Standard temperature and pressure, at sea level. Note that in practice, we zero to launch elevation
    /// instead of 0 MSL.
    fn default() -> Self {
        Self {
            pressure: 101_325.,
            altitude: 0.,
            /// Kelvin.
            temp: 288.15,
        }
    }
}

/// Estimate barometric QFE altitude (Ie how far above ground), from pressure and temperature.
/// Pressure is in Pa; temp is in K. Result is in m.
/// Uses a linear map between 2 pointers: Either from the standard atmosphere model, or from
/// GPS points, if available.
/// `altimeter_setting` is the ground pressure in vacinityh of operation, in Pa. Temp is in K.
/// https://en.wikipedia.org/wiki/Bar3ometric_formula
/// https://physics.stackexchange.com/questions/333475/how-to-calculate-altitude-from-current-temperature-and-pressure
/// [NOAA formula](https://en.wikipedia.org/wiki/Pressure_altitude)
pub fn estimate_altitude_msl(pressure: f32, temp: f32, ground_cal: &AltitudeCalPt) -> f32 {
    // P = 101.29 * ((temp)/288.08)^5.256   (in kPa)
    // T = 150.4 - .00649h

    // let (point_0, point_1) = if self.gps_cal_init.is_some() && self.gps_cal_air.is_some() {
    //     //     (
    //     //         self.gps_cal_init.as_ref().unwrap(),
    //     //         self.gps_cal_air.as_ref().unwrap(),
    //     //     )
    //     // } else {
    //     //     (&POINT_0, &POINT_1)
    //     // };

    // todo: Which approach? how do we modify the below for temp
    // todo and atmospheric pressure.
    // NOAA formula (See Wikipedia link above). `h` is in feet.
    // Convert pressure from Pa to millibars.
    let p_mb = pressure * 0.01;
    let h = 145_366.45 * (1. - (p_mb / 1_013.25).powf(0.190284));
    // Convert feet to meters.
    h * 0.3048;

    // todo: You need to take ground cal's temp into account, at minimum!
    (((ground_cal.pressure / pressure).powf(1. / 5.257) - 1.) * temp) / 0.00649

    // log_lapse_rate(P/POINT_0.pressure) = (POINT_0.temp + (alt - POINT_0.altitude) * )

    // todo: Temp compensate!
    // todo: You probably want a non-linear, eg exponential model.
    // util::map_linear(pressure, (point_0.pressure, POINT_1.pressure), (point_0.altitude, point_1.altitude))
}
