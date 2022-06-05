//! This module contains an atmospheric model, for the purpose of mapping
//! barometer readings to altitude. Agnostic to the specific barometer used.
//!
//! https://en.wikipedia.org/wiki/U.S._Standard_Atmosphere

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
#[derive(Default)]
pub struct AltitudeCalPt {
    pub pressure: f32, // Pa
    pub altitude: f32, // MSL, via GPS, in meters
    pub temp: f32,     // K
}

// impl Default for AltitudeCalPt {
//     /// Standard temperature and pressure, at sea level. Note that in practice, we zero to launch elevation
//     /// instead of 0 MSL. ( todo: But as an adjustment for MSL, or with this model?)
//     fn default() -> Self {
//         Self {
//             pressure: 101_325.,
//             altitude: 0.,
//             temp: 288.15,
//         }
//     }
// }
