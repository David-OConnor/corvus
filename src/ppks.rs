//! Present-position keeping system. Fuses GPS, and dead-reckoning.

pub const WAYPOINT_MAX_NAME_LEN: usize = 7; // Characters

#[derive(Clone, Copy)]
pub enum LocationType {
    /// Lattitude and longitude. Available after a GPS fix
    LatLon,
    /// Start at 0, and count in meters away from it.
    Rel0,
}

impl Default for LocationType {
    fn default() -> Self {
        Self::Rel0
    }
}

/// If type is LatLon, `x` and `y` are in degrees. If Rel0, in meters. `z` is in m MSL.
#[derive(Default, Clone)]
pub struct Location {
    pub type_: LocationType,
    // todo: If you use location for other purposes, consider making a separate Waypoint
    // todo type.
    pub name: [u8; WAYPOINT_MAX_NAME_LEN], // utf-8 encoding
    /// Latitude in radians
    pub lat: f32,
    /// Longitude in radians
    pub lon: f32,
    /// Altitude in meters MSL, QFE.
    pub alt_msl: f32,
}

impl Location {
    pub fn new(type_: LocationType, lat: f32, lon: f32, alt_msl: f32) -> Self {
        Self {
            type_,
            name: [0; WAYPOINT_MAX_NAME_LEN],
            lat,
            lon,
            alt_msl,
        }
    }
}
