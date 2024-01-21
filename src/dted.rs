//! Digital Terrain Elevation Data. Stores elevation data. This can be used, for example, to generated
//! an AGL altitude from GNSS data.
//!
//! GNSS data is stored to flash memory not onboard the MCU, and loaded into RAM as required.
//!
//! It includes functionality that defines areas that are likely to have tall buildings, or trees.
//!
//! Uses Earth Gravitational Model 1996 (EGM96) geoid, not WGS84 ellipsoid. (todo: Come back to this,
//! depending on the source you use)
//!
//! todo: What DTED Level? Likely DTED 0, ie 30 arc second post separation (1km). Option to go
//! todo higher res for areas of interest.

use hal::flash::{Flash};

/// A single point of elevation data
struct DtedPost {
    lat: f32,
    lon: f32,
    /// Meters. todo: Unit? Reference?
    elevation: f32
}