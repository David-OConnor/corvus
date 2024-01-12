//! Parses DroneCAN fixes into our own Fix format.

use ahrs::{Fix, FixType};
use chrono::naive::NaiveDateTime;
use defmt::println;
use dronecan::{
    f16,
    gnss::{EcefPositionVelocity, FixDronecan, FixStatus, GnssMode, GnssSubMode, GnssTimeStandard},
    ConfigCommon, PAYLOAD_SIZE_CONFIG_COMMON,
};
use packed_struct::{prelude::*, PackedStruct};

/// Parse a DroneCAN Fix2. Output our AHRS Fix format.
pub fn parse_fix(buf: &[u8]) -> Result<Fix, PackingError> {
    // todo: QC the len etc of the buf, and def don't unwrap below.
    let fix_dc = FixDronecan::unpack(buf.try_into().unwrap())?;

    // This mirror `can_fix_from_ublox_fix` in the GNSS firmware; opposite direction.

    let type_ = match fix_dc.fix_status {
        FixStatus::NoFix => FixType::NoFix,
        FixStatus::Fix2d => FixType::Fix2d,
        FixStatus::Fix3d => FixType::Fix3d,
        FixStatus::TimeOnly => FixType::TimeOnly,
    };

    // DC stores f32 as a u32 due to PacketStruct limitations; coerce bytes to f32.
    let ned_v_0 = f32::from_le_bytes(fix_dc.ned_velocity[0].to_le_bytes());
    let ned_v_1 = f32::from_le_bytes(fix_dc.ned_velocity[1].to_le_bytes());
    let ned_v_2 = f32::from_le_bytes(fix_dc.ned_velocity[2].to_le_bytes());

    // NED velocity from Ublox is in i32, mm/s. Dronecan uses f32, m/s.
    let ned_velocity = [
        (ned_v_0 * 1_000.) as i32,
        (ned_v_1 * 1_000.) as i32,
        (ned_v_2 * 1_000.) as i32,
    ];

    // todo: Do we want this fix, or
    let result = Fix {
        timestamp_s: fix_dc.timestamp as f32 / 1_000_000.,
        datetime: NaiveDateTime::from_timestamp_micros(fix_dc.gnss_timestamp as i64)
            .unwrap_or_default(),
        type_,
        lat_e7: (fix_dc.latitude_deg_1e8 / 10) as i32,
        lon_e7: (fix_dc.longitude_deg_1e8 / 10) as i32,
        elevation_hae: fix_dc.height_ellipsoid_mm,
        elevation_msl: fix_dc.height_msl_mm,
        ground_speed: 0, // todo Do this.
        ned_velocity,
        heading: None, // todo: Infer from NED?
        sats_used: fix_dc.sats_used,
        pdop: fix_dc.pdop as u16 * 100,
    };

    Ok(result)
}
