//! C+P from gps_mag_can, `protocol.rs`

//! Contains code for transmitting fixes and related info
//! using the DroneCan or Cyphal protocols.
//!
//! [DroneCAN standard types ref](https://dronecan.github.io/Specification/7._List_of_standard_data_types/)

use core::sync::atomic::AtomicUsize;

use packed_struct::{prelude::*, PackedStruct};

use dronecan::{
    f16,
    gnss::{EcefPositionVelocity, FixDronecan, FixStatus, GnssMode, GnssSubMode, GnssTimeStandard},
    ConfigCommon, PAYLOAD_SIZE_CONFIG_COMMON,
};

use crate::gnss::Covariance;

use ahrs::{Fix, FixType};

pub const GNSS_PAYLOAD_SIZE: usize = 38;

pub const GNSS_FIX_ID: u16 = 1_063;

pub const CONFIG_SIZE: usize = 4 + PAYLOAD_SIZE_CONFIG_COMMON;

pub static TRANSFER_ID_FIX: AtomicUsize = AtomicUsize::new(0);

/// Create a Dronecan Fix2 from our Fix format, based on Ublox's.
pub fn can_fix_from_ublox_fix(fix: &Fix, cov: &Covariance) -> FixDronecan {
    let fix_status = match fix.type_ {
        FixType::NoFix => FixStatus::NoFix,
        FixType::DeadReckoning => FixStatus::Fix2d, // todo?
        FixType::Fix2d => FixStatus::Fix2d,
        FixType::Fix3d => FixStatus::Fix3d,
        FixType::Combined => FixStatus::Fix3d,
        FixType::TimeOnly => FixStatus::TimeOnly,
    };

    // todo
    let ecef_position_velocity = EcefPositionVelocity {
        velocity_xyz: [0.; 3],
        position_xyz_mm: [0; 3], // [i36; 3]
        // todo: Tail optimization (no len field) since this is the last field?
        covariance: [None; 36], // todo: [f16; <=36?]
    };

    // NED velocity from Ublox is in i32, mm/s. Dronecan uses f32, m/s.
    // `packed_struct` doesn't support floats; convert to integers.
    let ned0_bytes = (fix.ned_velocity[0] as f32 / 1_000.).to_le_bytes();
    let ned1_bytes = (fix.ned_velocity[1] as f32 / 1_000.).to_le_bytes();
    let ned2_bytes = (fix.ned_velocity[2] as f32 / 1_000.).to_le_bytes();

    let ned_velocity = [
        u32::from_le_bytes(ned0_bytes),
        u32::from_le_bytes(ned1_bytes),
        u32::from_le_bytes(ned2_bytes),
    ];

    // packed-struct workaround for not having floats.
    // And handling 100x factor between Ublox and DroneCan.
    let pdop = f16::from_f32((fix.pdop as f32) / 100.);
    let pdop_bytes = pdop.to_le_bytes();
    let pdop = u16::from_le_bytes([pdop_bytes[0], pdop_bytes[1]]);

    // For now, only position covariance.
    // todo: QC order.
    let covariance = [
        u16::from_le_bytes(f16::from_f32(cov.pos_nn).to_le_bytes()),
        u16::from_le_bytes(f16::from_f32(cov.pos_ne).to_le_bytes()),
        u16::from_le_bytes(f16::from_f32(cov.pos_nd).to_le_bytes()),
        u16::from_le_bytes(f16::from_f32(cov.pos_ee).to_le_bytes()),
        u16::from_le_bytes(f16::from_f32(cov.pos_ed).to_le_bytes()),
        u16::from_le_bytes(f16::from_f32(cov.pos_dd).to_le_bytes()),
    ];

    FixDronecan {
        timestamp: (fix.timestamp_s * 1_000_000.) as u64, // us.
        gnss_timestamp: fix.datetime.timestamp_micros() as u64,
        gnss_time_standard: GnssTimeStandard::Utc, // todo
        // 13-bit pad
        num_leap_seconds: 0, // todo
        // We must multiply by 10 due to the higher precion format used
        // in DroneCan.
        longitude_deg_1e8: (fix.lon_e7 as i64) * 10,
        latitude_deg_1e8: (fix.lat_e7 as i64) * 10,
        height_ellipsoid_mm: fix.elevation_hae,
        height_msl_mm: fix.elevation_msl,
        ned_velocity,
        sats_used: fix.sats_used,
        fix_status,
        mode: GnssMode::Single, // Hard-set; not using DGPS, RTK, or PPP
        sub_mode: GnssSubMode::DgpsOtherRtkFloat,
        covariance_len: 6,
        covariance,
        pdop,
        // ecef_position_velocity: 0, // Must be 0.
        pad: 0, // Must be 0.
                // ecef_position_velocity,
    }
}
