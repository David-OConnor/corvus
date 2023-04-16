//! C+P from gps_mag_can, `protocol.rs`

//! Contains code for transmitting fixes and related info
//! using the DroneCan or Cyphal protocols.
//!
//! [DroneCAN standard types ref](https://dronecan.github.io/Specification/7._List_of_standard_data_types/)

use core::sync::atomic::AtomicUsize;

use packed_struct::{prelude::*, PackedStruct};

use crate::gps::{Fix, FixType};

pub const GNSS_PAYLOAD_SIZE: usize = 38;

pub const GNSS_FIX_ID: u16 = 1_063;

pub const CONFIG_SIZE: usize = 4;

pub static TRANSFER_ID_FIX: AtomicUsize = AtomicUsize::new(0);

// todo: Node status.

pub struct Config {
    pub fd_mode: bool,
    /// Kbps
    pub can_bitrate: u16,
    /// Hz. Maximum of 18Hz with a single constellation. Lower rates with fused data. For example,
    /// GPS + GAL is 10Hz max.
    pub broadcast_rate: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            fd_mode: false,
            can_bitrate: 1_000,
            broadcast_rate: 5,
        }
    }
}

impl Config {
    pub fn from_bytes(buf: &[u8; CONFIG_SIZE]) -> Self {
        Self {
            fd_mode: buf[0] != 0,
            can_bitrate: u16::from_le_bytes(buf[1..3].try_into().unwrap()),
            broadcast_rate: buf[3]
        }
    }

    pub fn to_bytes(&self) -> [u8; CONFIG_SIZE] {
        let mut result = [0; CONFIG_SIZE];

        result[0] = self.fd_mode as u8;
        result[1..3].clone_from_slice(&self.can_bitrate.to_le_bytes());
        result[3] = self.broadcast_rate;

        result
    }
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum GnssTimeStandard {
    None = 0,
    Tai = 1,
    Utc = 2,
    Gps = 3,
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum FixStatus {
    NoFix = 0,
    TimeOnly = 1,
    Fix2d = 2,
    Fix3d = 3,
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum GnssMode {
    Single = 0,
    Dgps = 1,
    Rtk = 2,
    Ppp = 3,
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum GnssSubMode {
    DgpsOtherRtkFloat = 0,
    DgpsSbasRtkFixed = 1,
}

/// Optional subdata for Fix
pub struct EcefPositionVelocity {
    pub velocity_xyz: [f32; 3],
    pub position_xyz_mm: [i64; 3], // [i36; 3]
    // todo: Tail optimization (no len field) since this is the last field?
    pub covariance: [Option<f32>; 36], // todo: [f16; <=36?]
}

/// https://dronecan.github.io/Specification/7._List_of_standard_data_types/
/// See `Fix2` data type. Contains the whole packet.
/// See this for field descriptions.
#[derive(PackedStruct)]
// todo?
#[packed_struct(bit_numbering = "msb0", endian = "lsb")] // todo: Bit little-endian?
pub struct FixDronecan {
    #[packed_field(bits = "0..7")]
    pub timestamp: u8, // 7 bits
    #[packed_field(bits = "7..14")]
    pub gnss_timestamp: u8, // 7 bits
    #[packed_field(bits = "14..17", ty = "enum")]
    pub gnss_time_standard: GnssTimeStandard, // 3 bits
    // 13-bit pad
    #[packed_field(bits = "30..38")]
    pub num_leap_seconds: u8, // 0 for unknown
    #[packed_field(bits = "39..76")]
    pub longitude_deg_1e8: i64, // 37 bits
    #[packed_field(bits = "76..113")]
    pub latitude_deg_1e8: i64, // 37 bits
    #[packed_field(bits = "113..140")]
    pub height_ellipsoid_mm: i32, // 27 bits
    #[packed_field(bits = "140..167")]
    pub height_msl_mm: i32, // 27 bits
    #[packed_field(bits = "167..263")]
    // pub ned_velocity: [f32; 3],
    pub ned_velocity: [u32; 3], // todo: packed_struct currently doesn't support float.
    #[packed_field(bits = "263..269")]
    pub sats_used: u8, // 6 bits.
    #[packed_field(bits = "269..271", ty = "enum")]
    pub fix_status: FixStatus, // 2 bits.
    #[packed_field(bits = "271..275", ty = "enum")]
    pub mode: GnssMode, // 4 bits.
    #[packed_field(bits = "275..281", ty = "enum")]
    pub sub_mode: GnssSubMode, // 6 bits. todo: Why 6 bits?
    /// Note re variable-length arrays in DroneCAN:
    /// "Normally, a dynamic array will be encoded as a sequence of encoded items,
    /// prepended with an unsigned integer field representing the number of contained items
    /// - the length field. The bit width of the length field is a function of the maximum number
    /// of items in the array: ⌈log2(X + 1)⌉, where X is the maximum number of items in the array.
    /// For example, if the maximum number of items is 251, the length field bit width must be 8
    /// bits, or if the maximum number of items is 1, the length field bit width will be just a
    /// single bit.
    ///
    /// For len of 36, we get 5.2. So, 6-bits len field, or 5?
    // pub covariance: [Option<f32>; 36], // todo: [f16; <=36?] // todo: Currently unused.
    #[packed_field(bits = "281..297")]
    // pub pdop: f32, // 16 bits
    pub pdop: u16, // 16 bits  // todo: packed_struct currently doesn't support float.
                   // pub ecef_position_velocity: Option<EcefPositionVelocity>, // 0 or 1.  // todo: Currently unused.
}

impl FixDronecan {
    pub fn from_fix(fix: &Fix) -> Self {
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

        // `packed_struct` doesn't support floats; convert to integers.
        let ned0_bytes = fix.ned_velocity[0].to_le_bytes();
        let ned1_bytes = fix.ned_velocity[1].to_le_bytes();
        let ned2_bytes = fix.ned_velocity[2].to_le_bytes();

        let ned_velocity = [
            u32::from_le_bytes(ned0_bytes),
            u32::from_le_bytes(ned1_bytes),
            u32::from_le_bytes(ned2_bytes),
        ];

        let pdop_bytes = (fix.pdop * 100.).to_le_bytes(); // todo: units?

        // todo: order?
        let pdop = u16::from_le_bytes([pdop_bytes[1], pdop_bytes[0]]);

        FixDronecan {
            // todo: Current timestamp, or from the fix?
            timestamp: (fix.timestamp * 1_000.) as u8, // todo?
            gnss_timestamp: 0,                         // todo
            gnss_time_standard: GnssTimeStandard::Utc, // todo
            // 13-bit pad
            num_leap_seconds: 0, // todo
            longitude_deg_1e8: (fix.lon * 100_000_000.) as i64,
            latitude_deg_1e8: (fix.lat * 100_000_000.) as i64,
            height_ellipsoid_mm: (fix.elevation_hae * 1_000.) as i32,
            height_msl_mm: (fix.elevation_msl * 1_000.) as i32,
            // todo: Groundspeed? Or is that only from NED vel?
            // todo: NED vel DroneCAN is in m/s, like our format, right?
            ned_velocity,
            sats_used: fix.sats_used,
            fix_status,
            mode: GnssMode::Dgps,                     // todo
            sub_mode: GnssSubMode::DgpsOtherRtkFloat, // todo?
            // covariance: [None; 36],                   // todo?
            // pdop: fix.pdop * 100., // todo: units?
            pdop, // todo: units?
                  // ecef_position_velocity,
        }
    }
}
