//! Contains misc and utility functions.

use cmsis_dsp_sys as dsp_sys;

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: dsp_sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

/// Utility fn to make up for `core::cmp::max` requiring f32 to impl `Ord`, which it doesn't.
pub fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

pub fn abs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
}

/// Utility function to linearly map an input value to an output
pub fn map_linear(val: f32, range_in: (f32, f32), range_out: (f32, f32)) -> f32 {
    // todo: You may be able to optimize calls to this by having the ranges pre-store
    // todo the total range vals.
    let portion = (val - range_in.0) / (range_in.1 - range_in.0);

    portion * (range_out.1 - range_out.0) + range_out.0
}

/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
pub fn crc_init(lut: &mut [u8; 256], poly: u8) {
    for i in 0..256 {
        let mut crc = i as u8;
        for _ in 0..8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
        }
        lut[i] = crc;
    }
}

/// CRC8 using a specific poly, includes all bytes from type (buffer[2]) to end of payload.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
pub fn calc_crc(lut: &[u8; 256], data: &[u8], mut size: u8) -> u8 {
    let mut crc = 0;
    let mut i = 0;

    while size > 0 {
        size -= 1;
        crc = lut[(crc ^ data[i]) as usize];
        i += 1;
    }
    crc
}

/// Alternative, non-LUT impl from iNav: https://github.com/iNavFlight/inav/wiki/MSP-V2
/// "crc8_dvb_s2 checksum algorithm. This is a single byte CRC algorithm that is ... robust"
pub fn crc8_dvb_s2(crc: u8, a: u8) -> u8 {
    let mut crc = crc ^ a;

    for _ in 0..8 {
        if (crc & 0x80) != 0 {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc <<= 1;
        }
    }
    crc
}
