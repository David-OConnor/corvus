//! Contains misc and utility functions.

use cmsis_dsp_sys as dsp_sys;

use crate::sensors_shared::BattCellCount;

/// Used to satisfy RTIC resource Send requirements.
pub struct IirInstWrapper {
    pub inner: dsp_sys::arm_biquad_casd_df1_inst_f32,
}
unsafe impl Send for IirInstWrapper {}

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
/// C+P from firmware.
pub const fn crc_init(poly: u8) -> [u8; 256] {
    let mut lut = [0; 256];

    let mut i = 0;
    while i < 256 {
        // Can't use for loops in const fns
        let mut crc = i as u8;

        let mut j = 0;
        while j < 8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
            j += 1;
        }
        lut[i] = crc;

        i += 1;
    }

    lut
}

/// CRC8 using a specific poly, includes all bytes from type (buffer[2]) to end of payload.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
/// /// C+P from firmware.
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

const BATT_LUT: [(f32, f32); 21] = [
    (3.27, 0.),
    (3.61, 0.05),
    (3.69, 0.1),
    (3.71, 0.15),
    (3.73, 0.2),
    (3.75, 0.25),
    (3.77, 0.3),
    (3.79, 0.35),
    (3.8, 0.4),
    (3.82, 0.45),
    (3.84, 0.5),
    (3.85, 0.55),
    (3.87, 0.6),
    (3.91, 0.65),
    (3.95, 0.7),
    (3.98, 0.75),
    (4.02, 0.8),
    (4.08, 0.85),
    (4.11, 0.9),
    (4.15, 0.95),
    (4.20, 1.),
];

/// Returns an estimate of battery life, with 0. being empty, and 1. being full.
/// Input is in volts.
/// From Corvus.
/// [Reference with table](https://blog.ampow.com/lipo-voltage-chart/)
pub fn batt_left_from_v(v: f32, cell_count: BattCellCount) -> f32 {
    let per_cell = v / cell_count.num_cells();
    // todo: Temp. Refine this.
    // let empty_v = 3.5;
    // let full_v = 4.2;

    let mut i = 0;

    // todo: Refactor to use an if/else cascade
    if per_cell > BATT_LUT[1].0 {
        i = 1;
    }
    if per_cell > BATT_LUT[2].0 {
        i = 2;
    }
    if per_cell > BATT_LUT[3].0 {
        i = 3;
    }
    if per_cell > BATT_LUT[4].0 {
        i = 4;
    }
    if per_cell > BATT_LUT[5].0 {
        i = 5;
    }
    if per_cell > BATT_LUT[6].0 {
        i = 6;
    }
    if per_cell > BATT_LUT[7].0 {
        i = 7;
    }
    if per_cell > BATT_LUT[8].0 {
        i = 8;
    }
    if per_cell > BATT_LUT[9].0 {
        i = 9;
    }
    if per_cell > BATT_LUT[10].0 {
        i = 10;
    }
    if per_cell > BATT_LUT[11].0 {
        i = 11;
    }
    if per_cell > BATT_LUT[12].0 {
        i = 12;
    }
    if per_cell > BATT_LUT[13].0 {
        i = 13;
    }
    if per_cell > BATT_LUT[14].0 {
        i = 14;
    }
    if per_cell > BATT_LUT[15].0 {
        i = 15;
    }
    if per_cell > BATT_LUT[16].0 {
        i = 16;
    }
    if per_cell > BATT_LUT[17].0 {
        i = 17;
    }
    if per_cell > BATT_LUT[18].0 {
        i = 18;
    }
    if per_cell > BATT_LUT[19].0 {
        i = 19;
    }
    if per_cell > BATT_LUT[20].0 {
        i = 20;
    }

    // todo. Not linear! Just for now.

    let port_through = (per_cell - BATT_LUT[i].0) / (BATT_LUT[i + 1].0 - BATT_LUT[i].0);
    port_through * (BATT_LUT[i + 1].1 - BATT_LUT[i].1) + BATT_LUT[i].1
}
