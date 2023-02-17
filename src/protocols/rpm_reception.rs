//! This module contains code for interpreting (bidirectional) DSHOT RPM readings.
//! Management of the timers, motor lines, DMA reception etc is handled in the `dshot` module,
//! and in ISRs in `main`. This module handles interpretation of the buffers collected
//! by those processes.
//!
//!
//! How to convert edge timings to bits:
//!
//! Buf: (With idle motor)
//! What is this telling us?
//! [5608, 6508, 6927, 7369, 7811, 8712, 9143, 9582, 10024, 10910, 11353, 11798, 12238, 12680, 13123,
//! 14451, 0, 0..]
//!
//! One bit = about 450 ticks
//!
//! low, high, low, high...
//! 2, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 3 (end)
//! = 338257
//!
//! Comparing to a prev image, this means:
//! 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1(1 time, since that's 20 bits))
//!
//! How to convert these 20 bits to RPM:
//!
//! Example, for 0 throttle:
//!
//! Raw data: 0b01010_01010_01010_10001 = 338257
//! post shift: 507385
//!
//! nibbles: 15, 15, 15, 25 = f, f, f, 0
//!
//! compiled 16-bit number = 65_520 = 0xfff0)
//! = 0b1111_1111_1111_0000
//!
//!
//! shift = 0b111
//! val = 0b1_1111_1111
//! crc = b0000
//!
//! period in us = val << shift = 65408 (Is this right?)
//! (more handling (scaling etc) is required to get RPM from this value)
//!
//! ---------
//! As above, with an idle throttle level:
//!
//! GCR = 432427
//! post shift (20-bit): 382398
//! 16-bit: 46556 = 1011_0101_1101_1100
//!
//! shift = 0b101
//! val = 1_0101_1101
//! crc = 1100
//!
//! period in us = 11168
//!
//! CRC passes.

use super::dshot::{self, calc_crc, DSHOT_SPEED, REC_BUF_LEN, TIM_CLK};

use num_traits::float::FloatCore; // round

use crate::flight_ctrls::common::RpmReadings;

use defmt::println;

// Number of counter ticks per bit.
// The differences tend to come out a bit lower, b ut this is the number I've calced.
// This corresponds to a period of 5/4 * the DSHOT freq, per its spec.
const BIT_LEN: u16 = (TIM_CLK / (5 * DSHOT_SPEED / 4) - 1) as u16;

const GCR_LEN: usize = 20;

#[derive(Clone, Copy)]
enum EscData {
    /// Revolutions per minute
    Rpm(f32),
    Telem(EscTelemType, u8),
}

#[derive(Clone, Copy)]
// todo: These could hold a value
enum EscTelemType {
    Temp,
    Voltage,
    Current,
    Debug1,
    Debug2,
    Debug3,
    State,
}

/// Computes RPM, in revolutions-per-minute, from the decoded data packet.
/// Hardware-level decoding, and initial processing is handled upstream of this.
/// See https://brushlesswhoop.com/dshot-and-bidirectional-dshot/, "eRPM Telemetry Frame (from ESC)".
fn rpm_from_data(packet: u16, pole_count: u8) -> Result<EscData, RpmError> {
    let crc_read = packet & 0b1111;
    let data = packet >> 4;

    // 0 RPM is encoded as this value; if we were to complete the computation, we'd
    // get a low, but positive RPM value that this protocol is unable to directly encode
    // as 0.
    if data == 0xfff {
        return Ok(EscData::Rpm(0.));
    }

    // Right shift 4 to exclude the CRC itself from the calculation.
    if crc_read != calc_crc(data) {
        // todo: Put this print statement back, and get to the bottom of this!
        // println!("C {}", packet);
        return Err(RpmError::Crc);
    }

    // todo: Come back to telemetry later.
    // Parse extended telemetry if avail. (This may be required to avoid misreading the data?)
    if ((data >> 8) & 1) == 0 {
        let telem_type_val = packet >> 8;
        // Telemetry is passed

        let val = packet & 0xff; // 8 bits vice 9 for rpm data

        let telem_type = match telem_type_val {
            0x02 => EscTelemType::Temp,
            0x04 => EscTelemType::Voltage,
            0x06 => EscTelemType::Current,
            0x08 => EscTelemType::Debug1,
            0x0A => EscTelemType::Debug2,
            0x0C => EscTelemType::Debug3,
            0x0E => EscTelemType::State,
            _ => return Err(RpmError::TelemType),
        };

        Ok(EscData::Telem(telem_type, val as u8))
    } else {
        // println!("R");
        // RPM data
        let shift = data >> 9;
        let base = data & 0b1_1111_1111;
        let period_us = base << shift;

        // println!("S {} V {} C {}", shift, base, crc_read);

        // Period is in us, and is "erpm". Convert to RPM with further scaling.

        // Some info on BF how to do this. This appears to be the source document for
        // RPM decoding:
        // https://github.com/betaflight/betaflight/blob/f39f267301bcad27ec34bdbbf987c4bf595ea136/src/main/drivers/dshot.c#L320
        // I'd like to move away from bidir DSHOT; I don't like this.

        // Scaler.
        const C1: f32 = 1_000_000. * 60. / 100.;

        let erpm = (C1 + period_us as f32 / 2.) / period_us as f32;
        let rpm = erpm * 200. / pole_count as f32;

        // Note: This is currently revolutions per second; not minute.
        Ok(EscData::Rpm(rpm))
    }
}

pub enum RpmError {
    Gcr,
    Crc,
    TelemType,
    TempTelem,
}

/// Convert our arrays of high and low edge counts to a 20-bit integer.  u32 since it's 20 bits.
/// `counts` alternates low and high edges; counts[0] is low.
pub fn edge_counts_to_u32(counts: &[u16]) -> Result<u32, RpmError> {
    // Start at index 1 of edges; we compare to i-1.
    let mut edge_i = 1;

    // println!("co {} {} {} {}", counts[0], counts[1], counts[2], counts[3]);

    // Assemble bit lengths of each (high or low) value from edge timings.
    let mut value_lens = [0; 20]; // Generally smaller than this.

    for _ in 0..25 {
        if counts[edge_i] == 0 {
            // A 0 value means we're past the last edge.
            break;
        }

        // Likely 2 triggers on the same count.
        let mut bits_since_last_edge = if counts[edge_i - 1] > counts[edge_i] {
            0
        } else {
            let as_f = (counts[edge_i] - counts[edge_i - 1]) as f32 / BIT_LEN as f32;
            as_f.round() as u16
        };

        // We discard the first bit; it's a low bit to indicate the start of the sequence.
        if edge_i == 1 {
            bits_since_last_edge -= 1;

            // If the first bit is high, and bits_since_last_edge (after subtraction) is 0,
            // append 0 and move on. It is the only place a 0 is allowed here.
            // This preserved the even-numbers-are-low logic below, while allowing
            // the first bit to be high.
            if bits_since_last_edge == 0 {
                value_lens[edge_i - 1] = bits_since_last_edge;
                edge_i += 1;
                continue;
            }
        }

        if bits_since_last_edge == 0 {
            // Continue without incrementing `edge_i`, or modifying our counts.
            continue;
        }

        value_lens[edge_i - 1] = bits_since_last_edge;
        edge_i += 1;
    }

    // println!("E: {} {} {} {} {}", value_lens[0], value_lens[1], value_lens[2], value_lens[3], edge_i);

    // `edge_i` is now the number of values we have + 1.
    let num_vals = edge_i - 1;

    let mut bits = [false; GCR_LEN];
    let mut bits_i = 0;

    // For example, for 0 RPM, we may now have the following `value_lens`:
    // 1, 1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 3
    // This translates into:
    // low high low high low low high low high low low high low high low high low low low high
    // (The last high is implicit). `num_vals` is 15.

    // In thsi example, bits_i should increment as follows:
    // 0, 1, 2, 3, 4, 6, 1, 8, 9, 11, 12, 13, 14, 15, 16, 19

    let mut final_bit_i = 0;

    for i in 0..num_vals {
        let len_this_pulse = value_lens[i];

        for bit_i in bits_i..bits_i + len_this_pulse {
            if bit_i > 19 {
                println!("ESC read error");
                return Err(RpmError::Gcr);
            } else {
                // Even-indexed `value_lens` correspond to low edges. (ie 0)
                // So, we make it true (line is high, value is 1) on odd indices.
                bits[bit_i as usize] = i % 2 != 0;

                final_bit_i = bit_i;
            }
        }

        // We've hit the last edge; set the remaining bits through
        // the end of the buffer to this edge.
        if i == num_vals - 1 {
            for i_final in final_bit_i + 1..20 {
                bits[i_final as usize] = !bits[final_bit_i as usize];
            }
        }

        bits_i += len_this_pulse;
    }

    // println!("Bits: {} {} {} {} {} {}", bits[14], bits[15], bits[16], bits[17], bits[18], bits[19]);

    // Assemble the resulting 20-bit integer from our array of true and false bits.
    let mut result = 0;
    for (i, v) in bits.iter().enumerate() {
        result |= (*v as u32) << (GCR_LEN - 1 - i)
    }

    // println!("G{}", result);
    Ok(result)
}

/// Map 5-bit nibbles to 4-bit nibbles, per the DSHOT RPM protocol.
/// It outputs `u16`, since these are part of a 16-bit integer when combined.
pub fn reduce_bit_count_map(val: u8) -> Result<u16, RpmError> {
    let result = match val {
        0x19 => 0,
        0x1b => 1,
        0x12 => 2,
        0x13 => 3,
        0x1d => 4,
        0x15 => 5,
        0x16 => 6,
        0x17 => 7,
        0x1a => 8,
        0x09 => 9,
        0x0a => 0xa,
        0x0b => 0xb,
        0x1e => 0xc,
        0x0d => 0xd,
        0x0e => 0xe,
        0x0f => 0xf,
        _ => {
            return Err(RpmError::Gcr);
        }
    };

    Ok(result)
}

/// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/: `eRPM Transmission`
/// Convert from 20-bits to 16-bits, as part of the RPM decoding process.
pub fn reduce_bit_count(val: u32) -> Result<u16, RpmError> {
    let mask = 0b1_1111;

    let nibble_0 = (val & mask) as u8;
    let nibble_1 = ((val >> 5) & mask) as u8;
    let nibble_2 = ((val >> 10) & mask) as u8;
    let nibble_3 = ((val >> 15) & mask) as u8;

    let result = reduce_bit_count_map(nibble_0)?
        | (reduce_bit_count_map(nibble_1)? << 4)
        | (reduce_bit_count_map(nibble_2)? << 8)
        | (reduce_bit_count_map(nibble_3)? << 12);

    // println!("20 {} 16 {}", val, result);

    Ok(result)
}

/// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/: `Decoding eRPM frame`
fn gcr_step_1(val: u32) -> u32 {
    val ^ (val >> 1)
}

/// Update RPM satus for a single motor. This goes through each step.
fn process_rpm(payload: &[u16; REC_BUF_LEN], pole_count: u8) -> Result<f32, RpmError> {
    // todo: Telemetry?

    // Parse our GCR data from edge timings, with an initial bit-shift maneuver.
    let gcr = gcr_step_1(edge_counts_to_u32(payload)?);

    // Convert our 20-bit raw GCR data to the 16-bit data packet, using a specific mapping.
    let packet = reduce_bit_count(gcr)?;

    // todo: Don't hard code the motor mapping!!
    match rpm_from_data(packet, pole_count)? {
        EscData::Rpm(rpm) => Ok(rpm),
        EscData::Telem(_, _) => {
            // todo: We are treating this as an error for now.
            Err(RpmError::TempTelem)
        }
    }
}

// Helper to process error handling. Kind of temp, as masks most errors as an Option.
fn error_helper(payload: &[u16; REC_BUF_LEN], fault: &mut bool, pole_count: u8) -> Option<f32> {
    match process_rpm(payload, pole_count) {
        Ok(rpm) => Some(rpm),
        Err(e) => {
            match e {
                RpmError::TempTelem => (),
                _ => {
                    *fault = true;
                }
            }
            None
        }
    }
}

/// Update the motor RPM struct with our buffer data.
/// We delegate to a sub-function for each motor, so we can propogate motor-specific
/// statuses.
pub fn read_rpms(fault: &mut bool, pole_count: u8) -> RpmReadings {
    // todo: Don't hard-code the mapping!
    RpmReadings {
        aft_right: error_helper(&unsafe { dshot::PAYLOAD_REC_1 }, fault, pole_count),
        // aft_right: None,
        front_right: error_helper(&unsafe { dshot::PAYLOAD_REC_2 }, fault, pole_count),
        // front_right: None,
        aft_left: error_helper(&unsafe { dshot::PAYLOAD_REC_3 }, fault, pole_count),
        front_left: error_helper(&unsafe { dshot::PAYLOAD_REC_4 }, fault, pole_count),
        // front_left: None,
    }
}
