//! This module contains code for interpreting (bidirectional) DSHOT RPM readings.
//! Management of the timers, motor lines, DMA reception etc is handled in the `dshot` module,
//! and in ISRs in `main`. This module handles interpretation of the buffers collected
//! by those processes.

use super::dshot::{self, calc_crc, DSHOT_SPEED, REC_BUF_LEN, TIM_CLK};

use num_traits::float::FloatCore; // round

use crate::{
    flight_ctrls::{
        common::{Motor, MotorRpm},
        ControlMapping,
    },
    setup::MotorTimer,
};

use defmt::println;

// Number of counter ticks per bit.
// The differences tend to come out a bit lower, b ut this is the number I've calced.
// This corresponds to a period of 5/4 * the DSHOT freq, per its spec.
const BIT_LEN: u16 = (TIM_CLK / (5 * DSHOT_SPEED / 4) - 1) as u16;

const GCR_LEN: usize = 20;

#[derive(Clone, Copy)]
enum EscData {
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

/// Return RPM in revolutions-per-second
/// See https://brushlesswhoop.com/dshot-and-bidirectional-dshot/, "eRPM Telemetry Frame (from ESC)".
fn rpm_from_data(packet: u16) -> Result<EscData, RpmError> {
    let crc_read = packet & 0b1111;
    let data = packet >> 4;

    // Right shift 4 to exclude the CRC itself from the calculation.
    if crc_read != calc_crc(data) {
        println!("C");
        // println!("CRC: {}, p: {}", unsafe { &PAYLOAD_REC_BB_3 }, packet);
        return Err(RpmError::Crc);
    }
    // println!("G");

    // todo: Come back to telemetry later.
    // Parse extended telemetry if avail. (This may be required to avoid misreading the data?)
    if ((data >> 8) & 1) == 0 {
        let telem_type_val = packet >> 8;
        // println!("T");
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
        let base = packet & 0b1_1111_1111;
        let period_us = base << shift;

        // Period is in us. Convert to Radians-per-second using motor pole count.
        // todo: Pole count in user cfg.

        let num_poles = 1.; // todo placeholder

        Ok(EscData::Rpm(1_000_000. / (period_us as f32 * num_poles)))
    }
}

pub enum RpmError {
    Gcr,
    Crc,
    TelemType,
}

/// Convert our arrays of high and low edge counts to a 20-bit integer.  u32 since it's 20 bits.
/// `counts` alternates low and high edges; counts[0] is low.
pub fn edge_counts_to_u32(counts: &[u16]) -> Result<u32, RpmError> {
    // Start at index 1 of edges; we compare to i-1.
    let mut edge_i = 1;

    // Assemble bit lengths of each (high or low) value from edge timings.
    let mut value_lens = [0; 20]; // Generally smaller than this.

    for _ in 1..25 {
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
        }

        if bits_since_last_edge == 0 {
            // Continue without incrementing `edge_i`, or modifying our counts.
            continue;
        }

        value_lens[edge_i - 1] = bits_since_last_edge;
        edge_i += 1;
    }
    // println!("E: {} {} {} {} {}", value_lens[0], value_lens[1], value_lens[2], value_lens[3], value_lens[4]);

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

    // println!("{}", result);
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
            // println!("M {}", val);
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

    Ok(reduce_bit_count_map(nibble_0)?
        | (reduce_bit_count_map(nibble_1)? << 4)
        | (reduce_bit_count_map(nibble_2)? << 8)
        | (reduce_bit_count_map(nibble_3)? << 12))

    //
    //
    // let mut mapped = 0;
    // let mut left_shift = 0;
    //
    // // todo: I think the aboev and below code is equiv
    //
    // // for(int i = 0; i < 20; i += 5) {
    // for i in 0..4 {
    //     let v = ((val >> (i * 5)) & 0x1F) as u8;
    //     let new_value = reduce_bit_count_map(v)?;
    //     mapped |= new_value << left_shift;
    //     left_shift += 4;
    // }
    //
    // Ok(mapped)
}

/// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/: `Decoding eRPM frame`
fn gcr_step_1(val: u32) -> u32 {
    val ^ (val >> 1)
}

/// Helper fn
fn update_rpm_from_packet(rpm: &mut f32, packet: Result<u16, RpmError>) -> Result<(), RpmError> {
    match rpm_from_data(packet?)? {
        EscData::Rpm(rpm_) => {
            *rpm = rpm_;
        }
        EscData::Telem(_, _) => {
            // todo
        }
    }

    Ok(())
}

/// Update the motor RPM struct with our buffer data.
pub fn update_rpms(rpms: &mut MotorRpm, fault: &mut bool) -> Result<(), RpmError> {
    // pub fn update_rpms(rpms: &mut MotorRpm, mapping: &ControlMapping, fault: &mut bool) {

    // Convert our arrays of high and low timings to a 20-bit integer.
    // let gcr1 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_1)? };
    // let gcr2 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_2)? };
    let gcr3 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_3)? };
    // let gcr4 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_4)? };
    // todo temp
    let gcr1 = 0;
    let gcr2 = 0;
    // let gcr3 = 0;
    let gcr4 = 0;

    // Perform some initial de-obfuscation using a bit shift and xor
    let gcr1 = gcr_step_1(gcr1);
    let gcr2 = gcr_step_1(gcr2);
    let gcr3 = gcr_step_1(gcr3);
    let gcr4 = gcr_step_1(gcr4);

    // Convert our 20-bit raw GCR data to the 16-bit data packet.
    let packet1 = reduce_bit_count(gcr1);
    let packet2 = reduce_bit_count(gcr2);
    let packet3 = reduce_bit_count(gcr3);
    let packet4 = reduce_bit_count(gcr4);

    // todo: Don't hard code the motor mapping!!

    if update_rpm_from_packet(&mut rpms.aft_right, packet1).is_err() {
        *fault = true;
    }
    if update_rpm_from_packet(&mut rpms.front_right, packet2).is_err() {
        *fault = true;
    }
    if update_rpm_from_packet(&mut rpms.aft_left, packet3).is_err() {
        *fault = true;
    } else {
    }
    if update_rpm_from_packet(&mut rpms.front_left, packet4).is_err() {
        *fault = true;
    }

    Ok(())
}
