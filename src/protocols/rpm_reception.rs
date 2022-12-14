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

/// Covnert our arrays of high and low edge counts to a 20-bit integer.  u32 since it's 20 bits.
/// `counts` alternates low and high edges; counts[0] is low.
pub fn edge_counts_to_u32(counts: &[u16]) -> u32 {
    // On G4, with DSHOT 300: Pulse width is ~430 ticks.

    let mut bits = [false; 20];

    let mut bits_i = 0;
    // Iterate over edges. What the max number of edge? Around 20?
    for i in 1..20 {
        // We'll have fewer edges for sequences of 1s or 0s in a row, so
        // we will likely have fewer than 20 edges; abort when we hit the end
        // the used part of our buffer; where the 0 starts.
        if counts[i] == 0 {
            // We build bits based on their closing edge; for the last bit, we
            // may not have one, so set up remaining bits with the last edge.
            if bits_i <= 19 {
                for j in bits_i..20 {
                    bits[j as usize] = i % 2 == 0;
                }
            }
            break;
        }

        let bits_since_last_edge = (counts[i] - counts[i - 1]) as f32 / BIT_LEN as f32;

        let mut bits_since_last_edge = bits_since_last_edge.round() as u16;

        // We discard the first bit; it's a low bit to indicate the start of the sequence.
        if i == 1 {
            bits_since_last_edge -= 1;
        }

        for j in bits_i..bits_i + bits_since_last_edge {
            if j > 19 {
                // todo: Kludge to prevent overflows.
                // tood: When does this come up?
                break;
            }
            bits[j as usize] = i % 2 == 0
        }

        bits_i += bits_since_last_edge;
    }

    // println!("B: {}", bits);

    // Assemble the result from bits.
    // println!("bits: {}", bits);
    let mut result = 0;
    for (i, v) in bits.iter().enumerate() {
        result |= (*v as u32) << (REC_BUF_LEN - i)
    }

    result
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
pub fn update_rpms(rpms: &mut MotorRpm, fault: &mut bool) {
    // pub fn update_rpms(rpms: &mut MotorRpm, mapping: &ControlMapping, fault: &mut bool) {

    // Convert our arrays of high and low timings to a 20-bit integer.
    let gcr1 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_1_HIGH) };
    let gcr2 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_2_HIGH) };
    let gcr3 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_3_HIGH) };
    let gcr4 = unsafe { edge_counts_to_u32(&dshot::PAYLOAD_REC_4_HIGH) };
    // todo temp
    // let gcr2 = 0;
    // let gcr3 = 0;
    // let gcr4 = 0;

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
}
