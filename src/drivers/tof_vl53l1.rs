#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(clippy::all)]
#![allow(dead_code)]
#![allow(unused)]

//! This module contains code for the ST VL53L1CB time-of-flight sensor. We use it to find AGL altitude
//! when near the ground in level flight. Advertised at up to 8M range.
//!
//! See ST UM2133 for details; note that this is separate from the datasheet.
//! See Imaging software here: https://www.st.com/en/embedded-software/stsw-img019.html
//! In particular, use the Ultra light driver: https://www.st.com/en/embedded-software/stsw-img009.html
//! the User Manual doesn't include the actual reg writes, and there are thousands or regs!
//!
//! Note: 60Hz update rate.

// Use 'Ranging mode' for AGL alt.
// Or maybe 'Lite ranging mode, intended for MCUs, to minimize post-prcessing.
// Perhaps `Multimode scanning mode` for terrain mapping or TF>

// todo: Multi-zone TOF for fwd or both TOF sensors?

use core::f32::consts::TAU;

// use cmsis_dsp_sys::{arm_cos_f32 as cos, arm_sqrt_f32}; // todo: sqrt missing?
use cmsis_dsp_sys::arm_cos_f32;
use lin_alg2::f32::{Quaternion, Vec3};
use hal::{
    i2c::{self, I2c},
    pac::I2C1,
};

pub enum TofError {
    NotConnected,
    BankThreshExceeded,
    DistThreshExceeded,
}

impl From<i2c::Error> for TofError {
    fn from(e: i2c::Error) -> Self {
        Self::NotConnected
    }
}

/// Square a number.
/// todo: Alternatively, use the `num_traits` dep
/// todo: Move to a utility mod, or sensor_fusion etc as required.
fn sq(val: f32) -> f32 {
    val * val
}

// /// API improvement wrapper for `CMSIS-DSP`'s real-number square root fn. Doesn't check for
// /// negative inputs.
// fn sqrt(val: f32) -> f32 {
//     let mut result = 0.;
//     arm_sqrt_f32(val, &mut result);
//
//     result
// }

/// Helper to wrap the unsafe, making code below more legible.
fn cos(val: f32) -> f32 {
    unsafe { arm_cos_f32(val) }
}

pub const ADDR: u8 = 0x52;

// todo: Make sure to compensate for A/C angle.

// Outside these thresholds, ignore TOF data.
const THRESH_DIST: f32 = 12.; // meters. IOC VL53L1CB specs, and extended
const THRESH_ANGLE: f32 = 0.03 * TAU; // radians, from level, in any direction.
const READING_QUAL_THRESH: f32 = 0.7;

pub fn setup(i2c: &mut I2c<I2C1>) -> Result<(), TofError> {
    // todo

    return Err(TofError::NotConnected); // todo: Until we finish work on it

    Ok(())
}

// todo: Rethink this once you learn the sensor's caps adn API
struct Reading {
    dist: f32,
    quality: f32, // Scale of 0 to 1.
}

// todo: Don't use a blocking read. Use DMA etc.
/// Read from the sensor. Result is in meters. Return `None` if the measured reading,
/// or aircraft attitude is outside the maximum range we consider acceptable,
pub fn read(attitude: Quaternion, i2c: &mut I2c<I2C1>) -> Result<f32, TofError> {
    let down = Vec3::new(0., -1., 0.);
    let down_ac = attitude.rotate_vec(down);

    // todo: Figure out how to to acos.
    // let aircraft_angle_from_down = (down.dot(down_ac)).arccos();
    let aircraft_angle_from_down = 0.; // todo temp; above should be right, but need acos.

    if aircraft_angle_from_down > THRESH_ANGLE {
        return Err(TofError::BankThreshExceeded);
    }

    let mut result = [0];
    // i2c.write_read(ADDR, &[READ_REG, 0], &mut result)?;

    let reading = result[0];

    // todo: What's the conversion from reg reads?
    let reading = Reading {
        dist: result[0] as f32,
        quality: 1.,
    };

    if reading.dist > THRESH_DIST || reading.quality < READING_QUAL_THRESH {
        return Err(TofError::DistThreshExceeded);
    }

    Ok(reading.dist * cos(aircraft_angle_from_down))
}

// todo: Consider if you want to move the ST lib to one or more separate files, or a module.

// Standalone, copy+pasted versions from `stm32-hal`. An alternative is passing `&mut i2c` to
// many of the functions here.

macro_rules! busy_wait {
    ($regs:expr, $flag:ident) => {
        loop {
            let isr = $regs.isr.read();

            if isr.$flag().bit_is_set() {
                break;
            } else if isr.berr().bit_is_set() {
                $regs.icr.write(|w| w.berrcf().set_bit());
                // return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                $regs.icr.write(|w| w.arlocf().set_bit());
                // return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                $regs.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());

                // If a pending TXIS flag is set, write dummy data to TXDR
                if $regs.isr.read().txis().bit_is_set() {
                    $regs.txdr.write(|w| w.txdata().bits(0));
                }

                // If TXDR is not flagged as empty, write 1 to flush it
                if $regs.isr.read().txe().bit_is_clear() {
                    $regs.isr.write(|w| w.txe().set_bit());
                }

                // return Err(Error::Nack);
            } else {
                // try again
            }
        }
    };
}

/// Helper function to prevent repetition between `write`, `write_read`, and `write_dma`.
fn set_cr2_write(addr: u8, len: u8, autoend: bool) {
    let regs = unsafe { &(*I2C1::ptr()) };

    // L44 RM: "Master communication initialization (address phase)
    // In order to initiate the communication, the user must program the following parameters for
    // the addressed slave in the I2C_CR2 register:
    regs.cr2.write(|w| {
        // Addressing mode (7-bit or 10-bit): ADD10
        w.sadd().bits((addr << 1) as u16);
        // Transfer direction: RD_WRN
        w.rd_wrn().clear_bit();
        w.nbytes().bits(len);
        w.autoend().bit(autoend);
        w.start().set_bit()
    });
    // Note on start bit (RM):
    // If the I2C is already in master mode with AUTOEND = 0, setting this bit generates a
    // Repeated Start condition when RELOAD=0, after the end of the NBYTES transfer.
    // Otherwise setting this bit generates a START condition once the bus is free.
    // (This is why we don't set autoend on the write portion of a write_read.)
}

/// Helper function to prevent repetition between `read`, `write_read`, and `read_dma`.
fn set_cr2_read(addr: u8, len: u8) {
    let regs = unsafe { &(*I2C1::ptr()) };

    regs.cr2.write(|w| {
        w.sadd().bits((addr << 1) as u16);
        w.rd_wrn().set_bit(); // read
        w.nbytes().bits(len);
        w.autoend().set_bit();
        w.start().set_bit()
    });
}

fn i2c_write(addr: u8, bytes: &[u8]) {
    let regs = unsafe { &(*I2C1::ptr()) };

    while regs.cr2.read().start().bit_is_set() {}

    set_cr2_write(addr, bytes.len() as u8, true);

    for byte in bytes {
        busy_wait!(regs, txis); // TXDR register is empty
        regs.txdr.write(|w| w.txdata().bits(*byte));
    }
}

fn i2c_write_read(addr: u8, bytes: &[u8], buffer: &mut [u8]) {
    let regs = unsafe { &(*I2C1::ptr()) };

    while regs.cr2.read().start().bit_is_set() {}

    set_cr2_write(addr, bytes.len() as u8, false);

    for byte in bytes {
        busy_wait!(regs, txis); // TXDR register is empty
        regs.txdr.write(|w| w.txdata().bits(*byte));
    }

    busy_wait!(regs, tc); // transfer is complete

    set_cr2_read(addr, buffer.len() as u8);

    for byte in buffer {
        // Wait until we have received something
        busy_wait!(regs, rxne);

        *byte = regs.rxdr.read().rxdata().bits();
    }
}

// From VL53L1_platform.c

// todo: These `platform` I2C read/write fns are listed as returning i8, but most
// uses expect u8.

// These i2c function is used with our translated C code to perform I2C writes.
fn VL53L1_WrByte(dev: u16, index: u16, data: u8) -> i8 {
    // todo: Make sure this and read are correct! Why 16 bit addr?
    i2c_write(dev as u8, &[(index >> 8) as u8, index as u8, data]);
    0
}

fn VL53L1_WrWord(dev: u16, index: u16, data: u16) -> i8 {
    // todo: Make sure this and read are correct! Why 16 bit addr?
    i2c_write(
        dev as u8,
        &[
            (index >> 8) as u8,
            index as u8,
            (data >> 8) as u8,
            data as u8,
        ],
    );
    0
}

fn VL53L1_WrDWord(dev: u16, index: u16, data: u32) -> i8 {
    i2c_write(
        dev as u8,
        &[
            (index >> 8) as u8,
            index as u8,
            (data >> 24) as u8,
            (data >> 16) as u8,
            (data >> 8) as u8,
            data as u8,
        ],
    );
    0
}

fn VL53L1_RdByte(dev: u16, index: u16, data: &mut u8) -> i8 {
    let mut buf = [0];
    i2c_write_read(dev as u8, &[(index >> 8) as u8, index as u8], &mut buf);

    *data = buf[0];
    0
}

fn VL53L1_RdWord(dev: u16, index: u16, data: &mut u16) -> i8 {
    let mut buf = [0, 0];
    i2c_write_read(dev as u8, &[(index >> 8) as u8, index as u8], &mut buf);

    *data = ((buf[0] as u16) << 8) | buf[1] as u16;
    0
}

fn VL53L1_RdDWord(dev: u16, index: u16, data: &mut u32) -> i8 {
    let mut buf = [0, 0, 0, 0];
    i2c_write_read(dev as u8, &[(index >> 8) as u8, index as u8], &mut buf);

    *data =
        ((buf[0] as u32) << 24) | ((buf[1] as u32) << 16) | ((buf[2] as u32) << 8) | buf[1] as u32;
    0
}

fn VL53L1_ReadMulti(dev: u16, index: u16, pdata: &mut [u8], count: u32) -> i8 {
    let mut buf = [0];
    // todo: FIgure this out.

    for i in 0..count as usize {
        i2c_write_read(dev as u8, &[(index >> 8) as u8, index as u8], &mut buf);

        pdata[i] = buf[0];
    }
    0
}

// start VL53L1X_api.h

/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file : part of VL53L1 Core and : dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document : strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/

const VL53L1X_IMPLEMENTATION_VER_MAJOR: u8 = 3;
const VL53L1X_IMPLEMENTATION_VER_MINOR: u8 = 5;
const VL53L1X_IMPLEMENTATION_VER_SUB: u8 = 1;
const VL53L1X_IMPLEMENTATION_VER_REVISION: u8 = 0000;

type VL53L1X_ERROR = i8;

const SOFT_RESET: u16 = 0x0000;
const VL53L1_I2C_SLAVE__DEVICE_ADDRESS: u16 = 0x0001;
const VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008;
const ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS: u16 = 0x0016;
const ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS: u16 = 0x0018;
const ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS: u16 = 0x001A;
const ALGO__PART_TO_PART_RANGE_OFFSET_MM: u16 = 0x001E;
const MM_CONFIG__INNER_OFFSET_MM: u16 = 0x0020;
const MM_CONFIG__OUTER_OFFSET_MM: u16 = 0x0022;
const GPIO_HV_MUX__CTRL: u16 = 0x0030;
const GPIO__TIO_HV_STATUS: u16 = 0x0031;
const SYSTEM__INTERRUPT_CONFIG_GPIO: u16 = 0x0046;
const PHASECAL_CONFIG__TIMEOUT_MACROP: u16 = 0x004B;
const RANGE_CONFIG__TIMEOUT_MACROP_A_HI: u16 = 0x005E;
const RANGE_CONFIG__VCSEL_PERIOD_A: u16 = 0x0060;
const RANGE_CONFIG__VCSEL_PERIOD_B: u16 = 0x0063;
const RANGE_CONFIG__TIMEOUT_MACROP_B_HI: u16 = 0x0061;
const RANGE_CONFIG__TIMEOUT_MACROP_B_LO: u16 = 0x0062;
const RANGE_CONFIG__SIGMA_THRESH: u16 = 0x0064;
const RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS: u16 = 0x0066;
const RANGE_CONFIG__VALID_PHASE_HIGH: u16 = 0x0069;
const VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD: u16 = 0x006C;
const SYSTEM__THRESH_HIGH: u16 = 0x0072;
const SYSTEM__THRESH_LOW: u16 = 0x0074;
const SD_CONFIG__WOI_SD0: u16 = 0x0078;
const SD_CONFIG__INITIAL_PHASE_SD0: u16 = 0x007A;
const ROI_CONFIG__USER_ROI_CENTRE_SPAD: u16 = 0x007F;
const ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE: u16 = 0x0080;
const SYSTEM__SEQUENCE_CONFIG: u16 = 0x0081;
const VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD: u16 = 0x0082;
const SYSTEM__INTERRUPT_CLEAR: u16 = 0x0086;
const SYSTEM__MODE_START: u16 = 0x0087;
const VL53L1_RESULT__RANGE_STATUS: u16 = 0x0089;
const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x008C;
const RESULT__AMBIENT_COUNT_RATE_MCPS_SD: u16 = 0x0090;
const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: u16 = 0x0096;
const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0: u16 = 0x0098;
const VL53L1_RESULT__OSC_CALIBRATE_VAL: u16 = 0x00DE;
const VL53L1_FIRMWARE__SYSTEM_STATUS: u16 = 0x00E5;
const VL53L1_IDENTIFICATION__MODEL_ID: u16 = 0x010F;
const VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD: u16 = 0x013E;

/****************************************
 * PRIVATE define do not edit
 ****************************************/

/**
 *  @brief defines SW Version
 */
struct VL53L1X_Version_t {
    /// major number
    major: u8,
    /// minor number
    minor: u8,
    /// build number
    build: u8,
    /// revision number
    revision: u32,
}

/// Defines packed reading results type
struct VL53L1X_Result_t {
    ///ResultStatus
    Status: u8,
    ///ResultDistance
    Distance: u16,
    ///ResultAmbient
    Ambient: u16,
    ///ResultSignalPerSPAD
    SigPerSPAD: u16,
    ///ResultNumSPADs
    NumSPADs: u16,
}

// start VL53L1X_API.c]

// todo: Apply doc strings from the header.

static mut VL51L1X_NVM_CONFIGURATION: [u8; 34] = [
    0x00, /* 0x00 : not user-modifiable */
    0x29, /* 0x01 : 7 bits I2C address (default=0x29), use SetI2CAddress(). Warning: after changing the register value to a new I2C address, the device will only answer to the new address */
    0x00, /* 0x02 : not user-modifiable */
    0x00, /* 0x03 : not user-modifiable */
    0x00, /* 0x04 : not user-modifiable */
    0x00, /* 0x05 : not user-modifiable */
    0x00, /* 0x06 : not user-modifiable */
    0x00, /* 0x07 : not user-modifiable */
    0x00, /* 0x08 : not user-modifiable */
    0x50, /* 0x09 : not user-modifiable */
    0x00, /* 0x0A : not user-modifiable */
    0x00, /* 0x0B : not user-modifiable */
    0x00, /* 0x0C : not user-modifiable */
    0x00, /* 0x0D : not user-modifiable */
    0x0a, /* 0x0E : not user-modifiable */
    0x00, /* 0x0F : not user-modifiable */
    0x00, /* 0x10 : not user-modifiable */
    0x00, /* 0x11 : not user-modifiable */
    0x00, /* 0x12 : not user-modifiable */
    0x00, /* 0x13 : not user-modifiable */
    0x00, /* 0x14 : not user-modifiable */
    0x00, /* 0x15 : not user-modifiable */
    0x00, /* 0x16 : Xtalk calibration value MSB (7.9 format in kcps), use SetXtalk() */
    0x00, /* 0x17 : Xtalk calibration value LSB */
    0x00, /* 0x18 : not user-modifiable */
    0x00, /* 0x19 : not user-modifiable */
    0x00, /* 0x1a : not user-modifiable */
    0x00, /* 0x1b : not user-modifiable */
    0x00, /* 0x1e : Part to Part offset x4 MSB (in mm), use SetOffset() */
    0x50, /* 0x1f : Part to Part offset x4 LSB */
    0x00, /* 0x20 : not user-modifiable */
    0x00, /* 0x21 : not user-modifiable */
    0x00, /* 0x22 : not user-modifiable */
    0x00, /* 0x23 : not user-modifiable */
];

const VL51L1X_DEFAULT_CONFIGURATION: [u8; 91] = [
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00, /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
];

const status_rtn: [u8; 24] = [
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11,
    12,
];

/// This function returns the SW driver version
fn VL53L1X_GetSWVersion(pVersion: &mut VL53L1X_Version_t) -> VL53L1X_ERROR {
    let Status = 0;

    pVersion.major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
    pVersion.minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
    pVersion.build = VL53L1X_IMPLEMENTATION_VER_SUB;
    pVersion.revision = VL53L1X_IMPLEMENTATION_VER_REVISION as u32;
    return Status;
}

/// This function sets the sensor I2C address used in case multiple devices application, default address 0x52
fn VL53L1X_SetI2CAddress(dev: u16, new_address: u8) -> VL53L1X_ERROR {
    let mut status = 0;

    status |= VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
    return status;
}

///  This function loads the 135 bytes default values to initialize the sensor.
///  * @param dev Device address
///  * @return 0:success, != 0:failed
fn VL53L1X_SensorInit(dev: u16) -> VL53L1X_ERROR {
    let mut status = 0;
    // let Addr = 0x00;
    let mut tmp;

    for Addr in 0x2D..=0x87 {
        // for (Addr = 0x2D; Addr <= 0x87; Addr++){
        status |= VL53L1_WrByte(
            dev,
            Addr,
            VL51L1X_DEFAULT_CONFIGURATION[Addr as usize - 0x2D],
        );
    }
    status |= VL53L1X_StartRanging(dev);
    tmp = 0;
    while tmp == 0 {
        status |= VL53L1X_CheckForDataReady(dev, &mut tmp);
    }
    status |= VL53L1X_ClearInterrupt(dev);
    status |= VL53L1X_StopRanging(dev);
    status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
    status |= VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
    return status;
}

///  This function clears the interrupt, to be called after a ranging data reading
///  * to arm the interrupt for the next data ready event.
fn VL53L1X_ClearInterrupt(dev: u16) -> VL53L1X_ERROR {
    let mut status = 0;

    status |= VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
    return status;
}

///  This function programs the interrupt polarity\n
///  * 1=active high (default), 0=active low
fn VL53L1X_SetInterruptPolarity(dev: u16, NewPolarity: u8) -> VL53L1X_ERROR {
    let mut Temp = 0;
    let mut status = 0;

    status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &mut Temp);
    Temp = Temp & 0xEF;
    status |= VL53L1_WrByte(dev, GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
    return status;
}

/// /**
///  This function returns the current interrupt polarity\n
///  * 1=active high (default), 0=active low
fn VL53L1X_GetInterruptPolarity(dev: u16, pInterruptPolarity: &mut u8) -> VL53L1X_ERROR {
    let mut Temp = 0;
    let mut status = 0;

    status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &mut Temp);
    Temp = Temp & 0x10;
    *pInterruptPolarity = !(Temp >> 4);
    return status;
}

///  This function starts the ranging distance operation\n
///  * The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready\n
///  * 1=active high (default), 0=active low, use SetInterruptPolarity() to change the interrupt polarity if required.
fn VL53L1X_StartRanging(dev: u16) -> VL53L1X_ERROR {
    let mut status = 0;

    status |= VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40); /* Enable VL53L1X */
    return status;
}

/// This function stops the ranging.
fn VL53L1X_StopRanging(dev: u16) -> VL53L1X_ERROR {
    let mut status = 0;

    status |= VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x00); /* Disable VL53L1X */

    return status;
}

///  This function checks if the new ranging data is available by polling the dedicated register.
///  * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
fn VL53L1X_CheckForDataReady(dev: u16, isDataReady: &mut u8) -> VL53L1X_ERROR {
    let mut Temp = 0;
    let mut IntPol = 0;
    let mut status = 0;

    status |= VL53L1X_GetInterruptPolarity(dev, &mut IntPol);
    status |= VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &mut Temp);
    /* Read in the register to check if a new value is available */
    if status == 0 {
        if (Temp & 1) == IntPol {
            *isDataReady = 1;
        } else {
            *isDataReady = 0;
        }
    }
    return status;
}

///  This function programs the timing budget in ms.
///  * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
fn VL53L1X_SetTimingBudgetInMs(dev: u16, TimingBudgetInMs: u16) -> VL53L1X_ERROR {
    let mut DM = 0;
    let mut status = 0;

    status |= VL53L1X_GetDistanceMode(dev, &mut DM);
    if DM == 0 {
        return 1;
    } else if DM == 1 {
        /* Short DistanceMode */
        match TimingBudgetInMs {
            15 => {
                /* only available in short distance mode */
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
            }
            20 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            }
            33 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            }
            50 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
            }
            100 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
            }
            200 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
            }
            500 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
            }
            _ => {
                status = 1;
            }
        }
    } else {
        match TimingBudgetInMs {
            20 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
            }
            33 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            }
            50 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
            }
            100 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
            }
            200 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
            }
            500 => {
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
                VL53L1_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
            }
            _ => {
                status = 1;
            }
        }
    }
    return status;
}

/// This function returns the current timing budget in ms.
fn VL53L1X_GetTimingBudgetInMs(dev: u16, pTimingBudget: &mut u16) -> VL53L1X_ERROR {
    let mut Temp = 0;
    let mut status = 0;

    status |= VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &mut Temp);

    // todo: SHould these |s be range (..) ??
    match Temp {
        0x001D => {
            *pTimingBudget = 15;
        }
        0x0051 | 0x001E => {
            *pTimingBudget = 20;
        }
        0x00D6 | 0x0060 => {
            *pTimingBudget = 33;
        }
        0x1AE | 0x00AD => {
            *pTimingBudget = 50;
        }
        0x02E1 | 0x01CC => {
            *pTimingBudget = 100;
        }
        0x03E1 | 0x02D9 => {
            *pTimingBudget = 200;
        }
        0x0591 | 0x048F => {
            *pTimingBudget = 500;
        }
        _ => {
            status = 1;
            *pTimingBudget = 0;
        }
    }
    return status;
}

///  This function programs the distance mode (1=short, 2=long(default)).
///  * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
///  * Long mode can range up to 4 m in the dark with 200 ms timing budget.
fn VL53L1X_SetDistanceMode(dev: u16, DM: u16) -> VL53L1X_ERROR {
    let mut TB = 0;
    let mut status = 0;

    status |= VL53L1X_GetTimingBudgetInMs(dev, &mut TB);
    if status != 0 {
        return 1;
    }
    match DM {
        1 => {
            status = VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
            status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
            status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
            status = VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
            status = VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0705);
            status = VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
        }
        2 => {
            status = VL53L1_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
            status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
            status = VL53L1_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
            status = VL53L1_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
            status = VL53L1_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0F0D);
            status = VL53L1_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
        }
        _ => {
            status = 1;
        }
    }

    if status == 0 {
        status |= VL53L1X_SetTimingBudgetInMs(dev, TB);
    }
    return status;
}

/// This function returns the current distance mode (1=short, 2=long).
fn VL53L1X_GetDistanceMode(dev: u16, DM: &mut u16) -> VL53L1X_ERROR {
    let mut TempDM = 0;
    let mut status = 0;

    status |= VL53L1_RdByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, &mut TempDM);
    if TempDM == 0x14 {
        *DM = 1;
    }
    if TempDM == 0x0A {
        *DM = 2;
    }
    return status;
}

///  This function programs the Intermeasurement period in ms\n
///  * Intermeasurement period must be >/= timing budget. This condition is not checked by the API,
///  * the customer has the duty to check the condition. Default = 100 ms
fn VL53L1X_SetInterMeasurementInMs(dev: u16, InterMeasMs: u32) -> VL53L1X_ERROR {
    let mut ClockPLL = 0;
    let mut status = 0;

    status |= VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &mut ClockPLL);
    ClockPLL = ClockPLL & 0x3FF;
    VL53L1_WrDWord(
        dev,
        VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
        (ClockPLL as f32 * InterMeasMs as f32 * 1.075) as u32,
    );
    return status;
}

/// This function returns the Intermeasurement period in ms.
fn VL53L1X_GetInterMeasurementInMs(dev: u16, pIM: &mut u16) -> VL53L1X_ERROR {
    let mut ClockPLL = 0;
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &mut tmp);
    *pIM = tmp as u16;
    status |= VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &mut ClockPLL);
    ClockPLL = ClockPLL & 0x3FF;
    *pIM = (*pIM as f32 / (ClockPLL as f32 * 1.065)) as u16;
    return status;
}

/// This function returns the boot state of the device (1:booted, 0:not booted)
fn VL53L1X_BootState(dev: u16, state: &mut u8) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &mut tmp);
    *state = tmp;
    return status;
}

/// This function returns the sensor id, sensor Id must be 0xEEAC
fn VL53L1X_GetSensorId(dev: u16, sensorId: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &mut tmp);
    *sensorId = tmp;
    return status;
}

/// This function returns the distance measured by the sensor in mm
fn VL53L1X_GetDistance(dev: u16, distance: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= (VL53L1_RdWord(
        dev,
        VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
        &mut tmp,
    ));
    *distance = tmp;
    return status;
}

///  This function returns the returned signal per SPAD in kcps/SPAD.
///  * With kcps stands for Kilo Count Per Second
fn VL53L1X_GetSignalPerSpad(dev: u16, signalRate: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut SpNb = 1;
    let mut signal = 0;

    status |= VL53L1_RdWord(
        dev,
        VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0,
        &mut signal,
    );
    status |= VL53L1_RdWord(
        dev,
        VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0,
        &mut SpNb,
    );
    *signalRate = (200.0 * signal as f32 / SpNb as f32) as u16;
    return status;
}

/// This function returns the ambient per SPAD in kcps/SPAD
fn VL53L1X_GetAmbientPerSpad(dev: u16, ambPerSp: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut AmbientRate = 0;
    let mut SpNb = 1;

    status |= VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &mut AmbientRate);
    status |= VL53L1_RdWord(
        dev,
        VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0,
        &mut SpNb,
    );
    *ambPerSp = (200.0 * AmbientRate as f32 / SpNb as f32) as u16;
    return status;
}

/// This function returns the returned signal in kcps.
fn VL53L1X_GetSignalRate(dev: u16, signal: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(
        dev,
        VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0,
        &mut tmp,
    );
    *signal = tmp * 8;
    return status;
}

/// This function returns the current number of enabled SPADs
fn VL53L1X_GetSpadNb(dev: u16, spNb: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &mut tmp);
    *spNb = tmp >> 8;
    return status;
}

/// This function returns the ambient rate in kcps
fn VL53L1X_GetAmbientRate(dev: u16, ambRate: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &mut tmp);
    *ambRate = tmp * 8;
    return status;
}

///  This function returns the ranging status error \n
//  * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
fn VL53L1X_GetRangeStatus(dev: u16, rangeStatus: &mut u8) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut RgSt = 0;

    *rangeStatus = 255;
    status |= VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &mut RgSt);
    RgSt = RgSt & 0x1F;
    if RgSt < 24 {
        *rangeStatus = status_rtn[RgSt as usize];
    }
    return status;
}

/// This function returns measurements and the range status in a single read access
fn VL53L1X_GetResult(dev: u16, pResult: &mut VL53L1X_Result_t) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut Temp = [0_u8; 17];
    let mut RgSt = 255;

    status |= VL53L1_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, &mut Temp, 17);
    RgSt = Temp[0] & 0x1F;
    if RgSt < 24 {
        RgSt = status_rtn[RgSt as usize];
    }
    pResult.Status = RgSt;
    pResult.Ambient = ((Temp[7] as u16) << 8 | Temp[8] as u16) * 8;
    pResult.NumSPADs = Temp[3] as u16;
    pResult.SigPerSPAD = ((Temp[15] as u16) << 8 | Temp[16] as u16) * 8;
    pResult.Distance = (Temp[13] as u16) << 8 | Temp[14] as u16;

    return status;
}

///  This function programs the offset correction in mm
///  * @param OffsetValue:the offset correction value to program in mm
fn VL53L1X_SetOffset(dev: u16, OffsetValue: i16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut Temp = 0;

    Temp = (OffsetValue * 4);
    status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, Temp as u16);
    status |= VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
    status |= VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
    return status;
}

/// This function returns the programmed offset correction value in mm
fn VL53L1X_GetOffset(dev: u16, offset: &mut i16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut Temp = 0;

    status |= VL53L1_RdWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, &mut Temp);
    Temp = Temp << 3;
    Temp = Temp >> 5;
    *offset = Temp as i16;
    return status;
}

///  This function programs the xtalk correction value in cps (Count Per Second).\n
///  * This is the number of photons reflected back from the cover glass in cps.
fn VL53L1X_SetXtalk(dev: u16, XtalkValue: u16) -> VL53L1X_ERROR {
    /* XTalkValue in count per second to avoid float type */
    let mut status = 0;

    status |= VL53L1_WrWord(
        dev,
        ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
        0x0000,
    );
    status |= VL53L1_WrWord(
        dev,
        ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
        0x0000,
    );
    status |= VL53L1_WrWord(
        dev,
        ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
        (XtalkValue << 9) / 1000,
    ); /* * << 9 (7.9 format) and /1000 to convert cps to kpcs */
    return status;
}

/// This function returns the current programmed xtalk correction value in cps
fn VL53L1X_GetXtalk(dev: u16, xtalk: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;

    status |= VL53L1_RdWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, xtalk);
    *xtalk = ((*xtalk * 1000) >> 9) as u16; /* * 1000 to convert kcps to cps and >> 9 (7.9 format) */
    return status;
}

/// /**
///  This function programs the threshold detection mode\n
///  * Example:\n
///  * VL53L1X_SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
///  * VL53L1X_SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
///  * VL53L1X_SetDistanceThreshold(dev,100,300,2,1): Out of window \n
///  * VL53L1X_SetDistanceThreshold(dev,100,300,3,1): In window \n
///  * @param   dev : device address
///  * @param  	ThreshLow(in mm) : the threshold under which one the device raises an interrupt if Window = 0
///  * @param 	ThreshHigh(in mm) :  the threshold above which one the device raises an interrupt if Window = 1
///  * @param   Window detection mode : 0=below, 1=above, 2=out, 3=in
///  * @param   IntOnNo
fn VL53L1X_SetDistanceThreshold(
    dev: u16,
    ThreshLow: u16,
    ThreshHigh: u16,
    Window: u8,
    IntOnNoTarget: u8,
) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut Temp = 0;

    status |= VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &mut Temp);
    Temp = Temp & 0x47;
    if IntOnNoTarget == 0 {
        status = VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, (Temp | (Window & 0x07)));
    } else {
        status = VL53L1_WrByte(
            dev,
            SYSTEM__INTERRUPT_CONFIG_GPIO,
            ((Temp | (Window & 0x07)) | 0x40),
        );
    }
    status |= VL53L1_WrWord(dev, SYSTEM__THRESH_HIGH, ThreshHigh);
    status |= VL53L1_WrWord(dev, SYSTEM__THRESH_LOW, ThreshLow);
    return status;
}

///  This function returns the window detection mode (0=below; 1=above; 2=out; 3=in)
fn VL53L1X_GetDistanceThresholdWindow(dev: u16, window: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;
    status |= VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &mut tmp);
    *window = (tmp as u16 & 0x7) as u16;
    return status;
}

/// This function returns the low threshold in mm
fn VL53L1X_GetDistanceThresholdLow(dev: u16, low: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, SYSTEM__THRESH_LOW, &mut tmp);
    *low = tmp;
    return status;
}

/// This function returns the high threshold in mm
fn VL53L1X_GetDistanceThresholdHigh(dev: u16, high: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, SYSTEM__THRESH_HIGH, &mut tmp);
    *high = tmp;
    return status;
}

///  This function programs the ROI (Region of Interest)\n
///  * The ROI position is centered, only the ROI size can be reprogrammed.\n
///  * The smallest acceptable ROI size = 4\n
///  * @param X:ROI Width; Y=ROI Height
fn VL53L1X_SetROICenter(dev: u16, ROICenter: u8) -> VL53L1X_ERROR {
    let mut status = 0;
    status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
    return status;
}

///  This function returns the current user ROI center
fn VL53L1X_GetROICenter(dev: u16, ROICenter: &mut u8) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;
    status |= VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, &mut tmp);
    *ROICenter = tmp;
    return status;
}

///  This function programs the ROI (Region of Interest)\n
///  * The ROI position is centered, only the ROI size can be reprogrammed.\n
///  * The smallest acceptable ROI size = 4\n
///  * @param X:ROI Width; Y=ROI Height
///  */
fn VL53L1X_SetROI(dev: u16, mut X: u16, mut Y: u16) -> VL53L1X_ERROR {
    let mut OpticalCenter = 0;
    let mut status = 0;

    status |= VL53L1_RdByte(
        dev,
        VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD,
        &mut OpticalCenter,
    );
    if X > 16 {
        X = 16;
    }
    if Y > 16 {
        Y = 16;
    }
    if X > 10 || Y > 10 {
        OpticalCenter = 199;
    }
    status |= VL53L1_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
    status |= VL53L1_WrByte(
        dev,
        ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
        ((Y - 1) << 4 | (X - 1)) as u8,
    );
    return status;
}

///  This function returns width X and height Y
fn VL53L1X_GetROI_XY(dev: u16, ROI_X: &mut u16, ROI_Y: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status = VL53L1_RdByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &mut tmp);
    *ROI_X |= (tmp as u16 & 0x0F) + 1;
    *ROI_Y |= ((tmp as u16 & 0xF0) >> 4) + 1;
    return status;
}

/// This function programs a new signal threshold in kcps (default=1024 kcps)
fn VL53L1X_SetSignalThreshold(dev: u16, Signal: u16) -> VL53L1X_ERROR {
    let mut status = 0;

    status |= VL53L1_WrWord(
        dev,
        RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
        Signal >> 3,
    );
    return status;
}

/// This function returns the current signal threshold in kcps
fn VL53L1X_GetSignalThreshold(dev: u16, signal: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &mut tmp);
    *signal = tmp << 3;
    status
}

/// This function programs a new sigma threshold in mm (default=15 mm)
fn VL53L1X_SetSigmaThreshold(dev: u16, Sigma: u16) -> VL53L1X_ERROR {
    let mut status = 0;

    if (Sigma > (0xFFFF >> 2)) {
        return 1;
    }
    /* 16 bits register 14.2 format */
    status |= VL53L1_WrWord(dev, RANGE_CONFIG__SIGMA_THRESH, Sigma << 2);
    status
}

/// This function returns the current sigma threshold in mm
fn VL53L1X_GetSigmaThreshold(dev: u16, sigma: &mut u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_RdWord(dev, RANGE_CONFIG__SIGMA_THRESH, &mut tmp);
    *sigma = tmp >> 2;
    status
}

///  This function performs the temperature calibration.
///  * It is recommended to call this function any time the temperature might have changed by more than 8 deg C
///  * without sensor ranging activity for an extended period.
fn VL53L1X_StartTemperatureUpdate(dev: u16) -> VL53L1X_ERROR {
    let mut status = 0;
    let mut tmp = 0;

    status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81); /* full VHV */
    status |= VL53L1_WrByte(dev, 0x0B, 0x92);
    status |= VL53L1X_StartRanging(dev);
    while tmp == 0 {
        status |= VL53L1X_CheckForDataReady(dev, &mut tmp);
    }
    tmp = 0;
    status |= VL53L1X_ClearInterrupt(dev);
    status |= VL53L1X_StopRanging(dev);
    status |= VL53L1_WrByte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
    status |= VL53L1_WrByte(dev, 0x0B, 0); /* start VHV from the previous temperature */
    status
}

// Start of VL53L1X_calibration.c

/**
This function performs the offset calibration.\n
* The function returns the offset value found and programs the offset compensation into the device.
* @param TargetDistInMm target distance in mm, ST recommended 100 mm
* Target reflectance = grey17%
* @return 0:success, !=0: failed
* @return offset pointer contains the offset found in mm
*/
fn VL53L1X_CalibrateOffset(dev: u16, TargetDistInMm: u16, offset: &mut i16) -> i8 {
    let mut i = 0;
    let mut tmp = 0;
    let mut AverageDistance = 0;
    let mut distance = 0;
    let mut status: i8 = 0;

    status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
    status |= VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
    status |= VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
    status |= VL53L1X_StartRanging(dev); /* Enable VL53L1X sensor */

    for i in 0..50 {
        tmp = 0;
        while tmp == 0 {
            status |= VL53L1X_CheckForDataReady(dev, &mut tmp);
        }
        status |= VL53L1X_GetDistance(dev, &mut distance);
        status |= VL53L1X_ClearInterrupt(dev);
        AverageDistance += distance;
    }
    status |= VL53L1X_StopRanging(dev);
    AverageDistance /= 50;
    *offset = TargetDistInMm as i16 - AverageDistance as i16;
    status |= VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset as u16 * 4);
    status
}

/**
This function performs the xtalk calibration.\n
* The function returns the xtalk value found and programs the xtalk compensation to the device
* @param TargetDistInMm target distance in mm\n
* The target distance : the distance where the sensor start to "under range"\n
* due to the influence of the photons reflected back from the cover glass becoming strong\n
* It's also called inflection point\n
* Target reflectance = grey 17%
* @return 0: success, !=0: failed
* @return xtalk pointer contains the xtalk value found in cps (number of photons in count per second)
*/
fn VL53L1X_CalibrateXtalk(dev: u16, TargetDistInMm: u16, xtalk: &mut u16) -> i8 {
    let mut tmp = 0;
    let mut AverageSignalRate = 0;
    let mut AverageDistance = 0;
    let mut AverageSpadNb = 0;
    let mut distance = 0;
    let mut spadNum = 0;
    let mut sr = 0;
    let mut status = 0;
    let mut calXtalk = 0;

    status |= VL53L1_WrWord(dev, 0x0016, 0);
    status |= VL53L1X_StartRanging(dev);
    for _ in 0..50 {
        // for (i = 0; i < 50; i++) {
        tmp = 0;
        while tmp == 0 {
            status |= VL53L1X_CheckForDataReady(dev, &mut tmp);
        }
        status |= VL53L1X_GetSignalRate(dev, &mut sr);
        status |= VL53L1X_GetDistance(dev, &mut distance);
        status |= VL53L1X_ClearInterrupt(dev);
        AverageDistance += distance;
        status = VL53L1X_GetSpadNb(dev, &mut spadNum);
        AverageSpadNb += spadNum;
        AverageSignalRate = AverageSignalRate + sr;
    }
    status |= VL53L1X_StopRanging(dev);
    AverageDistance = AverageDistance / 50;
    AverageSpadNb = AverageSpadNb / 50;
    AverageSignalRate = AverageSignalRate / 50;
    /* Calculate Xtalk value */
    calXtalk = (512 * (AverageSignalRate * (1 - (AverageDistance / TargetDistInMm)))
        / AverageSpadNb) as u16;

    *xtalk = ((calXtalk * 1000) >> 9) as u16;
    status |= VL53L1_WrWord(dev, 0x0016, calXtalk as u16);
    return status as i8;
}
