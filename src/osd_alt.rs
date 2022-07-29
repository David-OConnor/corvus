//! This module contains code for interfacing with DJI's OSD system, via UART.
//! It uses the MSP protocol.
//!
//! https://github.com/bri3d/msp-osd
//! https://github.com/chris1seto/PX4-Autopilot/tree/turbotimber/src/modules/msp_osd

// MSP requires a free serial port, and its speed defaults to 115200 baud.
// SERIAL2_PROTOCOL = 33

use stm32_hal2::{pac::USART2, usart::Usart};

use crate::{lin_alg, protocols::msp, util};

use defmt::println;

use num_traits::float::Float;

// todo: Maybe put in a struct etc? It's constant, but we use a function call to populate it.
// Note: LUT is here, since it depends on the poly.
static mut CRC_LUT: [u8; 256] = [0; 256];
const CRC_POLY: u8 = 0xd;

/// From the `metaverse` project.
#[rustfmt::skip]
pub fn new_perspective_rh(fov_y: f32, aspect_ratio: f32, near: f32, far: f32) -> lin_alg::Mat4 {
    let f = 1. / (fov_y / 2.).tan();
    let range_inv = 1. / (near - far);

    // todo: Still needs work and QC!!
    lin_alg::Mat4 {
        data: [
            f / aspect_ratio, 0., 0., 0.,
            0., f, 0., 0.,
            // 0., 0., (near + far) * range_inv, -1.,
            0., 0., far * range_inv, -1.,
            // 0., 0., (2. * far * near) * range_inv, 0.
            0., 0., (far * near) * range_inv, 0.
        ]
    }
}

/// From the `metaverse` project.
#[rustfmt::skip]
pub fn new_rotation(val: Vec3) -> lin_alg::Mat4 {
    let (sin_x, cos_x) = val.x.sin_cos();
    let (sin_y, cos_y) = val.y.sin_cos();
    let (sin_z, cos_z) = val.z.sin_cos();

    let rot_x = lin_alg::Mat4 {
        data: [
            1., 0., 0., 0.,
            0., cos_x, sin_x, 0.,
            0., -sin_x, cos_x, 0.,
            0., 0., 0., 1.
        ]
    };

    let rot_y = lin_alg::Mat4 {
        data: [
            cos_y, 0., -sin_y, 0.,
            0., 1., 0., 0.,
            sin_y, 0., cos_y, 0.,
            0., 0., 0., 1.
        ]
    };

    let rot_z = lin_alg::Mat4 {
        data: [
            cos_z, sin_z, 0., 0.,
            -sin_z, cos_z, 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.
        ]
    };

    // todo: What order to apply these three ?
    // todo: TO avoid gimbal lock, consider rotating aroudn an arbitrary unit axis immediately.

    rot_x * rot_y * rot_z
}
