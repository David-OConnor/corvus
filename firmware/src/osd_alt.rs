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

fn draw(uart: &mut Usart<USART2>) {
    let packet = msp::Packet {
        message_type: msp::MessageType::Request,
        function: 0,     // todo
        payload_size: 0, // todo
    };
}

/// Initial config for the OSD
pub fn setup(uart: &mut Usart<USART2>) {
    // todo: DMA?
    util::crc_init(unsafe { &mut CRC_LUT }, CRC_POLY);
}

/// Draw the current airspeed
pub fn draw_airspeed() {}

/// Draw the current Mean Sea Level altitude
pub fn draw_alt_msl() {}

/// Draw the current Above Ground Level altitude
pub fn draw_alt_agl() {}

/// Draw the heading indicator
pub fn draw_heading() {}

/// Draw sideslip. (Horizontal motion relative to flight path)
pub fn draw_side_slip() {}

/// Draw elevation bars on the HUD
pub fn draw_elevation_bars() {}

/// Draw the vertical velocity indicator (flight path indicator)
pub fn draw_vvi() {}

/// Draw a depiction of battery life, including voltage.
pub fn draw_battery() {}

// msp-osd: dji_display.h/c

struct DjiDisplayState {
    duss_disp_plane_id_t plane_id;
    duss_hal_obj_handle_t disp_handle;
    duss_hal_obj_handle_t ion_handle;
    vop_id: DussDispVopId,
    ion_buf_0: DussHalMemHandle,
    ion_buf_1: DussHalMemHandle,
    // void * fb0_virtual_addr;
    // void * fb0_physical_addr;
    // void * fb1_virtual_addr;
    // void * fb1_physical_addr;
    duss_disp_instance_handle_t *disp_instance_handle;
    duss_frame_buffer_t *fb_0;
    duss_frame_buffer_t *fb_1;
    duss_disp_plane_blending_t *pb_0;
    is_v2_goggles: u8,
}

const V2_SERVICE_NAME: &'static str = "dji.glasses_wm150_service";
const V1_SERVICE_NAME: &'static str = "dji.glasses_service";
const DEVICE_PROPERTY_NAME: &'static str = "ro.product.device";
const V2_GOGGLES_DEVICE: &'static str = "pigeon_wm170_gls";

// todo type?
const GOGGLES_V1_VOFFSET: u16 = 575;
const GOGGLES_V2_VOFFSET: u16 = 215;

// static duss_result_t pop_func(duss_disp_instance_handle_t *disp_handle,duss_disp_plane_id_t plane_id, duss_frame_buffer_t *frame_buffer,void *user_ctx) {
//     return 0;
// }

fn dji_display_state_alloc(is_v2_goggles: u8) -> DjiDisplayState {
    dji_display_state_t *display_state = calloc(1, sizeof(dji_display_state_t));
    display_state.disp_instance_handle = (duss_disp_instance_handle_t *)calloc(1, sizeof(duss_disp_instance_handle_t));
    display_state.fb_0 = (duss_frame_buffer_t *)calloc(1,sizeof(duss_frame_buffer_t));
    display_state.fb_1 = (duss_frame_buffer_t *)calloc(1,sizeof(duss_frame_buffer_t));
    display_state.pb_0 = (duss_disp_plane_blending_t *)calloc(1, sizeof(duss_disp_plane_blending_t));
    display_state.is_v2_goggles = is_v2_goggles;
    display_state

    DisplayState {
        is_v2_goggles,
    }

}

// todo: These should probably be methods since they all include DjiDispState as default arg.

// fn dji_display_state_free(display_state: &DjiDisplayState) {
//     free(display_state.disp_instance_handle);
//     free(display_state.fb_0);
//     free(display_state.fb_1);
//     free(display_state.pb_0);
//     free(display_state);
// }

fn dji_display_close_framebuffer(display_state: &DjiDisplayState) {

    duss_hal_display_port_enable(display_state.disp_instance_handle, 3, 0);
    duss_hal_display_release_plane(display_state.disp_instance_handle, display_state.plane_id);
    duss_hal_display_close(display_state.disp_handle, &display_state.disp_instance_handle);
    duss_hal_mem_free(display_state.ion_buf_0);
    duss_hal_mem_free(display_state.ion_buf_1);
    duss_hal_device_close(display_state.disp_handle);
    duss_hal_device_stop(display_state.ion_handle);
    duss_hal_device_close(display_state.ion_handle);
    duss_hal_deinitialize();
}

fn dji_display_open_framebuffer(display_state: &DjiDisplayState, plane_id: DussDispPlanId) {
    let mut hal_device_open_unk = 0_u32;
    let mut res: DussResult = 0;

    display_state.plane_id = plane_id;

    // PLANE BLENDING

    display_state.pb_0.is_enable = 1;
    display_state.pb_0.voffset = GOGGLES_V1_VOFFSET; // TODO just check hwid to figure this out
    display_state.pb_0.hoffset = 0;
    display_state.pb_0.order = 2;

    // Global alpha - disable as we want per pixel alpha.

    display_state.pb_0.glb_alpha_en = 0;
    display_state.pb_0.glb_alpha_val = 0;

    // Blending algorithm 1 seems to work.

    display_state.pb_0.blending_alg = 1;

    let device_descs: [DussHalDeviceDesc; 3] = [
        ["/dev/dji_display", &duss_hal_attach_disp, &duss_hal_detach_disp, 0x0],
        ["/dev/ion", &duss_hal_attach_ion_mem, &duss_hal_detach_ion_mem, 0x0],
        [0,0,0,0]
    ];

    duss_hal_initialize(device_descs);

    res = duss_hal_device_open("/dev/dji_display",&hal_device_open_unk,&display_state.disp_handle);
    if res != 0 {
        println!("failed to open dji_display device");
        return;
    }
    res = duss_hal_display_open(display_state.disp_handle, &display_state.disp_instance_handle, 0);
    if res != 0 {
        println!("failed to open display hal");
        return;
    }

    res = duss_hal_display_reset(display_state.disp_instance_handle);
    if res != 0 {
        println!("failed to reset display");
        return;
    }

    // No idea what this "plane mode" actually does but it's different on V2
    let acquire_plane_mode = if display_state.is_v2_goggles { 6 } else { 0 };

    res = duss_hal_display_aquire_plane(display_state.disp_instance_handle,acquire_plane_mode,&plane_id);
    if res != 0 {
        println!("failed to acquire plane");
        return;
    }
    res = duss_hal_display_register_frame_cycle_callback(display_state.disp_instance_handle, plane_id, &pop_func, 0);
    if res != 0 {
        println!("failed to register callback");
        return;
    }
    res = duss_hal_display_port_enable(display_state.disp_instance_handle, 3, 1);
    if res != 0 {
        println!("failed to enable display port");
        return;
    }

    res = duss_hal_display_plane_blending_set(display_state.disp_instance_handle, plane_id, display_state.pb_0);

    if res != 0 {
        println!("failed to set blending");
        return;
    }
    res = duss_hal_device_open("/dev/ion", &hal_device_open_unk, &display_state.ion_handle);
    if res != 0 {
        println!("failed to open shared VRAM");
        return;
    }
    res = duss_hal_device_start(display_state.ion_handle,0);
    if res != 0 {
        println!("failed to start VRAM device");
        return;
    }

    res = duss_hal_mem_alloc(display_state.ion_handle,&display_state.ion_buf_0,0x473100,0x400,0,0x17);
    if res != 0 {
        println!("failed to allocate VRAM");
        return;
    }
    res = duss_hal_mem_map(display_state.ion_buf_0, &display_state.fb0_virtual_addr);
    if res != 0 {
        println!("failed to map VRAM");
        return;
    }
    res = duss_hal_mem_get_phys_addr(display_state.ion_buf_0, &display_state.fb0_physical_addr);
    if res != 0 {
        println!("failed to get FB0 phys addr");
        return;
    }
    println!("first buffer VRAM mapped virtual memory is at %p : %p\n", display_state.fb0_virtual_addr, display_state.fb0_physical_addr);

    res = duss_hal_mem_alloc(display_state.ion_handle,&display_state.ion_buf_1,0x473100,0x400,0,0x17);
    if res != 0 {
        println!("failed to allocate FB1 VRAM");
        return;
    }
    res = duss_hal_mem_map(display_state.ion_buf_1,&display_state.fb1_virtual_addr);
    if res != 0 {
        println!("failed to map FB1 VRAM");
        return;
    }
    res = duss_hal_mem_get_phys_addr(display_state.ion_buf_1, &display_state.fb1_physical_addr);
    if res != 0 {
        println!("failed to get FB1 phys addr");
        return;
    }
    println!("second buffer VRAM mapped virtual memory is at %p : %p\n", display_state.fb1_virtual_addr, display_state.fb1_physical_addr);

    for i in 0..2 {
        duss_frame_buffer_t *fb = if i != 0 { display_state.fb_1 } else { display_state.fb_0 };
        fb.buffer = if i != 0 { display_state.ion_buf_1 } else { display_state.ion_buf_0 };
        fb.pixel_format = if display_state.is_v2_goggles != 0 { DUSS_PIXFMT_RGBA8888_GOGGLES_V2 } else { DUSS_PIXFMT_RGBA8888 }; // 20012 instead on V2
        fb.frame_id = i;
        fb.planes[0].bytes_per_line = 0x1680;
        fb.planes[0].offset = 0;
        fb.planes[0].plane_height = 810;
        fb.planes[0].bytes_written = 0x473100;
        fb.width = 1440;
        fb.height = 810;
        fb.plane_count = 1;
    }
}

fn dji_display_push_frame(display_state: &DjiDisplayState, which_fb: u8) {
    let fb: DussFrameBuffer = if which_fb { display_state.fb_1 } else { display_state.fb_0};
    duss_hal_mem_sync(fb.buffer, 1);
    duss_hal_display_push_frame(display_state.disp_instance_handle, display_state.plane_id, fb);
}

fn dji_display_get_fb_address(display_state: &DjiDisplayState, which_fb: u8) {
     if which_fb != 0 { display_state.fb1_virtual_addr } else { display_state.fb0_virtual_addr }
}
