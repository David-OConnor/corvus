//! This module handles CAN reception, as from the appropriate ISR

use rtic::mutex_prelude::*;

use defmt::println;

use fdcan::{id::Id, interrupt::Interrupt};

use dronecan::{
    CanBitrate, CanId, FrameType, GetSetRequest, MsgType, NodeHealth, NodeMode, NodeStatus, Value,
    NAME_CUTOFF,
};

use packed_struct::PackedStruct;

use crate::{app, drivers::gnss_can};

static mut RX_BUF_CAN: [u8; 100] = [0; 100];

pub fn run(mut cx: app::can_isr::Context) {
    // todo: Set up appropriate hardware filters, like Fix2, AHRS, IMU etc.
    let rx_buf = unsafe { &mut RX_BUF_CAN };
    *rx_buf = [0; 100]; // Clear out existing data.

    cx.shared.can.lock(|can| {
        can.clear_interrupt(Interrupt::RxFifo0NewMsg);

        println!("CAN ISR");
        let rx_result = can.receive0(rx_buf);

        match dronecan::get_frame_info(rx_result) {
            Ok(frame_info) => {
                let id = match frame_info.id {
                    Id::Standard(id) => id.as_raw() as u32,
                    Id::Extended(id) => id.as_raw(),
                };

                let can_id = CanId::from_value(id);

                // Code cleaners as we use these in many arms.
                let fd_mode = false; // todo: True!

                // We use this for service transfers; not used otherwise.
                // let mut addressed_to_us = false;
                //
                // if let FrameType::Service(svc_data) = &can_id.frame_type {
                //     addressed_to_us = svc_data.dest_node_id == *node_id;
                // }

                // todo: Keep this in sync with Sail.
                match can_id.type_id {
                    1_063 => {
                        println!("Parsing DroneCAN fix.");
                        let fix = gnss_can::parse_fix(
                            &rx_buf[..dronecan::MsgType::Fix2.payload_size() as usize],
                        );

                        match fix {
                            Ok(f) => {
                                println!(
                                    "Fix. Time: {}. Lat: {}. Lon: {}. Msl: {}",
                                    f.timestamp_s, f.lat_e7, f.lon_e7, f.elevation_msl,
                                );
                            }
                            Err(_) => {
                                println!("Error parsing fix");
                            }
                        }
                    }
                    29 => {
                        println!("AHRS solution");
                    }
                    1_028 => {
                        let pressure = f32::from_le_bytes(rx_buf[0..4].try_into().unwrap());
                        println!("Pressure: {} kPa", pressure / 1_000.);
                    }
                    1_029 => {
                        println!("Temp");
                        // let temp =
                        //     f32::from(f16::from_le_bytes(rx_buf[0..2].try_into().unwrap()));
                        // println!("Temp: {} K", temp);
                    }
                    1_002 => {
                        println!("Magnetic field strength");
                        // let x =
                        //     f32::from(f16::from_le_bytes(rx_buf[1..3].try_into().unwrap()));
                        // let y =
                        //     f32::from(f16::from_le_bytes(rx_buf[3..5].try_into().unwrap()));
                        // let z =
                        //     f32::from(f16::from_le_bytes(rx_buf[5..7].try_into().unwrap()));
                        // println!("Mag. x: {}, y: {}, z: {}", x, y, z);
                    }
                    341 => {
                        let uptime = u32::from_le_bytes(rx_buf[0..4].try_into().unwrap());
                        println!(
                            "Node status. Uptime sec: {}, health: {}, mode; {}",
                            uptime, rx_buf[4], rx_buf[5]
                        );
                    }
                    3_115 => {
                        println!("Position fused");
                    }
                    1_140 => {
                        println!("RC input");
                    }
                    1_141 => {
                        println!("Link stats");
                    }
                    _ => {
                        println!("Unknown message type received: {}", can_id.type_id);
                        println!("Rx buf: {:?}", rx_buf);
                    }
                }
            }
            Err(_) => {
                println!("Error getting frame info")
            }
        }
    });
}
