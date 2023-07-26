//! This module handles CAN reception, as from the appropriate ISR

use crate::app;

use rtic::mutex_prelude::*;

use defmt::println;

use dronecan;

use packed_struct::PackedStruct;

use fdcan::{id::Id, interrupt::Interrupt};

static mut RX_BUF_CAN: [u8; 64] = [0; 64];

pub fn run(mut cx: app::can_isr::Context) {
    // todo: Set up appropriate hardware filters, like Fix2, AHRS, IMU etc.
    cx.shared.can.lock(|can| {
        can.clear_interrupt(Interrupt::RxFifo0NewMsg);

        let rx_buf = unsafe { &mut RX_BUF_CAN }; // todo

        let rx_result = can.receive0(rx_buf);

        match dronecan::get_frame_info(rx_result) {
            Ok(frame_info) => {
                let id = match frame_info.id {
                    Id::Standard(id) => id.as_raw() as u32,
                    Id::Extended(id) => id.as_raw(),
                };

                let can_id = dronecan::CanId::from_value(id);
                // println!(
                //     "Frame info. Len: {}, ts: {}, pri: {}, type_id: {}, source id: {}",
                //     frame_info.len,
                //     frame_info.time_stamp,
                //     can_id.priority.val(),
                //     can_id.type_id,
                //     can_id.source_node_id
                // );

                // let tail_byte = dronecan::get_tail_byte(&rx_buf, frame_info.len).ok();
                let tail_byte = dronecan::get_tail_byte(rx_buf, frame_info.len).ok();

                if let Some(tail_byte) = tail_byte {
                    //     println!(
                    //         "Start of xfer: {}, end: {}, toggle: {}, transfer_id: {}",
                    //         tail_byte.start_of_transfer,
                    //         tail_byte.end_of_transfer,
                    //         tail_byte.toggle,
                    //         tail_byte.transfer_id
                    //     );

                    if can_id.type_id != 20007 {
                        println!("Id: {}", can_id.type_id);
                    }

                    // todo: See notes on GNSS CAN firmware about why we hard-code these
                    // todo match arm vals.

                    match can_id.type_id {
                        1_063 => {
                            let fix = dronecan::gnss::FixDronecan::unpack(
                                rx_buf[0..dronecan::MsgType::Fix2.payload_size() as usize]
                                    .try_into()
                                    .unwrap(),
                            );
                            match fix {
                                Ok(f) => {
                                    println!(
                                        "Fix. Time: {}. Lat: {}. Lon: {}. Msl: {}",
                                        f.gnss_timestamp,
                                        f.latitude_deg_1e8 as f32 / 10_000_000.,
                                        f.longitude_deg_1e8 as f32 / 10_000_000.,
                                        f.height_msl_mm as f32 / 1_000.,
                                    );
                                }
                                Err(_) => {
                                    println!("Error unpacking fix");
                                }
                            }
                            // println!("Test broadcast");
                            // // todo temp: Testing config get/set.
                            // dronecan::broadcast(
                            //     can,
                            //     dronecan::MsgPriority::Slow,
                            //     2_110,
                            //     1,
                            //     0,
                            //     &mut [5, 6, 7, 8, 9, 10, 11, 12],
                            //     8, //
                            //     true,
                            // ).ok();

                            // let mut cfg_to_set = &[0; 8]
                            // dronecan::broadcast(
                            //     can,
                            //     dronecan::MsgPriority::Slow,
                            //     2_111,
                            //     0,
                            //     0,
                            //     &mut cfg_to_set,
                            //     0,
                            //     true,
                            // ).ok();
                        }
                        1_028 => {
                            let pressure = f32::from_le_bytes(rx_buf[0..4].try_into().unwrap());
                            println!("Pressure: {} kPa", pressure / 1_000.);
                        }
                        1_029 => {
                            // let temp =
                            //     f32::from(f16::from_le_bytes(rx_buf[0..2].try_into().unwrap()));
                            // println!("Temp: {} K", temp);
                        }
                        1_002 => {
                            // todo
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
                        _ => {
                            println!("Unknown message type received: {}", can_id.type_id);
                            println!("Rx buf: {:?}", rx_buf);
                        }
                    }
                }
            }
            Err(_) => {
                println!("Error getting frame info")
            }
        }
    });
}
