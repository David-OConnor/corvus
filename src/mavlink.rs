//! Mavlink interface to a GCS

use mavlink::{self, common::{MavType, MavAutopilot, MavModeFlag, MavState, MavMsg, HEARBEAT_DATA}, MavlinkVersion};

fn mavlink_header() -> mavlink::MavHeader {
    mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    }
}

pub fn mavlink_heartbeat_message() -> mavlink::common::MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_SUBMARINE,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

pub fn send() {
    let header = mavlink_header();
    let heartbeat = mavlink_heartbeat_message();

    mavlink::write_versioned_msg(&mut tx, MavlinkVersion::V2, header, &heartbeat)
        .unwrap();
}