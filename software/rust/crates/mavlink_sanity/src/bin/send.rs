use std::thread;
use std::time::{Duration, Instant};

use mavlink::Message;
use mavlink::{
    common::{
        self, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavState, MavType,
    },
    connect, MavConnection, MavHeader,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // In PX4 SITL, your offboard app usually listens here.
    let mut conn = connect("udpin:0.0.0.0:14540")?;

    println!("Waiting for PX4 heartbeat on 14540...");

    let (px4_header, heartbeat) = loop {
        let (header, msg) = conn.recv()?;
        if let MavMessage::HEARTBEAT(hb) = msg {
            println!(
                "Got heartbeat from system={} component={}, type={:?}",
                header.system_id, header.component_id, hb.mavtype
            );
            break (header, hb);
        }
    };

    // We are the companion computer / offboard app.
    let my_header = MavHeader {
        system_id: 42,
        component_id: 191, // MAV_COMP_ID_ONBOARD_COMPUTER-ish convention
        sequence: 0,
    };

    // 1) Send our own heartbeat so PX4 sees us as a MAVLink peer.
    let hb = MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

    conn.send(&my_header, &hb)?;
    println!("Sent heartbeat to PX4.");

    // 2) Ask PX4 to publish LOCAL_POSITION_NED at 10 Hz.
    // This is a nice transmit test because you can verify the response by receiving the messages.
    let req = MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
        target_system: px4_header.system_id,
        target_component: px4_header.component_id,
        command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        confirmation: 0,
        param1: common::MavMessage::LOCAL_POSITION_NED(Default::default()).message_id() as f32,
        param2: 100_000.0, // microseconds => 10 Hz
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });

    conn.send(&my_header, &req)?;
    println!("Requested LOCAL_POSITION_NED at 10 Hz.");

    // 3) Read back messages for a few seconds to confirm PX4 accepted the request.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        let (_header, msg) = conn.recv()?;
        match msg {
            MavMessage::COMMAND_ACK(ack) => {
                println!(
                    "COMMAND_ACK: command={:?}, result={:?}",
                    ack.command, ack.result
                );
            }
            MavMessage::LOCAL_POSITION_NED(pos) => {
                println!(
                    "LOCAL_POSITION_NED: x={:.2} y={:.2} z={:.2} vx={:.2} vy={:.2} vz={:.2}",
                    pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz
                );
            }
            _ => {}
        }
    }

    println!("Done.");
    Ok(())
}