// This is for testing the recieveing ofg data trough mavlink


use anyhow::Result;
use mavlink::{connect, MavConnection};
use mavlink::common::MavMessage;
use std::time::{Duration, Instant};

fn main() -> Result<()> {
    // Listen for incoming MAVLink packets on UDP port 14540 (change if needed)
    let mut conn = connect::<MavMessage>("udpin:0.0.0.0:14540")?;

    println!("Listening on udpin:0.0.0.0:14540 ... waiting for HEARTBEAT");

    // Wait for heartbeat so we know we're connected
    loop {
        let (header, msg) = conn.recv()?;
        if matches!(msg, MavMessage::HEARTBEAT(_)) {
            println!(
                "Heartbeat received from sys={}, comp={}",
                header.system_id, header.component_id
            );
            break;
        }
    }

    // Print at most once per interval
    let interval = Duration::from_secs_f32(1.0); // <-- change to 0.5, 2.0, etc.
    let mut next_print = Instant::now();

    // We'll keep the latest values and print them on schedule
    let mut last_att: Option<(f32, f32, f32, f32, f32, f32)> = None;
    let mut last_pos: Option<(f32, f32, f32, f32, f32, f32)> = None;

    loop {
        let (_header, msg) = conn.recv()?;

        match msg {
            MavMessage::ATTITUDE(a) => {
                last_att = Some((a.roll, a.pitch, a.yaw, a.rollspeed, a.pitchspeed, a.yawspeed));
            }
            MavMessage::LOCAL_POSITION_NED(p) => {
                last_pos = Some((p.x, p.y, p.z, p.vx, p.vy, p.vz));
            }
            _ => {}
        }

        if Instant::now() >= next_print {
            if let Some((roll, pitch, yaw, p, q, r)) = last_att {
                println!(
                    "ATTITUDE roll={:.4} pitch={:.4} yaw={:.4} p={:.4} q={:.4} r={:.4}",
                    roll, pitch, yaw, p, q, r
                );
            }
            if let Some((x, y, z, vx, vy, vz)) = last_pos {
                println!(
                    "LOCAL_POSITION_NED x={:.2} y={:.2} z={:.2} vx={:.2} vy={:.2} vz={:.2}",
                    x, y, z, vx, vy, vz
                );
            }
            println!("---");
            next_print += interval;
        }
    }
}
