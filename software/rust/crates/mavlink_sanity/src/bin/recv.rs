// This is for testing the recieveing ofg data trough mavlink


use anyhow::Result;
use mavlink::common::MavMessage;
use mavlink_io::types::Px4Client;

use std::time::{Duration, Instant};

fn main() -> Result<()> {
    let mut px4 = Px4Client::connect("udpin:0.0.0.0:14540", "udpout:127.0.0.1:14550")?;

    px4.wait_heartbeat()?;

    // Print once per second
    let interval = Duration::from_secs(1);
    let mut next_print = Instant::now() + interval;

    // Cache latest values
    let mut last_att: Option<(f32, f32, f32)> = None; // yaw, pitch, roll
    let mut last_pos: Option<(f32, f32, f32, f32, f32, f32)> = None; // x,y,z,vx,vy,vz

    loop {
        let (_h, msg) = px4.recv()?;

        match msg {
            MavMessage::ATTITUDE(a) => {
                last_att = Some((a.yaw, a.pitch, a.roll));
            }
            MavMessage::LOCAL_POSITION_NED(p) => {
                last_pos = Some((p.x, p.y, p.z, p.vx, p.vy, p.vz));
            }
            _ => {}
        }

        let now = Instant::now();
        if now >= next_print {
            if let Some((yaw, pitch, roll)) = last_att {
                println!("ATT yaw={:.3}, pitch={:.3}, roll={:.3}", yaw, pitch, roll);
            } else {
                println!("ATT (no data yet)");
            }

            if let Some((x, y, z, vx, vy, vz)) = last_pos {
                println!(
                    "POS x={:.2}, y={:.2}, z={:.2} | v=({:.2},{:.2},{:.2})",
                    x, y, z, vx, vy, vz
                );
            } else {
                println!("POS (no data yet)");
            }

            println!("---");
            next_print += interval; // keeps cadence even if we print late once
        }
    }
}
