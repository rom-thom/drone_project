// This is for testing the recieveing ofg data trough mavlink


use anyhow::Result;
use mavlink::common::MavMessage;
use mavlink_io::types::Px4Client;

fn main() -> Result<()> {

    let mut px4 = Px4Client::connect("udpin:0.0.0.0:14540", "udpout:127.0.0.1:14550")?;

    px4.wait_heartbeat()?;

    loop {
        let (_h, msg) = px4.recv()?;
        match msg {
            MavMessage::ATTITUDE(a) => println!("yaw={:.3}, pitch={:.3}, roll={:.3}", a.yaw, a.pitch, a.roll),
            _ => {}
        }
    }
}
