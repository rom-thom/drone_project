



use crate::types::Px4Client;

use anyhow::Result;
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::Duration,
};
use mavlink::common::*;

pub fn run_offboard_sequence(rx_addr: &str, tx_addr: &str) -> Result<()> {
    // Main client: used for receive + "one-shot" commands (arm/mode).
    let mut client = Px4Client::connect(rx_addr, tx_addr)?;

    // 1) Wait for PX4 heartbeat (vehicle presence)
    client.wait_heartbeat()?;

    // Create dedicated TX-only clients for periodic loops
    let mut hb_sender = Px4Client::connect(rx_addr, tx_addr)?;
    hb_sender.target_sys = client.target_sys;
    hb_sender.target_comp = client.target_comp;

    let mut sp_sender = Px4Client::connect(rx_addr, tx_addr)?;
    sp_sender.target_sys = client.target_sys;
    sp_sender.target_comp = client.target_comp;

    let running = Arc::new(AtomicBool::new(true));

    // 2) Start sending your own heartbeat (~1 Hz)
    {
        let running = running.clone();
        thread::spawn(move || {
            while running.load(Ordering::Relaxed) {
                let _ = hb_sender.send_gcs_heartbeat();
                thread::sleep(Duration::from_millis(1000));
            }
        });
    }

    // 3) Start streaming Offboard setpoints (~20 Hz)
    {
        let running = running.clone();
        thread::spawn(move || {
            // Example: hold at (0,0,-1) with yaw 0 rad.
            // (In NED, z=-1 is ~1m above origin)
            while running.load(Ordering::Relaxed) {
                let _ = sp_sender.send_offboard_setpoint_local_ned(0.0, 0.0, -1.0, 0.0);
                thread::sleep(Duration::from_millis(50)); // 20 Hz
            }
        });
    }

    // Give PX4 time to see >2Hz setpoints for >1s
    thread::sleep(Duration::from_millis(1200));

    // 4) Send arm command
    client.arm()?;

    // 5) Switch to Offboard mode
    client.set_mode_offboard()?;

    // 6) Keep streaming setpoints continuously
    // (your setpoint thread keeps running; just keep the program alive)
    println!("✅ Armed + Offboard. Streaming setpoints. Press Ctrl+C to stop.");
    loop {
        // Optionally: read telemetry here to confirm mode / position / etc.
        let _ = client.recv();
        thread::sleep(Duration::from_millis(10));
    }

    // If you later implement clean shutdown:
    // running.store(false, Ordering::Relaxed);
    // Ok(())
}
