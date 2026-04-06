


use crate::types::Px4Client;
use anyhow::Result;
use mavlink::MavConnection;
use mavlink::common::MavMessage;




impl Px4Client {
    /// Block until a HEARTBEAT is received; sets target sys/comp based on sender.
    pub fn wait_heartbeat(&mut self) -> Result<()> {
        println!("Waiting for HEARTBEAT ...");
        loop {
            let (h, m) = self.recv()?;
            if matches!(m, MavMessage::HEARTBEAT(_)) {
                self.target_sys = h.system_id;
                self.target_comp = h.component_id;
                println!(
                    "Heartbeat from sys={}, comp={}, seq={}",
                    self.target_sys, self.target_comp, h.sequence
                );
                return Ok(());
            }
        }
    }
}