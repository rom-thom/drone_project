
use crate::types::Px4Client;
use anyhow::{anyhow, Result};
use mavlink::{connect, MavConnection, MavHeader};
use mavlink::common::{MavCmd, MavMessage, COMMAND_ACK_DATA};
use std::time::Instant;



impl Px4Client {
    /// Create a new client with a dedicated RX and TX connection.
    /// Example:
    ///   rx = "udpin:0.0.0.0:14560"
    ///   tx = "udpout:127.0.0.1:14540"
    pub fn connect(rx_addr: &str, tx_addr: &str) -> Result<Self> {
        let rx = connect::<MavMessage>(rx_addr)?;
        let tx = connect::<MavMessage>(tx_addr)?;

        Ok(Self {
            rx,
            tx,
            target_sys: 0,
            target_comp: 0,
            header: MavHeader {
                system_id: 255,     // "us" (GCS/app id)
                component_id: 190,  // arbitrary component id
                sequence: 0,
            },
            start: Instant::now(),
        })
    }

    /// Time since this client started
    pub fn time_elapsed_ms(&self) -> u32 {
        self.start.elapsed().as_millis() as u32
    }

    /// Block until a HEARTBEAT is received; sets target sys/comp based on sender.
    pub fn wait_heartbeat(&mut self) -> Result<()> {
        println!("Waiting for HEARTBEAT ...");
        loop {
            let (h, m) = self.rx.recv()?;
            if matches!(m, MavMessage::HEARTBEAT(_)) {
                self.target_sys = h.system_id;
                self.target_comp = h.component_id;
                println!(
                    "Heartbeat from sys={}, comp={}",
                    self.target_sys, self.target_comp
                );
                return Ok(());
            }
        }
    }


    /// Whaiting for acknolegment that the command was sendt. It tries to listen for an ack sendt from Px4
    pub fn wait_command_ack(&mut self, cmd: MavCmd, timeout_ms: u64) -> Result<COMMAND_ACK_DATA> { 
        let deadline = std::time::Instant::now() + std::time::Duration::from_millis(timeout_ms);

        while std::time::Instant::now() < deadline {
            let (_h, msg) = self.rx.recv()?; // blocks until a message arrives
            dbg!(&msg);
            if let MavMessage::COMMAND_ACK(ack) = msg {
                if ack.command == cmd {
                    return Ok(ack);
                }
            }
        }
        Err(anyhow!("Timed out waiting for COMMAND_ACK for {:?}", cmd))
    }

    /// Receive next MAVLink message from RX.
    pub fn recv(&mut self) -> Result<(MavHeader, MavMessage)> {
        let (h, m) = self.rx.recv()?;
        Ok((h, m))
    }

    /// Send a MAVLink message using TX (requires we know target_sys/comp for many commands).
    pub fn send(&mut self, msg: &MavMessage) -> Result<()> {
        // Increment sequence number for nicer hygiene (not strictly required in many setups)
        self.header.sequence = self.header.sequence.wrapping_add(1);
        self.tx.send(&self.header, msg)?;
        Ok(())
    }

    /// Helper to ensure we already discovered target ids.
    pub fn ensure_target(&self) -> Result<()> {
        if self.target_sys == 0 {
            Err(anyhow!("target_sys not set: call wait_heartbeat() first"))
        } else {
            Ok(())
        }
    }
}
