
use mavlink::{Connection, common::MavMessage, MavHeader};





pub struct Px4Client {
    pub rx: Connection<MavMessage>,
    pub tx: Connection<MavMessage>,
    pub target_sys: u8,
    pub target_comp: u8,
    pub header: MavHeader,
    pub start: std::time::Instant,
}
