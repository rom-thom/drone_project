pub mod heartbeat;    // wait_heartbeat, identify sys/comp
pub mod commands;     // arm/disarm, set_mode, ACK parsing
pub mod transport;    // udp in/out, ports, connect
pub mod telemetry;    // read attitude/position (helpers)
pub mod offboard;     // setpoint messages, streaming helpers
pub mod types;        // shared structs/config (PortConfig etc.)