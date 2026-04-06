use mavlink_io::offboard::run_offboard_sequence;



fn main() -> anyhow::Result<()> {
    run_offboard_sequence("udpin:0.0.0.0:14540", "udpout:127.0.0.1:14550")?;
    Ok(())
}
