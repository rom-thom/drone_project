// This is for testing the send functionality (usinf recv for confirming the sending)


//!!!!! it does not work at the moment


use mavlink_io::types::Px4Client;
use anyhow::Result;
use mavlink::common::{MavCmd, MavMessage, COMMAND_LONG_DATA};


fn main() -> Result<()> {
    let mut px4 = Px4Client::connect("udpin:0.0.0.0:14540", "udpout:127.0.0.1:14550")?;
    px4.wait_heartbeat()?;


    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: px4.target_sys,
        target_component: px4.target_comp,
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        confirmation: 0,
        param1: 1.0, // message id: SYS_STATUS = 1
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });

    px4.send(&msg)?;
    let ack = px4.wait_command_ack(MavCmd::MAV_CMD_REQUEST_MESSAGE, 3000)?;

    println!("ACK result = {:?} (0=accepted, 1=temporarily rejected, 4=denied, ...)", ack.result);
    Ok(())
}
