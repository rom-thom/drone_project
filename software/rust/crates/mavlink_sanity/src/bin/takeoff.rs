use std::thread;
use std::time::{Duration, Instant};

use mavlink::{
    common::{
        self, MavAutopilot, MavCmd, MavFrame, MavMessage, MavModeFlag, MavResult,
        MavState, MavType,
    },
    connect, MavHeader,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut conn = connect("udpin:0.0.0.0:14540")?;

    println!("Waiting for PX4 heartbeat...");
    let px4 = wait_heartbeat(&mut conn)?;

    println!(
        "Connected to PX4: system={}, component={}",
        px4.system_id, px4.component_id
    );

    let mut me = MavHeader {
        system_id: 42,
        component_id: 191,
        sequence: 0,
    };

    // Announce ourselves
    send_heartbeat(&mut conn, &mut me)?;

    // Target hover point in LOCAL_NED:
    // x=0, y=0, z=-2 means hover 2 meters above local origin.
    let hover_x = 0.0_f32;
    let hover_y = 0.0_f32;
    let hover_z = -2.0_f32;

    println!("Streaming setpoints for 1.5 seconds before entering OFFBOARD...");
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(1500) {
        send_position_setpoint(&mut conn, &mut me, &px4, hover_x, hover_y, hover_z)?;
        drain_messages(&mut conn, Duration::from_millis(20))?;
        thread::sleep(Duration::from_millis(100)); // 10 Hz
    }

    println!("Requesting OFFBOARD mode...");
    set_offboard_mode(&mut conn, &mut me, &px4)?;

    println!("Arming...");
    arm(&mut conn, &mut me, &px4, true)?;

    println!("Holding hover setpoint. Press Ctrl+C to stop.");
    loop {
        send_position_setpoint(&mut conn, &mut me, &px4, hover_x, hover_y, hover_z)?;
        drain_messages(&mut conn, Duration::from_millis(20))?;
        thread::sleep(Duration::from_millis(100)); // 10 Hz
    }
}

fn wait_heartbeat<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
) -> Result<MavHeader, Box<dyn std::error::Error>> {
    loop {
        let (header, msg) = conn.recv()?;
        if let MavMessage::HEARTBEAT(hb) = msg {
            println!(
                "Heartbeat: type={:?}, armed_flag={:?}, system_status={:?}",
                hb.mavtype, hb.base_mode, hb.system_status
            );
            return Ok(header);
        }
    }
}

fn send_heartbeat<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    me: &mut MavHeader,
) -> Result<(), Box<dyn std::error::Error>> {
    let msg = MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

    send(conn, me, &msg)
}

fn send_position_setpoint<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    me: &mut MavHeader,
    px4: &MavHeader,
    x: f32,
    y: f32,
    z: f32,
) -> Result<(), Box<dyn std::error::Error>> {
    // We want PX4 to use position x/y/z and yaw,
    // and ignore velocity, acceleration, yaw rate.
    //
    // In POSITION_TARGET_TYPEMASK:
    // bit=1 means "ignore this field"
    //
    // Ignore:
    // vx, vy, vz
    // afx, afy, afz
    // yaw_rate
    //
    // Do NOT ignore:
    // x, y, z, yaw
    let type_mask = (
    common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
        | common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE
        | common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
        | common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
        | common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | common::PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);

    let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(
        common::SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,
            target_system: px4.system_id,
            target_component: px4.component_id,
            coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
            type_mask,
            x,
            y,
            z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        },
    );

    send(conn, me, &msg)
}


fn wait_until_offboard<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
) -> Result<(), Box<dyn std::error::Error>> {
    let deadline = Instant::now() + Duration::from_secs(2);

    while Instant::now() < deadline {
        let (_header, msg) = conn.recv()?;
        if let MavMessage::HEARTBEAT(hb) = msg {
            let main_mode = (hb.custom_mode >> 16) & 0xFF;
            if main_mode == 6 {
                println!("PX4 is now in OFFBOARD mode");
                return Ok(());
            }
        }
    }

    Err("Timed out waiting for OFFBOARD mode".into())
}


#[allow(deprecated)]
fn set_offboard_mode<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    me: &mut MavHeader,
    px4: &MavHeader,
) -> Result<(), Box<dyn std::error::Error>> {
    const PX4_CUSTOM_MAIN_MODE_OFFBOARD: u32 = 6;
    let custom_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16;

    let msg = MavMessage::SET_MODE(common::SET_MODE_DATA {
        target_system: px4.system_id,
        base_mode: common::MavMode::MAV_MODE_GUIDED_DISARMED,
        custom_mode,
    });

    send(conn, me, &msg)?;
    wait_until_offboard(conn)
}
fn arm<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    me: &mut MavHeader,
    px4: &MavHeader,
    arm: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let msg = MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
        target_system: px4.system_id,
        target_component: px4.component_id,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: if arm { 1.0 } else { 0.0 },
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });

    send(conn, me, &msg)?;
    wait_command_ack(conn, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM)
}

fn wait_command_ack<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    cmd: MavCmd,
) -> Result<(), Box<dyn std::error::Error>> {
    let deadline = Instant::now() + Duration::from_secs(2);

    while Instant::now() < deadline {
        let (header, msg) = conn.recv()?;
        if let MavMessage::COMMAND_ACK(ack) = msg {
            if ack.command == cmd {
                println!(
                    "ACK from system={} component={}: command={:?}, result={:?}",
                    header.system_id, header.component_id, ack.command, ack.result
                );
                if ack.result == MavResult::MAV_RESULT_ACCEPTED {
                    return Ok(());
                } else {
                    return Err(format!("Command {:?} rejected: {:?}", cmd, ack.result).into());
                }
            }
        }
    }

    Err(format!("Timeout waiting for ACK for {:?}", cmd).into())
}

fn drain_messages<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    max_time: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    let deadline = Instant::now() + max_time;

    while Instant::now() < deadline {
        match conn.recv() {
            Ok((_header, msg)) => match msg {
                MavMessage::LOCAL_POSITION_NED(pos) => {
                    println!(
                        "pos x={:.2} y={:.2} z={:.2} vx={:.2} vy={:.2} vz={:.2}",
                        pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz
                    );
                }
                MavMessage::STATUSTEXT(text) => {
                    let bytes = text.text;
                    let nul = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
                    let s = String::from_utf8_lossy(&bytes[..nul]);
                    println!("STATUSTEXT: {}", s);
                }
                _ => {}
            },
            Err(_) => break,
        }
    }

    Ok(())
}

fn send<C: mavlink::MavConnection<MavMessage>>(
    conn: &mut C,
    me: &mut MavHeader,
    msg: &MavMessage,
) -> Result<(), Box<dyn std::error::Error>> {
    conn.send(me, msg)?;
    me.sequence = me.sequence.wrapping_add(1);
    Ok(())
}