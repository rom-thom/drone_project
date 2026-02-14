use anyhow::{anyhow, Result};
use mavlink::common::{
    MavCmd, MavMessage, COMMAND_LONG_DATA, LOCAL_POSITION_NED_DATA,
    SET_POSITION_TARGET_LOCAL_NED_DATA, MavFrame,
};
use mavlink::common::PositionTargetTypemask;
use mavlink_io::types::Px4Client;
use std::time::{Duration, Instant};

/// Helper: build and send a position setpoint (position + yaw), ignore vel/accel.
fn send_position_setpoint(px4: &mut Px4Client, x: f32, y: f32, z: f32, yaw: f32) -> Result<()> {
    // type_mask bits: ignore vx,vy,vz, ax,ay,az, yaw_rate (we control x,y,z,yaw only)
    // These constants are generated in the mavlink crate; to avoid fighting names,
    // we use the numeric mask directly.
    //
    // Bits (from MAVLink spec):
    // 0..2 position ignore, 3..5 velocity ignore, 6..8 accel ignore, 9 force, 10 yaw ignore, 11 yaw_rate ignore
    // We want: position USED (bits 0..2 = 0), velocity ignored (3..5=1),
    // accel ignored (6..8=1), yaw USED (bit 10 = 0), yaw_rate ignored (bit 11 = 1).
    
let type_mask =
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE |
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE |
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE |
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE |
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
        time_boot_ms: px4.time_elapsed_ms(),
        target_system: px4.target_sys,
        target_component: px4.target_comp,
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
        yaw,
        yaw_rate: 0.0,
    });

    px4.send(&msg)
}

/// Send ARM/DISARM (does not wait for ACK)
fn send_arm(px4: &mut Px4Client, arm: bool) -> Result<()> {
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: px4.target_sys,
        target_component: px4.target_comp,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: if arm { 1.0 } else { 0.0 },
        param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0,
    });
    px4.send(&msg)
}

/// Switch to OFFBOARD (does not wait for ACK)
fn send_mode_offboard(px4: &mut Px4Client) -> Result<()> {
    // PX4 commonly uses custom main mode 6 for OFFBOARD when using MAV_CMD_DO_SET_MODE
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: px4.target_sys,
        target_component: px4.target_comp,
        command: MavCmd::MAV_CMD_DO_SET_MODE,
        confirmation: 0,
        // param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
        param1: 1.0,
        // param2 = custom main mode (PX4): 6 = OFFBOARD (common)
        param2: 6.0,
        param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0,
    });
    px4.send(&msg)
}

/// Block until we get a LOCAL_POSITION_NED
fn wait_local_position(px4: &mut Px4Client, timeout: Duration) -> Result<LOCAL_POSITION_NED_DATA> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let (_h, msg) = px4.recv()?;
        if let MavMessage::LOCAL_POSITION_NED(p) = msg {
            return Ok(p);
        }
    }
    Err(anyhow!("Timed out waiting for LOCAL_POSITION_NED"))
}

fn main() -> Result<()> {
    // Typical SITL: PX4 sends telemetry to 14550, listens for commands on 14540
    let mut px4 = Px4Client::connect("udpin:0.0.0.0:14540", "udpout:127.0.0.1:14550")?;

    px4.wait_heartbeat()?;
    let p0 = wait_local_position(&mut px4, Duration::from_secs(5))?;
    let x0 = p0.x;
    let y0 = p0.y;
    let z0 = p0.z;

    println!("Start pos NED: x={:.2} y={:.2} z={:.2}", x0, y0, z0);

    let takeoff_alt_m = 2.0;
    let z_takeoff = z0 - takeoff_alt_m; // NED: up is more negative z
    let yaw = 0.0;

    // Offboard requires setpoints before switching mode
    let hz = 20.0;
    let dt = Duration::from_secs_f32(1.0 / hz);

    println!("Pre-stream hold setpoints (2s) ...");
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(2) {
        send_position_setpoint(&mut px4, x0, y0, z0, yaw)?;
        std::thread::sleep(dt);
    }

    println!("Switching OFFBOARD + ARM ...");
    send_mode_offboard(&mut px4)?;
    send_arm(&mut px4, true)?;

    // Takeoff ramp
    println!("Takeoff ramp ...");
    let ramp = Duration::from_secs(3);
    let t1 = Instant::now();
    while t1.elapsed() < ramp {
        let a = (t1.elapsed().as_secs_f32() / ramp.as_secs_f32()).clamp(0.0, 1.0);
        let z_cmd = (1.0 - a) * z0 + a * z_takeoff;
        send_position_setpoint(&mut px4, x0, y0, z_cmd, yaw)?;
        std::thread::sleep(dt);
    }

    println!("Hover (5s) ...");
    let t2 = Instant::now();
    while t2.elapsed() < Duration::from_secs(5) {
        send_position_setpoint(&mut px4, x0, y0, z_takeoff, yaw)?;
        std::thread::sleep(dt);
    }

    // Land ramp back to start z
    println!("Land ramp ...");
    let land = Duration::from_secs(5);
    let t3 = Instant::now();
    while t3.elapsed() < land {
        let a = (t3.elapsed().as_secs_f32() / land.as_secs_f32()).clamp(0.0, 1.0);
        let z_cmd = (1.0 - a) * z_takeoff + a * z0;
        send_position_setpoint(&mut px4, x0, y0, z_cmd, yaw)?;
        std::thread::sleep(dt);
    }

    // Movement-based sanity: confirm we actually went up at some point.
    // (We can do a quick read loop for 1s and check z got near z_takeoff.)
    println!("Sanity check: did altitude change?");
    let mut saw_takeoff = false;
    let deadline = Instant::now() + Duration::from_secs(1);
    while Instant::now() < deadline {
        let (_h, msg) = px4.recv()?;
        if let MavMessage::LOCAL_POSITION_NED(p) = msg {
            if (p.z - z_takeoff).abs() < 0.5 {
                saw_takeoff = true;
                break;
            }
        }
    }
    println!("Takeoff observed: {}", saw_takeoff);

    // Optional: disarm (may be denied if PX4 doesn't think it's landed yet)
    println!("Attempt DISARM ...");
    let _ = send_arm(&mut px4, false);

    println!("Done.");
    Ok(())
}
