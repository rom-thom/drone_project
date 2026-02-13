#!/usr/bin/env python3
import time
from pymavlink import mavutil

PORT = 14550
SETPOINT_HZ = 20.0
DT = 1.0 / SETPOINT_HZ

TAKEOFF_ALT_M = 2.0
HOVER_TIME_S = 5.0
LAND_TIME_S = 6.0

SCRIPT_START = time.time()

def now_boot_ms_u32():
    return int((time.time() - SCRIPT_START) * 1000) & 0xFFFFFFFF

def wait_cmd_ack(master, command, timeout_s=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return msg
    return None

def send_arm(master, arm: bool):
    cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    master.mav.command_long_send(
        master.target_system, master.target_component,
        cmd, 0,
        1.0 if arm else 0.0,
        0, 0, 0, 0, 0, 0
    )
    return wait_cmd_ack(master, cmd, timeout_s=3.0)

def set_mode_offboard(master):
    cmd = mavutil.mavlink.MAV_CMD_DO_SET_MODE
    master.mav.command_long_send(
        master.target_system, master.target_component,
        cmd, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0   # PX4 main mode 6 is commonly OFFBOARD
    )
    return wait_cmd_ack(master, cmd, timeout_s=3.0)

def send_position_setpoint(master, x, y, z, yaw_rad):
    # Ignore velocity/accel, control position + yaw
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        now_boot_ms_u32(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw_rad, 0
    )

def get_local_position(master, timeout_s=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
        if msg:
            return msg
    return None

def main():
    master = mavutil.mavlink_connection(f"udp:127.0.0.1:{PORT}")
    print(f"Connecting on UDP {PORT} ...")
    master.wait_heartbeat(timeout=10)
    print(f"Heartbeat OK. System={master.target_system} Component={master.target_component}")

    pos = get_local_position(master)
    if not pos:
        raise RuntimeError("No LOCAL_POSITION_NED received. Wait for EKF, and ensure Gazebo is playing.")
    x0, y0, z0 = pos.x, pos.y, pos.z
    print(f"Start local pos: x={x0:.2f} y={y0:.2f} z={z0:.2f} (NED, meters)")

    target_x, target_y = x0, y0
    target_z_takeoff = z0 - TAKEOFF_ALT_M  # up = more negative in NED
    yaw = 0.0

    print("Streaming setpoints (pre-offboard) ...")
    t = time.time()
    while time.time() - t < 2.0:
        send_position_setpoint(master, target_x, target_y, z0, yaw)
        time.sleep(DT)

    print("Switching to OFFBOARD ...")
    print("OFFBOARD ack:", set_mode_offboard(master))

    print("Arming ...")
    print("ARM ack:", send_arm(master, True))

    print("Taking off ...")
    ramp_time = 3.0
    t = time.time()
    while time.time() - t < ramp_time:
        a = (time.time() - t) / ramp_time
        z_cmd = (1 - a) * z0 + a * target_z_takeoff
        send_position_setpoint(master, target_x, target_y, z_cmd, yaw)
        time.sleep(DT)

    print(f"Hovering for {HOVER_TIME_S}s ...")
    t = time.time()
    while time.time() - t < HOVER_TIME_S:
        send_position_setpoint(master, target_x, target_y, target_z_takeoff, yaw)
        time.sleep(DT)

    print("Landing ...")
    t = time.time()
    while time.time() - t < LAND_TIME_S:
        a = (time.time() - t) / LAND_TIME_S
        z_cmd = (1 - a) * target_z_takeoff + a * z0
        send_position_setpoint(master, target_x, target_y, z_cmd, yaw)
        time.sleep(DT)

    print("Disarming ...")
    print("DISARM ack:", send_arm(master, False))
    print("Done.")

if __name__ == "__main__":
    main()
