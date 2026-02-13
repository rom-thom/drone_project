#!/usr/bin/env python3
import time
from pymavlink import mavutil

PORT = 14550

def wait_cmd_ack(master, command, timeout_s=2.0):
    """Wait for COMMAND_ACK for a specific MAVLink command id."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if not msg:
            continue
        # msg.command is the MAV_CMD id, msg.result is MAV_RESULT
        if msg.command == command:
            return msg
    return None

def send_arm(master, arm: bool):
    cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    param1 = 1.0 if arm else 0.0

    # target_system / target_component are set after heartbeat
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        cmd,
        0,      # confirmation
        param1, # param1: 1 arm, 0 disarm
        0, 0, 0, 0, 0, 0
    )
    ack = wait_cmd_ack(master, cmd, timeout_s=3.0)
    return ack

def main():
    master = mavutil.mavlink_connection(f"udp:127.0.0.1:{PORT}")

    print(f"Connecting on UDP {PORT} ...")
    hb = master.wait_heartbeat(timeout=10)
    print(f"Heartbeat OK. System={master.target_system} Component={master.target_component}")

    # Prove we're receiving something too (optional)
    msg = master.recv_match(type=["SYS_STATUS", "HEARTBEAT"], blocking=True, timeout=2)
    print("Received:", msg.get_type() if msg else "nothing (that's fine)")

    print("Sending ARM ...")
    ack = send_arm(master, True)
    if ack:
        print("ARM ACK:", ack)
        if ack.result == 1:
            print("Unable to arm")
        else:
            print("Arming succsessful")
    else:
        print("No ACK for ARM (still may have armed). Check QGC or PX4 console.")

    time.sleep(2)

    print("Sending DISARM ...")
    ack = send_arm(master, False)
    if ack:
        print("DISARM ACK:", ack)
        if ack.result == 1:
            print("Unable to disarm")
        else:
            print("Disarming succsessful")
    else:
        print("No ACK for DISARM.")

    print("Done.")

if __name__ == "__main__":
    main()
