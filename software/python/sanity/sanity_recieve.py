import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
master.wait_heartbeat(timeout=10)
print("Connected")

last = time.time()
while True:
    msg = master.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED"], blocking=True, timeout=1)
    if not msg:
        continue

    now = time.time()
    if now - last > 0.5:   # 2 Hz
        print(msg)
        last = now
