import time
import math
from pymavlink import mavutil

# Dùng udpclient để gửi thẳng vào MAVProxy (chạy trên 14550) hoặc TCP
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

current_dist_m = 3.0 
print("Simulating Ping Altimeter (ID=1).")
print("Press Ctrl+C to exit.")

while True:
    try:
        master.mav.distance_sensor_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            20, 3000, int(current_dist_m * 100),
            0, 1, 0, 0
        )
        current_dist_m = 1.5 + 1.3 * math.sin(time.time() * 0.5)
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
