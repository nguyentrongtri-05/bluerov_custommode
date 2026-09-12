import re

with open("README.md", "r") as f:
    text = f.read()

old_pattern = r"## Custom Branch: BlueROV EKF Distance Hold.*?libraries/AP_CustomEKF`\."

new_block = """## Custom Branch: BlueROV EKF Distance Hold
This repository contains a custom ArduSub flight mode for obstacle avoidance, designed for BlueROV or similar underwater vehicles:
- **New Mode:** `PosHoldDistance` (PHDS, MAVLink Mode ID `22` or `19` if overridden).
- **Functionality:** Inherits standard `PosHold` behaviors but adds forward obstacle avoidance using a custom 1D Kalman Filter (KF) or Extended Kalman Filter (EKF). 
- **Sensors:** Fuses forward-facing Ping Altimeter (distance) and downward-facing DVL (velocity) data.
- **Dynamic Parameters:**
  - `PHDS_USE_EKF`: Toggle between KF (0: Ping only) and EKF (1: DVL + Ping).
  - `PHDS_DIST_MIN`: Minimum allowed distance to obstacle in meters (default: 0.5m). 
- **Safety Logic:** Automatically decelerates when an obstacle is within `PHDS_DIST_MIN + 0.5m` and completely blocks forward motion at `PHDS_DIST_MIN` to prevent collision.
- **Real-time Telemetry:** Sends `KF_Dist_cm` variable via `NAMED_VALUE_INT` MAVLink message at 5Hz to plot real-time filtered distance in QGroundControl.
- **Implementation:** Code is located in `ArduSub/mode_poshold_dist.cpp` and `libraries/AP_CustomEKF`."""

updated = re.sub(old_pattern, new_block, text, flags=re.DOTALL)

with open("README.md", "w") as f:
    f.write(updated)

print("README updated successfully")
