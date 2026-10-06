import re

with open("ArduSub/mode_poshold_dist.cpp", "r") as f:
    code = f.read()

# 1. Fix the run() method to include the ROTATION_PITCH_270 patch
old_run = """        RangeFinder *rangefinder = RangeFinder::get_singleton();
        if (rangefinder && rangefinder->has_data_orient(ROTATION_NONE)) {
            dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
            has_ping = true;
        }"""

new_run = """        RangeFinder *rangefinder = RangeFinder::get_singleton();
        if (rangefinder) {
            if (rangefinder->has_data_orient(ROTATION_NONE)) {
                dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
                has_ping = true;
            } else if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
                dist_m = rangefinder->distance_cm_orient(ROTATION_PITCH_270) * 0.01f;
                has_ping = true;
            }
        }"""

code = code.replace(old_run, new_run)

# 2. Fix the undefined dist_m in control_horizontal()
old_log = """        sub.gcs().send_named_int("Raw_Dist_cm", dist_m * 100);"""

new_log = """        float raw_dist_m = 0.0f;
        RangeFinder *rangefinder = RangeFinder::get_singleton();
        if (rangefinder) {
            if (rangefinder->has_data_orient(ROTATION_NONE)) {
                raw_dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
            } else if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
                raw_dist_m = rangefinder->distance_cm_orient(ROTATION_PITCH_270) * 0.01f;
            }
        }
        sub.gcs().send_named_int("Raw_Dist_cm", (int32_t)(raw_dist_m * 100));"""

code = code.replace(old_log, new_log)

with open("ArduSub/mode_poshold_dist.cpp", "w") as f:
    f.write(code)

print("File mode_poshold_dist.cpp fixed")
