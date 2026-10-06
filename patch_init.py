import re

with open("ArduSub/mode_poshold_dist.cpp", "r") as f:
    code = f.read()

old_init = """bool ModePosholdDist::init(bool ignore_checks)
{
    if (!ModePoshold::init(ignore_checks)) {
        return false;
    }

    // Initialize both filters
    _ekf.init(0.0f, 0.0f);
    _kf.init(0.0f, 0.0f);
    _last_ekf_update_ms = AP_HAL::millis();

    return true;
}"""

new_init = """bool ModePosholdDist::init(bool ignore_checks)
{
    if (!ModePoshold::init(ignore_checks)) {
        return false;
    }

    // Lấy khoảng cách thô hiện tại ngay khi vừa chuyển Mode để làm giá trị khởi tạo
    float initial_dist_m = 0.0f;
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder) {
        if (rangefinder->has_data_orient(ROTATION_NONE)) {
            initial_dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
        } else if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            initial_dist_m = rangefinder->distance_cm_orient(ROTATION_PITCH_270) * 0.01f;
        }
    }

    // Khởi tạo bộ lọc với khoảng cách thực tế (thay vì 0.0) để không bị kẹt phanh lúc mới bật
    _ekf.init(initial_dist_m, 0.0f);
    _kf.init(initial_dist_m, 0.0f);
    _last_ekf_update_ms = AP_HAL::millis();

    return true;
}"""

code = code.replace(old_init, new_init)

with open("ArduSub/mode_poshold_dist.cpp", "w") as f:
    f.write(code)

print("Init patched successfully")
