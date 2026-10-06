import re

with open("ArduSub/Parameters.cpp", "r") as f:
    code = f.read()

old_param = """    // @Param: PHDS_USE_EKF
    // @DisplayName: PosHoldDist Filter Selection
    // @Description: Chọn bộ lọc KF hoặc EKF cho Mode 22. 0:KF (Chỉ Ping), 1:EKF (DVL+Ping)
    // @Values: 0:KF, 1:EKF
    // @User: Standard
    GSCALAR(phds_use_ekf, "PHDS_USE_EKF", 0),"""

new_param = """    // @Param: PHDS_USE_EKF
    // @DisplayName: PosHoldDist Filter Selection
    // @Description: Chọn bộ lọc khoảng cách Ping. 0: Raw (Gốc), 1: KF (Chỉ Ping), 2: EKF (DVL+Ping)
    // @Values: 0:Raw, 1:KF, 2:EKF
    // @User: Standard
    GSCALAR(phds_use_ekf, "PHDS_USE_EKF", 0),"""

code = code.replace(old_param, new_param)

with open("ArduSub/Parameters.cpp", "w") as f:
    f.write(code)

with open("ArduSub/mode_poshold_dist.cpp", "r") as f:
    code = f.read()

# Update run() logic
old_run_filter = """        // Lựa chọn bộ lọc để cập nhật
        if ((sub.g.phds_use_ekf.get() == 1)) {
            // Lựa chọn 2: Dùng EKF (DVL + Ping)
            _ekf.predict(accel_body.x, dt);
            if (has_ping) {
                _ekf.update_distance(dist_m);
            }
            
            Vector3f vel_ned;
            if (ahrs.get_velocity_NED(vel_ned)) {
                Vector2f vel_body2d = ahrs.earth_to_body2D(Vector2f(vel_ned.x, vel_ned.y));
                _ekf.update_velocity(vel_body2d.x);
            }
        } else {
            // Lựa chọn 1: Dùng KF (Chỉ Ping)
            _kf.predict(accel_body.x, dt);
            if (has_ping) {
                _kf.update_distance(dist_m);
            }
        }"""

new_run_filter = """        // Lựa chọn bộ lọc để cập nhật
        int filter_type = sub.g.phds_use_ekf.get();
        if (filter_type == 2) {
            // Lựa chọn 2: Dùng EKF (DVL + Ping)
            _ekf.predict(accel_body.x, dt);
            if (has_ping) {
                _ekf.update_distance(dist_m);
            }
            
            Vector3f vel_ned;
            if (ahrs.get_velocity_NED(vel_ned)) {
                Vector2f vel_body2d = ahrs.earth_to_body2D(Vector2f(vel_ned.x, vel_ned.y));
                _ekf.update_velocity(vel_body2d.x);
            }
        } else if (filter_type == 1) {
            // Lựa chọn 1: Dùng KF (Chỉ Ping)
            _kf.predict(accel_body.x, dt);
            if (has_ping) {
                _kf.update_distance(dist_m);
            }
        }
        // Nếu filter_type == 0 (Raw), không cần chạy bộ lọc KF/EKF"""

code = code.replace(old_run_filter, new_run_filter)

# Update control_horizontal() logic
old_ch = """void ModePosholdDist::control_horizontal()
{
    float lateral_out = 0;
    float forward_out = 0;

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    // --- Tránh vật cản ---
    // Lấy giá trị khoảng cách tùy thuộc vào lựa chọn của người dùng
    float current_dist_m;
    if ((sub.g.phds_use_ekf.get() == 1)) {
        current_dist_m = _ekf.get_distance(); // 2. Lấy từ EKF
    } else {
        current_dist_m = _kf.get_distance();  // 1. Lấy từ KF
    }"""

new_ch = """void ModePosholdDist::control_horizontal()
{
    float lateral_out = 0;
    float forward_out = 0;

    // Lấy khoảng cách thô trực tiếp từ Rangefinder
    float raw_dist_m = 0.0f;
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder) {
        if (rangefinder->has_data_orient(ROTATION_NONE)) {
            raw_dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
        } else if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            raw_dist_m = rangefinder->distance_cm_orient(ROTATION_PITCH_270) * 0.01f;
        }
    }

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    // --- Tránh vật cản ---
    // Lấy giá trị khoảng cách tùy thuộc vào lựa chọn của người dùng
    float current_dist_m = raw_dist_m; // Mặc định là 0 (Raw)
    int filter_type = sub.g.phds_use_ekf.get();
    
    if (filter_type == 2) {
        current_dist_m = _ekf.get_distance(); // 2. Lấy từ EKF
    } else if (filter_type == 1) {
        current_dist_m = _kf.get_distance();  // 1. Lấy từ KF
    }"""

code = code.replace(old_ch, new_ch)


# Remove the redundant raw reading in the logging part of control_horizontal
old_log = """        // sub.gcs().send_named_int("KF_Dist_cm", (int32_t)(current_dist_m * 100));
        float raw_dist_m = 0.0f;
        RangeFinder *rangefinder = RangeFinder::get_singleton();
        if (rangefinder) {
            if (rangefinder->has_data_orient(ROTATION_NONE)) {
                raw_dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
            } else if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
                raw_dist_m = rangefinder->distance_cm_orient(ROTATION_PITCH_270) * 0.01f;
            }
        }
        sub.gcs().send_named_int("Raw_Dist_cm", (int32_t)(raw_dist_m * 100));"""

new_log = """        // Gửi cả 2 giá trị lên QGC nếu cần, hiện tại chỉ gửi Raw để đồ thị không chập chờn
        // sub.gcs().send_named_int("KF_Dist_cm", (int32_t)(current_dist_m * 100));
        sub.gcs().send_named_int("Raw_Dist_cm", (int32_t)(raw_dist_m * 100));"""

code = code.replace(old_log, new_log)

with open("ArduSub/mode_poshold_dist.cpp", "w") as f:
    f.write(code)

print("Patch applied successfully")
