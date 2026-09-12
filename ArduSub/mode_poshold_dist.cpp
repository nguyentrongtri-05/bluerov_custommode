// ArduSub position hold flight mode with distance tracking
// Inherits from PosHold and uses EKF/KF distance to avoid obstacles

#include "Sub.h"

#if POSHOLD_ENABLED

ModePosholdDist::ModePosholdDist()
{
    _last_ekf_update_ms = 0;
}

bool ModePosholdDist::init(bool ignore_checks)
{
    if (!ModePoshold::init(ignore_checks)) {
        return false;
    }

    // Initialize both filters
    _ekf.init(0.0f, 0.0f);
    _kf.init(0.0f, 0.0f);
    _last_ekf_update_ms = AP_HAL::millis();

    return true;
}

void ModePosholdDist::run()
{
    uint32_t tnow = AP_HAL::millis();
    if (_last_ekf_update_ms == 0) {
        _last_ekf_update_ms = tnow;
    }
    
    float dt = (tnow - _last_ekf_update_ms) * 0.001f;
    if (dt > 0.01f && dt < 1.0f) { // Only update if dt is reasonable
        // Lấy gia tốc thân (forward accel)
        const AP_InertialSensor &ins = AP::ins();
        Vector3f accel_body = ins.get_accel(); 
        
        // Lấy dữ liệu Ping Altimeter
        float dist_m = 0.0f;
        bool has_ping = false;
        RangeFinder *rangefinder = RangeFinder::get_singleton();
        if (rangefinder && rangefinder->has_data_orient(ROTATION_NONE)) {
            dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
            has_ping = true;
        }

        // Lựa chọn bộ lọc để cập nhật
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
        }
        
        _last_ekf_update_ms = tnow;
    }

    // Run original poshold mode run logic
    ModePoshold::run();
}

void ModePosholdDist::control_horizontal()
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
    }
    
    float min_dist_m = sub.g.phds_dist_min.get();
    float max_dist_m = min_dist_m + 0.3f; // Vùng giảm tốc bắt đầu trước 50cm so với mức min

    // Giữ khoảng cách an toàn
    if (current_dist_m <= min_dist_m && body_rates_cms.x > 0) {
        body_rates_cms.x = 0; // Chặn hoàn toàn tốc độ tới
    } 
    // Giảm tốc độ dần trong khoảng 50cm trước khi chạm mức min
    else if (current_dist_m < max_dist_m && body_rates_cms.x > 0) {
        float allowed_ratio = (current_dist_m - min_dist_m) / 0.3f; // Từ 0.0 đến 1.0
        float max_speed_cms = g.pilot_speed * allowed_ratio;
        if (body_rates_cms.x > max_speed_cms) {
            body_rates_cms.x = max_speed_cms;
        }
    }
    // --------------------------------------------------

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }

        // convert to the earth frame and set target rates
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // update the xy controller
        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

        // --- Gửi dữ liệu khoảng cách lên QGroundControl ---
    static uint32_t last_log_ms = 0;
    uint32_t now = AP_HAL::millis();
    if (now - last_log_ms > 200) { // Gửi với tần số 5Hz (200ms/lần) để tránh nghẽn mạng
        sub.gcs().send_named_int("KF_Dist_cm", (int32_t)(current_dist_m * 100));
        last_log_ms = now;
    }

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}

#endif // POSHOLD_ENABLED
