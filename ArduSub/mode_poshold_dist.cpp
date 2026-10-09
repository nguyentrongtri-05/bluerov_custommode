// ArduSub position hold flight mode with distance tracking
// Inherits from PosHold and limits forward motion using the forward Ping (RNGFND1).
// Distance source selected by PHDS_USE_EKF: 0 Raw, 1 KF (Ping only), 2 EKF (Ping + DVL velocity)

#include "Sub.h"
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#if POSHOLD_ENABLED

// bộ lọc coi là mất Ping nếu không có mẫu mới trong khoảng này
#define PHDS_PING_TIMEOUT_MS 500

ModePosholdDist::ModePosholdDist() :
    _last_filter_us(0),
    _last_ping_reading_ms(0),
    _last_fused_ms(0),
    _last_send_ms(0),
    _last_log_ms(0),
    _stall_start_ms(0),
    _contact(false)
{
}

bool ModePosholdDist::get_raw_distance(float &dist_m, uint32_t &reading_ms) const
{
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        return false;
    }
    // RNGFND1 = instance 0
    const AP_RangeFinder_Backend *backend = rangefinder->get_backend(0);
    if (backend == nullptr || !backend->has_data()) {
        return false;
    }
    // cùng quy tắc chất lượng tín hiệu với RNGFND_SQ_MIN của ArduSub (-1 = không rõ)
    const int8_t quality = backend->signal_quality_pct();
    if (quality != -1 && quality < g.rangefinder_signal_min) {
        return false;
    }
    dist_m = backend->distance();
    reading_ms = backend->last_reading_ms();
    return true;
}

bool ModePosholdDist::filters_valid(uint32_t now_ms) const
{
    return _last_fused_ms != 0 && (now_ms - _last_fused_ms) < PHDS_PING_TIMEOUT_MS;
}

bool ModePosholdDist::init(bool ignore_checks)
{
    if (!ModePoshold::init(ignore_checks)) {
        return false;
    }

    // bộ lọc sẽ được khởi tạo từ mẫu Ping mới đầu tiên trong update_filters()
    _last_fused_ms = 0;
    _last_ping_reading_ms = 0;
    _last_filter_us = AP_HAL::micros();

    _contact = false;
    _stall_start_ms = 0;

    return true;
}

void ModePosholdDist::update_filters()
{
    const uint32_t now_us = AP_HAL::micros();
    const float dt = (now_us - _last_filter_us) * 1.0e-6f;
    _last_filter_us = now_us;

    // gia tốc tới theo phương ngang: accel_ef chỉ chứa trọng trường ở trục z,
    // nên lấy thành phần ngang rồi xoay theo yaw sẽ không bị lẫn trọng trường khi tàu nghiêng
    const float accel_fwd = ahrs.earth_to_body2D(ahrs.get_accel_ef().xy()).x;

    // vận tốc tới từ AHRS (EKF3 đã fuse DVL)
    Vector3f vel_ned;
    const bool have_vel = ahrs.get_velocity_NED(vel_ned);
    const float vel_fwd = have_vel ? ahrs.earth_to_body2D(vel_ned.xy()).x : 0.0f;

    if (dt > 0.0f && dt < 0.5f) {
        _kf.predict(accel_fwd, dt);
        _ekf.predict(accel_fwd, dt);
        if (have_vel) {
            _ekf.update_velocity(vel_fwd);
        }
    }

    // chỉ update khi có mẫu Ping mới, tránh fuse lặp lại cùng một mẫu ở 400Hz
    float dist_m;
    uint32_t reading_ms;
    if (!get_raw_distance(dist_m, reading_ms) || reading_ms == _last_ping_reading_ms) {
        return;
    }
    _last_ping_reading_ms = reading_ms;

    const uint32_t now_ms = AP_HAL::millis();
    if (!filters_valid(now_ms)) {
        // lần đầu hoặc vừa mất Ping: khởi tạo lại từ mẫu đo
        _kf.init(dist_m, vel_fwd);
        _ekf.init(dist_m, vel_fwd);
    } else {
        _kf.update_distance(dist_m);
        _ekf.update_distance(dist_m);
    }
    _last_fused_ms = now_ms;
}

void ModePosholdDist::run()
{
    update_filters();

    // Run original poshold mode run logic
    ModePoshold::run();
}

void ModePosholdDist::control_horizontal()
{
    float lateral_out = 0;
    float forward_out = 0;

    float raw_dist_m = 0.0f;
    uint32_t reading_ms;
    const bool raw_ok = get_raw_distance(raw_dist_m, reading_ms);
    const bool filt_ok = filters_valid(AP_HAL::millis());

    // Chọn nguồn khoảng cách theo PHDS_USE_EKF. 0 = không hợp lệ -> chặn tiến ở bên dưới
    const uint8_t filter_type = sub.g.phds_use_ekf.get();
    float current_dist_m = 0.0f;
    switch (filter_type) {
    case 1:
        current_dist_m = filt_ok ? _kf.get_distance() : 0.0f;
        break;
    case 2:
        current_dist_m = filt_ok ? _ekf.get_distance() : 0.0f;
        break;
    default:
        current_dist_m = raw_ok ? raw_dist_m : 0.0f;
        break;
    }

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    // --- Tránh vật cản ---
    
    float min_dist_m = sub.g.phds_dist_min.get();
    float max_dist_m = min_dist_m + 0.3f; // Vùng giảm tốc bắt đầu trước 30cm so với mức min

    // --- Giữ khoảng cách và Tự động lùi ---
    // Chỉ kích hoạt tự lùi nếu cảm biến Ping trả về giá trị hợp lệ (> 10cm)
    // Đề phòng trường hợp Ping bị tuột dây (trả về 0) làm tàu lùi điên cuồng vô tận.
    const bool contact_mode = (sub.g.phds_action.get() == 1);
    if (contact_mode) {
        // Chế độ áp sát (PHDS_ACTION=1): không giới hạn theo Ping, xử lý ở phần "Áp sát" bên dưới
    } else if (current_dist_m > 0.1f) {
        if (current_dist_m < min_dist_m) {
            // Tàu lún vào vùng cấm -> Ép tàu bơi lùi
            float error_m = min_dist_m - current_dist_m; // Xâm nhập bao nhiêu mét
            float auto_reverse_speed_cms = -error_m * 100.0f; // Kp = 100 (Ví dụ: lún 0.1m -> lùi 10cm/s)
            
            // Giới hạn tốc độ lùi tối đa (Max là 50% tốc độ bay thông thường)
            if (auto_reverse_speed_cms < -g.pilot_speed * 0.5f) {
                auto_reverse_speed_cms = -g.pilot_speed * 0.5f;
            }
            
            // Ghi đè tay ga: Dù phi công đẩy tới hay buông tay, tàu vẫn phải lùi
            if (body_rates_cms.x > auto_reverse_speed_cms) {
                body_rates_cms.x = auto_reverse_speed_cms;
            }
        } 
        // Vùng đệm giảm tốc (từ min_dist đến min_dist+30cm)
        else if (current_dist_m < max_dist_m && body_rates_cms.x > 0) {
            float allowed_ratio = (current_dist_m - min_dist_m) / 0.3f;
            float max_speed_cms = g.pilot_speed * allowed_ratio;
            if (body_rates_cms.x > max_speed_cms) {
                body_rates_cms.x = max_speed_cms;
            }
        }
    } else {
        // Tín hiệu Ping < 10cm (có thể do lỗi cáp, Ping chết hoặc quá sát vách). 
        // Tạm thời chặn tay ga tiến (an toàn tuyệt đối), nhưng cho phép phi công lùi tay.
        if (body_rates_cms.x > 0) {
            body_rates_cms.x = 0;
        }
    }
    // --------------------------------------------------

    // --- Áp sát: thoát khi phi công kéo cần lùi (hoặc tắt PHDS_ACTION) ---
    if (_contact && (!contact_mode || body_rates_cms.x < 0.0f)) {
        _contact = false;
        _stall_start_ms = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PHDS: contact released");
    }

    if (_contact) {
        // Đang tì vào bề mặt: KHÔNG gọi position controller để nó chuyển sang inactive,
        // bỏ lực tích lũy (khâu I). Khi thoát, nhánh bên dưới sẽ khởi tạo lại từ đầu.
        // Chỉ đẩy tới một lực nhỏ cố định, trục ngang điều khiển tay.
        forward_out = constrain_float(sub.g.phds_push.get(), 0.0f, 0.3f);
        lateral_out = (g.pilot_speed > 0) ? body_rates_cms.y / (float)g.pilot_speed : 0.0f;
    } else if (sub.position_ok()) {
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

    // --- Áp sát: phát hiện bị bề mặt chặn lại ---
    // Phi công đẩy tới rõ ràng + bộ điều khiển đang ra lực tới + vận tốc tới (DVL qua EKF3) gần 0
    // liên tục trong PHDS_STALL_T giây -> xác nhận đã chạm
    if (contact_mode && !_contact) {
        const float stall_vel = sub.g.phds_stall_vel.get();
        const bool pushing = (body_rates_cms.x * 0.01f > 2.0f * stall_vel) && (forward_out > 0.0f);
        Vector3f vel_ned;
        if (sub.position_ok() && pushing && ahrs.get_velocity_NED(vel_ned) &&
            ahrs.earth_to_body2D(vel_ned.xy()).x < stall_vel) {
            const uint32_t stall_now_ms = AP_HAL::millis();
            if (_stall_start_ms == 0) {
                _stall_start_ms = stall_now_ms;
            } else if (stall_now_ms - _stall_start_ms >= (uint32_t)(sub.g.phds_stall_t.get() * 1000.0f)) {
                _contact = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PHDS: contact, push %.0f%%",
                              (double)(constrain_float(sub.g.phds_push.get(), 0.0f, 0.3f) * 100.0f));
            }
        } else {
            _stall_start_ms = 0;
        }
    } else if (!contact_mode) {
        _stall_start_ms = 0;
    }

    // --- Gửi cả 3 giá trị lên QGC (5Hz) để so sánh. -1 = không hợp lệ ---
    // tên NAMED_VALUE_INT tối đa 10 ký tự
    const uint32_t now = AP_HAL::millis();
    if (now - _last_send_ms > 200) {
        sub.gcs().send_named_int("PHDS_Raw", raw_ok ? (int32_t)(raw_dist_m * 100) : -1);
        sub.gcs().send_named_int("PHDS_KF", filt_ok ? (int32_t)(_kf.get_distance() * 100) : -1);
        sub.gcs().send_named_int("PHDS_EKF", filt_ok ? (int32_t)(_ekf.get_distance() * 100) : -1);
        _last_send_ms = now;
    }

#if HAL_LOGGING_ENABLED
    // --- Ghi log PHDS (10Hz) để phân tích sau bằng MAVExplorer ---
    if (now - _last_log_ms >= 100) {
        AP::logger().Write("PHDS", "TimeUS,Sel,Raw,KF,EKF,KFv,EKFv,RawOk,FiltOk,Cont", "QBfffffBBB",
                           AP_HAL::micros64(),
                           filter_type,
                           raw_dist_m,
                           _kf.get_distance(),
                           _ekf.get_distance(),
                           _kf.get_velocity(),
                           _ekf.get_velocity(),
                           (uint8_t)raw_ok,
                           (uint8_t)filt_ok,
                           (uint8_t)_contact);
        _last_log_ms = now;
    }
#endif

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}

#endif // POSHOLD_ENABLED
