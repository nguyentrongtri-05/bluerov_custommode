// ArduSub position hold flight mode with distance tracking
// Inherits from PosHold and uses EKF distance to avoid obstacles

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

    // Initialize EKF
    _ekf.init(0.0f, 0.0f);
    _last_ekf_update_ms = AP_HAL::millis();

    return true;
}

void ModePosholdDist::run()
{
    // Update EKF
    uint32_t tnow = AP_HAL::millis();
    if (_last_ekf_update_ms == 0) {
        _last_ekf_update_ms = tnow;
    }
    
    float dt = (tnow - _last_ekf_update_ms) * 0.001f;
    if (dt > 0.01f && dt < 1.0f) { // Only update if dt is reasonable
        // Lấy gia tốc thân (forward accel)
        const AP_InertialSensor &ins = AP::ins();
        Vector3f accel_body = ins.get_accel(); 
        
        // Bước Predict
        _ekf.predict(accel_body.x, dt);
        
        // Cập nhật từ Ping Altimeter
        RangeFinder *rangefinder = RangeFinder::get_singleton();
        if (rangefinder && rangefinder->has_data_orient(ROTATION_NONE)) {
            float dist_m = rangefinder->distance_cm_orient(ROTATION_NONE) * 0.01f;
            _ekf.update_distance(dist_m);
        }

        // Cập nhật từ DVL (Vận tốc NED -> Body)
        Vector3f vel_ned;
        if (ahrs.get_velocity_NED(vel_ned)) {
            Vector2f vel_body2d = ahrs.earth_to_body2D(Vector2f(vel_ned.x, vel_ned.y));
            _ekf.update_velocity(vel_body2d.x);
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

    // --- Tránh vật cản sử dụng khoảng cách từ EKF ---
    float current_dist_m = _ekf.get_distance();
    
    // Nếu khoảng cách <= 0.5m và đang cố di chuyển tới, chặn tốc độ tới
    if (current_dist_m <= 0.5f && body_rates_cms.x > 0) {
        body_rates_cms.x = 0;
    } 
    // Giảm tốc độ dần nếu khoảng cách từ 1.0m đến 0.5m
    else if (current_dist_m < 1.0f && body_rates_cms.x > 0) {
        float allowed_ratio = (current_dist_m - 0.5f) / 0.5f; // Từ 0.0 đến 1.0
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

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}

#endif // POSHOLD_ENABLED
