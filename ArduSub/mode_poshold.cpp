// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED

// poshold_init - initialise PosHold controller
bool ModePoshold::init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!sub.position_ok()) {
        return false;
    }

    // initialize vertical speeds and acceleration
    // All limits must be positive
    position_control->NE_set_max_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_set_correction_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->NE_init_controller_stopping_point();
    position_control->D_init_controller();

    // Stop all thrusters
    attitude_control->set_throttle_out(NEUTRAL_THROTTLE ,true, g.throttle_filt);
    attitude_control->relax_attitude_controllers();
    position_control->D_relax_controller(0.5f);

    sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
    _pilot_moving_xy = false;
    _last_fwd_out = 0.0f;
    _last_lat_out = 0.0f;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void ModePoshold::run()
{
    uint32_t tnow = AP_HAL::millis();
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE ,true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->NE_init_controller_stopping_point();
        position_control->D_relax_controller(0.5f);
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /////////////////////
    // Update attitude //

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

    // update attitude controller targets
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading_rad = ahrs.get_yaw_rad();
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw manoeuvres
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading_rad = ahrs.get_yaw_rad(); // update heading to hold

        } else { // call attitude controller holding absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, rad_to_cd(sub.last_pilot_heading_rad), true);
        }
    }

    // update z axis
    control_depth();

    // update xy axis
    // call this after Sub::get_pilot_desired_climb_rate is called so that THR_DZ is reasonable
    control_horizontal();
}

void ModePoshold::apply_pier_limits_and_slew(float &forward_out, float &lateral_out, bool pos_ctrl_active)
{
    // 1. Giới hạn bất đối xứng khi bật nút Custom 1 (Pier Mode):
    //    - Chiều Tiến vào trụ (forward_out > 0): khóa tối đa theo PHDS_FWD_MAX (mặc định 30%)
    //    - Chiều Ngang chống dòng chảy (lateral_out) & Chiều Lùi (forward_out < 0): giới hạn theo PHDS_LAT_MAX (mặc định 85%)
    if (sub.pier_limit_enabled) {
        const float fwd_max = constrain_float(g.phds_fwd_max.get(), 0.10f, 1.0f);
        const float lat_max = constrain_float(g.phds_lat_max.get(), 0.20f, 1.0f);
        const bool limited = (forward_out > fwd_max) || (forward_out < -lat_max) || (fabsf(lateral_out) > lat_max);
        forward_out = constrain_float(forward_out, -lat_max, fwd_max);
        lateral_out = constrain_float(lateral_out, -lat_max, lat_max);
        if (limited && pos_ctrl_active) {
            position_control->NE_set_externally_limited();
        }
    }

    // 2. Khởi động mềm & Dừng mềm (Thrust Slew-Rate Limiter) bảo vệ động cơ:
    const float slew_rate = g.phds_slew_rate.get();
    if (slew_rate > 0.0f && G_Dt > 0.0f) {
        const float max_delta = slew_rate * G_Dt;
        forward_out = constrain_float(forward_out, _last_fwd_out - max_delta, _last_fwd_out + max_delta);
        lateral_out = constrain_float(lateral_out, _last_lat_out - max_delta, _last_lat_out + max_delta);
    }
    _last_fwd_out = forward_out;
    _last_lat_out = lateral_out;
}

void ModePoshold::control_horizontal() {
    float lateral_out = 0;
    float forward_out = 0;

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };
    if (sub.pier_limit_enabled) {
        body_rates_cms *= constrain_float(g.phds_spd_scl.get(), 0.1f, 1.0f);
    }

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }

        // convert to the earth frame and set target rates
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        const bool pilot_moving = !body_rates_cms.is_zero();
        if (pilot_moving) {
            // Khi đang đẩy cần: giữ điểm đặt vị trí trùng với vị trí thực tế hiện tại
            // để điểm đặt không chạy vượt lên trước xuyên vào trụ cầu
            position_control->NE_stop_pos_stabilisation();
        } else if (_pilot_moving_xy) {
            // Ngay khoảnh khắc vừa nhả cần về 0: chốt điểm giữ vị trí tại chỗ và xóa vận tốc đặt
            position_control->NE_stop_pos_stabilisation();
            position_control->set_vel_desired_NE_ms({0.0f, 0.0f});
            position_control->set_accel_desired_NE_mss({0.0f, 0.0f});
        }
        _pilot_moving_xy = pilot_moving;

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // update the xy controller
        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        _pilot_moving_xy = false;
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

    apply_pier_limits_and_slew(forward_out, lateral_out, sub.position_ok());

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}
#endif  // POSHOLD_ENABLED
