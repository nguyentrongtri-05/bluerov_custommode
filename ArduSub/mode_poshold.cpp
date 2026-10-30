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
    _pilot_moving_fwd = false;
    _pilot_moving_lat = false;
    _last_fwd_dir = 0;
    _last_lat_dir = 0;
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
    (void)forward_out;
    (void)lateral_out;
    (void)pos_ctrl_active;
}

void ModePoshold::update_pilot_xy_leash_and_release(const Vector2f &pilot_body_rates_cms)
{
    const bool pilot_fwd_moving = !is_zero(pilot_body_rates_cms.x);
    const bool pilot_lat_moving = !is_zero(pilot_body_rates_cms.y);
    const bool fwd_released = (!pilot_fwd_moving && _pilot_moving_fwd);
    const bool lat_released = (!pilot_lat_moving && _pilot_moving_lat);

    if (pilot_fwd_moving) {
        _last_fwd_dir = is_positive(pilot_body_rates_cms.x) ? 1 : -1;
    }
    if (pilot_lat_moving) {
        _last_lat_dir = is_positive(pilot_body_rates_cms.y) ? 1 : -1;
    }

    // Khi vừa nhả cần (fwd_released hoặc lat_released):
    // 1. Đưa vận tốc đặt (vel_desired) của trục vừa nhả về 0 tức thì để điểm đặt DP dừng tiến ngay
    //    (triệt tiêu hoàn toàn độ trễ giảm tốc 1-2s của bộ tạo quỹ đạo shape_vel_accel_xy).
    // 2. Giữ nguyên độ lệch vị trí (pos_lead = DP - P) trong phạm vi dây cương giữ vị trí (0.25m)
    //    để bảo toàn 100% lực P chống dòng chảy / lực kéo dây cáp như code gốc; chỉ kẹp phần
    //    pos_lead bị tích lũy dư vượt quá 0.25m theo đúng chiều vừa đẩy cần.
    if (fwd_released || lat_released) {
        static constexpr float RELEASE_HOLD_LEASH_M = 0.25f;
        const Vector2p pos_est_ne_m = position_control->get_pos_estimate_NED_m().xy();
        const Vector2f pos_lead_ne_m = (position_control->get_pos_desired_NED_m().xy() - pos_est_ne_m).tofloat();
        Vector2f pos_lead_body_m = ahrs.earth_to_body2D(pos_lead_ne_m);
        Vector2f vel_des_body_ms = ahrs.earth_to_body2D(position_control->get_vel_desired_NED_ms().xy());

        if (fwd_released) {
            vel_des_body_ms.x = 0.0f;
            if (_last_fwd_dir > 0 && pos_lead_body_m.x > RELEASE_HOLD_LEASH_M) {
                pos_lead_body_m.x = RELEASE_HOLD_LEASH_M;
            } else if (_last_fwd_dir < 0 && pos_lead_body_m.x < -RELEASE_HOLD_LEASH_M) {
                pos_lead_body_m.x = -RELEASE_HOLD_LEASH_M;
            }
            _last_fwd_dir = 0;
        }
        if (lat_released) {
            vel_des_body_ms.y = 0.0f;
            if (_last_lat_dir > 0 && pos_lead_body_m.y > RELEASE_HOLD_LEASH_M) {
                pos_lead_body_m.y = RELEASE_HOLD_LEASH_M;
            } else if (_last_lat_dir < 0 && pos_lead_body_m.y < -RELEASE_HOLD_LEASH_M) {
                pos_lead_body_m.y = -RELEASE_HOLD_LEASH_M;
            }
            _last_lat_dir = 0;
        }

        const Vector2f new_lead_ne_m = ahrs.body_to_earth2D(pos_lead_body_m);
        position_control->set_pos_desired_NE_m(pos_est_ne_m + new_lead_ne_m.topostype());
        position_control->set_vel_desired_NE_ms(ahrs.body_to_earth2D(vel_des_body_ms));
        position_control->set_accel_desired_NE_mss({0.0f, 0.0f});
    }

    _pilot_moving_fwd = pilot_fwd_moving;
    _pilot_moving_lat = pilot_lat_moving;
}

void ModePoshold::control_horizontal() {
    float lateral_out = 0;
    float forward_out = 0;

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }

        // convert to the earth frame and set target rates
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // Chốt vị trí tức thì khi nhả cần độc lập từng trục Tiến / Ngang (không giảm lực trục nào)
        update_pilot_xy_leash_and_release(body_rates_cms);

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // update the xy controller
        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        _pilot_moving_fwd = false;
        _pilot_moving_lat = false;
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}
#endif  // POSHOLD_ENABLED
