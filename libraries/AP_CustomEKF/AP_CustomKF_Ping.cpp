#include "AP_CustomKF_Ping.h"

AP_CustomEKF_Base::AP_CustomEKF_Base() :
    _state{0.0f, 0.0f},
    _P00(1.0f),
    _P01(0.0f),
    _P11(1.0f),
    _reject_count(0)
{
}

void AP_CustomEKF_Base::init(float initial_distance, float initial_velocity)
{
    _state[0] = initial_distance;
    _state[1] = initial_velocity;

    // Reset hiệp phương sai: khoảng cách lấy từ mẫu Ping, vận tốc chưa chắc chắn
    _P00 = _R_ping;
    _P01 = 0.0f;
    _P11 = 1.0f;

    _reject_count = 0;
}

void AP_CustomEKF_Base::predict(float accel_forward, float dt)
{
    // Cập nhật trạng thái
    // Nếu di chuyển tới (v > 0), khoảng cách tới vật cản sẽ giảm
    _state[0] = _state[0] - _state[1] * dt - 0.5f * accel_forward * dt * dt;
    _state[1] = _state[1] + accel_forward * dt;

    // Ma trận P đối xứng mới: P = F * P * F^T + Q với F = [1, -dt; 0, 1]
    const float dt2 = dt * dt;
    const float FP00 = _P00 - dt * _P01;
    const float FP01 = _P01 - dt * _P11;

    _P00 = FP00 - dt * FP01 + 0.25f * _Q_accel * dt2 * dt2;
    _P01 = FP01 - 0.5f * _Q_accel * dt2 * dt;
    _P11 = _P11 + _Q_accel * dt2;
}

void AP_CustomEKF_Base::update_distance(float measured_distance)
{
    // Đo lường khoảng cách với H = [1, 0] -> S = P00 + R_ping
    const float S = _P00 + _R_ping;
    if (is_zero(S)) {
        return; // Tránh chia cho 0
    }

    // Sai số đo lường
    const float y = measured_distance - _state[0];

    // Cổng loại nhiễu (xem AP_CustomEKF_Base)
    if (y * y > INNOV_GATE_SIGMA * INNOV_GATE_SIGMA * S) {
        if (y < 0.0f || ++_reject_count >= INNOV_RESET_COUNT) {
            init(measured_distance, _state[1]);
        }
        return;
    }
    _reject_count = 0;

    // Kalman Gain: K = P * H^T / S = [P00 / S, P01 / S]^T
    const float K0 = _P00 / S;
    const float K1 = _P01 / S;

    // Cập nhật trạng thái: x = x + K * y
    _state[0] += K0 * y;
    _state[1] += K1 * y;

    // Cập nhật hiệp phương sai đối xứng: P = (I - K * H) * P
    const float P01_old = _P01;
    _P00 *= (1.0f - K0);
    _P01 *= (1.0f - K0);
    _P11 -= K1 * P01_old;
}


