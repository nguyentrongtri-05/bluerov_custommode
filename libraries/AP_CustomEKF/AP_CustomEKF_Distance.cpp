#include "AP_CustomEKF_Distance.h"

void AP_CustomEKF_Distance::update_velocity(float measured_velocity)
{
    // DVL cung cấp giá trị vận tốc tới với H = [0, 1] -> S = P11 + R_dvl
    const float S = _P11 + _R_dvl;
    if (is_zero(S)) {
        return; // Tránh chia cho 0
    }

    // Sai số đo lường: y = z - h(x)
    const float y = measured_velocity - _state[1];

    // Kalman Gain: K = P * H^T / S = [P01 / S, P11 / S]^T
    const float K0 = _P01 / S;
    const float K1 = _P11 / S;

    // Cập nhật trạng thái: x = x + K * y
    _state[0] += K0 * y;
    _state[1] += K1 * y;

    // Cập nhật hiệp phương sai đối xứng: P = (I - K * H) * P
    const float P01_old = _P01;
    _P00 -= K0 * P01_old;
    _P01 *= (1.0f - K1);
    _P11 *= (1.0f - K1);
}


