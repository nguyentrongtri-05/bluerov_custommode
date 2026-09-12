#include "AP_CustomKF_Ping.h"

AP_CustomKF_Ping::AP_CustomKF_Ping()
{
    _state[0] = 0.0f;
    _state[1] = 0.0f;
    
    _P[0][0] = 1.0f;
    _P[0][1] = 0.0f;
    _P[1][0] = 0.0f;
    _P[1][1] = 1.0f;

    // Nhiễu hệ thống Q khá lớn vì vận tốc có thể thay đổi đột ngột mà không có DVL bù trừ
    _Q_accel = 0.5f; 
    
    // Nhiễu đo lường R của riêng Ping Altimeter
    _R_ping = 0.8f; 
}

void AP_CustomKF_Ping::init(float initial_distance, float initial_velocity)
{
    _state[0] = initial_distance;
    _state[1] = initial_velocity;
}

void AP_CustomKF_Ping::predict(float accel_forward, float dt)
{
    // Cập nhật trạng thái
    // Nếu di chuyển tới (v > 0), khoảng cách tới vật cản sẽ giảm
    _state[0] = _state[0] - _state[1] * dt - 0.5f * accel_forward * dt * dt;
    _state[1] = _state[1] + accel_forward * dt;

    // Tính ma trận Jacobian F:
    float F00 = 1.0f;
    float F01 = -dt;
    float F10 = 0.0f;
    float F11 = 1.0f;

    float P00 = _P[0][0];
    float P01 = _P[0][1];
    float P10 = _P[1][0];
    float P11 = _P[1][1];

    // F * P
    float FP00 = F00 * P00 + F01 * P10;
    float FP01 = F00 * P01 + F01 * P11;
    float FP10 = F10 * P00 + F11 * P10;
    float FP11 = F10 * P01 + F11 * P11;

    // P = (F * P) * F^T + Q
    _P[0][0] = FP00 * F00 + FP01 * F01 + 0.25f * _Q_accel * dt * dt * dt * dt;
    _P[0][1] = FP00 * F10 + FP01 * F11 - 0.5f * _Q_accel * dt * dt * dt;
    _P[1][0] = FP10 * F00 + FP11 * F01 - 0.5f * _Q_accel * dt * dt * dt;
    _P[1][1] = FP10 * F10 + FP11 * F11 + _Q_accel * dt * dt;
}

void AP_CustomKF_Ping::update_distance(float measured_distance)
{
    // Ma trận H cho đo lường khoảng cách là [1, 0]
    float H[2] = {1.0f, 0.0f};
    
    // Tính S = H * P * H^T + R
    float PH0 = _P[0][0] * H[0] + _P[0][1] * H[1];
    float PH1 = _P[1][0] * H[0] + _P[1][1] * H[1];
    
    float S = H[0] * PH0 + H[1] * PH1 + _R_ping;
    
    if (is_zero(S)) {
        return; // Tránh chia cho 0
    }
    
    // Tính Kalman Gain: K = P * H^T * S^-1
    float K[2];
    K[0] = PH0 / S;
    K[1] = PH1 / S;
    
    // Sai số đo lường
    float y = measured_distance - _state[0];
    
    // Cập nhật trạng thái
    _state[0] += K[0] * y;
    _state[1] += K[1] * y;
    
    // Cập nhật hiệp phương sai
    float I_KH[2][2];
    I_KH[0][0] = 1.0f - K[0] * H[0];
    I_KH[0][1] = -K[0] * H[1];
    I_KH[1][0] = -K[1] * H[0];
    I_KH[1][1] = 1.0f - K[1] * H[1];
    
    float P00 = _P[0][0];
    float P01 = _P[0][1];
    float P10 = _P[1][0];
    float P11 = _P[1][1];
    
    _P[0][0] = I_KH[0][0] * P00 + I_KH[0][1] * P10;
    _P[0][1] = I_KH[0][0] * P01 + I_KH[0][1] * P11;
    _P[1][0] = I_KH[1][0] * P00 + I_KH[1][1] * P10;
    _P[1][1] = I_KH[1][0] * P01 + I_KH[1][1] * P11;
}

