#pragma once

#include "AP_CustomEKF_Base.h"

/*
 * 1D Kalman Filter (KF) for smoothing Ping Altimeter values.
 * - Predicts distance and velocity using IMU (or constant velocity if accel is 0).
 * - Updates states solely using distance measurements from the Ping Altimeter.
 */
class AP_CustomKF_Ping : public AP_CustomEKF_Base {
public:
    AP_CustomKF_Ping();

    void init(float initial_distance, float initial_velocity) override;
    
    // Bước dự đoán (Predict)
    void predict(float accel_forward, float dt) override;
    
    // Cập nhật từ Ping Altimeter (Update)
    void update_distance(float measured_distance) override;
    
    // Hàm này không dùng đến vì chỉ dùng Ping, nhưng phải override từ lớp ảo
    void update_velocity(float measured_velocity) override {}

    // Lấy giá trị sau khi đã lọc nhiễu
    float get_distance() const override { return _state[0]; }
    float get_velocity() const override { return _state[1]; }

private:
    // Vector trạng thái: [0] = Khoảng cách, [1] = Vận tốc
    float _state[2];

    // Ma trận hiệp phương sai
    float _P[2][2];

    float _Q_accel; // Nhiễu hệ thống (process noise)
    float _R_ping;  // Nhiễu đo lường của riêng cảm biến Ping
};

