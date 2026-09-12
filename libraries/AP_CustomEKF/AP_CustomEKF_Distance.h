#pragma once

#include "AP_CustomEKF_Base.h"

/*
 * EKF Filter for fusing Ping Altimeter (Distance) and DVL (Velocity)
 * - DVL facing down, providing X/Y/Z velocities (we extract the forward velocity component)
 * - Ping Altimeter facing 90 degrees forward, providing distance to obstacle
 */
class AP_CustomEKF_Distance : public AP_CustomEKF_Base {
public:
    AP_CustomEKF_Distance();

    void init(float initial_distance, float initial_velocity) override;
    
    // Bước dự đoán dựa vào gia tốc tới (forward acceleration từ IMU)
    void predict(float accel_forward, float dt) override;
    
    // Cập nhật từ cảm biến Ping Altimeter (đo khoảng cách phía trước)
    void update_distance(float measured_distance) override;
    
    // Cập nhật từ cảm biến DVL (vận tốc di chuyển tới)
    void update_velocity(float measured_velocity) override;

    // Trả về giá trị khoảng cách tới vật cản
    float get_distance() const override { return _state[0]; }
    
    // Trả về giá trị vận tốc tới
    float get_velocity() const override { return _state[1]; }

private:
    // Vector trạng thái:
    // _state[0] = Khoảng cách phía trước (Distance)
    // _state[1] = Vận tốc phía trước (Velocity)
    float _state[2];

    // Ma trận hiệp phương sai sai số (Covariance matrix)
    float _P[2][2];

    // Nhiễu hệ thống (Process noise) - phụ thuộc vào IMU
    float _Q_accel;

    // Nhiễu đo lường (Measurement noise)
    float _R_ping; // Nhiễu Ping altimeter
    float _R_dvl;  // Nhiễu DVL

    // Hàm cập nhật ma trận KF chung
    void update_matrix(const float H[2], float R, float measurement, float predicted_measurement);
};

