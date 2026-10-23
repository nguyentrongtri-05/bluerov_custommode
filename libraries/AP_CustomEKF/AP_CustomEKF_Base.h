#pragma once

#include <AP_Math/AP_Math.h>

/*
 * Base class for Custom 1D Distance/Velocity Kalman Filter (Ping + IMU/DVL)
 * Không dùng hàm ảo (virtual) để tiết kiệm vptr (RAM) và vtable (Flash) trên STM32F427.
 */
class AP_CustomEKF_Base {
public:
    AP_CustomEKF_Base();

    // Khởi tạo các trạng thái
    void init(float initial_distance, float initial_velocity);

    // Bước dự đoán (Predict) dựa trên gia tốc
    void predict(float accel_forward, float dt);

    // Cập nhật khoảng cách từ cảm biến Ping Altimeter (Update distance)
    void update_distance(float measured_distance);

    // Lấy giá trị ước lượng
    float get_distance() const { return _state[0]; }
    float get_velocity() const { return _state[1]; }

protected:
    // Cổng loại nhiễu cho mẫu Ping (đơn vị: độ lệch chuẩn của innovation).
    // Mẫu gần hơn dự đoán vượt cổng -> reset ngay về mẫu đo (ưu tiên an toàn).
    // Mẫu xa hơn dự đoán vượt cổng -> bỏ qua, sau INNOV_RESET_COUNT mẫu liên tiếp thì reset.
    static constexpr float INNOV_GATE_SIGMA = 5.0f;
    static constexpr uint8_t INNOV_RESET_COUNT = 5;

    // Các thông số nhiễu cố định đặt ở Flash (constexpr) thay vì tốn RAM trên mỗi instance
    static constexpr float _Q_accel = 0.5f; // Nhiễu hệ thống (Process noise)
    static constexpr float _R_ping  = 0.05f; // Nhiễu đo lường Ping altimeter
    static constexpr float _R_dvl   = 0.1f;  // Nhiễu đo lường DVL

    // Vector trạng thái: [0] = Khoảng cách phía trước (m), [1] = Vận tốc phía trước (m/s)
    float _state[2];

    // Ma trận hiệp phương sai đối xứng 2x2 chỉ lưu 3 phần tử độc lập (tiết kiệm RAM & phép tính)
    float _P00;
    float _P01;
    float _P11;

    uint8_t _reject_count; // số mẫu Ping liên tiếp bị cổng loại bỏ
};

