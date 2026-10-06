#pragma once

#include <AP_Math/AP_Math.h>

/*
 * Base class for Custom EKF
 */
class AP_CustomEKF_Base {
public:
    AP_CustomEKF_Base() {}
    virtual ~AP_CustomEKF_Base() {}

    // Khởi tạo các trạng thái
    virtual void init(float initial_distance, float initial_velocity) = 0;

    // Bước dự đoán (Predict) dựa trên gia tốc
    virtual void predict(float accel_forward, float dt) = 0;

    // Cập nhật khoảng cách (Update distance)
    virtual void update_distance(float measured_distance) = 0;

    // Cập nhật vận tốc (Update velocity)
    virtual void update_velocity(float measured_velocity) = 0;

    // Lấy giá trị ước lượng
    virtual float get_distance() const = 0;
    virtual float get_velocity() const = 0;

protected:
    // Cổng loại nhiễu cho mẫu Ping (đơn vị: độ lệch chuẩn của innovation).
    // Mẫu gần hơn dự đoán vượt cổng -> reset ngay về mẫu đo (ưu tiên an toàn).
    // Mẫu xa hơn dự đoán vượt cổng -> bỏ qua, sau INNOV_RESET_COUNT mẫu liên tiếp thì reset.
    static constexpr float INNOV_GATE_SIGMA = 5.0f;
    static constexpr uint8_t INNOV_RESET_COUNT = 5;
};
