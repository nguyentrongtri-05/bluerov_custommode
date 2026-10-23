#pragma once

#include "AP_CustomEKF_Base.h"

/*
 * EKF Filter for fusing Ping Altimeter (Distance) and DVL (Velocity)
 * - DVL facing down, providing X/Y/Z velocities (we extract the forward velocity component)
 * - Ping Altimeter facing 90 degrees forward, providing distance to obstacle
 */
class AP_CustomEKF_Distance : public AP_CustomEKF_Base {
public:
    using AP_CustomEKF_Base::AP_CustomEKF_Base;

    // Cập nhật từ cảm biến DVL (vận tốc di chuyển tới)
    void update_velocity(float measured_velocity);
};


