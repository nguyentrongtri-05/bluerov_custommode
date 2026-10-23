#pragma once

#include "AP_CustomEKF_Base.h"

/*
 * 1D Kalman Filter (KF) for smoothing Ping Altimeter values.
 * - Predicts distance and velocity using IMU (or constant velocity if accel is 0).
 * - Updates states solely using distance measurements from the Ping Altimeter.
 */
class AP_CustomKF_Ping : public AP_CustomEKF_Base {
public:
    using AP_CustomEKF_Base::AP_CustomEKF_Base;
};


