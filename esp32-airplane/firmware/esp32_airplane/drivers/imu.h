// drivers/imu.h — accelerometer + gyroscope (checklist C1)
// Public interface is board-agnostic; the part is chosen at compile time in
// config.h (-DIMU_GY91 / -DIMU_GY87) and implemented by drivers/imu_*.cpp.
#pragma once
#include "../ahrs.h"

bool imu_init();
// Returns true on success. accel in g, gyro in rad/s (bias removed).
bool imu_read(Vec3& accel_g, Vec3& gyro_rps);
// Capture gyro bias at rest (C1 calibration). Blocks ~0.8 s.
bool imu_calibrate_rest();
// True if the last read was fresh (not stalled on the bus).
bool imu_healthy();
