// drivers/imu.h — IMU driver stub (checklist C1)
// TODO: implement SPI/I2C read for the chosen part (ICM-20602 / MPU6050).
#pragma once
#include "../ahrs.h"

bool imu_init();
// Returns true on success. accel in g, gyro in rad/s.
bool imu_read(Vec3& accel_g, Vec3& gyro_rps);
// Capture gyro bias at rest (C1 calibration). Blocks ~0.5 s.
bool imu_calibrate_rest();
// True if the last read was fresh (not stalled on the bus).
bool imu_healthy();
