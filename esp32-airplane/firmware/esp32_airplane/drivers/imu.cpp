#include "imu.h"
#include "imu_board.h"
#include "../config.h"

// ---------------------------------------------------------------------------
// The public IMU API is just a thin adapter over whichever board was selected
// at compile time (config.h: -DIMU_GY91 or -DIMU_GY87). Keeping this indirection
// means ahrs/control/guidance never see the part number.
// ---------------------------------------------------------------------------
bool imu_init() { return board_imu_init(); }

bool imu_read(Vec3& accel_g, Vec3& gyro_rps) {
    return board_imu_read(accel_g, gyro_rps);
}

bool imu_calibrate_rest() { return board_imu_calibrate_rest(); }

bool imu_healthy() { return board_imu_healthy(); }
