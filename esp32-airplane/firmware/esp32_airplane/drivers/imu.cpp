#include "imu.h"
#include <math.h>

// ---------------------------------------------------------------------------
// STUB: emits a plausible stationary signal so the control stack can be
// exercised on the bench before the real IMU driver is written (C1).
// Replace the body of imu_read() with a real SPI/I2C transaction.
// ---------------------------------------------------------------------------
static bool s_imu_ok = false;

bool imu_init() {
    // TODO: SPI.begin(...); WHO_AM_I check; configure FS ranges from config.h
    s_imu_ok = true;
    return s_imu_ok;
}

bool imu_read(Vec3& accel_g, Vec3& gyro_rps) {
    if (!s_imu_ok) return false;
    // TODO: real transaction.
    // Synthetic: level aircraft, no rotation, 1 g on Z.
    accel_g  = {0.0f, 0.0f, 1.0f};
    gyro_rps = {0.0f, 0.0f, 0.0f};
    return true;
}

bool imu_calibrate_rest() {
    // TODO: average N samples at rest, store bias (persist to NVS per C1).
    return true;
}

bool imu_healthy() { return s_imu_ok; }
