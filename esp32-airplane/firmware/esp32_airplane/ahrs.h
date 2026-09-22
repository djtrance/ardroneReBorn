// ahrs.h — attitude estimation: gyro + accel + magnetometer fusion (checklist C8)
#pragma once

struct Vec3 { float x, y, z; };

struct Attitude {
    float roll;    // rad
    float pitch;   // rad
    float yaw;     // rad (0 = magnetic north at boot)
    float gx, gy, gz;   // bias-corrected body rates (rad/s)
    bool  mag_ok;       // heading trustworthy
};

// Call once at boot (gyro bias from rest, mag calibration assumed loaded).
void ahrs_init();

// Feed one IMU sample (accel in g, gyro in rad/s) + mag (uT, optional).
// dt in seconds. Returns fused attitude.
Attitude ahrs_update(const Vec3& accel_g, const Vec3& gyro_rps,
                     const Vec3& mag_ut, bool mag_valid, float dt);

// Returns true once the filter has settled (checklist C8 convergence gate).
bool ahrs_converged();
