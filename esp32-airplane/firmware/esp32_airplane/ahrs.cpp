#include "ahrs.h"
#include "config.h"
#include <math.h>

static Attitude s_att;
static bool     s_converged = false;
static float    s_settle_t  = 0.0f;

// Gyro bias captured at rest (C1 calibration). TODO: replace with real
// 6-face accel + gyro-at-rest calibration stored in NVS.
static Vec3     s_gyro_bias = {0, 0, 0};

void ahrs_init() {
    s_att.roll = s_att.pitch = s_att.yaw = 0.0f;
    s_att.gx = s_att.gy = s_att.gz = 0.0f;
    s_att.mag_ok = false;
    s_converged = false;
    s_settle_t  = 0.0f;
}

static inline float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

Attitude ahrs_update(const Vec3& a_g, const Vec3& g_rps,
                     const Vec3& mag, bool mag_valid, float dt) {
    // ---------------------------------------------------------------------
    // Bias-corrected body rates
    // ---------------------------------------------------------------------
    float gx = g_rps.x - s_gyro_bias.x;
    float gy = g_rps.y - s_gyro_bias.y;
    float gz = g_rps.z - s_gyro_bias.z;
    s_att.gx = gx; s_att.gy = gy; s_att.gz = gz;

    // ---------------------------------------------------------------------
    // Accel-only pitch/roll reference (gravity in body frame)
    // ---------------------------------------------------------------------
    float ax = a_g.x, ay = a_g.y, az = a_g.z;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    bool  accel_ok = (norm > 0.75f && norm < 1.25f);   // reject during manoeuvre/vibration

    float roll_acc  = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

    // ---------------------------------------------------------------------
    // Complementary filter on roll/pitch
    //   high weight on gyro (fast, drifts), low on accel (absolute, noisy)
    // ---------------------------------------------------------------------
    s_att.roll  = AHRS_ALPHA      * (s_att.roll  + gx * dt) + (1.0f - AHRS_ALPHA)      * roll_acc;
    s_att.pitch = AHRS_ALPHA      * (s_att.pitch + gy * dt) + (1.0f - AHRS_ALPHA)      * pitch_acc;
    s_att.roll  = wrap_pi(s_att.roll);
    s_att.pitch = wrap_pi(s_att.pitch);

    // ---------------------------------------------------------------------
    // Yaw: gyro integration + magnetometer correction (tilt-compensated)
    // ---------------------------------------------------------------------
    float yaw_gyro = wrap_pi(s_att.yaw + gz * dt);
    float yaw_mag  = s_att.yaw;

    if (mag_valid && CFG_USE_MAG) {
        float cr = cosf(s_att.roll),  sr = sinf(s_att.roll);
        float cp = cosf(s_att.pitch), sp = sinf(s_att.pitch);
        // Tilt-compensate body mag into horizontal plane
        float mx =  mag.x * cp + mag.z * sp;
        float my =  mag.x * sr * sp + mag.y * cr - mag.z * sr * cp;
        if (fabsf(mx) > 1e-3f || fabsf(my) > 1e-3f) {
            yaw_mag = wrap_pi(-atan2f(my, mx));   // sign depends on mag orientation (C3)
            s_att.mag_ok = true;
        }
    } else {
        s_att.mag_ok = false;
    }

    // Blend gyro yaw with mag heading when mag is healthy
    if (s_att.mag_ok) {
        float err = wrap_pi(yaw_mag - yaw_gyro);
        s_att.yaw = wrap_pi(yaw_gyro + (1.0f - AHRS_YAW_ALPHA) * err);
    } else {
        s_att.yaw = yaw_gyro;
    }

    // ---------------------------------------------------------------------
    // Convergence gate — block arming until settled (C8 / H6)
    // ---------------------------------------------------------------------
    if (!s_converged) {
        s_settle_t += dt;
        if (s_settle_t > 1.5f && accel_ok) s_converged = true;
    }
    return s_att;
}

bool ahrs_converged() { return s_converged; }
