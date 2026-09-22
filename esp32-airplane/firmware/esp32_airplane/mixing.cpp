#include "mixing.h"
#include "config.h"

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float slew(float current, float target, float limit_per_sec, float dt) {
    float max_delta = limit_per_sec * dt;
    float d = target - current;
    if (d >  max_delta) d =  max_delta;
    if (d < -max_delta) d = -max_delta;
    return current + d;
}

// Map a normalised [-1,1] command to microseconds around a trim value.
static inline uint16_t to_us(float cmd, uint16_t trim_us, uint16_t span_us) {
    long us = (long)trim_us + (long)((cmd * 0.5f) * (float)(2 * span_us));
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return (uint16_t)us;
}

ElevonOut mix_elevons(float pitch_cmd, float roll_cmd, float throttle_cmd) {
    pitch_cmd = clampf(pitch_cmd, -1.0f, 1.0f);
    roll_cmd  = clampf(roll_cmd,  -1.0f, 1.0f);

    // --- D2: adverse-yaw differential -------------------------------------
    // No rudder on a flying wing: rolling creates adverse yaw. Differential
    // elevon deflection (up-going surface deflects more) counters it.
    float diff = ELEVON_DIFFERENTIAL * (roll_cmd < 0.0f ? -roll_cmd : roll_cmd);

    // --- D1: elevon mixing -------------------------------------------------
    // Standard flying-wing convention:
    //   left  = pitch - roll
    //   right = pitch + roll
    // NOTE: validate these signs on the bench (checklist D1) — inverted
    //       signs are the classic first-flight killer.
    float left_cmd  = pitch_cmd - roll_cmd + diff;
    float right_cmd = pitch_cmd + roll_cmd - diff;

    ElevonOut out;
    out.left_us      = to_us(left_cmd,  ELEVON_L_TRIM_US, ELEVON_PITCH_MAX_US);
    out.right_us     = to_us(right_cmd, ELEVON_R_TRIM_US, ELEVON_PITCH_MAX_US);
    out.throttle_us  = (uint16_t)clampf(throttle_cmd * (float)(ESC_MAX_US - ESC_MIN_US) + ESC_MIN_US,
                                        ESC_MIN_US, ESC_MAX_US);
    return out;
}
