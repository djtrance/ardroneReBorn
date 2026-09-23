#include "mixing.h"
#include "config.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Active mix parameters. Defaults come from config.h so the mixer works with
// no persisted settings at all; wifi_config pushes Settings.mix in here after
// every save (I5) and at boot.
// ---------------------------------------------------------------------------
static MixSettings s_mix = {
    (uint16_t)ELEVON_L_TRIM_US,
    (uint16_t)ELEVON_R_TRIM_US,
    (uint16_t)ELEVON_PITCH_MAX_US,
    (uint16_t)ELEVON_ROLL_MAX_US,
    0,                          // elevon_l_reverse
    0,                          // elevon_r_reverse
    0,                          // throttle_reverse
    1,                          // passthrough (etapa 1 default, same as NVS)
    ELEVON_DIFFERENTIAL,
    (uint16_t)ESC_MIN_US,
    (uint16_t)ESC_MAX_US,
    (uint16_t)ESC_ARM_US,
};

void mixing_load(const MixSettings* m) {
    if (m) s_mix = *m;
}

const MixSettings& mixing_settings() { return s_mix; }

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

// Map a normalised command to microseconds around a trim value, clamped to the
// configured deflection span FIRST (so `span` is a real mechanical limit) and
// only then to the electrical servo range.
static inline uint16_t to_us(float cmd, uint16_t trim_us, uint16_t span_us,
                             bool reverse) {
    float us = (float)trim_us + cmd * (float)span_us;
    if (reverse) us = 2.0f * (float)trim_us - us;   // invert about the neutral
    if (us < (float)SERVO_MIN_US) us = (float)SERVO_MIN_US;
    if (us > (float)SERVO_MAX_US) us = (float)SERVO_MAX_US;
    return (uint16_t)(us + 0.5f);
}

ElevonOut mix_elevons(float pitch_cmd, float roll_cmd, float throttle_cmd) {
    pitch_cmd = clampf(pitch_cmd, -1.0f, 1.0f);
    roll_cmd  = clampf(roll_cmd,  -1.0f, 1.0f);

    // --- D2: adverse-yaw differential -------------------------------------
    // No rudder on a flying wing: rolling creates adverse yaw. Differential
    // elevon deflection (up-going surface deflects more) counters it.
    float diff = s_mix.differential * fabsf(roll_cmd);

    // --- D1: elevon mixing -------------------------------------------------
    // Standard flying-wing convention:
    //   left  = pitch - roll
    //   right = pitch + roll
    // NOTE: validate these signs on the bench (checklist D1) — inverted
    //       signs are the classic first-flight killer.
    float left_cmd  = pitch_cmd - roll_cmd + diff;
    float right_cmd = pitch_cmd + roll_cmd - diff;

    // Each surface may saturate independently; clamping here (instead of on
    // the summed command) is what keeps full-deflection roll from also
    // driving the wing to full pitch.
    ElevonOut out;
    out.left_us  = to_us(clampf(left_cmd,  -1.0f, 1.0f),
                         s_mix.elevon_l_trim_us, s_mix.pitch_span_us,
                         s_mix.elevon_l_reverse != 0);
    out.right_us = to_us(clampf(right_cmd, -1.0f, 1.0f),
                         s_mix.elevon_r_trim_us, s_mix.pitch_span_us,
                         s_mix.elevon_r_reverse != 0);

    float thr = clampf(throttle_cmd, 0.0f, 1.0f);
    float lo  = (float)(s_mix.throttle_reverse ? s_mix.esc_max_us
                                               : s_mix.esc_min_us);
    float hi  = (float)(s_mix.throttle_reverse ? s_mix.esc_min_us
                                               : s_mix.esc_max_us);
    if (thr < 0.0f) thr = 0.0f;
    if (thr > 1.0f) thr = 1.0f;
    float tus = lo + thr * (hi - lo);
    out.throttle_us = (uint16_t)(tus + 0.5f);
    return out;
}
