// mixing.h — elevon mixing + adverse-yaw differential  (checklist D1/D2/D3)
#pragma once
#include <stdint.h>

// Normalised commands in [-1, +1]:
//   pitch_cmd > 0  => nose up
//   roll_cmd  > 0  => roll right
// Output: microseconds for each surface, already trimmed and clamped.
struct ElevonOut {
    uint16_t left_us;
    uint16_t right_us;
    uint16_t throttle_us;
};

// mix_elevons(): the function that replaces the quad's 4-motor mixer.
// Sign convention MUST be validated on the bench with props removed (D1).
ElevonOut mix_elevons(float pitch_cmd, float roll_cmd, float throttle_cmd);

// Utility: clamp helper used across modules.
float clampf(float v, float lo, float hi);

// Rate-limited slew toward `target` at `limit_dps_deg` per second.
float slew(float current, float target, float limit_per_sec, float dt);
