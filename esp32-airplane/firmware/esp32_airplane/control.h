// control.h — flight control loops + envelope protection (checklist F1/F2/F3)
#pragma once
#include "ahrs.h"

// Pilot / navigation inputs, all normalised or SI.
struct ControlCmd {
    float pitch;      // [-1,1]  nose-up positive (from stick or nav)
    float roll;       // [-1,1]  roll-right positive
    float throttle;   // [0,1]
    float phi_cmd;    // rad, desired bank   (from guidance / pilot)
    float h_cmd;      // m,   desired altitude (RTH / nav)
    float v_cmd;      // m/s, desired airspeed (RTH / nav)
    bool  nav_active; // true => use phi_cmd/h_cmd/v_cmd instead of sticks
};

// Observable control state for telemetry/logging (I2, I3).
struct ControlDebug {
    float roll_rate_meas, pitch_rate_meas;   // rad/s
    float roll_rate_cmd,  pitch_rate_cmd;    // rad/s
    float phi_meas, phi_cmd_l;               // rad
    float throttle_out;                      // [0,1]
    float b0_roll, b0_pitch;                 // scheduled plant gains
    float f_roll, f_pitch;                   // ESO total-disturbance estimate
    float airspeed_est;                      // m/s
    float ctrl_pitch_out, ctrl_roll_out;     // final normalised surface cmds
    bool  env_stall, env_bank, env_vne, env_g;   // envelope flags
};

void control_init();

// Called at DT_RATE_HZ. Produces normalised pitch/roll/throttle commands.
void control_update(const Attitude& att,
                    const ControlCmd& cmd,
                    float airspeed_mps,
                    float dt,
                    ControlDebug& dbg);

// --- Envelope protection (F3): pure functions, testable on host -----------
struct EnvelopeLimits {
    float phi_max_rad;
    float v_min, v_ne;
    float climb_max, sink_max;
    float n_max_pos, n_max_neg;
};

// Clamp a bank command to +/- phi_max.
float env_clamp_bank(float phi_cmd, const EnvelopeLimits& lim, bool& hit);

// True if airspeed is below V_min => caller must force nose down + full throttle.
bool  env_stall(float v, const EnvelopeLimits& lim);

// Estimated load factor from bank (level turn): n = 1/cos(phi). Used when no accel.
float env_load_factor(float phi_rad);
