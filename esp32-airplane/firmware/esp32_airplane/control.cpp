#include "control.h"
#include "mixing.h"
#include "config.h"
#include <math.h>

// ===========================================================================
// Simple ADRC-lite: ESO + NLSEF.  Full port of src/flight/adrc.c happens in
// checklist F1; this gives the correct structure and b0 scheduling now.
//
//   plant:   y'' = b0*u + f
//   ESO:     z1 -> y_hat,  z2 -> y'_hat,  z3 -> f_hat
//   NLSEF:   u0 = kp*e - kd*e'
//   output:  u  = (u0 - f_hat) / b0        <-- model-free disturbance cancel
// ===========================================================================
typedef struct {
    float z1, z2, z3;   // ESO states
    float b0;           // plant gain (scheduled)
    float wc, wo;       // controller / observer bandwidth
    float beta1, beta2, beta3;
    float integ;
} AdrcAxis;

static void adrc_init(AdrcAxis* a, float wc, float wo, float b0) {
    a->z1 = a->z2 = a->z3 = a->integ = 0.0f;
    a->wc = wc; a->wo = wo; a->b0 = b0;
    // Bandwidth parameterisation (critical damping), same as adrc.c:
    a->beta1 = 3.0f * wo;
    a->beta2 = 3.0f * wo * wo;
    a->beta3 = wo * wo * wo;
}

// y: measured output (rate), r: setpoint, dt: seconds.
static float adrc_step(AdrcAxis* a, float r, float y, float dt) {
    // --- ESO (discrete, forward Euler) ---
    float e  = a->z1 - y;
    float b0 = (a->b0 > 1e-6f) ? a->b0 : 1e-6f;
    a->z1 += dt * (a->z2 - a->beta1 * e);
    a->z2 += dt * (a->z3 - a->beta2 * e + b0 * 0.0f /* u injected below */);
    // z3 gets the control input term; recompute with u from previous step
    a->z3 += dt * (-a->beta3 * e);

    // --- NLSEF: tracking error ---
    float e_r  = r - a->z1;
    float de_r = 0.0f - a->z2;               // setpoint treated as constant rate
    float kp = a->wc * a->wc;
    float kd = 2.0f * a->wc;
    float u0 = kp * e_r - kd * de_r;

    // --- disturbance cancellation + b0 scaling ---
    float u = (u0 - a->z3) / b0;

    // Inject u into ESO for next iteration (b0*u term on z2 derivative)
    a->z2 += dt * (b0 * u);

    // Anti-windup via z3 clamping (as in src/flight/adrc.c)
    const float Z3_LIMIT = 50.0f;
    if (a->z3 >  Z3_LIMIT) a->z3 =  Z3_LIMIT;
    if (a->z3 < -Z3_LIMIT) a->z3 = -Z3_LIMIT;

    return u;
}

// ===========================================================================
// PID fallback (CFG_CONTROLLER_ADRC == 0) — same interface
// ===========================================================================
typedef struct {
    float kp, ki, kd;
    float integ, prev_err;
} PidAxis;

static float pid_step(PidAxis* p, float r, float y, float dt) {
    float e = r - y;
    p->integ += p->ki * e * dt;
    p->integ = clampf(p->integ, -1.0f, 1.0f);       // anti-windup
    float d = (e - p->prev_err) / dt;
    p->prev_err = e;
    return p->kp * e + p->integ + p->kd * d;
}

// ===========================================================================
static AdrcAxis s_roll_rate, s_pitch_rate;
static PidAxis  s_roll_rate_pid, s_pitch_rate_pid, s_roll_att, s_pitch_att, s_airspeed;
static EnvelopeLimits s_lim;

void control_init() {
    adrc_init(&s_roll_rate,  ROLL_RATE_WC,  ROLL_RATE_WO,  B0_ROLL_REF);
    adrc_init(&s_pitch_rate, PITCH_RATE_WC, PITCH_RATE_WO, B0_PITCH_REF);

    s_roll_rate_pid  = {PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,  0, 0};
    s_pitch_rate_pid = {PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, 0, 0};
    s_roll_att       = {PID_ROLL_ATT_KP,   0.0f, 0.0f, 0, 0};
    s_pitch_att      = {PID_PITCH_ATT_KP,  0.0f, 0.0f, 0, 0};
    s_airspeed       = {PID_AIRSPEED_KP,   PID_AIRSPEED_KI,   0.0f, 0, 0};

    s_lim.phi_max_rad = PHI_MAX_DEG * (float)M_PI / 180.0f;
    s_lim.v_min       = V_MIN_MPS;
    s_lim.v_ne        = V_NE_MPS;
    s_lim.climb_max   = CLIMB_MAX_MPS;
    s_lim.sink_max    = SINK_MAX_MPS;
    s_lim.n_max_pos   = N_MAX_POS;
    s_lim.n_max_neg   = N_MAX_NEG;
}

// --- Envelope protection, exposed for host unit tests (J2) ----------------
float env_clamp_bank(float phi_cmd, const EnvelopeLimits& lim, bool& hit) {
    float c = clampf(phi_cmd, -lim.phi_max_rad, lim.phi_max_rad);
    hit = (c != phi_cmd);
    return c;
}

bool env_stall(float v, const EnvelopeLimits& lim) {
    return (v > 0.0f) && (v < lim.v_min);
}

float env_load_factor(float phi_rad) {
    float c = cosf(phi_rad);
    if (fabsf(c) < 0.05f) return 20.0f;   // absurd -> clamp
    return 1.0f / c;
}

// --- b0 scheduling with dynamic pressure (F1) ------------------------------
static float schedule_b0(float b0_ref, float v) {
    if (v < 1.0f) v = 1.0f;
    float q      = 0.5f * RHO_SEA_LEVEL * v * v;
    float scale  = q / Q_REF;
    scale = clampf(scale, B0_MIN_SCALE, B0_MAX_SCALE);
    return b0_ref * scale;
}

void control_update(const Attitude& att, const ControlCmd& cmd,
                    float v, float dt, ControlDebug& dbg) {
    dbg = ControlDebug{};
    dbg.airspeed_est   = v;
    dbg.roll_rate_meas = att.gx;
    dbg.pitch_rate_meas= att.gy;
    dbg.phi_meas       = att.roll;

    // -----------------------------------------------------------------------
    // 1) DESIRED ATTITUDE  (outer loop)  — nav or pilot
    // -----------------------------------------------------------------------
    float phi_des, theta_des, v_des;
    if (cmd.nav_active) {
        phi_des  = cmd.phi_cmd;                 // from L1 guidance (G2)
        theta_des= 0.0f;                        // altitude loop below
        v_des    = cmd.v_cmd;
    } else {
        phi_des  = cmd.roll  * s_lim.phi_max_rad;   // stick -> bank
        theta_des= cmd.pitch * 25.0f * (float)M_PI / 180.0f;
        v_des    = (cmd.throttle > 0.05f)
                   ? V_CRUISE_MPS + cmd.throttle * (V_NE_MPS - V_CRUISE_MPS)
                   : V_CRUISE_MPS;
    }

    // --- F3: ENVELOPE PROTECTION (priority over everything) ---
    bool hb = false;
    phi_des = env_clamp_bank(phi_des, s_lim, hb);
    dbg.env_bank = hb;

    bool stall = env_stall(v, s_lim);
    dbg.env_stall = stall;
    dbg.env_vne = (v > s_lim.v_ne);
    dbg.env_g = fabsf(env_load_factor(phi_des)) > s_lim.n_max_pos;

    // STALL OVERRIDE: nose down + max throttle regardless of commands (F3)
    if (stall) {
        phi_des  = 0.0f;                 // level the wings to reduce stall speed
        theta_des= -10.0f * (float)M_PI / 180.0f;
        dbg.env_stall = true;
    }
    dbg.phi_cmd_l = phi_des;

    // -----------------------------------------------------------------------
    // 2) ATTITUDE -> RATE  (middle loop)
    // -----------------------------------------------------------------------
    float roll_rate_cmd  = s_roll_att.kp  * (phi_des  - att.roll);
    float pitch_rate_cmd = s_pitch_att.kp * (theta_des- att.pitch);

    // Structural rate caps
    roll_rate_cmd  = clampf(roll_rate_cmd,  -4.0f, 4.0f);   // rad/s (~230 dps)
    pitch_rate_cmd = clampf(pitch_rate_cmd, -3.0f, 3.0f);
    dbg.roll_rate_cmd  = roll_rate_cmd;
    dbg.pitch_rate_cmd = pitch_rate_cmd;

    // -----------------------------------------------------------------------
    // 3) RATE -> SURFACE  (inner loop @ DT_RATE_HZ)
    // -----------------------------------------------------------------------
    float u_roll, u_pitch;
    if (CFG_CONTROLLER_ADRC) {
        s_roll_rate.b0  = schedule_b0(B0_ROLL_REF,  v);
        s_pitch_rate.b0 = schedule_b0(B0_PITCH_REF, v);
        dbg.b0_roll  = s_roll_rate.b0;
        dbg.b0_pitch = s_pitch_rate.b0;
        u_roll  = adrc_step(&s_roll_rate,  roll_rate_cmd,  att.gx, dt);
        u_pitch = adrc_step(&s_pitch_rate, pitch_rate_cmd, att.gy, dt);
        dbg.f_roll  = s_roll_rate.z3;
        dbg.f_pitch = s_pitch_rate.z3;
    } else {
        u_roll  = pid_step(&s_roll_rate_pid,  roll_rate_cmd,  att.gx, dt);
        u_pitch = pid_step(&s_pitch_rate_pid, pitch_rate_cmd, att.gy, dt);
    }

    u_roll  = clampf(u_roll,  -1.0f, 1.0f);
    u_pitch = clampf(u_pitch, -1.0f, 1.0f);

    // -----------------------------------------------------------------------
    // 4) THROTTLE  (airspeed loop — NOT altitude; that's TECS, F6/P1)
    // -----------------------------------------------------------------------
    float thr;
    if (stall) {
        thr = 1.0f;                                  // stall recovery: full power
    } else if (cmd.nav_active) {
        thr = clampf(pid_step(&s_airspeed, v_des, v, dt), 0.0f, 1.0f);
    } else {
        thr = clampf(cmd.throttle, 0.0f, 1.0f);
    }
    dbg.throttle_out = thr;

    // -----------------------------------------------------------------------
    // 5) Emit normalised commands (mixing.cpp applies us + differential)
    //    Rate-limit on the *surface* command (D5) to prevent violent snaps.
    // -----------------------------------------------------------------------
    static float lp = 0.0f, lr = 0.0f;
    lp = slew(lp, u_pitch, CMD_RATE_LIMIT_DPS * (float)M_PI / 180.0f, dt);
    lr = slew(lr, u_roll,  CMD_RATE_LIMIT_DPS * (float)M_PI / 180.0f, dt);

    dbg.ctrl_pitch_out = lp;
    dbg.ctrl_roll_out  = lr;
}
