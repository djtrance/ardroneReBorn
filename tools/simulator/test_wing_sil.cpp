// test_wing_sil.cpp — checklist J1: 6-DOF wing SIL + F1/F2/F3 validation.
//
// Three layers:
//   A) plant sanity: trim is an exact fixed point, free flight is stable
//   B) aero + identification: stability derivatives, stall break, and the
//      b0 system ID that config.h B0_*_REF (F1 "TODO identify in SIL") must
//      match within 15%
//   C) closed loop over the REAL firmware control.cpp (mixing.cpp linked for
//      clampf/slew): attitude tracking (F2), envelope priority + stall
//      recovery + bank clamp + slew limit (F3), airspeed loop, fuzz
//
// Run: make test_wing_sil_run   (tools/simulator)
#include "wing_plant.h"
#include "control.h"
#include "config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;
#define ck(cond, ...) do {                                   \
        if (cond) { ++g_pass; }                              \
        else { ++g_fail; printf("FAIL(L%d): ", __LINE__);    \
               printf(__VA_ARGS__); printf("\n"); }          \
    } while (0)

// --- harness ---------------------------------------------------------------
static WingParams   g_prm;
static WingState    g_st;
static ControlCmd   g_cmd;
static ControlDebug g_dbg;
static const double g_wind0[3] = { 0.0, 0.0, 0.0 };
static const double DT = 1.0 / (double)DT_RATE_HZ;   // firmware rate-loop cadence

static double airspeed() {
    WingAirdata ad;
    wing_airdata(g_st, g_prm, g_wind0, &ad);
    return ad.V;
}

static bool finite_state() {
    double v[12] = { g_st.pn, g_st.pe, g_st.pd, g_st.u, g_st.v, g_st.w,
                     g_st.phi, g_st.theta, g_st.psi, g_st.p, g_st.q, g_st.r };
    for (int i = 0; i < 12; ++i)
        if (!isfinite(v[i])) return false;
    return isfinite(g_dbg.ctrl_pitch_out) && isfinite(g_dbg.ctrl_roll_out);
}

static double trim_throttle(double V) {
    double a, t;
    wing_trim_solve(g_prm, V, &a, &t);
    return t;
}

static void sil_reset(double V, double alt) {
    wing_default_params(&g_prm);
    wing_trim_state(&g_st, g_prm, V, alt);
    control_init();
    memset(&g_cmd, 0, sizeof g_cmd);
    memset(&g_dbg, 0, sizeof g_dbg);
    g_cmd.throttle = (float)trim_throttle(V);
}

static void sil_step() {
    WingAirdata ad;
    wing_airdata(g_st, g_prm, g_wind0, &ad);
    Attitude att;
    memset(&att, 0, sizeof att);
    att.roll = (float)g_st.phi;
    att.pitch = (float)g_st.theta;
    att.yaw = (float)g_st.psi;
    att.gx = (float)g_st.p;
    att.gy = (float)g_st.q;
    att.gz = (float)g_st.r;
    att.mag_ok = true;                     // perfect sensing for J1 (J1b: noise)
    control_update(att, g_cmd, (float)ad.V, (float)DT, g_dbg);

    WingInput in;
    in.pitch = g_dbg.ctrl_pitch_out;        // controller intent -> plant signs
    in.roll = g_dbg.ctrl_roll_out;
    in.throttle = g_dbg.throttle_out;
    wing_step(&g_st, in, g_prm, g_wind0, DT);
}

static void sil_run(double sec) {
    long n = (long)(sec / DT + 0.5);
    while (n-- > 0) sil_step();
}

static void sil_run_open(double sec) {      // plant only, fixed trim input
    WingInput in = { 0.0, 0.0, g_cmd.throttle };
    long n = (long)(sec / DT + 0.5);
    while (n-- > 0) wing_step(&g_st, in, g_prm, g_wind0, DT);
}

static double measure_b0(bool pitch_axis, double V) {
    sil_reset(V, 50.0);
    WingInput i0 = { 0.0, 0.0, g_cmd.throttle };
    WingInput i1 = i0;
    if (pitch_axis) i1.pitch = 1.0; else i1.roll = 1.0;
    double dx0[W_N], dx1[W_N];
    wing_deriv(g_st, i0, g_prm, g_wind0, dx0);
    wing_deriv(g_st, i1, g_prm, g_wind0, dx1);
    int idx = pitch_axis ? W_Q : W_P;
    return dx1[idx] - dx0[idx];            // cancels any static-moment bias
}

static WingState state_at_alpha(double a) {
    WingState s;
    memset(&s, 0, sizeof s);
    s.pd = -50.0;
    s.u = 15.0 * cos(a);
    s.w = 15.0 * sin(a);
    s.theta = a;
    return s;
}

static WingAirdata air_of(const WingState& s) {
    WingAirdata ad;
    wing_airdata(s, g_prm, g_wind0, &ad);
    return ad;
}

int main() {
    const double rad2deg = 180.0 / M_PI;

    // ======================================================================
    // A) params + trim + hands-off stability
    // ======================================================================
    sil_reset(15.0, 50.0);
    ck(g_prm.c > 0.2 && g_prm.c < 0.4, "chord %.3f m plausible for S/b", g_prm.c);
    ck(fabs(g_prm.c - g_prm.S / g_prm.b) < 1e-12, "c == S/b (AR = %.1f)",
       g_prm.b / g_prm.c);
    ck(g_prm.Ixx > 0 && g_prm.Iyy > g_prm.Ixx && g_prm.Izz > g_prm.Ixx,
       "inertia ordering Ixx < Iyy/Izz");
    ck(g_prm.alpha_stall > 0.15 && g_prm.alpha_stall < 0.35,
       "stall alpha %.2f rad", g_prm.alpha_stall);
    ck(g_prm.delta_max > 0.2 && g_prm.delta_max < 0.6,
       "surface travel %.2f rad at full cmd", g_prm.delta_max);
    ck(g_prm.thrust_max / (g_prm.mass * 9.81) > 1.5,
       "static thrust-to-weight %.1f", g_prm.thrust_max / (g_prm.mass * 9.81));

    double alpha, thr;
    wing_trim_solve(g_prm, 15.0, &alpha, &thr);
    ck(alpha > 0.02 && alpha < 0.06, "trim alpha %.4f rad (%.1f deg)",
       alpha, alpha * rad2deg);
    ck(thr > 0.15 && thr < 0.5, "trim throttle %.2f", thr);

    WingAirdata ad = air_of(g_st);
    ck(fabs(ad.V - 15.0) < 1e-9, "trim state airspeed exact (%.6f)", ad.V);
    ck(fabs(ad.alpha - alpha) < 1e-12, "state alpha == solved trim alpha");
    ck(ad.CL > 0.18 && ad.CL < 0.20, "trim CL %.4f == W/qS", ad.CL);

    double dx[W_N];
    WingInput in0 = { 0.0, 0.0, thr };
    wing_deriv(g_st, in0, g_prm, g_wind0, dx);
    double lin = sqrt(dx[W_U] * dx[W_U] + dx[W_V] * dx[W_V] +
                      dx[W_W] * dx[W_W]);
    ck(lin < 0.03, "trim linear accel %.4f m/s^2 ~ 0 (exact L=W-T sin a solve)",
       lin);
    ck(fabs(dx[W_P]) < 0.05 && fabs(dx[W_Q]) < 0.05 && fabs(dx[W_R]) < 0.05,
       "trim angular accel ~ 0 (p %.4f q %.4f r %.4f)",
       dx[W_P], dx[W_Q], dx[W_R]);
    ck(fabs(dx[W_PD]) < 1e-9, "trim flight path horizontal (pdot %.2e)",
       dx[W_PD]);

    sil_run_open(5.0);
    double Vs = airspeed();
    ck(Vs > 13.5 && Vs < 16.5, "5 s hands-off holds speed (%.2f m/s)", Vs);
    double alt = -g_st.pd;
    ck(alt > 47.0 && alt < 53.0, "5 s hands-off altitude drift < 3 m (%.2f)",
       alt);
    ck(fabs(g_st.phi) < 0.05, "hands-off wings level (%.4f rad)", g_st.phi);
    ck(fabs(g_st.theta) < 0.12, "hands-off pitch near trim (%.4f rad)",
       g_st.theta);
    ck(finite_state(), "hands-off run finite");

    // ======================================================================
    // B) stability derivatives, control effectiveness, stall, b0 ID
    // ======================================================================
    sil_reset(15.0, 50.0);
    double a0 = atan2(g_st.w, g_st.u);
    WingState sp = g_st;                    // nose up, same flight path
    sp.u = 15.0 * cos(a0 + 0.15);
    sp.w = 15.0 * sin(a0 + 0.15);
    sp.theta = a0 + 0.15;
    wing_deriv(sp, in0, g_prm, g_wind0, dx);
    ck(dx[W_Q] < -1.0, "static pitch stability: alpha+0.15 -> qdot %.1f < 0",
       dx[W_Q]);

    sil_reset(15.0, 50.0);                  // control signs = controller intent
    WingInput in_roll = { 0.0, 1.0, g_cmd.throttle };
    wing_deriv(g_st, in_roll, g_prm, g_wind0, dx);
    ck(dx[W_P] > 20.0, "roll cmd +1 -> rolls right (pdot %.1f > 0)", dx[W_P]);
    ck(dx[W_R] < 0.0, "adverse yaw: roll cmd +1 yaws left (rdot %.1f)",
       dx[W_R]);
    WingInput in_pitch = { 1.0, 0.0, g_cmd.throttle };
    wing_deriv(g_st, in_pitch, g_prm, g_wind0, dx);
    ck(dx[W_Q] > 20.0, "pitch cmd +1 -> nose up (qdot %.1f > 0)", dx[W_Q]);

    WingState ss = g_st;                    // slip right (wind from the right)
    ss.v = 2.0;
    wing_deriv(ss, in0, g_prm, g_wind0, dx);
    ck(dx[W_P] < -5.0, "dihedral effect: slip right -> roll left (pdot %.1f)",
       dx[W_P]);
    ck(dx[W_R] > 1.0, "weathervane: slip right -> yaw right (rdot %.1f)",
       dx[W_R]);

    WingAirdata a05 = air_of(state_at_alpha(0.05));
    WingAirdata a20 = air_of(state_at_alpha(0.20));
    WingAirdata a26 = air_of(state_at_alpha(0.26));
    WingAirdata a28 = air_of(state_at_alpha(0.28));
    WingAirdata a40 = air_of(state_at_alpha(0.40));
    ck(a20.CL < a26.CL, "CL climbs to the stall break (%.3f < %.3f)",
       a20.CL, a26.CL);
    ck(a28.CL < a26.CL, "CL declines immediately past stall (%.3f < %.3f)",
       a28.CL, a26.CL);
    ck(a40.CL < a26.CL, "CL well down at 23 deg (%.3f < %.3f)", a40.CL,
       a26.CL);
    ck(a40.CD > 2.0 * a05.CD,
       "post-stall drag jumps (%.3f vs %.3f at 3 deg)", a40.CD, a05.CD);
    ck(a20.CD < a40.CD, "polar: deeper alpha costs drag (%.3f < %.3f)",
       a20.CD, a40.CD);

    // --- J1 identification: plant b0 vs config B0_*_REF (F1) --------------
    double b0r15 = measure_b0(false, 15.0);
    double b0p15 = measure_b0(true, 15.0);
    ck(fabs(b0r15 - wing_b0_roll(g_prm, 15.0)) < 1e-6 * wing_b0_roll(g_prm, 15.0),
       "deriv b0 == closed form (roll)");
    ck(fabs(b0p15 - wing_b0_pitch(g_prm, 15.0)) < 1e-6 * wing_b0_pitch(g_prm, 15.0),
       "deriv b0 == closed form (pitch)");
    ck(b0r15 > 60.0 && b0r15 < 120.0,
       "identified b0_roll %.1f rad/s^2 in physical band", b0r15);
    ck(b0p15 > 55.0 && b0p15 < 95.0,
       "identified b0_pitch %.1f rad/s^2 in physical band", b0p15);
    ck(fabs((double)B0_ROLL_REF - b0r15) <= 0.15 * b0r15,
       "config B0_ROLL_REF=%.0f within 15%% of identified %.1f (F1)",
       (double)B0_ROLL_REF, b0r15);
    ck(fabs((double)B0_PITCH_REF - b0p15) <= 0.15 * b0p15,
       "config B0_PITCH_REF=%.0f within 15%% of identified %.1f (F1)",
       (double)B0_PITCH_REF, b0p15);

    double b0r20 = measure_b0(false, 20.0);
    double ratio = b0r20 / b0r15;
    ck(fabs(ratio - (20.0 * 20.0) / (15.0 * 15.0)) < 1e-6,
       "plant b0 scales with dynamic pressure (ratio %.4f)", ratio);
    double q20 = 0.5 * (double)RHO_SEA_LEVEL * 20.0 * 20.0;
    double scale20 = q20 / (double)Q_REF;
    if (scale20 > (double)B0_MAX_SCALE) scale20 = (double)B0_MAX_SCALE;
    if (scale20 < (double)B0_MIN_SCALE) scale20 = (double)B0_MIN_SCALE;
    double sched20 = (double)B0_ROLL_REF * scale20;
    ck(fabs(sched20 - b0r20) <= 0.15 * b0r20,
       "config schedule_b0 at 20 m/s matches plant (%.1f vs %.1f)",
       sched20, b0r20);

    // ======================================================================
    // C) closed loop over the real control.cpp
    // ======================================================================
    sil_reset(15.0, 50.0);                  // level hold, manual throttle
    double alt0 = -g_st.pd;
    sil_run(2.0);
    ck(fabs(g_st.phi) < 0.06, "2 s level hold: bank %.3f rad", g_st.phi);
    double vhold = airspeed();
    ck(vhold > 13.0 && vhold < 17.0, "level hold keeps speed (%.2f m/s)",
       vhold);
    ck(alt0 - (-g_st.pd) < 5.0, "level hold sink < 5 m in 2 s (%.2f m)",
       alt0 - (-g_st.pd));
    ck(finite_state(), "level hold finite");

    sil_reset(15.0, 50.0);                  // roll step (F2 tracking)
    g_cmd.roll = 0.4f;                      // -> phi_des = 0.4 * PHI_MAX
    double max_phi = 0, max_p = 0, max_roll_out = 0;
    for (int i = 0; i < 1200; ++i) {        // 3 s
        sil_step();
        double ap = fabs(g_st.phi);
        if (ap > max_phi) max_phi = ap;
        if (fabs(g_st.p) > max_p) max_p = fabs(g_st.p);
        if (fabs((double)g_dbg.ctrl_roll_out) > max_roll_out)
            max_roll_out = fabs((double)g_dbg.ctrl_roll_out);
    }
    ck(g_st.phi > 0.14 && g_st.phi < 0.56,
       "roll stick 0.4 -> bank %.1f deg in [8,32]", g_st.phi * rad2deg);
    ck(max_phi < 0.70, "no overshoot past 40 deg (%.1f deg)",
       max_phi * rad2deg);
    ck(max_p < 5.0, "roll rate bounded under rate caps (%.2f rad/s)", max_p);
    ck(max_roll_out > 0.1, "surface actually commanded (max %.2f)",
       max_roll_out);
    g_cmd.roll = 0.0f;
    sil_run(2.0);
    ck(fabs(g_st.phi) < 0.25, "returns near level (%.1f deg)",
       g_st.phi * rad2deg);

    sil_reset(15.0, 50.0);                  // F3 bank clamp, nav request 80 deg
    g_cmd.nav_active = true;
    g_cmd.phi_cmd = 1.4f;
    g_cmd.v_cmd = 15.0f;
    sil_run(1.0);
    ck(g_dbg.env_bank, "over-limit bank request flagged (env_bank)");
    ck(g_dbg.phi_cmd_l <= (double)PHI_MAX_DEG / rad2deg + 1e-4,
       "bank command clamped to %.0f deg (%.3f rad)",
       (double)PHI_MAX_DEG, g_dbg.phi_cmd_l);
    ck(fabs(g_st.phi) < 0.96, "actual bank stays <= ~55 deg (%.1f deg)",
       g_st.phi * rad2deg);

    // F3 stall: envelope beats a full nose-up stick, then recovers
    sil_reset(10.0, 50.0);                  // below V_MIN = 11.7
    g_cmd.pitch = 1.0f;
    bool saw_stall = false, cleared = false, saw_full_thr = false;
    double min_pitch_out = 0.0, v_max = 0.0;
    for (int i = 0; i < 1000 && finite_state(); ++i) {   // 2.5 s
        sil_step();
        if (g_dbg.env_stall) {
            saw_stall = true;
            if ((double)g_dbg.ctrl_pitch_out < min_pitch_out)
                min_pitch_out = g_dbg.ctrl_pitch_out;
            if (g_dbg.throttle_out >= 0.999f) saw_full_thr = true;
        } else if (saw_stall) {
            cleared = true;
        }
        double V = airspeed();
        if (V > v_max) v_max = V;
    }
    ck(saw_stall, "stall detected below V_MIN despite... (env_stall)");
    ck(saw_full_thr, "stall override forces full throttle");
    ck(min_pitch_out < -0.05,
       "stall override forces nose-down despite full up stick (%.3f)",
       min_pitch_out);
    ck(v_max > (double)V_MIN_MPS,
       "dive+power lifts airspeed above V_MIN (%.2f > %.1f m/s)",
       v_max, (double)V_MIN_MPS);
    ck(cleared, "stall flag clears once recovered");
    ck(finite_state(), "stall sequence stays finite");

    sil_reset(31.0, 50.0);                  // F3 V_NE flag
    sil_run(0.05);
    ck(g_dbg.env_vne, "flag raised over V_NE (%.1f > %.0f m/s)",
       airspeed(), (double)V_NE_MPS);

    sil_reset(15.0, 50.0);                  // F3 surface slew limit (D5 path)
    g_cmd.roll = 1.0f;
    sil_step();                             // lp/lr are statics inside
    double prev = g_dbg.ctrl_roll_out;      // control_update: re-baseline here
    double max_du = 0;
    for (int i = 0; i < 40; ++i) {
        sil_step();
        double d = fabs((double)g_dbg.ctrl_roll_out - prev);
        if (d > max_du) max_du = d;
        prev = g_dbg.ctrl_roll_out;
    }
    double slew_lim = (double)CMD_RATE_LIMIT_DPS / rad2deg * DT;
    ck(max_du <= slew_lim + 1e-6,
       "surface slew limited (%.5f <= %.5f per 400 Hz step)", max_du,
       slew_lim);

    sil_reset(15.0, 50.0);                  // airspeed loop (nav mode, PI)
    g_cmd.nav_active = true;
    g_cmd.phi_cmd = 0.0f;
    g_cmd.h_cmd = 50.0f;
    g_cmd.v_cmd = 18.0f;
    double v_start = airspeed();
    sil_run(5.0);
    double v_end = airspeed();
    ck(v_end > v_start + 1.0,
       "airspeed loop closes through the plant (%.1f -> %.1f m/s)",
       v_start, v_end);
    ck(v_end > 14.0 && v_end < 21.0, "airspeed bounded, no runaway (%.2f)",
       v_end);

    sil_reset(15.0, 50.0);                  // 4 s random-stick fuzz
    unsigned seed = 12345u;
    bool fuzz_ok = true;
    double vmax_f = 0, vmin_f = 1e9, maxphi_f = 0;
    for (int i = 0; i < 1600; ++i) {
        seed = seed * 1664525u + 1013904223u;
        g_cmd.roll = (float)((double)((seed >> 16) & 0xFFFF) / 32768.0 - 1.0);
        seed = seed * 1664525u + 1013904223u;
        g_cmd.pitch = (float)((double)((seed >> 16) & 0xFFFF) / 32768.0 - 1.0);
        seed = seed * 1664525u + 1013904223u;
        g_cmd.throttle = (float)((double)((seed >> 16) & 0xFFFF) / 65535.0);
        sil_step();
        if (!finite_state()) { fuzz_ok = false; break; }
        double V = airspeed();
        if (V > vmax_f) vmax_f = V;
        if (V < vmin_f) vmin_f = V;
        if (fabs(g_st.phi) > maxphi_f) maxphi_f = fabs(g_st.phi);
    }
    ck(fuzz_ok, "4 s random-stick fuzz stays finite");
    ck(fabs(g_st.phi) < 1.4, "fuzz bank bounded (%.2f rad)", g_st.phi);
    ck(vmin_f > 3.0, "fuzz never collapses airspeed (%.2f m/s)", vmin_f);
    ck(maxphi_f < 1.5, "fuzz bank excursions bounded (%.2f rad)", maxphi_f);

    // ======================================================================
    printf("identified b0 @ %.0f m/s: roll %.1f  pitch %.1f rad/s^2"
           " | config B0_ROLL_REF=%.0f B0_PITCH_REF=%.0f\n",
           (double)V_CRUISE_MPS, b0r15, b0p15,
           (double)B0_ROLL_REF, (double)B0_PITCH_REF);
    printf("== wing SIL: %d passed, %d failed ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
