#include "adrc.h"
#include <math.h>
#include <string.h>

/* ADRC scaling constants (from ADRC-inav) */
#define ADRC_WC_SCALE  1.0f
#define ADRC_WO_SCALE  1.0f
#define ADRC_B0_SCALE  10.0f
#define ADRC_B0_FALLBACK 50.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ================================================================== */
/*  ADRC Initialization                                                */
/* ================================================================== */

void adrc_init(adrc_state_t *s, const adrc_gains_t *gains, float dt) {
    if (!s || !gains) return;
    memset(s, 0, sizeof(adrc_state_t));

    float wc = gains->wc;
    float wo = gains->wo;
    float b0 = gains->b0;

    /* Sane fallbacks */
    if (wc < 1.0f) wc = 10.0f;
    if (wo < 1.0f) wo = 30.0f;
    if (b0 < 1.0f) b0 = ADRC_B0_FALLBACK;

    s->b0 = b0;

    /* Controller gains (linear PD with disturbance cancellation) */
    s->kp = wc * wc;       /* = wc² */
    s->kd = 2.0f * wc;     /* = 2·wc (critical damping ratio ζ=1) */

    /* Discrete ESO gains (bandwidth parameterization, ZOH discretization)
     *
     * Pole placement: all observer poles at -wo
     * r = exp(-wo * dt)
     * l1 = 3·(1 - r)
     * l2 = (1 - r)²·(5 + r) / (2·dt)
     * l3 = (1 - r)³ / dt²
     */
    if (dt < 0.00001f) dt = 0.000125f;  /* Fallback for 8kHz */
    float r = expf(-wo * dt);
    float one_minus_r = 1.0f - r;

    s->l1 = 3.0f * one_minus_r;
    s->l2 = one_minus_r * one_minus_r * (5.0f + r) / (2.0f * dt);
    s->l3 = one_minus_r * one_minus_r * one_minus_r / (dt * dt);

    /* Initialize ESO states */
    s->z1 = 0.0f;
    s->z2 = 0.0f;
    s->z3 = 0.0f;
    s->last_output = 0.0f;
    s->output = 0.0f;
}

void adrc_reset(adrc_state_t *s, float initial_rate) {
    if (!s) return;
    s->z1 = initial_rate;
    s->z2 = 0.0f;
    s->z3 = 0.0f;
    s->last_output = 0.0f;
    s->output = 0.0f;
}

/* ================================================================== */
/*  ADRC Main Update (ESO + NLSEF)                                     */
/* ================================================================== */

float adrc_update(adrc_state_t *s, const adrc_gains_t *gains,
                  float rate_target, float gyro_rate,
                  float dt, float out_max) {
    if (!s || !gains || dt < 0.00001f) return 0.0f;

    float b0 = s->b0;
    float kp = s->kp;
    float kd = s->kd;

    /* --- Extended State Observer (DESO) Update --- */
    /* Discrete ZOH discretization of the continuous ESO:
     *
     * ḋz1/dt = z2 + l1·(y - z1)
     * ḋz2/dt = z3 + b0·u + l2·(y - z1)
     * ḋz3/dt = l3·(y - z1)
     *
     * Discrete form (Forward Euler / ZOH):
     * z1[k+1] = z1[k] + dt·z2[k] + (dt²/2)·(z3[k] + b0·u[k]) - l1·e[k]
     * z2[k+1] = z2[k] + dt·(z3[k] + b0·u[k]) - l2·e[k]
     * z3[k+1] = z3[k] - l3·e[k]
     *
     * where e[k] = z1[k] - y[k] (observer error)
     */
    float error_eso = s->z1 - gyro_rate;
    float dt2_2 = 0.5f * dt * dt;
    float z3_b0_u = s->z3 + b0 * s->last_output;

    float z1_next = s->z1 + dt * s->z2 + dt2_2 * z3_b0_u - s->l1 * error_eso;
    float z2_next = s->z2 + dt * z3_b0_u - s->l2 * error_eso;
    float z3_next = s->z3 - s->l3 * error_eso;

    s->z1 = z1_next;
    s->z2 = z2_next;
    s->z3 = z3_next;

    /* Anti-windup: clamp disturbance estimate */
    float dist_limit = out_max * b0 * 2.0f;
    s->z3 = clampf(s->z3, -dist_limit, dist_limit);

    /* --- NLSEF (Linear PD + Disturbance Cancellation) --- */
    /* Control law:
     *   u = (kp·(v - z1) - kd·z2 - z3) / b0
     *
     * where:
     *   v  = rate target
     *   z1 = estimated rate
     *   z2 = estimated acceleration (used as derivative term)
     *   z3 = estimated disturbance (actively canceled)
     *
     * This is equivalent to a PD controller with:
     *   - Derivative on measurement (not error, no derivative kick)
     *   - Active disturbance cancellation (replaces integral term)
     */
    float new_output = (kp * (rate_target - s->z1)
                      - kd * s->z2
                      - s->z3) / b0;

    /* Output saturation */
    new_output = clampf(new_output, -out_max, out_max);
    s->last_output = new_output;
    s->output = new_output;

    return new_output;
}

/* ================================================================== */
/*  State Accessors                                                    */
/* ================================================================== */

void adrc_get_states(const adrc_state_t *s,
                     float *z1_rate, float *z2_accel, float *z3_disturbance) {
    if (!s) return;
    if (z1_rate) *z1_rate = s->z1;
    if (z2_accel) *z2_accel = s->z2;
    if (z3_disturbance) *z3_disturbance = s->z3;
}

void adrc_get_gains(const adrc_state_t *s,
                    float *kp, float *kd, float *l1, float *l2, float *l3) {
    if (!s) return;
    if (kp) *kp = s->kp;
    if (kd) *kd = s->kd;
    if (l1) *l1 = s->l1;
    if (l2) *l2 = s->l2;
    if (l3) *l3 = s->l3;
}

/* ================================================================== */
/*  PID → ADRC Conversion                                              */
/* ================================================================== */

adrc_gains_t adrc_from_pid(float p, float i, float d) {
    adrc_gains_t g;
    g.wc = p * ADRC_WC_SCALE;
    g.wo = i * ADRC_WO_SCALE;
    g.b0 = d * ADRC_B0_SCALE;

    /* Sane fallbacks */
    if (g.wc < 1.0f) g.wc = 10.0f;
    if (g.wo < 1.0f) g.wo = 30.0f;
    if (g.b0 < 1.0f) g.b0 = ADRC_B0_FALLBACK;

    g.out_max = 1.0f;
    return g;
}
