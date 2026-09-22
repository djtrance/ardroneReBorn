// wing_plant.cpp — checklist J1: 6-DOF flying-wing plant (see wing_plant.h).
//
// Model notes
//   * Body axes with the omega x v term on dv_b/dt (rotating frame).
//   * Gravity enters as a FORCE only (at the CG -> no moment).
//   * Stall: linear CL blends over [alpha_stall, alpha_stall+0.15] into a
//     flat plate (1.1*sin(2a)); extra drag 2*sin^2(alpha) rides the same
//     blend weight, so CL peaks just past alpha_stall and drops while CD
//     climbs — a real stall break, monotone in both channels.
//   * Trim solve uses the exact level-flight balance L = W - T*sin(alpha),
//     T = D/cos(alpha) (thrust tilted with the body helps lift).
#include "wing_plant.h"
#include "config.h"      // MASS_KG, WING_AREA_M2, RHO_SEA_LEVEL, V_CRUISE_MPS
#include <math.h>

static const double WING_G = 9.80665;

void wing_default_params(WingParams* prm) {
    prm->mass = (double)MASS_KG;
    prm->S    = (double)WING_AREA_M2;
    prm->b    = 1.10;                       // TODO §A measure span
    prm->c    = prm->S / prm->b;            // AR = b/c = b^2/S ~ 4.1
    prm->Ixx  = 0.025;                      // TODO §A (mass at centre -> small)
    prm->Iyy  = 0.030;
    prm->Izz  = 0.045;
    prm->rho  = (double)RHO_SEA_LEVEL;

    prm->CL0 = 0.02;
    prm->CLa = 4.5;                         // /rad, AR~4 wing
    prm->alpha_stall = 0.26;                // 15 deg
    prm->CD0 = 0.035;
    prm->k_ind = 0.098;                     // 1/(pi*AR*e), e=0.8

    prm->Cy_b = -0.5;
    prm->Cl_b = -0.09;                      // dihedral effect
    prm->Cl_p = -0.50;
    prm->Cl_r = 0.10;
    prm->Cm_a = -0.80;                      // static stability
    prm->Cm_q = -12.0;
    prm->Cn_b = 0.08;                       // weathervane
    prm->Cn_p = -0.02;
    prm->Cn_r = -0.12;

    prm->Cl_d = 0.12;                       // per rad, full elevon roll
    prm->Cm_d = 0.50;                       // per rad, elevon pitch
    prm->Cn_d = -0.03;                      // adverse yaw of a roll command
    prm->delta_max = 0.40;                  // 23 deg at |input| = 1
    prm->thrust_max = 14.0;                 // ~1.8 x weight static

    // Place the CG so the wing trims hands-off at V_CRUISE: Cm(alpha_t)=0.
    double a_t, thr_t;
    wing_trim_solve(*prm, (double)V_CRUISE_MPS, &a_t, &thr_t);
    prm->Cm0 = -prm->Cm_a * a_t;
    (void)thr_t;
}

void wing_trim_solve(const WingParams& prm, double V, double* alpha,
                     double* throttle) {
    const double W   = prm.mass * WING_G;
    const double qS  = 0.5 * prm.rho * V * V * prm.S;
    double CL = W / qS;                     // first guess: L = W
    double a  = (CL - prm.CL0) / prm.CLa;
    double T  = 0.0;
    for (int i = 0; i < 32; ++i) {          // converges in ~3 iterations
        double CD = prm.CD0 + prm.k_ind * CL * CL;
        double ca = cos(a);
        if (ca < 0.1) ca = 0.1;
        T  = qS * CD / ca;                  // T*cos(a) = D
        CL = (W - T * sin(a)) / qS;         // L + T*sin(a) = W
        a  = (CL - prm.CL0) / prm.CLa;
    }
    double CDf = prm.CD0 + prm.k_ind * CL * CL;
    double ca  = cos(a);
    if (ca < 0.1) ca = 0.1;
    double thr = sqrt((qS * CDf / ca) / prm.thrust_max);
    if (thr > 1.0) thr = 1.0;
    *alpha    = a;
    *throttle = thr;
}

void wing_trim_state(WingState* s, const WingParams& prm, double V,
                     double alt) {
    double a, thr;
    wing_trim_solve(prm, V, &a, &thr);
    *s = WingState{};
    s->pn = 0.0; s->pe = 0.0; s->pd = -alt;
    s->u = V * cos(a);                      // gamma = 0 -> theta = alpha
    s->v = 0.0;
    s->w = V * sin(a);
    s->phi = 0.0; s->theta = a; s->psi = 0.0;
    s->p = 0.0; s->q = 0.0; s->r = 0.0;
}

// --- helpers ---------------------------------------------------------------
static double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Body -> NED rotation (Rz(psi)*Ry(theta)*Rx(phi)).
static void b2n(double phi, double th, double ps, const double vb[3],
                double vn[3]) {
    double sF = sin(phi), cF = cos(phi);
    double sT = sin(th),  cT = cos(th);
    double sP = sin(ps),  cP = cos(ps);
    vn[0] = cT*cP*vb[0] + (sF*sT*cP - cF*sP)*vb[1] + (cF*sT*cP + sF*sP)*vb[2];
    vn[1] = cT*sP*vb[0] + (sF*sT*sP + cF*cP)*vb[1] + (cF*sT*sP - sF*cP)*vb[2];
    vn[2] =      -sT*vb[0] + (sF*cT)*vb[1]           + (cF*cT)*vb[2];
}

// NED -> body (transpose of the above).
static void n2b(double phi, double th, double ps, const double vn[3],
                double vb[3]) {
    double sF = sin(phi), cF = cos(phi);
    double sT = sin(th),  cT = cos(th);
    double sP = sin(ps),  cP = cos(ps);
    vb[0] = cT*cP*vn[0] + cT*sP*vn[1] -      sT*vn[2];
    vb[1] = (sF*sT*cP - cF*sP)*vn[0] + (sF*sT*sP + cF*cP)*vn[1] + (sF*cT)*vn[2];
    vb[2] = (cF*sT*cP + sF*sP)*vn[0] + (cF*sT*sP - sF*cP)*vn[1] + (cF*cT)*vn[2];
}

// Lift/drag with the stall blend (shared by airdata and deriv).
static void lift_drag(const WingParams& prm, double alpha, double* CL,
                      double* CD) {
    double aa = fabs(alpha);
    double s  = 0.0;                        // blend weight past the stall
    if (aa > prm.alpha_stall) {
        double t = (aa - prm.alpha_stall) / 0.15;
        if (t > 1.0) t = 1.0;
        s = t;                              // linear: CL declines right away
    }
    double cl_lin = prm.CL0 + prm.CLa * alpha;
    double cl_fp  = 1.1 * sin(2.0 * alpha); // flat plate post-stall
    double cd_fp  = 2.0 * sin(aa) * sin(aa);
    *CL = (1.0 - s) * cl_lin + s * cl_fp;
    *CD = prm.CD0 + prm.k_ind * (*CL) * (*CL) + s * cd_fp;
}

void wing_airdata(const WingState& s, const WingParams& prm,
                  const double wind_ned[3], WingAirdata* ad) {
    double vs[3] = { s.u, s.v, s.w }, vn[3], wr[3], vr[3];
    b2n(s.phi, s.theta, s.psi, vs, vn);
    for (int i = 0; i < 3; ++i) wr[i] = vn[i] - wind_ned[i];
    n2b(s.phi, s.theta, s.psi, wr, vr);
    double V = sqrt(vr[0]*vr[0] + vr[1]*vr[1] + vr[2]*vr[2]);
    ad->V     = V;
    ad->alpha = atan2(vr[2], vr[0]);
    ad->beta  = asin(clampd(V > 1e-9 ? vr[1] / V : 0.0, -1.0, 1.0));
    ad->qbar  = 0.5 * prm.rho * V * V;
    lift_drag(prm, ad->alpha, &ad->CL, &ad->CD);
}

void wing_deriv(const WingState& s, const WingInput& in, const WingParams& prm,
                const double wind_ned[3], double dx[W_N]) {
    WingAirdata ad;
    wing_airdata(s, prm, wind_ned, &ad);
    const double V  = ad.V;
    const double Vs = V > 2.0 ? V : 2.0;    // rate non-dimension denominator

    const double dp = in.pitch * prm.delta_max;   // rad, controller intent
    const double dr = in.roll  * prm.delta_max;
    const double thr = clampd(in.throttle, 0.0, 1.0);
    const double T = thr * thr * prm.thrust_max;  // prop: T ~ rpm^2

    const double sa = sin(ad.alpha), ca = cos(ad.alpha);
    const double qS = ad.qbar * prm.S;

    // --- forces in body axes ---
    double Fx = qS * (-ad.CD * ca + ad.CL * sa) + T;
    double Fy = qS * (prm.Cy_b * ad.beta);
    double Fz = qS * (-ad.CD * sa - ad.CL * ca);

    // gravity as force only (CG = origin -> no moment), body components
    const double g = WING_G;
    double gbx = -g * sin(s.theta);
    double gby =  g * cos(s.theta) * sin(s.phi);
    double gbz =  g * cos(s.theta) * cos(s.phi);

    // dv_b/dt = f/m + g_b - omega x v   (body frame is rotating)
    dx[W_U] = Fx / prm.mass + gbx - (s.q * s.w - s.r * s.v);
    dx[W_V] = Fy / prm.mass + gby - (s.r * s.u - s.p * s.w);
    dx[W_W] = Fz / prm.mass + gbz - (s.p * s.v - s.q * s.u);

    // --- moments (dimensional aero coefficients) ---
    double ph = s.p * prm.b / (2.0 * Vs);
    double qh = s.q * prm.c / (2.0 * Vs);
    double rh = s.r * prm.b / (2.0 * Vs);
    double Cl = prm.Cl_b * ad.beta + prm.Cl_p * ph + prm.Cl_r * rh
              + prm.Cl_d * dr;
    double Cm = prm.Cm0 + prm.Cm_a * ad.alpha + prm.Cm_q * qh
              + prm.Cm_d * dp;
    double Cn = prm.Cn_b * ad.beta + prm.Cn_p * ph + prm.Cn_r * rh
              + prm.Cn_d * dr;
    double Lm = qS * prm.b * Cl;
    double Mm = qS * prm.c * Cm;
    double Nm = qS * prm.b * Cn;

    dx[W_P] = (Lm + (prm.Iyy - prm.Izz) * s.q * s.r) / prm.Ixx;
    dx[W_Q] = (Mm + (prm.Izz - prm.Ixx) * s.p * s.r) / prm.Iyy;
    dx[W_R] = (Nm + (prm.Ixx - prm.Iyy) * s.p * s.q) / prm.Izz;

    // --- attitude kinematics (guard near theta = +-90) ---
    double cT = cos(s.theta);
    if (fabs(cT) < 0.05) cT = cT < 0 ? -0.05 : 0.05;
    double tT = sin(s.theta) / cT;
    dx[W_PHI]   = s.p + (s.q * sin(s.phi) + s.r * cos(s.phi)) * tT;
    dx[W_THETA] = s.q * cos(s.phi) - s.r * sin(s.phi);
    dx[W_PSI]   = (s.q * sin(s.phi) + s.r * cos(s.phi)) / cT;

    // --- position: velocity rotated into NED ---
    double vn[3] = { s.u, s.v, s.w };
    double r[3];
    b2n(s.phi, s.theta, s.psi, vn, r);
    dx[W_PN] = r[0];
    dx[W_PE] = r[1];
    dx[W_PD] = r[2];
}

// --- RK4 plumbing ----------------------------------------------------------
static void st_add(WingState& o, const WingState& b, const double k[W_N],
                   double h) {
    o.pn = b.pn + h*k[W_PN]; o.pe = b.pe + h*k[W_PE]; o.pd = b.pd + h*k[W_PD];
    o.u  = b.u  + h*k[W_U];  o.v  = b.v  + h*k[W_V];  o.w  = b.w  + h*k[W_W];
    o.phi   = b.phi   + h*k[W_PHI];
    o.theta = b.theta + h*k[W_THETA];
    o.psi   = b.psi   + h*k[W_PSI];
    o.p = b.p + h*k[W_P]; o.q = b.q + h*k[W_Q]; o.r = b.r + h*k[W_R];
}

static void st_mix(WingState& o, const double k1[W_N], const double k2[W_N],
                   const double k3[W_N], const double k4[W_N], double h) {
    double a[W_N];
    for (int i = 0; i < W_N; ++i)
        a[i] = (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    o.pn += a[W_PN]; o.pe += a[W_PE]; o.pd += a[W_PD];
    o.u  += a[W_U];  o.v  += a[W_V];  o.w  += a[W_W];
    o.phi += a[W_PHI]; o.theta += a[W_THETA]; o.psi += a[W_PSI];
    o.p += a[W_P]; o.q += a[W_Q]; o.r += a[W_R];
}

void wing_step(WingState* s, const WingInput& in, const WingParams& prm,
               const double wind_ned[3], double dt) {
    double k1[W_N], k2[W_N], k3[W_N], k4[W_N];
    WingState t;
    wing_deriv(*s, in, prm, wind_ned, k1);
    st_add(t, *s, k1, 0.5 * dt); wing_deriv(t, in, prm, wind_ned, k2);
    st_add(t, *s, k2, 0.5 * dt); wing_deriv(t, in, prm, wind_ned, k3);
    st_add(t, *s, k3, dt);       wing_deriv(t, in, prm, wind_ned, k4);
    st_mix(*s, k1, k2, k3, k4, dt);
}

// --- J1 identification: b0 = d(angular accel)/d(normalized input) ----------
double wing_b0_roll(const WingParams& prm, double V) {
    double qbar = 0.5 * prm.rho * V * V;
    return qbar * prm.S * prm.b * (prm.Cl_d * prm.delta_max) / prm.Ixx;
}

double wing_b0_pitch(const WingParams& prm, double V) {
    double qbar = 0.5 * prm.rho * V * V;
    return qbar * prm.S * prm.c * (prm.Cm_d * prm.delta_max) / prm.Iyy;
}
