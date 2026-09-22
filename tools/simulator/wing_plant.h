// wing_plant.h — checklist J1: 6-DOF plant model for the ESP32 flying wing.
//
// Aerospace body axes (x forward, y right, z down), NED position, Euler 3-2-1
// attitude, RK4 integration. Inputs follow the *controller intent* sign
// convention of control.cpp so the firmware can be closed over this plant
// without touching servo-link direction (that stays a bench question, D1):
//
//     +pitch = nose up    (what ctrl_pitch_out > 0 must produce: qdot > 0)
//     +roll  = right wing down (ctrl_roll_out > 0 -> pdot > 0)
//
// Numbers are physically-plausible PLACEHOLDERS until checklist §A measures
// the real airframe: mass/area/rho/cruise come from config.h, the rest is
// marked TODO §A. The dynamic-pressure b0 helpers below are what F1
// identification (config B0_*_REF) is checked against.
#pragma once

// State layout — wing_deriv writes its 12 derivatives in this order.
enum {
    W_PN = 0, W_PE, W_PD,      // NED position [m], pd = -altitude
    W_U,  W_V,  W_W,           // body velocity [m/s]
    W_PHI, W_THETA, W_PSI,     // Euler attitude [rad]
    W_P,  W_Q,  W_R,           // body rates [rad/s]
    W_N
};

struct WingParams {
    // geometry / inertia
    double mass, S, b, c;      // kg, m^2, span, chord (c = S/b)
    double Ixx, Iyy, Izz;      // kg m^2 (Ixz neglected)
    double rho;
    // lift / drag
    double CL0, CLa;           // CL = CL0 + CLa*alpha (pre-stall)
    double alpha_stall;        // rad, linear->flat-plate blend starts here
    double CD0, k_ind;         // polar: CD = CD0 + k*CL^2 (+ stall extra)
    // stability derivatives (per rad / non-dimensional rates)
    double Cy_b;               // side force
    double Cl_b, Cl_p, Cl_r;   // roll
    double Cm_a, Cm_q, Cm0;    // pitch (Cm0 set to trim at V_CRUISE = CG place)
    double Cn_b, Cn_p, Cn_r;   // yaw
    // control derivatives PER RADIAN (signs = controller intent)
    double Cl_d, Cm_d, Cn_d;   // Cn_d < 0 = adverse yaw from roll command
    double delta_max;          // rad of surface travel at |input| = 1
    double thrust_max;         // N static thrust at throttle = 1
};

struct WingState {
    double pn, pe, pd;
    double u, v, w;
    double phi, theta, psi;
    double p, q, r;
};

struct WingInput {
    double pitch, roll;        // [-1,1] controller intent
    double throttle;           // [0,1]
};

struct WingAirdata {
    double V, alpha, beta, qbar, CL, CD;
};

// Factory defaults (placeholder airframe, §A).
void wing_default_params(WingParams* prm);

// Steady wings-level solution at airspeed V (gamma = 0): returns trim alpha
// and throttle. Used by wing_trim_state and by default_params for the Cm0 CG
// placement at V_CRUISE.
void wing_trim_solve(const WingParams& prm, double V, double* alpha,
                     double* throttle);

// Initialise a state in that trim condition at `alt` metres, heading north.
void wing_trim_state(WingState* s, const WingParams& prm, double V, double alt);

// Air data with respect to wind_ned (m/s, NED; {0,0,0} = still air).
void wing_airdata(const WingState& s, const WingParams& prm,
                  const double wind_ned[3], WingAirdata* ad);

// dx[W_N] = time derivatives of the state.
void wing_deriv(const WingState& s, const WingInput& in, const WingParams& prm,
                const double wind_ned[3], double dx[W_N]);

// One RK4 step of dt seconds.
void wing_step(WingState* s, const WingInput& in, const WingParams& prm,
               const double wind_ned[3], double dt);

// J1 system identification: plant gain b0 = angular acceleration at full
// normalized deflection, still air, zero rates (what ADRC's B0_*_REF must
// match; scales with dynamic pressure like schedule_b0 does).
double wing_b0_roll(const WingParams& prm, double V);
double wing_b0_pitch(const WingParams& prm, double V);
