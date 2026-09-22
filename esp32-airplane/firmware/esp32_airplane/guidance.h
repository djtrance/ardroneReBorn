// guidance.h — path following + RTH sequencing (checklist G2, G4, G5)
#pragma once
#include <stdint.h>
#include <stdbool.h>

// Flight phase exposed to telemetry and the failsafe FSM.
enum class Phase : uint8_t {
    DISARMED,
    ARMED_MANUAL,     // stabilization only, pilot sticks
    NAV_WAYPOINT,     // L1 to active waypoint
    NAV_RTH,          // wing-specific RTH sequence
    NAV_LOITER,       // circle over home (too high to glide down yet)
    FINAL_GLIDE,      // hold best-glide, descend on slope
    FLARE,            // 1-2 m AGL: reduce speed, cut throttle
    LANDED,
    FAILSAFE_GLIDE    // deadstick: hold V_BR, steer toward field (H5)
};

const char* phase_name(Phase p);

struct NavState {
    double  home_lat, home_lon;
    double  cur_lat,  cur_lon;
    float   alt_m;            // baro/GPS altitude (m MSL or AGL — pick one, G4)
    float   ground_mps;
    float   course_deg;
    float   dist_home_m;
    float   bearing_home_deg;
    float   phi_cmd;          // rad, output bank command
    float   v_cmd;            // m/s, output airspeed command
    float   h_cmd;            // m,   output altitude command
    bool    gps_ok;
    bool    fence_breach;     // G5
};

struct Waypoint { double lat, lon; float alt_m; float accept_m; };

void   guidance_init();
void   guidance_set_home(double lat, double lon);
void   guidance_load_default_route();   // TODO: load from flash/NVS (G3)
void   guidance_trigger_rth();          // called by failsafe / pilot switch
void   guidance_arm();                  // DISARMED -> ARMED_MANUAL (preflight passed)
void   guidance_disarm();               // back to DISARMED (on ground)

// Advance the state machine. Returns updated nav outputs.
void   guidance_update(NavState& ns, float dt);

// Current flight phase (read by telemetry / failsafe).
Phase  guidance_phase();

// --- Pure functions, unit-testable on host (J2) ---------------------------

// L1 lateral guidance: cross-track error -> bank command (G2).
//   e_lat_m     : signed cross-track distance (m), + = target to the right
//   v_mps       : ground speed (m/s)
//   Returns bank command in radians, already limited to +/- PHI_MAX.
float l1_bank_cmd(float e_lat_m, float v_mps, float lookahead_m);

// Level-turn radius: R = V^2 / (g * tan(phi)).  Sizes RTH loiter (G4).
float turn_radius_m(float v_mps, float phi_rad);

// Glide budget check: can I reach home from here at best glide?
//   L_over_D * (current altitude above home) must exceed dist_home.
bool  can_glide_home(float dist_home_m, float alt_above_home_m, float l_over_d);

// True if inside the circular geofence (G5).
bool  inside_geofence(double lat, double lon,
                      double fence_lat, double fence_lon, float radius_m);
