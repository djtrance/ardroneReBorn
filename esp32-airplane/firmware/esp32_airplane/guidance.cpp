#include "guidance.h"
#include "gps_nav.h"
#include "config.h"
#include <math.h>

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define G_EARTH 9.80665f

static Phase      s_phase = Phase::DISARMED;
static Waypoint   s_route[8];
static int        s_route_n = 0;
static int        s_wp_idx  = 0;
static double     s_home_lat = 0.0, s_home_lon = 0.0;
static bool       s_have_home = false;

const char* phase_name(Phase p) {
    switch (p) {
        case Phase::DISARMED:        return "DISARMED";
        case Phase::ARMED_MANUAL:    return "ARMED_MANUAL";
        case Phase::NAV_WAYPOINT:    return "NAV_WAYPOINT";
        case Phase::NAV_RTH:         return "NAV_RTH";
        case Phase::NAV_LOITER:      return "NAV_LOITER";
        case Phase::FINAL_GLIDE:     return "FINAL_GLIDE";
        case Phase::FLARE:           return "FLARE";
        case Phase::LANDED:          return "LANDED";
        case Phase::FAILSAFE_GLIDE:  return "FAILSAFE_GLIDE";
        default:                     return "?";
    }
}

void guidance_init() {
    s_phase = Phase::DISARMED;
    s_route_n = 0;
    s_wp_idx = 0;
    s_have_home = false;
}

void guidance_set_home(double lat, double lon) {
    s_home_lat = lat; s_home_lon = lon; s_have_home = true;
}

// TODO (G3): load route from flash / parameter block.
void guidance_load_default_route() { s_route_n = 0; s_wp_idx = 0; }

void guidance_trigger_rth() {
    if (s_have_home) s_phase = Phase::NAV_RTH;
}

void guidance_arm() {
    if (s_phase == Phase::DISARMED) s_phase = Phase::ARMED_MANUAL;
}

void guidance_disarm() {
    s_phase = Phase::DISARMED;
    s_wp_idx = 0;
}

Phase guidance_phase() { return s_phase; }

// ===========================================================================
// Pure functions (J2 unit tests)
// ===========================================================================

// L1 lateral guidance (G2): cross-track -> bank.
//   eta   = atan2(e_lat, L1)
//   a_s   = 2 * sin(eta) * V^2 / L1
//   phi   = atan(a_s / g)
float l1_bank_cmd(float e_lat_m, float v_mps, float lookahead_m) {
    if (lookahead_m < 1.0f) lookahead_m = 1.0f;
    if (v_mps < 1.0f)       v_mps = 1.0f;

    float eta = atan2f(e_lat_m, lookahead_m);
    float a_s = 2.0f * sinf(eta) * v_mps * v_mps / lookahead_m;
    float phi = atanf(a_s / G_EARTH);

    // Envelope clamp (F3) — same limit the control layer enforces.
    float phi_max = PHI_MAX_DEG * (float)M_PI / 180.0f;
    if (phi >  phi_max) phi =  phi_max;
    if (phi < -phi_max) phi = -phi_max;
    return phi;
}

float turn_radius_m(float v_mps, float phi_rad) {
    float t = tanf(phi_rad);
    if (fabsf(t) < 0.02f) return 1e6f;   // near-level => huge radius
    return (v_mps * v_mps) / (G_EARTH * t);
}

bool can_glide_home(float dist_home_m, float alt_above_home_m, float l_over_d) {
    if (l_over_d <= 0.0f) return false;
    float reach = l_over_d * alt_above_home_m;
    return reach >= dist_home_m;
}

bool inside_geofence(double lat, double lon,
                     double fence_lat, double fence_lon, float radius_m) {
    return haversine_m(lat, lon, fence_lat, fence_lon) < radius_m;
}

// ===========================================================================
// State machine — wing-specific RTH (algorithm-mapping §4.2)
// ===========================================================================
void guidance_update(NavState& ns, float dt) {
    (void)dt;
    ns.gps_ok         = true;
    ns.dist_home_m    = s_have_home ? (float)haversine_m(ns.cur_lat, ns.cur_lon,
                                                         s_home_lat, s_home_lon) : 0.0f;
    ns.bearing_home_deg = s_have_home ? (float)bearing_deg(ns.cur_lat, ns.cur_lon,
                                                           s_home_lat, s_home_lon) : 0.0f;

    // --- G5 geofence: independent hard safety layer ------------------------
    bool outside_circle = s_have_home &&
        !inside_geofence(ns.cur_lat, ns.cur_lon, s_home_lat, s_home_lon,
                         GEOFENCE_RADIUS_M);
    bool alt_out = (ns.alt_m > GEOFENCE_ALT_MAX_M) || (ns.alt_m < GEOFENCE_ALT_MIN_M);
    ns.fence_breach = outside_circle || alt_out;

    float phi_max = PHI_MAX_DEG * (float)M_PI / 180.0f;
    float V = (ns.ground_mps > 5.0f) ? ns.ground_mps : V_CRUISE_MPS;

    // Default outputs
    ns.phi_cmd = 0.0f;
    ns.v_cmd   = V_CRUISE_MPS;
    ns.h_cmd   = RTH_ALTITUDE_AGL_M;

    // Geofence always wins: turn back toward home.
    if (ns.fence_breach && s_phase != Phase::LANDED && s_phase != Phase::DISARMED) {
        s_phase = Phase::NAV_RTH;
    }

    switch (s_phase) {
        case Phase::DISARMED:
        case Phase::ARMED_MANUAL:
        case Phase::LANDED:
            break;

        // -----------------------------------------------------------------
        // WING RTH: energy-aware, bank-limited, loiter before descending
        // -----------------------------------------------------------------
        case Phase::NAV_RTH: {
            // 1) point nose toward home using L1 against the bearing line
            float track_err = ns.bearing_home_deg - ns.course_deg;
            while (track_err >  180.0f) track_err -= 360.0f;
            while (track_err < -180.0f) track_err += 360.0f;
            // Convert heading error to an equivalent cross-track at L1 range
            float e_lat = sinf(track_err * (float)M_PI / 180.0f) * L1_LOOKAHEAD_M;
            ns.phi_cmd  = l1_bank_cmd(e_lat, V, L1_LOOKAHEAD_M);

            // 2) energy check (G4): if too high to glide down efficiently, loiter
            float alt_above_home = ns.alt_m - RTH_ALTITUDE_AGL_M;
            if (ns.dist_home_m < RTH_LOITER_RADIUS_M && alt_above_home > 15.0f) {
                s_phase = Phase::NAV_LOITER;
            }
            // 3) close enough + right height -> final glide
            else if (ns.dist_home_m < 100.0f && alt_above_home < 15.0f) {
                s_phase = Phase::FINAL_GLIDE;
            }
            break;
        }

        // -----------------------------------------------------------------
        // LOITER: hold a circle over home sized by phi_max
        //   R = V^2 / (g*tan(phi_max)); tangent to home point
        // -----------------------------------------------------------------
        case Phase::NAV_LOITER: {
            float R = turn_radius_m(V, phi_max);
            if (R > RTH_LOITER_RADIUS_M) R = RTH_LOITER_RADIUS_M;
            // Cross-track to the circle: positive = outside, fly inward
            float radial = ns.dist_home_m;
            float e_lat  = (radial - R);      // want to sit on the circle
            // steer tangentially (right-hand orbit): offset the target point
            float heading_to_home = ns.bearing_home_deg;
            float orbit_target = heading_to_home + 90.0f;   // tangent
            float track_err = orbit_target - ns.course_deg;
            while (track_err >  180.0f) track_err -= 360.0f;
            while (track_err < -180.0f) track_err += 360.0f;
            float et = sinf(track_err * (float)M_PI / 180.0f) * L1_LOOKAHEAD_M
                     + e_lat * 0.5f;
            ns.phi_cmd = l1_bank_cmd(et, V, L1_LOOKAHEAD_M);

            // Bleed altitude by reducing bank slightly each second
            ns.h_cmd = RTH_ALTITUDE_AGL_M;
            float alt_above_home = ns.alt_m - RTH_ALTITUDE_AGL_M;
            if (alt_above_home < 15.0f) s_phase = Phase::FINAL_GLIDE;
            break;
        }

        // -----------------------------------------------------------------
        // FINAL GLIDE: wings level-ish, hold best glide, descend on slope
        // -----------------------------------------------------------------
        case Phase::FINAL_GLIDE: {
            float track_err = ns.bearing_home_deg - ns.course_deg;
            while (track_err >  180.0f) track_err -= 360.0f;
            while (track_err < -180.0f) track_err += 360.0f;
            float e_lat = sinf(track_err * (float)M_PI / 180.0f) * L1_LOOKAHEAD_M;
            ns.phi_cmd  = l1_bank_cmd(e_lat * 0.6f, V, L1_LOOKAHEAD_M);  // gentler
            ns.v_cmd    = V_BEST_GLIDE_MPS;
            ns.h_cmd    = 0.0f;

            if (ns.alt_m < FLARE_ALT_M + 1.0f) s_phase = Phase::FLARE;
            break;
        }

        // -----------------------------------------------------------------
        // FLARE: reduce speed, hold nose, then land
        // -----------------------------------------------------------------
        case Phase::FLARE: {
            ns.phi_cmd = 0.0f;
            ns.v_cmd   = V_STALL_MPS + 1.5f;   // just above stall
            if (ns.ground_mps < V_STALL_MPS * 0.7f || ns.alt_m < 0.3f)
                s_phase = Phase::LANDED;
            break;
        }

        // -----------------------------------------------------------------
        // DEADSTICK GLIDE (H5) — replaces ETH spinning-flight recovery
        // Hold best glide, steer toward the nearest safe field (= home here).
        // -----------------------------------------------------------------
        case Phase::FAILSAFE_GLIDE: {
            float track_err = ns.bearing_home_deg - ns.course_deg;
            while (track_err >  180.0f) track_err -= 360.0f;
            while (track_err < -180.0f) track_err += 360.0f;
            float e_lat = sinf(track_err * (float)M_PI / 180.0f) * L1_LOOKAHEAD_M;
            ns.phi_cmd  = l1_bank_cmd(e_lat, V, L1_LOOKAHEAD_M);
            ns.v_cmd    = V_BEST_GLIDE_MPS;
            ns.h_cmd    = 0.0f;
            if (ns.alt_m < FLARE_ALT_M) s_phase = Phase::FLARE;
            break;
        }

        // -----------------------------------------------------------------
        // WAYPOINT following via L1 (G3)
        // -----------------------------------------------------------------
        case Phase::NAV_WAYPOINT: {
            if (s_wp_idx >= s_route_n) { s_phase = Phase::NAV_RTH; break; }
            const Waypoint& w = s_route[s_wp_idx];
            float d = (float)haversine_m(ns.cur_lat, ns.cur_lon, w.lat, w.lon);
            if (d < w.accept_m) { s_wp_idx++; break; }
            // Cross-track relative to the leg's desired track
            float desired = (float)bearing_deg(ns.cur_lat, ns.cur_lon, w.lat, w.lon);
            float track_err = desired - ns.course_deg;
            while (track_err >  180.0f) track_err -= 360.0f;
            while (track_err < -180.0f) track_err += 360.0f;
            float e_lat = sinf(track_err * (float)M_PI / 180.0f) * L1_LOOKAHEAD_M;
            ns.phi_cmd  = l1_bank_cmd(e_lat, V, L1_LOOKAHEAD_M);
            ns.v_cmd    = V_CRUISE_MPS;
            ns.h_cmd    = w.alt_m;
            break;
        }
    }
}
