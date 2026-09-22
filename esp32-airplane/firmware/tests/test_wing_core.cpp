// test_wing_core.cpp — host-side unit tests for the ESP32 flying wing.
// Mirrors the tools/simulator/test_*.c style used for the AR.Drone quad.
//
// Build & run:
//   cd firmware && g++ -std=gnu++17 -Wall -Wextra -O2 -Iesp32_airplane \
//       tests/test_wing_core.cpp esp32_airplane/{mixing,ahrs,control,gps_nav,\
//       guidance,failsafe}.cpp esp32_airplane/drivers/ld2450.cpp \
//       -o /tmp/test_wing && /tmp/test_wing
//
// Checklist ref: J2 (unit tests), J1 (SIL precursor).
 #include "mixing.h"
 #include "control.h"
 #include "guidance.h"
 #include "gps_nav.h"
 #include "failsafe.h"
 #include "drivers/ld2450.h"
 #include "config.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>

static int g_pass = 0, g_fail = 0;
static void ck(bool ok, const char* name) {
    if (ok) { g_pass++; printf("  ok   %s\n", name); }
    else    { g_fail++; printf("  FAIL %s\n", name); }
}
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

// Build an NMEA sentence with a *correct* computed checksum, so the parser's
// checksum validation is tested rather than a magic constant.
static void nmea_body(char* out, size_t n, const char* body) {
    uint8_t cs = 0;
    for (const char* q = body; *q; ++q) cs ^= (uint8_t)*q;
    snprintf(out, n, "$%s*%02X", body, cs);
}

// ===========================================================================
// 1. Elevon mixing — signs are the classic first-flight killer (D1)
// ===========================================================================
static void test_mixing() {
    printf("[mixing]\n");

    // Neutral => both surfaces at trim
    ElevonOut n = mix_elevons(0.0f, 0.0f, 0.0f);
    ck(n.left_us == ELEVON_L_TRIM_US && n.right_us == ELEVON_R_TRIM_US,
       "neutral -> both surfaces at trim");

    // Pure pitch-up: BOTH surfaces move the SAME direction (symmetric)
    ElevonOut p = mix_elevons(+1.0f, 0.0f, 0.0f);
    ck(p.left_us > ELEVON_L_TRIM_US && p.right_us > ELEVON_R_TRIM_US,
       "pitch-up deflects both elevons same way");

    // Pure roll-right: surfaces move OPPOSITE directions
    ElevonOut r = mix_elevons(0.0f, +1.0f, 0.0f);
    ck((r.right_us > ELEVON_R_TRIM_US) != (r.left_us > ELEVON_L_TRIM_US),
       "roll deflects elevons differentially");

    // Symmetric inputs must stay symmetric despite differential
    ElevonOut s = mix_elevons(0.5f, 0.0f, 0.0f);
    ck(abs((int)s.left_us - ELEVON_L_TRIM_US) ==
       abs((int)s.right_us - ELEVON_R_TRIM_US),
       "pure pitch stays symmetric (diff = 0)");

    // Throttle mapping: 0 -> ESC_MIN, 1 -> ESC_MAX, clamped beyond
    ElevonOut t0 = mix_elevons(0, 0, 0.0f);
    ElevonOut t1 = mix_elevons(0, 0, 1.0f);
    ElevonOut tX = mix_elevons(0, 0, 2.0f);
    ck(t0.throttle_us == ESC_MIN_US, "throttle 0 -> 1000us");
    ck(t1.throttle_us == ESC_MAX_US, "throttle 1 -> 2000us");
    ck(tX.throttle_us == ESC_MAX_US, "throttle over-range clamped");

    // Output always inside servo range
    bool in_range = true;
    for (float a = -1.5f; a <= 1.5f; a += 0.1f) {
        ElevonOut o = mix_elevons(a, a, 1.0f);
        if (o.left_us < SERVO_MIN_US || o.left_us > SERVO_MAX_US ||
            o.right_us < SERVO_MIN_US || o.right_us > SERVO_MAX_US) in_range = false;
    }
    ck(in_range, "all outputs within [SERVO_MIN, SERVO_MAX]");
}

// ===========================================================================
// 2. Envelope protection — the #1 killer (F3 / N4)
// ===========================================================================
static EnvelopeLimits mk_lim() {
    EnvelopeLimits l{};
    l.phi_max_rad = PHI_MAX_DEG * (float)M_PI / 180.0f;
    l.v_min = V_MIN_MPS; l.v_ne = V_NE_MPS;
    l.climb_max = CLIMB_MAX_MPS; l.sink_max = SINK_MAX_MPS;
    l.n_max_pos = N_MAX_POS; l.n_max_neg = N_MAX_NEG;
    return l;
}

static void test_envelope() {
    printf("[envelope]\n");
    EnvelopeLimits l = mk_lim();

    bool hit = false;
    float c = env_clamp_bank(0.0f, l, hit);
    ck(near(c, 0.0f, 1e-6f) && !hit, "bank within limit passes through");

    float over = env_clamp_bank(90.0f * (float)M_PI / 180.0f, l, hit);
    ck(hit && near(over, l.phi_max_rad, 1e-5f), "bank 90deg clamped to PHI_MAX");
    hit = false;
    float under = env_clamp_bank(-90.0f * (float)M_PI / 180.0f, l, hit);
    ck(hit && near(under, -l.phi_max_rad, 1e-5f), "bank -90deg clamped to -PHI_MAX");

    ck(!env_stall(V_CRUISE_MPS, l),  "cruise speed is not a stall");
    ck( env_stall(V_MIN_MPS - 0.5f, l), "below V_min => stall");
    ck(!env_stall(V_MIN_MPS + 0.5f, l), "above V_min => no stall");
    ck(!env_stall(0.0f, l), "v=0 (no estimate) does not false-trigger stall");

    // Load factor from bank: level = 1g, 60 deg = 2g, 0 deg = 1g
    ck(near(env_load_factor(0.0f), 1.0f, 0.01f), "0 bank -> 1g");
    ck(near(env_load_factor(60.0f * (float)M_PI / 180.0f), 2.0f, 0.02f), "60deg bank -> 2g");

    // Config sanity the preflight gate checks (A6 must be filled coherently)
    ck(V_NE_MPS > V_MIN_MPS && V_MIN_MPS > V_STALL_MPS,
       "V_ne > V_min > V_stall ordering");
    ck(PHI_MAX_DEG > 5.0f && PHI_MAX_DEG < 80.0f, "PHI_MAX in sane range");
}

// ===========================================================================
// 3. Control loop — step response settles, stall override works
// ===========================================================================
static void test_control() {
    printf("[control]\n");
    control_init();

    Attitude att{};
    ControlCmd cmd{};
    ControlDebug dbg{};
    cmd.roll = 0.0f; cmd.pitch = 0.0f; cmd.throttle = 0.5f;
    cmd.nav_active = false;

    // Start banked 10 deg, command level -> should drive toward level.
    att.roll = 10.0f * (float)M_PI / 180.0f;
    float v = V_CRUISE_MPS;
    // Simulate: measured rate responds to surface output.
    float gx = 0.0f;
    for (int i = 0; i < 400; i++) {           // 1 s @ 400 Hz
        float dt = 1.0f / DT_RATE_HZ;
        att.gx = gx;
        control_update(att, cmd, v, dt, dbg);
        // crude plant: surface output produces angular accel
        gx += dbg.ctrl_roll_out * 6.0f * dt;
        gx *= 0.995f;                          // damping
        att.roll += gx * dt;
    }
    ck(fabsf(att.roll) < 10.0f * (float)M_PI / 180.0f,
       "roll loop drives toward commanded attitude");

    // b0 scheduling: lower speed => smaller b0 (control authority drops)
    control_init();
    ControlDebug d1, d2;
    Attitude a0{}; a0.roll = 0;
    control_update(a0, cmd, V_CRUISE_MPS, 0.0025f, d1);
    control_init();
    control_update(a0, cmd, V_CRUISE_MPS * 0.7f, 0.0025f, d2);
    ck(d2.b0_roll < d1.b0_roll, "b0 scheduled DOWN at lower airspeed");
    ck(d1.b0_roll > 0.0f && d2.b0_roll > 0.0f, "b0 stays positive");

    // Stall override: nose must go down and throttle must go to 1.0
    control_init();
    Attitude a1{}; a1.roll = 0; a1.pitch = 0;
    ControlCmd c1{}; c1.pitch = +1.0f; c1.roll = 0.4f; c1.nav_active = false;
    ControlDebug dv;
    control_update(a1, c1, V_STALL_MPS * 0.8f, 0.0025f, dv);
    ck(dv.env_stall, "stall flag set below V_min");
    ck(near(dv.throttle_out, 1.0f, 1e-4f), "stall -> full throttle");
    ck(near(dv.phi_cmd_l, 0.0f, 1e-4f), "stall -> wings level (bank zeroed)");
    ck(dv.ctrl_pitch_out < 0.0f, "stall -> nose-down pitch command");

    // Envelope: bank command beyond PHI_MAX gets clamped in output
    control_init();
    Attitude a2{}; a2.roll = 0;
    ControlCmd c2{}; c2.nav_active = true;
    c2.phi_cmd = 89.0f * (float)M_PI / 180.0f;   // nav asks for absurd bank
    c2.v_cmd = V_CRUISE_MPS; c2.v_cmd = V_CRUISE_MPS;
    ControlDebug d3;
    control_update(a2, c2, V_CRUISE_MPS, 0.0025f, d3);
    ck(d3.env_bank, "bank over PHI_MAX flagged");
    ck(fabsf(d3.phi_cmd_l) <= PHI_MAX_DEG * (float)M_PI / 180.0f + 1e-4f,
       "bank output never exceeds PHI_MAX");
}

// ===========================================================================
// 4. Guidance — L1, turn radius, glide budget, geofence, RTH sequencing
// ===========================================================================
static void test_guidance() {
    printf("[guidance]\n");

    // L1: positive cross-track (target to the right) => right bank (positive)
    float phi_r = l1_bank_cmd(+10.0f, V_CRUISE_MPS, L1_LOOKAHEAD_M);
    float phi_l = l1_bank_cmd(-10.0f, V_CRUISE_MPS, L1_LOOKAHEAD_M);
    ck(phi_r > 0.0f, "L1: target right -> bank right");
    ck(phi_l < 0.0f, "L1: target left  -> bank left");
    ck(near(phi_r, -phi_l, 1e-5f), "L1: symmetric about track");

    // On-track => zero bank
    ck(near(l1_bank_cmd(0.0f, V_CRUISE_MPS, L1_LOOKAHEAD_M), 0.0f, 1e-5f),
       "L1: zero cross-track -> zero bank");

    // L1 never exceeds PHI_MAX
    bool over = false;
    for (float e = -500.0f; e <= 500.0f; e += 10.0f) {
        float p = l1_bank_cmd(e, V_NE_MPS, 5.0f);
        if (fabsf(p) > PHI_MAX_DEG * (float)M_PI / 180.0f + 1e-4f) over = true;
    }
    ck(!over, "L1 output always within PHI_MAX");

    // Turn radius: R = V^2/(g tan phi). 15 m/s @ 45deg ~ 23 m
    float R = turn_radius_m(15.0f, 45.0f * (float)M_PI / 180.0f);
    ck(R > 15.0f && R < 35.0f, "turn radius 15 m/s @45deg in [15,35] m");
    ck(near(R, 15.0f * 15.0f / (9.80665f * 1.0f), 1.5f), "turn radius matches V^2/(g tan45)");
    ck(turn_radius_m(15.0f, 0.0f) > 1e5f, "level flight -> (near) infinite radius");

    // Tighter bank => smaller radius
    ck(turn_radius_m(15.0f, 30.0f * (float)M_PI / 180.0f) >
       turn_radius_m(15.0f, 60.0f * (float)M_PI / 180.0f),
       "more bank -> smaller turn radius");

    // Glide budget: L/D * altitude must exceed distance home
    ck( can_glide_home(200.0f, 50.0f, 10.0f), "10:1 glide from 50 m reaches 200 m");
    ck(!can_glide_home(600.0f, 50.0f, 10.0f), "10:1 glide from 50 m cannot reach 600 m");
    ck(!can_glide_home(10.0f,  50.0f, 0.0f),  "zero L/D never reaches");

    // Geofence circle
    ck( inside_geofence(0.0, 0.0, 0.0, 0.0, 100.0f), "at fence centre -> inside");
    ck(!inside_geofence(0.002, 0.0, 0.0, 0.0, 100.0f), "~222 m away with 100 m fence -> outside");

    // --- RTH sequencing: no home => RTH request ignored (safe) ---
    guidance_init();
    NavState ns{};
    ns.alt_m = 50.0f; ns.ground_mps = V_CRUISE_MPS; ns.course_deg = 0;
    ns.cur_lat = 1.0; ns.cur_lon = 1.0;
    ns.gps_ok = true;
    guidance_trigger_rth();
    guidance_update(ns, 0.04f);
    ck(guidance_phase() == Phase::ARMED_MANUAL || guidance_phase() == Phase::DISARMED,
       "RTH without home set is ignored");

    // With home: RTH engages and commands a bank toward home
    guidance_init();
    guidance_set_home(0.0, 0.0);
    NavState n2{};
    n2.cur_lat = 0.01; n2.cur_lon = 0.0;   // north of home
    n2.alt_m = 50.0f; n2.ground_mps = V_CRUISE_MPS; n2.course_deg = 0.0f;
    guidance_trigger_rth();
    guidance_update(n2, 0.04f);
    ck(guidance_phase() == Phase::NAV_RTH, "RTH engages when home is set");
    ck(n2.dist_home_m > 900.0f && n2.dist_home_m < 1300.0f,
       "distance home ~1.1 km (0.01 deg lat)");
    ck(near(n2.bearing_home_deg, 180.0f, 5.0f), "home bearing ~180deg (south)");
    ck(fabsf(n2.phi_cmd) <= PHI_MAX_DEG * (float)M_PI / 180.0f + 1e-4f,
       "RTH bank within envelope");

    // Geofence breach forces RTH
    guidance_init();
    guidance_set_home(0.0, 0.0);
    guidance_arm();                          // vehicle must be flying (H1)
    NavState n3{};
    n3.cur_lat = 0.05; n3.cur_lon = 0.0;   // ~5.5 km out, fence 300 m
    n3.alt_m = 50.0f; n3.ground_mps = V_CRUISE_MPS; n3.course_deg = 0;
    guidance_update(n3, 0.04f);
    ck(n3.fence_breach, "far outside fence -> breach flagged");
    ck(guidance_phase() == Phase::NAV_RTH, "fence breach forces RTH (armed)");

    // A fence breach must NOT yank a disarmed aircraft into RTH
    guidance_init();
    guidance_set_home(0.0, 0.0);
    NavState n5{};
    n5.cur_lat = 0.05; n5.cur_lon = 0.0;
    n5.alt_m = 50.0f; n5.ground_mps = V_CRUISE_MPS;
    guidance_update(n5, 0.04f);
    ck(n5.fence_breach && guidance_phase() == Phase::DISARMED,
       "fence breach ignored while DISARMED");

    // Altitude above fence max -> also a breach
    guidance_init();
    guidance_set_home(0.0, 0.0);
    NavState n4{}; n4.cur_lat = 0.0; n4.cur_lon = 0.0;
    n4.alt_m = 500.0f; n4.ground_mps = V_CRUISE_MPS;
    guidance_update(n4, 0.04f);
    ck(n4.fence_breach, "altitude above fence max -> breach");
}

// ===========================================================================
// 5. GPS — Haversine, bearing, NMEA parsing
// ===========================================================================
static void test_gps() {
    printf("[gps]\n");

    // Known reference: 1 deg latitude ~ 111.19 km
    double d = haversine_m(0.0, 0.0, 1.0, 0.0);
    ck(d > 110000.0 && d < 112000.0, "1 deg lat ~ 111 km");

    ck(haversine_m(40.0, -3.0, 40.0, -3.0) < 0.5, "zero distance for same point");

    ck(near((float)bearing_deg(0.0, 0.0, 1.0, 0.0), 0.0f, 0.5f),   "north bearing = 0");
    ck(near((float)bearing_deg(0.0, 0.0, -1.0, 0.0), 180.0f, 0.5f), "south bearing = 180");
    ck(near((float)bearing_deg(0.0, 0.0, 0.0, 1.0), 90.0f, 0.5f),   "east bearing = 90");

    // NMEA coordinate conversion: 4807.038 N = 48 + 7.038/60 = 48.1173
    double deg = nmea_to_deg("4807.038", 'N');
    ck(near((float)deg, 48.11730f, 0.001f), "NMEA lat 4807.038N -> 48.1173");
    double w = nmea_to_deg("12311.000", 'W');
    ck(near((float)w, -123.18333f, 0.001f), "NMEA lon 12311.000W -> -123.1833");
    double s = nmea_to_deg("0130.500", 'S');
    ck(s < 0.0, "southern hemisphere is negative");

    // Full sentence parse — checksum computed at runtime
    char s1[128];
    nmea_body(s1, sizeof(s1),
              "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    GpsFix f{};
    bool ok = gps_parse(s1, f);
    ck(ok, "GPGGA sentence parsed");
    ck(f.valid && f.fix_quality == 1, "GGA reports valid fix q=1");
    ck(f.num_sv == 8 && near(f.hdop, 0.9f, 0.01f), "GGA sats=8 hdop=0.9");
    ck(near(f.alt_m, 545.4f, 0.1f), "GGA altitude 545.4");

    char s2[128];
    nmea_body(s2, sizeof(s2),
              "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    GpsFix r{};
    bool ok2 = gps_parse(s2, r);
    ck(ok2, "GPRMC sentence parsed");
    ck(r.valid, "RMC active");
    ck(r.ground_mps > 11.0f && r.ground_mps < 12.0f, "RMC 22.4 kn -> 11.5 m/s");
    ck(near(r.course_deg, 84.4f, 0.1f), "RMC course 84.4");

    // Bad checksum must be rejected: take a valid sentence and corrupt the CS
    char s3[128];
    nmea_body(s3, sizeof(s3),
              "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    char* st = strchr(s3, '*');
    st[1] = '0'; st[2] = '0';          // force a wrong checksum
    GpsFix bad{};
    ck(!gps_parse(s3, bad), "bad checksum rejected");

    // A well-formed sentence with the right checksum but no fix => !valid
    char s4[96];
    nmea_body(s4, sizeof(s4), "GPGGA,123519,,,,,0,00,99.99,,,,,,");
    GpsFix nofix{};
    gps_parse(s4, nofix);
    ck(!nofix.valid && nofix.fix_quality == 0, "GGA with no fix -> invalid");

    // Trust gate (C4): needs sats + HDOP
    GpsFix t{}; t.valid = true; t.fix_quality = 1; t.num_sv = 8; t.hdop = 0.9f;
    ck(gps_trustworthy(t), "good fix trusted");
    t.hdop = 5.0f;
    ck(!gps_trustworthy(t), "HDOP 5.0 not trusted");
    t.hdop = 0.9f; t.num_sv = 3;
    ck(!gps_trustworthy(t), "only 3 sats not trusted");
}

// ===========================================================================
// 6. LD2450 radar frame parser (C7)
// ===========================================================================
static void build_ld2450_frame(uint8_t* b, int16_t x, int16_t y, int16_t spd, uint16_t res) {
    memset(b, 0, LD2450_FRAME_LEN);
    b[0]=0xAA; b[1]=0xFF; b[2]=0x03; b[3]=0x00;   // header
    b[4]=0x01; b[5]=0x00;                          // cmd
    b[6] =  x & 0xFF;        b[7]  = (x >> 8) & 0xFF;
    b[8] =  y & 0xFF;        b[9]  = (y >> 8) & 0xFF;
    b[10]= spd & 0xFF;       b[11] = (spd >> 8) & 0xFF;
    b[12]= res & 0xFF;       b[13] = (res >> 8) & 0xFF;
    // targets 2 and 3 empty (y = 0)
    b[LD2450_FRAME_LEN-2] = 0x55; b[LD2450_FRAME_LEN-1] = 0xCC;   // footer
}

static void test_ld2450() {
    printf("[ld2450]\n");
    uint8_t f[LD2450_FRAME_LEN];
    build_ld2450_frame(f, -250, 3000, -120, 8);

    Ld2450Frame out{};
    ck(ld2450_parse(f, sizeof(f), out), "frame parsed");
    ck(out.count == 1, "one valid target");
    ck(out.targets[0].x_mm == -250, "x = -250 mm (signed)");
    ck(out.targets[0].y_mm == 3000, "y = 3000 mm range");
    ck(out.targets[0].speed_mmps == -120, "speed = -120 mm/s (signed)");
    ck(!out.targets[1].valid && !out.targets[2].valid, "empty slots invalid");

    // Header/footer corruption rejected
    uint8_t bad[LD2450_FRAME_LEN];
    memcpy(bad, f, sizeof(bad)); bad[0] = 0x00;
    Ld2450Frame o2{};
    ck(!ld2450_parse(bad, sizeof(bad), o2), "bad header rejected");
    memcpy(bad, f, sizeof(bad)); bad[LD2450_FRAME_LEN-1] = 0x00;
    ck(!ld2450_parse(bad, sizeof(bad), o2), "bad footer rejected");

    // Misaligned buffer: frame preceded by garbage still found
    uint8_t big[40]; memset(big, 0xEE, 6);
    memcpy(big + 6, f, LD2450_FRAME_LEN);
    Ld2450Frame o3{};
    ck(ld2450_parse(big, sizeof(big), o3) && o3.targets[0].y_mm == 3000,
       "frame found at misaligned offset");

    // Streaming reassembly across byte-at-a-time delivery
    Ld2450Stream st;
    Ld2450Frame got{}; bool done = false;
    for (int i = 0; i < LD2450_FRAME_LEN; i++)
        if (st.feed(f[i], got)) { done = true; }
    ck(done && got.targets[0].y_mm == 3000, "stream reassembles one frame");

    // Track matching: same position matches, far position does not
    Ld2450Frame a, b2;
    build_ld2450_frame((uint8_t[LD2450_FRAME_LEN]){0}, 0, 0, 0, 0); // placeholder
    ld2450_parse(f, sizeof(f), a);
    uint8_t f2[LD2450_FRAME_LEN];
    build_ld2450_frame(f2, -260, 3100, -100, 8);   // moved 100 mm
    ld2450_parse(f2, sizeof(f2), b2);
    int8_t assign[LD2450_MAX_TARGETS];
    ld2450_match_tracks(a, b2, assign);
    ck(assign[0] == 0, "neighbour within gate matched to track 0");

    Ld2450Frame c3;
    uint8_t f3[LD2450_FRAME_LEN];
    build_ld2450_frame(f3, 4000, 5500, 0, 8);      // way far away
    ld2450_parse(f3, sizeof(f3), c3);
    ld2450_match_tracks(a, c3, assign);
    ck(assign[0] == -1, "target beyond 1.5 m gate => no match");
}

// ===========================================================================
// 7. AHRS — converges, responds to tilt
// ===========================================================================
static void test_ahrs() {
    printf("[ahrs]\n");
    ahrs_init();

    Vec3 a{0,0,1}, g{0,0,0}, m{22,0,-40};
    Attitude at{};
    for (int i = 0; i < 400; i++) at = ahrs_update(a, g, m, true, 0.005f);  // 2.0 s
    ck(ahrs_converged(), "AHRS converges within 2 s");
    ck(near(at.roll, 0.0f, 0.02f) && near(at.pitch, 0.0f, 0.02f),
       "level accel -> level attitude");

    // Pitch up: accel shows -x component (nose up => gravity projects on x)
    ahrs_init();
    Vec3 a2{-0.5f, 0.0f, 0.866f};   // ~30 deg nose up
    Attitude a2att{};
    for (int i = 0; i < 400; i++) a2att = ahrs_update(a2, g, m, true, 0.005f);
    ck(a2att.pitch > 15.0f * (float)M_PI / 180.0f, "accel pitch-up detected");

    // Roll right: accel shows +y
    ahrs_init();
    Vec3 a3{0.0f, 0.5f, 0.866f};
    Attitude a3att{};
    for (int i = 0; i < 400; i++) a3att = ahrs_update(a3, g, m, true, 0.005f);
    ck(a3att.roll > 15.0f * (float)M_PI / 180.0f, "accel roll-right detected");

    // Gyro integration with no mag -> yaw drifts but stays wrapped
    ahrs_init();
    Vec3 gz{0,0,0.5f};
    Attitude y{};
    for (int i = 0; i < 1000; i++) y = ahrs_update(Vec3{0,0,1}, gz, m, false, 0.005f);
    ck(fabsf(y.yaw) <= (float)M_PI + 1e-3f, "yaw stays within [-pi, pi]");
    ck(y.yaw > 0.0f, "positive gz integrates to positive yaw");
    ck(!y.mag_ok, "mag_ok false when mag invalid");
}

// ===========================================================================
// 8. Failsafe FSM + preflight gate (H1/H6)
// ===========================================================================
static void test_failsafe() {
    printf("[failsafe]\n");
    failsafe_init();

    // No faults -> nothing requested
    FailsafeStatus s = failsafe_update(FsEvent::NONE, true, 10, true, 0.1f,
                                        3.9f, true, false, false, 1000);
    ck(s.last_event == FsEvent::NONE && !s.rth_requested, "nominal: no failsafe");

    // RC loss -> RTH
    s = failsafe_update(FsEvent::NONE, false, 5000, true, 0.1f,
                        3.9f, true, false, false, 2000);
    ck(s.last_event == FsEvent::RC_LOSS, "RC loss detected");
    ck(s.rth_requested, "RC loss requests RTH");

    // Battery tiers
    s = failsafe_update(FsEvent::NONE, true, 10, true, 0.1f,
                        VBAT_WARN_CELL - 0.01f, true, false, false, 3000);
    ck(s.last_event == FsEvent::BATTERY_WARN, "battery warn tier");
    s = failsafe_update(FsEvent::NONE, true, 10, true, 0.1f,
                        VBAT_RTH_CELL - 0.01f, true, false, false, 3100);
    ck(s.last_event == FsEvent::BATTERY_RTH && s.rth_requested, "battery RTH tier");
    s = failsafe_update(FsEvent::NONE, true, 10, true, 0.1f,
                        VBAT_CRIT_CELL - 0.01f, true, false, false, 3200);
    ck(s.last_event == FsEvent::BATTERY_CRITICAL && s.land_requested,
       "battery critical -> land");

    // IMU loss is the most critical: glide, not RTH
    s = failsafe_update(FsEvent::NONE, true, 10, true, 0.1f,
                        3.9f, false, false, false, 4000);
    ck(s.last_event == FsEvent::IMU_LOSS && s.glide_requested,
       "IMU loss -> glide (not RTH)");

    // Geofence breach -> RTH
    s = failsafe_update(FsEvent::NONE, true, 10, true, 0.1f,
                        3.9f, true, false, true, 5000);
    ck(s.last_event == FsEvent::FENCE_BREACH && s.rth_requested,
       "fence breach -> RTH");

    // Pilot switch
    s = failsafe_update(FsEvent::PILOT_RTH, true, 10, true, 0.1f,
                        3.9f, true, false, false, 6000);
    ck(s.last_event == FsEvent::PILOT_RTH && s.rth_requested, "pilot RTH switch");

    // Event name lookup (used in telemetry)
    ck(strcmp(fs_event_name(FsEvent::STALL), "STALL") == 0, "event name lookup");

    // --- Preflight gate (H6) ---
    GpsFix g{}; g.valid = true; g.fix_quality = 1; g.num_sv = 9; g.hdop = 0.8f;
    PreflightReport pf = preflight_check(true, true, true, g, true,
                                         3.9f, true, true);
    ck(pf.all_ok, "preflight passes when everything healthy");

    pf = preflight_check(true, false, true, g, true, 3.9f, true, true);
    ck(!pf.all_ok && !pf.ahrs_converged, "preflight blocks if AHRS not converged");

    pf = preflight_check(true, true, false, g, true, 3.9f, true, true);
    ck(!pf.all_ok && !pf.mag_ok, "preflight blocks without magnetometer");

    GpsFix nogps{}; nogps.num_sv = 3;
    pf = preflight_check(true, true, true, nogps, true, 3.9f, true, true);
    ck(!pf.all_ok && !pf.gps_ok, "preflight blocks on poor GPS");

    pf = preflight_check(true, true, true, g, true, VBAT_CRIT_CELL - 0.1f, true, true);
    ck(!pf.all_ok && !pf.battery_ok, "preflight blocks on critical battery");

    pf = preflight_check(true, true, true, g, true, 3.9f, true, true);
    ck(pf.envelope_cfg_ok, "envelope config sanity passes (A6 filled)");

    // --- arm / disarm transitions ---
    guidance_init();
    guidance_set_home(40.0, -3.0);          // RTH requires a home (safe default)
    ck(guidance_phase() == Phase::DISARMED, "starts DISARMED");
    guidance_arm();
    ck(guidance_phase() == Phase::ARMED_MANUAL, "arm -> ARMED_MANUAL");
    guidance_trigger_rth();
    ck(guidance_phase() == Phase::NAV_RTH, "RTH while armed");
    guidance_disarm();
    ck(guidance_phase() == Phase::DISARMED, "disarm returns to DISARMED");
}

// ===========================================================================
int main() {
    printf("=== esp32-airplane host tests ===\n\n");
    test_mixing();
    test_envelope();
    test_control();
    test_guidance();
    test_gps();
    test_ld2450();
    test_ahrs();
    test_failsafe();

    printf("\n== %d passed, %d failed ==\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
