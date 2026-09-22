#include "failsafe.h"
#include "config.h"
#include <math.h>

static FailsafeStatus s_fs;

const char* fs_event_name(FsEvent e) {
    switch (e) {
        case FsEvent::NONE:              return "NONE";
        case FsEvent::RC_LOSS:           return "RC_LOSS";
        case FsEvent::GPS_LOSS:          return "GPS_LOSS";
        case FsEvent::BATTERY_WARN:      return "BAT_WARN";
        case FsEvent::BATTERY_RTH:       return "BAT_RTH";
        case FsEvent::BATTERY_CRITICAL:  return "BAT_CRIT";
        case FsEvent::BARO_LOSS:         return "BARO_LOSS";
        case FsEvent::MAG_LOSS:          return "MAG_LOSS";
        case FsEvent::IMU_LOSS:          return "IMU_LOSS";
        case FsEvent::STALL:             return "STALL";
        case FsEvent::FENCE_BREACH:      return "FENCE";
        case FsEvent::MOTOR_FAIL:        return "MOTOR_FAIL";
        case FsEvent::PILOT_RTH:         return "PILOT_RTH";
        default:                         return "?";
    }
}

void failsafe_init() { s_fs = FailsafeStatus{}; }

FailsafeStatus failsafe_update(FsEvent pilot, bool rc_ok, float rc_age_ms,
                               bool gps_ok, float gps_age_s,
                               float vbat_cell,
                               bool imu_ok, bool stall,
                               bool fence_breach, uint32_t now_ms) {
    s_fs.rc_age_ms = rc_age_ms;
    s_fs.gps_age_s = gps_age_s;

    auto fire = [&](FsEvent e) {
        if (s_fs.last_event != e) {
            s_fs.last_event     = e;
            s_fs.last_event_ms  = now_ms;
        }
    };

    // --- IMU failure: most critical — controlled surface neutral + glide (H4)
    if (!imu_ok) {
        fire(FsEvent::IMU_LOSS);
        s_fs.glide_requested   = true;   // FAILSAFE_GLIDE
        s_fs.motors_must_stop  = false;  // wing: keep motor if it still works
        return s_fs;
    }

    // --- RC link loss (H2) -------------------------------------------------
    if (!rc_ok || rc_age_ms > (float)RC_LOSS_TIMEOUT_MS) {
        fire(FsEvent::RC_LOSS);
        s_fs.rth_requested = true;       // G4 RTH on link loss
    }

    // --- Battery tiers (H3) ------------------------------------------------
    if      (vbat_cell <= VBAT_CRIT_CELL) { fire(FsEvent::BATTERY_CRITICAL); s_fs.land_requested = true; }
    else if (vbat_cell <= VBAT_RTH_CELL)  { fire(FsEvent::BATTERY_RTH);      s_fs.rth_requested  = true; }
    else if (vbat_cell <= VBAT_WARN_CELL) { fire(FsEvent::BATTERY_WARN); }

    // --- GPS loss (H4): degrade, do NOT instantly land if flying well -----
    if (!gps_ok || gps_age_s > (float)NAV_LOSS_TIMEOUT_MS / 1000.0f) {
        fire(FsEvent::GPS_LOSS);
        // Decision (H4): hold attitude + return-by-dead-reckoning is unsafe;
        // we hold attitude and land, unless RTH already in progress and
        // coasting on last fix. TODO: finalize in checklist H4.
    }

    // --- Stall (F3) --------------------------------------------------------
    if (stall) fire(FsEvent::STALL);

    // --- Geofence (G5) -----------------------------------------------------
    if (fence_breach) { fire(FsEvent::FENCE_BREACH); s_fs.rth_requested = true; }

    // --- Pilot switch ------------------------------------------------------
    if (pilot == FsEvent::PILOT_RTH) { fire(FsEvent::PILOT_RTH); s_fs.rth_requested = true; }

    return s_fs;
}

PreflightReport preflight_check(bool imu_ok, bool ahrs_converged, bool mag_ok,
                                const GpsFix& gps, bool baro_ok,
                                float vbat_cell, bool rc_ok, bool fence_ok) {
    PreflightReport r{};
    r.imu_ok          = imu_ok;
    r.ahrs_converged  = ahrs_converged;
    r.mag_ok          = mag_ok;
    r.gps_ok          = gps_trustworthy(gps) && gps.num_sv >= 6 &&
                        gps.hdop <= GPS_MAX_HDOP;
    r.baro_ok         = baro_ok;
    r.battery_ok      = vbat_cell > VBAT_CRIT_CELL;
    r.rc_ok           = rc_ok;
    r.fence_ok        = fence_ok;
    // A6 sanity: envelope values must be physically plausible (config filled)
    r.envelope_cfg_ok = (V_STALL_MPS > 1.0f) &&
                        (V_NE_MPS > V_MIN_MPS) &&
                        (V_MIN_MPS > V_STALL_MPS) &&
                        (PHI_MAX_DEG > 5.0f && PHI_MAX_DEG < 80.0f);
    r.all_ok = r.imu_ok && r.ahrs_converged && r.mag_ok && r.gps_ok &&
               r.baro_ok && r.battery_ok && r.rc_ok && r.fence_ok &&
               r.envelope_cfg_ok;
    return r;
}
