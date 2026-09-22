// failsafe.h — failsafe state machine + preflight gate (checklist H1/H6)
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gps_nav.h"
#include "ahrs.h"

// Preflight checks (H6) — any failure blocks arming.
struct PreflightReport {
    bool imu_ok;
    bool ahrs_converged;
    bool mag_ok;
    bool gps_ok;          // fix quality + sats + HDOP (C4)
    bool baro_ok;
    bool battery_ok;
    bool rc_ok;
    bool fence_ok;
    bool envelope_cfg_ok; // A6 values filled & sane
    bool all_ok;
};

enum class FsEvent : uint8_t {
    NONE,
    RC_LOSS,           // H2
    GPS_LOSS,          // H4
    BATTERY_WARN,      // H3
    BATTERY_RTH,       // H3
    BATTERY_CRITICAL,  // H3
    BARO_LOSS,         // H4
    MAG_LOSS,          // H4
    IMU_LOSS,          // H4
    STALL,             // F3
    FENCE_BREACH,      // G5
    MOTOR_FAIL,        // H5
    PILOT_RTH          // manual switch
};

const char* fs_event_name(FsEvent e);

struct FailsafeStatus {
    FsEvent last_event;
    uint32_t last_event_ms;
    bool rth_requested;
    bool glide_requested;
    bool land_requested;
    bool motors_must_stop;
    uint8_t rc_loss_ms;
    uint8_t gps_loss_s;
    float   rc_age_ms;
    float   gps_age_s;
};

void failsafe_init();

// Evaluate all triggers. rc_ok/gps_ok come from their drivers; timestamps in ms.
FailsafeStatus failsafe_update(FsEvent pilot_event,
                               bool rc_ok, float rc_age_ms,
                               bool gps_ok, float gps_age_s,
                               float vbat_cell,
                               bool imu_ok, bool stall,
                               bool fence_breach,
                               uint32_t now_ms);

PreflightReport preflight_check(bool imu_ok, bool ahrs_converged, bool mag_ok,
                                const GpsFix& gps, bool baro_ok,
                                float vbat_cell, bool rc_ok, bool fence_ok);
