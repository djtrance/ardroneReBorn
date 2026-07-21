#include "recovery.h"

void recovery_init(recovery_t *r) {
    r->last_navdata_ms = 0;
    r->last_sc_cmd_ms = 0;
    r->state_start_ms = 0;
    r->state = RECOVERY_INIT;
    r->gps_fix = 0;
}

void recovery_update(recovery_t *r, uint64_t now_ms, int has_navdata, int has_sc_cmd, int gps_fix) {
    if (has_navdata) r->last_navdata_ms = now_ms;
    if (has_sc_cmd)  r->last_sc_cmd_ms = now_ms;
    r->gps_fix = gps_fix;
}

recovery_state_t recovery_tick(recovery_t *r, uint64_t now_ms) {
    int navdata_ok = r->last_navdata_ms > 0 && (now_ms - r->last_navdata_ms) < NAVDATA_TIMEOUT_MS;
    int sc_ok = r->last_sc_cmd_ms > 0 && (now_ms - r->last_sc_cmd_ms) < SC_CMD_TIMEOUT_MS;

    if (!navdata_ok && r->last_navdata_ms > 0) {
        r->state = RECOVERY_DRONE_LOST;
        return r->state;
    }

    if (r->last_navdata_ms == 0) {
        r->state = RECOVERY_INIT;
        return r->state;
    }

    if (r->last_sc_cmd_ms == 0) {
        if (r->state != RECOVERY_INIT) {
            r->state = RECOVERY_INIT;
            r->state_start_ms = now_ms;
        }
        return r->state;
    }

    switch (r->state) {
    case RECOVERY_INIT:
        if (navdata_ok && sc_ok) {
            r->state = RECOVERY_OK;
            r->state_start_ms = 0;
        }
        break;

    case RECOVERY_OK:
        if (!sc_ok) {
            r->state = RECOVERY_HOVER;
            r->state_start_ms = now_ms;
        }
        break;

    case RECOVERY_HOVER:
        if (sc_ok) {
            r->state = RECOVERY_OK;
            r->state_start_ms = 0;
        } else if ((now_ms - r->state_start_ms) > HOVER_TIMEOUT_MS) {
            r->state = RECOVERY_LAND;
            r->state_start_ms = now_ms;
        }
        break;

    case RECOVERY_LAND:
        if (sc_ok) {
            r->state = RECOVERY_OK;
            r->state_start_ms = 0;
        } else if ((now_ms - r->state_start_ms) > LAND_TIMEOUT_MS) {
            r->state = RECOVERY_DRONE_LOST;
        }
        break;

    case RECOVERY_DRONE_LOST:
        if (navdata_ok) {
            r->state = RECOVERY_INIT;
            r->state_start_ms = now_ms;
        }
        break;
    }

    return r->state;
}

const char *recovery_state_name(recovery_state_t s) {
    switch (s) {
    case RECOVERY_INIT:       return "INIT";
    case RECOVERY_OK:         return "OK";
    case RECOVERY_HOVER:      return "HOVER";
    case RECOVERY_LAND:       return "LAND";
    case RECOVERY_DRONE_LOST: return "DRONE_LOST";
    default:                  return "?";
    }
}
