#ifndef RECOVERY_H
#define RECOVERY_H

#include <stdint.h>

#define NAVDATA_TIMEOUT_MS  2000
#define SC_CMD_TIMEOUT_MS   3000
#define HOVER_TIMEOUT_MS    5000
#define LAND_TIMEOUT_MS     15000

typedef enum {
    RECOVERY_INIT,
    RECOVERY_OK,
    RECOVERY_HOVER,
    RECOVERY_LAND,
    RECOVERY_DRONE_LOST,
} recovery_state_t;

typedef struct {
    uint64_t last_navdata_ms;
    uint64_t last_sc_cmd_ms;
    uint64_t state_start_ms;
    recovery_state_t state;
    int gps_fix;
} recovery_t;

void recovery_init(recovery_t *r);
void recovery_update(recovery_t *r, uint64_t now_ms, int has_navdata, int has_sc_cmd, int gps_fix);
recovery_state_t recovery_tick(recovery_t *r, uint64_t now_ms);
const char *recovery_state_name(recovery_state_t s);

#endif
