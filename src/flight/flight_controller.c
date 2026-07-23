#include "flight_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/*  Utility                                                           */
/* ------------------------------------------------------------------ */

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Degrees to radians */
static double deg2rad(double d) { return d * M_PI / 180.0; }

/* ------------------------------------------------------------------ */
/*  PID controller                                                    */
/* ------------------------------------------------------------------ */

static void pid_reset(pid_state_t *s) {
    s->integral   = 0.0f;
    s->prev_error = 0.0f;
    s->output     = 0.0f;
}

static float pid_compute(pid_state_t *s, const pid_gains_t *g,
                         float error, float dt) {
    /* Proportional */
    float p = g->kp * error;

    /* Integral with anti-windup */
    s->integral += error * dt;
    if (s->integral >  g->imax) s->integral =  g->imax;
    if (s->integral < -g->imax) s->integral = -g->imax;
    float i = g->ki * s->integral;

    /* Derivative */
    float d = 0.0f;
    if (dt > 0.0001f) {
        d = g->kd * (error - s->prev_error) / dt;
    }
    s->prev_error = error;

    /* Sum and saturate */
    s->output = p + i + d;
    s->output = clampf(s->output, -g->out_max, g->out_max);
    return s->output;
}

/* ------------------------------------------------------------------ */
/*  GPS helpers                                                       */
/* ------------------------------------------------------------------ */

/* Approximate meters per degree at given latitude */
static void meters_per_degree(double lat, double *m_per_deg_lat, double *m_per_deg_lon) {
    double lat_rad = deg2rad(lat);
    *m_per_deg_lat = 111132.92 - 559.82 * cos(2.0 * lat_rad)
                     + 1.175 * cos(4.0 * lat_rad);
    *m_per_deg_lon = 111412.84 * cos(lat_rad)
                     - 93.5 * cos(3.0 * lat_rad);
}

/* ------------------------------------------------------------------ */
/*  Default configuration                                             */
/* ------------------------------------------------------------------ */

void flight_default_config(flight_config_t *cfg) {
    memset(cfg, 0, sizeof(flight_config_t));

    /* Velocity PID (m/s error → command -1..1) */
    cfg->pid_vel_x = (pid_gains_t){ .kp = 0.3f, .ki = 0.05f, .kd = 0.1f,
                                    .imax = 0.5f, .out_max = 0.5f };
    cfg->pid_vel_y = (pid_gains_t){ .kp = 0.3f, .ki = 0.05f, .kd = 0.1f,
                                    .imax = 0.5f, .out_max = 0.5f };
    cfg->pid_vel_z = (pid_gains_t){ .kp = 0.4f, .ki = 0.1f,  .kd = 0.15f,
                                    .imax = 0.3f, .out_max = 0.5f };

    /* Position PID (m error → velocity m/s) */
    cfg->pid_pos_lat = (pid_gains_t){ .kp = 0.5f, .ki = 0.02f, .kd = 0.15f,
                                      .imax = 2.0f, .out_max = 2.0f };
    cfg->pid_pos_lon = (pid_gains_t){ .kp = 0.5f, .ki = 0.02f, .kd = 0.15f,
                                      .imax = 2.0f, .out_max = 2.0f };

    /* Altitude PID (m error → vertical velocity m/s) */
    cfg->pid_alt = (pid_gains_t){ .kp = 0.6f, .ki = 0.1f, .kd = 0.2f,
                                  .imax = 1.0f, .out_max = 1.0f };

    /* Yaw PID (deg error → yaw rate -1..1) */
    cfg->pid_yaw = (pid_gains_t){ .kp = 0.02f, .ki = 0.0f, .kd = 0.005f,
                                  .imax = 0.3f, .out_max = 0.3f };

    /* Geofence */
    cfg->geofence.max_distance_m  = 100.0f;
    cfg->geofence.max_altitude_m  = 30.0f;
    cfg->geofence.max_speed_ms    = 3.0f;
    cfg->geofence.min_altitude_m  = 0.5f;

    /* Obstacle avoidance */
    cfg->avoidance.enabled          = true;
    cfg->avoidance.avoidance_speed_ms = 1.0f;
    cfg->avoidance.looming_threshold  = 80;
    cfg->avoidance.asym_threshold     = 40;

    /* Navigation */
    cfg->default_speed_ms  = 1.0f;
    cfg->hover_altitude_m  = 2.0f;
    cfg->rth_altitude_m    = 3.0f;
    cfg->land_speed_ms     = 0.3f;
}

int flight_init(flight_config_t *cfg) {
    if (!cfg) return -1;
    flight_default_config(cfg);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Mode management                                                   */
/* ------------------------------------------------------------------ */

void flight_set_mode(flight_state_t *state, flight_mode_t mode) {
    if (!state) return;
    state->prev_mode = state->mode;
    state->mode = mode;

    /* Reset PID states on mode change */
    pid_reset(&state->pid_vx);
    pid_reset(&state->pid_vy);
    pid_reset(&state->pid_vz);
    pid_reset(&state->pid_lat);
    pid_reset(&state->pid_lon);
    pid_reset(&state->pid_alt);
    pid_reset(&state->pid_yaw);

    printf("[FC] Mode: %d → %d\n", state->prev_mode, mode);
}

void flight_set_target(flight_state_t *state,
                       double lat, double lon, float alt_m) {
    if (!state) return;
    state->target_lat = (float)lat;
    state->target_lon = (float)lon;
    state->target_alt_m = alt_m;
    printf("[FC] Target: %.7f, %.7f @ %.1fm\n", lat, lon, alt_m);
}

void flight_set_home(flight_state_t *state, const gps_state_t *gps) {
    if (!state || !gps || !gps->has_fix) return;
    state->home_lat = (float)gps->pos.lat;
    state->home_lon = (float)gps->pos.lon;
    state->home_alt_m = gps->pos.alt;
    state->home_set = true;
    printf("[FC] Home set: %.7f, %.7f\n", gps->pos.lat, gps->pos.lon);
}

void flight_arm(flight_state_t *state) {
    if (!state) return;
    state->motors_armed = true;
    state->failsafe = false;
    printf("[FC] Motors ARMED\n");
}

void flight_disarm(flight_state_t *state) {
    if (!state) return;
    state->motors_armed = false;
    printf("[FC] Motors DISARMED\n");
}

int flight_emergency_stop(ardrone_connection_t *conn, flight_state_t *state) {
    if (!conn || !state) return -1;
    state->failsafe = true;
    state->motors_armed = false;
    flight_set_mode(state, FLIGHT_MODE_EMERGENCY_LAND);
    printf("[FC] EMERGENCY STOP\n");
    return ardrone_emergency(conn);
}

/* ------------------------------------------------------------------ */
/*  Geofence check                                                    */
/* ------------------------------------------------------------------ */

static bool check_geofence(flight_state_t *state, const flight_config_t *cfg) {
    if (!state->home_set) return false; /* no home = no geofence */

    double mlat, mlon;
    meters_per_degree(state->current_lat, &mlat, &mlon);

    double dlat = (double)(state->current_lat - state->home_lat) * mlat;
    double dlon = (double)(state->current_lon - state->home_lon) * mlon;
    double dist = sqrt(dlat * dlat + dlon * dlon);

    if (dist > cfg->geofence.max_distance_m) {
        printf("[FC] GEOFENCE: distance %.1fm > %.1fm\n",
               dist, cfg->geofence.max_distance_m);
        return true;
    }
    if (state->current_alt_m > cfg->geofence.max_altitude_m) {
        printf("[FC] GEOFENCE: alt %.1fm > %.1fm\n",
               state->current_alt_m, cfg->geofence.max_altitude_m);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/*  Obstacle avoidance logic                                          */
/* ------------------------------------------------------------------ */

static void apply_obstacle_avoidance(flight_state_t *state,
                                     const flight_config_t *cfg,
                                     float *cmd_vx, float *cmd_vy) {
    if (!cfg->avoidance.enabled) return;
    if (!state->obstacle_detected) return;

    float avoid_x = 0.0f, avoid_y = 0.0f;

    /* Looming: reduce forward speed */
    if (state->obstacle_looming > cfg->avoidance.looming_threshold) {
        float scale = 1.0f - (float)state->obstacle_looming / 255.0f;
        *cmd_vx *= scale;
        avoid_x = -cfg->avoidance.avoidance_speed_ms * scale;
        printf("[FC] Avoid looming %d, slowing + backing %.1fm/s\n",
               state->obstacle_looming, avoid_x);
    }

    /* Asymmetry: dodge laterally */
    if (abs(state->obstacle_asymmetry) > cfg->avoidance.asym_threshold) {
        avoid_y = -copysignf(cfg->avoidance.avoidance_speed_ms,
                             (float)state->obstacle_asymmetry);
        printf("[FC] Avoid asym %d, dodge %.1fm/s\n",
               state->obstacle_asymmetry, avoid_y);
    }

    *cmd_vx += avoid_x;
    *cmd_vy += avoid_y;
}

/* ------------------------------------------------------------------ */
/*  Normalize angle to -180..+180 (reserved for heading PID)          */
/* ------------------------------------------------------------------ */

static float __attribute__((unused)) normalize_angle(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* ------------------------------------------------------------------ */
/*  Velocity command → AT*PCMD conversion                             */
/* ------------------------------------------------------------------ */

static void send_velocity_cmd(ardrone_connection_t *conn,
                              float vx_ms, float vy_ms, float vz_ms,
                              float yaw_rate,
                              const flight_config_t *cfg) {
    /* AR.Drone PCMD flag=1: velocity control
     *   roll  = lateral   (+right)
     *   pitch = forward   (+forward)
     *   gaz   = vertical  (+up)
     *   yaw   = angular   (+turn right)
     *
     * The drone's internal controller maps -1..1 to its max velocity
     * (~1 m/s default, ~2 m/s in military mode).
     * We scale our m/s commands to -1..1.
     */
    float max_h = cfg->geofence.max_speed_ms;
    if (max_h < 0.1f) max_h = 1.0f;

    float roll  = clampf(vy_ms / max_h, -1.0f, 1.0f);   /* vy = east */
    float pitch = clampf(vx_ms / max_h, -1.0f, 1.0f);   /* vx = north */
    float gaz   = clampf(vz_ms / 1.0f, -1.0f, 1.0f);   /* vz = up */
    float yaw   = clampf(yaw_rate, -1.0f, 1.0f);

    ardrone_move(conn, roll, pitch, gaz, yaw);
}

/* ------------------------------------------------------------------ */
/*  Main update loop                                                  */
/* ------------------------------------------------------------------ */

int flight_update(ardrone_connection_t *conn,
                  const ardrone_navdata_t *navdata,
                  const gps_state_t *gps,
                  const vision_result_t *vision,
                  flight_state_t *state,
                  const flight_config_t *cfg) {
    if (!conn || !state || !cfg) return -1;
    if (!state->motors_armed) return 0;

    uint32_t now = now_ms();
    float dt = (state->last_update_ms > 0)
               ? (float)(now - state->last_update_ms) / 1000.0f
               : 0.02f; /* 50Hz default */
    state->last_update_ms = now;
    state->loop_hz = (dt > 0.0f) ? (uint32_t)(1.0f / dt) : 50;

    /* --- Update state from sensors --- */

    /* Navdata: attitude + velocity + altitude */
    if (navdata) {
        state->current_heading_deg = navdata->psi;
        state->current_vx_ms = navdata->vx / 1000.0f; /* mm/s → m/s */
        state->current_vy_ms = navdata->vy / 1000.0f;
        state->current_vz_ms = navdata->vz / 1000.0f;
        state->current_alt_m = navdata->altitude / 1000.0f; /* mm → m */
    }

    /* GPS: position */
    if (gps && gps->has_fix) {
        state->current_lat = (float)gps->pos.lat;
        state->current_lon = (float)gps->pos.lon;
        state->current_alt_m = gps->pos.alt;
    }

    /* Vision: obstacle detection */
    if (vision) {
        state->obstacle_looming   = 0;  /* TODO: extract from vision */
        state->obstacle_asymmetry = 0;
        state->obstacle_detected  = false;
    }

    /* --- Geofence check --- */
    state->geofence_breach = check_geofence(state, cfg);
    if (state->geofence_breach && state->mode != FLIGHT_MODE_RTH) {
        printf("[FC] Geofence breach → switching to RTH\n");
        flight_set_mode(state, FLIGHT_MODE_RTH);
    }

    /* --- Mode-specific control --- */

    float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_vz = 0.0f, cmd_yaw = 0.0f;

    switch (state->mode) {

    case FLIGHT_MODE_MANUAL:
        /* No autonomous control, pilot flies */
        break;

    case FLIGHT_MODE_HOVER: {
        /* Position hold: PID on GPS error → velocity → PCMD */
        if (!state->home_set) {
            /* No home set, just hold altitude */
            float alt_err = cfg->hover_altitude_m - state->current_alt_m;
            cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z, alt_err, dt);
            break;
        }

        /* Compute position error in meters */
        double mlat, mlon;
        meters_per_degree(state->current_lat, &mlat, &mlon);

        float err_lat_m = (float)((double)(state->target_lat - state->current_lat) * mlat);
        float err_lon_m = (float)((double)(state->target_lon - state->current_lon) * mlon);

        /* Position PID → desired velocity */
        float des_vx = pid_compute(&state->pid_lat, &cfg->pid_pos_lat, err_lat_m, dt);
        float des_vy = pid_compute(&state->pid_lon, &cfg->pid_pos_lon, err_lon_m, dt);

        /* Velocity PID → command */
        float err_vx = des_vx - state->current_vx_ms;
        float err_vy = des_vy - state->current_vy_ms;
        cmd_vx = pid_compute(&state->pid_vx, &cfg->pid_vel_x, err_vx, dt);
        cmd_vy = pid_compute(&state->pid_vy, &cfg->pid_vel_y, err_vy, dt);

        /* Altitude hold */
        float alt_err = cfg->hover_altitude_m - state->current_alt_m;
        cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z, alt_err, dt);

        break;
    }

    case FLIGHT_MODE_NAVIGATE: {
        /* Fly to target GPS coordinate */
        if (!state->home_set) {
            flight_set_mode(state, FLIGHT_MODE_HOVER);
            break;
        }

        double mlat, mlon;
        meters_per_degree(state->current_lat, &mlat, &mlon);

        float err_lat_m = (float)((double)(state->target_lat - state->current_lat) * mlat);
        float err_lon_m = (float)((double)(state->target_lon - state->current_lon) * mlon);
        float dist = sqrtf(err_lat_m * err_lat_m + err_lon_m * err_lon_m);

        /* Arrived? */
        if (dist < 1.0f) {
            printf("[FC] Arrived at target (%.1fm)\n", dist);
            flight_set_mode(state, FLIGHT_MODE_HOVER);
            break;
        }

        /* Position PID → velocity */
        float des_vx = pid_compute(&state->pid_lat, &cfg->pid_pos_lat, err_lat_m, dt);
        float des_vy = pid_compute(&state->pid_lon, &cfg->pid_pos_lon, err_lon_m, dt);

        /* Clamp to navigation speed */
        float spd = sqrtf(des_vx * des_vx + des_vy * des_vy);
        if (spd > cfg->default_speed_ms) {
            des_vx = des_vx / spd * cfg->default_speed_ms;
            des_vy = des_vy / spd * cfg->default_speed_ms;
        }

        /* Velocity PID → command */
        float err_vx = des_vx - state->current_vx_ms;
        float err_vy = des_vy - state->current_vy_ms;
        cmd_vx = pid_compute(&state->pid_vx, &cfg->pid_vel_x, err_vx, dt);
        cmd_vy = pid_compute(&state->pid_vy, &cfg->pid_vel_y, err_vy, dt);

        /* Altitude hold */
        float alt_err = cfg->hover_altitude_m - state->current_alt_m;
        cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z, alt_err, dt);

        break;
    }

    case FLIGHT_MODE_RTH: {
        /* Return to home */
        if (!state->home_set) {
            printf("[FC] RTH: no home set → emergency land\n");
            flight_set_mode(state, FLIGHT_MODE_EMERGENCY_LAND);
            break;
        }

        double mlat, mlon;
        meters_per_degree(state->current_lat, &mlat, &mlon);

        float err_lat_m = (float)((double)(state->home_lat - state->current_lat) * mlat);
        float err_lon_m = (float)((double)(state->home_lon - state->current_lon) * mlon);
        float dist = sqrtf(err_lat_m * err_lat_m + err_lon_m * err_lon_m);

        /* Phase 1: climb to RTH altitude if too low */
        if (state->current_alt_m < cfg->rth_altitude_m - 0.3f) {
            cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z,
                                 cfg->rth_altitude_m - state->current_alt_m, dt);
            printf("[FC] RTH: climbing %.1f/%.1fm\n",
                   state->current_alt_m, cfg->rth_altitude_m);
            break;
        }

        /* Phase 2: navigate home */
        if (dist > 1.5f) {
            float des_vx = pid_compute(&state->pid_lat, &cfg->pid_pos_lat, err_lat_m, dt);
            float des_vy = pid_compute(&state->pid_lon, &cfg->pid_pos_lon, err_lon_m, dt);

            float spd = sqrtf(des_vx * des_vx + des_vy * des_vy);
            if (spd > cfg->default_speed_ms) {
                des_vx = des_vx / spd * cfg->default_speed_ms;
                des_vy = des_vy / spd * cfg->default_speed_ms;
            }

            float err_vx = des_vx - state->current_vx_ms;
            float err_vy = des_vy - state->current_vy_ms;
            cmd_vx = pid_compute(&state->pid_vx, &cfg->pid_vel_x, err_vx, dt);
            cmd_vy = pid_compute(&state->pid_vy, &cfg->pid_vel_y, err_vy, dt);

            /* Maintain RTH altitude */
            float alt_err = cfg->rth_altitude_m - state->current_alt_m;
            cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z, alt_err, dt);

            printf("[FC] RTH: %.1fm to home\n", dist);
            break;
        }

        /* Phase 3: above home → land */
        printf("[FC] RTH: above home, landing\n");
        cmd_vz = -cfg->land_speed_ms;
        if (state->current_alt_m < 0.3f) {
            printf("[FC] RTH: landed\n");
            flight_set_mode(state, FLIGHT_MODE_HOVER);
            ardrone_land(conn);
        }
        break;
    }

    case FLIGHT_MODE_FOLLOW_ME: {
        /* Same as NAVIGATE but target updates from GPS beacon */
        /* Target is set externally via flight_set_target() */
        if (!state->home_set) {
            flight_set_mode(state, FLIGHT_MODE_HOVER);
            break;
        }

        double mlat, mlon;
        meters_per_degree(state->current_lat, &mlat, &mlon);

        float err_lat_m = (float)((double)(state->target_lat - state->current_lat) * mlat);
        float err_lon_m = (float)((double)(state->target_lon - state->current_lon) * mlon);
        float dist = sqrtf(err_lat_m * err_lat_m + err_lon_m * err_lon_m);

        /* Keep 3m behind target (or hold if close) */
        float follow_dist = 3.0f;
        if (dist < follow_dist) {
            cmd_vx = 0; cmd_vy = 0;
        } else {
            float des_vx = pid_compute(&state->pid_lat, &cfg->pid_pos_lat, err_lat_m, dt);
            float des_vy = pid_compute(&state->pid_lon, &cfg->pid_pos_lon, err_lon_m, dt);

            float spd = sqrtf(des_vx * des_vx + des_vy * des_vy);
            float max_spd = cfg->default_speed_ms * 0.8f;
            if (spd > max_spd) {
                des_vx = des_vx / spd * max_spd;
                des_vy = des_vy / spd * max_spd;
            }

            float err_vx = des_vx - state->current_vx_ms;
            float err_vy = des_vy - state->current_vy_ms;
            cmd_vx = pid_compute(&state->pid_vx, &cfg->pid_vel_x, err_vx, dt);
            cmd_vy = pid_compute(&state->pid_vy, &cfg->pid_vel_y, err_vy, dt);
        }

        /* Altitude hold */
        float alt_err = cfg->hover_altitude_m - state->current_alt_m;
        cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z, alt_err, dt);
        break;
    }

    case FLIGHT_MODE_WAYPOINT: {
        /* Fly mission waypoints */
        if (!state->waypoints || state->current_waypoint >= state->num_waypoints) {
            printf("[FC] Mission complete\n");
            flight_set_mode(state, FLIGHT_MODE_RTH);
            break;
        }

        waypoint_t *wp = &state->waypoints[state->current_waypoint];

        double mlat, mlon;
        meters_per_degree(state->current_lat, &mlat, &mlon);

        float err_lat_m = (float)((double)(wp->lat - state->current_lat) * mlat);
        float err_lon_m = (float)((double)(wp->lon - state->current_lon) * mlon);
        float dist = sqrtf(err_lat_m * err_lat_m + err_lon_m * err_lon_m);

        if (dist < wp->accept_radius_m) {
            printf("[FC] Waypoint %d reached (%.1fm)\n",
                   state->current_waypoint, dist);
            state->current_waypoint++;
            pid_reset(&state->pid_lat);
            pid_reset(&state->pid_lon);
            pid_reset(&state->pid_vx);
            pid_reset(&state->pid_vy);
            break;
        }

        float des_vx = pid_compute(&state->pid_lat, &cfg->pid_pos_lat, err_lat_m, dt);
        float des_vy = pid_compute(&state->pid_lon, &cfg->pid_pos_lon, err_lon_m, dt);

        float spd = sqrtf(des_vx * des_vx + des_vy * des_vy);
        float max_spd = wp->speed_ms > 0 ? wp->speed_ms : cfg->default_speed_ms;
        if (spd > max_spd) {
            des_vx = des_vx / spd * max_spd;
            des_vy = des_vy / spd * max_spd;
        }

        float err_vx = des_vx - state->current_vx_ms;
        float err_vy = des_vy - state->current_vy_ms;
        cmd_vx = pid_compute(&state->pid_vx, &cfg->pid_vel_x, err_vx, dt);
        cmd_vy = pid_compute(&state->pid_vy, &cfg->pid_vel_y, err_vy, dt);

        /* Altitude: fly to waypoint altitude */
        float alt_err = wp->alt_m - state->current_alt_m;
        cmd_vz = pid_compute(&state->pid_vz, &cfg->pid_vel_z, alt_err, dt);
        break;
    }

    case FLIGHT_MODE_EMERGENCY_LAND:
        /* Slow descent */
        cmd_vz = -cfg->land_speed_ms;
        if (state->current_alt_m < 0.3f) {
            printf("[FC] Emergency landed\n");
            ardrone_land(conn);
            state->motors_armed = false;
        }
        break;
    }

    /* --- Obstacle avoidance (applies to all nav modes) --- */
    if (state->mode == FLIGHT_MODE_NAVIGATE ||
        state->mode == FLIGHT_MODE_FOLLOW_ME ||
        state->mode == FLIGHT_MODE_WAYPOINT) {
        apply_obstacle_avoidance(state, cfg, &cmd_vx, &cmd_vy);
    }

    /* --- Send command --- */
    state->cmd_roll  = clampf(cmd_vy / cfg->geofence.max_speed_ms, -1.0f, 1.0f);
    state->cmd_pitch = clampf(cmd_vx / cfg->geofence.max_speed_ms, -1.0f, 1.0f);
    state->cmd_gaz   = clampf(cmd_vz, -1.0f, 1.0f);
    state->cmd_yaw   = clampf(cmd_yaw, -1.0f, 1.0f);

    if (state->mode != FLIGHT_MODE_MANUAL) {
        send_velocity_cmd(conn, cmd_vx, cmd_vy, cmd_vz, cmd_yaw, cfg);
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Waypoint mission                                                  */
/* ------------------------------------------------------------------ */

int flight_load_mission(flight_state_t *state,
                        waypoint_t *waypoints, int count) {
    if (!state || !waypoints || count <= 0) return -1;
    state->waypoints = waypoints;
    state->num_waypoints = count;
    state->current_waypoint = 0;
    printf("[FC] Loaded %d waypoints\n", count);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Telemetry                                                         */
/* ------------------------------------------------------------------ */

void flight_get_telemetry(const flight_state_t *state,
                          float *lat, float *lon, float *alt,
                          float *vx, float *vy, float *vz,
                          float *heading, flight_mode_t *mode) {
    if (!state) return;
    if (lat)     *lat     = state->current_lat;
    if (lon)     *lon     = state->current_lon;
    if (alt)     *alt     = state->current_alt_m;
    if (vx)      *vx      = state->current_vx_ms;
    if (vy)      *vy      = state->current_vy_ms;
    if (vz)      *vz      = state->current_vz_ms;
    if (heading) *heading = state->current_heading_deg;
    if (mode)    *mode    = state->mode;
}
