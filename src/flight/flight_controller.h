#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "../connection.h"
#include "../navigation/gps.h"
#include "../vision/types.h"

/* Flight controller for AR.Drone 2.0
 *
 * Integrates navdata (attitude, altitude, velocity) + GPS (position) +
 * vision (optical flow, obstacle detection) to command the drone via
 * AT*PCMD commands.
 *
 * Modes:
 *   MANUAL     - pilot controls via RC, FC monitors only
 *   HOVER      - hold current position (GPS + altitude)
 *   NAVIGATE   - fly to target GPS coordinate
 *   RTH        - return to home position
 *   FOLLOW_ME  - follow GPS beacon (phone/GPS tracker)
 *   WAYPOINT   - fly mission waypoint list
 */

/* Flight modes */
typedef enum {
    FLIGHT_MODE_MANUAL = 0,
    FLIGHT_MODE_HOVER,
    FLIGHT_MODE_NAVIGATE,
    FLIGHT_MODE_RTH,
    FLIGHT_MODE_FOLLOW_ME,
    FLIGHT_MODE_WAYPOINT,
    FLIGHT_MODE_EMERGENCY_LAND
} flight_mode_t;

/* PID gains */
typedef struct {
    float kp;
    float ki;
    float kd;
    float imax;    /* integral anti-windup clamp */
    float out_max; /* output saturation */
} pid_gains_t;

/* PID state */
typedef struct {
    float integral;
    float prev_error;
    float output;
} pid_state_t;

/* Geofence limits */
typedef struct {
    float max_distance_m;   /* max distance from home */
    float max_altitude_m;   /* max altitude AGL */
    float max_speed_ms;     /* max horizontal speed */
    float min_altitude_m;   /* minimum altitude (safety) */
} geofence_t;

/* Obstacle avoidance config */
typedef struct {
    bool  enabled;
    float avoidance_speed_ms; /* speed to deviate around obstacle */
    uint8_t looming_threshold; /* 0-255, trigger avoidance */
    uint8_t asym_threshold;    /* 0-127, trigger lateral dodge */
} avoidance_t;

/* Waypoint */
typedef struct {
    double lat;
    double lon;
    float  alt_m;
    float  speed_ms;      /* speed to fly to this waypoint */
    float  accept_radius_m; /* "arrived" threshold */
} waypoint_t;

/* Full flight controller config */
typedef struct {
    /* PID gains */
    pid_gains_t pid_vel_x;   /* velocity X (north) PID */
    pid_gains_t pid_vel_y;   /* velocity Y (east) PID */
    pid_gains_t pid_vel_z;   /* velocity Z (altitude) PID */
    pid_gains_t pid_pos_lat; /* latitude position PID */
    pid_gains_t pid_pos_lon; /* longitude position PID */
    pid_gains_t pid_alt;     /* altitude hold PID */
    pid_gains_t pid_yaw;     /* yaw heading PID */

    /* Geofence */
    geofence_t geofence;

    /* Obstacle avoidance */
    avoidance_t avoidance;

    /* Navigation */
    float default_speed_ms;  /* default navigation speed */
    float hover_altitude_m;  /* target hover altitude */
    float rth_altitude_m;    /* altitude during RTH */
    float land_speed_ms;     /* descent speed */
} flight_config_t;

/* Flight controller state */
typedef struct {
    /* Mode */
    flight_mode_t mode;
    flight_mode_t prev_mode;

    /* PID states */
    pid_state_t pid_vx;
    pid_state_t pid_vy;
    pid_state_t pid_vz;
    pid_state_t pid_lat;
    pid_state_t pid_lon;
    pid_state_t pid_alt;
    pid_state_t pid_yaw;

    /* Current state */
    float current_lat;
    float current_lon;
    float current_alt_m;
    float current_vx_ms;  /* m/s north */
    float current_vy_ms;  /* m/s east */
    float current_vz_ms;  /* m/s up */
    float current_heading_deg;

    /* Target state */
    float target_lat;
    float target_lon;
    float target_alt_m;
    float target_heading_deg;

    /* Home position */
    float home_lat;
    float home_lon;
    float home_alt_m;
    bool  home_set;

    /* Obstacle state */
    uint8_t obstacle_looming;
    int8_t  obstacle_asymmetry;
    bool    obstacle_detected;

    /* Geofence violation */
    bool geofence_breach;

    /* Waypoints */
    waypoint_t *waypoints;
    int num_waypoints;
    int current_waypoint;

    /* Timing */
    uint32_t last_update_ms;
    uint32_t loop_hz;

    /* Output commands (for telemetry/logging) */
    float cmd_roll;   /* -1..1 */
    float cmd_pitch;  /* -1..1 */
    float cmd_gaz;    /* -1..1 (throttle) */
    float cmd_yaw;    /* -1..1 */

    /* Safety */
    bool motors_armed;
    bool failsafe;
    uint32_t failsafe_timer_ms;
} flight_state_t;

/* Initialize flight controller with default config */
int flight_init(flight_config_t *cfg);

/* Get default configuration */
void flight_default_config(flight_config_t *cfg);

/* Main update loop - call at fixed rate (e.g., 50Hz)
 *
 * Reads navdata + GPS + vision, computes commands, sends AT*PCMD.
 * Returns 0 on success, negative on error.
 */
int flight_update(ardrone_connection_t *conn,
                  const ardrone_navdata_t *navdata,
                  const gps_state_t *gps,
                  const vision_result_t *vision,
                  flight_state_t *state,
                  const flight_config_t *cfg);

/* Set flight mode */
void flight_set_mode(flight_state_t *state, flight_mode_t mode);

/* Set navigation target */
void flight_set_target(flight_state_t *state,
                       double lat, double lon, float alt_m);

/* Set home position from current GPS */
void flight_set_home(flight_state_t *state, const gps_state_t *gps);

/* Arm/disarm motors */
void flight_arm(flight_state_t *state);
void flight_disarm(flight_state_t *state);

/* Emergency stop - kill motors immediately */
int flight_emergency_stop(ardrone_connection_t *conn, flight_state_t *state);

/* Load waypoint mission */
int flight_load_mission(flight_state_t *state,
                        waypoint_t *waypoints, int count);

/* Get current flight telemetry */
void flight_get_telemetry(const flight_state_t *state,
                          float *lat, float *lon, float *alt,
                          float *vx, float *vy, float *vz,
                          float *heading, flight_mode_t *mode);

#endif /* FLIGHT_CONTROLLER_H */
