#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include "../connection.h"
#include "../navigation/gps.h"
#include "../vision/types.h"
#include "../flight/flight_controller.h"

static volatile int running = 1;

static void sighandler(int sig) {
    (void)sig;
    running = 0;
    printf("\n[TEST] SIGINT received, stopping...\n");
}

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static const char* mode_name(flight_mode_t m) {
    switch (m) {
        case FLIGHT_MODE_MANUAL:       return "MANUAL";
        case FLIGHT_MODE_HOVER:        return "HOVER";
        case FLIGHT_MODE_NAVIGATE:     return "NAVIGATE";
        case FLIGHT_MODE_RTH:          return "RTH";
        case FLIGHT_MODE_FOLLOW_ME:    return "FOLLOW_ME";
        case FLIGHT_MODE_WAYPOINT:     return "WAYPOINT";
        case FLIGHT_MODE_EMERGENCY_LAND: return "EMERGENCY_LAND";
        default: return "UNKNOWN";
    }
}

static void print_telemetry(uint32_t t, const flight_state_t *s) {
    printf("[%5u.%03u] mode=%-10s lat=%.7f lon=%.7f alt=%.1f "
           "vx=%.1f vy=%.1f vz=%.1f hdg=%.0f "
           "cmd[%.2f,%.2f,%.2f,%.2f] %s%s\n",
           t / 1000, t % 1000,
           mode_name(s->mode),
           s->current_lat, s->current_lon, s->current_alt_m,
           s->current_vx_ms, s->current_vy_ms, s->current_vz_ms,
           s->current_heading_deg,
           s->cmd_roll, s->cmd_pitch, s->cmd_gaz, s->cmd_yaw,
           s->geofence_breach ? " [GEOFENCE]" : "",
           s->failsafe ? " [FAILSAFE]" : "");
}

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n"
           "\n"
           "Options:\n"
           "  -i <ip>       Drone IP (default: 192.168.1.1)\n"
           "  -d <device>   Camera device (default: /dev/video0)\n"
           "  -g <device>   GPS device (default: /dev/ttyUSB0)\n"
           "  -t <seconds>  Test duration (default: 30)\n"
           "  -m <mode>     Initial mode: hover, navigate, rth, follow\n"
           "  -s <lat,lon>  Target for navigate mode\n"
           "  -n            Dry run (no AT commands, just telemetry)\n"
           "  -v            Verbose output\n"
           "  -h            This help\n"
           "\n"
           "Modes:\n"
           "  hover       - Hold current GPS position\n"
           "  navigate    - Fly to target GPS coordinate\n"
           "  rth         - Return to home\n"
           "  follow      - Follow GPS beacon (sets target repeatedly)\n"
           "\n"
           "Examples:\n"
           "  %s -m hover              # Take off, hold position\n"
           "  %s -m navigate -s 40.7128,-74.0060  # Fly to NYC\n"
           "  %s -m rth                # Return to home\n",
           prog, prog, prog, prog);
}

int main(int argc, char *argv[]) {
    const char *drone_ip = ARDRONE_DEFAULT_IP;
    const char *cam_dev  = "/dev/video0";
    const char *gps_dev  = "/dev/ttyUSB0";
    int test_duration    = 30;
    flight_mode_t mode   = FLIGHT_MODE_HOVER;
    int dry_run          = 0;
    int verbose          = 0;
    double target_lat    = 0.0, target_lon = 0.0;
    int have_target      = 0;

    /* Parse arguments */
    int opt;
    while ((opt = getopt(argc, argv, "i:d:g:t:m:s:nvh")) != -1) {
        switch (opt) {
            case 'i': drone_ip = optarg; break;
            case 'd': cam_dev = optarg; break;
            case 'g': gps_dev = optarg; break;
            case 't': test_duration = atoi(optarg); break;
            case 'm':
                if (strcmp(optarg, "hover") == 0)       mode = FLIGHT_MODE_HOVER;
                else if (strcmp(optarg, "navigate") == 0) mode = FLIGHT_MODE_NAVIGATE;
                else if (strcmp(optarg, "rth") == 0)      mode = FLIGHT_MODE_RTH;
                else if (strcmp(optarg, "follow") == 0)   mode = FLIGHT_MODE_FOLLOW_ME;
                else {
                    fprintf(stderr, "Unknown mode: %s\n", optarg);
                    return 1;
                }
                break;
            case 's':
                if (sscanf(optarg, "%lf,%lf", &target_lat, &target_lon) == 2) {
                    have_target = 1;
                } else {
                    fprintf(stderr, "Invalid target: %s (use lat,lon)\n", optarg);
                    return 1;
                }
                break;
            case 'n': dry_run = 1; break;
            case 'v': verbose = 1; break;
            case 'h': print_usage(argv[0]); return 0;
            default:  print_usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    printf("=== Flight Controller Test ===\n");
    printf("Drone: %s, Camera: %s, GPS: %s\n", drone_ip, cam_dev, gps_dev);
    printf("Mode: %s, Duration: %ds, Dry run: %s\n",
           mode_name(mode), test_duration, dry_run ? "yes" : "no");

    /* --- Connect to drone --- */
    ardrone_connection_t conn;
    printf("\n[1] Connecting to drone at %s...\n", drone_ip);
    if (ardrone_connect(&conn, drone_ip) < 0) {
        fprintf(stderr, "Failed to connect to drone\n");
        return 1;
    }
    printf("    Connected (cmd_fd=%d, nav_fd=%d)\n", conn.cmd_fd, conn.nav_fd);

    /* --- Enable navdata --- */
    ardrone_config(&conn, "general:navdata_demo", "TRUE");

    /* --- Init GPS (optional) --- */
    gps_state_t gps;
    memset(&gps, 0, sizeof(gps));
    int gps_ok = 0;
    printf("\n[2] Initializing GPS on %s...\n", gps_dev);
    if (gps_init(gps_dev, 9600) == 0) {
        printf("    GPS initialized\n");
        gps_ok = 1;
    } else {
        printf("    GPS not available, using navdata altitude only\n");
    }

    /* --- Init flight controller --- */
    flight_config_t fcfg;
    flight_init(&fcfg);

    flight_state_t fstate;
    memset(&fstate, 0, sizeof(fstate));
    fstate.mode = FLIGHT_MODE_MANUAL;

    /* --- Takeoff --- */
    if (!dry_run) {
        printf("\n[3] Taking off...\n");
        ardrone_takeoff(&conn);
        usleep(2000000); /* wait 2s for takeoff */
    }

    /* --- Set home position --- */
    if (gps_ok) {
        gps_update();
        gps_get_state(&gps);
        if (gps.has_fix) {
            flight_set_home(&fstate, &gps);
            printf("    Home: %.7f, %.7f @ %.1fm\n",
                   gps.pos.lat, gps.pos.lon, gps.pos.alt);
        }
    }

    /* --- Arm and set mode --- */
    flight_arm(&fstate);
    fstate.target_alt_m = fcfg.hover_altitude_m;

    if (have_target) {
        flight_set_target(&fstate, target_lat, target_lon, fcfg.hover_altitude_m);
    } else {
        /* Use current position as target for hover */
        if (fstate.home_set) {
            flight_set_target(&fstate, fstate.home_lat, fstate.home_lon,
                              fcfg.hover_altitude_m);
        }
    }

    flight_set_mode(&fstate, mode);

    /* --- Main loop --- */
    printf("\n[4] Running flight controller at 50Hz for %ds...\n", test_duration);
    printf("    Press Ctrl+C to stop\n\n");

    ardrone_navdata_t navdata;
    uint32_t start = now_ms();
    uint32_t last_log = 0;
    int loop_count = 0;

    while (running && (now_ms() - start) < (uint32_t)test_duration * 1000) {
        uint32_t loop_start = now_ms();

        /* Read navdata */
        if (ardrone_recv_navdata(&conn, &navdata, 10) == 0) {
            if (verbose) {
                printf("    navdata: batt=%d%% alt=%dmm vx=%d vy=%d vz=%d "
                       "phi=%.1f theta=%.1f psi=%.1f\n",
                       navdata.battery, (int)navdata.altitude,
                       (int)navdata.vx, (int)navdata.vy, (int)navdata.vz,
                       navdata.phi, navdata.theta, navdata.psi);
            }
        }

        /* Read GPS */
        if (gps_ok) {
            gps_update();
            gps_get_state(&gps);

            /* Update follow-me target from GPS (simulated) */
            if (mode == FLIGHT_MODE_FOLLOW_ME && gps.has_fix) {
                flight_set_target(&fstate, gps.pos.lat, gps.pos.lon,
                                  fcfg.hover_altitude_m);
            }
        }

        /* Vision stub (no camera in this test) */
        vision_result_t vision;
        memset(&vision, 0, sizeof(vision));

        /* Run flight controller */
        if (!dry_run) {
            flight_update(&conn, &navdata, &gps, &vision, &fstate, &fcfg);
        } else {
            /* Dry run: just update state for telemetry */
            if (navdata.altitude > 0) {
                fstate.current_alt_m = navdata.altitude / 1000.0f;
            }
            fstate.current_vx_ms = navdata.vx / 1000.0f;
            fstate.current_vy_ms = navdata.vy / 1000.0f;
            fstate.current_vz_ms = navdata.vz / 1000.0f;
            fstate.current_heading_deg = navdata.psi;
        }

        /* Log telemetry every 500ms */
        uint32_t elapsed = now_ms() - start;
        if (elapsed - last_log >= 500) {
            print_telemetry(elapsed, &fstate);
            last_log = elapsed;
        }

        loop_count++;

        /* Maintain 50Hz (20ms per loop) */
        uint32_t elapsed_loop = now_ms() - loop_start;
        if (elapsed_loop < 20) {
            usleep((20 - elapsed_loop) * 1000);
        }
    }

    /* --- Cleanup --- */
    printf("\n[5] Stopping...\n");

    if (!dry_run) {
        ardrone_hover(&conn);
        usleep(1000000);
        ardrone_land(&conn);
        usleep(2000000);
    }

    flight_disarm(&fstate);

    float avg_hz = (loop_count > 0 && test_duration > 0)
                   ? (float)loop_count / (float)test_duration : 0;
    printf("\n=== Results ===\n");
    printf("Loops: %d, Avg: %.1f Hz\n", loop_count, avg_hz);
    printf("Final alt: %.1fm, Final mode: %s\n",
           fstate.current_alt_m, mode_name(fstate.mode));

    gps_shutdown();
    ardrone_disconnect(&conn);

    printf("Done.\n");
    return 0;
}
