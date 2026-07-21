/*
 * test_gps.c — Test GPS NMEA parser
 *
 * Usage:
 *   ./test_gps                      # Auto-detect first GPS serial device
 *   ./test_gps /dev/ttyUSB0 9600    # Specify device and baud rate
 *
 * Output:
 *   Prints GPS position, velocity, time, and satellite info every fix update.
 *
 * Compiles with soft-float toolchain for on-drone use.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "gps.h"

static volatile int g_running = 1;
static int g_update_count = 0;

static void handle_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static void on_gps_update(const gps_state_t *state, void *userdata) {
    (void)userdata;
    g_update_count++;

    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    struct tm *tm = gmtime(&now.tv_sec);

    printf("\r[%02d:%02d:%02d] ", tm->tm_hour, tm->tm_min, tm->tm_sec);

    if (state->has_fix) {
        printf("FIX:%d SAT:%02d HDOP:%.1f "
               "%.6f %.6f ALT:%.0fm SPD:%.1fkn CRS:%.0f° "
               "HOME:%.0fm @%.0f°",
               state->pos.fix_quality,
               state->pos.satellites,
               state->pos.hdop,
               state->pos.lat,
               state->pos.lon,
               state->pos.alt,
               state->vel.speed_kn,
               state->vel.course_deg,
               gps_distance_to_home_m(),
               gps_bearing_to_home_deg());
    } else {
        printf("NO FIX (age: %.1fs)", state->last_fix_age);
    }
    printf("    ");
    fflush(stdout);
}

int main(int argc, char **argv) {
    const char *device = GPS_SERIAL_DEVICE;
    int baud = GPS_BAUD;

    if (argc >= 2) device = argv[1];
    if (argc >= 3) baud = atoi(argv[2]);

    signal(SIGINT, handle_signal);

    printf("=== GPS Test ===\n");
    printf("Device: %s @ %d baud\n", device, baud);

    /* Probe common devices if auto */
    if (argc < 2) {
        const char *devices[] = {
            "/dev/ttyUSB0", "/dev/ttyUSB1",
            "/dev/ttyACM0", "/dev/ttyACM1",
            NULL
        };
        for (int i = 0; devices[i]; i++) {
            if (access(devices[i], F_OK) == 0) {
                device = devices[i];
                printf("Auto-detected: %s\n", device);
                break;
            }
        }
    }

    if (gps_init(device, baud) < 0) {
        fprintf(stderr, "Failed to open GPS.\n");
        fprintf(stderr, "Make sure kernel modules are loaded:\n");
        fprintf(stderr, "  insmod /data/video/usbserial.ko\n");
        fprintf(stderr, "  insmod /data/video/cp210x.ko (or pl2303/ftdi_sio)\n");
        return 1;
    }

    gps_set_callback(on_gps_update, NULL);
    printf("Waiting for GPS fix...\n");

    unsigned long total_updates = 0;
    struct timespec t_start;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    int home_set = 0;

    while (g_running) {
        int n = gps_update();
        if (n > 0) total_updates += n;

        /* Set home position on first 3D fix */
        gps_state_t state;
        gps_get_state(&state);
        if (!home_set && gps_has_3d_fix()) {
            gps_set_home();
            home_set = 1;
        }

        usleep(10000); /* 10ms polling */
    }

    struct timespec t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double elapsed = (t_end.tv_sec - t_start.tv_sec) +
                     (t_end.tv_nsec - t_start.tv_nsec) / 1e9;

    printf("\n\n=== Stats ===\n");
    printf("Total NMEA sentences parsed: %lu\n", total_updates);
    printf("GPS fix updates: %d\n", g_update_count);
    printf("Elapsed: %.1f seconds\n", elapsed);
    printf("Rate: %.1f sentences/sec\n", total_updates / elapsed);

    gps_shutdown();
    return 0;
}
