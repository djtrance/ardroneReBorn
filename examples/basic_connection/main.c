/*
 * basic_connection — Minimal example: connect to AR.Drone 2.0,
 * send takeoff/land commands, and read navdata.
 *
 * Cross-compile:
 *   cd build && make
 *
 * Deploy:
 *   cd tools && ./deploy.sh ../build/bin/drone_encoder 192.168.1.1
 *
 * Run on drone:
 *   telnet 192.168.1.1
 *   /data/video/drone_encoder
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "connection.h"

static ardrone_connection_t conn;
static volatile int running = 1;

void handle_signal(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    running = 0;
}

int main(int argc, char **argv) {
    const char *ip = (argc > 1) ? argv[1] : ARDRONE_DEFAULT_IP;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("Connecting to AR.Drone 2.0 at %s...\n", ip);

    if (ardrone_connect(&conn, ip) < 0) {
        fprintf(stderr, "Failed to connect\n");
        return 1;
    }

    printf("Connected.\n");

    // Enable navdata
    ardrone_config(&conn, "general:navdata_demo", "TRUE");
    sleep(1);

    // Flat trim (calibrate)
    ardrone_ftrim(&conn);
    sleep(1);

    printf("Sending takeoff command...\n");
    ardrone_takeoff(&conn);

    // Read navdata for 5 seconds
    for (int i = 0; i < 50 && running; i++) {
        ardrone_navdata_t nav;
        if (ardrone_recv_navdata(&conn, &nav, 100) == 0) {
            printf("[%d] bat=%d%% alt=%.1fm vel=(%.2f,%.2f,%.2f) "
                   "rpy=(%.1f,%.1f,%.1f)\n",
                   nav.sequence, nav.battery, nav.altitude,
                   nav.vx, nav.vy, nav.vz,
                   nav.phi, nav.theta, nav.psi);
        }

        // Hover in place
        ardrone_hover(&conn);
        usleep(100000);
    }

    printf("Landing...\n");
    ardrone_land(&conn);
    sleep(2);

    ardrone_disconnect(&conn);
    printf("Disconnected.\n");

    return 0;
}
