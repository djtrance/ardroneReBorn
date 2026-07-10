#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "connection.h"

static volatile int g_running = 1;

static void handle_signal(int sig) {
  (void)sig;
  g_running = 0;
}

int main(int argc, char **argv) {
  const char *ip = argc > 1 ? argv[1] : ARDRONE_DEFAULT_IP;

  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  ardrone_connection_t conn;
  if (ardrone_connect(&conn, ip) != 0) {
    fprintf(stderr, "Failed to connect to %s\n", ip);
    return 1;
  }
  printf("Connected to %s\n", ip);

  /* Send config to enable navdata */
  ardrone_config(&conn, "general:navdata_demo", "TRUE");
  usleep(100000);

  /* Send a few AT commands to establish communication */
  ardrone_send_at(&conn, "AT*PCMD=%d,0,0,0,0,0\r", conn.seq++);
  ardrone_send_at(&conn, "AT*COMWDG=%d\r", conn.seq++);

  printf("Receiving navdata (Ctrl+C to stop)...\n");
  printf("%-6s %-8s %-8s %-8s %-8s %-8s\n",
         "batt%", "alt(cm)", "vx", "vy", "vz", "heading");

  int frames = 0;
  while (g_running && frames < 300) {
    ardrone_navdata_t nav;
    if (ardrone_recv_navdata(&conn, &nav, 1000) == 0) {
      printf("%-6d %-8.0f %-8.0f %-8.0f %-8.0f %-8.0f\r",
             nav.battery, (double)nav.altitude,
             (double)nav.vx, (double)nav.vy, (double)nav.vz,
             (double)nav.psi);
      fflush(stdout);
      frames++;
    }
  }

  printf("\nDisconnected after %d navdata frames\n", frames);
  ardrone_disconnect(&conn);
  return 0;
}
