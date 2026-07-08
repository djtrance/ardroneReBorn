#ifndef ARDRONE_CONNECTION_H
#define ARDRONE_CONNECTION_H

#include <stdint.h>
#include <netinet/in.h>

#define ARDRONE_CMD_PORT      5556
#define ARDRONE_NAVDATA_PORT  5554
#define ARDRONE_VIDEO_PORT    5000
#define ARDRONE_DEFAULT_IP    "192.168.1.1"
#define ARDRONE_NAVDATA_SIZE  4096

#define ARDRONE_NAVDATA_DEMO_TAG 0

typedef struct {
    char ip[16];
    int cmd_fd;
    int nav_fd;
    struct sockaddr_in cmd_addr;
    int seq;
    int connected;
} ardrone_connection_t;

typedef struct {
    uint32_t header;
    uint32_t state;
    uint32_t sequence;
    uint32_t vision;

    // Demo navdata (parsed)
    int battery;
    float altitude;
    float vx, vy, vz;
    float phi, theta, psi;
} ardrone_navdata_t;

typedef struct __attribute__((packed)) {
    uint16_t tag;
    uint16_t size;
    uint32_t ctrl_state;
    uint32_t battery_percentage;
    uint32_t theta;
    uint32_t phi;
    uint32_t psi;
    int32_t altitude;
    int32_t velocity[3];
} ardrone_navdata_demo_t;

int ardrone_connect(ardrone_connection_t *conn, const char *ip);
int ardrone_send_at(ardrone_connection_t *conn, const char *format, ...);
int ardrone_takeoff(ardrone_connection_t *conn);
int ardrone_land(ardrone_connection_t *conn);
int ardrone_emergency(ardrone_connection_t *conn);
int ardrone_ftrim(ardrone_connection_t *conn);
int ardrone_move(ardrone_connection_t *conn, float roll, float pitch, float gaz, float yaw);
int ardrone_hover(ardrone_connection_t *conn);
int ardrone_config(ardrone_connection_t *conn, const char *key, const char *value);
int ardrone_recv_navdata(ardrone_connection_t *conn, ardrone_navdata_t *nav, int timeout_ms);
void ardrone_disconnect(ardrone_connection_t *conn);

#endif
