#include "connection.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

int ardrone_connect(ardrone_connection_t *conn, const char *ip) {
    if (!conn || !ip) return -1;

    memset(conn, 0, sizeof(ardrone_connection_t));
    strncpy(conn->ip, ip, 15);
    conn->ip[15] = '\0';

    // UDP socket for AT commands (send to drone)
    conn->cmd_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (conn->cmd_fd < 0) {
        perror("socket cmd");
        return -1;
    }

    memset(&conn->cmd_addr, 0, sizeof(conn->cmd_addr));
    conn->cmd_addr.sin_family = AF_INET;
    conn->cmd_addr.sin_port = htons(ARDRONE_CMD_PORT);
    conn->cmd_addr.sin_addr.s_addr = inet_addr(ip);

    // UDP socket for navdata (receive from drone)
    conn->nav_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (conn->nav_fd < 0) {
        perror("socket nav");
        close(conn->cmd_fd);
        return -1;
    }

    struct sockaddr_in nav_bind;
    memset(&nav_bind, 0, sizeof(nav_bind));
    nav_bind.sin_family = AF_INET;
    nav_bind.sin_port = htons(ARDRONE_NAVDATA_PORT);
    nav_bind.sin_addr.s_addr = INADDR_ANY;

    if (bind(conn->nav_fd, (struct sockaddr *)&nav_bind, sizeof(nav_bind)) < 0) {
        perror("bind nav");
        close(conn->cmd_fd);
        close(conn->nav_fd);
        return -1;
    }

    // Set socket timeout for navdata
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(conn->nav_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    conn->seq = 1;
    conn->connected = 1;
    return 0;
}

int ardrone_send_at(ardrone_connection_t *conn, const char *format, ...) {
    if (!conn || !conn->connected) return -1;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    ssize_t sent = sendto(conn->cmd_fd, buffer, strlen(buffer), 0,
                          (struct sockaddr *)&conn->cmd_addr, sizeof(conn->cmd_addr));
    return (sent < 0) ? -1 : 0;
}

int ardrone_takeoff(ardrone_connection_t *conn) {
    return ardrone_send_at(conn, "AT*REF=%d,290718208\r", conn->seq++);
}

int ardrone_land(ardrone_connection_t *conn) {
    return ardrone_send_at(conn, "AT*REF=%d,290717696\r", conn->seq++);
}

int ardrone_emergency(ardrone_connection_t *conn) {
    return ardrone_send_at(conn, "AT*REF=%d,290717952\r", conn->seq++);
}

int ardrone_ftrim(ardrone_connection_t *conn) {
    return ardrone_send_at(conn, "AT*FTRIM=%d,\r", conn->seq++);
}

static inline int float_to_bits(float f) {
    union { float f; int i; } u = { .f = f };
    return u.i;
}

int ardrone_move(ardrone_connection_t *conn, float roll, float pitch, float gaz, float yaw) {
    int _roll = float_to_bits(roll);
    int _pitch = float_to_bits(pitch);
    int _gaz = float_to_bits(gaz);
    int _yaw = float_to_bits(yaw);
    return ardrone_send_at(conn, "AT*PCMD=%d,1,%d,%d,%d,%d\r",
                           conn->seq++, _roll, _pitch, _gaz, _yaw);
}

int ardrone_hover(ardrone_connection_t *conn) {
    return ardrone_move(conn, 0.0f, 0.0f, 0.0f, 0.0f);
}

int ardrone_config(ardrone_connection_t *conn, const char *key, const char *value) {
    return ardrone_send_at(conn, "AT*CONFIG=%d,\"%s\",\"%s\"\r", conn->seq++, key, value);
}

int ardrone_recv_navdata(ardrone_connection_t *conn, ardrone_navdata_t *nav, int timeout_ms) {
    if (!conn || !conn->connected || !nav) return -1;

    struct timeval tv = { .tv_sec = timeout_ms / 1000, .tv_usec = (timeout_ms % 1000) * 1000 };
    setsockopt(conn->nav_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buffer[ARDRONE_NAVDATA_SIZE];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);

    ssize_t n = recvfrom(conn->nav_fd, buffer, sizeof(buffer), 0,
                         (struct sockaddr *)&from, &from_len);
    if (n < 0) return -1;

    // Parse navdata options
    nav->header = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
    nav->state = buffer[4] | (buffer[5] << 8) | (buffer[6] << 16) | (buffer[7] << 24);
    nav->sequence = buffer[8] | (buffer[9] << 8) | (buffer[10] << 16) | (buffer[11] << 24);
    nav->vision = buffer[12] | (buffer[13] << 8) | (buffer[14] << 16) | (buffer[15] << 24);

    int offset = 16;

    while (offset + 4 < n) {
        int tag = buffer[offset] | (buffer[offset + 1] << 8);
        int size = buffer[offset + 2] | (buffer[offset + 3] << 8);

        if (tag == ARDRONE_NAVDATA_DEMO_TAG && (unsigned int)size >= sizeof(ardrone_navdata_demo_t)) {
            ardrone_navdata_demo_t *demo = (ardrone_navdata_demo_t *)&buffer[offset];
            nav->battery = demo->battery_percentage;
            nav->altitude = demo->altitude;
            nav->vx = demo->velocity[0];
            nav->vy = demo->velocity[1];
            nav->vz = demo->velocity[2];
            nav->phi = demo->phi / 1000.0f;
            nav->theta = demo->theta / 1000.0f;
            nav->psi = demo->psi / 1000.0f;
        }

        offset += size;
        if (offset % 4 != 0) offset += 4 - (offset % 4);
    }

    return 0;
}

void ardrone_disconnect(ardrone_connection_t *conn) {
    if (conn) {
        if (conn->cmd_fd >= 0) close(conn->cmd_fd);
        if (conn->nav_fd >= 0) close(conn->nav_fd);
        memset(conn, 0, sizeof(ardrone_connection_t));
    }
}
