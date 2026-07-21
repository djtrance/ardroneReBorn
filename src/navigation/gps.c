#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define EARTH_RADIUS_M 6371000.0
#define KNOT_TO_MPS 0.514444

static int gps_fd = -1;
static gps_state_t g_state;
static gps_callback_t g_callback = NULL;
static void *g_callback_data = NULL;

/* Home position */
static bool home_set = false;
static double home_lat, home_lon;

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* --- NMEA sentence parsing --- */

/* Parse $GPGGA - GPS Fix Data */
static int parse_gga(const char *sentence) {
    /* $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47 */
    double lat_deg = 0, lon_deg = 0;
    float alt = 0, hdop = 99.9;
    int fix = 0, sats = 0;
    char lat_ns = 0, lon_ew = 0;
    char alt_unit = 0, geo_unit = 0;

    int n = sscanf(sentence,
        "$GPGGA,%*d,%lf,%c,%lf,%c,%d,%d,%f,%f,%c,%*f,%c,%*f,%*f",
        &lat_deg, &lat_ns, &lon_deg, &lon_ew,
        &fix, &sats, &hdop, &alt, &alt_unit, &geo_unit);

    if (n < 9) return -1;

    /* Convert NMEA format (DDMM.MMMM) to decimal degrees */
    double lat_deg_int = (int)(lat_deg / 100);
    double lat_min = lat_deg - lat_deg_int * 100;
    g_state.pos.lat = lat_deg_int + lat_min / 60.0;
    if (lat_ns == 'S') g_state.pos.lat = -g_state.pos.lat;

    double lon_deg_int = (int)(lon_deg / 100);
    double lon_min = lon_deg - lon_deg_int * 100;
    g_state.pos.lon = lon_deg_int + lon_min / 60.0;
    if (lon_ew == 'W') g_state.pos.lon = -g_state.pos.lon;

    g_state.pos.alt = alt;
    g_state.pos.hdop = hdop;
    g_state.pos.fix_quality = fix;
    g_state.pos.satellites = sats;

    if (fix >= 1) {
        g_state.has_fix = true;
        g_state.last_update_us = now_us();
    }

    return 0;
}

/* Parse $GPRMC - Recommended Minimum Navigation Info */
static int parse_rmc(const char *sentence) {
    /* $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A */
    double lat_deg = 0, lon_deg = 0, speed_kn = 0, course_d = 0;
    char lat_ns = 0, lon_ew = 0, status = 'V';  /* V=invalid, A=valid */
    int time_h = 0, time_m = 0, time_s = 0;
    int date_d = 0, date_m = 0, date_y = 0;

    int n = sscanf(sentence,
        "$GPRMC,%2d%2d%2d,%c,%lf,%c,%lf,%c,%lf,%lf,%2d%2d%2d,%*f,%c",
        &time_h, &time_m, &time_s, &status,
        &lat_deg, &lat_ns, &lon_deg, &lon_ew,
        &speed_kn, &course_d,
        &date_d, &date_m, &date_y, &lon_ew);

    if (n < 13) return -1;

    if (status == 'A') {
        double lat_deg_int = (int)(lat_deg / 100);
        double lat_min = lat_deg - lat_deg_int * 100;
        g_state.pos.lat = lat_deg_int + lat_min / 60.0;
        if (lat_ns == 'S') g_state.pos.lat = -g_state.pos.lat;

        double lon_deg_int = (int)(lon_deg / 100);
        double lon_min = lon_deg - lon_deg_int * 100;
        g_state.pos.lon = lon_deg_int + lon_min / 60.0;
        if (lon_ew == 'W') g_state.pos.lon = -g_state.pos.lon;

        g_state.vel.speed_kn = (float)speed_kn;
        g_state.vel.course_deg = (float)course_d;

        g_state.time.hour = time_h;
        g_state.time.minute = time_m;
        g_state.time.second = time_s;
        g_state.time.day = date_d;
        g_state.time.month = date_m;
        g_state.time.year = 2000 + date_y;

        g_state.has_fix = true;
        g_state.last_update_us = now_us();
    }

    return 0;
}

/* Parse $GPGSA - DOP and active satellites */
static int parse_gsa(const char *sentence) {
    /* $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39 */
    (void)sentence;
    return 0;  /* Placeholder - we get hdop from GGA */
}

/* Route NMEA sentence to appropriate parser */
static void parse_nmea(const char *sentence) {
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
        strncmp(sentence, "$GNGGA", 6) == 0) {
        parse_gga(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 ||
               strncmp(sentence, "$GNRMC", 6) == 0) {
        parse_rmc(sentence);
    } else if (strncmp(sentence, "$GPGSA", 6) == 0 ||
               strncmp(sentence, "$GNGSA", 6) == 0) {
        parse_gsa(sentence);
    }
}

/* --- Serial port --- */

static int set_baud(int fd, int baud) {
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) < 0) return -1;

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  /* 100ms timeout */

    speed_t speed;
    switch (baud) {
    case 4800:   speed = B4800;   break;
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:     speed = B9600;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    return tcsetattr(fd, TCSANOW, &tio);
}

int gps_init(const char *device, int baud) {
    if (gps_fd >= 0) close(gps_fd);

    gps_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (gps_fd < 0) {
        fprintf(stderr, "GPS: Cannot open %s: %s\n", device, strerror(errno));
        return -1;
    }

    if (set_baud(gps_fd, baud) < 0) {
        fprintf(stderr, "GPS: Cannot set baud rate: %s\n", strerror(errno));
        close(gps_fd);
        gps_fd = -1;
        return -1;
    }

    memset(&g_state, 0, sizeof(g_state));
    g_state.pos.hdop = 99.9;
    printf("GPS: Opened %s @ %d baud\n", device, baud);
    return 0;
}

int gps_update(void) {
    if (gps_fd < 0) return -1;

    static char buf[GPS_BUFFER_SIZE];
    static int pos = 0;

    int n = read(gps_fd, buf + pos, sizeof(buf) - 1 - pos);
    if (n <= 0) return 0;

    pos += n;
    buf[pos] = 0;

    int parsed = 0;
    char *line = buf;
    char *nl;

    while ((nl = strchr(line, '\n')) != NULL) {
        *nl = 0;

        char *cr = strchr(line, '\r');
        if (cr) *cr = 0;

        if (line[0] == '$' && strlen(line) > 4) {
            parse_nmea(line);
            parsed++;
        }

        line = nl + 1;
    }

    /* Move remaining partial data to start of buffer */
    if (line > buf && line < buf + pos) {
        int remaining = (buf + pos) - line;
        memmove(buf, line, remaining);
        pos = remaining;
    } else {
        pos = 0;
    }

    /* Update fix age */
    if (g_state.has_fix && g_state.last_update_us > 0) {
        uint64_t age = now_us() - g_state.last_update_us;
        g_state.last_fix_age = age / 1000000.0f;
        if (g_state.last_fix_age > 10.0f) {
            /* Fix too old, mark as stale */
            g_state.has_fix = false;
        }
    }

    /* Notify on new data */
    if (parsed > 0 && g_callback) {
        g_callback(&g_state, g_callback_data);
    }

    return parsed;
}

void gps_get_state(gps_state_t *out) {
    if (out) memcpy(out, &g_state, sizeof(g_state));
}

void gps_set_callback(gps_callback_t cb, void *userdata) {
    g_callback = cb;
    g_callback_data = userdata;
}

void gps_set_home(void) {
    if (g_state.has_fix) {
        home_lat = g_state.pos.lat;
        home_lon = g_state.pos.lon;
        home_set = true;
        printf("GPS: Home set at %.6f, %.6f\n", home_lat, home_lon);
    } else {
        fprintf(stderr, "GPS: Cannot set home — no fix\n");
    }
}

float gps_distance_to_home_m(void) {
    if (!home_set || !g_state.has_fix) return -1.0f;
    return gps_distance_between(home_lat, home_lon,
                                g_state.pos.lat, g_state.pos.lon);
}

float gps_bearing_to_home_deg(void) {
    if (!home_set || !g_state.has_fix) return -1.0f;
    return gps_bearing_between(g_state.pos.lat, g_state.pos.lon,
                               home_lat, home_lon);
}

float gps_distance_between(double lat1, double lon1,
                            double lat2, double lon2) {
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double a = lat1 * DEG_TO_RAD;
    double b = lat2 * DEG_TO_RAD;

    double sin_dlat = sin(dlat / 2);
    double sin_dlon = sin(dlon / 2);
    double h = sin_dlat * sin_dlat +
               cos(a) * cos(b) * sin_dlon * sin_dlon;

    return (float)(2.0 * EARTH_RADIUS_M * atan2(sqrt(h), sqrt(1 - h)));
}

float gps_bearing_between(double lat1, double lon1,
                           double lat2, double lon2) {
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double a = lat1 * DEG_TO_RAD;
    double b = lat2 * DEG_TO_RAD;

    double y = sin(dlon) * cos(b);
    double x = cos(a) * sin(b) - sin(a) * cos(b) * cos(dlon);

    return (float)(fmod(atan2(y, x) * RAD_TO_DEG + 360.0, 360.0));
}

bool gps_has_fix(void) {
    return g_state.has_fix && g_state.pos.fix_quality >= 1;
}

bool gps_has_3d_fix(void) {
    return g_state.has_fix && g_state.pos.fix_quality >= 2;
}

void gps_shutdown(void) {
    if (gps_fd >= 0) {
        close(gps_fd);
        gps_fd = -1;
    }
    printf("GPS: Shutdown\n");
}
