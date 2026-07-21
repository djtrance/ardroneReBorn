#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#define GPS_BUFFER_SIZE     256
#define MAX_NMEA_SENTENCE   120
#define GPS_SERIAL_DEVICE   "/dev/ttyUSB0"
#define GPS_BAUD            9600

/* Position in degrees (decimal) */
typedef struct {
    double lat;       /* latitude,  degrees, negative=South */
    double lon;       /* longitude, degrees, negative=West  */
    float  alt;       /* altitude above MSL, meters         */
    float  hdop;      /* horizontal dilution of precision   */
    int    fix_quality; /* 0=no, 1=GPS, 2=DGPS             */
    int    satellites;  /* number of satellites tracked     */
} gps_position_t;

/* Velocity from ground course */
typedef struct {
    float speed_kn;   /* speed over ground, knots          */
    float course_deg; /* true course, degrees               */
    float climb_ms;   /* vertical speed, m/s, +up           */
} gps_velocity_t;

/* Date/time from GPS */
typedef struct {
    int year;   /* 4-digit  */
    int month;  /* 1-12     */
    int day;    /* 1-31     */
    int hour;   /* 0-23 UTC */
    int minute; /* 0-59     */
    int second; /* 0-59     */
} gps_time_t;

/* Combined GPS state */
typedef struct {
    gps_position_t pos;
    gps_velocity_t vel;
    gps_time_t     time;
    uint64_t       last_update_us; /* monotonic timestamp */
    bool           has_fix;
    float          last_fix_age;   /* seconds since last fix */
} gps_state_t;

/* Callback type for new position data */
typedef void (*gps_callback_t)(const gps_state_t *state, void *userdata);

/* Initialize GPS on a serial device */
int gps_init(const char *device, int baud);

/* Poll: read from serial, parse any available NMEA sentences.
 * Returns number of sentences parsed. */
int gps_update(void);

/* Get current GPS state (thread-safe copy) */
void gps_get_state(gps_state_t *out);

/* Register callback for position updates (called from gps_update) */
void gps_set_callback(gps_callback_t cb, void *userdata);

/* Set home position to current fix */
void gps_set_home(void);

/* Get distance/bearing from home to current position */
float gps_distance_to_home_m(void);
float gps_bearing_to_home_deg(void);

/* Get distance/bearing between two coordinates (Haversine) */
float gps_distance_between(double lat1, double lon1,
                            double lat2, double lon2);
float gps_bearing_between(double lat1, double lon1,
                           double lat2, double lon2);

/* Validate fix quality */
bool gps_has_fix(void);
bool gps_has_3d_fix(void);

/* Cleanup */
void gps_shutdown(void);

#endif /* GPS_H */
