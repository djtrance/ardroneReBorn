#ifndef SKYPROXY_H
#define SKYPROXY_H

#include <stdint.h>

#define SC_PORT       54321
#define SC_TELEM_PORT 55004
#define AT_PORT       5556
#define NAVDATA_PORT  5554
#define VID_SRC_PORT  5555
#define VID_SRC_PORT2 5000
#define VID_DST_PORT  55004

#define ARDRONE3_PROJECT  0x01
#define PILOTING_CLASS    0x00

#define NAVDATA_BUF_SIZE  4096
#define VIDEO_BUF_SIZE    131072
#define MAX_SPS_PPS       128

typedef struct __attribute__((packed)) {
    uint8_t flag;
    int8_t  roll;
    int8_t  pitch;
    int8_t  yaw;
    int8_t  gaz;
    float   psi;
} pcmd_args_t;

typedef struct __attribute__((packed)) {
    uint8_t project;
    uint8_t cls;
    uint8_t cmd;
    uint8_t is_ack;
} arsdk3_cmd_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gaz;
} joystick_t;

typedef struct {
    int   battery;
    float altitude;
    float vx, vy, vz;
    float phi, theta, psi;
} telemetry_t;

#endif
