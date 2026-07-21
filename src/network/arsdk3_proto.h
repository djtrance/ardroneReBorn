#ifndef ARSDK3_PROTO_H
#define ARSDK3_PROTO_H

#include <stdint.h>
#include <string.h>

/* ARNetworkAL frame header (7 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  type;    /* 2=DATA, 3=DATA_LOW_LATENCY, 4=DATA_WITH_ACK */
    uint8_t  id;      /* buffer identifier */
    uint8_t  seq;     /* sequence number */
    uint32_t size;    /* total frame size including header (little-endian) */
} arnet_frame_t;

#define ARNET_TYPE_DATA            2
#define ARNET_TYPE_DATA_LOW_LATENCY 3
#define ARNET_TYPE_DATA_WITH_ACK   4
#define ARNET_TYPE_ACK             1

#define ARNET_BUF_VIDEO_DATA   125  /* 0x7D ARStream video fragments */
#define ARNET_BUF_VIDEO_ACK    13   /* 0x0D ARStream video acks */
#define ARNET_BUF_COMMAND      127  /* 0x7F c2d commands */
#define ARNET_BUF_EVENT        126  /* 0x7E d2c events */

/* ARStream video fragment header (5 bytes after ARNetworkAL header) */
typedef struct __attribute__((packed)) {
    uint16_t frame_number;
    uint8_t  frame_flags;       /* bit 0 = FLUSH_FRAME (I-frame) */
    uint8_t  fragment_number;
    uint8_t  fragments_per_frame;
} arstream_video_hdr_t;

/* ARStream ACK payload (18 bytes) */
typedef struct __attribute__((packed)) {
    uint16_t frame_number;
    uint64_t high_packets_ack;
    uint64_t low_packets_ack;
} arstream_ack_t;

/* ARSDK3 command/event header (4 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  project;
    uint8_t  cls;
    uint16_t cmd_id;            /* little-endian */
} arsdk3_event_hdr_t;

/* Helper: build ARNetworkAL frame header */
static inline int build_arnet_header(uint8_t *buf, int buf_len,
                                     uint8_t type, uint8_t id,
                                     uint8_t seq, int payload_len)
{
    if (buf_len < 7) return -1;
    int total = 7 + payload_len;
    buf[0] = type;
    buf[1] = id;
    buf[2] = seq;
    buf[3] = total & 0xFF;
    buf[4] = (total >> 8) & 0xFF;
    buf[5] = (total >> 16) & 0xFF;
    buf[6] = (total >> 24) & 0xFF;
    return 7;
}

/* Helper: build ARStream video fragment
 * Returns total packet length, or -1 on error.
 * Writes ARNetworkAL header + ARStream header + h264_data into buf.
 */
static inline int build_arstream_packet(uint8_t *buf, int buf_len,
                                        uint8_t seq_num,
                                        uint16_t frame_num,
                                        int is_idr,
                                        int fragment_idx,
                                        int fragments_total,
                                        const uint8_t *h264_data,
                                        int h264_len)
{
    int hdr_sz = 7 + 5;
    if (buf_len < hdr_sz + h264_len) return -1;

    int off = build_arnet_header(buf, buf_len, ARNET_TYPE_DATA_LOW_LATENCY,
                                  ARNET_BUF_VIDEO_DATA, seq_num,
                                  5 + h264_len);
    if (off < 0) return -1;

    buf[off]     = frame_num & 0xFF;
    buf[off+1]   = (frame_num >> 8) & 0xFF;
    buf[off+2]   = is_idr ? 1 : 0;  /* FLUSH_FRAME flag */
    buf[off+3]   = (uint8_t)fragment_idx;
    buf[off+4]   = (uint8_t)fragments_total;
    off += 5;

    memcpy(buf + off, h264_data, h264_len);
    off += h264_len;

    /* Update size in header */
    int total = off;
    buf[3] = total & 0xFF;
    buf[4] = (total >> 8) & 0xFF;
    buf[5] = (total >> 16) & 0xFF;
    buf[6] = (total >> 24) & 0xFF;

    return total;
}

/* Helper: build ARSDK3 event frame (project 0 = Common, project 1 = ARDrone3)
 * Returns total packet length, or -1 on error.
 */
static inline int build_event_frame(uint8_t *buf, int buf_len,
                                    uint8_t seq_num,
                                    uint8_t project, uint8_t cls,
                                    uint16_t cmd_id,
                                    const uint8_t *args, int args_len)
{
    int payload_sz = 4 + args_len;
    int off = build_arnet_header(buf, buf_len, ARNET_TYPE_DATA_WITH_ACK,
                                  ARNET_BUF_EVENT, seq_num, payload_sz);
    if (off < 0) return -1;

    buf[off]   = project;
    buf[off+1] = cls;
    buf[off+2] = cmd_id & 0xFF;
    buf[off+3] = (cmd_id >> 8) & 0xFF;
    off += 4;

    if (args_len > 0 && args) {
        memcpy(buf + off, args, args_len);
        off += args_len;
    }

    int total = off;
    buf[3] = total & 0xFF;
    buf[4] = (total >> 8) & 0xFF;
    buf[5] = (total >> 16) & 0xFF;
    buf[6] = (total >> 24) & 0xFF;
    return total;
}

/* Pre-built event argument builders */
static inline int build_battery_event(uint8_t *buf, int buf_len,
                                      uint8_t seq_num, int percentage)
{
    uint8_t args[1] = { (uint8_t)percentage };
    return build_event_frame(buf, buf_len, seq_num,
                             0, 5, 1, args, 1);
}

static inline int build_attitude_event(uint8_t *buf, int buf_len,
                                       uint8_t seq_num,
                                       float roll, float pitch, float yaw)
{
    uint8_t args[12];
    float v[3] = { roll, pitch, yaw };
    memcpy(args, v, 12);
    return build_event_frame(buf, buf_len, seq_num,
                             1, 0, 3, args, 12);
}

static inline int build_altitude_event(uint8_t *buf, int buf_len,
                                       uint8_t seq_num, double alt)
{
    uint8_t args[8];
    memcpy(args, &alt, 8);
    return build_event_frame(buf, buf_len, seq_num,
                             1, 0, 4, args, 8);
}

static inline int build_speed_event(uint8_t *buf, int buf_len,
                                    uint8_t seq_num,
                                    float vx, float vy, float vz)
{
    uint8_t args[12];
    float v[3] = { vx, vy, vz };
    memcpy(args, v, 12);
    return build_event_frame(buf, buf_len, seq_num,
                             1, 0, 5, args, 12);
}

static inline int build_gps_position_event(uint8_t *buf, int buf_len,
                                            uint8_t seq_num,
                                            double lat, double lon, double alt)
{
    uint8_t args[24];
    memcpy(args, &lat, 8);
    memcpy(args + 8, &lon, 8);
    memcpy(args + 16, &alt, 8);
    return build_event_frame(buf, buf_len, seq_num,
                             1, 4, 4, args, 24);
}

static inline int build_flying_state_event(uint8_t *buf, int buf_len,
                                            uint8_t seq_num, uint8_t state)
{
    uint8_t args[4] = { state, 0, 0, 0 };
    return build_event_frame(buf, buf_len, seq_num,
                             1, 0, 22, args, 4);
}

static inline int build_wifi_signal_event(uint8_t *buf, int buf_len,
                                           uint8_t seq_num,
                                           int16_t rssi)
{
    uint8_t args[2];
    args[0] = rssi & 0xFF;
    args[1] = (rssi >> 8) & 0xFF;
    return build_event_frame(buf, buf_len, seq_num,
                             0, 5, 7, args, 2);
}

/* Video ACK IDs */
#define ARNET_VIDEO_ACK_ID  13

/* Parse incoming ARNetworkAL frame header from buf (len >= 7).
 * Returns 0 on success, -1 on error.
 * Sets *type_out, *id_out, *seq_out, *payload_offset, *payload_len.
 */
static inline int parse_arnet_frame(const uint8_t *buf, int len,
                                     uint8_t *type_out, uint8_t *id_out,
                                     uint8_t *seq_out,
                                     int *payload_offset, int *payload_len)
{
    if (len < 7) return -1;
    *type_out = buf[0];
    *id_out   = buf[1];
    *seq_out  = buf[2];
    uint32_t total = (uint32_t)buf[3]
                   | ((uint32_t)buf[4] << 8)
                   | ((uint32_t)buf[5] << 16)
                   | ((uint32_t)buf[6] << 24);
    if (total < 7 || total > (uint32_t)len) return -1;
    *payload_offset = 7;
    *payload_len = total - 7;
    return 0;
}

/* Check if a frame is a video ACK */
static inline int is_video_ack(uint8_t id, int payload_len)
{
    return (id == ARNET_VIDEO_ACK_ID && payload_len >= 18);
}

#endif
