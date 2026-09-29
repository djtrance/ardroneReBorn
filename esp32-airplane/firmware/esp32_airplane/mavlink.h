// mavlink.h — minimal MAVLink 2.0 telemetry link (checklist I1).
//
// Purpose: stream live aircraft state to a ground station (QGroundControl,
// Mission Planner, MAVProxy) over a configurable UART, and accept one
// inbound message (REQUEST_DATA_STREAM) so the GCS can steer the rate.
//
//   v2 frames only (magic 0xFD), no signing, no param protocol, no commands
//   — telemetry output plus the stream-rate handshake. docs/mavlink-telemetry.md
//
// Design: pure C++ (no Arduino) so the encoder can be byte-compared on the
// host against frames produced by the official pymavlink generator. The CRC
// is CRC-16/MCRF4XX (poly 0x8408 reflected, init 0xFFFF, no xorout) over
// every frame byte EXCEPT the magic, plus the message's crc_extra tacked on
// the end — same rule mavlink C uses in mavlink_finalize_message().
//
// MAVLink 2 zero-truncation: trailing zero bytes of the payload are dropped
// before the CRC (optional per spec, pymavlink does it too — our golden
// vectors rely on it, e.g. VFR_HUD with throttle=65 ships 19 of 20 bytes).

#pragma once
#include <stdint.h>
#include <stddef.h>

#define MAVLINK_MAGIC2          0xFD
#define MAV_PAYLOAD_MAX         64      // biggest payload we build (STATUSTEXT 55)
#define MAV_FRAME_MAX           (10 + MAV_PAYLOAD_MAX + 2)
#define MAV_RX_BUF              280     // 10 hdr + 255 payload + 2 crc + slack

#define MAV_RATE_MIN            1
#define MAV_RATE_MAX            25      // LOG_RATE_HZ — the tick that feeds us

// --- message ids (the subset we speak) --------------------------------------
#define MAV_MSG_HEARTBEAT              0
#define MAV_MSG_SYS_STATUS             1
#define MAV_MSG_GPS_RAW_INT            24
#define MAV_MSG_ATTITUDE               30
#define MAV_MSG_GLOBAL_POSITION_INT    33
#define MAV_MSG_SERVO_OUTPUT_RAW       36
#define MAV_MSG_REQUEST_DATA_STREAM    66
#define MAV_MSG_VFR_HUD                74
#define MAV_MSG_STATUSTEXT             253

// --- CRC extras (verified against pymavlink common dialect) ----------------
#define MAV_CRC_HEARTBEAT              50
#define MAV_CRC_SYS_STATUS             124
#define MAV_CRC_GPS_RAW_INT            24
#define MAV_CRC_ATTITUDE               39
#define MAV_CRC_GLOBAL_POSITION_INT    104
#define MAV_CRC_SERVO_OUTPUT_RAW       222
#define MAV_CRC_REQUEST_DATA_STREAM    148
#define MAV_CRC_VFR_HUD                20
#define MAV_CRC_STATUSTEXT             83

// --- enum values (common.xml) ----------------------------------------------
#define MAV_TYPE_FIXED_WING            1
#define MAV_AUTOPILOT_GENERIC          0
#define MAV_MODE_FLAG_CUSTOM_MODE      16
#define MAV_MODE_FLAG_SAFETY_ARMED     128
#define MAV_STATE_STANDBY              3
#define MAV_STATE_ACTIVE               4
#define MAV_SEVERITY_INFO              6

// MAV_SYS_STATUS_SENSOR bits (common.xml)
#define MAV_SYS_3D_GYRO                0x00000001u
#define MAV_SYS_3D_ACCEL               0x00000002u
#define MAV_SYS_3D_MAG                 0x00000004u
#define MAV_SYS_ABSOLUTE_PRESSURE      0x00000008u
#define MAV_SYS_GPS                    0x00000020u
#define MAV_SYS_RC_RECEIVER            0x00010000u

#define MAV_GPS_FIX_NONE               0
#define MAV_GPS_FIX_3D                 3
#define MAV_GPS_FIX_DGPS               4
#define MAV_HDG_UNKNOWN                65535   // UINT16_MAX

// ---------------------------------------------------------------------------
// CRC-16/MCRF4XX — running accumulator (init 0xFFFF, no final xorout)
// ---------------------------------------------------------------------------
uint16_t mav_crc16(uint16_t crc, uint8_t byte);
uint16_t mav_crc16_buf(const uint8_t* data, size_t len);

// ---------------------------------------------------------------------------
// Generic v2 frame encoder.
//
// Writes magic, trimmed payload length, incompat/compat flags (0 = unsigned),
// seq, sysid, compid, msgid, payload, CRC. Returns total frame bytes, or 0 if
// the buffer is too small / arguments are bad.
// ---------------------------------------------------------------------------
size_t mavlink_encode(uint8_t seq, uint8_t sysid, uint8_t compid,
                      uint8_t msgid, const uint8_t* payload, uint8_t plen,
                      uint8_t crc_extra, uint8_t* out, size_t cap);

// ---------------------------------------------------------------------------
// Payload packers — each returns the payload length written into p[].
// Field orders are the on-wire orders of the official generator (largest
// primitives first, extensions trimmed by mavlink_encode's zero-truncation).
// ---------------------------------------------------------------------------
uint8_t mav_pack_heartbeat(uint8_t* p, uint8_t type, uint8_t autopilot,
                           uint8_t base_mode, uint32_t custom_mode,
                           uint8_t system_status);

uint8_t mav_pack_attitude(uint8_t* p, uint32_t t_ms,
                          float roll, float pitch, float yaw,
                          float rollspeed, float pitchspeed, float yawspeed);

uint8_t mav_pack_global_position_int(uint8_t* p, uint32_t t_ms,
                                     int32_t lat_e7, int32_t lon_e7,
                                     int32_t alt_mm, int32_t relative_alt_mm,
                                     int16_t vx_cms, int16_t vy_cms,
                                     int16_t vz_cms, uint16_t hdg_cdeg);

uint8_t mav_pack_gps_raw_int(uint8_t* p, uint64_t t_us, uint8_t fix_type,
                             int32_t lat_e7, int32_t lon_e7, int32_t alt_mm,
                             uint16_t eph_cm, uint16_t epv_cm,
                             uint16_t vel_cms, uint16_t cog_cdeg,
                             uint8_t satellites_visible);

uint8_t mav_pack_vfr_hud(uint8_t* p, float airspeed, float groundspeed,
                         int16_t heading_deg, uint16_t throttle_pct,
                         float alt_m, float climb_ms);

uint8_t mav_pack_sys_status(uint8_t* p, uint32_t present, uint32_t enabled,
                            uint32_t health, uint16_t load,
                            uint16_t voltage_mv, int16_t current_ca,
                            int8_t battery_pct);

uint8_t mav_pack_servo_output_raw(uint8_t* p, uint32_t t_us, uint8_t port,
                                  uint16_t s1, uint16_t s2, uint16_t s3,
                                  uint16_t s4, uint16_t s5, uint16_t s6,
                                  uint16_t s7, uint16_t s8);

uint8_t mav_pack_statustext(uint8_t* p, uint8_t severity, const char* text);

// ---------------------------------------------------------------------------
// Stream scheduler — which messages are due at this tick.
// ---------------------------------------------------------------------------
#define MAV_DUE_HEARTBEAT   (1u << 0)   // 1 Hz, fixed
#define MAV_DUE_ATTITUDE    (1u << 1)   // rate_hz
#define MAV_DUE_POSITION    (1u << 2)   // rate_hz
#define MAV_DUE_GPS         (1u << 3)   // 1 Hz, fixed
#define MAV_DUE_SYS         (1u << 4)   // 1 Hz, fixed
#define MAV_DUE_SERVO       (1u << 5)   // rate_hz
#define MAV_DUE_VFR         (1u << 6)   // rate_hz

typedef struct {
    uint32_t t_hb, t_att, t_pos, t_gps, t_sys, t_srv, t_vfr;  // next due ms
    uint8_t  rate_hz;                 // fast-message rate
} MavlinkSchedule;

void     mav_schedule_init(MavlinkSchedule* s, uint8_t rate_hz);
void     mav_schedule_set_rate(MavlinkSchedule* s, uint8_t rate_hz);
uint32_t mav_schedule_due(MavlinkSchedule* s, uint32_t now_ms);

// ---------------------------------------------------------------------------
// RX — hunts magic, assembles a frame, verifies the CRC against our crc_extra
// table, and reports complete frames. Unknown message ids are consumed but
// dropped (their crc_extra is not in the table, so the checksum can't be
// validated); signed frames (incompat bit 0) are dropped too.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t msgid;
    uint8_t sysid, compid;
    uint8_t payload[256];
    uint8_t len;
} MavlinkRxMsg;

typedef struct {
    uint8_t buf[MAV_RX_BUF];
    uint16_t n;         // bytes accumulated; 0 = hunting for magic
    uint16_t total;     // expected frame size once the header is in
} MavlinkRx;

void mavlink_rx_init(MavlinkRx* rx);
// Feed one byte. Returns true when a complete verified frame landed in *out.
bool mavlink_rx_feed(MavlinkRx* rx, uint8_t b, MavlinkRxMsg* out);

// If msg is a REQUEST_DATA_STREAM aimed at us (target 0/255 broadcast or our
// sysid) with start_stop != 0, returns true and the requested rate clamped
// to [MAV_RATE_MIN, MAV_RATE_MAX].
bool mavlink_rds_rate(const MavlinkRxMsg* msg, uint8_t my_sysid,
                      uint8_t* rate_hz);
