// mavlink.cpp — MAVLink 2.0 frame encoder + minimal RX path (checklist I1).
// Pure C++ (host-tested against pymavlink golden vectors — see
// tests/test_mavlink.cpp). Wire rules confirmed against the official
// generator: zero-truncated payload, CRC-16/MCRF4XX over everything except
// the magic byte plus crc_extra.

#include "mavlink.h"
#include <string.h>

// ---------------------------------------------------------------------------
// CRC-16/MCRF4XX — reflected poly 0x8408, init 0xFFFF, no final xorout.
// ---------------------------------------------------------------------------
uint16_t mav_crc16(uint16_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++)
        crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408)
                        : (uint16_t)(crc >> 1);
    return crc;
}

uint16_t mav_crc16_buf(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    if (!data) return crc;
    for (size_t i = 0; i < len; i++) crc = mav_crc16(crc, data[i]);
    return crc;
}

// --- little-endian payload writers -----------------------------------------
static void put_u8(uint8_t*& p, uint8_t v)   { *p++ = v; }
static void put_i8(uint8_t*& p, int8_t v)    { *p++ = (uint8_t)v; }
static void put_u16(uint8_t*& p, uint16_t v) { *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); }
static void put_i16(uint8_t*& p, int16_t v)  { put_u16(p, (uint16_t)v); }
static void put_u32(uint8_t*& p, uint32_t v) {
    *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8);
    *p++ = (uint8_t)(v >> 16); *p++ = (uint8_t)(v >> 24);
}
static void put_i32(uint8_t*& p, int32_t v)  { put_u32(p, (uint32_t)v); }
static void put_u64(uint8_t*& p, uint64_t v) {
    put_u32(p, (uint32_t)v);
    put_u32(p, (uint32_t)(v >> 32));
}
static void put_f32(uint8_t*& p, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put_u32(p, bits);
}

// ---------------------------------------------------------------------------
// Frame encoder: v2 header + trimmed payload + CRC.
// ---------------------------------------------------------------------------
size_t mavlink_encode(uint8_t seq, uint8_t sysid, uint8_t compid,
                      uint8_t msgid, const uint8_t* payload, uint8_t plen,
                      uint8_t crc_extra, uint8_t* out, size_t cap) {
    if (!payload && plen) return 0;

    // MAVLink 2 zero-truncation: drop trailing zero payload bytes.
    while (plen > 0 && payload[plen - 1] == 0) plen--;

    const size_t total = 10u + plen + 2u;
    if (!out || cap < total) return 0;

    out[0] = MAVLINK_MAGIC2;
    out[1] = plen;
    out[2] = 0;                      // incompat: unsigned (no signing)
    out[3] = 0;                      // compat flags
    out[4] = seq;
    out[5] = sysid;
    out[6] = compid;
    out[7] = (uint8_t)(msgid & 0xFF);
    out[8] = (uint8_t)((msgid >> 8) & 0xFF);
    out[9] = (uint8_t)((msgid >> 16) & 0xFF);
    memcpy(out + 10, payload, plen);

    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 10u + plen; i++) crc = mav_crc16(crc, out[i]);
    crc = mav_crc16(crc, crc_extra);
    out[10 + plen]     = (uint8_t)(crc & 0xFF);
    out[10 + plen + 1] = (uint8_t)(crc >> 8);
    return total;
}

// ---------------------------------------------------------------------------
// Payload packers (on-wire field orders of the official generator).
// ---------------------------------------------------------------------------
uint8_t mav_pack_heartbeat(uint8_t* p, uint8_t type, uint8_t autopilot,
                           uint8_t base_mode, uint32_t custom_mode,
                           uint8_t system_status) {
    uint8_t* d = p;
    put_u32(d, custom_mode);         // 4-byte first
    put_u8(d, type);
    put_u8(d, autopilot);
    put_u8(d, base_mode);
    put_u8(d, system_status);
    put_u8(d, 3);                    // mavlink_version
    return (uint8_t)(d - p);         // 9
}

uint8_t mav_pack_attitude(uint8_t* p, uint32_t t_ms,
                          float roll, float pitch, float yaw,
                          float rollspeed, float pitchspeed, float yawspeed) {
    uint8_t* d = p;
    put_u32(d, t_ms);
    put_f32(d, roll);
    put_f32(d, pitch);
    put_f32(d, yaw);
    put_f32(d, rollspeed);
    put_f32(d, pitchspeed);
    put_f32(d, yawspeed);
    return (uint8_t)(d - p);         // 28
}

uint8_t mav_pack_global_position_int(uint8_t* p, uint32_t t_ms,
                                     int32_t lat_e7, int32_t lon_e7,
                                     int32_t alt_mm, int32_t relative_alt_mm,
                                     int16_t vx_cms, int16_t vy_cms,
                                     int16_t vz_cms, uint16_t hdg_cdeg) {
    uint8_t* d = p;
    put_u32(d, t_ms);
    put_i32(d, lat_e7);
    put_i32(d, lon_e7);
    put_i32(d, alt_mm);
    put_i32(d, relative_alt_mm);
    put_i16(d, vx_cms);
    put_i16(d, vy_cms);
    put_i16(d, vz_cms);
    put_u16(d, hdg_cdeg);
    return (uint8_t)(d - p);         // 28
}

uint8_t mav_pack_gps_raw_int(uint8_t* p, uint64_t t_us, uint8_t fix_type,
                             int32_t lat_e7, int32_t lon_e7, int32_t alt_mm,
                             uint16_t eph_cm, uint16_t epv_cm,
                             uint16_t vel_cms, uint16_t cog_cdeg,
                             uint8_t satellites_visible) {
    uint8_t* d = p;
    put_u64(d, t_us);
    put_i32(d, lat_e7);
    put_i32(d, lon_e7);
    put_i32(d, alt_mm);
    put_u16(d, eph_cm);
    put_u16(d, epv_cm);
    put_u16(d, vel_cms);
    put_u16(d, cog_cdeg);
    put_u8(d, fix_type);
    put_u8(d, satellites_visible);
    return (uint8_t)(d - p);         // 30
}

uint8_t mav_pack_vfr_hud(uint8_t* p, float airspeed, float groundspeed,
                         int16_t heading_deg, uint16_t throttle_pct,
                         float alt_m, float climb_ms) {
    uint8_t* d = p;
    put_f32(d, airspeed);            // floats first, decl order
    put_f32(d, groundspeed);
    put_f32(d, alt_m);
    put_f32(d, climb_ms);
    put_i16(d, heading_deg);
    put_u16(d, throttle_pct);
    return (uint8_t)(d - p);         // 20
}

uint8_t mav_pack_sys_status(uint8_t* p, uint32_t present, uint32_t enabled,
                            uint32_t health, uint16_t load,
                            uint16_t voltage_mv, int16_t current_ca,
                            int8_t battery_pct) {
    uint8_t* d = p;
    put_u32(d, present);
    put_u32(d, enabled);
    put_u32(d, health);
    put_u16(d, load);
    put_u16(d, voltage_mv);
    put_i16(d, current_ca);
    put_u16(d, 0);                   // drop_rate_comm
    put_u16(d, 0);                   // errors_comm
    put_u16(d, 0);                   // errors_count1
    put_u16(d, 0);
    put_u16(d, 0);
    put_u16(d, 0);                   // errors_count4
    put_i8(d, battery_pct);          // 1-byte field sorts last
    return (uint8_t)(d - p);         // 31
}

uint8_t mav_pack_servo_output_raw(uint8_t* p, uint32_t t_us, uint8_t port,
                                  uint16_t s1, uint16_t s2, uint16_t s3,
                                  uint16_t s4, uint16_t s5, uint16_t s6,
                                  uint16_t s7, uint16_t s8) {
    uint8_t* d = p;
    put_u32(d, t_us);
    put_u16(d, s1);
    put_u16(d, s2);
    put_u16(d, s3);
    put_u16(d, s4);
    put_u16(d, s5);
    put_u16(d, s6);
    put_u16(d, s7);
    put_u16(d, s8);
    put_u8(d, port);                 // 1-byte field sorts last
    return (uint8_t)(d - p);         // 21
}

uint8_t mav_pack_statustext(uint8_t* p, uint8_t severity, const char* text) {
    uint8_t* d = p;
    put_u8(d, severity);
    size_t n = text ? strlen(text) : 0;
    if (n > 50) n = 50;
    if (n) memcpy(d, text, n);
    memset(d + n, 0, 50 - n);
    d += 50;
    put_u16(d, 0);                   // id (chunking unused)
    put_u16(d, 0);                   // chunk_seq
    return (uint8_t)(d - p);         // 55; trailing zeros trim in encode()
}

// ---------------------------------------------------------------------------
// Stream scheduler
// ---------------------------------------------------------------------------
static uint8_t clamp_rate(uint8_t r) {
    if (r < MAV_RATE_MIN) return MAV_RATE_MIN;
    if (r > MAV_RATE_MAX) return MAV_RATE_MAX;
    return r;
}

void mav_schedule_init(MavlinkSchedule* s, uint8_t rate_hz) {
    memset(s, 0, sizeof(*s));        // next-due = 0: everything fires tick 1
    s->rate_hz = clamp_rate(rate_hz);
}

void mav_schedule_set_rate(MavlinkSchedule* s, uint8_t rate_hz) {
    s->rate_hz = clamp_rate(rate_hz);
}

static uint32_t due_one(uint32_t now, uint32_t* next, uint32_t interval,
                        uint32_t bit) {
    if ((int32_t)(now - *next) >= 0) {
        *next = now + interval;      // re-arm from "now" (caller is periodic)
        return bit;
    }
    return 0;
}

uint32_t mav_schedule_due(MavlinkSchedule* s, uint32_t now_ms) {
    uint32_t fast = 1000u / clamp_rate(s->rate_hz);
    if (fast == 0) fast = 1;
    uint32_t due = 0;
    due |= due_one(now_ms, &s->t_hb,  1000,       MAV_DUE_HEARTBEAT);
    due |= due_one(now_ms, &s->t_att, fast,        MAV_DUE_ATTITUDE);
    due |= due_one(now_ms, &s->t_pos, fast,        MAV_DUE_POSITION);
    due |= due_one(now_ms, &s->t_gps, 1000,        MAV_DUE_GPS);
    due |= due_one(now_ms, &s->t_sys, 1000,        MAV_DUE_SYS);
    due |= due_one(now_ms, &s->t_srv, fast,        MAV_DUE_SERVO);
    due |= due_one(now_ms, &s->t_vfr, fast,        MAV_DUE_VFR);
    return due;
}

// ---------------------------------------------------------------------------
// RX path
// ---------------------------------------------------------------------------
void mavlink_rx_init(MavlinkRx* rx) {
    rx->n = 0;
    rx->total = 0;
}

// Messages whose crc_extra AND payload length we know. Everything else is
// consumed but dropped (an unverifiable frame is not worth trusting) — and
// because the msgid arrives early (byte 7..9), an unrecognised frame can be
// abandoned BEFORE its payload even finishes arriving, which is what lets the
// parser resynchronise after a stray magic byte without swallowing a whole
// following frame. HEARTBEAT is recognised so a GCS presence can be detected
// later; only REQUEST_DATA_STREAM is dispatched today.
struct MavKnownMsg { uint32_t id; uint8_t crc; uint8_t len; };
static const MavKnownMsg k_known[] = {
    { MAV_MSG_HEARTBEAT,           MAV_CRC_HEARTBEAT,           9 },
    { MAV_MSG_REQUEST_DATA_STREAM, MAV_CRC_REQUEST_DATA_STREAM, 6 },
};
static const MavKnownMsg* known_msg(uint32_t msgid) {
    for (size_t i = 0; i < sizeof(k_known) / sizeof(k_known[0]); i++)
        if (k_known[i].id == msgid) return &k_known[i];
    return nullptr;
}

// Step past the suspect frame start and hunt for the next magic byte.
static void rx_drop_to_magic(MavlinkRx* rx) {
    if (rx->n > 0) memmove(rx->buf, rx->buf + 1, --rx->n);
    while (rx->n > 0 && rx->buf[0] != MAVLINK_MAGIC2)
        memmove(rx->buf, rx->buf + 1, --rx->n);
    rx->total = 0;
}

bool mavlink_rx_feed(MavlinkRx* rx, uint8_t b, MavlinkRxMsg* out) {
    if (!rx || !out) return false;

    if (rx->n == 0) {                            // hunting for the magic
        if (b != MAVLINK_MAGIC2) return false;
        rx->buf[0] = b;
        rx->n = 1;
    } else {
        if (rx->n >= (uint16_t)sizeof(rx->buf)) { rx->n = 0; rx->total = 0; return false; }
        rx->buf[rx->n++] = b;
    }

    // Validate whatever frame sits at the head of the buffer; on any red
    // flag step one byte forward and hunt again (same strategy as the SBUS
    // decoder), so a good frame buried behind garbage still comes out.
    for (;;) {
        if (rx->n < 2) { rx->total = 0; return false; }   // need the len byte
        const uint16_t expect = (uint16_t)(10u + rx->buf[1] + 2u);
        if (expect > (uint16_t)sizeof(rx->buf)) { rx_drop_to_magic(rx); continue; }

        if (rx->n >= 3 && (rx->buf[2] & 0x01)) {  // signed: unsupported
            rx_drop_to_magic(rx);
            continue;
        }
        if (rx->n >= 10) {                        // msgid is fully buffered
            const uint32_t msgid = (uint32_t)rx->buf[7] |
                                   ((uint32_t)rx->buf[8] << 8) |
                                   ((uint32_t)rx->buf[9] << 16);
            const MavKnownMsg* k = known_msg(msgid);
            // Unknown id => not worth waiting for. A KNOWN id may legally be
            // SHORTER than expected (MAVLink2 zero-truncation), never longer.
            if (!k || rx->buf[1] > k->len) {
                rx_drop_to_magic(rx);
                continue;
            }
        }
        rx->total = expect;
        if (rx->n < rx->total) return false;      // wait for the rest

        // Complete plausible frame — CRC decides.
        const MavKnownMsg* k = known_msg((uint32_t)rx->buf[7] |
                                         ((uint32_t)rx->buf[8] << 8) |
                                         ((uint32_t)rx->buf[9] << 16));
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 1; i < (uint16_t)(rx->total - 2); i++)
            crc = mav_crc16(crc, rx->buf[i]);
        crc = mav_crc16(crc, k->crc);
        const uint16_t got = (uint16_t)(rx->buf[rx->total - 2] |
                                        (rx->buf[rx->total - 1] << 8));
        if (crc == got) {
            out->msgid  = rx->buf[7];
            out->sysid  = rx->buf[5];
            out->compid = rx->buf[6];
            out->len    = k->len;                 // full logical length
            memcpy(out->payload, rx->buf + 10, rx->buf[1]);
            if (out->len > rx->buf[1])            // zero-trimmed payload:
                memset(out->payload + rx->buf[1], 0,                   // NOLINT
                       out->len - rx->buf[1]);     // decoder pads with zeros
            // Consume the frame; leave any remainder at its next magic so
            // the following byte validates it (streams never stop anyway).
            if (rx->n > rx->total)
                memmove(rx->buf, rx->buf + rx->total, rx->n - rx->total);
            rx->n = (rx->n > rx->total) ? (uint16_t)(rx->n - rx->total) : 0;
            rx->total = 0;
            while (rx->n > 0 && rx->buf[0] != MAVLINK_MAGIC2)
                memmove(rx->buf, rx->buf + 1, --rx->n);
            return true;
        }
        rx_drop_to_magic(rx);                     // bad CRC: hunt again
    }
}

bool mavlink_rds_rate(const MavlinkRxMsg* msg, uint8_t my_sysid,
                      uint8_t* rate_hz) {
    if (!msg || msg->msgid != MAV_MSG_REQUEST_DATA_STREAM || msg->len < 6)
        return false;

    const uint16_t rate   = (uint16_t)(msg->payload[0] | (msg->payload[1] << 8));
    const uint8_t  target = msg->payload[2];      // after the u16 (size order)
    const uint8_t  start  = msg->payload[5];
    if (target != my_sysid && target != 0 && target != 255) return false;
    if (start == 0) return false;                 // "stop" ignored: keep alive

    uint8_t r = MAV_RATE_MAX;
    if (rate < MAV_RATE_MAX) r = (uint8_t)rate;
    if (r < MAV_RATE_MIN) r = MAV_RATE_MIN;
    if (rate_hz) *rate_hz = r;
    return true;
}
