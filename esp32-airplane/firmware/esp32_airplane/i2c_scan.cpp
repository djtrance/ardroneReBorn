// i2c_scan.cpp — bus scanner + pure JSON formatter (see i2c_scan.h)
#include "i2c_scan.h"
#include <stdio.h>
#include <string.h>

#ifdef ARDUINO
#include <Wire.h>

// Read n bytes at reg, false on NACK / short read.
static bool scan_rd(uint8_t addr, uint8_t reg, uint8_t* b, uint8_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;      // repeated start
    if (Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
    for (uint8_t i = 0; i < n; i++) b[i] = (uint8_t)Wire.read();
    return true;
}

void i2c_scan_run(I2cScan& out) {
    memset(&out, 0, sizeof(out));

    for (uint8_t a = 0x08; a <= 0x77; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;           // nobody home
        if (out.n < I2C_SCAN_MAX) out.addrs[out.n++] = a;

        uint8_t id[3] = {0, 0, 0};
        switch (a) {
        case 0x68:                                             // MPU6050
        case 0x69:
            out.mpu_ok = scan_rd(a, 0x75, id, 1);
            out.mpu_who = id[0];
            break;
        case 0x1E:                                             // HMC5883L
            out.hmc_ok = scan_rd(a, 0x0A, id, 3);
            memcpy(out.hmc_id, id, 3);
            break;
        case 0x0D:                                             // QMC5883L
            out.qmc_ok = scan_rd(a, 0x0D, id, 1);
            out.qmc_id = id[0];
            break;
        case 0x76:                                             // BMP180/280
        case 0x77:
            out.bmp_ok = scan_rd(a, 0xD0, id, 1);
            out.bmp_id = id[0];
            break;
        default:
            break;
        }
    }
}
#else
void i2c_scan_run(I2cScan& out) { memset(&out, 0, sizeof(out)); }
#endif

// ---------------------------------------------------------------------------
// JSON formatter (pure — host-unit-tested)
// ---------------------------------------------------------------------------
static int jid(char* p, size_t cap, bool ok, uint8_t v) {
    return ok ? snprintf(p, cap, "\"%02x\"", (unsigned)v)
              : snprintf(p, cap, "null");
}

size_t i2c_scan_json(const I2cScan& s, char* buf, size_t cap) {
    if (!buf || cap < 32) return 0;
    int n = snprintf(buf, cap, "{\"addrs\":[");
    if (n < 0 || (size_t)n >= cap) return 0;
    size_t off = (size_t)n;

    for (uint8_t i = 0; i < s.n && i < I2C_SCAN_MAX; i++) {
        n = snprintf(buf + off, cap - off, "%s0x%02x", i ? "," : "",
                     (unsigned)s.addrs[i]);
        if (n < 0 || (size_t)n >= cap - off) return 0;
        off += (size_t)n;
    }

    // HMC id is 3 bytes -> one string ("483433")
    char id[16];
    int m = s.hmc_ok ? snprintf(id, sizeof(id), "\"%02x%02x%02x\"",
                                s.hmc_id[0], s.hmc_id[1], s.hmc_id[2])
                     : snprintf(id, sizeof(id), "null");
    if (m < 0 || (size_t)m >= sizeof(id)) return 0;

    n = snprintf(buf + off, cap - off,
                 "],\"mpu\":");
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = jid(buf + off, cap - off, s.mpu_ok, s.mpu_who);
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = snprintf(buf + off, cap - off, ",\"hmc\":%s", id);
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = snprintf(buf + off, cap - off, ",\"qmc\":");
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = jid(buf + off, cap - off, s.qmc_ok, s.qmc_id);
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = snprintf(buf + off, cap - off, ",\"bmp\":");
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = jid(buf + off, cap - off, s.bmp_ok, s.bmp_id);
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;

    n = snprintf(buf + off, cap - off, "}");
    if (n < 0 || (size_t)n >= cap - off) return 0;
    off += (size_t)n;
    return off;
}
