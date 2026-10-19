// i2c_scan.h — I2C bus scanner for bench bring-up (checklist T0)
//
// Answers the first question every "sensor init failed" bench session asks:
// which devices actually ACK on the bus, and do the known parts answer with
// the expected ID bytes? Run once at boot (printed on serial) and on demand
// from the config portal (GET /api/i2c) — e.g. when `mag=0`.
//
// The scan needs the Arduino Wire bus, but the JSON formatter is pure C so
// `make test` pins the exact output without hardware.
#pragma once
#include <stddef.h>
#include <stdint.h>

#define I2C_SCAN_MAX    16          // cap on the reported address list

struct I2cScan {
    uint8_t addrs[I2C_SCAN_MAX];    // every address in 0x08..0x77 that ACKed
    uint8_t n;                      // devices found (may saturate addrs[])
    // ID registers of the parts we know, read only where the device answered:
    bool    mpu_ok;   uint8_t mpu_who;      // 0x68 WHO_AM_I  (reg 0x75)
    bool    hmc_ok;   uint8_t hmc_id[3];    // 0x1E ID "H433" (reg 0x0A..0x0C)
    bool    qmc_ok;   uint8_t qmc_id;       // 0x0D chip id   (reg 0x0D)
    bool    bmp_ok;   uint8_t bmp_id;       // 0x76/0x77 id   (reg 0xD0)
};

// Walk 0x08..0x77, collect ACKs and read the known ID registers.
// Arduino-only by nature; on the host this is a no-op (never called there).
void i2c_scan_run(I2cScan& out);

// Compact JSON:
//   {"addrs":[0x1e,0x68,0x77],"mpu":"68","hmc":"483433","qmc":null,"bmp":"55"}
// Absent parts are null. Returns bytes written (excl. NUL), or 0 on any
// failure (null buffer, cap too small) — callers drop the line, never truncate.
size_t i2c_scan_json(const I2cScan& s, char* buf, size_t cap);
