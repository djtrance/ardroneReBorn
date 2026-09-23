// test_rc_settings.cpp — host tests for the new configuration + RC + sensor
// layers added for the "fly on RC, configure over WiFi, swap IMUs at build
// time" requirement.
//
//   settings     blob codec (magic/version/size/CRC16), clamping, NVS stub
//   rc_input     SBUS + Spektrum frame decode, channel map, expo, arm switch
//   sensor_math  MPU/HMC/AK8963 scaling + BMP180/BMP280 datasheet vectors
//   mixing       runtime trim / span / reverse driven by Settings
//   gps          u-blox 6 NMEA byte stream + UBX CFG-RATE/CFG-MSG packets
//
// Checklist ref: B2, D1/D2/D3, C1/C2/C3, C4, H2, I5, J2.
#include "settings.h"
#include "rc_input.h"
#include "mixing.h"
#include "config.h"
#include "gps_nav.h"
#include "logger.h"
#include "drivers/sensor_math.h"
#include "drivers/sensors.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>

static int g_pass = 0, g_fail = 0;
static void ck(bool ok, const char* name) {
    if (ok) { g_pass++; printf("  ok   %s\n", name); }
    else    { g_fail++; printf("  FAIL %s\n", name); }
}
static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

// ===========================================================================
// 1. Settings blob codec
// ===========================================================================
static void test_settings_codec() {
    printf("[settings codec]\n");

    Settings s;
    settings_defaults(s);
    ck(s.magic == SETTINGS_MAGIC, "defaults magic");
    ck(s.version == SETTINGS_VERSION, "defaults version");
    ck(s.size == sizeof(Settings), "defaults size");
    ck(settings_validate(s) == 0, "defaults are already in range");

    // Round trip through the serialised form (this is what NVS stores).
    uint8_t buf[SETTINGS_BLOB_MAX];
    size_t len = 0;
    ck(settings_serialize(s, buf, sizeof(buf), &len), "serialize");
    ck(len == sizeof(Settings), "serialize length == sizeof(Settings)");

    Settings back;
    settings_defaults(back);
    ck(settings_deserialize(buf, len, back), "deserialize");
    ck(memcmp(&s, &back, sizeof(Settings)) == 0, "round-trip is bit-exact");

    // CRC must catch a single flipped bit anywhere in the payload.
    buf[10] ^= 0x01;
    Settings reject;
    settings_defaults(reject);
    ck(!settings_deserialize(buf, len, reject), "CRC rejects one flipped bit");
    ck(reject.magic == SETTINGS_MAGIC, "rejected blob leaves target untouched");
    buf[10] ^= 0x01;

    // Wrong version => refuse (the struct layout may have changed).
    uint8_t saved_v = buf[5];
    buf[5] ^= 0xFF;
    ck(!settings_deserialize(buf, len, reject), "version mismatch rejected");
    buf[5] = saved_v;

    // Wrong length => refuse.
    ck(!settings_deserialize(buf, len - 1, reject), "length mismatch rejected");

    ck(!settings_serialize(s, buf, 4, &len), "serialize refuses undersized buffer");
}

static void test_settings_validate() {
    printf("[settings validate]\n");

    Settings s;
    settings_defaults(s);

    s.mix.pitch_span_us   = 5000;     // would drive the horn past its stops
    s.mix.differential    = 9.0f;
    s.rc.ch_pitch         = 99;       // no such channel
    s.rc.ch_arm           = 77;
    s.rc.proto            = 42;
    s.rc.expo             = -3.0f;
    s.telem.rate_hz       = 9999;
    s.wifi.http_port      = 0;

    int fixed = settings_validate(s);
    ck(fixed > 0, "validate reports corrections");
    ck(s.mix.pitch_span_us == 900, "span clamped to mechanical limit");
    ck(s.mix.differential  <= 0.40f, "differential clamped");
    ck(s.rc.expo          >= 0.0f,  "stick expo clamped (no NaN / negatives)");
    ck(s.rc.ch_pitch       <  RC_MAX_CH, "pitch channel remapped");
    ck(s.rc.ch_arm         == 0xFF, "impossible arm channel disabled");
    ck(s.rc.proto          == RC_PROTO_NONE, "unknown protocol falls back to NONE");
    ck(s.telem.rate_hz     <= 200, "telemetry rate clamped");
    ck(s.wifi.http_port    >= 1, "http port clamped");
    ck(settings_validate(s) == 0, "validate is idempotent");

    // NaN must never survive clamping (a fat-fingered portal field).
    Settings s2;
    settings_defaults(s2);
    s2.rc.expo = nanf("");
    settings_validate(s2);
    ck(s2.rc.expo == 0.0f, "NaN field replaced by a safe default");
}

static void test_settings_proto_defaults() {
    printf("[settings rc protocol ranges]\n");

    RcSettings rc;
    memset(&rc, 0, sizeof(rc));
    rc.proto = RC_PROTO_SBUS;
    rc_apply_proto_defaults(rc);
    ck(rc.min_raw == 172 && rc.mid_raw == 992 && rc.max_raw == 1811,
       "SBUS stick range 172/992/1811");

    rc.proto = RC_PROTO_SPEKTRUM;
    rc_apply_proto_defaults(rc);
    ck(rc.min_raw == 0 && rc.mid_raw == 512 && rc.max_raw == 1023,
       "Spektrum 1024-domain range 0/512/1023");
    ck(rc.loss_timeout_ms == RC_LOSS_TIMEOUT_MS, "loss timeout from config.h");

    rc.proto = RC_PROTO_NONE;
    rc_apply_proto_defaults(rc);
    ck(rc.mid_raw == 1500, "plain PWM fallback range");
}

// ===========================================================================
// 2. SBUS decode
// ===========================================================================
// Independent bit-packer: channel n starts at bit 11*n of the data area.
static void sbus_pack(uint8_t* b, const uint16_t* ch, uint8_t flags) {
    memset(b, 0, 25);
    b[0] = 0x0F;
    uint8_t data[22];
    memset(data, 0, sizeof(data));
    for (int i = 0; i < 16; i++) {
        uint32_t v = ch[i] & 0x7FF;
        for (int k = 0; k < 11; k++) {
            if ((v >> k) & 1u) {
                int p = i * 11 + k;
                data[p >> 3] |= (uint8_t)(1u << (p & 7));
            }
        }
    }
    memcpy(b + 1, data, 22);
    b[23] = flags;
    b[24] = 0x00;
}

static void test_sbus_decode() {
    printf("[rc sbus]\n");

    uint16_t expect[16];
    for (int i = 0; i < 16; i++) expect[i] = (uint16_t)(172 + i * 100);
    expect[0] = 172;    expect[1] = 992;    expect[2] = 1811;

    uint8_t frame[25];
    sbus_pack(frame, expect, 0x00);

    RcFrame f;
    ck(sbus_decode(frame, f), "valid SBUS frame accepted");
    ck(f.frame_ok, "frame_ok set");
    ck(!f.failsafe && !f.frame_lost, "no failsafe with flags=0");
    bool all = true;
    for (int i = 0; i < 16; i++) if (f.raw[i] != expect[i]) all = false;
    ck(all, "all 16 channels round-trip bit-exact");

    // Failsafe bit (0x10) and frame-lost bit (0x08).
    sbus_pack(frame, expect, 0x10);
    ck(sbus_decode(frame, f) && f.failsafe, "failsafe flag decoded");
    sbus_pack(frame, expect, 0x08);
    ck(sbus_decode(frame, f) && f.frame_lost, "frame-lost flag decoded");

    // Header and footer are validated, so noise never reaches the mixer.
    sbus_pack(frame, expect, 0x00);
    frame[0] = 0x5A;
    ck(!sbus_decode(frame, f), "bad header rejected");
    frame[0] = 0x0F;
    frame[24] = 0x7E;
    ck(!sbus_decode(frame, f), "bad footer rejected");
    ck(!sbus_decode(nullptr, f), "null buffer rejected");
}

// ===========================================================================
// 3. Spektrum decode
// ===========================================================================
static void spek_pack(uint8_t* b, const uint16_t* ids, const uint16_t* vals,
                      int n, bool mode2048) {
    memset(b, 0, 16);
    b[0] = mode2048 ? 0x01 : 0x00;    // bit0 selects the sub-mode
    b[1] = 0x00;
    b[2] = 0x00;                      // not in failsafe
    for (int i = 0; i < 6; i++) {
        uint16_t id  = (i < n) ? ids[i]  : 0;
        uint16_t val = (i < n) ? vals[i] : 0;
        uint16_t w;
        if (mode2048) w = (uint16_t)((id << 11) | (val & 0x7FF));
        else          w = (uint16_t)((id << 10) | (val & 0x3FF));
        b[3 + 2 * i] = (uint8_t)(w >> 8);
        b[4 + 2 * i] = (uint8_t)(w & 0xFF);
    }
    b[15] = 0xC8;                      // signal quality
}

static void test_spektrum_decode() {
    printf("[rc spektrum]\n");

    // --- 1024 mode (DSM2): 6-bit channel id in bits15-10, value in bits9-0 ---
    uint16_t ids[6]  = {0, 1, 2, 3, 4, 5};
    uint16_t vals[6] = {0, 512, 1023, 256, 768, 100};
    uint8_t frame[16];
    spek_pack(frame, ids, vals, 6, false);

    RcFrame f;
    ck(spektrum_decode(frame, f), "1024-mode frame accepted");
    ck(f.count == 6, "six distinct channels");
    ck(f.raw[0] == 0 && f.raw[1] == 512 && f.raw[2] == 1023,
       "1024-mode values decoded");
    ck(f.raw[3] == 256 && f.raw[4] == 768 && f.raw[5] == 100,
       "1024-mode values decoded (tail)");
    ck(f.link_quality == 0xC8, "signal-quality byte captured");
    ck(!f.failsafe, "status byte 0 => not failsafe");

    // --- 2048 mode (DSMX): 4-bit channel id in bits14-11, 11-bit value -------
    uint16_t ids11[6]  = {0, 1, 2, 3, 4, 5};
    uint16_t vals11[6] = {2047, 1024, 0, 1500, 512, 2000};
    spek_pack(frame, ids11, vals11, 6, true);
    ck(spektrum_decode(frame, f), "2048-mode frame accepted");
    ck(f.raw[0] == 1023, "2048-mode 2047 -> 1023 (normalised to 10 bits)");
    ck(f.raw[1] == 512 && f.raw[2] == 0, "2048-mode midpoint/zero");
    ck(f.raw[3] == 750 && f.raw[4] == 256 && f.raw[5] == 1000,
       "2048-mode remaining channels normalised");

    // --- failsafe -----------------------------------------------------------
    spek_pack(frame, ids, vals, 6, false);
    frame[2] = 0x01;
    ck(spektrum_decode(frame, f) && f.failsafe, "Spektrum failsafe bit decoded");

    ck(!spektrum_decode(nullptr, f), "null buffer rejected");
}

// ===========================================================================
// 4. Channel mapping / normalisation
// ===========================================================================
static void test_rc_mapping() {
    printf("[rc mapping]\n");

    RcSettings rc;
    memset(&rc, 0, sizeof(rc));
    rc.proto = RC_PROTO_SBUS;
    rc_apply_proto_defaults(rc);
    rc.deadband_raw = 20;
    rc.expo = 0.0f;
    // AETR-ish map on channel indices 0..6 (this is what the portal edits).
    rc.ch_roll = 0; rc.ch_pitch = 1; rc.ch_throttle = 2; rc.ch_yaw = 3;
    rc.ch_arm = 4; rc.ch_rth = 5; rc.ch_flightmode = 6;
    rc.arm_high_is_armed = 1;       // safe default: switch DOWN = disarmed

    ck(near(rc_raw_to_norm(rc.min_raw, rc), -1.0f, 1e-6), "SBUS low = -1");
    ck(near(rc_raw_to_norm(rc.mid_raw, rc),  0.0f, 1e-6), "SBUS mid =  0");
    ck(near(rc_raw_to_norm(rc.max_raw, rc),  1.0f, 1e-6), "SBUS high = +1");
    ck(near(rc_raw_to_norm(rc.mid_raw + 10, rc), 0.0f, 1e-6),
       "centre deadband swallows a mis-trimmed stick");
    ck(rc_raw_to_norm(3000, rc) == 1.0f, "over-range clamped to +1");
    ck(rc_raw_to_norm(0, rc) == -1.0f, "under-range clamped to -1");

    ck(near(rc_apply_expo(1.0f, 0.7f), 1.0f, 1e-6), "expo keeps the endpoint");
    ck(near(rc_apply_expo(-1.0f, 0.7f), -1.0f, 1e-6), "expo keeps -endpoint");
    ck(near(rc_apply_expo(0.5f, 0.0f), 0.5f, 1e-6), "expo 0 = linear");
    ck(rc_apply_expo(0.5f, 1.0f) < 0.5f, "expo softens the centre");

    // --- full frame -> sticks -----------------------------------------------
    uint16_t raw[16];
    for (int i = 0; i < 16; i++) raw[i] = 992;
    raw[0] = 1811;   // roll full right
    raw[1] = 172;    // pitch full down
    raw[2] = 1811;   // throttle high
    raw[4] = 172;    // arm switch low
    raw[5] = 1811;   // rth switch high
    raw[6] = 992;    // mode switch centre

    uint8_t frame[25];
    sbus_pack(frame, raw, 0x00);

    RcFrame f;
    RcInput in;
    rc_input_defaults(in);
    ck(sbus_decode(frame, f), "frame for mapping");
    ck(rc_map(f, rc, 1000, in), "rc_map succeeds");
    ck(near(in.roll, 1.0f, 1e-6), "roll stick mapped");
    ck(near(in.pitch, -1.0f, 1e-6), "pitch stick mapped");
    ck(near(in.throttle, 1.0f, 1e-6), "throttle mapped to 0..1");
    ck(!in.arm, "arm switch LOW disarms (arm_high_is_armed=1, the safe default)");
    ck(in.rth, "RTH switch HIGH asserted");
    ck(in.mode == 1, "mode switch centre = position 1");
    ck(in.ok, "frame is good");

    // Flip arm polarity — a very common radio-setup mistake, and the reason
    // it lives in the portal instead of a #define.
    rc.arm_high_is_armed = 0;
    ck(rc_map(f, rc, 1000, in) && in.arm, "arm polarity honoured (LOW arms)");
    raw[4] = 1811;
    sbus_pack(frame, raw, 0x00);
    sbus_decode(frame, f);
    ck(rc_map(f, rc, 1000, in) && !in.arm, "HIGH disarms under that polarity");
    rc.arm_high_is_armed = 1;
    raw[4] = 172;

    // Failsafe frame => not ok, even though channels still decode.
    sbus_pack(frame, raw, 0x10);
    sbus_decode(frame, f);
    rc_map(f, rc, 1000, in);
    ck(!in.ok, "failsafe frame => rc not ok");
}

// ===========================================================================
// 5. Sensor maths
// ===========================================================================
static void put_be16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }
static void put_be16s(uint8_t* p, int v)     { put_be16(p, (uint16_t)(int16_t)v); }
static void put_le16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static void put_le16s(uint8_t* p, int v)     { put_le16(p, (uint16_t)(int16_t)v); }

static void test_sensor_math() {
    printf("[sensor math]\n");

    // --- MPU6050 / MPU9250 full-scale scaling -------------------------------
    ck(near(mpu_accel_lsb_per_g(0), 16384.0f, 1e-6), "+/-2g  -> 16384 LSB/g");
    ck(near(mpu_accel_lsb_per_g(3),  2048.0f, 1e-6), "+/-16g ->  2048 LSB/g");
    ck(near(mpu_gyro_lsb_per_dps(0),   131.0f, 1e-6), "+/-250 dps -> 131");
    ck(near(mpu_gyro_lsb_per_dps(3),    16.4f, 1e-6), "+/-2000 dps -> 16.4");

    int16_t a[3] = {0, 0, 16384};            // 1 g on +Z at +/-2g
    Vec3 av = mpu_accel_to_g(a, 0);
    ck(near(av.z, 1.0f, 1e-6) && near(av.x, 0.0f, 1e-6), "1 g -> 1.0");

    int16_t g[3] = {164, 0, -328};           // 1 and -2 dps at +/-200 dps
    Vec3 gv = mpu_gyro_to_dps(g, 0);
    ck(near(gv.x, 164.0f / 131.0f, 1e-4), "gyro raw -> dps");
    ck(near(gv.z, -328.0f / 131.0f, 1e-4), "gyro sign preserved");

    // --- HMC5883L gain table ------------------------------------------------
    ck(near(hmc_lsb_per_ut(0x20), 10.90f, 1e-4), "gain 1090 LSB/Ga -> 10.9 LSB/uT");
    int16_t hm[3] = {1090, 0, -1090};
    Vec3 hv = hmc_to_ut(hm, 0x20);
    ck(near(hv.x, 100.0f, 1e-3) && near(hv.z, -100.0f, 1e-3),
       "1090 LSB/Ga == 100 uT");

    // --- AK8963 -------------------------------------------------------------
    ck(near(ak8963_lsb_per_ut(true), 4912.0f / 32768.0f, 1e-6), "AK8963 16-bit scale");
    ck(near(ak8963_lsb_per_ut(false), 4912.0f / 8192.0f, 1e-6), "AK8963 14-bit scale");
    int16_t ak[3] = {32767, 0, 0};
    float asa[3] = {128, 128, 128};           // unity adjustment
    Vec3 kv = ak8963_to_ut(ak, true, asa);
    ck(near(kv.x, 32767.0f * (4912.0f / 32768.0f), 0.01), "AK8963 unity ASA");
    float asa_hot[3] = {255, 128, 1};         // strong positive factory trim
    Vec3 kv2 = ak8963_to_ut(ak, true, asa_hot);
    ck(kv2.x > kv.x, "ASA trim scales the axis");
    ck(ak8963_lsb_per_ut(true) * 32768.0f > 4911.0f, "full scale ~4912 uT");

    // --- BMP180: Bosch datasheet worked example ----------------------------
    // cal AC1=408 AC2=-72 AC3=-14383 AC4=32741 AC5=32757 AC6=23153
    //     B1=6190 B2=4 MB=-32767 MC=-8711 MD=2868, UT=27898 UP=23843 oss=0
    // -> T = 150 (15.0 C), p = 69965 Pa
    uint8_t cal22[22];
    put_be16s(cal22 +  0,   408);
    put_be16s(cal22 +  2,   -72);
    put_be16s(cal22 +  4, -14383);
    put_be16 (cal22 +  6,  32741);
    put_be16 (cal22 +  8,  32757);
    put_be16 (cal22 + 10,  23153);
    put_be16s(cal22 + 12,   6190);
    put_be16s(cal22 + 14,      4);
    put_be16s(cal22 + 16, -32767);
    put_be16s(cal22 + 18,  -8711);
    put_be16s(cal22 + 20,   2868);

    Bmp180Cal c180;
    ck(bmp180_parse_cal(cal22, c180), "bmp180 calibration parses");
    ck(c180.ac1 == 408 && c180.ac3 == -14383 && c180.ac6 == 23153,
       "bmp180 signed/unsigned fields decoded big-endian");
    ck(c180.b1 == 6190 && c180.mc == -8711 && c180.md == 2868,
       "bmp180 B1/MC/MD decoded");

    float press = 0, temp = 0;
    bmp180_compensate(c180, 23843, 27898, 0, press, temp);
    ck(near(temp, 15.0f, 0.05), "bmp180 datasheet T = 15.0 C");
    ck(near(press, 69965.0f, 1.0), "bmp180 datasheet p = 69965 Pa");

    uint8_t all_ff[22];
    memset(all_ff, 0xFF, sizeof(all_ff));
    Bmp180Cal dummy;
    ck(!bmp180_parse_cal(all_ff, dummy), "bmp180 rejects unprogrammed part");

    // --- BMP280: Bosch datasheet worked example ----------------------------
    // dig_T1..T3 = 27504 / 26435 / -1000, adc_T = 519888 -> T = 25.08 C
    // dig_P1..P9 = 36477 / -10685 / 3024 / 2855 / 140 / -7 / 15500 /
    //              -14600 / 6000,  adc_P = 415148 -> p ~ 100653 Pa
    uint8_t cal24[24];
    put_le16 (cal24 +  0, 27504);
    put_le16s(cal24 +  2, 26435);
    put_le16s(cal24 +  4, -1000);
    put_le16 (cal24 +  6, 36477);
    put_le16s(cal24 +  8, -10685);
    put_le16s(cal24 + 10,  3024);
    put_le16s(cal24 + 12,  2855);
    put_le16s(cal24 + 14,   140);
    put_le16s(cal24 + 16,    -7);
    put_le16s(cal24 + 18, 15500);
    put_le16s(cal24 + 20, -14600);
    put_le16s(cal24 + 22,  6000);

    Bmp280Cal c280;
    ck(bmp280_parse_cal(cal24, c280), "bmp280 calibration parses");
    ck(c280.t1 == 27504 && c280.t2 == 26435 && c280.t3 == -1000,
       "bmp280 temperature coefficients little-endian");
    ck(c280.p9 == 6000 && c280.p8 == -14600, "bmp280 pressure coefficients");

    int32_t tf = 0;
    float t280 = bmp280_compensate_temp_c(c280, 519888, tf);
    ck(near(t280, 25.08f, 0.02), "bmp280 datasheet T = 25.08 C");

    float p280 = bmp280_compensate_press_pa(c280, 415148, tf);
    ck(p280 > 100500.0f && p280 < 100800.0f, "bmp280 datasheet p ~ 100653 Pa");

    // Higher raw pressure register == lower physical pressure (20-bit, inverted).
    int32_t tf2 = 0;
    bmp280_compensate_temp_c(c280, 519888, tf2);
    float p_lo = bmp280_compensate_press_pa(c280, 500000, tf2);
    float p_hi = bmp280_compensate_press_pa(c280, 300000, tf2);
    ck(p_hi > p_lo, "bmp280 raw/physical pressure relationship is inverted");
    ck(p_lo > 0.0f, "bmp280 never returns a negative altitude datum");

    Bmp280Cal bad;
    uint8_t zeros[24];
    memset(zeros, 0, sizeof(zeros));
    ck(!bmp280_parse_cal(zeros, bad), "bmp280 rejects unprogrammed part");
}

// ===========================================================================
// 6. GPS — u-blox 6 NMEA byte stream + UBX config packets  (checklist C4)
// ===========================================================================
// Classic NMEA worked examples, checksums recomputed: GGA *47, RMC *6A.
static const char* k_gga =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const char* k_gga_nofix =
    "$GPGGA,123519,,,,,0,00,,,M,,M,,*6B\r\n";
static const char* k_rmc =
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\n";

static void test_gps_ublox6() {
    printf("[gps ublox6]\n");

    GpsFix f;
    memset(&f, 0, sizeof(f));
    gps_init();                                  // host path: flag + reset
    ck(!gps_healthy(), "unhealthy before any line arrives");

    // --- byte-by-byte GGA, split arbitrarily by the UART ------------------
    bool upd = false;
    for (const char* p = k_gga; *p; ++p)          // '\r' closes it, '\n' is
        upd = upd || gps_feed_byte(*p, f);        // then an empty line
    ck(upd, "GGA parsed when fed one byte at a time");
    ck(f.valid && f.fix_quality == 1, "GGA quality 1 => valid fix");
    ck(near(f.lat, 48.1173, 1e-6) && near(f.lon, 11.5166667, 1e-6),
       "GGA lat/lon converted to decimal degrees");
    ck(near(f.alt_m, 545.4, 0.01), "GGA altitude metres");
    ck(f.num_sv == 8 && near(f.hdop, 0.9, 1e-4) && f.utc == 123519,
       "GGA sats / hdop / utc");
    ck(gps_lines_seen() == 1, "CRLF counts as exactly one line");
    ck(gps_healthy(), "healthy after first checksum-valid line");
    ck(gps_trustworthy(f), "healthy GGA fix passes the arm/RTH gate");

    // --- RMC in two chunks (as the UART would deliver them) ---------------
    gps_feed(k_rmc, 17, f);
    ck(f.ground_mps == 0.0f && gps_lines_seen() == 1,
       "half a sentence updates nothing");
    gps_feed(k_rmc + 17, strlen(k_rmc) - 17, f);
    ck(near(f.ground_mps, 11.5235, 1e-3), "RMC 22.4 kt -> 11.52 m/s");
    ck(near(f.course_deg, 84.4, 1e-3), "RMC course-over-ground");
    ck(gps_lines_seen() == 2, "second sentence counted");

    // --- gate rejects degraded fixes --------------------------------------
    GpsFix g = f; g.hdop = 3.0f;
    ck(!gps_trustworthy(g), "hdop above GPS_MAX_HDOP rejected");
    g = f; g.num_sv = 4;
    ck(!gps_trustworthy(g), "fewer than 6 satellites rejected");

    // --- no-fix GGA: EMPTY fields must clear the stale valid flag ---------
    // (strtok would collapse the empties and shift the fields — this is the
    //  case the GPS_LOSS failsafe depends on).
    GpsFix nf;
    memset(&nf, 0, sizeof(nf));
    gps_feed(k_gga, strlen(k_gga), nf);          // get a fix first
    ck(nf.valid, "fix established before signal loss");
    gps_feed(k_gga_nofix, strlen(k_gga_nofix), nf);
    ck(!nf.valid && nf.fix_quality == 0 && nf.num_sv == 0,
       "no-fix GGA clears valid (no stale position for failsafe)");
    ck(!gps_trustworthy(nf), "no-fix GGA fails the gate");

    // --- resynchronisation -------------------------------------------------
    GpsFix r;
    memset(&r, 0, sizeof(r));
    gps_line_reset();
    const char* noise = "\xFF\xFE" "PGGA,garbage,,,\r\n";   // no leading '$'
    gps_feed(noise, strlen(noise), r);
    ck(gps_lines_seen() == 0 && !r.valid, "line noise without '$' never counts");

    memset(&r, 0, sizeof(r));
    gps_feed(k_gga, strlen(k_gga), r);
    ck(r.valid && gps_lines_seen() == 1, "clean sentence after noise");

    gps_line_reset();
    memset(&r, 0, sizeof(r));
    const char* trunc = "$GPGGA,123519,4807.038,N,011";
    gps_feed(trunc, strlen(trunc), r);                  // truncated frame
    ck(!r.valid && gps_lines_seen() == 0, "truncated frame yields nothing");
    gps_feed(k_gga, strlen(k_gga), r);
    ck(r.valid && gps_lines_seen() == 1, "'$' resync drops the truncated frame");

    gps_line_reset();
    memset(&r, 0, sizeof(r));
    const char* bad =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n";
    gps_feed(bad, strlen(bad), r);
    ck(gps_lines_seen() == 0 && !r.valid, "wrong checksum rejected");
    gps_feed(k_gga, strlen(k_gga), r);
    ck(r.valid && gps_lines_seen() == 1, "good sentence right after a bad one");

    gps_line_reset();
    memset(&r, 0, sizeof(r));
    for (int i = 0; i < 400; ++i) gps_feed_byte('x', r);   // buffer overflow
    gps_feed_byte('\n', r);
    ck(gps_lines_seen() == 0, "overlong garbage line dropped");
    gps_feed(k_gga, strlen(k_gga), r);
    ck(r.valid && gps_lines_seen() == 1, "recovers after buffer overflow");

    // --- UBX-CFG-RATE (spec worked example, 200 ms => ... DE 6A) ----------
    uint8_t pkt[20];
    static const uint8_t want_rate200[14] = {
        0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A };
    int n = ubx_build_cfg_rate(pkt, sizeof(pkt), 200);
    ck(n == 14 && memcmp(pkt, want_rate200, 14) == 0,
       "CFG-RATE 200 ms matches u-blox worked example");

    ck(GPS_RATE_MS == 200,
       "configured rate == NEO-6 datasheet maximum (5 Hz / 200 ms)");
    n = ubx_build_cfg_rate(pkt, sizeof(pkt), GPS_RATE_MS);
    ck(n == 14 && memcmp(pkt, want_rate200, 14) == 0,
       "configured rate builds the spec vector");
    ck(ubx_build_cfg_rate(pkt, sizeof(pkt), 100) == 0,
       "CFG-RATE rejects 10 Hz — out of spec for NEO-6 (packet loss/resets)");
    ck(ubx_build_cfg_rate(pkt, 13, 200) == 0, "CFG-RATE rejects short buffer");

    // --- UBX-CFG-PRT: UART1 -> 115200 8N1 (the verified line-rate jump) ----
    static const uint8_t want_prt[28] = {
        0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
        0x00,0xC2,0x01,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xBC,0x5E };
    uint8_t prt[28];
    n = ubx_build_cfg_prt_uart(prt, sizeof(prt), 115200);
    ck(n == 28 && memcmp(prt, want_prt, 28) == 0,
       "CFG-PRT UART1 -> 115200 8N1 (mode 0x8D0, UBX|NMEA in/out)");
    n = ubx_build_cfg_prt_uart(prt, sizeof(prt), 9600);
    ck(n == 28 && prt[14] == 0x80 && prt[15] == 0x25 &&
       prt[16] == 0x00 && prt[17] == 0x00,
       "CFG-PRT baud field is little-endian (9600 = 0x2580)");
    ck(ubx_build_cfg_prt_uart(prt, sizeof(prt), 12345) == 0,
       "CFG-PRT rejects a baud outside the spec list");
    ck(ubx_build_cfg_prt_uart(prt, 27, 115200) == 0,
       "CFG-PRT rejects short buffer");
    ck(GPS_USE_HI_BAUD == 1 && GPS_BAUD_HI == 115200,
       "config enables the verified jump to 115200");

    // --- UBX-CFG-MSG: GSV off, RMC on, UART1 only -------------------------
    static const uint8_t want_gsv[16] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,
        0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38 };
    n = ubx_build_cfg_msg_nmea(pkt, sizeof(pkt), 0x03, 0);
    ck(n == 16 && memcmp(pkt, want_gsv, 16) == 0,
       "CFG-MSG disables GSV (all ports zero)");

    static const uint8_t want_rmc[16] = {
        0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,
        0x00,0x01,0x00,0x00,0x00,0x00,0x04,0x44 };
    n = ubx_build_cfg_msg_nmea(pkt, sizeof(pkt), 0x04, 1);
    ck(n == 16 && memcmp(pkt, want_rmc, 16) == 0,
       "CFG-MSG enables RMC on UART1 only");
    ck(ubx_build_cfg_msg_nmea(pkt, sizeof(pkt), 0x04, 3) == 0,
       "CFG-MSG rejects an out-of-range rate");
    ck(ubx_build_cfg_msg_nmea(pkt, 15, 0x04, 1) == 0,
       "CFG-MSG rejects short buffer");
}

// ===========================================================================
// 7. Mixing driven by runtime settings
// ===========================================================================
static void test_mixing_settings() {
    printf("[mixing settings]\n");

    MixSettings base = mixing_settings();     // config.h defaults

    // Span is a real mechanical limit: full roll saturates at trim +/- span
    // instead of running into the electrical servo stop.
    ElevonOut sat = mix_elevons(1.0f, 1.0f, 0.0f);
    ck(sat.right_us == (uint16_t)(base.elevon_r_trim_us + base.pitch_span_us),
       "combined cmd saturates at trim+span");

    // Servo reverse (the #1 thing a pilot must flip from the portal).
    MixSettings m = base;
    m.elevon_l_reverse = 1;
    mixing_load(&m);
    ElevonOut r = mix_elevons(0.5f, 0.0f, 0.0f);
    ck(r.left_us < base.elevon_l_trim_us, "reversed elevon moves below trim");
    ck(r.right_us > base.elevon_r_trim_us, "unreversed elevon still above trim");

    // Per-surface trim offset moves the neutral point.
    m = base;
    m.elevon_l_trim_us = 1560;
    mixing_load(&m);
    ElevonOut t = mix_elevons(0.0f, 0.0f, 0.0f);
    ck(t.left_us == 1560 && t.right_us == base.elevon_r_trim_us,
       "left trim applied to the neutral point");

    // Differential is a setting, not a compile-time constant. It must push
    // the two surfaces in *opposite* directions (that is the whole point:
    // more up-elevon deflection, less down-elevon deflection).
    ElevonOut d_zero;
    {
        MixSettings m0 = base;
        m0.differential = 0.0f;
        mixing_load(&m0);
        d_zero = mix_elevons(0.2f, 0.5f, 0.0f);
        mixing_load(&base);
    }
    ElevonOut d_base = mix_elevons(0.2f, 0.5f, 0.0f);
    mixing_load(&base);
    int dl = (int)d_base.left_us  - (int)d_zero.left_us;
    int dr = (int)d_base.right_us - (int)d_zero.right_us;
    ck(dl > 0 && dr < 0 && dl == -dr,
       "differential shifts the two surfaces in opposite directions");

    // ESC range is configurable too (some ESCs want 1050..1950).
    m = base;
    m.esc_min_us = 1050;
    m.esc_max_us = 1950;
    mixing_load(&m);
    ElevonOut e0 = mix_elevons(0, 0, 0.0f);
    ElevonOut e1 = mix_elevons(0, 0, 1.0f);
    ck(e0.throttle_us == 1050 && e1.throttle_us == 1950, "ESC range honoured");

    mixing_load(nullptr);                     // back to config.h defaults
    ElevonOut n = mix_elevons(0, 0, 0.0f);
    ck(n.left_us == base.elevon_l_trim_us, "NULL restores compile-time defaults");
}

// ===========================================================================
// Stage-1 passthrough setting (etapa 1: RX -> elevon mix, test-campaign T0)
// ===========================================================================
static void test_settings_passthrough() {
    printf("[settings passthrough]\n");

    Settings s;
    settings_defaults(s);
    ck(s.mix.passthrough == 1, "default passthrough ON (stage-1 safe)");
    ck(settings_validate(s) == 0, "defaults validate clean");

    s.mix.passthrough = 7;
    ck(settings_validate(s) == 0, "passthrough normalises without 'fixed' count");
    ck(s.mix.passthrough == 1, "any non-zero becomes 1");

    s.mix.passthrough = 0;
    ck(settings_validate(s) == 0 && s.mix.passthrough == 0, "0 survives validate");

    // Round trip through the blob (this is what NVS stores).
    uint8_t buf[SETTINGS_BLOB_MAX];
    size_t len = 0;
    ck(settings_serialize(s, buf, sizeof(buf), &len), "serialize with passthrough=0");
    Settings back;
    settings_defaults(back);
    ck(settings_deserialize(buf, len, back), "deserialize");
    ck(back.mix.passthrough == 0, "passthrough survives the blob round trip");
}

// ===========================================================================
// Real-time CSV logger formatter (I3) — the on-wire schema is a contract
// with tools/wing_logger/*.py, so column count and spot formats are pinned.
// ===========================================================================
static int count_char(const char* t, char c) {
    int n = 0;
    for (const char* p = t; *p; p++) if (*p == c) n++;
    return n;
}

static void test_logger_csv() {
    printf("[logger csv]\n");

    LogSample s;
    memset(&s, 0, sizeof(s));

    char line[LOGGER_MAX_LINE];
    size_t n = logger_format(line, sizeof(line), s);
    ck(n > 0, "formats a zeroed sample");
    ck(line[n] == 0, "NUL terminated");
    ck(strchr(line, '\n') != nullptr, "line ends with newline");
    ck(count_char(line, ',') == count_char(logger_csv_header(), ','),
       "data columns match header columns");
    ck(strncmp(line, "0,0,0,0,0,0,0,0,", 16) == 0, "integer preamble spot check");

    // A fully-populated sample: pin the exact formats used per column group.
    s.t_ms = 123456; s.mode = 1; s.armed = 1; s.rc_ok = 1;
    s.phase = 2; s.fs_evt = 7; s.env = 0x05; s.wind = 3;
    s.rc_p = -1.0f; s.rc_r = 0.5f; s.rc_t = 0.25f; s.rc_y = 0.125f;
    s.raw_p = 172; s.raw_r = 992; s.raw_t = 1811; s.raw_y = 512;
    s.ax = 1.0f; s.ay = -0.5f; s.az = 0.25f;
    s.gx = 0.001f; s.gy = -0.002f; s.gz = 0.003f;
    s.baro_pa = 101325.0f; s.baro_c = 21.5f;
    s.roll = 12.34f; s.pitch = -5.67f; s.yaw = 180.0f;
    s.fix = 1; s.sv = 11; s.hdop = 0.9f;
    s.gps_v = 15.2f; s.gps_alt = 123.4f; s.gps_trk = 270.0f; s.vest = 15.0f;
    s.phi_cmd = 0.5f; s.l_us = 1520; s.r_us = 1480;
    s.thr_out = 0.75f; s.overruns = 9;

    n = logger_format(line, sizeof(line), s);
    ck(n > 0, "formats a populated sample");
    ck(strncmp(line, "123456,1,1,1,2,7,5,3,", 21) == 0,
       "preamble: t,mode,armed,rc_ok,phase,fs,env,wind");
    ck(strstr(line, ",-1.000,0.500,0.250,0.125,172,992,1811,512,") != nullptr,
       "sticks (%.3f) + raw channel units (%u)");
    ck(strstr(line, ",101325.0,21.50,") != nullptr,
       "baro: %.1f Pa, %.2f C");
    ck(strstr(line, ",180.00,") != nullptr, "attitude in %.2f deg");
    ck(strstr(line, ",0.5000,1520,1480,0.750,9\n") != nullptr,
       "phi_cmd %.4f, surfaces %u/%u, throttle %.3f, overruns %lu");

    // Non-finite must still produce a parseable field, never an empty column.
    s.gps_v = nanf("");
    n = logger_format(line, sizeof(line), s);
    ck(n > 0 && strstr(line, "nan") != nullptr, "NaN renders as a 'nan' field");

    char tiny[8];
    ck(logger_format(tiny, sizeof(tiny), s) == 0, "undersized buffer rejected");
    ck(logger_format(nullptr, 64, s) == 0, "null buffer rejected");
}

// ===========================================================================
int main() {
    printf("=== esp32-airplane config/RC/sensor tests ===\n");
    test_settings_codec();
    test_settings_validate();
    test_settings_passthrough();
    test_settings_proto_defaults();
    test_sbus_decode();
    test_spektrum_decode();
    test_rc_mapping();
    test_sensor_math();
    test_gps_ublox6();
    test_mixing_settings();
    test_logger_csv();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
