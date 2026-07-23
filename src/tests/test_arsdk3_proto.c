#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../../src/network/arsdk3_proto.h"

static int failed = 0, passed = 0;

#define TEST(name) do { printf("  %-45s ", name); } while(0)
#define PASS() do { printf("PASS\n"); passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); failed++; } while(0)

static void test_build_arnet_header(void) {
    TEST("ARNetworkAL header size + fields");
    uint8_t buf[64];
    int off = build_arnet_header(buf, sizeof(buf), 4, 127, 1, 20);
    if (off != 7) { FAIL("wrong offset"); return; }
    if (buf[0] != 4)  { FAIL("type"); return; }
    if (buf[1] != 127){ FAIL("id"); return; }
    if (buf[2] != 1)  { FAIL("seq"); return; }
    uint32_t total = buf[3] | (buf[4]<<8) | (buf[5]<<16) | (buf[6]<<24);
    if (total != 27)  { FAIL("size"); return; }
    PASS();
}

static void test_build_arstream_basic(void) {
    TEST("ARStream single fragment packet");
    uint8_t buf[128];
    uint8_t h264[] = {0x00,0x00,0x00,0x01,0x65,0x88,0x84};
    int len = build_arstream_packet(buf, sizeof(buf), 0, 1, 1, 0, 1, h264, sizeof(h264));
    if (len < 0) { FAIL("build failed"); return; }
    if (buf[0] != 3)  { FAIL("type not DATA_LOW_LATENCY"); return; }
    if (buf[1] != 125){ FAIL("id not VIDEO_DATA"); return; }
    int ars_hdr = 7;
    uint16_t fn = buf[ars_hdr] | (buf[ars_hdr+1]<<8);
    if (fn != 1)      { FAIL("frame_number wrong"); return; }
    if (buf[ars_hdr+2] != 1) { FAIL("FLUSH_FRAME not set"); return; }
    if (buf[ars_hdr+3] != 0) { FAIL("fragment_number wrong"); return; }
    if (buf[ars_hdr+4] != 1) { FAIL("fragments_per_frame wrong"); return; }
    if (memcmp(buf + ars_hdr + 5, h264, sizeof(h264)) != 0) { FAIL("h264 data mismatch"); return; }
    uint32_t total = buf[3] | (buf[4]<<8) | (buf[5]<<16) | (buf[6]<<24);
    if ((int)total != len) { FAIL("total size mismatch"); return; }
    PASS();
}

static void test_build_arstream_multi_fragment(void) {
    TEST("ARStream multi-fragment (SPS+PPS+IDR)");
    uint8_t buf1[256], buf2[256], buf3[256];
    uint8_t sps[] = {0x67,0x42,0x00,0x1e};
    uint8_t pps[] = {0x68,0xce,0x3c,0x80};
    uint8_t idr[] = {0x65,0x88,0x84,0x00,0x01,0x02,0x03,0x04,0x05};

    int f1 = build_arstream_packet(buf1, sizeof(buf1), 0, 42, 0, 0, 3, sps, sizeof(sps));
    int f2 = build_arstream_packet(buf2, sizeof(buf2), 0, 42, 0, 1, 3, pps, sizeof(pps));
    int f3 = build_arstream_packet(buf3, sizeof(buf3), 0, 42, 1, 2, 3, idr, sizeof(idr));

    if (f1 < 0 || f2 < 0 || f3 < 0) { FAIL("build failed"); return; }
    if (buf1[7+3] != 0 || buf1[7+4] != 3) { FAIL("fragment 1 fields"); return; }
    if (buf2[7+3] != 1) { FAIL("fragment 2 index wrong"); return; }
    if (buf3[7+2] != 1) { FAIL("fragment 3 FLUSH_FRAME not set"); return; }
    if (buf3[7+3] != 2) { FAIL("fragment 3 index wrong"); return; }
    PASS();
}

static void test_parse_arnet_frame(void) {
    TEST("parse ARNetworkAL frame");
    uint8_t buf[64];
    build_arnet_header(buf, sizeof(buf), 2, 126, 5, 10);
    uint8_t type, id, seq;
    int poff, plen;
    int r = parse_arnet_frame(buf, 17, &type, &id, &seq, &poff, &plen);
    if (r != 0)           { FAIL("parse failed"); return; }
    if (type != 2)        { FAIL("type wrong"); return; }
    if (id != 126)        { FAIL("id wrong"); return; }
    if (seq != 5)         { FAIL("seq wrong"); return; }
    if (poff != 7)        { FAIL("payload offset wrong"); return; }
    if (plen != 10)       { FAIL("payload len wrong"); return; }
    PASS();
}

static void test_parse_arnet_too_short(void) {
    TEST("parse ARNetworkAL rejects short frame");
    uint8_t buf[] = {0,1,2,0,0,0,0};
    uint8_t type, id, seq;
    int poff, plen;
    int r = parse_arnet_frame(buf, 6, &type, &id, &seq, &poff, &plen);
    if (r == 0) { FAIL("accepted <7 bytes"); return; }
    PASS();
}

static void test_parse_arnet_size_overflow(void) {
    TEST("parse ARNetworkAL rejects size>len");
    uint8_t buf[10];
    buf[0]=2; buf[1]=125; buf[2]=0;
    buf[3]=50; buf[4]=0; buf[5]=0; buf[6]=0; /* claims 50 bytes but only 10 */
    uint8_t type, id, seq;
    int poff, plen;
    int r = parse_arnet_frame(buf, 10, &type, &id, &seq, &poff, &plen);
    if (r == 0) { FAIL("accepted oversized"); return; }
    PASS();
}

static void test_build_battery_event(void) {
    TEST("build battery event (Common.BatteryState)");
    uint8_t buf[64];
    int len = build_battery_event(buf, sizeof(buf), 0, 85);
    if (len < 0) { FAIL("build failed"); return; }
    if (buf[0] != 4)  { FAIL("type not DATA_WITH_ACK"); return; }
    if (buf[1] != 126){ FAIL("id not EVENT"); return; }
    if (buf[7] != 0)  { FAIL("project not Common"); return; }
    if (buf[8] != 5)  { FAIL("class wrong"); return; }
    if (buf[9] != 1 || buf[10] != 0) { FAIL("cmd_id wrong"); return; }
    if (buf[11] != 85) { FAIL("battery value wrong"); return; }
    PASS();
}

static void test_build_attitude_event(void) {
    TEST("build attitude event (ARDrone3.PilotingState.Attitude)");
    uint8_t buf[64];
    int len = build_attitude_event(buf, sizeof(buf), 1, 1.5f, -2.3f, 180.0f);
    if (len < 0) { FAIL("build failed"); return; }
    if (buf[7] != 1)  { FAIL("project not ARDrone3"); return; }
    if (buf[8] != 0)  { FAIL("class not Piloting"); return; }
    if (buf[9] != 3 || buf[10] != 0) { FAIL("cmd_id wrong"); return; }
    float roll, pitch, yaw;
    memcpy(&roll, buf+11, 4);
    memcpy(&pitch, buf+15, 4);
    memcpy(&yaw, buf+19, 4);
    if (roll != 1.5f || pitch != -2.3f || yaw != 180.0f) {
        FAIL("attitude values wrong"); return;
    }
    PASS();
}

static void test_build_gps_event(void) {
    TEST("build GPS position event");
    uint8_t buf[128];
    int len = build_gps_position_event(buf, sizeof(buf), 2, 48.8566, 2.3522, 35.0);
    if (len < 0) { FAIL("build failed"); return; }
    if (buf[7] != 1)  { FAIL("project"); return; }
    if (buf[8] != 4)  { FAIL("class not GPSSettings"); return; }
    if (buf[9] != 4 || buf[10] != 0) { FAIL("cmd_id"); return; }
    double lat, lon, alt;
    memcpy(&lat, buf+11, 8);
    memcpy(&lon, buf+19, 8);
    memcpy(&alt, buf+27, 8);
    if (lat < 48.8 || lat > 48.9) { FAIL("lat wrong"); return; }
    if (lon < 2.3 || lon > 2.4)   { FAIL("lon wrong"); return; }
    if (alt < 34.0 || alt > 36.0) { FAIL("alt wrong"); return; }
    PASS();
}

static void test_build_flying_state(void) {
    TEST("build flying state event");
    uint8_t buf[64];
    int len = build_flying_state_event(buf, sizeof(buf), 3, 1);
    if (len < 0) { FAIL("build failed"); return; }
    if (buf[11] != 1) { FAIL("state=1 expected"); return; }
    PASS();
}

static void test_build_wifi_signal(void) {
    TEST("build WiFi signal event");
    uint8_t buf[64];
    int len = build_wifi_signal_event(buf, sizeof(buf), 4, -50);
    if (len < 0) { FAIL("build failed"); return; }
    int16_t rssi;
    memcpy(&rssi, buf+11, 2);
    if (rssi != -50) { FAIL("rssi wrong"); return; }
    PASS();
}

static void test_is_video_ack(void) {
    TEST("is_video_ack detects ACK frames");
    if (!is_video_ack(13, 18)) { FAIL("missed ACK"); return; }
    if (is_video_ack(13, 10))  { FAIL("too short"); return; }
    if (is_video_ack(125, 18)) { FAIL("false positive"); return; }
    PASS();
}

static void test_arnet_frame_roundtrip(void) {
    TEST("ARNetworkAL build+parse roundtrip");
    uint8_t buf[64];
    uint8_t payload[] = {0x01, 0x00, 0x05, 0x00, 0x41, 0x00, 0x00, 0x00};
    int off = build_arnet_header(buf, sizeof(buf), 4, 127, 7, sizeof(payload));
    memcpy(buf+off, payload, sizeof(payload));
    uint8_t type, id, seq;
    int poff, plen;
    int r = parse_arnet_frame(buf, off+sizeof(payload), &type, &id, &seq, &poff, &plen);
    if (r != 0)                     { FAIL("parse"); return; }
    if (type != 4 || id != 127)     { FAIL("type/id"); return; }
    if (plen != (int)sizeof(payload)){ FAIL("payload len"); return; }
    if (memcmp(buf+poff, payload, sizeof(payload)) != 0) { FAIL("payload mismatch"); return; }
    PASS();
}

static void test_buffer_overflow_safety(void) {
    TEST("build functions reject tiny buffers");
    uint8_t tiny[6];
    int r = build_arnet_header(tiny, 6, 2, 125, 0, 10);
    if (r >= 0) { FAIL("arnet accepted tiny buf"); return; }
    r = build_arstream_packet(tiny, 6, 0, 1, 1, 0, 1, tiny, 0);
    if (r >= 0) { FAIL("arstream accepted tiny buf"); return; }
    r = build_event_frame(tiny, 6, 0, 1, 0, 3, NULL, 0);
    if (r >= 0) { FAIL("event accepted tiny buf"); return; }
    PASS();
}

int main(void) {
    printf("\n=== ARSDK3 Protocol Tests ===\n\n");

    test_build_arnet_header();
    test_build_arstream_basic();
    test_build_arstream_multi_fragment();
    test_parse_arnet_frame();
    test_parse_arnet_too_short();
    test_parse_arnet_size_overflow();
    test_build_battery_event();
    test_build_attitude_event();
    test_build_gps_event();
    test_build_flying_state();
    test_build_wifi_signal();
    test_is_video_ack();
    test_arnet_frame_roundtrip();
    test_buffer_overflow_safety();

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
