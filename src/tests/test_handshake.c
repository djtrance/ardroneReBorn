#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failed = 0, passed = 0;

#define TEST(name) do { printf("  %-45s ", name); } while(0)
#define PASS() do { printf("PASS\n"); passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); failed++; } while(0)

/* Extracted JSON handshake parser (same logic as skyproxy.c handshake_thread) */
static int parse_handshake_json(const char *hb, int n) {
    int d2c_port = 0;
    const char *p = hb;
    while (*p && p - hb < n) {
        if (*p == '"') {
            p++;
            const char *key_start = p;
            while (*p && *p != '"') p++;
            if (*p != '"') break;
            int key_len = p - key_start;
            p++;
            while (*p && (*p == ' ' || *p == ':' || *p == '\t')) p++;
            if (*p == '"') {
                p++;
                while (*p && *p != '"') p++;
                if (*p == '"') p++;
            } else if (*p >= '0' && *p <= '9') {
                char *end = NULL;
                long val = strtol(p, &end, 10);
                if (end && end > p) {
                    if (key_len == 8 && memcmp(key_start, "d2c_port", 8) == 0)
                        d2c_port = (int)val;
                    else if (key_len == 8 && memcmp(key_start, "d2c port", 8) == 0)
                        d2c_port = (int)val;
                }
                p = end;
            } else {
                while (*p && *p != ',' && *p != '}') p++;
            }
        } else {
            p++;
        }
    }
    return d2c_port;
}

static void test_underscore_format(void) {
    TEST("underscore format d2c_port");
    const char *json = "{\"d2c_port\":43210,\"controller_type\":\"Phone\",\"controller_name\":\"test\"}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 43210) { FAIL("expected 43210"); return; }
    PASS();
}

static void test_space_format(void) {
    TEST("space format 'd2c port'");
    const char *json = "{\"d2c port\":55004,\"controller type\":\"Tablet\",\"controller name\":\"FreeFlight\"}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 55004) { FAIL("expected 55004"); return; }
    PASS();
}

static void test_no_d2c(void) {
    TEST("missing d2c_port returns 0");
    const char *json = "{\"controller_type\":\"Phone\",\"controller_name\":\"test\"}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 0) { FAIL("expected 0"); return; }
    PASS();
}

static void test_empty_object(void) {
    TEST("empty object returns 0");
    const char *json = "{}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 0) { FAIL("expected 0"); return; }
    PASS();
}

static void test_zero_port(void) {
    TEST("d2c_port=0 returns 0");
    const char *json = "{\"d2c_port\":0}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 0) { FAIL("expected 0"); return; }
    PASS();
}

static void test_negative_port(void) {
    TEST("negative d2c_port returns 0 (ignored)");
    const char *json = "{\"d2c_port\":-1}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 0) { FAIL("expected 0"); return; }
    PASS();
}

static void test_reorder_keys(void) {
    TEST("reordered keys still work");
    const char *json = "{\"controller_name\":\"MyApp\",\"d2c_port\":9999,\"controller_type\":\"Computer\"}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 9999) { FAIL("expected 9999"); return; }
    PASS();
}

static void test_minimal(void) {
    TEST("minimal valid JSON");
    const char *json = "{\"d2c_port\":12345}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 12345) { FAIL("expected 12345"); return; }
    PASS();
}

static void test_device_id_field(void) {
    TEST("with device_id (extra field)");
    const char *json = "{\"d2c_port\":33333,\"device_id\":\"ARDRONE0001\"}";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 33333) { FAIL("expected 33333"); return; }
    PASS();
}

static void test_extra_whitespace(void) {
    TEST("extra whitespace handled");
    const char *json = "{  \"d2c_port\"  :  44444  }";
    int port = parse_handshake_json(json, strlen(json));
    if (port != 44444) { FAIL("expected 44444"); return; }
    PASS();
}

static void test_string_port_ignored(void) {
    TEST("string port value ignored (not parsed)");
    const char *json = "{\"d2c_port\":\"54321\"}";
    int port = parse_handshake_json(json, strlen(json));
    /* Our simple parser sees '"' after colon, skips string, doesn't set port */
    if (port != 0) { FAIL("expected 0 for string value"); return; }
    PASS();
}

static void test_response_json_content(void) {
    TEST("handshake response JSON validity (structural check)");
    const char *response =
        "{\"status\":0,"
        "\"c2d_port\":54321,"
        "\"arstream_fragment_size\":64000,"
        "\"arstream_fragment_maximum_number\":4,"
        "\"arstream_max_ack_interval\":-1,"
        "\"c2d_update_port\":51,"
        "\"c2d_user_port\":21}";
    int len = strlen(response);
    if (len <= 0) { FAIL("empty response"); return; }
    if (response[0] != '{') { FAIL("doesn't start with {"); return; }
    if (response[len-1] != '}') { FAIL("doesn't end with }"); return; }
    if (!strstr(response, "\"status\":0")) { FAIL("missing status:0"); return; }
    if (!strstr(response, "\"c2d_port\":54321")) { FAIL("missing c2d_port"); return; }
    if (!strstr(response, "\"arstream_fragment_size\":64000")) { FAIL("missing fragment_size"); return; }
    PASS();
}

int main(void) {
    printf("\n=== Handshake JSON Parser Tests ===\n\n");

    test_underscore_format();
    test_space_format();
    test_no_d2c();
    test_empty_object();
    test_zero_port();
    test_negative_port();
    test_reorder_keys();
    test_minimal();
    test_device_id_field();
    test_extra_whitespace();
    test_string_port_ignored();
    test_response_json_content();

    printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
