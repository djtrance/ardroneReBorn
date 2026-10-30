#include "wifi_config.h"
#include "config.h"
#include "i2c_scan.h"
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Live sample for the portal's live view (GET /api/live, page /attitude).
// The aircraft loop publishes at the log rate; the HTTP handlers read it.
// Both run on the same loop task, so a plain struct copy needs no locking.
// ---------------------------------------------------------------------------
static LogSample s_live;
static float     s_live_mag[3] = {0.0f, 0.0f, 0.0f};
static bool      s_live_mag_ok = false;
static bool      s_live_valid  = false;

void wifi_config_publish(const LogSample& s, const float mag_ut[3],
                         bool mag_ok) {
    s_live = s;
    if (mag_ut) {
        s_live_mag[0] = mag_ut[0];
        s_live_mag[1] = mag_ut[1];
        s_live_mag[2] = mag_ut[2];
    }
    s_live_mag_ok = mag_ok;
    s_live_valid  = true;
}

#ifdef ARDUINO

#include <WiFi.h>
#include <WebServer.h>

static Settings*   s_cfg = nullptr;
static void       (*s_on_change)(Settings&) = nullptr;
static WebServer*  s_srv = nullptr;
static bool        s_active = false;
static bool        s_was_ap = true;
static char        s_ssid[SSID_MAX + 1] = "wing";

bool wifi_config_active() { return s_active; }
const char* wifi_config_ssid() { return s_ssid; }

// ===========================================================================
// URL decoding — the portal posts `text/plain` with encodeURIComponent() on
// the client, so we own the whole decoding step here (no double-decode risk).
// ===========================================================================
static int hexv(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode_in_place(char* s) {
    char* w = s;
    for (char* r = s; *r; ) {
        if (*r == '+')      { *w++ = ' '; r++; }
        else if (*r == '%' && hexv(r[1]) >= 0 && hexv(r[2]) >= 0) {
            *w++ = (char)((hexv(r[1]) << 4) | hexv(r[2]));
            r += 3;
        } else {
            *w++ = *r++;
        }
    }
    *w = 0;
}

// Extract `key` from a `k=v&k=v` body into out[] (decoded). Returns false if absent.
static bool form_get(const char* body, const char* key, char* out, size_t cap) {
    if (!body || !key || !out || cap == 0) return false;
    size_t klen = strlen(key);
    const char* p = body;
    while (*p) {
        const char* eq = strchr(p, '=');
        const char* amp = strchr(p, '&');
        if (!amp) amp = p + strlen(p);
        if (eq && eq < amp && (size_t)(eq - p) == klen &&
            strncmp(p, key, klen) == 0) {
            size_t vlen = (size_t)(amp - eq - 1);
            if (vlen >= cap) vlen = cap - 1;
            memcpy(out, eq + 1, vlen);
            out[vlen] = 0;
            url_decode_in_place(out);
            return true;
        }
        p = (*amp == '&') ? amp + 1 : amp;
    }
    return false;
}

static long f_long(const char* b, const char* k, long def) {
    char v[48];
    if (!form_get(b, k, v, sizeof(v))) return def;
    char* end = nullptr;
    long r = strtol(v, &end, 10);
    return (end == v) ? def : r;
}
static double f_dbl(const char* b, const char* k, double def) {
    char v[48];
    if (!form_get(b, k, v, sizeof(v))) return def;
    char* end = nullptr;
    double r = strtod(v, &end);
    return (end == v) ? def : r;
}
// Checkboxes are simply absent when unchecked.
static bool f_chk(const char* b, const char* k) {
    char v[8];
    return form_get(b, k, v, sizeof(v));
}

// ===========================================================================
// Apply a posted body onto the live Settings, clamp it, persist it.
// ===========================================================================
static int apply_body(const char* b, Settings& s) {
    MixSettings& m = s.mix;
    m.elevon_l_trim_us = (uint16_t)f_long(b, "ml_trim", m.elevon_l_trim_us);
    m.elevon_r_trim_us = (uint16_t)f_long(b, "mr_trim", m.elevon_r_trim_us);
    m.pitch_span_us    = (uint16_t)f_long(b, "p_span",  m.pitch_span_us);
    m.roll_span_us     = (uint16_t)f_long(b, "r_span",  m.roll_span_us);
    m.elevon_l_reverse = f_chk(b, "l_rev") ? 1 : 0;
    m.elevon_r_reverse = f_chk(b, "r_rev") ? 1 : 0;
    m.throttle_reverse = f_chk(b, "t_rev") ? 1 : 0;
    m.passthrough      = f_chk(b, "pt") ? 1 : 0;
    m.differential     = (float)f_dbl(b, "diff",  m.differential);
    m.esc_min_us       = (uint16_t)f_long(b, "esc_min", m.esc_min_us);
    m.esc_max_us       = (uint16_t)f_long(b, "esc_max", m.esc_max_us);
    m.esc_idle_us      = (uint16_t)f_long(b, "esc_idle", m.esc_idle_us);

    RcSettings& rc = s.rc;
    uint8_t old_proto = rc.proto;
    rc.proto    = (uint8_t)f_long(b, "proto", rc.proto);
    rc.ch_pitch = (uint8_t)f_long(b, "ch_p", rc.ch_pitch);
    rc.ch_roll  = (uint8_t)f_long(b, "ch_r", rc.ch_roll);
    rc.ch_throttle = (uint8_t)f_long(b, "ch_t", rc.ch_throttle);
    rc.ch_yaw   = (uint8_t)f_long(b, "ch_y", rc.ch_yaw);
    rc.ch_arm       = (uint8_t)f_long(b, "ch_arm",    rc.ch_arm);
    rc.ch_rth       = (uint8_t)f_long(b, "ch_rth",    rc.ch_rth);
    rc.ch_flightmode= (uint8_t)f_long(b, "ch_fm",     rc.ch_flightmode);
    rc.arm_high_is_armed = f_chk(b, "arm_high") ? 1 : 0;
    rc.sbus_inverted = f_chk(b, "sbus_inv") ? 1 : 0;
    rc.deadband_raw  = (uint16_t)f_long(b, "db", rc.deadband_raw);
    rc.loss_timeout_ms = (uint16_t)f_long(b, "loss_ms", rc.loss_timeout_ms);
    rc.expo            = (float)f_dbl(b, "expo", rc.expo);
    if (rc.proto != old_proto) rc_apply_proto_defaults(rc);

    WifiSettings& w = s.wifi;
    char v[96];
    if (form_get(b, "ssid", v, sizeof(v))) strncpy(w.ssid, v, SSID_MAX);
    if (form_get(b, "pass", v, sizeof(v))) strncpy(w.pass, v, PASS_MAX);
    if (form_get(b, "hostname", v, sizeof(v))) strncpy(w.hostname, v, HOSTNAME_MAX);
    w.mode      = (uint8_t)f_long(b, "wmode", w.mode);
    w.http_port = (uint16_t)f_long(b, "hport", w.http_port);
    w.ssid[SSID_MAX] = 0; w.pass[PASS_MAX] = 0; w.hostname[HOSTNAME_MAX] = 0;

    TelemSettings& t = s.telem;
    t.enable    = f_chk(b, "t_en") ? 1 : 0;
    t.fast_only = f_chk(b, "t_fast") ? 1 : 0;
    t.rate_hz   = (uint16_t)f_long(b, "t_rate", t.rate_hz);

    MavlinkSettings& mv = s.mav;
    mv.enable  = f_chk(b, "mv_en") ? 1 : 0;
    mv.uart    = (uint8_t)f_long(b, "mv_uart", mv.uart);
    mv.baud    = (uint32_t)f_long(b, "mv_baud", (long)mv.baud);
    mv.rate_hz = (uint8_t)f_long(b, "mv_rate", mv.rate_hz);
    mv.sysid   = (uint8_t)f_long(b, "mv_sys",  mv.sysid);
    mv.tx_pin  = (int16_t)f_long(b, "mv_tx",   mv.tx_pin);
    mv.rx_pin  = (int16_t)f_long(b, "mv_rx",   mv.rx_pin);

    return settings_validate(s);
}

// ===========================================================================
// Responses
// ===========================================================================
static void send_text(int code, const char* type, const String& body) {
    s_srv->send(code, type, body);
}

static String html_index();
static String html_attitude();

static void handle_index() {
    send_text(200, "text/html", html_index());
}

static void handle_attitude() {
    send_text(200, "text/html", html_attitude());
}

// GET /api/live — newest published sample (same schema as the CSV logger)
// plus the magnetometer vector, which the CSV does not carry.
static void handle_live() {
    if (!s_live_valid) {
        send_text(503, "application/json", "{\"live\":false}");
        return;
    }
    const LogSample& s = s_live;
    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"t\":%lu,\"mode\":%u,\"armed\":%u,\"rc_ok\":%u,\"phase\":%u,"
        "\"fs_evt\":%u,\"env\":%u,\"wind\":%u,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
        "\"mag\":[%.1f,%.1f,%.1f],\"mag_ok\":%u,"
        "\"baro_pa\":%.1f,\"baro_c\":%.1f,"
        "\"fix\":%u,\"sv\":%u,\"hdop\":%.1f,"
        "\"gps_v\":%.2f,\"gps_alt\":%.1f,"
        "\"rc_p\":%.2f,\"rc_r\":%.2f,\"rc_t\":%.2f,"
        "\"l_us\":%u,\"r_us\":%u,\"thr\":%.2f,"
        "\"vest\":%.1f,\"overruns\":%lu}",
        (unsigned long)s.t_ms, s.mode, s.armed, s.rc_ok, s.phase,
        s.fs_evt, s.env, s.wind,
        (double)s.ax, (double)s.ay, (double)s.az,
        (double)s.gx, (double)s.gy, (double)s.gz,
        (double)s.roll, (double)s.pitch, (double)s.yaw,
        (double)s_live_mag[0], (double)s_live_mag[1], (double)s_live_mag[2],
        s_live_mag_ok ? 1u : 0u,
        (double)s.baro_pa, (double)s.baro_c,
        s.fix, s.sv, (double)s.hdop,
        (double)s.gps_v, (double)s.gps_alt,
        (double)s.rc_p, (double)s.rc_r, (double)s.rc_t,
        (unsigned)s.l_us, (unsigned)s.r_us, (double)s.thr_out,
        (double)s.vest, (unsigned long)s.overruns);
    send_text(200, "application/json", String(buf));
}

// GET /api/i2c — full bus scan with the known parts' ID registers. The scan
// blocks a few tens of ms inside the loop task, so it is refused while armed.
static void handle_i2c() {
    if (s_live_valid && s_live.armed) {
        send_text(409, "application/json", "{\"err\":\"armed\"}");
        return;
    }
    I2cScan sc;
    i2c_scan_run(sc);
    char buf[640];
    if (!i2c_scan_json(sc, buf, sizeof(buf))) {
        send_text(500, "application/json", "{\"err\":\"overflow\"}");
        return;
    }
    send_text(200, "application/json", String(buf));
}

static void handle_json() {
    const Settings& s = *s_cfg;
    char buf[1536];
    snprintf(buf, sizeof(buf),
        "{\"mix\":{\"ml_trim\":%u,\"mr_trim\":%u,\"p_span\":%u,\"r_span\":%u,"
        "\"l_rev\":%u,\"r_rev\":%u,\"t_rev\":%u,\"pt\":%u,\"diff\":%.3f,"
        "\"esc_min\":%u,\"esc_max\":%u,\"esc_idle\":%u},"
        "\"rc\":{\"proto\":%u,\"ch_p\":%u,\"ch_r\":%u,\"ch_t\":%u,\"ch_y\":%u,"
        "\"ch_arm\":%u,\"ch_rth\":%u,\"ch_fm\":%u,\"arm_high\":%u,"
        "\"sbus_inv\":%u,\"db\":%u,\"loss_ms\":%u,\"expo\":%.3f},"
        "\"wifi\":{\"ssid\":\"%s\",\"wmode\":%u,\"hostname\":\"%s\","
        "\"hport\":%u},"
        "\"telem\":{\"enable\":%u,\"fast_only\":%u,\"rate_hz\":%u},"
        "\"mav\":{\"enable\":%u,\"uart\":%u,\"baud\":%lu,\"rate\":%u,"
        "\"sysid\":%u,\"tx\":%d,\"rx\":%d},"
        "\"board\":\"%s\"}",
        s.mix.elevon_l_trim_us, s.mix.elevon_r_trim_us,
        s.mix.pitch_span_us, s.mix.roll_span_us,
        s.mix.elevon_l_reverse, s.mix.elevon_r_reverse, s.mix.throttle_reverse,
        s.mix.passthrough,
        (double)s.mix.differential,
        s.mix.esc_min_us, s.mix.esc_max_us, s.mix.esc_idle_us,
        s.rc.proto, s.rc.ch_pitch, s.rc.ch_roll, s.rc.ch_throttle, s.rc.ch_yaw,
        s.rc.ch_arm, s.rc.ch_rth, s.rc.ch_flightmode,
        s.rc.arm_high_is_armed, s.rc.sbus_inverted,
        s.rc.deadband_raw, s.rc.loss_timeout_ms, (double)s.rc.expo,
        s.wifi.ssid, s.wifi.mode, s.wifi.hostname, s.wifi.http_port,
        s.telem.enable, s.telem.fast_only, s.telem.rate_hz,
        s.mav.enable, s.mav.uart, (unsigned long)s.mav.baud, s.mav.rate_hz,
        s.mav.sysid, s.mav.tx_pin, s.mav.rx_pin,
        IMU_BOARD_NAME);
    send_text(200, "application/json", String(buf));
}

static void handle_save() {
    String body = s_srv->arg("plain");
    if (body.length() == 0) {
        send_text(400, "application/json", "{\"ok\":false,\"err\":\"empty body\"}");
        return;
    }
    int fixed = apply_body(body.c_str(), *s_cfg);
    bool saved = settings_save(*s_cfg);

    if (s_on_change) s_on_change(*s_cfg);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "{\"ok\":%s,\"saved\":%s,\"clamped\":%d,\"version\":%u}",
             saved ? "true" : "false", saved ? "true" : "false",
             fixed, (unsigned)SETTINGS_VERSION);
    send_text(200, "application/json", String(msg));
}

static void handle_factory() {
    settings_defaults(*s_cfg);
    bool saved = settings_save(*s_cfg);
    if (s_on_change) s_on_change(*s_cfg);
    send_text(200, "application/json",
              saved ? "{\"ok\":true}" : "{\"ok\":false}");
}

// ===========================================================================
// WiFi bring-up — STA if credentials are configured and reachable, AP
// otherwise, so the aircraft is never unreachable after a bad credential.
// ===========================================================================
static void wifi_start() {
    Settings& s = *s_cfg;

    if (s.wifi.mode == WIFI_CFG_MODE_STA && s.wifi.ssid[0]) {
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(s.wifi.hostname);
        WiFi.begin(s.wifi.ssid, s.wifi.pass);
        for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) delay(200);
        if (WiFi.status() == WL_CONNECTED) {
            strncpy(s_ssid, s.wifi.ssid, SSID_MAX);
            s_was_ap = false;
            return;
        }
        // fall through: bad credentials must not strand the aircraft
    }

    char ap_ssid[SSID_MAX + 1];
    if (s.wifi.ssid[0]) {
        strncpy(ap_ssid, s.wifi.ssid, SSID_MAX);
    } else {
        uint64_t mac = ESP.getEfuseMac();
        snprintf(ap_ssid, sizeof(ap_ssid), "wing-%04X",
                 (unsigned)(mac & 0xFFFF));
    }
    ap_ssid[SSID_MAX] = 0;
    strncpy(s_ssid, ap_ssid, SSID_MAX);
    s_was_ap = true;

    WiFi.mode(WIFI_AP);
    if (s.wifi.pass[0]) WiFi.softAP(ap_ssid, s.wifi.pass);
    else                WiFi.softAP(ap_ssid);          // open AP, bench use
}

bool wifi_config_begin(Settings& settings, void (*on_change)(Settings&)) {
    s_cfg       = &settings;
    s_on_change = on_change;
    s_active    = false;

    wifi_start();

    uint16_t port = settings.wifi.http_port ? settings.wifi.http_port : 80;
    s_srv = new WebServer(port);
    s_srv->on("/",             HTTP_GET,  handle_index);
    s_srv->on("/attitude",     HTTP_GET,  handle_attitude);
    s_srv->on("/api/settings", HTTP_GET,  handle_json);
    s_srv->on("/api/live",     HTTP_GET,  handle_live);
    s_srv->on("/api/i2c",      HTTP_GET,  handle_i2c);
    s_srv->on("/api/save",     HTTP_POST, handle_save);
    s_srv->on("/api/factory",  HTTP_POST, handle_factory);
    s_srv->onNotFound([]() { send_text(404, "text/plain", "not found"); });
    s_srv->begin();

    s_active = true;
    return true;
}

void wifi_config_loop() {
    if (s_srv) s_srv->handleClient();
}

#else  // ---------------------------------------------------------------------
// Host build: the portal is ESP32-only, keep the symbols linkable.
bool wifi_config_begin(Settings&, void (*)(Settings&)) { return false; }
void wifi_config_loop() {}
bool wifi_config_active() { return false; }
const char* wifi_config_ssid() { return ""; }
#endif

// ===========================================================================
// The page. Kept as one raw literal; the JS posts encodeURIComponent()'d
// pairs as text/plain, which the server decodes.
// ===========================================================================
#ifdef ARDUINO
static String html_index() {
    return String(
R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Wing config</title><style>
body{font:15px system-ui,sans-serif;margin:0;background:#12161c;color:#dfe6ee}
header{background:#1b2531;padding:14px 18px;font-weight:700;letter-spacing:.5px}
main{padding:14px;max-width:640px;margin:auto}
section{background:#1b2531;border-radius:10px;padding:14px 16px;margin-bottom:14px}
h2{margin:0 0 10px;font-size:14px;text-transform:uppercase;color:#7fd1ff}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
label{display:block;font-size:12px;color:#9fb0c3;margin-bottom:3px}
input,select{width:100%;padding:7px;border-radius:6px;border:1px solid #33445c;
background:#0e131a;color:#dfe6ee;box-sizing:border-box}
.chk{display:flex;align-items:center;gap:8px;font-size:13px;padding-top:16px}
.chk input{width:auto}
button{width:100%;padding:12px;border:0;border-radius:8px;background:#2f9e44;
color:#fff;font-size:16px;font-weight:700;margin-top:4px}
button.alt{background:#37506b}
a.btn{display:block;padding:12px;border-radius:8px;background:#37506b;
color:#fff;font-size:16px;font-weight:700;text-decoration:none;
text-align:center;margin-bottom:14px}
#msg{margin:10px 2px;font-size:13px;min-height:18px}
small{color:#7b8ea5}
.help{display:block;color:#9fb0c3;line-height:1.5;margin-top:8px}
.help b{color:#dfe6ee}.help code{color:#7fd1ff}
</style></head><header>WING &mdash; configuration</header><main>
<a class=btn href=/attitude>&#9673; VISOR DE ACTITUD (IMU) &rarr;</a>
<form id=f>
<section><h2>Mezcla / servos (D1-D3)</h2><div class=grid>
<div><label>Trim elevon L (us)</label><input name=ml_trim type=number min=1000 max=2000></div>
<div><label>Trim elevon R (us)</label><input name=mr_trim type=number min=1000 max=2000></div>
<div><label>Span pitch (us)</label><input name=p_span type=number min=50 max=900></div>
<div><label>Span roll (us)</label><input name=r_span type=number min=50 max=900></div>
<div><label>Diferencial 0-0.4</label><input name=diff type=number step=0.01 min=0 max=0.4></div>
<div><label>Expo 0-1</label><input name=expo type=number step=0.05 min=0 max=1></div>
<div class=chk><input name=l_rev id=l_rev type=checkbox><label for=l_rev>Invertir elevon L</label></div>
<div class=chk><input name=r_rev id=r_rev type=checkbox><label for=r_rev>Invertir elevon R</label></div>
<div class=chk><input name=t_rev id=t_rev type=checkbox><label for=t_rev>Invertir acelerador</label></div>
<div class=chk><input name=pt id=pt type=checkbox><label for=pt>Passthrough etapa 1 (RX &rarr; mezcla directa)</label></div>
</div></section>

<section><h2>RC entrada (H2)</h2><div class=grid>
<div><label>Protocolo</label><select name=proto>
<option value=0>NINGUNO</option><option value=1>SBUS</option>
<option value=2>Spektrum</option><option value=3>PPM-SUM (GPIO33)</option></select></div>
<div><label>Ch pitch</label><input name=ch_p type=number min=0 max=15></div>
<div><label>Ch roll</label><input name=ch_r type=number min=0 max=15></div>
<div><label>Ch throttle</label><input name=ch_t type=number min=0 max=15></div>
<div><label>Ch yaw</label><input name=ch_y type=number min=0 max=15></div>
<div><label>Ch arm (255=none)</label><input name=ch_arm type=number min=0 max=255></div>
<div><label>Ch RTH</label><input name=ch_rth type=number min=0 max=255></div>
<div><label>Ch modo de vuelo</label><input name=ch_fm type=number min=0 max=255></div>
<div><label>Deadband (raw)</label><input name=db type=number min=0 max=100></div>
<div><label>Timeout RC (ms)</label><input name=loss_ms type=number min=100 max=5000></div>
<div class=chk><input name=arm_high id=arm_high type=checkbox>
<label for=arm_high>ARM en alto</label></div>
<div class=chk><input name=sbus_inv id=sbus_inv type=checkbox>
<label for=sbus_inv>SBUS invertido</label></div>
</div><small class=help><b>PPM-SUM — cableado:</b> solo si el receptor tiene salida
PPM-SUM (no PWM individual): <code>SUM/PPM → GPIO33</code>, <code>GND → GND</code>,
VCC del receptor según su manual (normalmente BEC 5 V). La señal GPIO33 no debe
superar 3.3 V; si entrega 5 V, usa level shifter/divisor. Selecciona PPM-SUM aquí.
El decoder admite 5–16 canales; el mapa default usa CH0–CH6, así que para un RX de
5 canales pon Ch RTH y Ch modo de vuelo en <code>255</code>. PPM no lleva flag de
failsafe: configura el RX para ARM desactivado + throttle mínimo si pierde señal;
si sigue emitiendo pulsos no se detecta la pérdida. Prueba con hélices quitadas.</small></section>

<section><h2>WiFi</h2><div class=grid>
<div><label>SSID</label><input name=ssid maxlength=32></div>
<div><label>Contraseña</label><input name=pass type=password maxlength=64></div>
<div><label>Hostname</label><input name=hostname maxlength=23></div>
<div><label>Puerto</label><input name=hport type=number min=1 max=65535></div>
<div><label>Modo</label><select name=wmode>
<option value=0>AP siempre</option><option value=1>Estación, si no AP</option>
</select></div>
</div></section>

<section><h2>Telemetría al mando (I1)</h2><div class=grid>
<div><label>Rate S.Port (Hz)</label><input name=t_rate type=number min=1 max=200></div>
<div class=chk><input name=t_en id=t_en type=checkbox><label for=t_en>Activar S.Port</label></div>
<div class=chk><input name=t_fast id=t_fast type=checkbox><label for=t_fast>Solo campos rapidos</label></div>
</div><small>Solo funciona con receptor FrSky (S.Port/FPort) &mdash; ver
docs/rc-and-telemetry.md</small></section>

<section><h2>Telemetría MAVLink (I1)</h2><div class=grid>
<div><label>Puerto UART</label><select name=mv_uart>
<option value=0>0 &mdash; USB (CSV serial a UDP)</option>
<option value=1>1 &mdash; GPS (roba GPS)</option>
<option value=2>2 &mdash; RC UART (roba SBUS/Spektrum)</option></select></div>
<div><label>Baudios</label><select name=mv_baud>
<option value=9600>9600</option><option value=57600>57600</option>
<option value=115200>115200</option><option value=230400>230400</option>
<option value=460800>460800</option><option value=921600>921600</option>
</select></div>
<div><label>Rate rapido (Hz)</label><input name=mv_rate type=number min=1 max=25></div>
<div><label>System ID</label><input name=mv_sys type=number min=1 max=255></div>
<div><label>TX pin (-1=def)</label><input name=mv_tx type=number min=-1 max=39></div>
<div><label>RX pin (-1=def)</label><input name=mv_rx type=number min=-1 max=39></div>
<div class=chk><input name=mv_en id=mv_en type=checkbox><label for=mv_en>Activar MAVLink</label></div>
</div><small class=help><b>MAVLink UART2 + RC por PPM:</b> elige Puerto UART=2 y conecta
<code>GPIO23 TX → RX</code> de la radio/GCS, <code>GPIO18 RX ← TX</code> de la radio/GCS,
con GND común. Ajusta TX pin=<code>23</code>, RX pin=<code>18</code>; el receptor PPM
permanece en GPIO33. No uses UART2 así con SBUS/Spektrum: ese RC quedaría desconectado.
UART 1/2 exigen TX+RX pin; UART1 desactiva GPS. Con MAVLink en UART0 el CSV
serial se apaga (usa capture.py --udp). Señal serie ESP32 = 3.3 V. Ver
docs/mavlink-telemetry.md</small></section>

<button type=submit>GUARDAR</button>
<div id=msg></div>
</form>
<button class=alt type=button onclick="if(confirm('Restablecer?'))
fetch('/api/factory',{method:'POST'}).then(()=>location.reload())">
RESTABLECER FABRICA</button>
</main><script>
const f=document.getElementById('f'),msg=document.getElementById('msg');
async function fill(){const r=await (await fetch('/api/settings')).json();
 const set=(n,v)=>{const e=f[n];if(!e)return;
   if(e.type==='checkbox')e.checked=!!v;else e.value=v;};
 set('ml_trim',r.mix.ml_trim);set('mr_trim',r.mix.mr_trim);
 set('p_span',r.mix.p_span);set('r_span',r.mix.r_span);
 set('l_rev',r.mix.l_rev);set('r_rev',r.mix.r_rev);set('t_rev',r.mix.t_rev);
 set('pt',r.mix.pt);
 set('diff',r.mix.diff);set('expo',r.rc.expo);
 set('proto',r.rc.proto);set('ch_p',r.rc.ch_p);set('ch_r',r.rc.ch_r);
 set('ch_t',r.rc.ch_t);set('ch_y',r.rc.ch_y);set('ch_arm',r.rc.ch_arm);
 set('ch_rth',r.rc.ch_rth);set('ch_fm',r.rc.ch_fm);
 set('arm_high',r.rc.arm_high);set('sbus_inv',r.rc.sbus_inv);
 set('db',r.rc.db);set('loss_ms',r.rc.loss_ms);
 set('ssid',r.wifi.ssid);set('hostname',r.wifi.hostname);
 set('hport',r.wifi.hport);set('wmode',r.wifi.wmode);
 set('t_en',r.telem.enable);set('t_fast',r.telem.fast_only);
 set('t_rate',r.telem.rate_hz);
 set('mv_en',r.mav.enable);set('mv_uart',r.mav.uart);set('mv_baud',r.mav.baud);
 set('mv_rate',r.mav.rate);set('mv_sys',r.mav.sysid);
 set('mv_tx',r.mav.tx);set('mv_rx',r.mav.rx);
 msg.textContent='Placa: '+r.board;}
f.onsubmit=async e=>{e.preventDefault();
 const p=new URLSearchParams(new FormData(f));
 const r=await(await fetch('/api/save',{method:'POST',
   headers:{'Content-Type':'text/plain'},
   body:p.toString()})).json();
 msg.textContent=r.ok?('Guardado'+(r.clamped?' ('+r.clamped+' valores corregidos)':''))
  :'ERROR al guardar';};
fill();
</script></html>)HTML");
}

// ===========================================================================
// Attitude / raw-IMU viewer (page /attitude): a canvas artificial horizon fed
// by GET /api/live, plus the numbers behind it — what the IMU actually sees.
// ===========================================================================
static String html_attitude() {
    return String(
R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Wing &mdash; actitud</title><style>
body{font:14px system-ui,sans-serif;margin:0;background:#0b0f14;color:#dfe6ee}
header{background:#1b2531;padding:12px 16px;display:flex;
justify-content:space-between;align-items:center;font-weight:700}
header a{color:#7fd1ff;text-decoration:none}
main{padding:12px;max-width:520px;margin:0 auto;text-align:center}
canvas{background:#101418;border-radius:12px;max-width:100%}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}
.c{background:#1b2531;border-radius:8px;padding:8px 4px}
.c span{display:block;font-size:10px;color:#9fb0c3;letter-spacing:.5px}
.c b{font-size:14px;font-weight:600}
#st{margin-top:10px;font-size:12px}
.chip{display:inline-block;padding:2px 8px;border-radius:10px;margin:2px;
font-size:11px;background:#8b3a3a}
.chip.on{background:#2f9e44}
.chip.neu{background:#37506b}
</style></head>
<header><a href=/>&larr; Config</a><span>ACTITUD / IMU</span>
<span id=age style="font-size:11px;color:#9fb0c3">—</span></header>
<main>
<canvas id=ah width=320 height=320></canvas>
<div class=grid>
<div class=c><span>ROLL °</span><b id=roll>—</b></div>
<div class=c><span>PITCH °</span><b id=pitch>—</b></div>
<div class=c><span>YAW °</span><b id=yaw>—</b></div>
<div class=c><span>GYRO rad/s</span><b id=gyro>—</b></div>
<div class=c><span>ACCEL g</span><b id=acc>—</b></div>
<div class=c><span>MAG µT</span><b id=mag>—</b></div>
<div class=c><span>BARO</span><b id=baro>—</b></div>
<div class=c><span>GPS</span><b id=gps>—</b></div>
<div class=c><span>SUPERF µs</span><b id=surf>—</b></div>
</div>
<div id=st>conectando…</div>
</main><script>
const cv=document.getElementById('ah'),ctx=cv.getContext('2d');
const R2D=180/Math.PI;
function ah(r,p){                       // roll, pitch in DEGREES
 const W=cv.width,H=cv.height,mx=W/2,my=H/2,R=W/2,ppd=3.2;
 ctx.save();ctx.beginPath();ctx.rect(0,0,W,H);ctx.clip();
 ctx.translate(mx,my);ctx.rotate(-r/R2D);   // right bank -> horizon tilts
 const dy=p*ppd;                            // nose up -> horizon drops
 ctx.fillStyle='#3f7fcb';ctx.fillRect(-W*2,-H*2+dy,W*4,H*2);
 ctx.fillStyle='#7a5230';ctx.fillRect(-W*2,dy,W*4,H*2);
 ctx.strokeStyle='#fff';ctx.lineWidth=2;
 ctx.beginPath();ctx.moveTo(-W*2,dy);ctx.lineTo(W*2,dy);ctx.stroke();
 ctx.font='11px sans-serif';ctx.textAlign='center';ctx.fillStyle='#fff';
 for(let d=-40;d<=40;d+=10){
  if(d===0)continue;
  const yy=dy-d*ppd,hw=(d%20===0)?36:20;
  ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(-hw,yy);ctx.lineTo(hw,yy);
  ctx.stroke();
  ctx.fillText(Math.abs(d),hw+15,yy+4);
  ctx.fillText(Math.abs(d),-hw-15,yy+4);
 }
 ctx.restore();
 ctx.save();ctx.translate(mx,my);
 // fixed bank scale
 ctx.strokeStyle='#9fb0c3';ctx.lineWidth=2;
 [-60,-45,-30,-20,-10,0,10,20,30,45,60].forEach(a=>{
  const rad=(a-90)/R2D,l=(a%30===0)?13:8;
  ctx.beginPath();
  ctx.moveTo(Math.cos(rad)*(R-3),Math.sin(rad)*(R-3));
  ctx.lineTo(Math.cos(rad)*(R-3-l),Math.sin(rad)*(R-3-l));
  ctx.stroke();
 });
 // bank pointer: right bank -> swings right (clockwise)
 ctx.rotate(r/R2D);ctx.fillStyle='#ffd433';
 ctx.beginPath();ctx.moveTo(0,-R+18);ctx.lineTo(-7,-R+5);
 ctx.lineTo(7,-R+5);ctx.closePath();ctx.fill();
 ctx.restore();
 // fixed aircraft symbol + rim
 ctx.save();ctx.translate(mx,my);
 ctx.strokeStyle='#ffd433';ctx.lineWidth=3;ctx.lineCap='round';
 ctx.beginPath();ctx.moveTo(-70,0);ctx.lineTo(-22,0);ctx.lineTo(-13,9);
 ctx.moveTo(70,0);ctx.lineTo(22,0);ctx.lineTo(13,9);ctx.stroke();
 ctx.beginPath();ctx.arc(0,0,3.5,0,7);ctx.fillStyle='#ffd433';ctx.fill();
 ctx.strokeStyle='#33445c';ctx.lineWidth=4;
 ctx.beginPath();ctx.arc(0,0,R-2,0,7);ctx.stroke();
 ctx.restore();
}
function chip(n,v,neu){return '<span class="chip '+
 (neu?'neu':(v?'on':'off'))+'">'+n+'</span>'}
let last=0;
async function tick(){
 try{
  const d=await (await fetch('/api/live')).json();
  if(d.live===false){st.textContent='esperando primera muestra…';return;}
  ah(d.roll,d.pitch);
  roll.textContent=d.roll.toFixed(1);
  pitch.textContent=d.pitch.toFixed(1);
  yaw.textContent=d.yaw.toFixed(1);
  gyro.textContent=d.gx.toFixed(2)+' '+d.gy.toFixed(2)+' '+d.gz.toFixed(2);
  acc.textContent=d.ax.toFixed(2)+' '+d.ay.toFixed(2)+' '+d.az.toFixed(2);
  mag.textContent=d.mag_ok
   ? d.mag.map(v=>v.toFixed(0)).join(' ') : 'no ok';
  baro.textContent=(d.baro_pa/100).toFixed(1)+' hPa '+
   d.baro_c.toFixed(0)+'°';
  gps.textContent=d.fix
   ? d.sv+'sv '+d.gps_v.toFixed(1)+' m/s' : 'sin fix';
  surf.textContent=d.l_us+' '+d.r_us;
  last=Date.now();
  st.innerHTML=chip('ARMED',d.armed)+chip('RC',d.rc_ok)+
   chip('PT',d.mode,true)+chip('MAG',d.mag_ok)+chip('FIX',d.fix)+
   chip('PH'+d.phase,d.phase!==0,true)+
   '<span class="chip neu">fs '+d.fs_evt+'</span>'+
   '<span class="chip neu">ovr '+d.overruns+'</span>';
  age.textContent='t='+d.t+' ms';
 }catch(e){
  st.innerHTML='<span class=chip>sin conexion</span>';age.textContent='—';
 }
}
setInterval(tick,100);tick();
</script></html>)HTML");
}
#endif
