#include "wifi_config.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>

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

    return settings_validate(s);
}

// ===========================================================================
// Responses
// ===========================================================================
static void send_text(int code, const char* type, const String& body) {
    s_srv->send(code, type, body);
}

static String html_index();

static void handle_index() {
    send_text(200, "text/html", html_index());
}

static void handle_json() {
    const Settings& s = *s_cfg;
    char buf[1024];
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

    if (s.wifi.mode == WIFI_MODE_STA && s.wifi.ssid[0]) {
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
    s_srv->on("/api/settings", HTTP_GET,  handle_json);
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
#msg{margin:10px 2px;font-size:13px;min-height:18px}
small{color:#7b8ea5}
</style></head><header>WING &mdash; configuration</header><main>
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
<option value=2>Spektrum</option></select></div>
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
</div></section>

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
#endif
