// wifi_config.h — on-aircraft configuration portal over WiFi (checklist I5)
//
// Lets a pilot change, without reflashing:
//   * servo direction (elevon L/R reverse) and trims      (D1/D3)
//   * mixing spans, adverse-yaw differential, expo        (D2)
//   * RC protocol, channel map, arm switch polarity       (H2)
//   * WiFi credentials, telemetry enable                  (I1)
//
// Every change is written straight into the live `Settings` object (so it
// takes effect on the next control tick) and then persisted to NVS, which is
// what makes it survive a power cycle.
//
// Safety: the portal never touches the arm state. Arming still requires the
// physical RC arm switch plus a clean preflight (H1/H6).

#pragma once
#include "settings.h"

// Start AP/STA and the HTTP server. `settings` is the live object the portal
// edits. `on_change` (may be null) is invoked after a successful save so the
// caller can re-apply anything cached outside Settings.
bool wifi_config_begin(Settings& settings, void (*on_change)(Settings&));

// Call from loop() to service HTTP clients.
void wifi_config_loop();

// True once the portal is serving.
bool wifi_config_active();

// SSID actually being advertised / joined, for the boot banner.
const char* wifi_config_ssid();
