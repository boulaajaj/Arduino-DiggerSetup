// ports — the dashboard network service contract (#181). A link-time seam
// (#130): free functions, ONE linked implementation
// (infrastructure/network/ — WifiService + DashboardServer + SseStream).
// MONITORING ONLY: the service streams telemetry out; no control input is
// ever accepted over Wi-Fi (permanent safety rule).
#pragma once

#include <stdint.h>

// Bring up the Wi-Fi AP + HTTP server. Returns true when the AP is
// listening — the caller owns the ready-beep. `pageHtml` is the embedded
// dashboard (the caller owns web_page.h); credentials stay caller-owned
// (arduino_secrets.h never leaves the sketch).
bool dashboardServiceInitialize(const char* ssid, const char* password,
                                uint8_t apChannel, const char* pageHtml);

// Serve at most ONE modem WRITE per call — one page chunk OR one
// request/response OR one SSE frame (the #69 non-starvation contract; the
// bounded request read may perform multiple socket reads first). Call
// every loop pass; a no-op until initialize succeeded.
void dashboardServiceUpdate();

// The radio's raw status code (WiFi.status()) — bench diagnostics only.
uint8_t dashboardServiceRadioStatus();
