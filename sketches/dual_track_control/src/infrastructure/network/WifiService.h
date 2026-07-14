// infrastructure/network — Wi-Fi AP service + the serving state shared by
// DashboardServer and SseStream (#117 step 10 final slice, #181). The
// state is external linkage so the test harness can reset and inspect it
// (the escL / sbusRx precedent). Only the .cpp files in this folder touch
// the WiFiS3 hardware objects for serving.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <WiFiS3.h>

// ── serving tunables (moved from [CONFIG] with the machine, values
//    unchanged — single home; the target config/ layer re-homes them later)
const uint32_t SSE_INTERVAL_MS       = 200;   // dashboard push period = 5 Hz
const uint32_t WIFI_MODEM_TIMEOUT_MS = 50;    // per-call modem cap so one stalled
                                              // write can't freeze loop() (#54)
const size_t   SSE_FRAME_CAP         = 448;   // ": hb\ndata: " + JSON + "\n\n"
const size_t   WIFI_PAGE_CHUNK       = 1024;  // dashboard HTML bytes per loop pass

// ── shared serving state (owned here; extern for the harness) ────────────
extern WiFiServer wifiServer;
extern bool       wifiUp;
extern char       pageEtag[24];    // content-hash ETag for the static dashboard
extern const char *dashboardPageHtml;  // caller-owned embedded page

// Server-Sent Events: one persistent connection streams telemetry.
extern WiFiClient sseClient;
extern uint32_t   sseLastMs;
extern bool       sseActive;       // we are holding a live SSE socket (#77)

// Incremental dashboard transfer (#69): pageRemaining > 0 = in flight.
extern WiFiClient pageClient;
extern const char *pagePointer;
extern size_t     pageRemaining;
