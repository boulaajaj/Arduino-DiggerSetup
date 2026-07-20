// application — observer implementations (#189), verbatim from the .ino.
// This is the ONE translation unit that owns the embedded dashboard page
// (web_page.h) and the Wi-Fi credentials (arduino_secrets.h, gitignored).
#include "Monitoring.h"

#include <stdio.h>

#include "../../types.h"
#include "../../web_page.h"
#include "AlertControl.h"
#include "FirmwareState.h"
#include "MotorOutput.h"
#include "OperatorInput.h"
#include "../config/AlertConfig.h"
#include "../config/BuildInfo.h"
#include "../config/InputConfig.h"
#include "../config/DriveConfig.h"
#include "../config/TelemetryConfig.h"
#include "../config/WifiConfig.h"
#include "../telemetry/JsonEncoder.h"
#include "../telemetry/CsvEncoder.h"
#include "../ports/ClockPort.h"
#include "../ports/DashboardServicePort.h"
#include "../ports/DebugConsolePort.h"
#include "../ports/TelemetryFrameSource.h"

// ═══════════════════════════════════════════════════════════════
// [WIFI] — AP + HTTP telemetry server (WiFiS3, stock UNO R4 WiFi)
// ═══════════════════════════════════════════════════════════════
// Hosts a Wi-Fi access point and streams telemetry to the dashboard
// (dashboard/index.html). MONITORING ONLY — no control inputs are ever
// accepted over Wi-Fi (safety).
//
// SAFETY (#69): serving must NEVER starve the control loop. Originally the
// ~33 KB page was sent as one blocking burst (~1-2 s), during which loop()
// froze while the Servo PWM hardware kept emitting the last throttle → a
// runaway. Two defenses now:
//   1. wifiUpdate() does AT MOST ONE modem write per loop pass (one page
//      chunk OR one request OR one SSE frame), so control + failsafe run
//      between every chunk and the loop is never blocked for long.
//   2. The hardware watchdog (WDT_TIMEOUT_MS, armed in setup) resets the MCU
//      if the loop is ever stalled past the timeout regardless of cause —
//      PWM stops and the ESCs go to neutral. Backstop, not the primary fix.

// Credentials come from arduino_secrets.h (gitignored, #125) — copy
// arduino_secrets.h.example in this folder and fill in real values.
#include "../../arduino_secrets.h"
const char WIFI_SSID[] = SECRET_WIFI_SSID;
const char WIFI_PASS[] = SECRET_WIFI_PASS;   // WPA2 needs >= 8 chars
// Wi-Fi tuning constants (AP channel, SSE rate, modem timeout, frame cap) live
// in [CONFIG] per the project's tunable-constants rule.
uint32_t wifiSeq = 0;

// The serving machine (AP bring-up + FNV-1a ETag, HTTP routing, incremental
// page transfer, SSE stream) lives in src/infrastructure/network/ (#181)
// behind ports/DashboardServicePort.h; the JSON body flows the other way
// through ports/TelemetryFrameSource.h. Serving state is adapter-owned.
extern bool wifiUp;  // read by wifiDebug() below

// Override switch → dashboard mode (0=RC, 1=joy/auto-middle, 2=blend).
int wifiMode() {
  if (!sbusValid) return 0;
  int ovr = rcOverride();
  if (ovr < OVR_LO) return 0;
  if (ovr > OVR_HI) return 2;
  return 1;
}

void wifiInit() {
  // AP bring-up (incl. banners + ETag hash) lives in the adapter; the
  // ready-beep stays application-side (alert policy, not networking).
  if (dashboardServiceInitialize(WIFI_SSID, WIFI_PASS, WIFI_AP_CHANNEL, INDEX_HTML)) {
    beepStart(BEEP_WIFI_READY, BEEP_WIFI_READY_LEN);   // "beep beep" — Wi-Fi AP is up/ready
  }
}

// Periodic Wi-Fi status line (every ~3 s) for bench diagnostics.
uint32_t wifiDbgPrev = 0;
void wifiDebug(uint32_t nowUs) {
  if (!debugConsoleReady() || (nowUs - wifiDbgPrev) < 3000000UL) return;
  wifiDbgPrev = nowUs;
  // Lines assembled with snprintf and printed whole (#187); the byte stream
  // matches the old Serial print-chains exactly (bool printed 1/0, decimal
  // status/counters, two-digit uppercase hex + trailing space per byte).
  char line[96];
  snprintf(line, sizeof(line), "# WIFI up=%d status=%d clients_seq=%lu",
           wifiUp ? 1 : 0, dashboardServiceRadioStatus(), (unsigned long)wifiSeq);
  debugConsolePrintLine(line);
}

// Observe the whole system into one immutable SystemSnapshot (#132). Called
// by the observer shims at their original observation points for exact
// timing parity; the once-per-cycle build moves into FirmwareApp (step 11).
SystemSnapshot buildSystemSnapshot(uint32_t nowMs) {
  SystemSnapshot snapshot = {};
  snapshot.nowMs = nowMs;
  snapshot.rcThrottleUs = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_THR])   : SVC;
  snapshot.rcSteeringUs = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_STEER]) : SVC;
  snapshot.rcGearUs     = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_GEAR])  : SVC;
  snapshot.rcOverrideUs = sbusValid ? sbusToServo(rcFrame.channels[SBUS_CH_OVR])   : SVMIN;
  snapshot.rcFailsafe = rcFrame.failsafe;
  snapshot.rcLostFrame = rcFrame.lostFrame;
  snapshot.joystickRawY = cachedJoy.rawY;
  snapshot.joystickRawX = cachedJoy.rawX;
  snapshot.outLeftUs = outL;
  snapshot.outRightUs = outR;
  snapshot.gear = (int)currentGear;
  snapshot.overrideMode = wifiMode();
  snapshot.batteryCutoffLatched = batteryCutoffLatched;
  snapshot.ecoLockLatched = ecoLockLatched;
  snapshot.temperatureWarnActive = tempWarnActive;
  snapshot.temperatureEcoActive = tempEcoActive;
  snapshot.temperatureCutActive = tempCutActive;
  snapshot.esc[0] = telem[0];
  snapshot.esc[1] = telem[1];
  return snapshot;
}

// Build the telemetry JSON into body; returns its length. Shared by the
// one-shot /data endpoint and the SSE stream. Formatting lives in
// telemetry/JsonEncoder (#132); this shim owns the frame counter and the
// observation point.
int buildTelemJson(char *body, size_t cap) {
  wifiSeq++;
  return encodeTelemetryJson(body, cap, buildSystemSnapshot(clockNowMs()), wifiSeq);
}

// The network layer pulls frames through ports/TelemetryFrameSource.h —
// the application encodes, infrastructure ships bytes (#181).
int telemetryFrameBuild(char *body, size_t capacity) {
  return buildTelemJson(body, capacity);
}

// Non-blocking, and bounded to AT MOST ONE modem write per loop pass (#69).
// The serving machine (page chunks, request routing, SSE) lives in
// infrastructure/network/DashboardServer.cpp (#181).
void wifiUpdate() { dashboardServiceUpdate(); }


// ═══════════════════════════════════════════════════════════════
// [DEBUG] — 10 Hz serial CSV (control + telemetry)
// ═══════════════════════════════════════════════════════════════
// Columns: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,
//          V0dV,I0dA,RPM0,TE0,TM0,OK0, V1dV,I1dA,RPM1,TE1,TM1,OK1
// Telemetry columns are integer-scaled: voltage in 0.1 V (dV), current in
// 0.1 A (dA), RPM in electrical Hz, temps in °C, OK = fresh-telemetry flag.

uint32_t prevPrint = 0;

void debugInit() {
  debugConsoleBegin(115200);
  if (debugConsoleReady()) {
    // Version + short tag only (#124) — the changelog lives in git history
    // and docs/FIRMWARE-UPLOAD-LOG.md, not in a string constant. Assembled
    // into one line; the byte stream matches the old print-chain exactly.
    char banner[96];
    snprintf(banner, sizeof(banner),
             "# === Digger %s — dual-input track control, GL10 FOC ===",
             FIRMWARE_VERSION);
    debugConsolePrintLine(banner);
    debugConsolePrintLine("# CSV: RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost,V0dV,I0dA,RPM0,TE0,TM0,OK0,V1dV,I1dA,RPM1,TE1,TM1,OK1");
  }
}

void debugPrint(uint32_t now) {
  if (!debugConsoleReady() || (now - prevPrint) < PRINT_INTERVAL) return;
  prevPrint = now;

  // Formatting (integer-scaled — no float printf on this core) lives in
  // telemetry/CsvEncoder (#132); this shim owns the cadence and the print.
  // NOTE: `now` is µs (loop clock); the snapshot contract is ms, so read
  // clockNowMs() here — the CSV does not use nowMs, so output is unchanged.
  char buf[200];
  encodeDebugCsv(buf, sizeof(buf), buildSystemSnapshot(clockNowMs()));
  debugConsolePrintLine(buf);
}


