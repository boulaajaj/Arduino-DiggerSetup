// ports — the ESC telemetry input contract (#117 step 10 slice 4, #178).
// A link-time seam (#130): free functions, ONE linked implementation
// (infrastructure/xc/XbusTelemetryAdapter.cpp). The CALLER owns the
// EscTelem array — alerts, safety, the dashboard JSON and debug all read
// it application-side; the adapter owns only the bus and its poll state.
#pragma once

#include <stdint.h>

// Telemetry sample from one ESC (via X.BUS Read Register 0x10). Moved
// verbatim from types.h (#178) — the port speaks the type it delivers;
// types.h includes this header so every existing consumer still resolves it.
struct EscTelem {
  float    voltage;     // V (battery / bus voltage, EMA smoothed)
  float    busCurrentA;  // A (bus current, EMA smoothed)
  int16_t  rpmHz;       // electrical Hz (instantaneous; ~×30 = mech RPM)
  float    motorTempC;  // degC (EMA smoothed)
  float    escTempC;    // degC (EMA smoothed)
  uint32_t lastGoodMs;  // millis() of last successful read
  bool     valid;       // fresh reading within the staleness window
};

// Start the telemetry bus.
void telemetrySourceInitialize();

// Poll the bus (non-blocking; call every loop pass). Fills/refreshes the
// caller's telemetry array and runs the per-ESC staleness watchdog.
// Contract: the caller value-initializes the array before the first call
// (e.g. `EscTelem telem[N] = {};`) — `valid` must start false, since the
// first sample of each ESC seeds the EMA instead of folding into it.
void telemetrySourceUpdate(EscTelem* telemetry, uint8_t escCount);
