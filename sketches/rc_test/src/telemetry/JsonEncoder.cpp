// telemetry — the ONE implementation of the dashboard JSON frame (#132).
#include "JsonEncoder.h"

#include <stdint.h>
#include <stdio.h>

#include "../application/SystemSnapshot.h"
#include "TelemetryScaling.h"

int encodeTelemetryJson(char *body, size_t capacity,
                        const SystemSnapshot &snapshot, unsigned long sequence) {
  // Per-ESC age (ms since last good frame); 999999 if never received.
  uint32_t age0 = snapshot.esc[0].lastGoodMs
                      ? (snapshot.nowMs - snapshot.esc[0].lastGoodMs) : 999999UL;
  uint32_t age1 = snapshot.esc[1].lastGoodMs
                      ? (snapshot.nowMs - snapshot.esc[1].lastGoodMs) : 999999UL;
  int n = snprintf(body, capacity,
    "{\"t\":%lu,\"seq\":%lu,\"gear\":%d,\"mode\":%d,\"fs\":%d,\"lost\":%d,"
    "\"eco\":%d,\"cut\":%d,\"hot\":%d,\"teco\":%d,\"tcut\":%d,\"outL\":%d,\"outR\":%d,"
    "\"e0\":{\"ok\":%d,\"age\":%lu,\"rpm\":%ld,\"cur\":%d,\"v\":%d,\"tE\":%d,\"tM\":%d},"
    "\"e1\":{\"ok\":%d,\"age\":%lu,\"rpm\":%ld,\"cur\":%d,\"v\":%d,\"tE\":%d,\"tM\":%d}}",
    (unsigned long)snapshot.nowMs, sequence, snapshot.gear, snapshot.overrideMode,
    snapshot.rcFailsafe ? 1 : 0, snapshot.rcLostFrame ? 1 : 0,
    snapshot.ecoLockLatched ? 1 : 0, snapshot.batteryCutoffLatched ? 1 : 0,
    snapshot.temperatureWarnActive ? 1 : 0, snapshot.temperatureEcoActive ? 1 : 0,
    snapshot.temperatureCutActive ? 1 : 0,
    snapshot.outLeftUs, snapshot.outRightUs,
    snapshot.esc[0].valid ? 1 : 0, (unsigned long)age0,
    electricalHzToDashboardRpm(snapshot.esc[0].rpmHz),
    scaleToDeciInteger(snapshot.esc[0].busCurrentA), scaleToDeciInteger(snapshot.esc[0].voltage),
    roundToWholeInteger(snapshot.esc[0].escTempC), roundToWholeInteger(snapshot.esc[0].motorTempC),
    snapshot.esc[1].valid ? 1 : 0, (unsigned long)age1,
    electricalHzToDashboardRpm(snapshot.esc[1].rpmHz),
    scaleToDeciInteger(snapshot.esc[1].busCurrentA), scaleToDeciInteger(snapshot.esc[1].voltage),
    roundToWholeInteger(snapshot.esc[1].escTempC), roundToWholeInteger(snapshot.esc[1].motorTempC));
  // snprintf returns the length it WOULD have written; clamp to the buffer so
  // callers never read past `body` (Content-Length and write length stay valid
  // even if a frame were ever to overflow `capacity`).
  if (n < 0) return 0;
  if ((size_t)n >= capacity) n = (int)capacity - 1;
  return n;
}
