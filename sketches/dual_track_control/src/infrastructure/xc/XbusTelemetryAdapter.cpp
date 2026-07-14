// infrastructure/xc — the ONE implementation of ports/TelemetrySource.h
// (#117 step 10 slice 4, #178). The 0x10 poller moved VERBATIM from
// dual_track_control.ino [TELEMETRY]: non-blocking, read-only, one request in flight,
// alternating ESCs, per-ESC staleness watchdog. Both ESCs share the one
// half-duplex bus on Serial1/D0-D1 (idle HIGH, standard UART polarity — no
// inverter). Behavior is locked end-to-end by the characterization suite
// via the stub Serial1 byte scripting.
#include "XbusTelemetryAdapter.h"

#include <Arduino.h>

uint8_t  telemetryPolledEscIndex   = 0;
bool     telemetryAwaitingResponse = false;
uint32_t telemetryLastPollMs       = 0;
uint32_t telemetryRequestStartUs   = 0;
uint8_t  telemetryReceiveBuffer[64];
int      telemetryReceiveLength    = 0;

uint32_t telemetryDebugReceiveTotal    = 0;
uint32_t telemetryDebugEchoCount       = 0;
uint32_t telemetryDebugSlaveCount      = 0;
uint8_t  telemetryDebugSnapshot[16]    = {0};
int      telemetryDebugSnapshotLength  = 0;
uint32_t telemetryDebugPrintPreviousMs = 0;

void telemetrySourceInitialize() {
  Serial1.begin(115200);  // X.BUS telemetry bus on D0/D1 (read-only, 0x10)
}

void telemetrySendReadRequest(uint8_t esc) {
  uint8_t tx[6 + TELEMETRY_REGISTER_COUNT];
  int n = 0;
  tx[n++] = XBUS_HEADER_MASTER;
  tx[n++] = esc;             // point-to-point slave address
  tx[n++] = 0x00;            // extra byte — ignored in service control
  tx[n++] = XBUS_FUNCTION_READ;
  tx[n++] = TELEMETRY_REGISTER_COUNT;  // data length = number of register addresses
  for (uint8_t i = 0; i < TELEMETRY_REGISTER_COUNT; i++) tx[n++] = TELEMETRY_REGISTERS[i];
  tx[n] = xbusChecksum(tx, n + 1);
  n++;
  while (Serial1.available()) Serial1.read();  // drop stale RX / prior echo
  Serial1.write(tx, n);                        // non-blocking (no flush())
  telemetryReceiveLength = 0;
}

// Fold one register value into the caller's EscTelem for one ESC.
static void applyRegister(EscTelem *t, uint8_t reg, uint16_t raw) {
  bool first = !t->valid;
  switch (reg) {
    case REGISTER_BATTERY_VOLTAGE: {
      float v = vbatRawToVolts(raw);
      t->voltage = emaFold(t->voltage, v, TELEMETRY_EMA_ALPHA_VOLTAGE, first);
      break;
    }
    case REGISTER_BUS_CURRENT: {
      float a = busCurrentRawToAmps(raw);
      t->busCurrentA = emaFold(t->busCurrentA, a, TELEMETRY_EMA_ALPHA_CURRENT, first);
      break;
    }
    case REGISTER_MOTOR_SPEED:
      t->rpmHz = motorSpeedRawToElectricalHz(raw);  // instantaneous
      break;
    case REGISTER_ESC_TEMPERATURE: {
      float c = temperatureRawToCelsius(raw);       // temp is the low byte, raw − 40
      t->escTempC = emaFold(t->escTempC, c, TELEMETRY_EMA_ALPHA_TEMPERATURE, first);
      break;
    }
    case REGISTER_MOTOR_TEMPERATURE: {
      float c = temperatureRawToCelsius(raw);
      t->motorTempC = emaFold(t->motorTempC, c, TELEMETRY_EMA_ALPHA_TEMPERATURE, first);
      break;
    }
  }
}

// Scan the RX buffer for a valid 0x10 response from `esc`; parse if complete.
// Skips our own half-duplex TX echo (which starts with 0x0F, not 0xF0).
static bool tryParseResponse(EscTelem *telemetry, uint8_t esc, uint32_t nowMs) {
  for (int i = 0; i + 6 <= telemetryReceiveLength; i++) {
    if (telemetryReceiveBuffer[i]     != XBUS_HEADER_SLAVE)  continue;
    if (telemetryReceiveBuffer[i + 1] != esc)                continue;
    if (telemetryReceiveBuffer[i + 3] != XBUS_FUNCTION_READ) continue;
    uint8_t dlen = telemetryReceiveBuffer[i + 4];
    if (dlen == 0 || dlen % 3 != 0) continue;                 // each reg = addr + 2 bytes
    if (i + 5 + dlen > telemetryReceiveLength) continue;      // full data not in yet
    const uint8_t *d = &telemetryReceiveBuffer[i + 5];
    for (uint8_t g = 0; g + 3 <= dlen; g += 3) {
      uint16_t val = d[g + 1] | (d[g + 2] << 8);
      applyRegister(&telemetry[esc], d[g], val);
    }
    telemetry[esc].lastGoodMs = nowMs;
    telemetry[esc].valid = true;
    return true;
  }
  return false;
}

// Call every loop iteration. Sends one request at a time, alternating ESCs,
// and never blocks: it drains RX across iterations and times out on silence.
void telemetrySourceUpdate(EscTelem *telemetry, uint8_t escCount) {
  if (escCount == 0) return;  // the alternation modulo would divide by zero
  if (telemetryPolledEscIndex >= escCount) {
    // Caller-provided count shrank below the adapter-owned index (never in
    // this firmware — escCount is always NUM_ESCS): restart the rotation
    // rather than index the caller's array out of bounds.
    telemetryPolledEscIndex = 0;
    telemetryAwaitingResponse = false;
  }
  uint32_t nowMs = millis();

  // Drain available bytes every iteration (cheap).
  while (Serial1.available() && telemetryReceiveLength < (int)sizeof(telemetryReceiveBuffer)) {
    uint8_t b = Serial1.read();
    telemetryReceiveBuffer[telemetryReceiveLength++] = b;
    telemetryDebugReceiveTotal++;                        // any byte on D0
    if (b == XBUS_HEADER_MASTER) telemetryDebugEchoCount++;   // 0x0F = our own TX echo
    if (b == XBUS_HEADER_SLAVE)  telemetryDebugSlaveCount++;  // 0xF0 = ESC reply
    if (telemetryDebugSnapshotLength < (int)sizeof(telemetryDebugSnapshot)) {
      telemetryDebugSnapshot[telemetryDebugSnapshotLength++] = b;
    }
  }

  if (telemetryAwaitingResponse) {
    if (tryParseResponse(telemetry, telemetryPolledEscIndex, nowMs)) {
      telemetryAwaitingResponse = false;
      telemetryPolledEscIndex = (telemetryPolledEscIndex + 1) % escCount;
    } else if ((micros() - telemetryRequestStartUs) > TELEMETRY_TIMEOUT_US) {
      telemetryAwaitingResponse = false;             // no/late response — move on
      telemetryPolledEscIndex = (telemetryPolledEscIndex + 1) % escCount;
    }
  } else if ((nowMs - telemetryLastPollMs) >= TELEMETRY_POLL_MS) {
    telemetryLastPollMs = nowMs;
    telemetrySendReadRequest(telemetryPolledEscIndex);
    telemetryRequestStartUs = micros();
    telemetryAwaitingResponse = true;
  }

  // Per-ESC freshness watchdog.
  for (uint8_t i = 0; i < escCount; i++) {
    if (telemetry[i].valid && (nowMs - telemetry[i].lastGoodMs) > TELEMETRY_STALE_MS) {
      telemetry[i].valid = false;
    }
  }
}
