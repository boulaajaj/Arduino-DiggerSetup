// infrastructure/xc — X.BUS telemetry poller: protocol constants, register
// scaling and the poll machine's contract (#117 step 10 slice 4, #178).
// Implementation (and the only Serial1 use) is in XbusTelemetryAdapter.cpp;
// this header is host-pure so the characterization and pure suites can
// include it directly. Poll state is declared extern so the test harness
// can reset and inspect the machine (the escL/escR / sbusRx precedent).
//
// The bus is READ-ONLY: function 0x10 is point-to-point service control —
// it never puts the ESC into BUS_MODE, so PWM control authority is fully
// preserved (0x50 would fight our PWM; see docs/XBUS-PROTOCOL.md).
#pragma once

#include <stdint.h>

#include "../../ports/TelemetrySource.h"

// ── X.BUS wire protocol (framing confirmed on the bench at bring-up) ─────
const uint8_t XBUS_HEADER_MASTER = 0x0F;
const uint8_t XBUS_HEADER_SLAVE  = 0xF0;
const uint8_t XBUS_FUNCTION_READ = 0x10;

// Registers read every poll (one request returns all of them).
const uint8_t REGISTER_BATTERY_VOLTAGE   = 0x0C;  // ×0.1 V
const uint8_t REGISTER_BUS_CURRENT       = 0x0D;  // ×0.1 A (signed)
const uint8_t REGISTER_MOTOR_SPEED       = 0x02;  // electrical Hz (signed)
const uint8_t REGISTER_ESC_TEMPERATURE   = 0x20;  // raw − 40 = °C
const uint8_t REGISTER_MOTOR_TEMPERATURE = 0x22;  // raw − 40 = °C
const uint8_t TELEMETRY_REGISTERS[] = {
    REGISTER_BATTERY_VOLTAGE, REGISTER_BUS_CURRENT, REGISTER_MOTOR_SPEED,
    REGISTER_ESC_TEMPERATURE, REGISTER_MOTOR_TEMPERATURE};
const uint8_t TELEMETRY_REGISTER_COUNT =
    sizeof(TELEMETRY_REGISTERS) / sizeof(TELEMETRY_REGISTERS[0]);

// ── poll cadence / freshness ─────────────────────────────────────────────
const uint32_t TELEMETRY_POLL_MS    = 10;     // short gap; alternating → ~30-40 Hz/ESC
const uint32_t TELEMETRY_TIMEOUT_US = 6000;   // GL10 replies in ~2-3ms; 6ms = margin +
                                              // fast give-up on a silent ESC
const uint32_t TELEMETRY_STALE_MS   = 5000;   // mark invalid after this long unheard

// EMA new-sample weights. Slow signals smoothed; RPM is instantaneous.
const float TELEMETRY_EMA_ALPHA_VOLTAGE     = 0.30f;
const float TELEMETRY_EMA_ALPHA_CURRENT     = 0.50f;
const float TELEMETRY_EMA_ALPHA_TEMPERATURE = 0.10f;

// Checksum = low 8 bits of the sum of bytes [1 .. len-2] (header & checksum
// excluded). Matches the framing confirmed on the bench during bring-up.
inline uint8_t xbusChecksum(const uint8_t *frame, int length) {
  uint8_t sum = 0;
  for (int i = 1; i < length - 1; i++) sum += frame[i];
  return sum;
}

// ── register decode: raw register value → engineering units ──────────────
// Moved from telemetry/TelemetryScaling.h (#178): infrastructure/ may not
// include telemetry/ (dependency table §3), and the decode belongs to the
// X.BUS register semantics. The ×10 wire ENCODE stays in TelemetryScaling.h.

inline float vbatRawToVolts(uint16_t raw) {          // 0x0C, ×0.1 V
  return raw * 0.1f;
}

inline float busCurrentRawToAmps(uint16_t raw) {     // 0x0D, ×0.1 A signed
  return (int16_t)raw * 0.1f;
}

inline int16_t motorSpeedRawToElectricalHz(uint16_t raw) {  // 0x02, signed
  return (int16_t)raw;
}

inline float temperatureRawToCelsius(uint16_t raw) {  // 0x20/0x22: low byte, raw − 40
  return (float)(raw & 0xFF) - 40.0f;
}

// Exponential moving average fold; the first sample seeds the average.
inline float emaFold(float previous, float sample, float alpha, bool first) {
  return first ? sample : previous + alpha * (sample - previous);
}

// ── poll machine state (owned by the adapter; extern for the harness) ────
extern uint8_t  telemetryPolledEscIndex;   // ESC currently being polled
extern bool     telemetryAwaitingResponse;  // true = request sent, awaiting reply
extern uint32_t telemetryLastPollMs;
extern uint32_t telemetryRequestStartUs;
extern uint8_t  telemetryReceiveBuffer[64];
extern int      telemetryReceiveLength;

// X.BUS RX byte-level diagnostics (temporary, post-solder bring-up).
extern uint32_t telemetryDebugReceiveTotal;      // total bytes ever seen on D0
extern uint32_t telemetryDebugEchoCount;         // 0x0F bytes (our own TX echo)
extern uint32_t telemetryDebugSlaveCount;        // 0xF0 bytes (ESC response header)
extern uint8_t  telemetryDebugSnapshot[16];
extern int      telemetryDebugSnapshotLength;
extern uint32_t telemetryDebugPrintPreviousMs;

// Frame and send one 0x10 read of all five registers to `esc` (drops any
// stale RX first). Exposed for the characterization suite.
void telemetrySendReadRequest(uint8_t esc);
