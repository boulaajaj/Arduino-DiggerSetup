// InvariantChecks.h — reusable safety-invariant assertions (#131).
//
// checkInvariants() is the issue's "one function reusable across all scenario
// tests". Pre-Phase-D form: it reads firmware globals and the servo stubs
// directly; once #132 lands it becomes checkInvariants(snapshot, outputs).
// runControlPasses() drives loop() at flight cadence and re-checks the
// per-instant invariants after EVERY pass, so any scenario built on it is
// property-checked continuously, not just at its endpoints.
//
// Include AFTER doctest.h (this header uses doctest macros). Including this
// header pulls in the whole firmware via FirmwareUnderTest.h — one test file
// per binary, same rule as the characterization suite.
#pragma once

#include "../characterization/FirmwareUnderTest.h"

// ── Per-instant invariants — must hold after every single loop() pass ───────

// Fail loudly with the pass index if a universal safety invariant is violated
// at this instant. Uses plain conditions + FAIL (not one CHECK per pass) so a
// 600-pass scenario reports a handful of meaningful assertions, not thousands.
inline void checkInvariantsNow(int passIndex) {
  const stub::ServoLog& left  = stub::servoOnPin(PIN_ESC_L);
  const stub::ServoLog& right = stub::servoOnPin(PIN_ESC_R);
  // #131-1: the last written command is inside the servo rails.
  if (outL < SVMIN || outL > SVMAX || outR < SVMIN || outR > SVMAX) {
    FAIL("pass " << passIndex << ": output outside [" << SVMIN << ", " << SVMAX
                 << "] us: left=" << outL << " right=" << outR);
  }
  // Firmware must never write to a detached servo (stub counts it as a bug).
  if (left.writesWhileDetached != 0 || right.writesWhileDetached != 0) {
    FAIL("pass " << passIndex << ": writeMicroseconds() on a detached servo");
  }
  // Gate state and hardware state must agree: OUT_CUT means detached.
  if (outState == OUT_CUT && (left.attached || right.attached)) {
    FAIL("pass " << passIndex << ": outState is OUT_CUT but a servo is attached");
  }
}

// ── Whole-scenario invariants — call once at the end of a scenario ──────────

// #131-1 over the FULL pulse history: every pulse ever recorded on either ESC
// pin stayed within [SVMIN, SVMAX], and nothing was written while detached.
inline void checkInvariants() {
  const int escPins[2] = {PIN_ESC_L, PIN_ESC_R};
  for (int pin : escPins) {
    const stub::ServoLog& log = stub::servoOnPin(pin);
    int outOfBounds = 0;
    for (const auto& write : log.writes) {
      if (write.second < SVMIN || write.second > SVMAX) outOfBounds++;
    }
    CHECK(outOfBounds == 0);
    CHECK(log.writesWhileDetached == 0);
  }
}

// #131-2/-3/-6: every pulse recorded at or after sinceMs is exactly neutral
// (SVC). Use for "once the gate finished closing, nothing can move the tracks
// again" — pass sinceMs = closeStart + CUTOFF_HOLD_MS (+ margin) so the
// intentional ease-out ramp is excluded.
inline void checkNeutralPulsesSince(uint32_t sinceMs) {
  const int escPins[2] = {PIN_ESC_L, PIN_ESC_R};
  for (int pin : escPins) {
    const stub::ServoLog& log = stub::servoOnPin(pin);
    int nonNeutral = 0;
    for (const auto& write : log.writes) {
      if (write.first >= sinceMs && write.second != SVC) nonNeutral++;
    }
    CHECK(nonNeutral == 0);
  }
}

// ── Scenario drivers and frame builders ──────────────────────────────────────

// Drive loop() at flight cadence: queue one S.BUS frame per pass (nullptr =
// RC silence, i.e. link loss), optionally re-stamp telemetry freshness so the
// 5 s staleness watchdog (TELEM_STALE_MS) does not fire mid-scenario, advance
// the fake clock stepMs, and re-check the per-instant invariants every pass.
inline void runControlPasses(int passes, const bfs::SbusData* frame,
                             uint32_t stepMs, bool refreshTelemetry) {
  for (int i = 0; i < passes; i++) {
    if (frame != nullptr) stub::queueSbusFrame(*frame);
    if (refreshTelemetry) {
      telem[0].lastGoodMs = millis();
      telem[1].lastGoodMs = millis();
    }
    loop();
    checkInvariantsNow(i);
    stub::advanceMillis(stepMs);
  }
}

// Raw S.BUS values landing squarely in the three bands of a 3-position switch
// after sbusToServo() (1000 / 1500 / 2000 us against the 1400/1600 thresholds).
const int16_t RAW_SWITCH_LOW  = stub::SBUS_RAW_MIN;     // Mode 1 / Eco
const int16_t RAW_SWITCH_MID  = stub::SBUS_RAW_CENTER;  // Mode 2 / Normal
const int16_t RAW_SWITCH_HIGH = stub::SBUS_RAW_MAX;     // Mode 3 / Boost

// Centered frame with throttle / steering / override / gear channels set.
inline bfs::SbusData driveFrame(int16_t throttleRaw, int16_t steeringRaw,
                                int16_t overrideRaw, int16_t gearRaw) {
  bfs::SbusData frame = stub::centeredSbusFrame();
  frame.ch[SBUS_CH_THR]   = throttleRaw;
  frame.ch[SBUS_CH_STEER] = steeringRaw;
  frame.ch[SBUS_CH_OVR]   = overrideRaw;
  frame.ch[SBUS_CH_GEAR]  = gearRaw;
  return frame;
}

// Average track command in µs — the quantity the gear and reverse caps bound
// (per-track excursions beyond the cap are legitimate turn headroom, #72/#113).
inline float averageTrackMicroseconds() {
  return (leftEscMicroseconds() + rightEscMicroseconds()) / 2.0f;
}
