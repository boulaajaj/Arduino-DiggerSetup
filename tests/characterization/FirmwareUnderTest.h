// FirmwareUnderTest.h — compiles the REAL production sketch (byte-for-byte
// unchanged) into a host test binary. Stub headers in tests/stubs shadow the
// Arduino core via include order (-Istubs before anything else), so
// rc_test.ino's own #includes resolve to the fakes.
//
// Each test file includes this header and becomes ONE self-contained binary
// with the whole firmware inside it (no ODR issues, no state bleed between
// test files). Inside a file, call resetHarness() at the top of every
// TEST_CASE to restore boot-fresh state.
#pragma once

#include "Arduino.h"
#include "Servo.h"
#include "WDT.h"
#include "WiFiS3.h"
#include "sbus.h"

// The production firmware, unmodified. Everything it defines — functions,
// globals, and (internal-linkage) constants — is visible to the tests.
#include "../../sketches/rc_test/rc_test.ino"  // NOLINT(build/include)

// Restore every MUTABLE firmware global to its boot value. Must be kept in
// sync with rc_test.ino: when a PR adds a mutable global there, it adds the
// reset here (the firmware-without-tests commit guard forces the pairing).
// Constants need no entry.
inline void resetFirmwareState() {
  // [CONFIG] gear + safety latches
  gearScale = GEAR_LOW_SCALE;
  currentGear = GEAR_LOW;
  batteryCutoffLatched = false;
  ecoLockLatched = false;
  batteryOkConfirmed = false;
  tempWarnActive = false;
  tempEcoActive = false;
  tempCutActive = false;
  // [RC]
  sbusData = bfs::SbusData{};
  sbusValid = false;
  sbusLastFrame = 0;
  // [JOYSTICK]
  cachedJoy = {ADC_CENTER, ADC_CENTER, 0.0f, 0.0f};
  cachedJoyCmd = {0.0f, 0.0f};
  lastAdcTime = 0;
  // [OUTPUT] — fresh Servo objects too: a stale pin binding from a previous
  // test case would otherwise let outputWrite() log into the cleared capture.
  escL = Servo{};
  escR = Servo{};
  outL = SVC;
  outR = SVC;
  outState = OUT_ACTIVE;
  outHoldMs = 0;
  rampFromL = SVC;
  rampFromR = SVC;
  // [TELEMETRY]
  telem[0] = EscTelem{};
  telem[1] = EscTelem{};
  telemEsc = 0;
  telemWaiting = false;
  telemLastPollMs = 0;
  telemReqStartUs = 0;
  telRxLen = 0;
  telDbgRxTotal = 0;
  telDbgEchoCount = 0;
  telDbgSlaveCount = 0;
  telDbgSnapLen = 0;
  telDbgPrintPrevMs = 0;
  // [BEEPER]
  hornActive = false;
  alarmOutputOn = false;
  beepSeq = nullptr;
  beepLen = 0;
  beepIdx = -1;
  beepPhaseMs = 0;
  // [ALERT]
  rcOffSinceMs = 0;
  lowVStartMs = 0;
  lowVoltLatched = false;
  alertBootMs = 0;
  alarmSeq = nullptr;
  alarmLen = 0;
  alarmIdx = 0;
  alarmPhaseMs = 0;
  // [SAFETY]
  cutoffStartMs = 0;
  ecoLockStartMs = 0;
  tempWarnSinceMs = 0;
  tempEcoSinceMs = 0;
  tempCutSinceMs = 0;
  // [WIFI]
  sseClient = WiFiClient{};
  pageClient = WiFiClient{};
  wifiUp = false;
  wifiSeq = 0;
  snprintf(pageEtag, sizeof(pageEtag), "\"d0\"");
  sseLastMs = 0;
  sseActive = false;
  pagePtr = nullptr;
  pageRemaining = 0;
  wifiDbgPrev = 0;
  // [DEBUG]
  prevPrint = 0;
}

// Fresh stubs + fresh firmware state — call first in every TEST_CASE.
inline void resetHarness() {
  stub::resetArduinoCore();
  stub::resetSerialPorts();
  stub::resetServos();
  stub::resetWatchdog();
  stub::resetSbus();
  stub::resetWiFi();
  resetFirmwareState();
}

// ── Shared scenario helpers ─────────────────────────────────────────────────

// Servo-µs shorthand for the two ESC output pins.
inline int leftEscMicroseconds()  { return stub::servoOnPin(PIN_ESC_L).lastMicroseconds; }
inline int rightEscMicroseconds() { return stub::servoOnPin(PIN_ESC_R).lastMicroseconds; }

// Queue one S.BUS frame and run the sbus-read portion the way loop() does,
// so sbusValid/sbusData/sbusLastFrame update exactly as in flight.
inline void feedSbusFrame(const bfs::SbusData& frame) {
  stub::queueSbusFrame(frame);
  uint32_t now = micros();
  if (sbusRx.Read()) {
    sbusData = sbusRx.data();
    sbusLastFrame = now;
    sbusValid = !sbusData.failsafe;
  }
}

// Mark both ESC telemetry packs valid at the given voltages (EMA state is
// written directly — the X.BUS parse path has its own suite).
inline void setPackVoltages(float voltage0, float voltage1) {
  telem[0].voltage = voltage0;
  telem[0].valid = true;
  telem[0].lastGoodMs = millis();
  telem[1].voltage = voltage1;
  telem[1].valid = true;
  telem[1].lastGoodMs = millis();
}

// Set all four temperature sensors (ESC + motor on both ESCs) to one value.
inline void setAllTemperatures(float temperatureC) {
  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    telem[i].escTempC = temperatureC;
    telem[i].motorTempC = temperatureC;
    telem[i].valid = true;
    telem[i].lastGoodMs = millis();
  }
}
