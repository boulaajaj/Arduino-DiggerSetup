// Characterization: end-to-end setup() + loop() (V7.34 baseline).
// Scripted S.BUS frames → real servo microseconds; boot battery gate +
// fail-open; RC failsafe; battery cutoff mid-drive; one WDT refresh per pass.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

// Run `passes` loop() iterations, `stepMs` of fake time apart, optionally
// feeding one copy of `frame` per pass (S.BUS frames arrive at ~70 Hz on the
// wire; 1 ms steps keep sbusValid fresh) and keeping telemetry unstale.
static void runLoops(int passes, const bfs::SbusData* frame, uint32_t stepMs,
                     bool refreshTelemetry = false) {
  for (int i = 0; i < passes; i++) {
    if (frame != nullptr) stub::queueSbusFrame(*frame);
    if (refreshTelemetry) {
      telem[0].lastGoodMs = millis();
      telem[1].lastGoodMs = millis();
    }
    loop();
    stub::advanceMillis(stepMs);
  }
}

TEST_CASE("setup(): 14-bit ADC, S.BUS started, ESCs at neutral, WDT armed last") {
  resetHarness();
  setup();
  CHECK(stub::analogReadBits == 14);
  CHECK(stub::sbusBeginCalled);
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(leftEscMicroseconds() == 1500);
  CHECK(rightEscMicroseconds() == 1500);
  CHECK(WDT.armed);
  CHECK(WDT.timeout == WDT_TIMEOUT_MS);
  CHECK(wifiUp);                                 // stub AP comes up
  CHECK(beepSeq == BEEP_WIFI_READY);             // ready beep queued
}

TEST_CASE("exactly one WDT refresh per loop pass, control path first") {
  resetHarness();
  setup();
  uint32_t before = WDT.refreshCount;
  runLoops(50, nullptr, 1);
  CHECK(WDT.refreshCount == before + 50);
}

TEST_CASE("no RC signal: outputs ease to neutral then PWM cuts entirely") {
  resetHarness();
  setup();
  runLoops(600, nullptr, 1);                     // 600 ms with no S.BUS
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  // Every pulse ever sent was neutral — nothing else was commanded.
  for (const auto& write : stub::servoOnPin(PIN_ESC_L).writes) {
    CHECK(write.second == 1500);
  }
}

TEST_CASE("boot gate (#65): no drive until battery confirms, fail-open after 3 s") {
  resetHarness();
  setup();
  bfs::SbusData fullForward = stub::centeredSbusFrame();
  fullForward.ch[SBUS_CH_THR] = stub::SBUS_RAW_MAX;

  // Valid RC + full throttle, but NO battery telemetry yet → gate stays shut.
  runLoops(100, &fullForward, 1);
  CHECK(leftEscMicroseconds() == 1500);

  // BATTERY_CONFIRM_MS after boot with telemetry still silent → fail OPEN
  // (a dead X.BUS must not permanently disable driving). Centered CH4 =
  // Normal gear: straight-line full stick = 1500 + 0.80*500 = 1900 µs.
  runLoops(3200, &fullForward, 1);
  CHECK(leftEscMicroseconds() == 1900);
  CHECK(rightEscMicroseconds() == 1900);
}

TEST_CASE("battery confirmed at boot: drive is allowed immediately") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  bfs::SbusData fullForward = stub::centeredSbusFrame();
  fullForward.ch[SBUS_CH_THR] = stub::SBUS_RAW_MAX;

  runLoops(20, &fullForward, 1, true);
  CHECK(leftEscMicroseconds() == 1900);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 1);   // gate never closed
}

TEST_CASE("S.BUS failsafe flag: drive stops with the ease-out ramp") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  bfs::SbusData fullForward = stub::centeredSbusFrame();
  fullForward.ch[SBUS_CH_THR] = stub::SBUS_RAW_MAX;
  runLoops(20, &fullForward, 1, true);
  REQUIRE(leftEscMicroseconds() == 1900);

  bfs::SbusData failsafe = fullForward;
  failsafe.failsafe = true;                      // transmitter gone
  runLoops(600, &failsafe, 1, true);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  // The ramp passed through intermediate values on its way to neutral.
  bool sawIntermediate = false;
  for (const auto& write : stub::servoOnPin(PIN_ESC_L).writes) {
    if (write.second > 1500 && write.second < 1900) sawIntermediate = true;
  }
  CHECK(sawIntermediate);
}

TEST_CASE("RC stream stops: 100 ms timeout closes the gate; frames returning reopen it") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  bfs::SbusData fullForward = stub::centeredSbusFrame();
  fullForward.ch[SBUS_CH_THR] = stub::SBUS_RAW_MAX;
  runLoops(20, &fullForward, 1, true);
  REQUIRE(leftEscMicroseconds() == 1900);

  runLoops(700, nullptr, 1, true);               // silence → timeout → ease → cut
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);

  runLoops(20, &fullForward, 1, true);           // RC-loss recovers automatically
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(leftEscMicroseconds() == 1900);
}

TEST_CASE("battery cutoff mid-drive: 1.5 s debounce, ease out, latched until power cycle") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  bfs::SbusData fullForward = stub::centeredSbusFrame();
  fullForward.ch[SBUS_CH_THR] = stub::SBUS_RAW_MAX;
  runLoops(20, &fullForward, 1, true);
  REQUIRE(leftEscMicroseconds() == 1900);

  telem[0].voltage = 9.5f;                       // pack collapses under load
  runLoops(1400, &fullForward, 1, true);
  CHECK_FALSE(batteryCutoffLatched);             // inside the sag debounce

  runLoops(800, &fullForward, 1, true);
  CHECK(batteryCutoffLatched);
  CHECK(lowVoltLatched);                         // alarm sounds WITH the cut
  runLoops(600, &fullForward, 1, true);          // ease-out completes
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);

  telem[0].voltage = 12.5f;                      // recovery does NOT reopen
  runLoops(100, &fullForward, 1, true);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
}

TEST_CASE("horn: RC CH7 above 1400 raw drives D8 while held") {
  resetHarness();
  setup();
  runLoops(1000, nullptr, 1);                    // let the Wi-Fi ready beep finish
  bfs::SbusData frame = stub::centeredSbusFrame();
  frame.ch[SBUS_CH_HORN] = 1500;                 // SWD pressed
  runLoops(2, &frame, 1);
  CHECK(stub::digitalLevels[PIN_BEEPER] == HIGH);

  frame.ch[SBUS_CH_HORN] = stub::SBUS_RAW_CENTER;
  runLoops(2, &frame, 1);
  CHECK(stub::digitalLevels[PIN_BEEPER] == LOW);
}
