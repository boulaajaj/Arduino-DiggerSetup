// Pure suite (#183): src/alerts/ tested directly on the host — no firmware,
// no stubs. End-to-end alarm behavior through the .ino shims stays covered
// by tests/characterization/test_alerts.cpp (expectations are law); this
// suite locks the extracted policy/sequencing contracts at the unit level.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/dual_track_control/src/alerts/AlertPolicy.h"
#include "../../sketches/dual_track_control/src/alerts/PatternPlayer.h"

static const uint16_t SEQ_A[] = {100, 100};
static const uint16_t SEQ_B[] = {500, 1500};

static AlertPatternTable exampleTable() {
  return AlertPatternTable{{SEQ_A, 2}, {SEQ_B, 2}, {SEQ_A, 2}, {SEQ_B, 2}};
}

TEST_CASE("priority ladder: cut > low-V > warn > inactivity, else silent") {
  AlertPatternTable table = exampleTable();
  CHECK(alertPatternSelect(true, true, true, true, table).sequence == table.thermalCut.sequence);
  CHECK(alertPatternSelect(false, true, true, true, table).sequence == table.lowVoltage.sequence);
  CHECK(alertPatternSelect(false, false, true, true, table).sequence == table.thermalWarn.sequence);
  CHECK(alertPatternSelect(false, false, false, true, table).sequence == table.inactivity.sequence);
  CHECK(alertPatternSelect(false, false, false, false, table).sequence == nullptr);
}

TEST_CASE("low-voltage latch: debounced, recovery resets, sticky once latched") {
  LowVoltageThresholds thresholds{10.5f, 6.0f, 13.0f, 3000, 60000};
  LowVoltageLatchState state{};
  // Below threshold but debounce not yet elapsed → not latched.
  lowVoltageLatchStep(state, 10.0f, 11.0f, true, true, 60000, 0, thresholds);
  lowVoltageLatchStep(state, 10.0f, 11.0f, true, true, 62000, 0, thresholds);
  CHECK_FALSE(state.latched);
  // Recovery before debounce resets the timer.
  lowVoltageLatchStep(state, 11.5f, 11.0f, true, true, 62500, 0, thresholds);
  CHECK(state.belowSinceMs == 0);
  // Sustained dip past the debounce latches.
  lowVoltageLatchStep(state, 10.0f, 11.0f, true, true, 63000, 0, thresholds);
  lowVoltageLatchStep(state, 10.0f, 11.0f, true, true, 66000, 0, thresholds);
  CHECK(state.latched);
  // Sticky: recovery does not unlatch.
  lowVoltageLatchStep(state, 12.6f, 12.6f, true, true, 70000, 0, thresholds);
  CHECK(state.latched);
}

TEST_CASE("low-voltage latch: startup grace and implausible readings stay silent") {
  LowVoltageThresholds thresholds{10.5f, 6.0f, 13.0f, 3000, 60000};
  LowVoltageLatchState state{};
  // Inside the startup grace nothing runs.
  lowVoltageLatchStep(state, 9.0f, 9.0f, true, true, 1000, 0, thresholds);
  CHECK(state.belowSinceMs == 0);
  // An implausible pack (not powered, ~0 V) resets the debounce.
  lowVoltageLatchStep(state, 10.0f, 11.0f, true, true, 60000, 0, thresholds);
  CHECK(state.belowSinceMs == 60000);
  lowVoltageLatchStep(state, 10.0f, 0.5f, true, true, 61000, 0, thresholds);
  CHECK(state.belowSinceMs == 0);
  CHECK_FALSE(state.latched);
}

TEST_CASE("inactivity: threshold fires after the off period, clears on RC return") {
  uint32_t offSinceMs = 0;
  CHECK_FALSE(inactivityAlarmStep(offSinceMs, false, 1000, 60000));
  CHECK(offSinceMs == 1000);
  CHECK(inactivityAlarmStep(offSinceMs, false, 61000, 60000));
  CHECK_FALSE(inactivityAlarmStep(offSinceMs, true, 62000, 60000));
  CHECK(offSinceMs == 0);
}

TEST_CASE("one-shot player: even phases sound, then silence at the end; idle stays off") {
  OneShotPatternState pattern{};
  pattern.index = -1;                                    // boot idle state
  CHECK_FALSE(oneShotPatternStep(pattern, 0));
  oneShotPatternStart(pattern, SEQ_B, 2, 1000);          // 500 on, 1500 off
  CHECK(oneShotPatternStep(pattern, 1000));              // ON phase
  CHECK_FALSE(oneShotPatternStep(pattern, 1500));        // advanced to OFF
  CHECK_FALSE(oneShotPatternStep(pattern, 3000));        // finished → silent
  CHECK_FALSE(oneShotPatternStep(pattern, 9000));        // stays silent
}

TEST_CASE("repeating player: loops, and only a CHANGED sequence restarts playback") {
  RepeatingPatternState alarm{};
  repeatingPatternSelect(alarm, SEQ_A, 2, 1000);         // 100 on, 100 off
  CHECK(repeatingPatternStep(alarm, 1000));              // ON
  CHECK_FALSE(repeatingPatternStep(alarm, 1100));        // OFF
  CHECK(repeatingPatternStep(alarm, 1200));              // loops back to ON
  uint32_t phaseBefore = alarm.phaseMs;
  repeatingPatternSelect(alarm, SEQ_A, 2, 5000);         // same sequence → no restart
  CHECK(alarm.phaseMs == phaseBefore);
  repeatingPatternSelect(alarm, SEQ_B, 2, 5000);         // new sequence → restart
  CHECK(alarm.index == 0);
  CHECK(alarm.phaseMs == 5000);
}
