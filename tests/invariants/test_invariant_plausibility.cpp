// Invariant #131-5: implausible battery / temperature readings are never
// trusted — the plausibility gates hold under fault injection. Covers: an
// unpowered pack (~0 V) or an impossibly high read (14.2 V) never trips the
// battery ladder; one silent ESC blocks the ladder (both-valid rule); a
// telemetry dropout mid-drive clears valid without tripping anything (drive is
// telemetry-optional); voltage sags and temperature spikes shorter than their
// debounce windows are rejected and the timers reset; a NaN voltage exposes a
// documented gap in the plausibility band (see the #131 FINDING below).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "InvariantChecks.h"

#include <cmath>

// Full-forward, centered steering, Mode 1 (RC only), Normal gear: both tracks
// at 1500 + 0.80 * 500 = 1900 us once the gate is open.
const int FULL_FORWARD_NORMAL_US = 1900;
inline bfs::SbusData fullForwardFrame() {
  return driveFrame(RAW_SWITCH_HIGH, RAW_SWITCH_MID, RAW_SWITCH_LOW, RAW_SWITCH_MID);
}

TEST_CASE("worstPackVoltage (#131): implausible or missing pack readings are rejected") {
  resetHarness();
  float worst = -1.0f;
  // Unpowered pack (~0 V) — below LOWV_PLAUS_MIN_V — on either slot.
  setPackVoltages(0.4f, 12.6f);
  CHECK(worstPackVoltage(&worst) == false);
  setPackVoltages(12.6f, 0.4f);
  CHECK(worstPackVoltage(&worst) == false);
  // Impossibly high read — above LOWV_PLAUS_MAX_V — on either slot.
  setPackVoltages(14.2f, 12.6f);
  CHECK(worstPackVoltage(&worst) == false);
  setPackVoltages(12.6f, 14.2f);
  CHECK(worstPackVoltage(&worst) == false);
  // One ESC silent: a plausible (even alarming) reading on the OTHER pack is
  // not enough — the ladder needs trustworthy data from both.
  setPackVoltages(9.0f, 9.0f);
  telem[1].valid = false;
  CHECK(worstPackVoltage(&worst) == false);
  // Both plausible → worst of the two; the band boundaries are inclusive.
  setPackVoltages(6.0f, 13.0f);
  CHECK(worstPackVoltage(&worst) == true);
  CHECK(worst == doctest::Approx(6.0f));
}

TEST_CASE("end-to-end (#131): implausible pack reading never trips the ladder or slows the drive") {
  resetHarness();
  setup();
  bfs::SbusData frame = fullForwardFrame();
  const float implausibleVoltages[] = {0.4f, 14.2f};  // unpowered / bad read
  for (float implausibleVoltage : implausibleVoltages) {
    setPackVoltages(implausibleVoltage, 12.6f);
    // 22 s sustained — far past every ladder debounce (1.5 s cutoff, 15 s eco
    // lock). Boot gate fails OPEN at 3 s since the pack can never confirm.
    runControlPasses(1100, &frame, 20, true);
    CHECK(batteryCutoffLatched == false);
    CHECK(ecoLockLatched == false);
    CHECK(batteryOkConfirmed == false);  // implausible data confirms nothing either
    CHECK(currentGear == GEAR_MID);      // gear never forced to Eco
    CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);
    CHECK(rightEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  }
  checkInvariants();
}

TEST_CASE("one ESC silent (#131): 9.0 V on the surviving pack alone never trips the ladder") {
  resetHarness();
  setup();
  setPackVoltages(9.0f, 9.0f);  // deep below cutoff / eco-lock / low-V thresholds...
  telem[1].valid = false;       // ...but ESC 1 never reports — both-valid rule blocks
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(1100, &frame, 20, true);  // 22 s, far past every debounce
  CHECK(telem[1].valid == false);
  CHECK(batteryCutoffLatched == false);
  CHECK(ecoLockLatched == false);
  CHECK(lowVoltLatched == false);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);  // drive continues (3 s fail-open)
  CHECK(rightEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  checkInvariants();
}

TEST_CASE("telemetry dropout mid-drive (#131): staleness clears valid, nothing trips, drive continues") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(50, &frame, 20, true);  // 1 s of healthy confirmed driving
  REQUIRE(batteryOkConfirmed == true);
  REQUIRE(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  // X.BUS goes silent: 8 s without refresh (> TELEM_STALE_MS = 5 s).
  runControlPasses(400, &frame, 20, false);
  CHECK(telem[0].valid == false);        // staleness watchdog fired on both ESCs
  CHECK(telem[1].valid == false);
  CHECK(batteryCutoffLatched == false);  // a dropout never trips a latch
  CHECK(ecoLockLatched == false);
  CHECK(tempCutActive == false);
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);  // telemetry-optional by design
  CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  CHECK(rightEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  checkInvariants();
}

TEST_CASE("voltage sag shorter than debounce (#131): no cutoff, and the timer resets between dips") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(50, &frame, 20, true);
  REQUIRE(batteryOkConfirmed == true);

  telem[0].voltage = 9.5f;                 // load sag below CUTOFF_THRESH_V...
  runControlPasses(50, &frame, 20, true);  // ...held 1.0 s < 1.5 s debounce
  CHECK(batteryCutoffLatched == false);
  telem[0].voltage = 12.6f;                // recovers before the debounce elapses
  runControlPasses(10, &frame, 20, true);
  CHECK(cutoffStartMs == 0);               // debounce timer fully reset
  telem[0].voltage = 9.5f;                 // a second 1.0 s dip later...
  runControlPasses(50, &frame, 20, true);
  CHECK(batteryCutoffLatched == false);    // ...never accumulates into a trip
  telem[0].voltage = 12.6f;
  runControlPasses(10, &frame, 20, true);

  telem[0].voltage = 10.6f;                 // below ECO_LOCK_THRESH_V 10.8...
  runControlPasses(500, &frame, 20, true);  // ...held 10 s < 15 s debounce
  CHECK(ecoLockLatched == false);
  telem[0].voltage = 12.6f;
  runControlPasses(10, &frame, 20, true);
  CHECK(ecoLockStartMs == 0);
  CHECK(currentGear == GEAR_MID);          // gear never got forced to Eco
  CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  checkInvariants();
}

TEST_CASE("temperature plausibility (#131): 250 C / -30 C faults ignored; sub-second 96 C spike rejected") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  setAllTemperatures(25.0f);
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(50, &frame, 20, true);
  REQUIRE(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);

  telem[0].escTempC = 250.0f;               // above TEMP_PLAUS_MAX_C — bad read
  runControlPasses(500, &frame, 20, true);  // 10 s sustained
  CHECK(tempWarnActive == false);
  CHECK(tempEcoActive == false);
  CHECK(tempCutActive == false);

  telem[0].escTempC = -30.0f;               // below TEMP_PLAUS_MIN_C — dead sensor
  runControlPasses(500, &frame, 20, true);
  CHECK(tempWarnActive == false);
  CHECK(tempEcoActive == false);
  CHECK(tempCutActive == false);

  telem[0].escTempC = 96.0f;               // plausible and above TEMP_CUT_ON_C...
  runControlPasses(40, &frame, 20, true);  // ...held 0.8 s < 1.0 s trip debounce
  CHECK(tempCutActive == false);
  telem[0].escTempC = 25.0f;               // spike ends before any stage can trip
  runControlPasses(10, &frame, 20, true);
  CHECK(tempCutActive == false);
  CHECK(tempWarnActive == false);          // warn shares the 1 s debounce — also too short
  CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);  // drive never interrupted
  checkInvariants();
}

TEST_CASE("NaN pack voltage (#131): plausibility band does not catch NaN — outcome depends on slot") {
  resetHarness();
  // #131 FINDING: NaN passes the [LOWV_PLAUS_MIN_V, LOWV_PLAUS_MAX_V] band —
  // every comparison against NaN is false, so NaN is never flagged implausible.
  // worstPackVoltage's min-select is the ternary `(v0 < v1) ? v0 : v1`, so the
  // outcome depends on WHICH slot holds the NaN:
  //   - NaN in slot 0: `NaN < v1` is false → the OTHER pack's value is returned
  //     as "worst"; the garbage reading is silently masked by a healthy pack
  //     (and batteryOkConfirmed CAN be set while one pack reads NaN).
  //   - NaN in slot 1: `v0 < NaN` is false → NaN itself propagates as "worst";
  //     every downstream threshold comparison is false → the ladder FREEZES
  //     (no trip, no confirm, no crash).
  // Unreachable via today's integer X.BUS parse path (voltages are built from
  // uint16 registers); filed as a follow-up firmware-hardening issue.
  float worst = -1.0f;
  setPackVoltages(NAN, 12.6f);
  CHECK(worstPackVoltage(&worst) == true);   // NaN NOT rejected by the gate
  CHECK(worst == doctest::Approx(12.6f));    // healthy pack masks the NaN
  worst = -1.0f;
  setPackVoltages(12.6f, NAN);
  CHECK(worstPackVoltage(&worst) == true);   // NaN NOT rejected by the gate
  CHECK(std::isnan(worst));                  // NaN propagates as "worst"
}

TEST_CASE("NaN pack voltage end-to-end (#131): ladder freezes or confirms by slot; outputs bounded") {
  resetHarness();
  setup();
  // Slot-1 NaN: worst = NaN → the ladder freezes (see FINDING above).
  setPackVoltages(12.6f, NAN);
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(1100, &frame, 20, true);  // 22 s of sustained NaN
  CHECK(batteryOkConfirmed == false);        // NaN >= 10.0 is false → never confirms
  CHECK(batteryCutoffLatched == false);      // NaN < 10.0 is false → never trips
  CHECK(ecoLockLatched == false);
  CHECK(lowVoltLatched == false);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);  // 3 s fail-open, then drives

  // #131 FINDING (continued): move the NaN to slot 0 and the ladder does NOT
  // freeze — the ternary hands the healthy 12.6 V through as "worst" and
  // batteryOkConfirmed IS set while pack 0 is reading garbage.
  setPackVoltages(NAN, 12.6f);
  runControlPasses(10, &frame, 20, true);
  CHECK(batteryOkConfirmed == true);         // confirmed despite a NaN pack reading
  CHECK(batteryCutoffLatched == false);
  CHECK(ecoLockLatched == false);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_NORMAL_US);
  checkInvariants();
}
