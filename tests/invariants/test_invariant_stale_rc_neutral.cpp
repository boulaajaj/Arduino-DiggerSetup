// Invariant #131-2: stale or invalid RC input NEVER produces non-neutral
// propulsion — in every override mode, with the joystick live or not.
// Two loss paths are covered end-to-end: S.BUS silence (100 ms SBUS_TIMEOUT)
// and the receiver failsafe bit (TX off, RX alive). Both must ease the tracks
// to neutral over CUTOFF_HOLD_MS and then detach the ESC pins entirely.
// A live (railed) joystick must have zero authority while RC is dead — the
// override mixer only runs when sbusValid, and rcOverride() falls back to
// Mode 1. Recovery after loss is legitimate and asserted separately.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "InvariantChecks.h"

// After RC loss begins, every pulse must be neutral once the loss is detected
// and the intentional ease-out ramp has finished (small margin for the
// one-pass detection granularity).
const uint32_t RAMP_MARGIN_MS = 25UL;
inline uint32_t neutralDeadlineAfterSilence(uint32_t silenceStartMs) {
  return silenceStartMs + (SBUS_TIMEOUT / 1000UL) + CUTOFF_HOLD_MS + RAMP_MARGIN_MS;
}

TEST_CASE("RC silence mid-drive (#131): full forward ramps to neutral then PWM cuts") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  // Mode 1 (RC only), Boost, full forward: both tracks at the 2000 us rail.
  bfs::SbusData fullForward =
      driveFrame(RAW_SWITCH_HIGH, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(20, &fullForward, 1, true);
  REQUIRE(leftEscMicroseconds() == 2000);
  REQUIRE(rightEscMicroseconds() == 2000);

  uint32_t silenceStartMs = millis();
  runControlPasses(700, nullptr, 1, true);       // link loss: no frames at all

  // The ease-out ramp actually happened (intermediate values on the way down).
  int rampWrites = 0;
  for (const auto& write : stub::servoOnPin(PIN_ESC_L).writes) {
    if (write.first >= silenceStartMs && write.second > SVC && write.second < 2000) rampWrites++;
  }
  CHECK(rampWrites > 0);
  // Ramp completed at neutral, then both ESC pins stopped pulsing entirely.
  CHECK(stub::servoOnPin(PIN_ESC_L).lastMicroseconds == SVC);
  CHECK(stub::servoOnPin(PIN_ESC_R).lastMicroseconds == SVC);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(outState == OUT_CUT);
  checkNeutralPulsesSince(neutralDeadlineAfterSilence(silenceStartMs));
  checkInvariants();
}

TEST_CASE("receiver failsafe bit mid-drive (#131): TX off with RX alive stops the drive") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  bfs::SbusData fullForward =
      driveFrame(RAW_SWITCH_HIGH, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(20, &fullForward, 1, true);
  REQUIRE(leftEscMicroseconds() == 2000);

  // Frames KEEP arriving, but the receiver marks them failsafe (transmitter
  // gone). Loss is detected on the very first flagged frame — no 100 ms wait.
  bfs::SbusData failsafeFrame = fullForward;
  failsafeFrame.failsafe = true;
  uint32_t failsafeStartMs = millis();
  runControlPasses(700, &failsafeFrame, 1, true);

  CHECK(stub::servoOnPin(PIN_ESC_L).lastMicroseconds == SVC);
  CHECK(stub::servoOnPin(PIN_ESC_R).lastMicroseconds == SVC);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(outState == OUT_CUT);
  checkNeutralPulsesSince(failsafeStartMs + CUTOFF_HOLD_MS + RAMP_MARGIN_MS);
  checkInvariants();
}

TEST_CASE("dead RC + railed joystick (#131): no propulsion in ANY override mode") {
  resetHarness();
  const int16_t overrideModes[] = {RAW_SWITCH_LOW, RAW_SWITCH_MID, RAW_SWITCH_HIGH};
  int modesChecked = 0;
  int joystickDroveWhileRcValid = 0;   // Modes 2 and 3 must drive BEFORE the loss
  for (int16_t overrideRaw : overrideModes) {
    resetHarness();
    setup();
    setPackVoltages(11.1f, 11.2f);
    // Rail BOTH joystick axes: full forward throttle + full steer.
    stub::setAnalogValue(PIN_JOY_Y, 16383);
    stub::setAnalogValue(PIN_JOY_X, 0);
    // Establish the override mode with valid, stick-centered RC frames (Boost).
    bfs::SbusData modeFrame = driveFrame(stub::SBUS_RAW_CENTER, stub::SBUS_RAW_CENTER,
                                         overrideRaw, RAW_SWITCH_HIGH);
    runControlPasses(30, &modeFrame, 1, true);
    if (overrideRaw != RAW_SWITCH_LOW &&
        (leftEscMicroseconds() != SVC || rightEscMicroseconds() != SVC)) {
      joystickDroveWhileRcValid++;     // scenario is meaningful: joystick had authority
    }

    uint32_t silenceStartMs = millis();
    runControlPasses(700, nullptr, 1, true);   // RC dies; joystick stays railed

    CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
    CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
    checkNeutralPulsesSince(neutralDeadlineAfterSilence(silenceStartMs));
    checkInvariants();
    modesChecked++;
  }
  CHECK(modesChecked == 3);
  CHECK(joystickDroveWhileRcValid == 2);
}

TEST_CASE("no valid RC ever + railed joystick (#131): not one non-neutral pulse from boot") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);       // battery is fine — RC absence alone blocks drive
  stub::setAnalogValue(PIN_JOY_Y, 16383);
  stub::setAnalogValue(PIN_JOY_X, 0);
  runControlPasses(700, nullptr, 1, true);   // never a single valid frame after boot
  // EVERY pulse ever recorded — including setup()'s neutral init and the
  // (degenerate, already-at-neutral) ease-out — was exactly SVC.
  checkNeutralPulsesSince(0);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(outState == OUT_CUT);
  checkInvariants();
}

TEST_CASE("RC loss during pivot (#131): counter-rotating tracks both ramp to neutral and detach") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  // Full-steer pivot: throttle centered, steering railed, Mode 1, Boost.
  // curvatureDrive counter-rotates the tracks (pivot cap 0.60 -> 1200/1800 us).
  bfs::SbusData pivotFrame =
      driveFrame(stub::SBUS_RAW_CENTER, RAW_SWITCH_HIGH, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(20, &pivotFrame, 1, true);
  REQUIRE(leftEscMicroseconds() == 1200);
  REQUIRE(rightEscMicroseconds() == 1800);
  // Tracks are genuinely counter-rotating (opposite sides of neutral).
  REQUIRE((leftEscMicroseconds() - SVC) * (rightEscMicroseconds() - SVC) < 0);

  uint32_t silenceStartMs = millis();
  runControlPasses(700, nullptr, 1, true);

  // BOTH tracks ramped to neutral — including the one that was in reverse.
  CHECK(stub::servoOnPin(PIN_ESC_L).lastMicroseconds == SVC);
  CHECK(stub::servoOnPin(PIN_ESC_R).lastMicroseconds == SVC);
  int reverseRampWrites = 0;           // left came UP from 1200 through the ramp
  for (const auto& write : stub::servoOnPin(PIN_ESC_L).writes) {
    if (write.first >= silenceStartMs && write.second > 1200 && write.second < SVC) reverseRampWrites++;
  }
  CHECK(reverseRampWrites > 0);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  checkNeutralPulsesSince(neutralDeadlineAfterSilence(silenceStartMs));
  checkInvariants();
}

TEST_CASE("recovery after RC loss (#131): frames returning re-attach and drive resumes") {
  resetHarness();
  setup();
  setPackVoltages(11.1f, 11.2f);
  bfs::SbusData fullForward =
      driveFrame(RAW_SWITCH_HIGH, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(20, &fullForward, 1, true);
  REQUIRE(leftEscMicroseconds() == 2000);

  runControlPasses(700, nullptr, 1, true);   // loss: ramp out, then cut
  REQUIRE_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  REQUIRE(stub::servoOnPin(PIN_ESC_L).attachCount == 1);
  REQUIRE(stub::servoOnPin(PIN_ESC_R).attachCount == 1);

  // The invariant forbids propulsion DURING staleness, not recovery after:
  // valid frames re-attach both ESCs (one extra attach each) and drive resumes.
  runControlPasses(20, &fullForward, 1, true);
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 2);
  CHECK(stub::servoOnPin(PIN_ESC_R).attachCount == 2);
  CHECK(leftEscMicroseconds() == 2000);
  CHECK(rightEscMicroseconds() == 2000);
  checkInvariants();
}
