// Invariant #131-4: a RISING thermal stage never increases allowed propulsion
// (monotone derating). 25 °C (none) → 85 °C (warn, audio only) → 92 °C (forced
// Eco) → 96 °C (cut: ease to neutral, detach) must produce non-increasing
// forward authority; the falling path (hysteresis release) is ALLOWED to
// restore it. A telemetry dropout HOLDS the current stage — it never releases
// a cut/eco and never trips a new stage. Thresholds: warn 80/78, eco 90/80,
// cut 95/75, trip debounce TEMP_DEBOUNCE_MS = 1000 ms, release immediate.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "InvariantChecks.h"

// Pass cadence is 1 ms. A plateau must outlast TEMP_DEBOUNCE_MS (1000 ms) for
// a stage to trip; the cut plateau additionally spans the CUTOFF_HOLD_MS
// (500 ms) ease-out so the servos are detached by the end of it.
const int SETTLE_PASSES     = 1300;
const int CUT_SETTLE_PASSES = 1900;
const int SPIKE_PASSES      = 800;   // < TEMP_DEBOUNCE_MS → must NOT trip
const int RELEASE_PASSES    = 50;    // release below OFF is immediate
const int DROPOUT_PASSES    = 5600;  // > TELEM_STALE_MS (5000 ms)

// Full authority in Boost = 1500 + 1.00*500; forced Eco = 1500 + 0.65*500.
const int FULL_FORWARD_BOOST_US = 2000;
const int FULL_FORWARD_ECO_US   = 1825;

// Full-forward throttle, centered steering, Mode 1 (RC only), Boost gear.
inline bfs::SbusData fullForwardBoostFrame() {
  return driveFrame(RAW_SWITCH_HIGH, RAW_SWITCH_MID, RAW_SWITCH_LOW,
                    RAW_SWITCH_HIGH);
}

// Boot into a healthy driving state: 12.6 V packs, cool sensors, then settle
// under the full-forward Boost frame until the output sits at the 2000 µs rail.
inline void bootHealthyFullForwardBoost(const bfs::SbusData& frame) {
  setup();
  setPackVoltages(12.6f, 12.6f);
  setAllTemperatures(25.0f);
  runControlPasses(50, &frame, 1, true);
  REQUIRE(leftEscMicroseconds() == FULL_FORWARD_BOOST_US);
  REQUIRE(rightEscMicroseconds() == FULL_FORWARD_BOOST_US);
}

TEST_CASE("thermal derating sweep (#131): 25→85→92→96 C authority is non-increasing, stage flags stack") {
  resetHarness();
  bfs::SbusData frame = fullForwardBoostFrame();
  bootHealthyFullForwardBoost(frame);
  int authorityUs[4];

  // Plateau 25 °C — no stage, full Boost authority.
  runControlPasses(SETTLE_PASSES, &frame, 1, true);
  authorityUs[0] = leftEscMicroseconds();
  CHECK(authorityUs[0] == FULL_FORWARD_BOOST_US);
  CHECK(tempWarnActive == false);
  CHECK(tempEcoActive == false);
  CHECK(tempCutActive == false);

  // Plateau 85 °C — warn only (audio); propulsion NOT derated.
  setAllTemperatures(85.0f);
  runControlPasses(SETTLE_PASSES, &frame, 1, true);
  authorityUs[1] = leftEscMicroseconds();
  CHECK(authorityUs[1] == FULL_FORWARD_BOOST_US);
  CHECK(tempWarnActive == true);
  CHECK(tempEcoActive == false);
  CHECK(tempCutActive == false);

  // Plateau 92 °C — warn holds (92 > OFF 78) AND eco trips: forced Eco gear
  // caps the Boost request to 1500 + 0.65*500.
  setAllTemperatures(92.0f);
  runControlPasses(SETTLE_PASSES, &frame, 1, true);
  authorityUs[2] = leftEscMicroseconds();
  CHECK(authorityUs[2] == FULL_FORWARD_ECO_US);
  CHECK(currentGear == GEAR_LOW);
  CHECK(tempWarnActive == true);
  CHECK(tempEcoActive == true);
  CHECK(tempCutActive == false);

  // Plateau 96 °C — all three stages; gate closes, eases to neutral, detaches.
  setAllTemperatures(96.0f);
  runControlPasses(CUT_SETTLE_PASSES, &frame, 1, true);
  authorityUs[3] = leftEscMicroseconds();
  CHECK(authorityUs[3] == SVC);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(tempWarnActive == true);
  CHECK(tempEcoActive == true);
  CHECK(tempCutActive == true);

  // The core of #131-4: authority never increased across the rising sweep.
  int increases = 0;
  for (int i = 1; i < 4; i++) {
    if (authorityUs[i] > authorityUs[i - 1]) increases++;
  }
  CHECK(increases == 0);
  checkInvariants();
}

TEST_CASE("thermal cut trip debounce (#131): a 96 C spike under 1000 ms never trips any stage") {
  resetHarness();
  bfs::SbusData frame = fullForwardBoostFrame();
  bootHealthyFullForwardBoost(frame);

  setAllTemperatures(96.0f);
  runControlPasses(SPIKE_PASSES, &frame, 1, true);   // 800 ms < debounce
  CHECK(tempCutActive == false);

  setAllTemperatures(25.0f);
  runControlPasses(400, &frame, 1, true);
  CHECK(tempCutActive == false);
  CHECK(tempEcoActive == false);
  CHECK(tempWarnActive == false);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_BOOST_US);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 1);  // gate never closed
  CHECK(stub::servoOnPin(PIN_ESC_L).detachCount == 0);
  checkInvariants();
}

TEST_CASE("thermal hysteresis release (#131): cut holds down to 75 C, then full authority returns") {
  resetHarness();
  bfs::SbusData frame = fullForwardBoostFrame();
  bootHealthyFullForwardBoost(frame);

  setAllTemperatures(96.0f);
  runControlPasses(CUT_SETTLE_PASSES, &frame, 1, true);
  REQUIRE(tempCutActive == true);
  REQUIRE_FALSE(stub::servoOnPin(PIN_ESC_L).attached);

  // 76.5 °C is below eco-OFF (80) and warn-OFF (78) but NOT below cut-OFF
  // (75): the shallower stages release, the cut HOLDS — the deepest stage
  // demands the most cooling, so authority stays at zero.
  setAllTemperatures(76.5f);
  runControlPasses(100, &frame, 1, true);
  CHECK(tempWarnActive == false);
  CHECK(tempEcoActive == false);
  CHECK(tempCutActive == true);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);

  // 74 °C < TEMP_CUT_OFF_C: release is immediate (no release debounce), the
  // gate re-opens, ESCs re-attach, and Boost authority returns — the FALLING
  // stage is allowed to increase propulsion.
  setAllTemperatures(74.0f);
  runControlPasses(RELEASE_PASSES, &frame, 1, true);
  CHECK(tempCutActive == false);
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_BOOST_US);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 2);  // setup + recovery
  checkInvariants();
}

TEST_CASE("telemetry dropout mid-cut (#131): stale telemetry HOLDS the cut; fresh cool reading releases it") {
  resetHarness();
  bfs::SbusData frame = fullForwardBoostFrame();
  bootHealthyFullForwardBoost(frame);

  setAllTemperatures(96.0f);
  runControlPasses(CUT_SETTLE_PASSES, &frame, 1, true);
  REQUIRE(tempCutActive == true);
  REQUIRE_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  uint32_t cutCompleteMs = millis();

  // Stop refreshing telemetry: the 5 s staleness watchdog clears valid, so
  // thermalUpdate has NO fresh plausible reading — every stage must hold.
  runControlPasses(DROPOUT_PASSES, &frame, 1, false);
  CHECK(telem[0].valid == false);
  CHECK(telem[1].valid == false);
  CHECK(tempCutActive == true);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 1);  // drive never returned
  checkNeutralPulsesSince(cutCompleteMs);  // nothing moved during the dropout

  // Only a FRESH plausible cool reading releases the cut and restores drive.
  setAllTemperatures(25.0f);
  runControlPasses(RELEASE_PASSES, &frame, 1, true);
  CHECK(tempCutActive == false);
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 2);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_BOOST_US);
  checkInvariants();
}

TEST_CASE("telemetry dropout mid-eco (#131): stale telemetry holds the forced-Eco derating") {
  resetHarness();
  bfs::SbusData frame = fullForwardBoostFrame();
  bootHealthyFullForwardBoost(frame);

  setAllTemperatures(92.0f);
  runControlPasses(SETTLE_PASSES, &frame, 1, true);
  REQUIRE(tempEcoActive == true);
  REQUIRE(leftEscMicroseconds() == FULL_FORWARD_ECO_US);

  // Dropout: eco holds (still derated), but driving CONTINUES — a dropout
  // never escalates to a cut on its own.
  runControlPasses(DROPOUT_PASSES, &frame, 1, false);
  CHECK(telem[0].valid == false);
  CHECK(tempEcoActive == true);
  CHECK(currentGear == GEAR_LOW);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_ECO_US);
  CHECK(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).detachCount == 0);

  // Fresh cool reading releases eco and Boost authority returns.
  setAllTemperatures(25.0f);
  runControlPasses(RELEASE_PASSES, &frame, 1, true);
  CHECK(tempEcoActive == false);
  CHECK(leftEscMicroseconds() == FULL_FORWARD_BOOST_US);
  checkInvariants();
}
