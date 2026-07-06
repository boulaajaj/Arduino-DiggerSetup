// Invariant #131-3: batteryCutoffLatched ⇒ output gate closed ⇒ both tracks
// neutral then PWM cut, regardless of drive request (every override mode x
// gear x full-stick demand), permanently until power-cycle — recovered
// voltage never unlatches. Plus the gate-timing half of #131-6: on gate close
// the output ramps monotonically from the live command toward SVC, never
// overshoots past neutral, the LAST pulse ever sent is exactly SVC and lands
// within CUTOFF_HOLD_MS of the latch, then pulses stop entirely. Includes the
// boot-with-low-pack fault injection: packs below CUTOFF_THRESH_V from the
// first pass never produce a single non-neutral pulse (the 1.5 s cutoff
// debounce beats the 3 s boot-gate fail-open).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "InvariantChecks.h"

// Full forward in Mode 1 (RC only), Normal gear: 1500 + 0.80f * 500 = 1900 µs.
static bfs::SbusData fullForwardFrame() {
  return driveFrame(stub::SBUS_RAW_MAX, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW,
                    RAW_SWITCH_MID);
}

// Boot healthy, drive full forward to 1900 µs, then sag pack 0 to 9.5 V
// (telemetry kept fresh so validity never gates the ladder) and run 1 ms
// passes until the cutoff latches. Returns millis() of the latch pass
// (runControlPasses advances the clock 1 ms AFTER the pass, hence -1), or 0
// if it never latched. *sagStartMs receives the time of the first sagged pass.
static uint32_t driveThenSagUntilCutoffLatched(const bfs::SbusData& frame,
                                               uint32_t* sagStartMs) {
  setPackVoltages(12.6f, 12.6f);                  // healthy → batteryOkConfirmed
  runControlPasses(20, &frame, 1, true);
  REQUIRE(leftEscMicroseconds() == 1900);
  REQUIRE(rightEscMicroseconds() == 1900);
  telem[0].voltage = 9.5f;                        // worst pack collapses under load
  *sagStartMs = millis();
  for (int pass = 0; pass < 2500; pass++) {
    runControlPasses(1, &frame, 1, true);
    if (batteryCutoffLatched) return millis() - 1;
  }
  return 0;
}

TEST_CASE("cutoff latch dominates (#131-3): every override x gear x full-stick request stays neutral and detached on healthy packs") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);                  // packs HEALTHY — only the latch gates
  batteryCutoffLatched = true;                    // fault injection: latch set directly
  // Rail the joystick so Modes 2/3 carry a full joystick demand as well.
  stub::setAnalogValue(PIN_JOY_Y, 16383);
  stub::setAnalogValue(PIN_JOY_X, 0);

  const int16_t overrideModes[] = {RAW_SWITCH_LOW, RAW_SWITCH_MID, RAW_SWITCH_HIGH};
  const int16_t gearPositions[] = {RAW_SWITCH_LOW, RAW_SWITCH_MID, RAW_SWITCH_HIGH};
  const struct { int16_t throttleRaw; int16_t steeringRaw; } requests[] = {
      {stub::SBUS_RAW_MAX, stub::SBUS_RAW_CENTER},   // full forward
      {stub::SBUS_RAW_MIN, stub::SBUS_RAW_CENTER},   // full reverse
      {stub::SBUS_RAW_CENTER, stub::SBUS_RAW_MAX}};  // full pivot

  // Let the gate finish its CUTOFF_HOLD_MS neutral ramp (it closes FROM
  // neutral: no drive ever happened) — then the ESCs must be detached.
  bfs::SbusData firstFrame = driveFrame(requests[0].throttleRaw, requests[0].steeringRaw,
                                        overrideModes[0], gearPositions[0]);
  runControlPasses(60, &firstFrame, 10, true);    // 600 ms > CUTOFF_HOLD_MS
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(outState == OUT_CUT);

  int combinations = 0;
  for (int16_t overrideRaw : overrideModes) {
    for (int16_t gearRaw : gearPositions) {
      for (const auto& request : requests) {
        bfs::SbusData frame = driveFrame(request.throttleRaw, request.steeringRaw,
                                         overrideRaw, gearRaw);
        runControlPasses(25, &frame, 10, true);   // 250 ms per combination
        combinations++;
      }
    }
  }
  CHECK(combinations == 3 * 3 * 3);               // ~7.4 s total of latched healthy-pack time
  // Latch is permanent: never re-attached, never one non-neutral pulse.
  CHECK(batteryCutoffLatched);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 1);
  CHECK(stub::servoOnPin(PIN_ESC_R).attachCount == 1);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  checkNeutralPulsesSince(0);                     // gate closed from neutral ⇒ ALL pulses ever = SVC
  checkInvariants();
}

TEST_CASE("cutoff mid-drive (#131-3/#131-6): 1.5 s debounce, last pulse exactly neutral within 500 ms of the latch, then silence") {
  resetHarness();
  setup();
  bfs::SbusData frame = fullForwardFrame();
  uint32_t sagStartMs = 0;
  uint32_t latchMs = driveThenSagUntilCutoffLatched(frame, &sagStartMs);
  REQUIRE(latchMs != 0);
  // Debounce clause: the latch fires CUTOFF_DEBOUNCE_MS after the sag begins.
  CHECK(latchMs - sagStartMs >= CUTOFF_DEBOUNCE_MS);
  CHECK(latchMs - sagStartMs <= CUTOFF_DEBOUNCE_MS + 5);

  runControlPasses(600, &frame, 1, true);         // well past CUTOFF_HOLD_MS
  const int escPins[2] = {PIN_ESC_L, PIN_ESC_R};
  for (int pin : escPins) {
    const stub::ServoLog& log = stub::servoOnPin(pin);
    CHECK_FALSE(log.attached);
    REQUIRE(log.writes.size() > 0);
    // #131-6 timing clause: the LAST pulse ever recorded is exactly SVC and
    // its timestamp is within CUTOFF_HOLD_MS (+small margin) of the latch.
    CHECK(log.writes.back().second == SVC);
    CHECK(log.writes.back().first >= latchMs);
    CHECK(log.writes.back().first <= latchMs + CUTOFF_HOLD_MS + 5);
  }
  // Writes are frozen: two more seconds produce not one additional pulse.
  const size_t leftWriteCount = stub::servoOnPin(PIN_ESC_L).writes.size();
  const size_t rightWriteCount = stub::servoOnPin(PIN_ESC_R).writes.size();
  runControlPasses(2000, &frame, 1, true);
  CHECK(stub::servoOnPin(PIN_ESC_L).writes.size() == leftWriteCount);
  CHECK(stub::servoOnPin(PIN_ESC_R).writes.size() == rightWriteCount);
  CHECK(batteryCutoffLatched);
  checkNeutralPulsesSince(latchMs + CUTOFF_HOLD_MS + 5);
  checkInvariants();
}

TEST_CASE("gate ramp (#131-6): from full forward the pulses move monotonically to SVC and never overshoot past neutral") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(20, &frame, 1, true);
  REQUIRE(leftEscMicroseconds() == 1900);
  REQUIRE(rightEscMicroseconds() == 1900);

  const size_t rampStartIndexLeft = stub::servoOnPin(PIN_ESC_L).writes.size();
  const size_t rampStartIndexRight = stub::servoOnPin(PIN_ESC_R).writes.size();
  batteryCutoffLatched = true;                    // close the gate this pass
  runControlPasses(600, &frame, 1, true);         // ramp + cut complete

  const struct { int pin; size_t rampStartIndex; } tracks[] = {
      {PIN_ESC_L, rampStartIndexLeft}, {PIN_ESC_R, rampStartIndexRight}};
  for (const auto& track : tracks) {
    const stub::ServoLog& log = stub::servoOnPin(track.pin);
    REQUIRE(log.writes.size() > track.rampStartIndex);
    int previousMicroseconds = 1900;              // gate captured the live 1900 output
    int monotonicityViolations = 0;               // any step moving AWAY from SVC
    int overshootPulses = 0;                      // any pulse past SVC into reverse
    int intermediatePulses = 0;                   // proof it ramped, not snapped
    for (size_t i = track.rampStartIndex; i < log.writes.size(); i++) {
      int pulse = log.writes[i].second;
      if (pulse > previousMicroseconds) monotonicityViolations++;
      if (pulse < SVC) overshootPulses++;
      if (pulse > SVC && pulse < 1900) intermediatePulses++;
      previousMicroseconds = pulse;
    }
    CHECK(monotonicityViolations == 0);
    CHECK(overshootPulses == 0);
    CHECK(intermediatePulses > 0);
    CHECK(log.writes.back().second == SVC);
    CHECK_FALSE(log.attached);
  }
  checkInvariants();
}

TEST_CASE("boot with low pack (#131-3): 9.5 V from the first pass — no non-neutral pulse ever in the first 10 s") {
  resetHarness();
  setup();
  setPackVoltages(9.5f, 9.5f);                    // both packs below CUTOFF_THRESH_V at boot
  bfs::SbusData frame = fullForwardFrame();
  runControlPasses(1000, &frame, 10, true);       // 10 s sweep, full forward held throughout
  // The 1.5 s cutoff debounce latches BEFORE the 3 s boot-gate fail-open, and
  // the boot gate holds neutral until then — drive is never allowed at all.
  CHECK(batteryCutoffLatched);
  CHECK_FALSE(batteryOkConfirmed);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 1);
  CHECK(stub::servoOnPin(PIN_ESC_R).attachCount == 1);
  checkNeutralPulsesSince(0);                     // not one non-neutral pulse, ever
  checkInvariants();
}

TEST_CASE("recovery voltage does not unlatch (#131-3): packs back at 12.6 V for 5 s — still latched, detached, silent") {
  resetHarness();
  setup();
  bfs::SbusData frame = fullForwardFrame();
  uint32_t sagStartMs = 0;
  uint32_t latchMs = driveThenSagUntilCutoffLatched(frame, &sagStartMs);
  REQUIRE(latchMs != 0);
  runControlPasses(600, &frame, 1, true);         // ramp + cut complete
  REQUIRE_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  REQUIRE_FALSE(stub::servoOnPin(PIN_ESC_R).attached);

  setPackVoltages(12.6f, 12.6f);                  // packs "recover" (fresh charge look-alike)
  const size_t leftWriteCount = stub::servoOnPin(PIN_ESC_L).writes.size();
  const size_t rightWriteCount = stub::servoOnPin(PIN_ESC_R).writes.size();
  runControlPasses(500, &frame, 10, true);        // 5 s of healthy voltage + full stick
  CHECK(batteryCutoffLatched);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_L).attached);
  CHECK_FALSE(stub::servoOnPin(PIN_ESC_R).attached);
  CHECK(stub::servoOnPin(PIN_ESC_L).attachCount == 1);
  CHECK(stub::servoOnPin(PIN_ESC_R).attachCount == 1);
  CHECK(stub::servoOnPin(PIN_ESC_L).writes.size() == leftWriteCount);
  CHECK(stub::servoOnPin(PIN_ESC_R).writes.size() == rightWriteCount);
  checkInvariants();
}
