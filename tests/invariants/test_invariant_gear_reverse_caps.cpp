// Invariant #131-7: no RC + joystick combination pushes the AVERAGE track
// command past the active gear's caps — forward average <= gearScale (Eco 0.65
// / Normal 0.80 / Boost 1.00), reverse average >= -reverseCap() (0.55
// Eco+Normal, 0.65 Boost, #113). The caps bound the AVERAGE of the two tracks
// ONLY: a single track legitimately exceeds them in turns and pivots (turn
// headroom #72), so nothing here asserts a per-track limit. Sampled at steady
// state (several loop passes spanning one full 10 ms joystick cache window);
// the one known transient gap is encoded at the bottom with a #131 FINDING.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "InvariantChecks.h"

#include <cmath>

// Per-gear average-µs bounds. The +1/-1 µs epsilon absorbs the int truncation
// in wheelSpeedsToServo (truncation biases toward neutral, so it is generous).
inline float forwardCeilingMicroseconds(Gear gear) {
  float scale = (gear == GEAR_LOW)  ? GEAR_LOW_SCALE
              : (gear == GEAR_HIGH) ? GEAR_HIGH_SCALE : GEAR_MID_SCALE;
  return (float)SVC + scale * SOFT_RANGE + 1.0f;
}

inline float reverseFloorMicroseconds(Gear gear) {
  float cap = (gear == GEAR_HIGH) ? REVERSE_CAP_BOOST : REVERSE_CAP_STD;
  return (float)SVC - cap * SOFT_RANGE - 1.0f;
}

// Count pulse-PAIR averages recorded at or after sinceMs that fall outside
// [floorMicroseconds, ceilingMicroseconds]. The left/right write logs stay 1:1
// paired (outputInit and outputWrite always write both pins together).
inline int pairAveragesOutside(uint32_t sinceMs, float floorMicroseconds,
                               float ceilingMicroseconds) {
  const auto& leftWrites  = stub::servoOnPin(PIN_ESC_L).writes;
  const auto& rightWrites = stub::servoOnPin(PIN_ESC_R).writes;
  REQUIRE(leftWrites.size() == rightWrites.size());
  int outside = 0;
  for (size_t writeIndex = 0; writeIndex < leftWrites.size(); writeIndex++) {
    if (leftWrites[writeIndex].first < sinceMs) continue;
    float pairAverage =
        (leftWrites[writeIndex].second + rightWrites[writeIndex].second) / 2.0f;
    if (pairAverage < floorMicroseconds || pairAverage > ceilingMicroseconds) outside++;
  }
  return outside;
}

TEST_CASE("steady-state sweep (#131): gear x override x RC sticks x joystick never exceeds the average caps") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  const struct { int16_t raw; Gear expected; } gearGrid[] = {
      {RAW_SWITCH_LOW, GEAR_LOW}, {RAW_SWITCH_MID, GEAR_MID}, {RAW_SWITCH_HIGH, GEAR_HIGH}};
  const int16_t overrideRaws[] = {RAW_SWITCH_LOW, RAW_SWITCH_MID, RAW_SWITCH_HIGH};
  const int16_t throttleRaws[] = {172, 582, 992, 1401, 1811};
  const int16_t steeringRaws[] = {172, 992, 1811};
  const struct { int throttleAdc; int steeringAdc; } joystickGrid[] = {
      {0, 8192}, {16383, 8192}, {16383, 0}, {8192, 8192}, {0, 16383}};
  int combinations = 0;
  int violations = 0;
  for (const auto& gear : gearGrid) {
    for (int16_t overrideRaw : overrideRaws) {
      for (int16_t throttleRaw : throttleRaws) {
        for (int16_t steeringRaw : steeringRaws) {
          for (const auto& joystick : joystickGrid) {
            stub::setAnalogValue(PIN_JOY_Y, joystick.throttleAdc);
            stub::setAnalogValue(PIN_JOY_X, joystick.steeringAdc);
            bfs::SbusData frame =
                driveFrame(throttleRaw, steeringRaw, overrideRaw, gear.raw);
            // 8 passes x 2 ms spans the 10 ms joystick cache window, so the
            // sampled average is steady state for THIS combination.
            runControlPasses(8, &frame, 2, true);
            float average = averageTrackMicroseconds();
            if (currentGear != gear.expected ||
                average > forwardCeilingMicroseconds(gear.expected) ||
                average < reverseFloorMicroseconds(gear.expected)) {
              violations++;
            }
            combinations++;
          }
        }
      }
    }
  }
  CHECK(violations == 0);
  CHECK(combinations == 3 * 3 * 5 * 3 * 5);
  checkInvariants();
}

TEST_CASE("forward cap attained (#131): Boost straight line reaches the 2000 rail, Eco reaches ~1825") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  bfs::SbusData boostForward = driveFrame(
      1811, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(20, &boostForward, 2, true);
  float boostAverage = averageTrackMicroseconds();
  CHECK(boostAverage >= 1998.0f);   // the rail is attainable, not just a bound
  CHECK(boostAverage <= 2001.0f);

  bfs::SbusData ecoForward = driveFrame(
      1811, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_LOW);
  runControlPasses(20, &ecoForward, 2, true);
  float ecoAverage = averageTrackMicroseconds();
  CHECK(ecoAverage >= 1823.0f);     // 1825 - 2 (int truncation lands on 1824)
  CHECK(ecoAverage <= 1827.0f);
  checkInvariants();
}

TEST_CASE("reverse floor attained, never exceeded (#131): 1225 Eco/Normal, 1175 Boost across full history") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  const struct { int16_t gearRaw; float floorMicroseconds; } gearGrid[] = {
      {RAW_SWITCH_LOW,  1225.0f},   // Eco:    1500 - 0.55*500
      {RAW_SWITCH_MID,  1225.0f},   // Normal: same reverse cap (#113)
      {RAW_SWITCH_HIGH, 1175.0f}};  // Boost:  1500 - 0.65*500
  for (const auto& gear : gearGrid) {
    uint32_t segmentStartMs = millis();
    bfs::SbusData fullReverse =
        driveFrame(172, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, gear.gearRaw);
    runControlPasses(20, &fullReverse, 2, true);
    float average = averageTrackMicroseconds();
    CHECK(average >= gear.floorMicroseconds - 2.0f);  // floor attained...
    CHECK(average <= gear.floorMicroseconds + 2.0f);
    // ...and no pulse pair anywhere in this segment went below the floor.
    CHECK(pairAveragesOutside(segmentStartMs, gear.floorMicroseconds - 1.0f,
                              (float)SVMAX + 1.0f) == 0);
  }
  checkInvariants();
}

TEST_CASE("Eco-lock dominance (#131): battery Eco lock beats a Boost request — average capped at Eco") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  ecoLockLatched = true;   // battery Eco lock, as latched after 15 s below 10.8 V
  bfs::SbusData boostForward = driveFrame(
      1811, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(30, &boostForward, 2, true);
  CHECK(currentGear == GEAR_LOW);
  float average = averageTrackMicroseconds();
  CHECK(average >= 1823.0f);                                // Eco ceiling attained
  CHECK(average <= forwardCeilingMicroseconds(GEAR_LOW));   // never exceeded
  CHECK(pairAveragesOutside(0, (float)SVMIN, forwardCeilingMicroseconds(GEAR_LOW)) == 0);
  checkInvariants();
}

TEST_CASE("thermal Eco dominance (#131): tempEcoActive holds at 85 C (hysteresis) and caps the average at Eco") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  // 85 C sits inside the hysteresis band: >= TEMP_ECO_OFF_C (80) so an
  // already-set stage HOLDS, < TEMP_ECO_ON_C (90) so it cannot re-trip on its
  // own. Setting the flag directly models "tripped past 90 C earlier, now
  // cooling through 85 C". thermalUpdate() runs every pass and must NOT
  // release the stage at this temperature.
  setAllTemperatures(85.0f);
  tempEcoActive = true;
  bfs::SbusData boostForward = driveFrame(
      1811, stub::SBUS_RAW_CENTER, RAW_SWITCH_LOW, RAW_SWITCH_HIGH);
  runControlPasses(30, &boostForward, 2, true);
  CHECK(tempEcoActive);                    // hysteresis held the stage all run
  CHECK(currentGear == GEAR_LOW);
  float average = averageTrackMicroseconds();
  CHECK(average >= 1823.0f);
  CHECK(average <= forwardCeilingMicroseconds(GEAR_LOW));
  CHECK(pairAveragesOutside(0, (float)SVMIN, forwardCeilingMicroseconds(GEAR_LOW)) == 0);
  checkInvariants();
}

TEST_CASE("full-steer pivot (#131): counter-rotation cancels — average neutral inside caps, tracks counter-rotate") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  const struct { int16_t raw; Gear expected; } gearGrid[] = {
      {RAW_SWITCH_LOW, GEAR_LOW}, {RAW_SWITCH_MID, GEAR_MID}, {RAW_SWITCH_HIGH, GEAR_HIGH}};
  const int16_t steeringRaws[] = {172, 1811};   // full left / full right lock
  int combinations = 0;
  int violations = 0;
  for (const auto& gear : gearGrid) {
    for (int16_t steeringRaw : steeringRaws) {
      bfs::SbusData pivotFrame = driveFrame(
          stub::SBUS_RAW_CENTER, steeringRaw, RAW_SWITCH_LOW, gear.raw);
      runControlPasses(10, &pivotFrame, 2, true);
      float average = averageTrackMicroseconds();
      bool insideCaps = average <= forwardCeilingMicroseconds(gear.expected) &&
                        average >= reverseFloorMicroseconds(gear.expected);
      bool neutralAverage = fabsf(average - (float)SVC) <= 1.0f;
      bool counterRotating = leftEscMicroseconds() != rightEscMicroseconds();
      if (!insideCaps || !neutralAverage || !counterRotating) violations++;
      combinations++;
    }
  }
  CHECK(violations == 0);
  CHECK(combinations == 3 * 2);
  checkInvariants();
}

TEST_CASE("gear shift with cached joystick (#131): one cache window can exceed the new gear's reverse cap") {
  resetHarness();
  setup();
  setPackVoltages(12.6f, 12.6f);
  // Mode 2 with RC sticks centered → the joystick drives. Full reverse in Eco:
  // updateJoystick clamps xSpeed to -0.55/0.65 = -0.846 (Eco-domain clamp).
  stub::setAnalogValue(PIN_JOY_Y, 0);
  bfs::SbusData ecoFrame = driveFrame(stub::SBUS_RAW_CENTER, stub::SBUS_RAW_CENTER,
                                      RAW_SWITCH_MID, RAW_SWITCH_LOW);
  runControlPasses(12, &ecoFrame, 2, true);
  REQUIRE(averageTrackMicroseconds() == doctest::Approx(1225.0f).epsilon(0.002f));
  // Land on the pass right AFTER a joystick cache refresh, so the next pass is
  // guaranteed not to re-clamp (the 100 Hz cache window has ~8 ms left).
  bool refreshObserved = false;
  for (int i = 0; i < 8 && !refreshObserved; i++) {
    uint32_t previousAdcTime = lastAdcTime;
    runControlPasses(1, &ecoFrame, 2, true);
    refreshObserved = (lastAdcTime != previousAdcTime);
  }
  REQUIRE(refreshObserved);
  // Operator flips CH4 Eco → Boost while holding full reverse joystick. This
  // pass mixes the STALE Eco-domain clamp (-0.846 xSpeed) with Boost gearScale.
  bfs::SbusData boostFrame = driveFrame(stub::SBUS_RAW_CENTER, stub::SBUS_RAW_CENTER,
                                        RAW_SWITCH_MID, RAW_SWITCH_HIGH);
  runControlPasses(1, &boostFrame, 2, true);
  float transientAverage = averageTrackMicroseconds();
  // #131 FINDING: for up to one 10 ms joystick cache window after an
  // Eco/Normal → Boost gear shift with full reverse joystick held, the average
  // track command exceeds the Boost reverse cap: -0.846 average ≈ 1077 µs vs
  // the 1175 µs floor. rcCommand() re-clamps every pass, but cachedJoyCmd
  // keeps the OLD gear's reverse clamp until the 100 Hz cache refreshes.
  // Encoding ACTUAL behavior — not weakening the steady-state invariant above.
  CHECK(transientAverage == doctest::Approx(1077.0f).epsilon(0.002f));
  CHECK(transientAverage < reverseFloorMicroseconds(GEAR_HIGH));
  // One cache window later the Boost clamp applies and the floor holds again.
  runControlPasses(10, &boostFrame, 2, true);
  CHECK(averageTrackMicroseconds() >= reverseFloorMicroseconds(GEAR_HIGH));
  checkInvariants();
}
