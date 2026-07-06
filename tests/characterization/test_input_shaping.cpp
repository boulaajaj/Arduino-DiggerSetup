// Characterization: RC + joystick input shaping (V7.34 baseline).
// sbusToServo mapping, deadbands, expo curves, rcCommand gains + reverse
// clamp, updateJoystick ADC→command pipeline (#90).
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

TEST_CASE("sbusToServo maps the S.BUS range onto 1000-2000 us") {
  resetHarness();
  CHECK(sbusToServo(SBUS_MIN) == 1000);          // 172 → full reverse
  CHECK(sbusToServo(SBUS_MAX) == 2000);          // 1811 → full forward
  CHECK(sbusToServo(992) == 1500);               // transmitter center → neutral
  CHECK(sbusToServo(0) == 1000);                 // below range clamps
  CHECK(sbusToServo(2100) == 2000);              // above range clamps
  // Integer map() truncates toward zero — lock in two mid points.
  CHECK(sbusToServo(500) == 1200);               // (500-172)*1000/1639 = 200.1 → 200
  CHECK(sbusToServo(1400) == 1749);              // (1400-172)*1000/1639 = 749.2 → 749
}

TEST_CASE("rcDeadband snaps ±50 us around center to exactly center") {
  resetHarness();
  CHECK(rcDeadband(1500) == 1500);
  CHECK(rcDeadband(1550) == 1500);               // boundary inside
  CHECK(rcDeadband(1551) == 1551);               // first value outside
  CHECK(rcDeadband(1450) == 1500);
  CHECK(rcDeadband(1449) == 1449);
}

TEST_CASE("joyDeadband snaps ±480 ADC counts around 8192 to exactly center") {
  resetHarness();
  CHECK(joyDeadband(8192) == 8192);
  CHECK(joyDeadband(8192 + 480) == 8192);
  CHECK(joyDeadband(8192 + 481) == 8192 + 481);
  CHECK(joyDeadband(8192 - 480) == 8192);
  CHECK(joyDeadband(8192 - 481) == 8192 - 481);
}

TEST_CASE("expoCurve blends linear and cubic on the magnitude") {
  resetHarness();
  // Throttle curve 0.4 linear + 0.6 cubic.
  CHECK(expoCurve(1.0f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC) == doctest::Approx(1.0f));
  CHECK(expoCurve(0.5f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC) == doctest::Approx(0.275f));
  CHECK(expoCurve(0.0f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC) == doctest::Approx(0.0f));
  // Steering curve 0.7 linear + 0.3 cubic — more authority at partial deflection.
  CHECK(expoCurve(0.5f, EXPO_STEER_LINEAR, EXPO_STEER_CUBIC) == doctest::Approx(0.3875f));
  // expoCurve works on |x| — the caller reapplies the sign.
  CHECK(expoCurve(-0.5f, EXPO_THROTTLE_LINEAR, EXPO_THROTTLE_CUBIC) == doctest::Approx(0.275f));
}

TEST_CASE("rc axis accessors return neutral when S.BUS is invalid") {
  resetHarness();
  REQUIRE(sbusValid == false);
  CHECK(rcThrottle() == SVC);
  CHECK(rcSteering() == SVC);
  CHECK(rcOverride() == SVMIN);                  // invalid → Mode 1 (RC only)
}

TEST_CASE("rcCommand: unity gains, full forward authority, per-gear reverse clamp") {
  resetHarness();
  bfs::SbusData frame = stub::centeredSbusFrame();
  frame.ch[SBUS_CH_THR] = SBUS_MAX;              // full forward
  feedSbusFrame(frame);
  REQUIRE(sbusValid);
  CHECK(rcCommand().xSpeed == doctest::Approx(1.0f));

  // Full reverse in Eco (gearScale 0.65): clamped to the 55% track-output
  // reverse cap → xSpeed = -0.55 / 0.65.
  frame.ch[SBUS_CH_THR] = SBUS_MIN;
  feedSbusFrame(frame);
  updateGear();                                  // centered CH4 (1500) → Normal gear
  CHECK(currentGear == GEAR_MID);
  CHECK(rcCommand().xSpeed == doctest::Approx(-REVERSE_CAP_STD / GEAR_MID_SCALE));

  // Boost allows 65% reverse.
  frame.ch[SBUS_CH_GEAR] = SBUS_MAX;
  feedSbusFrame(frame);
  updateGear();
  REQUIRE(currentGear == GEAR_HIGH);
  CHECK(rcCommand().xSpeed == doctest::Approx(-REVERSE_CAP_BOOST / GEAR_HIGH_SCALE));

  // Steering passes through at unity gain.
  frame.ch[SBUS_CH_STEER] = SBUS_MAX;
  feedSbusFrame(frame);
  CHECK(rcCommand().zRotation == doctest::Approx(1.0f));
}

TEST_CASE("updateJoystick: center stays neutral, ADC is cached at 100 Hz") {
  resetHarness();
  updateGear();                                  // sbus invalid → Eco failsafe
  stub::setAnalogValue(PIN_JOY_Y, ADC_CENTER);
  stub::setAnalogValue(PIN_JOY_X, ADC_CENTER);
  updateJoystick(micros());
  CHECK(cachedJoyCmd.xSpeed == doctest::Approx(0.0f));
  CHECK(cachedJoyCmd.zRotation == doctest::Approx(0.0f));

  // A new value inside the 10 ms cache window is NOT picked up...
  stub::setAnalogValue(PIN_JOY_Y, 16383);
  stub::advanceMillis(5);
  updateJoystick(micros());
  CHECK(cachedJoy.rawY == ADC_CENTER);
  // ...but is after the window elapses.
  stub::advanceMillis(5);
  updateJoystick(micros());
  CHECK(cachedJoy.rawY == 16383);
}

TEST_CASE("updateJoystick: full forward deflection hits the per-gear cap (#90)") {
  resetHarness();
  updateGear();                                  // Eco (S.BUS invalid failsafe)
  REQUIRE(currentGear == GEAR_LOW);
  stub::setAnalogValue(PIN_JOY_Y, 16383);        // full forward
  stub::setAnalogValue(PIN_JOY_X, ADC_CENTER);
  updateJoystick(micros());
  // Raw full deflection: norm ≈ 1.0 → expo ≈ 1.0 → ×1.40 gain = 1.40, then
  // clamped to the Eco forward cap in the xSpeed domain: 0.65 / 0.65 = 1.0.
  CHECK(cachedJoyCmd.xSpeed == doctest::Approx(JOY_CAP_ECO / GEAR_LOW_SCALE).epsilon(0.001));

  // Full reverse clamps to reverseCap(): 0.55 / 0.65.
  stub::setAnalogValue(PIN_JOY_Y, 0);
  stub::advanceMillis(10);
  updateJoystick(micros());
  CHECK(cachedJoyCmd.xSpeed == doctest::Approx(-REVERSE_CAP_STD / GEAR_LOW_SCALE).epsilon(0.001));
}

TEST_CASE("updateJoystick: steering polarity is flipped (JOY_STEER_DIR = -1)") {
  resetHarness();
  updateGear();
  stub::setAnalogValue(PIN_JOY_Y, ADC_CENTER);
  stub::setAnalogValue(PIN_JOY_X, 16383);        // stick full right (raw)
  updateJoystick(micros());
  CHECK(cachedJoyCmd.zRotation < 0.0f);          // flipped by the rewired-cable fix
  CHECK(cachedJoyCmd.zRotation == doctest::Approx(-expoCurve(1.0f, EXPO_STEER_LINEAR,
                                                             EXPO_STEER_CUBIC)).epsilon(0.001));
}
