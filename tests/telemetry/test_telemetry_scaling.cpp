// Pure suite (#117 step 4, #160): telemetry scaling tested directly on the
// host — no firmware, no stubs. The X.BUS register DECODE + EMA fold live
// in infrastructure/xc/XbusTelemetryAdapter.h (host-pure header, #178); the
// ×10 wire ENCODE stays in src/telemetry/TelemetryScaling.h. End-to-end
// behavior through the poller/buildTelemJson() stays covered by
// tests/characterization/test_telemetry_parse.cpp; this suite locks the
// scaling contracts at the unit level: register signedness, the low-byte
// temperature mask and −40 offset, EMA seeding, and the exact integer
// rounding the wire format carries.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "../../sketches/dual_track_control/src/infrastructure/xc/XbusTelemetryAdapter.h"
#include "../../sketches/dual_track_control/src/telemetry/TelemetryScaling.h"

TEST_CASE("VBAT decode: unsigned tenths of a volt") {
  CHECK(vbatRawToVolts(126) == doctest::Approx(12.6f));
  CHECK(vbatRawToVolts(0) == doctest::Approx(0.0f));
}

TEST_CASE("IBUS decode is SIGNED: regenerative current reads negative") {
  CHECK(busCurrentRawToAmps(105) == doctest::Approx(10.5f));
  CHECK(busCurrentRawToAmps((uint16_t)0xFFF6) == doctest::Approx(-1.0f));
}

TEST_CASE("motor speed decode is SIGNED electrical Hz") {
  CHECK(motorSpeedRawToElectricalHz(200) == 200);
  CHECK(motorSpeedRawToElectricalHz((uint16_t)0xFF9C) == -100);
}

TEST_CASE("temperature decode: low byte only, raw minus 40") {
  CHECK(temperatureRawToCelsius(65) == doctest::Approx(25.0f));
  CHECK(temperatureRawToCelsius(40) == doctest::Approx(0.0f));
  // A dirty high byte must not leak into the temperature.
  CHECK(temperatureRawToCelsius(0xAB41) == doctest::Approx(25.0f));
}

TEST_CASE("EMA fold: first sample seeds; later samples blend by alpha") {
  CHECK(emaFold(999.0f, 12.6f, 0.2f, true) == doctest::Approx(12.6f));
  CHECK(emaFold(10.0f, 20.0f, 0.25f, false) == doctest::Approx(12.5f));
  CHECK(emaFold(12.5f, 12.5f, 0.25f, false) == doctest::Approx(12.5f));
}

TEST_CASE("wire encode: deci-integers round half away from zero (lroundf)") {
  CHECK(scaleToDeciInteger(12.6f) == 126);
  CHECK(scaleToDeciInteger(12.25f) == 123);   // 122.5 exactly → away from zero
  CHECK(scaleToDeciInteger(-1.25f) == -13);   // -12.5 exactly → away from zero
  CHECK(scaleToDeciInteger(0.0f) == 0);
}

TEST_CASE("wire encode: whole-degree rounding and Hz-to-RPM factor") {
  CHECK(roundToWholeInteger(84.5f) == 85);
  CHECK(roundToWholeInteger(-0.5f) == -1);
  CHECK(electricalHzToDashboardRpm(100) == 3000L);
  CHECK(electricalHzToDashboardRpm(-50) == -1500L);
}
