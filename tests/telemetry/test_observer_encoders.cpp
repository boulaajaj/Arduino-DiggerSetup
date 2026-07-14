// Pure suite (#132): the observer encoders tested directly on the host — no
// firmware, no stubs. Locks the EXACT dashboard JSON and debug CSV bytes the
// firmware emitted before the extraction (formats moved verbatim from
// buildTelemJson()/debugPrint()); a changed byte here is a behavior change
// and needs its own issue.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstring>
#include <string>

#include "../../sketches/dual_track_control/src/application/SystemSnapshot.h"
#include "../../sketches/dual_track_control/src/telemetry/CsvEncoder.h"
#include "../../sketches/dual_track_control/src/telemetry/JsonEncoder.h"

// A fully-populated snapshot with distinct, hand-checkable values.
static SystemSnapshot exampleSnapshot() {
  SystemSnapshot s = {};
  s.nowMs = 1000;
  s.rcThrottleUs = 1500;
  s.rcSteeringUs = 1600;
  s.rcGearUs = 1200;
  s.rcOverrideUs = 1000;
  s.rcFailsafe = false;
  s.rcLostFrame = true;
  s.joystickRawY = 8192;
  s.joystickRawX = 4096;
  s.outLeftUs = 1500;
  s.outRightUs = 1735;
  s.gear = 2;
  s.overrideMode = 1;
  s.batteryCutoffLatched = false;
  s.ecoLockLatched = true;
  s.temperatureWarnActive = true;
  s.temperatureEcoActive = false;
  s.temperatureCutActive = false;
  s.esc[0].voltage = 12.6f;       // → v 126 (deci-volts)
  s.esc[0].busCurrentA = 2.5f;    // → cur 25 (deci-amps)
  s.esc[0].rpmHz = 100;           // → rpm 3000 (×30)
  s.esc[0].escTempC = 25.4f;      // → tE 25
  s.esc[0].motorTempC = 65.5f;    // → tM 66 (lroundf half away from zero)
  s.esc[0].lastGoodMs = 900;      // → age 100 at nowMs 1000
  s.esc[0].valid = true;
  // esc[1] left zeroed: never received → age 999999, ok 0.
  return s;
}

TEST_CASE("JSON frame: exact bytes of the pre-extraction buildTelemJson format") {
  char body[400];
  int n = encodeTelemetryJson(body, sizeof(body), exampleSnapshot(), 7);
  const std::string expected =
      "{\"t\":1000,\"seq\":7,\"gear\":2,\"mode\":1,\"fs\":0,\"lost\":1,"
      "\"eco\":1,\"cut\":0,\"hot\":1,\"teco\":0,\"tcut\":0,\"outL\":1500,\"outR\":1735,"
      "\"e0\":{\"ok\":1,\"age\":100,\"rpm\":3000,\"cur\":25,\"v\":126,\"tE\":25,\"tM\":66},"
      "\"e1\":{\"ok\":0,\"age\":999999,\"rpm\":0,\"cur\":0,\"v\":0,\"tE\":0,\"tM\":0}}";
  CHECK(std::string(body) == expected);
  CHECK(n == (int)expected.size());
}

TEST_CASE("JSON frame: overflow clamps the returned length to the buffer") {
  char body[64];  // far too small for a full frame
  int n = encodeTelemetryJson(body, sizeof(body), exampleSnapshot(), 7);
  CHECK(n == (int)sizeof(body) - 1);           // clamped, never past the buffer
  CHECK(std::strlen(body) == sizeof(body) - 1);  // snprintf NUL-terminated it
}

TEST_CASE("CSV line: exact bytes of the pre-extraction debugPrint format") {
  char line[200];
  encodeDebugCsv(line, sizeof(line), exampleSnapshot());
  CHECK(std::string(line) ==
        "1500,1600,1200,1000,8192,4096,1500,1735,2,0,1,"
        "126,25,100,25,66,1,0,0,0,0,0,0");
}
