// Characterization: X.BUS 0x10 telemetry (V7.34 baseline).
// Request framing + checksum, response parsing (echo skip), register
// scaling, EMA weights, ESC alternation, timeout, staleness watchdog.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

// Build a well-formed 0x10 response for `esc` carrying all five registers.
static std::vector<uint8_t> responseFrame(uint8_t esc, uint16_t vbatRaw, uint16_t ibusRaw,
                                          uint16_t speedRaw, uint16_t tescRaw, uint16_t tmotRaw) {
  std::vector<uint8_t> f = {XBUS_HDR_SLAVE, esc, 0x00, XBUS_FUNC_READ, 15};
  const uint8_t regs[5] = {REG_VBAT, REG_IBUS, REG_MSPEED, REG_TESC, REG_TMOT};
  const uint16_t vals[5] = {vbatRaw, ibusRaw, speedRaw, tescRaw, tmotRaw};
  for (int i = 0; i < 5; i++) {
    f.push_back(regs[i]);
    f.push_back((uint8_t)(vals[i] & 0xFF));
    f.push_back((uint8_t)(vals[i] >> 8));
  }
  f.push_back(xbusChecksum(f.data(), (int)f.size() + 1));
  return f;
}

// Drive telemUpdate through one full request→response cycle for ESC `esc`.
static void completePoll(uint8_t esc, const std::vector<uint8_t>& response) {
  stub::advanceMillis(TELEM_POLL_MS);
  telemUpdate();                                 // sends the request
  REQUIRE(telemWaiting);
  REQUIRE(telemEsc == esc);
  Serial1.scriptRxBytes(response.data(), response.size());
  telemUpdate();                                 // drains + parses
  REQUIRE_FALSE(telemWaiting);
}

// Let the poll of ESC `esc` time out (silent ESC).
static void timeoutPoll(uint8_t esc) {
  stub::advanceMillis(TELEM_POLL_MS);
  telemUpdate();
  REQUIRE(telemWaiting);
  REQUIRE(telemEsc == esc);
  stub::advanceMicros(TELEM_TIMEOUT_US + 1);
  telemUpdate();
  REQUIRE_FALSE(telemWaiting);
}

TEST_CASE("telemSendRequest frames a 0x10 read of all five registers") {
  resetHarness();
  uint8_t stale[3] = {0xAA, 0xBB, 0xCC};
  Serial1.scriptRxBytes(stale, 3);               // stale echo on the bus

  telemSendRequest(0);
  CHECK(Serial1.rxQueue.empty());                // stale RX dropped first
  std::vector<uint8_t> expected = {0x0F, 0x00, 0x00, 0x10, 0x05,
                                   0x0C, 0x0D, 0x02, 0x20, 0x22, 0x72};
  CHECK(Serial1.txCapture == expected);          // checksum 0x72 = sum of bytes 1..9
}

TEST_CASE("first samples land unfiltered; later samples are EMA-smoothed") {
  resetHarness();
  // First response: 11.6 V, 2.5 A, -10 Hz, ESC 65-40=25 C, motor 105-40=65 C.
  completePoll(0, responseFrame(0, 116, 25, (uint16_t)(int16_t)-10, 65, 105));
  CHECK(telem[0].valid);
  CHECK(telem[0].voltage == doctest::Approx(11.6f));
  CHECK(telem[0].busCurrentA == doctest::Approx(2.5f));
  CHECK(telem[0].rpmHz == -10);
  CHECK(telem[0].escTempC == doctest::Approx(25.0f));
  CHECK(telem[0].motorTempC == doctest::Approx(65.0f));
  CHECK(telemEsc == 1);                          // alternates to the other ESC

  timeoutPoll(1);                                // ESC1 silent this round

  // Second ESC0 response: EMA weights 0.30 V / 0.50 A / 0.10 temp; RPM raw.
  completePoll(0, responseFrame(0, 126, 45, 500, 67, 105));
  CHECK(telem[0].voltage == doctest::Approx(11.6f + 0.30f * (12.6f - 11.6f)));
  CHECK(telem[0].busCurrentA == doctest::Approx(2.5f + 0.50f * (4.5f - 2.5f)));
  CHECK(telem[0].rpmHz == 500);                  // instantaneous, no smoothing
  CHECK(telem[0].escTempC == doctest::Approx(25.0f + 0.10f * (27.0f - 25.0f)));
}

TEST_CASE("parser skips the half-duplex TX echo and other ESC's frames") {
  resetHarness();
  stub::advanceMillis(TELEM_POLL_MS);
  telemUpdate();                                 // request to ESC0 in flight
  REQUIRE(telemEsc == 0);

  // The bus carries our own echo, then a frame addressed to ESC1, then ESC0's.
  std::vector<uint8_t> bus = {0x0F, 0x00, 0x00, 0x10, 0x05, 0x0C, 0x0D, 0x02, 0x20, 0x22, 0x72};
  std::vector<uint8_t> wrongEsc = responseFrame(1, 999, 0, 0, 60, 60);
  bus.insert(bus.end(), wrongEsc.begin(), wrongEsc.end());
  std::vector<uint8_t> ours = responseFrame(0, 116, 0, 0, 60, 60);
  bus.insert(bus.end(), ours.begin(), ours.end());
  Serial1.scriptRxBytes(bus.data(), bus.size());

  telemUpdate();
  CHECK(telem[0].valid);
  CHECK(telem[0].voltage == doctest::Approx(11.6f));
  CHECK_FALSE(telem[1].valid);                   // ESC1 frame was not ours to parse
}

TEST_CASE("silent ESC times out fast and polling moves on") {
  resetHarness();
  timeoutPoll(0);
  CHECK_FALSE(telem[0].valid);
  CHECK(telemEsc == 1);                          // a flaky ESC barely stalls the good one
}

TEST_CASE("per-ESC staleness watchdog invalidates after 5 s without a good read") {
  resetHarness();
  completePoll(0, responseFrame(0, 116, 25, 0, 65, 65));
  REQUIRE(telem[0].valid);

  stub::advanceMillis(TELEM_STALE_MS + 1);
  telemUpdate();                                 // no new responses
  CHECK_FALSE(telem[0].valid);
}
