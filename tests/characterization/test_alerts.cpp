// Characterization: beeper + alarm system (#51/#68/#111, V7.34 baseline).
// Horn OR one-shot patterns OR repeating alarms on D8; alarm priority
// ladder; inactivity 60 s; low-V 10.5 V debounced/latched with 60 s
// startup grace.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "FirmwareUnderTest.h"

static int beeperPin() { return stub::digitalLevels[PIN_BEEPER]; }

TEST_CASE("horn ORs over everything on D8") {
  resetHarness();
  beeperInit();
  hornActive = true;
  beeperUpdate();
  CHECK(beeperPin() == HIGH);
  hornActive = false;
  beeperUpdate();
  CHECK(beeperPin() == LOW);
}

TEST_CASE("one-shot pattern: Wi-Fi ready beep-beep plays through and stops") {
  resetHarness();
  beeperInit();
  beepStart(BEEP_WIFI_READY, BEEP_WIFI_READY_LEN);   // 180 on / 140 off / 180 on
  beeperUpdate();
  CHECK(beeperPin() == HIGH);                    // phase 0 = ON
  stub::advanceMillis(180);
  beeperUpdate();
  CHECK(beeperPin() == LOW);                     // phase 1 = OFF
  stub::advanceMillis(140);
  beeperUpdate();
  CHECK(beeperPin() == HIGH);                    // phase 2 = ON
  stub::advanceMillis(180);
  beeperUpdate();
  CHECK(beeperPin() == LOW);                     // pattern exhausted → silent
  stub::advanceMillis(5000);
  beeperUpdate();
  CHECK(beeperPin() == LOW);
}

TEST_CASE("inactivity alarm after 60 s of RC off; clears when RC returns") {
  resetHarness();
  alertInit();
  alertUpdate(false);                            // RC off from now
  stub::advanceMillis(59999);
  alertUpdate(false);
  CHECK(alarmSeq == nullptr);                    // 1 ms short of the threshold

  stub::advanceMillis(1);
  alertUpdate(false);
  CHECK(alarmSeq == ALERT_INACT);
  CHECK(alarmOutputOn);                          // pattern starts in its ON phase

  // 500 ms ON / 1500 ms OFF cycle.
  stub::advanceMillis(500);
  alertUpdate(false);
  CHECK_FALSE(alarmOutputOn);
  stub::advanceMillis(1500);
  alertUpdate(false);
  CHECK(alarmOutputOn);

  alertUpdate(true);                             // transmitter back on
  CHECK(alarmSeq == nullptr);
  CHECK_FALSE(alarmOutputOn);
}

TEST_CASE("low-V alarm: suppressed for the first 60 s, then debounced 3 s, then latched") {
  resetHarness();
  alertInit();                                   // startup grace reference = now
  setPackVoltages(10.0f, 11.0f);                 // worst below 10.5 V

  stub::advanceMillis(30000);                    // inside the startup grace
  alertUpdate(true);
  CHECK_FALSE(lowVoltLatched);

  stub::advanceMillis(30000);                    // grace over — debounce starts
  setPackVoltages(10.0f, 11.0f);                 // keep readings fresh
  alertUpdate(true);
  CHECK_FALSE(lowVoltLatched);

  stub::advanceMillis(3000);                     // 3 s sustained → latch
  alertUpdate(true);
  CHECK(lowVoltLatched);
  CHECK(alarmSeq == ALERT_LOWV);

  setPackVoltages(12.6f, 12.6f);                 // latched until power cycle
  alertUpdate(true);
  CHECK(lowVoltLatched);
  CHECK(alarmSeq == ALERT_LOWV);
}

TEST_CASE("low-V debounce needs BOTH packs valid and plausible") {
  resetHarness();
  alertInit();
  stub::advanceMillis(60000);                    // clear the startup grace

  setPackVoltages(10.0f, 11.0f);
  alertUpdate(true);                             // debounce running
  stub::advanceMillis(2000);
  telem[1].valid = false;                        // pack drops out at 2 s
  alertUpdate(true);

  setPackVoltages(10.0f, 11.0f);                 // back — needs a fresh 3 s
  alertUpdate(true);
  stub::advanceMillis(2999);
  alertUpdate(true);
  CHECK_FALSE(lowVoltLatched);
}

TEST_CASE("alarm priority: thermal cut > low-V > thermal warn > inactivity") {
  resetHarness();
  alertInit();
  // Make every alarm want to sound at once.
  lowVoltLatched = true;
  tempWarnActive = true;
  tempCutActive = true;
  alertUpdate(false);
  stub::advanceMillis(60001);
  alertUpdate(false);                            // inactivity now active too
  CHECK(alarmSeq == ALERT_THERM_CUT);

  tempCutActive = false;
  alertUpdate(false);
  CHECK(alarmSeq == ALERT_LOWV);

  lowVoltLatched = false;
  alertUpdate(false);
  CHECK(alarmSeq == ALERT_THERM_WARN);

  tempWarnActive = false;
  alertUpdate(false);
  CHECK(alarmSeq == ALERT_INACT);
}
