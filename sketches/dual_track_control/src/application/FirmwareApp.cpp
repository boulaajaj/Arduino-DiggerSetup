// application — the control cycle (#189): setup()/loop() bodies moved
// VERBATIM from the .ino as firmwareSetup()/firmwareLoop(). The watchdog
// refresh stays exactly once per pass, after the control path (#69).
#include "FirmwareApp.h"

#include <stdio.h>

// ═══════════════════════════════════════════════════════════════
// FirmwareApp — the control cycle (bodies verbatim from setup()/loop())
// ═══════════════════════════════════════════════════════════════

void firmwareSetup() {
  joystickInitialize();    // 14-bit ADC resolution (adapter-owned, #187)
  rcInputInitialize();
  outputInit();
  beeperInit();
  alertInit();
  debugInit();
  telemetrySourceInitialize();  // X.BUS telemetry bus on D0/D1 (read-only, 0x10)
  wifiInit();             // Wi-Fi AP + telemetry server (monitoring only)

  // Arm the hardware watchdog LAST — after the slow Wi-Fi AP bring-up — so init
  // can't trip it. From here, loop() must call watchdogRefresh() within
  // WDT_TIMEOUT_MS or the MCU resets: PWM stops, the ESCs see no signal and go
  // to neutral/failsafe, and the machine stops instead of holding the last
  // throttle. This is the runaway backstop for ANY loop stall (#69).
  if (watchdogBegin(WDT_TIMEOUT_MS)) {
    if (debugConsoleReady()) {
      char line[48];
      snprintf(line, sizeof(line), "# WDT armed @ %lu ms",
               (unsigned long)watchdogTimeoutMs());
      debugConsolePrintLine(line);
    }
  } else if (debugConsoleReady()) {
    debugConsolePrintLine("# WDT FAILED to arm — no loop-stall backstop!");
  }
}

void firmwareLoop() {
  uint32_t now = clockNowUs();

  // 1. Read inputs
  if (rcInputReadFrame(&rcFrame)) {
    sbusLastFrame = now;
    sbusValid = !rcFrame.failsafe;
  }
  if ((now - sbusLastFrame) > SBUS_TIMEOUT) sbusValid = false;
  hornActive = sbusValid && (rcFrame.channels[SBUS_CH_HORN] > HORN_ON_RAW);

  // 1.5 Staged low-battery protection (#65) — evaluate BEFORE gear select so the
  // Eco lock can override it. Stage 1 (~11 V) forces Eco; Stage 2 (10 V) cuts.
  batteryEcoLockUpdate();
  batteryCutoffUpdate();
  thermalUpdate();           // motor/ESC over-temp stages (#111) — sets Eco/cut flags
  updateGear();              // honors ecoLockLatched + tempEcoActive (forces Eco when set)
  updateJoystick(now);

  // 2. Compute the drive mix (meaningful only when RC is valid).
  ServoOutput mix;
  if (sbusValid) {
    // Combine RC + joystick at the axis level (#90), then run curvatureDrive once
    // on the combined command so a single operator keeps full range.
    DriveCommand cmd = mixCommands(rcCommand(), rcOverride(), cachedJoyCmd);
    // Gear-selected pivot cap: Eco keeps the looser cap for maneuvering
    // authority. Read here so curvatureDrive stays a pure function (#119).
    float pivotCap = (currentGear == GEAR_LOW) ? PIVOT_SPEED_CAP_LOW
                                               : PIVOT_SPEED_CAP;
    mix = wheelSpeedsToServo(
        curvatureDrive(cmd.xSpeed, cmd.zRotation, gearScale, pivotCap));
  } else {
    mix.left = SVC;  mix.right = SVC;
  }

  // 3. Fail-safe output gate (#88 / #65) — drive ONLY when RC is valid AND the
  // battery is above the cutoff (latched in step 1.5). Otherwise command neutral
  // (GL10 decelerates smoothly) then cut PWM so the ESCs lose signal and beep.
  // RC-loss recovers; the battery cutoff latches until power-cycle. GL10's
  // internal accel/drag is the only command smoothing (no Arduino-side ramp).
  // Boot gate (#65): don't drive until a valid reading confirms the pack is above
  // the cutoff — so a low pack can't drive in the brief window after a watchdog
  // reset wipes the RAM latch. Fail OPEN if telemetry never reports
  // (BATTERY_CONFIRM_MS) so a dead X.BUS can't permanently disable driving.
  // The decision is extracted to src/domain/safety/SafetySupervisor.* (#117
  // step 8, #168); this call site owns the boundary reads (RC freshness,
  // battery flags, the clockNowMs()-based boot grace) and today consumes only
  // driveAllowed — the FailsafeReason feeds the OutputGate and
  // SystemSnapshot steps. The thermal cut (#111) is NOT latched: when the
  // motor cools below TEMP_CUT_OFF_C the gate re-opens and outputUpdate()
  // re-attaches the ESCs automatically.
  SafetyDecision safety = safetySupervisorDecide(SafetyInputs{
      sbusValid, batteryOkConfirmed,
      clockNowMs() - alertBootMs > BATTERY_CONFIRM_MS, batteryCutoffLatched,
      tempCutActive});
  outputUpdate(safety.driveAllowed, mix.left, mix.right);

  // Control path serviced this pass (inputs read + output gate run) — and ONLY
  // now do we kick the watchdog. This is the sole refresh point: if anything
  // below (telemetry/Wi-Fi) ever stalls the loop beyond WDT_TIMEOUT_MS, the
  // refresh is missed, the MCU resets, and the machine stops instead of holding
  // the last throttle command (#69).
  watchdogRefresh();

  // 3.5 Telemetry — non-blocking X.BUS Read Register (0x10), read-only.
  // Never enters BUS_MODE, so it cannot affect the control output above.
  telemUpdate();

  // 3.6 Wi-Fi — serve telemetry JSON to the dashboard (monitoring only).
  wifiUpdate();
  wifiDebug(now);
  alertUpdate(sbusValid);  // battery + inactivity alarms → alarmOutputOn (audio only)
  beeperUpdate();          // horn (RC SWD) + queued pattern + [ALERT] alarm (D8)

  // 4. Debug
  debugPrint(now);
}
