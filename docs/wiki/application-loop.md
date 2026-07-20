---
sources:
  - sketches/dual_track_control/src/application/FirmwareApp.cpp
---

# Application loop — one control cycle

What `firmwareLoop()` does each pass, in order. The order IS the safety
design; source of truth:
[FirmwareApp.cpp](../../sketches/dual_track_control/src/application/FirmwareApp.cpp).

1. **Read inputs** — S.BUS frame (one frame carries all channels, one
   freshness timeout covers them all), horn button, then the protection
   ladders: battery Eco-lock, battery cutoff, thermal stages. Ladders run
   BEFORE gear selection so a lock can force Eco.
2. **Gear + joystick** — gear select honors the Eco locks; the joystick is
   sampled on its own cadence and shaped (deadband, expo, gain).
3. **Mix** — RC and joystick combine at the axis level, then curvatureDrive
   runs once on the combined command (average-speed gear cap, pivot blend,
   desaturation). Invalid RC mixes to neutral.
4. **Safety decision + output gate** — the safety supervisor
   ([safety-system](safety-system.md)) decides drive-allowed from RC
   freshness, battery confirmation/boot grace, cutoff latch, and thermal
   cut; the output gate executes it (ACTIVE → ease-to-neutral HOLD → CUT
   detach, re-attach on recovery).
5. **Watchdog refresh — the ONLY refresh point, after the control path.**
   Anything below that stalls the loop misses the refresh, the MCU resets,
   PWM stops, and the ESCs failsafe — the runaway backstop.
6. **Observers** — X.BUS telemetry poll (read-only Read Register; it runs
   AFTER the gate and cannot affect the control output), Wi-Fi dashboard
   serving plus its periodic serving diagnostics (monitoring only), then
   the alert/beeper state machines
   ([telemetry-and-dashboard](telemetry-and-dashboard.md)).
7. **Debug output** — the CSV line, last.

Setup mirrors the same philosophy: adapters initialize first, the watchdog
arms LAST (after the slow Wi-Fi bring-up) so initialization cannot trip it.

Two ordering invariants worth naming: the ladders precede gear selection,
and nothing observable comes between the output gate and the watchdog
refresh. Both are locked by the host suites ([testing](testing.md)).
