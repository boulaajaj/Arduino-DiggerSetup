# Deferred Bench Verification — epic #116 refactor

No hardware is available during the architecture remediation (constraint locked
2026-07-05), so refactor PRs are verified by compile + host tests + review
only. **Every PR that would normally get a bench check appends its physical
checks here.** When the machine is available again, run this entire checklist
in one session, then log the flash in `docs/FIRMWARE-UPLOAD-LOG.md` as usual.

> Rule: an item is only removed once it has been physically verified and the
> result noted. If any item fails, the refactor PR that added it is the first
> suspect.

## Standing checks (run regardless of which PRs merged)

- [ ] Stick→track response identical to V7.34 baseline in all 3 gears, forward/reverse/pivot (feel + serial CSV compare)
- [ ] RC-loss failsafe: ease to neutral, then no pulses; ESCs beep; recovery on signal return
- [ ] Battery ladder on bench supply: Eco-lock ~10.8 V (15 s), alarm 10.5 V, cutoff 10.0 V latched
- [ ] Thermal stages (heat gun / simulated): trill ≥80 °C, Eco ≥90 °C, cut ≥95 °C, auto-recovery + restored flourish; confirm 95 °C sits just BELOW the GL10's own (unpublished) internal thermal limit so our warned cut fires first (src/config/ThermalConfig.h note)
- [ ] Wi-Fi dashboard: connects, SSE at ~5 Hz, page load doesn't perturb control (watchdog silent)
- [ ] Horn, Wi-Fi-ready beep, inactivity alarm patterns unchanged
- [ ] Loop rate still ~20 kHz idle (serial banner / measurement)

## Per-PR additions

Append rows below as PRs merge.

| Date | PR | What to verify physically |
| --- | --- | --- |
