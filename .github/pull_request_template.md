## Summary
<!-- 1-3 bullet points describing what this PR does and why -->

## Changes
<!-- List the specific files and what was modified -->

## Arduino Review Checklist

### Safety
- [ ] ESC outputs constrained to safe range (OUTPUT_MIN/OUTPUT_MAX)
- [ ] Failsafe triggers neutral on RC signal loss
- [ ] No blocking code in main loop (no `delay()`, no `pulseIn()`)
- [ ] Deadbands prevent unintended drift at neutral
- [ ] Power limit percentage is appropriate for current testing phase

### Code Quality
- [ ] No raw register access without comments explaining purpose
- [ ] Constants used instead of magic numbers
- [ ] ISR handlers are minimal (no Serial, no floating point, no long operations)
- [ ] `volatile` used for all variables shared between ISR and loop
- [ ] `noInterrupts()`/`interrupts()` used when reading ISR variables in loop

### Hardware Compatibility
- [ ] Pin assignments match physical wiring
- [ ] Correct board FQBN used for compilation
- [ ] ADC resolution matches board capability (10-bit for Nano V3, 14-bit for R4)
- [ ] Servo library compatible with target board
- [ ] Serial baud rate matches between Arduino and monitoring tools

### Testing
- [ ] Sketch compiles without warnings
- [ ] Flash and RAM usage within safe limits (<80%)
- [ ] Serial output format matches live_plot.py parser
- [ ] Tested on bench with ESCs disconnected (serial verification)
- [ ] Field tested with ESCs connected at reduced power

### Documentation
- [ ] Version number updated in sketch header
- [ ] PROJECT-PLAN.md reflects current state
- [ ] OPERATOR-GUIDE.md updated if behavior changed
- [ ] Tuning constants table updated if values changed

## Test Plan
<!-- How was this tested? What should reviewers verify? -->

---
Generated with [Claude Code](https://claude.com/claude-code)
