// ═══════════════════════════════════════════════════════════════
// Digger Control V7 — GL10 FOC + Speed-Adaptive Steering
// ═══════════════════════════════════════════════════════════════
//
// Dual-input controller for ride-on excavator (~50 lbs).
// RC operator (Jason) and joystick rider (Malaki) share control
// via 3-position override switch.
//
// Hardware (2026-04-25): GL10 FOC ESC + GL540L motor (replaced the
// earlier sensored-ESC setup). The GL10's FOC handles motor
// acceleration compensation and smoothness internally — Arduino's job
// shrinks to: input mixing, override switch, gear caps.
// V7.2 removed the Arduino-side inertia filter; the ESC's own
// Acceleration + Drag Force settings own command smoothing now.
// V7.6 removed the reverse-direction beeper entirely — audible
// alerts will return as a battery-aware system (see GitHub issue tracker).
// Telemetry: X.BUS Read Register (func 0x10) is polled non-blocking on
// Serial1 (D0/D1) — READ-ONLY. 0x10 is service control; it never puts the
// ESC into BUS_MODE, so PWM control authority is fully preserved. Both
// ESCs share the one half-duplex X.BUS. See [TELEMETRY].
//
// Signal flow:
//   RC (S.BUS) ──► curvatureDrive ──┐
//                                   ├─► Mixer ─► gear cap ─► PWM
//   Joystick ──► curvatureDrive ───┘
//
// Modules (search "[NAME]" to jump):
//   [CONFIG]     All tunable constants
//   [DRIVE]      curvatureDrive — symmetric add + desaturate, smoothstep blend into pivot
//   [RC]         S.BUS input — raw throttle + steering via sbusUart (SCI0)
//   [JOYSTICK]   ADC input — deadband, per-axis expo curve
//   [GEAR]       RC CH4 → Eco 65% / Normal 80% / Boost 100% average-speed cap
//   [MIXER]      Override switch — selects RC vs joystick
//   [OUTPUT]     ESC servo PWM
//   [TELEMETRY]  X.BUS Read Register (0x10) on Serial1 — V/I/RPM/temp
//   [BEEPER]     Active piezo on D8 — horn (RC SWD/CH7) + beep patterns
//   [ALERT]      Battery (< 10.5 V, latched) + inactivity (RC off) alarms on D8
//   [DEBUG]      10 Hz serial CSV (control + telemetry)
//
// Pin map:
//   A0  ← Joystick Y (throttle)        [14-bit ADC]
//   A1  ← Joystick X (steering)        [14-bit ADC]
//   D11 (unused — sbusUart TX on SCI0, S.BUS is RX-only)
//   D12 ← S.BUS RX (sbusUart on SCI0 via NPN inverter)
//   D8  → Active piezo — horn + Wi-Fi/battery/inactivity alarms
//   D9  → Left ESC                      [Servo PWM]
//   D10 → Right ESC                     [Servo PWM]
//   D0/D1 → Serial1 (X.BUS half-duplex telemetry bus to both ESCs)
//   USB-C → USB CDC Serial (debug + firmware upload)
//
// S.BUS wiring (unchanged inverter circuit, now lands on D12):
//   R7FG S.BUS signal ──[10K]──► NPN base
//   NPN emitter ──► GND
//   5V ──[10K]──┬──► NPN collector ──► D12 (sbusUart RX)

#include <Arduino.h>
#include "src/application/FirmwareApp.h"

// The firmware lives under src/ (Phase D, epic #116): application/ owns the
// control cycle and module shims, domain/ the pure logic, ports/ the
// hardware contracts, infrastructure/ the adapters, config/ the tunables.
// This sketch is only the composition root (#189).

void setup() { firmwareSetup(); }
void loop() { firmwareLoop(); }
