# Repository File Map

The annotated inventory of load-bearing files and directories — what each one
is and which issue shaped it. Moved verbatim from root `CLAUDE.md` (#197) so
the per-session context stays lean; this file is the detailed map's single
home. Layer rules live in `.claude/rules/architecture.md`; the canonical
architecture spec is [ARCHITECTURE-TARGET.md](ARCHITECTURE-TARGET.md).

```text
PROJECT-PLAN.md                          — Full technical specification
OPERATOR-GUIDE.md                        — User guide for Jason (RC) and Malaki (joystick)
sketches/dual_track_control/dual_track_control.ino             — COMPOSITION ROOT ONLY (#189): includes src/application/FirmwareApp.h and delegates setup()/loop(); the firmware lives under src/
sketches/dual_track_control/types.h                 — Shared structs (JoystickState, ...; EscTelem re-exported from ports/TelemetrySource.h)
sketches/dual_track_control/src/domain/battery/     — Extracted domain (#117): BatteryTypes.h + VoltagePlausibility.h/.cpp + BatteryLadder.h/.cpp (pure logic, host-testable)
sketches/dual_track_control/src/domain/thermal/     — Extracted domain (#117): ThermalTypes.h + ThermalHysteresis.h/.cpp + ThermalDerating.h/.cpp (pure logic, host-testable)
sketches/dual_track_control/src/domain/operator_input/ — Extracted domain (#117): ExpoCurve.h/.cpp + DeadbandPolicy.h/.cpp (pure input shaping, host-testable)
sketches/dual_track_control/src/domain/drive/       — Extracted domain (#117): DriveTypes.h + CurvatureDrive.h/.cpp + GearPolicy.h/.cpp + CommandMixer.h/.cpp (curvature mix, gear policy, RC/joystick mixer)
sketches/dual_track_control/src/domain/safety/      — Extracted domain (#117): SafetyTypes.h + SafetySupervisor.h/.cpp + OutputGate.h/.cpp (drive allow/deny + FailsafeReason; ACTIVE/HOLD/CUT gate)
sketches/dual_track_control/src/application/        — The application layer (#189): FirmwareApp.h/.cpp (composition-root marker + the verbatim setup/loop control cycle) + FirmwareState + OperatorInput + MotorOutput + SafetyControl + AlertControl + Monitoring (module shims moved from the .ino) + SystemSnapshot.h (#132) + RangeMath.h (#187)
sketches/dual_track_control/src/telemetry/          — Extracted observer layer (#117): TelemetryScaling.h (×10 wire encode, header-only; register decode moved to infrastructure/xc/, #178) + JsonEncoder (dashboard JSON frame) + CsvEncoder (debug CSV line) — both read only the SystemSnapshot (#132)
sketches/dual_track_control/src/alerts/             — Alert policy + players (#183): AlertTypes.h + AlertPolicy (priority ladder, low-V latch, inactivity) + PatternPlayer (one-shot + repeating) — pure logic; the piezo stays behind ports/AlertOutputPort.h
sketches/dual_track_control/src/config/             — Tunables per domain (#185): BuildInfo.h (FIRMWARE_VERSION SSOT) + Pins.h + Input/Drive/Battery/Thermal/Alert/Telemetry/Wifi/SafetyConfig.h — values are law, included by src/application/FirmwareApp.h (adapter-owned tunables stay single-homed in infrastructure/)
sketches/dual_track_control/src/ports/              — Hardware contracts as link-time seams (#130): EscOutputPort.h + JoystickPort.h + AlertOutputPort.h + RcInputPort.h (RcFrame) + TelemetrySource.h (EscTelem) + DashboardServicePort.h + TelemetryFrameSource.h (reverse seam: app encodes, network ships bytes) + ClockPort.h + WatchdogPort.h + DebugConsolePort.h (#187 — the last hardware seams)
sketches/dual_track_control/src/infrastructure/     — The ONLY layer with hardware includes (#130): arduino/PwmEscAdapter.cpp (owns the Servos) + arduino/AdcJoystickAdapter.cpp (ADC settle + resolution) + arduino/PiezoAdapter.cpp + arduino/ArduinoClock.cpp + arduino/WatchdogAdapter.cpp + arduino/SerialConsoleAdapter.cpp (#187) + radiolink/SbusReceiverAdapter.cpp (owns sbusUart + the S.BUS parser) + xc/XbusTelemetryAdapter.h/.cpp (X.BUS 0x10 poller + register decode) + network/ (WifiService + DashboardServer + SseStream — the #69/#54/#77 serving machine + its tunables, #181)
sketches/dual_track_control/arduino_secrets.h.example — Wi-Fi credential template (#125); copy to arduino_secrets.h (gitignored)
sketches/dual_track_control/web_page.h              — GENERATED from dashboard/index.html (scripts/generate_web_page.py) — never hand-edit
sketches/telemetry_check/telemetry_check.ino     — X.BUS 0x50 telemetry bench tool: throttle hard-wired to 0, but 0x50 puts the ESC in BUS_MODE — bench only, never alongside PWM control (production uses 0x10)
sketches/sbus_d12_test/sbus_d12_test.ino — S.BUS-on-D12 bring-up test
dashboard/index.html                     — Wi-Fi dashboard SOURCE (single source of truth; web_page.h generated from it, #120)
docs/WIRING-GUIDE-V8.md                  — Canonical hardware wiring reference (UNO R4 WiFi)
docs/INTERFACE-BOARD-PERFBOARD.md        — Soldered interface board build
docs/GL10-Manual.pdf                     — XC-ESC official user manual (image-based, 3 pages)
docs/GL10-PARAMETERS.md                  — GL10 parameter reference + code-context analysis
docs/GL10-OPERATION.md                   — Startup, throttle calibration, factory reset, LED/beep reference
docs/XBUS-PROTOCOL.md                    — XC X.BUS protocol reference (used by live telemetry)
docs/CONTROL-RESEARCH.md                 — Tank mix, RC input, loop patterns research
docs/MISSION.md                          — Project design philosophy (smoothness above all)
docs/DECISION-LOG.md                     — Technical decision log
docs/TESTING.md                          — Host characterization + invariants suites + commit gate (#47, #131)
docs/SAFETY.md                           — Safety-invariant registry: propulsion-path invariants + test links (#131)
docs/AGENT-EXAMPLES.md                   — Good/bad task-execution pairs for AI agents (#53, real precedents)
docs/wiki/                               — AI-maintained knowledge graph (Obsidian-visualizable, links-only — no constants; #141)
tests/                                   — Host test harness: stub Arduino env + doctest suites (characterization/ + invariants/ cross-cutting; domain/ + telemetry/ + alerts/ mirror src/, #195)
.claude/skills/                          — Project skills (#127): safety-review, flash-and-log, new-module, field-tune, wiki-impact-review, prepare-pull-request (one procedure per folder)
.claude/agents/                          — Read-only reviewer subagents (#127): architecture, safety, tests, documentation
.githooks/pre-commit                     — Commit-time test gate (activate: git config core.hooksPath .githooks)
live_plot.py                             — Real-time matplotlib monitor
monitor.py                               — Simple serial monitor
```
