# Decision Log

Technical decisions, test results, and confirmed hardware facts.
Updated by session hooks — only technical content, no personal info.

---

## 2026-07-19 — test-gate rewritten with real command parsing (#206)

- `.claude/hooks/test-gate.sh` (shell globs over the whole JSON payload) →
  `.claude/hooks/test-gate.py`: JSON + `shlex` parsing; bypass flags are
  checked ONLY among a real `git commit` invocation's own arguments.
  Fixes the observed false positives (benign `grep -n` in compound
  commands; issue/PR body prose quoting the flag names) and catches
  `git -C <path> commit -n`, which the old `"git commit"` glob missed.
  Two-attempt parse: line-oriented (newline = separator, catches multi-line
  bypasses) then whole-text (quoted heredoc commit messages legally span
  lines — the exact case the first cut blocked LIVE on its own commit);
  malformed input falls back to a first-line TOKEN check (whitespace
  split, same argument analysis). Bypass detection covers every git
  spelling: exact flags, bundled short options (`-anm` carries `-n`;
  value-taking shorts `m c C F t S` end the scan so `-mnope` passes),
  parse-options long abbreviation (`--no-v` prefix), and `--` ends option
  scanning (pathspecs). Tokenizing uses `shlex` punctuation_chars so glued
  separators cannot hide an invocation (`true;git commit -n` and
  `git commit --no-verify&&git push` were live bypasses of the exact-match
  cut — both reproduced, then sealed). The hooksPath check targets the
  commit's OWN repo (honors `git -C` / `--git-dir`).
  Wrapper/environment prefixes (`FOO=1 git`, `env git`, `sudo -u user
  git`) cannot hide the invocation — `git` is recognized in command
  position (leading assignments and executing wrappers keep it open;
  `git` as another command's data, e.g. `echo git commit -n`, does not
  block, even behind a wrapper: `env echo git commit -n`; duration
  positionals like `timeout 30 git ...` stay caught).
  Redirections are not command separators: `git 2>/dev/null commit -n`
  stays a caught invocation, trailing `> /dev/null 2>&1` on a clean
  commit passes. Both parse paths use the same command-position logic (git as data
  after a line-spanning quote stays data; a real bypass after one still
  blocks). Shell line continuations are joined first (backslash-newline), so a
  continuation-split `-n` is still caught.
  Shell -c strings (`sh -c 'git commit -n'`, incl. bundled `-lc`) are
  parsed recursively (bounded depth 5).
  `check_hook_registration.py` grew to 35 cases;
  structure-check allows
  `.py` under `.claude/hooks/` (hook scripts live with the config that
  registers them).

## 2026-07-16 — CLAUDE.md trimmed to ~200 lines (#197)

- Root CLAUDE.md 445 → 203 lines. Facts kept (purpose, covenant + gate,
  safety invariants, architecture boundaries, build commands, workflow
  pointers); detail moved to single homes: full file map →
  `docs/architecture/FILE-MAP.md` (verbatim; doc-sync hook message updated
  to point there); pins/UART detail → `docs/WIRING-GUIDE-V8.md` (already
  canonical there); X.BUS detail → `docs/XBUS-PROTOCOL.md`; Implementation
  Status section
  DELETED (board is SSOT) — its one untracked TODO became issue #208
  (track-speed asymmetry; its "#54 pending" line was stale, #54 closed).
- WIRING-GUIDE drift fixed while consolidating: D8 listed as free but the
  active piezo shipped on D8 in V7.11.

## 2026-07-15 — Phase E: project skills + read-only reviewer subagents (#127)

- Six project skills created under `.claude/skills/` (safety-review,
  flash-and-log, new-module, field-tune, wiki-impact-review,
  prepare-pull-request) — each skill OWNS its procedure; CLAUDE.md's ESC
  section reduced to the iron rule + a pointer (dedup per #127 acceptance).
- Four read-only reviewer subagents under `.claude/agents/` (architecture,
  safety, tests, documentation) — tool-restricted to Read/Grep/Glob
  (enforceably read-only; the caller supplies the diff); run before marking
  a PR ready.
- CLAUDE.md gained the Skills & Reviewer Agents index; agent-governance wiki
  note updated in the same PR.

## 2026-07-15 — test-gate hook wired + external-review verification (#193)

- `.claude/hooks/test-gate.sh` was dead code: `.claude/settings.json` had no
  `PreToolUse` entry, so the #47 agent-proof layer never executed (found by
  external repo review of `main@ea085d0`; TESTING.md had documented the wiring
  as if present). Registered as `PreToolUse[Bash]` and CI-guarded: new
  `scripts/check_hook_registration.py` (in `hooks-selftest.yml`) fails when a
  script in `.claude/hooks/` is unregistered, and functionally exercises the
  gate (blocks `--no-verify`/`-n`, blocks inactive `core.hooksPath`) — 7/7.
- Same review verified via API: `main` branch protection requires 1 approval
  but ZERO required status checks — all 9 CI workflows advisory (→ #196;
  already flagged 2026-07-13 as pending operator work). Wiki lint confirmed
  structural-only (links/orphans/reachability/tunable-drift) — semantic sync
  tracked in Phase F milestone (#199, #200, #202).
- Review follow-ups filed as #193–#203; Phase F milestone created.

## 2026-07-13 — Rename sweep: production sketch + bench abbreviations (#118)

- `sketches/rc_test/` → `sketches/dual_track_control/` — the production
  firmware (7 versions deep: safety ladders, watchdog, Wi-Fi dashboard) no
  longer carries a bring-up-test name. `dual_track_control` is the platform's
  white-label name (`.claude/rules/naming.md`, #136 context). Version lineage
  V7.x unchanged; FIRMWARE-UPLOAD-LOG history rows keep `rc_test`.
- Bench sketches with abbreviations not on the permitted-acronym list renamed
  to full words: `hw_diagnostic` → `hardware_diagnostic`, `telem_check` →
  `telemetry_check`, `telem_decode` → `telemetry_decode`. Acronym-compliant
  siblings (`pin_test`, `sbus_test`, `sbus_d12_test`, `usb_test`,
  `beeper_test`) keep their names.
- Follow-up in the same sweep (per the 2026-07-05 scope comment on #118):
  `live_plot_telem.py` → `live_plot_telemetry.py`, `tools/ui-test-batt.mjs`
  → `tools/ui-test-battery.mjs`, `tools/ui-test-batcolor.mjs` →
  `tools/ui-test-battery-color.mjs`; `sketches/serial2_test/` DELETED
  (operator-authorized — Nano-R4-era Serial2 collision test, obsolete on the
  UNO R4 WiFi).
- Mechanical sweep across 70 files (CI matrix, pre-commit gate, the
  tests/Makefile and FirmwareUnderTest.h include paths, scripts, docs,
  source comments);
  DECISION-LOG and FIRMWARE-UPLOAD-LOG history left untouched by design.
  Zero code changes — behavior-preserving by construction.

## 2026-07-13 — Phase D step 11b: FirmwareApp composition root — §9 COMPLETE (#189)

- The entire .ino shell moved VERBATIM into `src/application/`, split by
  responsibility: FirmwareState (mutable cross-module state + the cap
  delegates, now inline in its header for cross-TU odr-use), OperatorInput
  ([DRIVE]/[RC]/[GEAR]/[JOYSTICK]/[MIXER]), MotorOutput ([OUTPUT]),
  SafetyControl ([SAFETY]), AlertControl ([TELEMETRY]/[BEEPER]/[ALERT] —
  owns telem[]), Monitoring ([WIFI] remnant + [DEBUG] + SystemSnapshot
  construction; the ONE TU including web_page.h and arduino_secrets.h),
  and FirmwareApp.h/.cpp — the #150 MARKER + setup()/loop() bodies as
  firmwareSetup()/firmwareLoop() (watchdog refresh at its exact position).
  rc_test.ino is now a 12-line composition root; **check_ino is ACTIVE and
  passing** (the migration-window NOTE is gone).
- Two structural consequences, both verified behavior-neutral: (1) the
  AlertConfig pattern arrays became C++17 `inline` — ONE program-wide
  entity, preserving the pointer-identity semantics playback and the tests
  rely on (per-TU copies broke both); (2) config/Pins.h reaches A0/A1
  through the types.h bridge (outside-src include) since application TUs
  compile standalone; documented as interim until types.h dissolves.
- Harness: FirmwareUnderTest.h unchanged — everything resolves through the
  .ino → FirmwareApp.h chain; the pure suites now also exclude
  application/ .cpps (the infrastructure-filter precedent — application
  reaches Arduino.h via the types bridge).
- Verified: 27 host binaries green, ZERO expectation changes; compile
  flash BYTE-IDENTICAL (117796), RAM 9832 (−16, layout); fitness OK
  (4 advisory soft-size warnings, hard cap respected everywhere).
- **PHASE D §9 (steps 1–11) IS COMPLETE.** Remaining epic work: flip
  architecture-fitness/unit-tests/hooks-selftest to required checks
  (operator), the #118 rename sweep, #119/#121/#122, Phase E.

## 2026-07-13 — Phase D step 11a: the last hardware seams (#187)

- Step 11 split in two: 11a (this) builds the remaining seams so the .ino
  becomes hardware-include-free; 11b moves the shell into src/application/
  and activates check_ino. Rationale: application/ may not include
  Arduino.h/WDT.h/WiFiS3.h (dependency table), but the shims still called
  millis/micros, WDT, Serial, WiFi.status and analogReadResolution.
- New seams (all 1-line passthroughs): ports/ClockPort.h + ArduinoClock
  (clockNowMs/clockNowUs; ~13 mechanical call-site swaps),
  ports/WatchdogPort.h + WatchdogAdapter (begin/timeoutMs/refresh — the
  once-per-loop refresh invariant kept at its exact call site),
  ports/DebugConsolePort.h + SerialConsoleAdapter (begin keeps debugInit's
  verbatim boot delay(50); ready(); printLine). JoystickPort grew
  joystickInitialize() (analogReadResolution(14), same boot position);
  DashboardServicePort grew dashboardServiceRadioStatus() (WiFi.status —
  wifiDebug's last WiFiS3 touch). <WDT.h> + <WiFiS3.h> removed from the
  .ino; arduino-cli library detection re-verified via the src/ scan.
- Debug lines (banner, WDT-armed, wifiDebug x2) converted from Serial
  print-chains to snprintf-assembled whole lines — BYTE-IDENTICAL stream
  by construction (bool as 1/0, decimal counters, %02X + space per
  snapshot byte, println's CRLF via printLine). These lines are not
  test-locked; identity argued by string construction in the PR.
- Two documented mechanical substitutions (the #164 class) so 11b's moved
  code compiles without Arduino.h: constrain() → clampInt/clampFloat
  (exact macro ternary, 12 sites, types checked per site) and sbusToServo's
  map() → mapRange (ArduinoCore-API's exact long formula), both in
  src/application/RangeMath.h.
- Verified: 27 host binaries green, ZERO expectation changes (the
  control-loop suite's one-refresh-per-pass law now counts through
  watchdogRefresh() — same stub counter); compile clean — flash 117796
  (−152: line assembly replaced print-chain calls), RAM 9848 unchanged.

## 2026-07-13 — Step-11 pre-slice: config/ layer extraction (#185)

- Every `[CONFIG]` constant moved VERBATIM (values byte-identical) to
  `src/config/` per the target's grouping: BuildInfo.h (FIRMWARE_VERSION —
  the #124 SSOT gate is home-agnostic, now fires on this file), Pins.h
  (application pins; documented include-order contract — it uses the
  core's A0/A1 macros and must follow <Arduino.h>, config/ may not include
  hardware headers), InputConfig, DriveConfig (incl. CALIBRATION_MODE +
  the PIVOT_THROTTLE_TAPER static_assert), BatteryConfig, ThermalConfig,
  AlertConfig (patterns + tunables), TelemetryConfig (PRINT_INTERVAL),
  WifiConfig (AP channel), SafetyConfig (WDT_TIMEOUT_MS).
- Deliberately NOT moved: the mutable state interleaved in [CONFIG]
  (gearScale/currentGear, safety latches, thermal flags — state is not
  configuration; goes with FirmwareApp), the outCapToX/reverseCap
  delegates + Gear↔GearLevel static_assert (read globals + domain), and
  constant NAMES (SVC/OVR_*/JOY_* stay verbatim — dozens of test files
  reference them; renames are the #118 sweep's call).
- Single-homed adapter tunables stay put (X.BUS poll constants in xc/
  since #178; Wi-Fi serving tunables in network/ since #181).
- Verified: 27 host binaries green, ZERO test edits; compile
  BYTE-IDENTICAL (flash 117948, RAM 9848 — compile-time constants);
  fitness OK. [CONFIG] in the .ino is now the config include point +
  the state/delegate block.

## 2026-07-13 — Step-11 pre-slice: alerts/ layer extraction (#183)

- `[BEEPER]`/`[ALERT]` policy and sequencing moved VERBATIM to
  `src/alerts/`: AlertTypes (pattern/state/threshold structs), AlertPolicy
  (inactivity tracking, the low-voltage debounce/latch INCLUDING its own
  plausibility comparison form — deliberately NOT consolidated with
  domain/battery/VoltagePlausibility, the two diverge on NaN per the
  documented SAFETY.md gap — and the four-level priority ladder
  thermal-cut > low-V > thermal-warn > inactivity), PatternPlayer
  (one-shot + repeating, even index = ON). Time is a parameter throughout.
- The `.ino` shims keep every global and signature (mirror-globals
  precedent) — beepStart/beeperUpdate/alertInit/alertUpdate fold state
  through the structs per call; the `alertOutputSet(horn || pattern ||
  alarm)` OR stays in the shim; patterns + tunables stay `[CONFIG]`-owned
  until the config/ pre-slice. ZERO expectation changes (test_alerts.cpp
  characterization is law); new pure suite tests/alerts/ (6 cases:
  priority ladder, latch debounce/sticky/grace/implausible, inactivity,
  both players).
- Accepted micro-delta (same class as the #158 entry): the one-shot
  transition previously read millis() twice (compare, then phase
  re-anchor); the pure step takes one nowMs. Host-identical (stub time
  constant within a call); ≤1 ms phase re-anchor difference per transition
  on hardware.
- DEFERRED with reasoning (own issues when prioritized): lowVoltLatched
  ownership (#119 target: SafetyDecision.alarmRequested — [SAFETY] still
  borrows the global) and snapshot-consuming alert presentation (#132
  deferral — alertUpdate keeps reading telem[] directly).
- Verified: 27 host binaries green; compile clean — flash 117980 (+320,
  shim fold/unfold), RAM 9848 unchanged.

## 2026-07-13 — Phase D step 10 final slice: infrastructure/network/ (#181)

- The `[WIFI]` serving machine moved to `src/infrastructure/network/`
  behind `ports/DashboardServicePort.h`: `WifiService` (AP bring-up +
  FNV-1a ETag #109 + shared serving state, extern for the harness),
  `DashboardServer` (bounded request read/route, /data one-shot, ETag/304
  path, incremental one-chunk-per-pass page transfer #69 with 0-byte-write
  aborts), `SseStream` (SSE upgrade with unconditional prior-socket reap
  #77, single-write heartbeat+data frame #54). All #69/#54/#77 invariants
  moved line-for-line; SSE branch mechanically restructured from a nested
  else into early returns (same predicates, same order).
- **Dependency inversion:** `ports/TelemetryFrameSource.h` is a REVERSE
  link-time seam — declared in ports/, implemented by the .ino (delegates
  to buildTelemJson), called by the network layer where wifiUpdate used to
  call buildTelemJson. Infrastructure never includes application/ or
  telemetry/; it ships bytes.
- Identity stays sketch-owned and is passed at initialize: credentials
  (arduino_secrets.h never leaves the .ino), WIFI_AP_CHANNEL, and the
  INDEX_HTML page pointer. The four serving tunables (SSE_INTERVAL_MS 200,
  SSE_FRAME_CAP 448, WIFI_PAGE_CHUNK 1024, WIFI_MODEM_TIMEOUT_MS 50) moved
  from [CONFIG] into WifiService.h with the machine — values unchanged,
  single home (the target config/ layer re-homes them later). The ready
  beep stays application-side (initialize returns AP-up); the beep now
  fires ~1 ms later at boot (after the banner prints) — non-observable.
- `wifiMode()`, `wifiDebug()` (reads adapter state via extern) and
  `wifiSeq` stay in the .ino. `pagePtr` renamed `pagePointer` on move
  (naming.md). Verified: 26/26 host binaries green, ZERO expectation
  changes; flash 117660 (+64), RAM 9848 (+12: the page-pointer global —
  layout, not behavior). Step 10 COMPLETE — only step 11 FirmwareApp
  remains in Phase D's §9 sequence.

## 2026-07-13 — SystemSnapshot: observers read one immutable per-cycle struct (#132)

- `src/application/SystemSnapshot.h` (first `application/` file — the #150
  check_ino marker stays dormant until FirmwareApp.h): nowMs, RC servo-µs
  input view + failsafe/lost, raw joystick pair, outL/outR, gear + override
  mode, the five safety flags, and per-ESC `EscTelem` copies. Deliberately
  does NOT include types.h (which pulls Arduino.h) so the pure suites stay
  stub-free — gear is carried as a plain int.
- `telemetry/JsonEncoder` + `telemetry/CsvEncoder`: buildTelemJson()'s and
  debugPrint()'s exact format strings moved VERBATIM; both take
  `const SystemSnapshot&` and use the TelemetryScaling encode helpers —
  killing debugPrint's inline lroundf duplication (same math, the #121
  second half). wifiSeq stays [WIFI]-owned and is passed in.
- The .ino shims keep names/signatures and build the snapshot AT THE
  ORIGINAL OBSERVATION POINTS (buildTelemJson reads millis() exactly where
  it used to) for exact timing parity; the once-per-cycle build moves into
  FirmwareApp at step 11. Alert presentation deferred to the alerts/
  extraction with reasoning ([ALERT] owns lowVoltLatched — policy, not a
  read-only observer; documented on issue #132).
- NEW pure suite tests/telemetry/test_observer_encoders.cpp locks the EXACT
  JSON/CSV bytes (previously untested); 26 host binaries green. Compile
  clean — flash 117580 (+224: the snapshot copy at the observation points;
  emitted bytes identical), RAM 9836 unchanged.

## 2026-07-13 — Phase D step 10 slice 4: TelemetrySource + XbusTelemetryAdapter (#178)

- `ports/TelemetrySource.h` now owns the `EscTelem` struct (moved verbatim
  from `types.h`; types.h re-includes the port so every consumer still
  resolves it) + `telemetrySourceInitialize()` /
  `telemetrySourceUpdate(EscTelem*, escCount)`. The CALLER keeps owning the
  `telem[]` array — [ALERT], [SAFETY], the dashboard JSON and debug all read
  it sketch-side; the adapter owns only the bus + poll state.
- `infrastructure/xc/XbusTelemetryAdapter.h/.cpp` (first `xc/` folder): the
  whole 0x10 poller moved VERBATIM — protocol constants, checksum, request
  framing, echo-skipping parse, EMA fold, alternating poll state machine,
  per-ESC staleness watchdog, `Serial1.begin`. The header is host-pure
  (constants + inline scaling + extern state for the harness).
- Register DECODE + `emaFold` moved from `telemetry/TelemetryScaling.h`
  into the adapter header — forced by the dependency table (infrastructure/
  may not include telemetry/) and semantically right (decode = X.BUS
  register semantics). The ×10 wire ENCODE stays in TelemetryScaling.h.
  Each function still has exactly ONE implementation.
- Naming fixed on move (naming.md applies to MOVED code): `telemEsc` →
  `telemetryPolledEscIndex`, `telemWaiting` → `telemetryAwaitingResponse`,
  `telRx/telRxLen` → `telemetryReceiveBuffer/Length`, `telDbg*` →
  `telemetryDebug*`, `telemSendRequest` → `telemetrySendReadRequest`,
  `TELEM_*` → `TELEMETRY_*`, `XBUS_HDR_*` → `XBUS_HEADER_*`, `REG_*` →
  `REGISTER_*` full words. EscTelem FIELD names unchanged (dozens of
  untouched consumers — mass-rename prohibited). Tests updated mechanically;
  `telemUpdate()` stays as the .ino shim (loop call site + suite entry
  unchanged). `telDbgPrintPrevMs` found to be write-never-read (bring-up
  leftover) — moved as-is, no behavior question.
- Verified: all 25 host binaries green, ZERO expectation changes; compile
  clean — flash 117324 (−32 vs 117356), RAM 9836 (+4; layout). Remaining
  step-10 adapter: Wi-Fi/network; watchdog + clock land with FirmwareApp
  (step 11).

## 2026-07-13 — Phase D step 10 slice 3: RcInputPort + SbusReceiverAdapter (#176)

- `ports/RcInputPort.h` defines the RC input contract in domain types:
  `RcFrame {int16_t channels[16]; bool failsafe; bool lostFrame;}` +
  `rcInputInitialize()` / `rcInputReadFrame(RcFrame*)`. The bfs vendor
  S.BUS type never crosses the port — first slice to translate a vendor
  type at the boundary rather than pass it through.
- `infrastructure/radiolink/SbusReceiverAdapter.cpp` owns `UART
  sbusUart(11, 12)` (SCI0 — the not-`Serial2` collision detail) and
  `bfs::SbusRx` (moved from the `.ino`); on `Read()` success it copies 16
  channels + failsafe/lost_frame into the caller's `RcFrame`. First
  non-`arduino/` infrastructure folder (vendor-specific: radiolink).
- The `.ino`'s `bfs::SbusData sbusData` global became `RcFrame rcFrame`;
  every consumer renamed mechanically (`.ch[` → `.channels[`,
  `.lost_frame` → `.lostFrame`). The loop() read block keeps its exact
  freshness semantics (`sbusLastFrame = now` on frame, `SBUS_TIMEOUT`
  staleness unchanged). `#include "sbus.h"` moved to the adapter
  (empirically verified — arduino-cli library detection scans src/).
- Harness: `feedSbusFrame()` now scripts through `rcInputReadFrame()`
  exactly like loop(); `resetFirmwareState()` resets `rcFrame` and
  re-creates the extern `sbusRx` (parser residue = power-cycle
  equivalence). Stub sbus.h frame queue is global, so scripting is
  unchanged.
- Verified: all 25 host binaries green, ZERO expectation changes; compile
  clean — flash 117356 (+52), RAM 9832 (+4; linkage/layout of the moved
  objects, not behavior). Remaining step-10 adapters: X.BUS telemetry
  poller, Wi-Fi/network; watchdog + clock land with FirmwareApp (step 11).

## 2026-07-13 — Phase D step 10 slice 2: JoystickPort + AlertOutputPort adapters (#174)

- `ports/JoystickPort.h` + `infrastructure/arduino/AdcJoystickAdapter.cpp`:
  the ADC conditioning sequence moved VERBATIM (discard-read → 100 µs mux
  settle → real read, Y then X — hardware timing at the 100 Hz joystick
  cache cadence, non-blocking budget unchanged). `updateJoystick()` keeps
  the cadence guard, deadband/expo/gain pipeline and cachedJoy writes.
- `ports/AlertOutputPort.h` + `infrastructure/arduino/PiezoAdapter.cpp`:
  pinMode+LOW at initialize, one digitalWrite per set. The horn/pattern/
  alarm OR-priority stays in `[BEEPER]` (alert policy, not hardware —
  destined for the alerts/ layer later); `beeperUpdate()` computes the
  same boolean and calls `alertOutputSet(...)`.
- Verified: all 25 host binaries green, zero expectation changes (stub
  analogRead/GPIO captures now reached through the adapters); compile
  clean — flash 117304 (+68 vs 117236), RAM 9828 (+4: the piezo pin int).
  Remaining step-10 adapters: S.BUS receiver, X.BUS telemetry poller,
  Wi-Fi/network, watchdog + clock (port-less in the target — land with the
  FirmwareApp composition root, step 11).

## 2026-07-13 — Phase D step 10 slice 1: EscOutputPort + PwmEscAdapter (#172)

- First infrastructure slice: `ports/EscOutputPort.h` (link-time seam per
  #130 — free functions, no virtuals) + `infrastructure/arduino/
  PwmEscAdapter.cpp`, which now owns the `Servo escL, escR` objects (moved
  from the `.ino`) and remembers the pins passed at
  `escOutputInitialize(PIN_ESC_L, PIN_ESC_R)`.
- Behavior boundaries: the `constrain(SVMIN, SVMAX)` clamp and the
  `outL`/`outR` globals stay in the `.ino` (application-visible state read
  by buildTelemJson and the OutputGate shim); only the writeMicroseconds /
  attach / detach calls go through the port — same call sites, same order.
- Harness: pure suites now compile `SKETCH_PURE_CPPS` (infrastructure
  filtered out — hardware includes need the stubs the pure suites
  deliberately exclude); firmware suites compile everything (stub Servo.h
  captures unchanged). `FirmwareUnderTest.h` reaches the moved Servo
  objects via extern declarations. The adapter includes `<Arduino.h>`
  before `<Servo.h>` (the stub Servo.h uses millis()).
- Verified: all 25 host binaries green, zero expectation changes; compile
  clean — flash 117236 (+76), RAM 9824 (+12: the adapter's two remembered
  pin ints — first RAM delta of the series; memory layout, not behavior).
  architecture-fitness exercises ports/ + infrastructure/ layer rules for
  the first time: OK. Remaining adapters (joystick ADC, S.BUS, X.BUS,
  Wi-Fi, piezo, watchdog, clock) follow in later slices or with the
  FirmwareApp composition root (step 11).

## 2026-07-13 — Phase D step 9: OutputGate extracted to src/domain/safety/ (#170)

- `outputUpdate()`'s fail-safe state machine (#88/#65) moved VERBATIM to
  `src/domain/safety/OutputGate.h/.cpp` as `outputGateStep(OutputGateState&,
  driveAllowed, commanded, current, nowMs, neutralPulse, holdDurationMs) →
  OutputGateAction {attach, write, pulses, detach}`. The pure machine owns
  the ACTIVE/HOLD/CUT transitions and the exact float ease-out ramp
  (zero-duration guard, >1 clamp, `rampFrom + (int)((neutral−rampFrom)·t)`);
  the shim executes the hardware actions in the original order — attach,
  write, detach — so the ESCs still see the final neutral write before
  losing signal.
- Behavior laws carried and unit-locked: ease-out starts from the LIVE
  outputs snapshot; detach only after the ramp completes; resume-from-CUT
  re-attaches while recovery mid-HOLD does NOT (ESCs were never detached);
  CUT emits nothing. OutState↔OutputGateMode value correspondence
  static_asserted in the shim (the GearLevel↔Gear pattern).
- SVC and CUTOFF_HOLD_MS stay `[CONFIG]`, passed as parameters; millis()
  read by the shim. Gate globals (outState/outHoldMs/rampFromL/R) stay in
  the .ino, mirrored via the state struct — harness resets untouched.
- Verified: all 25 host binaries green, zero expectation changes (new
  `tests/safety/test_output_gate_domain.cpp`, 5 cases); compile clean —
  flash 117160 (+160 vs 117000), RAM unchanged 9812. This completes the
  DOMAIN extractions (§9 steps 1–9); steps 10–11 are infrastructure
  adapters + the FirmwareApp composition root.

## 2026-07-12 — Phase D step 8: SafetySupervisor extracted to src/domain/safety/ (#168)

- The inline drive gate in `loop()` — `driveAllowed = sbusValid &&
  (batteryOkConfirmed || bootGraceElapsed) && !batteryCutoffLatched &&
  !tempCutActive` — moved to `src/domain/safety/SafetySupervisor.h/.cpp` as
  `safetySupervisorDecide(SafetyInputs) → SafetyDecision {driveAllowed,
  FailsafeReason}`. The early-return shape is provably identical to the &&
  chain (unit-locked by an exhaustive 32-input equivalence case); the
  reason priority mirrors the chain order (RC_STALE > BOOT_GATE >
  BATTERY_CUTOFF > THERMAL_CUT).
- The fail-open boot gate (#65 law — a dead X.BUS never permanently
  disables driving) moved verbatim as `batteryConfirmed ||
  bootGraceElapsed`; the call site computes the millis()-based grace check
  and passes a bool (time is a parameter).
- The `FailsafeReason` is new surface consumed by the OutputGate (step 9)
  and SystemSnapshot (#132) steps; today's call site uses only
  `driveAllowed` (documented at the call site). `outputUpdate()`'s
  ease-to-neutral state machine is step 9 — untouched here.
- Verified: all 24 host binaries green, zero expectation changes; compile
  clean — flash 117000 (+112 vs 116888, cross-TU call + struct), RAM
  unchanged 9812. No bench check applicable (pure move).

## 2026-07-12 — Phase D step 7: GearPolicy + CommandMixer extracted to src/domain/drive/ (#166)

- `updateGear()`'s decision tree moved to `GearPolicy.h/.cpp` as
  `gearPolicySelect(gearPulse, rcValid, forceEco, parameters) →
  GearSelection` — RC-invalid failsafe, strict CH4 thresholds, forceEco
  (battery Eco lock OR thermal Eco) overriding last, verbatim. The
  `reverseCap()`/`outCapToX()` helpers became parameter-visible
  `reverseCapForGear()`/`trackCapToAxisDomain()`. The `.ino` shims own the
  boundary reads (sbus CH4, Eco forces) and mirror `gearScale`/`currentGear`.
- `maxOppose()` (#90) moved keeping its name/signature (definition deleted
  from the `.ino`, like `expoCurve` in #162); `mixCommands()` delegates to
  `mixAxisCommands()` with the `OVR_LO`/`OVR_HI` thresholds as parameters —
  Mode 2's exact-zero rcActive compare preserved verbatim (locked law).
- Domain `GearLevel` enum (GEAR_ECO/NORMAL/BOOST = 0/1/2) mirrors types.h's
  `Gear` values; the correspondence is `static_assert`ed in the `.ino`, so
  the shim casts are compile-time-proven and the SSE gear encoding is
  untouched. New `AxisCommand` (layout-identical to `DriveCommand`) joins
  `DriveTypes.h`.
- Verified: all 23 host binaries green with zero expectation changes (new
  `tests/drive/test_gear_mixer_domain.cpp`: threshold edges, failsafe and
  forceEco dominance, per-gear reverse caps, cap-domain conversion,
  maxOppose truth table, all three mixer modes incl. the exact-zero law).
  Compile clean — flash 116920 (+136 vs 116784, cross-TU calls), RAM
  unchanged 9812. Step-6 domain test scale constants renamed
  (GEAR_ECO→GEAR_ECO_SCALE) to clear the new enum's names — test-file
  naming only, no expectation changes.

## 2026-07-12 — Phase D step 6: CurvatureDrive extracted to src/domain/drive/ (#164)

- The propulsion-path core `curvatureDrive()` (`[DRIVE]`) moved to
  `src/domain/drive/CurvatureDrive.h/.cpp` as
  `curvatureDriveStep(xSpeed, zRotation, gearScale, CurvatureParameters)`
  over `DriveTypes.h` (`TrackCommand` — layout-identical to WheelSpeeds;
  `CurvatureParameters` carrying pivotCap/taper/blend band/turnTrackCap).
  The algorithm commentary (#72/#86/#96/#114 laws) moved with the code.
- Exactly two mechanical substitutions, both float-semantics-identical:
  Arduino's `constrain()` macro expanded as `clampFloat()` (same ternary),
  and tunables read from the parameters struct.
- **State-ownership sin fixed structurally**: `curvatureDrive()` was the
  documented example of a hidden global read (`currentGear` behind the
  parameter list). The `.ino` delegate now owns the gear read visibly —
  `pivotCap = (currentGear == GEAR_LOW) ? PIVOT_SPEED_CAP_LOW :
  PIVOT_SPEED_CAP` — and passes it in. Same value, same pass. The delegate
  keeps the exact `curvatureDrive(x, z, gearScale) -> WheelSpeeds`
  signature, so all 17 test references and both call sites are unchanged.
- Verified: all 22 host binaries green with ZERO expectation changes — the
  characterization suite's exact float32 expectations for pivot caps, the
  smoothstep blend, the faded-ceiling desaturation, and reverse-steering
  consistency pass unchanged against the extracted implementation. New
  `tests/drive/` pure suite (7 cases) locks the laws at the unit level.
  Compile clean — flash 116784 (+48 vs 116736, cross-TU call), RAM
  unchanged 9812. No bench check applicable (pure move; bench
  re-verification of the whole series remains queued for hardware return).

## 2026-07-12 — Phase D step 5: ExpoCurve + DeadbandPolicy extracted to src/domain/operator_input/ (#162)

- `expoCurve()` moved from `[JOYSTICK]` to
  `src/domain/operator_input/ExpoCurve.h/.cpp` with the SAME name and
  signature — the `.ino` definition was removed and every call site
  (including the characterization tests) resolves to the domain
  implementation directly; weights stay `[CONFIG]` parameters.
- `rcDeadband()` / `joyDeadband()` now delegate to one parameterized
  `centerSnapDeadband(value, center, width)` in `DeadbandPolicy.h/.cpp` —
  the two originals were the identical comparison with different constants.
  Arduino's `abs()` macro is expanded manually in the domain (equivalent
  for every reachable servo-µs/ADC value).
- Deliberately deferred: `InputNormalization` (entangled with
  `currentGear`/`reverseCap()` → extracts with GearPolicy/CommandMixer) and
  `sbusToServo()` (S.BUS protocol mapping on Arduino `map()` — reimplementing
  `map()` risks integer-rounding drift; infrastructure adapter territory).
- Verified: all 21 host binaries green (new `tests/operator_input/` suite:
  exact [CONFIG] weight-pair math at 0/0.5/1, magnitude-only symmetry,
  inclusive band edges for both RC and joystick parameterizations), zero
  expectation changes; compile clean — flash 116736 (+48 vs 116688,
  cross-TU calls), RAM unchanged 9812. No bench check applicable.

## 2026-07-12 — Phase D step 4: TelemetryScaling extracted to src/telemetry/ (#160)

- The X.BUS register decode math from `[TELEMETRY]` `telemApplyReg()`
  (VBAT ×0.1 V; IBUS ×0.1 A signed; MSPEED signed electrical Hz; TESC/TMOT
  low byte − 40 °C; the EMA fold) and the compact wire encode from `[WIFI]`
  `buildTelemJson()` (volts/amps → deci-integers via `lroundf(v*10)`,
  temperatures → whole °C, electrical Hz ×30 → dashboard RPM) moved to
  header-only `src/telemetry/TelemetryScaling.h` — first file in the
  observer `telemetry/` layer; the two `.ino` functions stay as callers
  delegating each expression. Observable behavior and wire format preserved
  exactly.
- Verified: all 20 host binaries green (new `tests/telemetry/` pure suite:
  register signedness, low-byte temperature mask, EMA seeding, half-away-
  from-zero deci rounding), zero expectation changes; compile clean — flash
  116688 BYTE-IDENTICAL to pre-change (inline header functions), RAM 9812
  unchanged. No bench check applicable (pure move).

## 2026-07-12 — Phase D step 3: thermal ladder extracted to src/domain/thermal/ (#158)

- `tempStageUpdate()` / `hottestMotorTemp()` / `thermalUpdate()` staging
  logic moved to `src/domain/thermal/` as `thermalStageStep()`
  (ThermalHysteresis), `hottestPlausibleTemperature()` +
  `thermalDeratingStep()` (ThermalDerating) over ThermalTypes structs —
  observable staging behavior preserved exactly; the shim-boundary deltas
  and one provably output-identical initialization change are itemized
  below for the audit trail.
  Thresholds/bounds stay in `[CONFIG]`, passed as parameters. The sketch
  keeps all three functions as delegate shims — all 28 test references
  unchanged. The firmware's three independent stage bools kept verbatim;
  the target glossary's ThermalStage enum is a later consolidation.
- Boundary work stays in the shims: clock read (time is a parameter),
  telem[]→ThermalReading flattening (per-ESC validity mirrored to both its
  sensors — comparison-identical to the original per-ESC loop), stage-global
  mirroring, and BEEP_THERM_RESTORED queued on the domain's cut-released
  return ([BEEPER] stays application-side, as with lowVoltLatched in #154).
- Same accepted non-observable delta as #154 (logged precedent): the shim
  reads `millis()` and builds the reading array unconditionally where the
  original early-returned on telemetry dropout. `hot` initialized to 0.0f
  (the #155-round lesson: never pass a possibly-unwritten out-param).
- New pure-domain suite `tests/thermal/test_thermal_domain.cpp` (8 cases)
  locks: trip at exactly since+debounce, timer reset on drop-back, the
  hysteresis HOLD inside the on/off gap incl. exactly-OFF (behavior law),
  immediate release below OFF, plausible-hottest selection with inclusive
  band edges and unwritten out-param on reject, dropout-holds-every-stage
  (timers too), cut-released edge fires exactly once, stage independence at
  86 C. Makefile: thermal/ glob + no-stubs rule mirroring battery/.
- Verified: full host suite green before/after, zero expectation changes;
  compile clean — flash 116688 (+200 vs 116488), RAM unchanged 9812. No
  bench check applicable (pure move).
- Review fix (Copilot): the hottest-selection running max now starts at
  `plausibleMinC` instead of the original `-1000.0f` sentinel — provably
  output-identical for every input (accepted readings are all >= minC), but
  correct for any parameter range now that the bounds are parameters (the
  sentinel was only safe with the fixed -20 °C constant). Unit-locked with a
  below-sentinel-band test case.

## 2026-07-12 — Documentation-sync hook + autonomous merge protocol (#156)

- **Documentation-sync hook** (operator directive): `PostToolUse` hook in
  `.claude/settings.json` runs `scripts/documentation_sync_hook.py` after
  every Edit/Write. When the file matches the watched paths
  (sketches/**/*.ino|.h|.cpp, dashboard/index.html, scripts/*.py,
  .github/workflows/*.yml, tests/** — excluding web_page.h,
  arduino_secrets.h, src/generated/, tests/build/, vendor/), it injects a
  once-per-session prompt telling the agent to verify docs/wiki +
  architecture docs + CLAUDE.md file map against the change in the same PR.
  Selftest (10 checks) wired to new `hooks-selftest.yml` CI.
- **Mechanism finding:** PostToolUse hooks that print plain stdout never
  reach the model (transcript-only) — the three pre-existing prompt-style
  PostToolUse hooks in `.claude/settings.json` are silently inert. The new
  hook uses the working `hookSpecificOutput.additionalContext` JSON form.
  Converting the old hooks is left to the operator (they would start firing
  for the first time).
- **Autonomous merge protocol** (operator, same day): after a PR's review
  rounds are resolved, 3 consecutive clean Copilot request→no-comments
  cycles (~5 min apart) + CI green + zero unresolved threads ⇒ the agent
  merges (squash, admin bypass of the 1-approval rule — bots cannot approve)
  and continues to the next issue. First use: PR #155 (c595722).

## 2026-07-10 — Phase D step 2: BatteryLadder extracted to src/domain/battery/ (#154)

- `batteryEcoLockUpdate()` / `batteryCutoffUpdate()` logic moved VERBATIM to
  `src/domain/battery/BatteryLadder.h/.cpp` as `batteryEcoLockStep()` /
  `batteryCutoffStep()` operating on a `BatteryLadderState` struct.
  Thresholds/debounces stay in `[CONFIG]`, passed as parameters. The sketch
  keeps both functions as delegate shims mirroring the existing globals
  (all five already in `resetFirmwareState()`), so every call site and test
  reference is unchanged.
- **Time is a parameter** introduced (state-ownership rule): domain code
  takes `nowMs`; the shim reads `millis()`. Delta accepted as non-observable:
  the shim reads `millis()` once per un-latched pass where the original read
  it only while below threshold (a register read, no behavior effect).
- **State-ownership sin fixed structurally, not behaviorally**: the domain
  cutoff step RETURNS "just latched" instead of writing [ALERT]'s
  `lowVoltLatched`; the shim asserts the alarm latch in the same pass at the
  same transition — alarm still starts WITH the cut.
- Shim early-returns preserve the original property that a latched stage
  reads no telemetry/clock.
- New pure-domain suite `tests/battery/test_battery_ladder_domain.cpp`
  (basename kept distinct from `characterization/test_battery_ladder.cpp` —
  flat-build duplicate guard) locks debounce boundaries (latch at exactly
  start+debounce), latch permanence, recovery/implausible timer resets,
  strict `<` thresholds, boot-gate confirmation at exactly 10.0 V, and the
  just-latched signal firing exactly once.
- Verified: full host suite green before/after with zero expectation changes;
  compile clean — flash 116472 bytes (+176 vs 116296: cross-TU calls + state
  mirror), RAM unchanged 9812. No bench check applicable (pure move).

## 2026-07-09 — Phase D step 1: VoltagePlausibility extracted to src/domain/battery/ (#151)

- First #117 extraction: `worstPackVoltage`'s logic moved VERBATIM to
  `src/domain/battery/VoltagePlausibility.h/.cpp` (+ `BatteryTypes.h` with
  `BatteryReading{voltage, valid}`). Plausibility bounds are parameters —
  the domain function has no config/global dependency. The sketch keeps
  `worstPackVoltage()` as a one-line delegate, so both ladder call sites and
  all existing test references are unchanged.
- The [ALERT] inline plausibility check stays put: its `&&`-form REJECTS NaN
  while the extracted `||`-form PASSES it (locked #131 FINDING) — the target
  architecture's "ONE implementation" consolidation is a behavior change
  requiring the NaN-gap issue first.
- Harness: test binaries now also compile `sketches/rc_test/src/**/*.cpp`;
  new pure-domain suite dir `tests/battery/` (no stubs, no firmware);
  duplicate-basename guard generalized across all suite dirs.
- Verified: full host suite green before/after with zero expectation changes;
  compile clean — flash 116296 bytes (+48 vs 116248: extracted function is a
  cross-TU call now), RAM unchanged 9812. No bench check applicable (pure
  move). architecture-fitness runs its first real src/ tree: OK.

## 2026-07-09 — Composition-root .ino rule deferred to a step-11 marker (#150)

- `check_ino` (`.ino` → `application/` only) fired the moment `src/` exists,
  which would have turned architecture-fitness red for the entire Phase D
  migration (steps 1–10: the `.ino` is the interim application shell and must
  include extracted `domain/` headers plus its existing external headers).
- Fix: the rule activates only when `src/application/FirmwareApp.h` exists —
  the exact artifact of migration step 11 ("there IS a composition root").
  Until then the checker emits a NOTE; extracted `src/` files are fully
  checked from step 1 regardless. Selftest covers both directions.

## 2026-07-09 — Wi-Fi credentials out of source: arduino_secrets.h pattern (#125)

- `WIFI_SSID`/`WIFI_PASS` now initialize from `SECRET_WIFI_SSID`/
  `SECRET_WIFI_PASS` in `sketches/rc_test/arduino_secrets.h` — gitignored
  (`sketches/*/arduino_secrets.h`); committed template:
  `arduino_secrets.h.example` (placeholders only). Both CI compile paths
  (`arduino-ci` matrix, `unit-tests` host suite — it compiles the real
  rc_test.ino) copy the template before building.
- Behavior-preserving: the local gitignored file carries the current real
  credentials, so locally-built firmware is unchanged. **Operator action
  pending: rotate the AP password on the next flash — the old one remains
  in git history.**

## 2026-07-08 — V7.35: FIRMWARE_VERSION single source of truth + docs-drift audit (#124)

- **Code:** `const char FIRMWARE_VERSION[] = "V7.35"` at the top of
  `[CONFIG]` — the only place the version is defined. `debugInit()` banner
  reduced from a 300-char changelog string to version + short tag
  (changelog belongs to git + FIRMWARE-UPLOAD-LOG). Authorized by #124.
  Dashboard version display: deferred (nice-to-have; would touch the SSE
  frame budget + dashboard UI test cycle — its own change when wanted).
- **Docs drift fixed (audited against [CONFIG]):** README "Turbo"→Boost
  (twice); "×1.05" joystick boost claim was wrong (JOY_THROTTLE_GAIN is a
  different value — README now describes behavior without the number);
  stale reverse-cap percentages (pre-#113 50%/62.5%) removed; gear table
  de-numbered. PROJECT-PLAN stale "Status (V7.14)" header + file-map
  version replaced with pointers to the SSOT.
- **New rule (CLAUDE.md Style):** README describes behavior, never tunable
  numbers — current values live in [CONFIG]. PROJECT-PLAN and OPERATOR-GUIDE
  are technical references: numbers allowed there but must match [CONFIG]
  (drift is a doc bug). OPERATOR-GUIDE table audited: already correct
  (65/80/100, 55/55/65, 72.5/60/60).

## 2026-07-08 — web_page.h generated from dashboard/index.html (#120)

- **What:** `scripts/generate_web_page.py` wraps `dashboard/index.html`
  verbatim (LF-normalized) into the PROGMEM raw-string in
  `sketches/rc_test/web_page.h`, with a GENERATED header, a 100 KB page
  budget gate (epic #81), and a raw-delimiter collision guard. `--check`
  mode byte-compares; new `dashboard-drift` CI job (structure-check
  workflow) fails any PR where the two files diverge.
- **Drift found and resolved:** the hand-maintained mirror lacked three
  JS comment blocks that had been added to the source (SoC curve, battery
  colour bands, safety banner) in the same commits — proven comment-only by
  byte-comparing comment-stripped payloads (identical). Regenerated
  web_page.h picks them up: served bytes +~620 (42.1 KB total, well under
  budget), ETag auto-updates, rendered dashboard and machine behavior
  unchanged. Compile clean (44% flash), host suite green.
- **Rule change:** web_page.h is never hand-edited again
  (`.claude/rules/dashboard.md`); the manual "keep in sync" discipline is
  retired in favor of the generator + CI check.

## 2026-07-07 — Wiki lint in CI (#145) — the Karpathy "lint" operation

- **What:** `scripts/check_wiki.py` + self-test, run by the `wiki-lint`
  workflow: link integrity, orphan notes, reachability from `home.md`, and a
  tunable-drift heuristic (numbers with units fail — the #141 anti-drift rule
  is now machine-enforced). Completes the Karpathy LLM-wiki operation triad:
  ingest (same-PR note updates), query (reading/Obsidian), lint (this).
- Pattern source verified: Karpathy's `llm-wiki.md` idea-file gist (Apr
  2026) — three layers (raw/wiki/schema) + three operations. Our mapping:
  canonical docs = raw, docs/wiki = wiki, README rules = schema, git
  history + this log = his log.md.

## 2026-07-07 — Architecture fitness functions in CI (#129, Phase C)

- **What:** `scripts/check_architecture.py` + `architecture_rules.py`
  (stdlib-only, identical locally and in CI) enforcing ARCHITECTURE-TARGET §3
  on every `sketches/<name>/src/` tree: forbidden includes + layer direction
  (incl. the observer-ring SystemSnapshot exception), file-size 150/250 with
  `.architecture-allowlist` (reason required), banned names, no mutable
  namespace-scope state in `domain/` (heuristic, namespace-transparent brace
  tracking), generated-file guard (dormant until #120), FIRMWARE_VERSION SSOT
  (dormant until #124). New `architecture-fitness` workflow runs
  `check_architecture_selftest.py` first — 10 deliberate violations must fire
  before the real check counts.
- **Dormant by design:** checks target `src/` trees only; bench sketches and
  the pre-Phase-D `rc_test` are exempt, so CI is green today and the rules
  activate file-by-file as Phase D extracts code.
- **Advisory → required:** not in branch protection until the Phase D
  extraction lands (per #129), then flipped.

## 2026-07-07 — AI-maintained knowledge graph at docs/wiki/ (#141, Karpathy LLM-wiki)

- **What:** `docs/wiki/` — 19 link-dense Markdown notes (root hub, 7 domain
  hubs, 11 concept notes) forming a graph layer over the canonical docs,
  visualizable in Obsidian (vault = repo root; `.obsidian/` gitignored).
- **Anti-drift rules (the design):** notes carry stable concepts and links
  ONLY — constants/thresholds/pins stay exclusively in canonical docs; a wiki
  note restating a number is a bug. Standard relative Markdown links (no
  `[[wikilink]]` syntax) so notes read on GitHub and lint clean; Obsidian
  graphs them natively. Agents own the folder: doc-changing PRs update
  affected notes in the same PR.
- **Placement:** `docs/wiki/` not top-level `wiki/` — structure-check CI
  restricts `.md` locations to root/docs/.github/.claude; no CI change
  needed.

## 2026-07-06 — Karpathy method adopted repo-wide; #53 re-prioritized FIRST; behavior-preservation covenant codified

- **Reprioritization (operator):** #53 moves from Phase E to NOW — before any
  Phase C/D refactoring. Rationale: the guardrails must exist before the
  code-moving starts, or the refactor risks destroying the field-tuned
  behavior it exists to preserve.
- **The covenant (operator, verbatim):** "organizing, refactoring and
  improving the code should never ever, ever cause change of controls or
  change of behavior unless we explicitly discuss it … we're not messing with
  the timing, the behavior, the parameters. Nothing should be touched.
  Everything that was handling inputs and outputs should remain exactly the
  same … if we upload the firmware again, I would like the machine to perform
  exactly like it used to perform before." Codified as
  `.claude/rules/behavior-preservation.md` (highest-priority rule).
- **Pragmatism amendment (operator):** severity triage, not dogma — blatantly
  dangerous findings get raised immediately and fixed in their own dedicated
  PR (safety outranks the covenant); bounded/latent findings get locked in
  tests + a follow-up issue; odd-but-intended behavior is law.
- **Fix protocol (operator, final form):** a behavior fix must be visible,
  discussed, deliberate, and have its OWN GitHub issue (what/why/how) created
  BEFORE the fix; never inside any other PR, never as a PR-review correction
  — no exceptions.
- **Surfaces:** CLAUDE.md Working Agreement (4 principles + goal-and-gate
  pattern wired to the real CI workflows), `.coderabbit.yaml` (NEW —
  epic-scoped path_instructions so bot reviews stop proposing behavior
  changes; TEMPORARY, relax at epic end), `.github/copilot-instructions.md`
  covenant section + principle mirror, `docs/AGENT-EXAMPLES.md` (5 good/bad
  pairs from real project history: Max-Reverse-Force checklist, the 1825 µs
  probe refutation, the NaN gap lock-don't-fix, failing-test-first, goal+gate
  framing).

## 2026-07-06 — Safety-invariant + fault-injection suite; two firmware gaps found and locked (#131)

- **Suite:** `tests/invariants/` — 6 property suites (35 test cases, ~800
  assertions) on the #47 harness, one binary per file, auto-discovered by the
  extended `tests/Makefile`. `InvariantChecks.h` provides the reusable
  checker: `runControlPasses()` drives the real `loop()` and re-checks the
  universal invariants (`checkInvariantsNow()`) after EVERY pass.
- **Registry decision:** invariants listed verbatim in **`docs/SAFETY.md`**
  (the #131 acceptance's first-named option) — survives the
  `rc_test → dual_track_control` rename; ARCHITECTURE-TARGET §11 and the
  future #126 `docs/ARCHITECTURE.md` link to it instead of duplicating.
  #69 (Wi-Fi stall runaway) recorded there as the motivating incident.
- **Verified invariants (all hold):** outputs within [1000, 2000] µs for any
  input incl. NaN/Inf (double constrain — the int-domain clamp stops even
  non-finite floats); stale/failsafe RC ⇒ neutral in every override mode with
  the joystick railed; cutoff latch ⇒ monotone ramp to SVC within 500 ms then
  detach, permanent through voltage recovery; boot with 9.5 V packs never
  emits one non-neutral pulse (1.5 s cutoff debounce beats the 3 s fail-open);
  thermal derating monotone (2000 → 2000 → 1825 → cut) with dropout holding
  the active stage; sub-debounce voltage/temperature spikes rejected with
  timer reset; gear/reverse caps hold on the AVERAGE track command across a
  675-combination sweep.
- **Gap 1 (locked, not fixed): NaN pack voltage** passes the [6, 13] V
  plausibility band (NaN comparisons all false); `worstPackVoltage()`'s
  ternary makes it slot-asymmetric — slot-1 NaN freezes the ladder, slot-0
  NaN is masked by the healthy pack and can set `batteryOkConfirmed`.
  Unreachable via the uint16 X.BUS parse path today. Temperature path immune.
- **Gap 2 (locked, not fixed): reverse-cap transient on gear upshift** — full
  reverse joystick held through Eco/Normal→Boost keeps the stale Eco-domain
  clamp (−0.846) for ≤ one 10 ms joystick-cache window: ~1077 µs average vs
  the 1175 µs Boost floor for up to 10 ms (pass count scales with loop rate).
  RC path re-clamps every pass; joystick cache does not.
- Both gaps documented in `docs/SAFETY.md` known-gaps; follow-up hardening
  issues drafted (operator to file); fixes are behavior changes requiring
  their own issues + updated test expectations.

## 2026-07-03 — Pivot-branch throttle taper CONFIRMED working (#114, V7.34)

- **Problem (field-observed):** feeding 10–20% throttle while steering shifted
  both tracks forward together (mirrored in reverse) — the machine surged out
  of a pivot instead of easing. Cause: throttle entered both tracks of
  `curvatureDrive()`'s pivot branch at full gain; the #72 blend-band width
  cannot fix this (both branches carry the same full-gain forward term).
- **Fix:** `pivotThr = xSpeed * (1 − PIVOT_THROTTLE_TAPER·|zRotation|)` with
  `PIVOT_THROTTLE_TAPER = 0.70` in [CONFIG]. Outer track keeps most of its
  pivot speed, inner keeps a slight counter-rotation that eases through zero;
  the machine tightens into an arc. **Operator-confirmed feel on the machine
  (2026-07-03): "really good."**
- **Proven invariants (swept, max |old−new| = 0.000000):** straight line
  fwd/rev (z = 0), pure pivot (x = 0), at-speed turns (|x| ≥ PIVOT_BLEND_END
  0.55). Reverse caps (#113), outer headroom (#72), turn ceiling (#96), and
  reverse-steer consistency (#86) untouched.
- **Deliberate choices:** taper follows the RAW steering stick (not
  `cappedRotation`) — past the pivot cap extra deflection adds no rotation but
  still means "hold the pivot", so full lock holds hardest. `fabsf(z)`
  introduces a slope kink at z = 0 (≤ ~1–2% output per 0.1 z mid-band) —
  value-continuous, judged imperceptible; revisit with a z² taper only if ever
  felt. Range [0,1] enforced by `static_assert` (above 1.0 the term flips
  sign: forward stick would command reverse).
- `tools/drive-trace.mjs` re-synced to the V7.34 math (#86 additive delta,
  #96 ceiling, #114 taper) — it had drifted three revisions behind the
  firmware.

## 2026-07-03 — GL10 throttle endpoints RECALIBRATED; reverse cap now per-gear (#113)

- **Root cause of reverse overshoot (field-observed):** the GL10s had been
  throttle-calibrated while the firmware capped reverse at 65%, so each ESC
  learned **65% PWM (1175 µs) as its 100% reverse endpoint**. A 65%-commanded
  reverse therefore drove ~100% motor. Straight-line, no steering — NOT the
  turn-headroom path. Confirmed by the operator.
- **Fix:** flashed a temporary calibration build (`CALIBRATION_MODE=true`) that
  passes the throttle stick straight to the full ±100% range (1000/1500/2000 µs)
  on both tracks, all caps/gear/steering bypassed. Operator recalibrated both
  ESCs (Option A, GL10-OPERATION §5) — **"lo, ro, do" ×4 success tone on both.**
  ESCs now know the true 1000/1500/2000 endpoints.
- **Decision:** reverse cap is now **per gear** via a single `reverseCap()` helper:
  **55% Eco/Normal, 65% Boost**. These are TRUE percentages post-recalibration.
- **Rule reinforced:** never lower the firmware reverse cap AND recalibrate at the
  same time, or the ESC re-learns the capped value as 100%.

## 2026-07-03 — Motor/ESC thermal cutoff is PROVISIONAL at 95 °C (#111)

- Staged thermal ladder shipped (V7.31/V7.33): warn 80 / Eco 90 / hard cutoff 95 °C,
  non-latching with hysteresis (rel 78/80/75), hottest-of-4 sensors, EMA + 1 s
  debounce, telemetry-dropout holds state (never cuts).
- **95 °C cutoff is PROVISIONAL** — must be bench-confirmed to sit *below* the
  GL10's own internal thermal limit (param 17 is a temp-controlled fan; the ESC's
  internal throttle/cutoff number is unpublished) so our warned graceful cut fires
  before the ESC's silent one. Do not rely on 95 °C until verified on the bench.

## 2026-06-29 — Eco lock + PWM ease-out cutoff CONFIRMED working (#65)

- Test build (eco 11.3 V / cutoff 11.1 V): worst-pack EMA ≤ 11.3 V for 15 s →
  gear forced to **Eco** regardless of RC switch + dashboard amber "ECO LOCKED"
  banner. **Operator-confirmed "works perfectly."**
- **PWM ease-out:** on cutoff/RC-loss the output gate now ramps the live track
  command down to neutral over CUTOFF_HOLD_MS (500 ms) before detaching → gentle
  controlled stop, no jerk. Confirmed good feel.
- Production thresholds: Eco lock **11.0 V**; hard cutoff **10.0 V** (recommended;
  3.33 V/cell on 3S, conservative). 10.5 V is the more battery-protective option.

## 2026-06-28 — Reverse turning is symmetric with forward (#87, #96)

- Decision: reverse is treated the same as forward in `curvatureDrive()`.
  `REVERSE_CAP` (0.65) bounds the reverse **average** speed; the outer track then
  borrows the same `TURN_TRACK_CAP` turn headroom forward uses, so a reverse turn
  swings precisely instead of slowing/snapping (smoothness mission).
- So the reverse outer track may exceed 65% **during a turn** — by design,
  mirroring forward. "Reverse 65%" = the straight-line/average reverse speed cap.
- V7.19 added a reverse-specific ceiling clamp (outer ≤ 65% in reverse turns) per
  a Copilot review; V7.20 (SHA 53abb4a) reverted it on operator decision — simpler
  (no reverse special-case) and consistent forward/reverse.
- **Field finding (operator):** reverse at 50% was severely underpowered on slight
  uphill / grass — couldn't climb. This is why the reverse cap was raised to 65%
  (REVERSE_CAP) and headroom kept; reverse confirmed working on terrain at 65%.

## 2026-06-28 — V7.16 dynamic outer-track turn cap (#96) — CONFIRMED working

- curvatureDrive fades the outer-track ceiling from the ESC rail (straight) down to
  `TURN_TRACK_CAP = 0.70` at full steer, by |zRotation| (open-loop).
- Bench-confirmed: joystick full throttle + full steer no longer pins one track to
  ~99% while the other is stopped; outer caps ~70%, gentle turns keep #72 headroom.
  Operator: "working really well." 70% kept as the field-test value.
- Also in same build (V7.15, #90): RC+joystick mix moved to axis level (single
  operator keeps full range; maxOppose dual-mix; joystick Boost cap).
- **Operator-confirmed working (2026-06-28, PR #93, V7.18):** shared-control mix
  (#90), dynamic outer-track turn cap 70% (#96), joystick per-gear caps 65/75/90
  with gain 1.40 (#87), and flat 65% reverse (#87) all behaving as intended on
  the bench. PR #93 marked ready for review.

## 2026-06-20 — SAFETY (P0): Wi-Fi serving stalled control loop → runaway under load

- Loaded test (~60 lb): digger made **uncommanded movement (runaway toward operator)**
  during Wi-Fi/dashboard activity. Injury hazard.
- Cause: dashboard serving (~36 × 1 KB chunked page writes + SSE) blocks `loop()` ~1–2 s
  on the shared RA4M1 core. Servo PWM is hardware-timed and holds the last pulse, so the
  ESC keeps executing the **last throttle command** while `loop()` — and the in-loop
  RC-lockout failsafe — is frozen.
- `modem.timeout(50)` caps each modem call but NOT the cumulative ~36-call page send.
- Failsafe required (tracked P0 in #69): hardware WDT (~250 ms) refreshed only in the
  control section → MCU reset to neutral if the loop is starved; bound Wi-Fi work per
  loop pass; fail-to-neutral latched. Permanent fix = #55 (ESP32-S3 offload).
- "Stop unless Wi-Fi client connected" is the WRONG mechanism — the hazard occurs WHEN a
  client is connected (that's what blocks the loop); no client = clean loop.
- Interim rule: do NOT load/refresh the dashboard while driving under load.

## 2026-06-20 — Dashboard page-load froze control loop; fixed by HTTP caching

- **Confirmed via USB-serial capture:** with no Wi-Fi client, loop runs clean
  ~10 Hz, both ESCs telemetry OK (12.4–12.5 V), X.BUS solid after the interface-
  board resolder (rx_total climbs steadily).
- **Confirmed problem:** loading/refreshing the dashboard froze the loop 1–2 s
  (24 stalls in 35 s, up to ~2 s), starving S.BUS reads, servo updates, and X.BUS
  polling (rx_total nearly flat during stalls).
- **Root cause:** 33 KB dashboard HTML served `Cache-Control: no-store` →
  re-downloaded every refresh as ~33 sequential 1 KB blocking modem writes. The
  SSE telemetry path was already numbers-only and fine.
- **V7.9 (SHA dabfbf1):** AP channel 11, SSE 10→5 Hz, coalesced SSE writes,
  `modem.timeout(50)` around `wifiUpdate()`. Caps each modem call at 50 ms but a
  full page is ~33 calls → still ~2 s per load.
- **V7.10 (SHA 4197350) — field-confirmed fix:** serve `/` with `ETag` +
  `Cache-Control: no-cache`; parse `If-None-Match`; reply `304` on match. Refreshes
  no longer re-download the page or freeze the loop. First load still ~2 s once
  (inherent to serving 33 KB on the RA4M1); eliminating that is the V9 ESP32-S3
  offload (#55).
- **Safety note:** during any Wi-Fi-induced loop freeze, servo PWM holds its last
  value and RC/joystick/failsafe are not serviced — do not load the dashboard
  while driving until #55 lands.

## 2026-04-14 — X.BUS protocol confirmed

- X.BUS is single-wire half-duplex, 115200 8N1, non-inverted (idle HIGH).
- Protocol is master-polled: slaves never transmit unsolicited.
- Manufacturer: Shenzhen XC-ESC Technology Co., Ltd (not Spektrum, not JR).
- Full protocol translated and documented in `docs/XBUS-PROTOCOL.md`.

## 2026-05-23 — X.BUS telemetry confirmed working on breadboard

- Both GL10 ESCs respond to polling with telemetry via X.BUS.
- Breadboard uses Schottky diode merge + pull-up resistor on shared bus.
- Arduino polls on D1 (TX), receives on D0 (RX), same physical bus node.
- Breadboard used Schottky diode merge + 4.7kΩ pull-up. Superseded by the interface board design (no diodes needed for master-polled X.BUS).
- Test sketch: `sketches/xbus_master/xbus_master.ino`.

## 2026-05-23 — Interface board design

- Purpose: replace breadboard X.BUS merge + S.BUS inverter with a soldered plug-and-play board.
- Build plan: `docs/INTERFACE-BOARD.md`.
- X.BUS merge: no diodes needed (master-polled, no bus contention). Both ESC yellows connect directly to bus node.
- Components: 4.7kΩ pull-up, 1kΩ TX series resistor (on board between J3 and bus), 2N3904 S.BUS inverter with 2× 10kΩ.
- Motor control via PWM (D9/D10 → ESCs direct), X.BUS for telemetry only (RPM, current, voltage, temp).
- X.BUS telemetry confirmed working on breadboard with both GL10 ESCs.
- Six servo connectors: 2× ESC telemetry IN, 1× TX to Arduino, 1× RX to Arduino, 1× S.BUS IN from receiver, 1× inverted S.BUS OUT to Arduino.
- D0 conflict: X.BUS and S.BUS both want Serial1 RX. Only one can be connected at a time until a dual-UART board is used.

## 2026-05-24 — UNO R4 WiFi migration: pin map, UART, power, 9-pin cable

- Migrating from Nano R4 to UNO R4 WiFi with Sensor Shield V5.0.
- **UART discovery:** On UNO R4 WiFi, SCI0 is on D11(TX)/D12(RX), NOT A4/A5 (those are I2C-only). Serial2 is reserved for WiFi ESP32-S3. Confirmed via ArduinoCore-renesas variant.cpp: D11=P411 (SCI0 TX), D12=P410 (SCI0 RX).
- **UART plan:** Serial1 (SCI2, D0/D1) for X.BUS 115200. sbusUart (SCI0, D12 RX) for S.BUS 100000 8E2. Both hardware UARTs, fully reliable.
- **Power:** GL10 BEC set to 7.4V → VIN. ISL854102FRZ buck converter → 5V on all rails. USB+VIN coexist (Schottky diodes). SEL jumper IN.
- **9-pin cable:** 5 signal (PWM L/R, XBUS TX/RX, SBUS) + 3 power (VIN, GND, 5V) + 1 spare. Joystick direct to shield A0/A1.
- **Full wiring spec:** `docs/WIRING-GUIDE-V8.md`.

## 2026-05-24 — GL10 throttle range calibration completed

- Both GL10 ESCs calibrated to match the same transmitter stick range.
- GL10 calibration sequence: full REVERSE on power-up → full FORWARD → NEUTRAL (not forward-first like generic ESCs).
- Confirmed success: red+green LEDs flash 4× with "so-mi-do" melody.
- Official procedure from GL10 User Manual Section 5 (`docs/GL10-Manual.pdf`).
- App name is TXC-Link (not XC-Link). Default Bluetooth password: 1234.
- Quick reference saved: `docs/GL10-QUICK-REFERENCE.md`.

## 2026-05-31 — S.BUS confirmed working on UNO R4 WiFi hardware

- `rc_test` V7.6 compiles for `arduino:renesas_uno:unor4wifi` (22% flash, 25% RAM) and runs on the board (COM7).
- S.BUS verified live on `sbusUart` SCI0 / D12 RX through the 2N3904 inverter. Frames valid, FS=0 / Lost=0.
- Inverter wiring fix: D12 (output) taps the **collector**; S.BUS-in from the receiver goes through 10 kΩ to the **base**; **emitter** to GND. Initial build had collector/base roles swapped (no signal); corrected.
- Verified end-to-end: throttle full range, steering, curvatureDrive differential on D9/D10, and Eco reverse cap producing 1375 µs (matches 1500 − (0.625 × 0.40 × 500) = 1375 µs, neutral offset included).
- X.BUS telemetry NOT active — `rc_test` does not poll it. Telemetry restore tracked in #36; Wi-Fi dashboard in #45.

## 2026-06-01 — Telemetry logging architecture + board-upgrade analysis (V8)

Research for the WiFi telemetry dashboard + black-box logging. Findings:

- **An SPI SD-card module cannot coexist with S.BUS on UNO R4 WiFi.** (The
  board has no onboard SD slot — this is about adding an SPI SD module.)
  Hardware SPI is on D11 (MOSI) / D12 (MISO) / D13 (SCK); `sbusUart` (SCI0) is
  on D11 (TX) / D12 (RX). SD-over-SPI lands on the exact S.BUS pins. No third
  hardware UART to relocate S.BUS to (Serial1/D0-D1 reserved for X.BUS). So an
  SPI SD card is not viable on this board without evicting S.BUS. (Clock/RTC is
  unaffected — I2C on SDA/SCL, or NTP over WiFi; WiFi is unaffected — ESP32-S3
  coprocessor.)
- **Stock UNO R4 WiFi offloads WiFi/TCP but NOT WebSocket serving from the
  control core.** The split: the ESP32-S3 runs the WiFi radio + TCP/IP stack
  (reached via the `WiFiS3` bridge). The RA4M1 runs the Arduino sketch AND the
  WebSocket layer — HTTP-upgrade handshake, frame (de)masking, ping/pong,
  server state machine — in libraries like UnoR4WiFi_WebServer / mWebSockets.
  That WebSocket state machine is serviced inside `loop()` on the RA4M1, the
  same core as the control loop, so a busy/blocked loop stalls it and vice
  versa. Net: TCP/WiFi is offloaded; WebSocket serving is not — it shares the
  control core and CAN perturb control timing. (The WiFiS3 bridge exposes only
  socket I/O, so the ESP32-S3's 8 MB flash is not addressable as RA4M1 sketch
  storage without custom ESP firmware. Corrects the assumption in #45 that
  serving a page can't affect control-loop timing.)
- **Agreed logging data model:** telemetry frames tagged with `millis()`
  (NOT `micros()` — micros wraps at ~71 min; millis at ~49.7 days) + a
  monotonic integer sequence number for dedup/backfill. Client takes one
  (arduino-ts, wall-clock) anchor per connection. X.BUS polling must be a
  non-blocking state machine, not a blocking request/wait.
- **Path A — custom ESP32-S3 firmware (split-brain):** runs WebSocket +
  LittleFS ring buffer on the 8 MB flash + dedup/backfill on the ESP32;
  RA4M1 fire-and-forget streams frames over the internal link. Hardware-safe
  and fully recoverable (ESP32-S3 mask-ROM bootloader can't be erased; force
  download mode via GND+Download pins on the 6-pin header, reflash with
  espflash; Arduino publishes the original bridge firmware). RA4M1 untouched
  by ESP32 flashing. CATCH: the ESP32-S3 is also the USB-serial bridge for
  programming the RA4M1 — overwriting it breaks sketch upload + Serial Monitor
  until the bridge firmware is restored. Recurring dev-loop friction.
- **Path B — Giga R1 WiFi:** STM32H747 dual-core (M7 480 / M4 240). Memory:
  1 MB RAM, 8 MB SDRAM, 2 MB MCU flash, 16 MB QSPI NOR (sketch-accessible).
  USB-A host for thumb-drive mass storage, 4 UARTs, onboard WiFi/BT. Programmed
  directly over USB-C (no bridge conflict). True offload of control vs I/O.
  COST: 3.3 V logic only — 5 V joystick needs level-shifting/divider, S.BUS
  inverter output swing must be re-verified; bigger enclosure + re-mount.
- **Capacity:** compact binary frame ~24-32 B → 8 MB ≈ ~250-300k frames
  ≈ ~15 h at 5 Hz.
- **Lean (at time of writing):** Giga for the full black-box + analytics goal.
  Path A is the no-new-hardware fallback. **Superseded — final decision is
  Option A (custom ESP32-S3 firmware); see the REVISED DECISION below.**

## 2026-06-01 — DECISION: stay on UNO R4 WiFi, stock config (Option C) — SUPERSEDED

> **Superseded** by the REVISED DECISION (Option A) below: black-box recording
> that survives WiFi drops is required, which stock Option C cannot provide.

- Operator chose **no new hardware / no mechanical change**. Giga (Option B)
  declined. Onboard black-box logging deemed non-essential for now → Option A
  (custom ESP32-S3 firmware) also declined.
- **Plan = Option C:** stock firmware, sketch on RA4M1, WiFi via `WiFiS3`,
  WebSocket telemetry streamed to a client; **client-side logging** + per-
  connection wall-clock anchor; `millis()`+sequence frame tagging retained.
- **Caveat:** WebSocket/WiFi serving runs on the RA4M1 control core — must be
  engineered non-blocking (1-5 Hz, fire-and-forget, time-sliced) so control
  timing is unaffected. If too jittery under load, fall back to A or Giga.
- **Optional:** small RA4M1 RAM ring buffer (~tens of seconds) for reconnect
  gap-tolerance across brief WiFi drops — not a full-session black box.
- **Accepted tradeoff:** data recorded only while a client is connected; no
  always-on onboard recording. Acceptable for tuning/monitoring sessions.
- **Clarification:** "bridge friction" = dev-time only (flashing the ESP32-S3
  removes the USB upload/Serial bridge until stock firmware is restored);
  unrelated to telemetry data loss, and NOT incurred under Option C.

## 2026-06-01 — REVISED DECISION: Option A (custom ESP32-S3 firmware)

Black-box recording (buffer + backfill, survives WiFi loss) is required, so
Option C is insufficient. Chosen path:

- **Option A:** RA4M1 runs control only; ESP32-S3 runs WiFi + WebSocket +
  backfill + ring buffer on its 8 MB flash. True dual-CPU offload — telemetry
  feels real-time, no blank spots. No new board; keeps 5 V + sensor shield.
  (No 5 V alternative exists with this feature set — Giga/ESP32/Teensy are all
  3.3 V; UNO R4 WiFi is the only 5 V board with a second onboard CPU.)
- **ESP32-S3 does WiFi AND our app simultaneously** (dual-core 240 MHz) — the
  WiFi job is unaffected.
- **Dev-update friction (control sketch / RA4M1):** jumper → restore stock
  bridge → upload sketch over USB → jumper → re-flash our ESP firmware.
  Normal RA4M1 sketch upload is automatic over USB and needs NO jumper, but
  only while the stock bridge is present; flashing custom ESP firmware
  REQUIRES the GND+Download jumper (physical).
- **Jumper requires physical access** to the board → mitigated by wiring the
  GND + Download pins to a **panel-mounted switch/header** on the enclosure.
  Then the cycle = flip switch + USB + run script (`arduino-cli`/`espflash`),
  no disassembly. Switch is the only manual step; flashing is scriptable.

## 2026-06-11 — Telemetry bring-up bench check (issue #36) — pre-flash baseline

Goal this session: verify X.BUS telemetry comes through from BOTH GL10 ESCs
on the new (post-migration) circuit, before enabling it over Wi-Fi (#45).
Scope intentionally limited to docs/memory/issue — no sketch or control changes.

- **Board confirmed:** Arduino UNO R4 WiFi on **COM7** (`arduino-cli board list`).
  Nano R4 → UNO R4 WiFi migration is complete; CLAUDE.md still lists the old
  Nano R4 / COM8 (stale, not corrected this session).
- **Flashed sketch = `rc_test` V7.6 (PWM-only), UNCHANGED.** Working tree clean
  (only untracked `dashboard/index.html`, unrelated #45 work). No edits to
  `rc_test.ino` or any control code. Control logic intact.
- **Live CSV columns are `RCThr,RCStr,RC4,RC5,JoyY,JoyX,OutL,OutR,Gear,FS,Lost`.**
  The trailing `0,0,0` are Gear/Failsafe-count/Lost-frames — **NOT telemetry.**
  `rc_test` does not poll X.BUS, so nothing reads the telemetry line yet.
- **Control side healthy:** S.BUS FS=0 / Lost=0 over a multi-second capture;
  RC channels live, joysticks centered (~8192 on 14-bit ADC), OutL/OutR mixing
  on D9/D10.
- **Telemetry NOT yet verified.** The circuit is wired on **D0/D1 (Serial1)**,
  unchanged through the migration, but cannot be confirmed until a poll sketch
  is flashed. Verification is the next step (deferred — no flash this session).
- **#36 "Options to evaluate" (bridge MCU / relocate S.BUS / switch board) are
  OBSOLETE.** They assumed Nano R4 UART scarcity. On UNO R4 WiFi both hardware
  UARTs are free: **Serial1 (D0/D1) → X.BUS @ 115200**, **sbusUart (SCI0/D12)
  → S.BUS**. Go straight to the telemetry scope.
- **Confirmed telemetry approach (unchanged from plan):** Read Register (func
  **0x10**), NOT Throttle (0x50 forces BUS_MODE and fights PWM). Registers
  0x0C VbatBus, 0x20 mos-Tem, 0x22 mot-Tem, 0x02 motSpeed (RPM). 1 Hz
  alternating per ESC, EMA + freshness watchdog. Note: 0x10 is documented-safe
  but **not yet confirmed on GL10 hardware** (only 0x50 has been, on breadboard
  2026-05-23) — flashing a 0x10 poller doubles as that confirmation.
- **Issue alignment:** work = **#36** (restore telemetry) → **#45** (Wi-Fi
  dashboard). **#48** (ESP32-S3 / Giga black-box logging architecture) is
  explicitly set aside — not part of this work.

### Telemetry TEST RESULT (flashed `sketches/telem_check`, read-only)

Flashed a read-only telemetry tool (proven 0x50 framing, **throttle hard-wired
to 0** — never drives motors) to confirm the new circuit. Motors did not move
(Thr_out=0%, RPM=0 throughout).

- **ESC0: telemetry confirmed working.** V=**11.9 V** (plausible 3S at rest),
  status flags **BUS_MODE + CAP_CHARGED**, frames steady. The X.BUS link on
  D0/D1 (Serial1) on the UNO R4 WiFi works.
- **ESC1: NO RESPONSE** — "no data yet" for the entire run. Good/Bad frame
  ratio held ~50/50 (every poll targeting ESC1 timed out), confirming exactly
  one of two ESCs answers.
- **Likely causes for ESC1 (to chase next):** (a) ESC1's X.BUS yellow not
  landed on the bus node / cold joint on the new board, or (b) both ESCs share
  X.BUS address 0 (register 0x05) so ESC1 isn't individually addressable — each
  ESC needs a distinct X.BUS ID (0 and 1) set in TXC-Link. Breadboard test
  2026-05-23 had both responding, so a new-circuit wiring fault is the leading
  suspect.
- **Decode bug (cosmetic, not hardware):** ESC-temp prints garbage (~15898 °C)
  because it's decoded as int16 `d[14]|d[15]<<8`. Raw bytes suggest ESC temp is
  a single byte (~62 → 22 °C ≈ room temp). Fix the telemetry temp decode to
  1-byte before the real `rc_test` integration; voltage decode (offset 12–13)
  is correct.
- **Flight firmware (`rc_test` V7.6) must be re-flashed** to restore the
  machine after this bench check — `telem_check` is a diagnostic only.

### V7.7 — telemetry integrated into flight firmware (telemetry WITH control)

Added non-blocking X.BUS **Read Register (0x10)** polling into `rc_test`
(`[TELEMETRY]` module on `Serial1`/D0-D1). Read-only — 0x10 is point-to-point
service control, never enters BUS_MODE, so PWM/S.BUS control authority is fully
preserved. Polls ESC0/ESC1 alternately ~6 Hz each (80 ms gap, 12 ms non-blocking
timeout), EMA-smooths V/I/temps, RPM instantaneous, 5 s per-ESC freshness watchdog.

- **0x10 CONFIRMED WORKING on GL10** (first time — previously only 0x50 was).
  ESC0 returns clean values: **V=11.9 V, ESC 28 °C, motor 22 °C, RPM tracking
  throttle (7→38 Hz electrical as stick went 1766→2000 µs).** Per-register 0x10
  decode also fixes the bogus temp seen via 0x50 — temps now read room-plausible.
- **Control fully intact alongside telemetry:** FS=0 / Lost=0, OutL/OutR follow
  the stick. Confirms telemetry-with-control is real (not telemetry-only).
- **ESC1 still silent** (OK1=0) — same as the read-only bench test. Blocks
  per-track capture of the 2nd track. Chase: ESC1 X.BUS yellow on the bus node,
  or distinct X.BUS address (reg 0x05) — both ESCs may be at default addr 0.
- **Bus current reads 0.0 A** at idle/unloaded — verify under load (reg 0x0D).
- **CSV format changed** (V7.7): appended `V0dV,I0dA,RPM0,TE0,TM0,OK0` +
  `…1` columns (integer-scaled). `live_plot.py` / `monitor.py` / dashboard
  parsers must be updated for the new columns.
- Firmware flashed to UNO R4 WiFi / COM7 (58.9 KB, 22% flash / 26% RAM).

## 2026-06-13 — BOTH ESCs telemetry confirmed under load (V7.7)

After a **full power cycle** of the board (USB/board power removed + reset,
not just the reset button), **ESC1 telemetry now responds** — both ESCs report.

- Full-throttle Turbo, both tracks driven: **ESC0** V=11.7 V, I=3.3 A,
  RPM=200 Hz, ESC 25 °C, motor 22 °C; **ESC1** V=11.7 V, I=3.2 A, RPM=197 Hz,
  ESC 23 °C, motor 21 °C. OK0=OK1=1.
- **Good left/right symmetry** at matched command (200 vs 197 Hz, 3.3 vs 3.2 A)
  — bears on the open track-asymmetry item; looks balanced here.
- **Bus current (reg 0x0D) works under load** (~3.2–3.3 A). The earlier 0.0 A
  was genuine no-load idle, not a decode bug.
- **RPM tracks throttle on both ESCs** (ESC0 76→200, ESC1 56→197 during ramp).
- ESC1's prior silence was a transient bad state (likely leftover BUS_MODE from
  the earlier 0x50 misstep), cleared by the power cycle — NOT confirmed as a
  permanent wiring/address fix. If ESC1 goes silent again, check its X.BUS
  yellow on the bus node and X.BUS address (reg 0x05).
- Op note: after USB unplug, the UNO R4 WiFi only re-enumerated as COM7 after a
  full power cycle of the ESP32-S3 USB bridge (RA4M1 reset alone didn't).

## 2026-06-13 — Wi-Fi telemetry dashboard live (V7.8, #45)

Added a Wi-Fi AP + HTTP telemetry server to `rc_test` ([WIFI] module, WiFiS3)
and rewired `dashboard/index.html` from simulation to live `/data` polling.

- **AP mode** SSID `Digger-Telemetry`, pass `<redacted — #125, rotated;
  see arduino_secrets.h>` (WPA2), IP
  **192.168.4.1**. `WiFi.beginAP` returns status 7 (WL_AP_LISTENING) on boot —
  AP confirmed broadcasting; telemetry both ESCs OK while AP up.
- **Server:** `GET /data` → compact JSON (millis `t`, `seq`, gear, mode, fs,
  lost, outL/outR, per-ESC ok/rpm/cur/v/tE/tM, integer-scaled). CORS `*`.
  `GET /` → plain-text info line. **Monitoring only — no control input over
  Wi-Fi** (safety, matches #45 decision).
- **Non-blocking design:** one client transaction per loop pass, 20 ms bounded
  request-read window, tiny payload. Servo PWM is hardware-timed so a brief
  loop stall can't glitch ESC output. (Heeds the RA4M1-shares-control-core
  caveat from the 2026-06-01 analysis — kept light, ~5 Hz client poll.)
- **Dashboard:** auto-targets `/data` when served by the board, else
  `http://192.168.4.1/data` when opened as a local file; 5 Hz poll, NO-LINK
  staleness watchdog, dims a stale ESC panel. RPM sent as electrical Hz ×30.
- Flash 26% / RAM 28% (WiFiS3 included). `wifiDebug()` prints a 3 s status
  comment line over USB during bring-up (remove once stable).
- **Open:** page is opened as a local file for now (laptop). To browse straight
  to 192.168.4.1 from a phone we'd embed the page in firmware (PROGMEM) — next
  step if wanted.

## 2026-06-13 — Dashboard served from board + SSE streaming (perf, #45)

Field-tested on iPhone (browse to 192.168.4.1). Two fixes:

- **Embedded the dashboard in firmware** (`web_page.h`, PROGMEM/flash, served at
  `/`). Any device — phone included — browses to 192.168.4.1; no local file.
  Page fetches same-origin so no CORS. Flash 36%, RAM unaffected (page in flash).
- **Update rate was ~1–2 Hz, not 5 Hz** — root cause: HTTP polling opened a NEW
  TCP connection every poll, and WiFiS3 per-connection setup is slow. **Switched
  to Server-Sent Events (SSE):** one persistent connection, server pushes a frame
  every 100 ms (10 Hz). `EventSource` also **auto-reconnects** after a dropout
  (helps the breadboard). Endpoint `GET /events` (text/event-stream); `/data`
  one-shot kept for debugging.
- **Faster X.BUS polling:** poll gap 80→10 ms, response timeout 12→6 ms, so a
  silent/flaky ESC barely stalls the healthy one (~30–40 Hz/ESC underlying).
- **Removed Google Fonts @import** — on the offline AP it stalled first paint
  (browser waiting on a fetch that can't succeed). Fixed the "loads messed up,
  then settles after 1–2 s" behavior; system-font fallbacks render instantly.

**Hardware observation (operator):** on the breadboard, one ESC's X.BUS telemetry
intermittently drops out under vibration (telemetry stops, then returns). Suspected
loose/unstable breadboard connection. Plan: solder the interface board (#43) to
stabilize. Not a firmware fault — control + the other ESC keep working through it.

### Dashboard responsiveness + direction (operator feedback)

- **Value lag fixed:** numbers (RPM/speed/current) eased down slowly after stick
  release because the dashboard applied its OWN EMA on top of the firmware's. The
  firmware already smooths V/I/temps and sends RPM raw, so the client EMA was pure
  lag — removed; values now render directly from the 10 Hz stream.
- **no-store on the page** so the iPhone always loads the latest dashboard build.
- **Direction indicators (open):** operator reports L/R arrows show the same
  direction during a pivot (counter-rotation). Dashboard already derives each
  arrow per-track from its own commanded output (outL→L, outR→R), which should
  differ in a pivot — to be diagnosed against live OutL/OutR + RPM-sign data
  before changing logic (is RPM signed on the GL10 in reverse?).

## 2026-06-14 — Interface board soldered + back together

- **Solder build complete.** Machine reassembled with the soldered interface
  board (replaces the breadboard). Standalone power confirmed: Left ESC BEC
  (7.4 V) → 9-pin cable yellow thick wire → Arduino barrel jack → buck →
  board powers up without USB.
- **S.BUS inverter circuit confirmed working** on the soldered board with
  R3 = **1 kΩ** (NOT the documented 10 kΩ; field-verified working). Math:
  drives Q1 base ~2.6 mA vs 0.26 mA at 10 k — well above saturation needs,
  storage-time slowdown at 100 kbaud S.BUS is negligible. 10 kΩ preferred
  on new builds (lighter receiver load); current 1 kΩ acceptable to keep.
- **X.BUS telemetry confirmed working** on the soldered board: both ESCs
  reporting (V0=11.6 V, V1=11.5 V; RPM tracking; temps 22–25 °C; OK0=OK1=1).
- **2N3904 orientation confirmed** for this build: flat side **facing AWAY**
  from operator → from operator's view, legs L→R are **C / B / E**.
- **Joystick cable rewiring:** Y axis cable flipped on board side to correct
  forward/backward polarity (was reading ~5100 instead of ~8192 at rest;
  re-orientation restored ~8192 center). After Y fix, X (steering) was
  inverted.
- **JOY_STEER_DIR = -1.0f** added to `rc_test.ino` `[CONFIG]` and applied
  to joystick X (steering only — RC steering unchanged). Right stick now
  turns right. Field-verified by operator.

## 2026-06-15 — Wi-Fi self-drive root cause + telemetry-dropout discriminator

- **Committed flight firmware (`rc_test` V7.8, PR #50) has NO Wi-Fi→motor
  path.** Code trace: the only writes to the ESCs (`outputWrite` →
  `escL/escR.writeMicroseconds`) are fed solely by S.BUS (RC) and the
  joystick ADC. `wifiUpdate()` serves `/`, `/data`, `/events` (SSE) only and
  never touches motor output; the dashboard pages contain no command/POST
  code. Failsafe holds neutral (SVC) when S.BUS is invalid.
- **Reported "digger moves by itself when the dashboard opens" came from an
  uncommitted local build**, not in git and not in FIRMWARE-UPLOAD-LOG (last
  logged flash = 0875a87, monitoring-only). Remediation: reflash the committed
  monitoring-only firmware from PR #50 (check out the branch first — the
  dangerous build may still be in the working copy).
- **Telemetry-dropout discriminator:** when control-derived fields
  (throttle/dir/gear/mode) keep updating in the dashboard while only
  RPM/temp/V/I go stale (panels dim to 0.35), the SSE/Wi-Fi link is healthy
  and the **X.BUS half-duplex bus (D0/D1) is dropping** — not Wi-Fi. A Wi-Fi
  loss would freeze the whole stream and trip the OFFLINE banner. Confirms the
  #43 breadboard-vibration X.BUS dropout; fix is the interface-board solder,
  not firmware.
- **DECISION: Wi-Fi telemetry stays monitoring-only, permanently.** No path
  from Wi-Fi to motor output. PR #50 to be hardened (single-source page, debug
  cruft removed) after it merges to main.

## 2026-06-18 — PR #50 lint/safety pass (zero-risk only; driving untouched)

Cleared the 3 failing CI lint checks + 2 review-bot findings WITHOUT changing
any tested driving behavior. Both sketches recompile clean (rc_test 39% flash /
29% RAM; telem_check 20%).

- **`rc_test.ino` (flight) — only 2 edits, both behavior-neutral:**
  (1) 3× `(const uint8_t *)` → `reinterpret_cast<const uint8_t *>` in the Wi-Fi
  `client.write`/`sseClient.write` calls (cppcheck cstyleCast; identical machine
  code). (2) `snprintf` return clamped in `buildTelemJson()` so callers never
  read past the 360 B buffer (CodeRabbit "critical"; a no-op unless a frame ever
  overflows — it currently does not). **No change to control loop, mixing, expo,
  gear caps, S.BUS, joystick, servo PWM, X.BUS polling, or telemetry values.**
- **`telem_check.ino` (bench tool, never flashed to the machine):** fixed
  garbage ESC-temp display (2-byte → 1-byte `d[14]`, per 2026-06-11 finding) +
  cpplint whitespace reformatting. Display/format only.
- **`INTERFACE-BOARD-PERFBOARD.md`:** blank lines for markdownlint.
- **DEFERRED (NOT changed), with reasons:** (a) X.BUS checksum validation —
  protocol checksum is officially ambiguous and the 0x10-response checksum was
  never hardware-verified; hard rejection could silently kill working telemetry,
  so verify during #54 hardware bring-up first. (b) Dashboard `s==0`-forward
  arrow — needs hardware diagnosis (2026-06-13). (c) `wifiUpdate()` blocking /
  SSE coalescing — that is issue #54, gated on hardware inspection.
- **Re-test:** flight edits are behavior-neutral, so driving cannot change; a
  quick re-flash smoke check is cheap insurance but not required for control.

## 2026-06-19 — Doc alignment to current hardware (issue #52)

Confirmed canonical board = **UNO R4 WiFi** (migration complete); X.BUS telemetry
and the Wi-Fi dashboard are LIVE (V7.8), not deferred — corrects #52's table.

- Rewrote `PROJECT-PLAN.md` and `CLAUDE.md` to GL10/GL540L + UNO R4 WiFi +
  **open-loop PWM control** (Arduino outputs servo PWM 1000–2000 µs on D9/D10;
  GL10 internal FOC owns smoothing; no Arduino-side PID/feedforward/inertia) +
  live X.BUS 0x10 telemetry (monitoring-only) + V7.8. FQBN `unor4wifi`, port COM7.
- Deleted superseded retired-hardware files: `docs/WIRING-GUIDE.md` (old
  E10/E3665), `docs/PLANT-CHARACTERIZATION.md`, `docs/XBUS-INVESTIGATION.md`, and
  sketches `xbus_master` / `xbus_probe` / `xbus_inverted_test` / `xbus_poll_test`
  (also removed from the arduino-ci matrix). `WIRING-GUIDE-V8.md` is the canonical
  wiring reference.
- Swept E10/E3665/CS7581/UNO-Q tokens out of all current files (acceptance grep
  clean); remaining "Nano R4" mentions are historical/comparative only.
- `XBUS-PROTOCOL.md` kept as a CURRENT reference (live telemetry uses func 0x10).
- All sketches compile on `unor4wifi`; markdownlint clean on changed docs.

## 2026-06-20 — Battery + inactivity beeper (V7.12, PR #71 / #51)

New `[ALERT]` module on the D8 piezo (audio only — no motor-path change; the
low-voltage motor cutoff is split to PR #2 / #65 by risk).

- **Inactivity alarm:** RC transmitter off (`sbusValid==false`) > 60 s → one long
  beep / 2 s (`500/1500`, non-latching). `sbusValid` already encodes RC-off
  (S.BUS failsafe or frame-loss timeout). Purpose: unplug LiPos (parasitic ~2 W
  draw deep-discharges packs over 1–2 weeks).
- **Low-voltage alarm:** worst-of-two pack < **10.5 V** → three fast chirps /
  ~1.2 s (`120/120 ×3, 600`), validity-gated (both ESCs plausible 6–13 V; a
  not-yet-powered ~0 V pack can't false-alarm), 3 s sag debounce, 60 s startup
  grace, **latched until power cycle**. Priority: low-V > inactivity.
- **10.5 V chosen** (≈3.5 V/cell, ~30%) deliberately conservative: large/expensive
  15 Ah packs, long runtime, and the operator's balance charger false-declares a
  pack "dead" if it arrives too low. Stop early, stay healthy.
- **Bench test PASS (2026-06-20):** flashed a TEST build at `LOWV_THRESH_V=12.0`
  (packs resting ~12.2/12.4). Confirmed the low-V alarm beeps when the worse pack
  reached ~12 V, and latches. Production value restored to **10.5 V** (same code
  path, constant only) and re-flashed (SHA 8a33a9b). Note: clearing the latch
  power-cycles the board, which drops USB/COM7 — replug to re-upload.
- **Open finding — voltage signal is fast, not a battery average:** the alarm
  reads `telem[].voltage`, whose EMA is `TELEM_A_VOLT=0.30` at ~30 Hz/ESC ≈ 0.3 s
  smoothing — it sags under load. The 3 s debounce kills short sag spikes but not
  a sustained hard pull, so 10.5 V could false-latch under load in the field.
  Planned refinement before trusting 10.5 V live: a dedicated ~30 s EMA (per #40)
  or much longer debounce for the alarm signal.
- Beep vocabulary kept distinct (count the beeps): 2-short=Wi-Fi ready,
  steady=horn, 1-long/2 s=inactivity, 3-fast/latched=low battery. Documented in
  OPERATOR-GUIDE.md "Beep meanings" table (#70).
- Follow-ups opened: #72 (smooth pivot→straight transition), #73 (dashboard
  visual alarm: red battery + error code, firmware-sourced).

## 2026-06-21 — P0 Wi-Fi runaway failsafe (V7.13, PR #74 / #69)

Root cause: serving the ~33 KB dashboard as one blocking burst froze loop()
~1–2 s while the hardware-timed Servo PWM held the last throttle → uncommanded
runaway under load. The in-loop S.BUS failsafe couldn't help (it was frozen too).

Fix (two coupled parts):

- **Hardware watchdog (RA4M1 WDT, 250 ms):** armed at end of setup() (after AP
  bring-up); WDT.refresh() called ONLY after a successful control update (inputs
  read + outputs written). Any loop stall > 250 ms resets the MCU → PWM stops →
  ESCs neutral. Accepted as last-resort emergency stop (full reboot; AP blips).
- **Incremental Wi-Fi serving:** wifiUpdate() does at most ONE modem write per
  loop pass (one 1 KB page chunk OR one request OR one SSE frame); body streams
  via a pageRemaining state machine; headers/304/data coalesced to single
  writes. Keeps each loop pass well under the WDT timeout so normal refresh never
  trips it.
- Robustness (Copilot review): snprintf returns guarded; 0-byte page write
  aborts the transfer instead of spinning forever (would starve SSE).

**Operator-confirmed (2026-06-21):** dashboard refresh no longer stalls/resets
the loop; the watchdog stands as the rare backstop only.

Permanent fix remains #55 (offload Wi-Fi/HTTP/SSE to the ESP32-S3 spare core),
which removes Wi-Fi from the control core entirely. WDT is the interim backstop.

## 2026-06-21 — Dashboard "connection lost" wedge: ESP32-S3 socket exhaustion

Symptom: after a few iPad Wi-Fi off→reconnect→refresh cycles, the dashboard
sticks on "connection lost, reconnecting" for minutes; only a power-cycle (or a
long wait) recovers.

Diagnosed via USB serial capture (`# WIFI ... clients_seq=` line, every 3 s)
while reproducing. Evidence: `clients_seq` (incremented per telemetry frame
built) ran 0 → 13 (one healthy SSE burst) → +1 per reconnect → **froze at 15**
for 35 s+. Throughout: `status=8` (AP up), control CSV never stopped, **no
reboot** (WDT never fired). So only the HTTP/SSE serving layer wedges — driving
is unaffected.

Root cause (confirmed against WiFiS3 source): TCP sockets live on the ESP32-S3
(~5 link IDs). When the iPad drops Wi-Fi abruptly there's no clean close → the
held `sseClient` socket becomes a half-open zombie. The firmware never checks the
SSE `write()` return, so it keeps the dead socket. `WiFiServer::available()` only
surfaces NEW connections — it cannot reap an established zombie — so once the pool
fills, new `/events` reconnects are never accepted → permanent wedge until the
ESP's own TCP timeout (minutes). `AT+CIPSTO` (idle timeout) is not exposed by
WiFiS3, so we can't tune it.

Decision (fix in a separate PR, not the merged dashboard PR #76): reap the SSE
socket proactively — on a 0-byte `sseClient.write()` (dead socket) call
`sseClient.stop()` (= AT+CIPCLOSE) to free the link ID immediately, so the next
reconnect can be accepted. Add a stale-SSE guard (no successful write for ~3 s →
stop) and, as a fallback only if testing still shows wedges, a server/AP recycle.
Permanent cure remains #55 (offload Wi-Fi to the ESP32-S3). Diagnostic method
(serial `clients_seq` watch) is reusable for verification.

## 2026-07-05 — Adopt domain-oriented target architecture (epic #116, ADR-0001)

Decided: migrate the production firmware to a layered, domain-oriented
structure under `sketches/dual_track_control/src/` with these layers:
`application/`, `domain/` (drive, operator_input, battery, thermal, safety),
`ports/`, `infrastructure/` (arduino, radiolink, xc, network), `telemetry/`,
`alerts/`, `config/`, `generated/`. Full spec: `docs/architecture/ARCHITECTURE-TARGET.md`.
Key rules: pure domain (no Arduino includes, no mutable namespace-scope state,
time as parameter), one owner per state, ports with link-time substitution
(virtuals only for genuine runtime swapping), observers read an immutable
per-cycle SystemSnapshot. File policy ≤150 lines soft / 250 hard, generic
names (Utils/Helpers/Manager/…) banned. CI fitness functions (#129) will
enforce. Rationale + rejected alternatives: ADR-0001.

Compile feasibility PROVEN before adoption: spike sketch with nested
`src/{application,domain,ports,infrastructure}` compiled clean on
`arduino:renesas_uno:unor4wifi` (arduino-cli 1.4.1), domain file with zero
Arduino includes. Arduino sketch spec compiles `src/` recursively.

## 2026-07-05 — Production sketch to be renamed rc_test → dual_track_control (#118)

Operator-chosen name; reflects that the firmware is a generic dual-track/tank
drive controller (no hydraulics/attachment control). Directory stays
lowercase_snake_case per structure-check CI; PascalCase reserved for C++ type
names inside src/.

## 2026-07-05 — Refactor verification strategy while hardware is unavailable

No machine access for the duration of the #116 remediation: no flashing, no
bench tests. Every refactor PR must (a) compile locally via arduino-cli before
push, (b) preserve behavior via characterization/host tests (#47), (c) change
no control-path behavior. Physical checks accumulate in
`docs/architecture/BENCH-VERIFICATION-DEFERRED.md`; one bench pass runs the
whole checklist when hardware returns, then is logged in
`docs/FIRMWARE-UPLOAD-LOG.md`.

## 2026-07-05 — Characterization harness compiles the real .ino with stub headers (#47)

Phase B safety net built as a host test suite (`tests/`): stub
Arduino/Servo/WiFiS3/WDT/sbus headers shadow the real ones via include order,
and each doctest binary `#include`s the unmodified `rc_test.ino` — zero
firmware changes, so V7.34 behavior is captured exactly as flashed. 9 suites /
59 cases / 751 assertions lock in curvature mixing (#72/#86/#96/#114), input
shaping, gear/reverse caps (#113), max/oppose mixer (#90), battery ladder
(#65), thermal stages (#111), output gate (#88), X.BUS parsing, and the
end-to-end loop (incl. exactly one WDT refresh per pass). Framework: doctest
2.4.12 vendored at `tests/vendor/` (lint-exempt). Verified the suite can fail
(mutation check). Test expectations are behavior law: changing one requires an
issue + operator sign-off.

Firmware behavior facts confirmed on the host while characterizing (all
consistent with field behavior, none changed): `telemTryParse` does not verify
the response checksum (header/address/function/length only); integer servo
conversion truncates toward zero (±235.625 µs → 1735/1265 symmetric);
boot-gate fail-open verified at exactly BATTERY_CONFIRM_MS with silent
X.BUS.

## 2026-07-05 — Commit-time test gate: hard block at three layers (#47)

Per the 2026-06-01 decision (hard block, no soft warnings): (1)
`.githooks/pre-commit` runs the host suite whenever firmware/tests/workflow
files are staged + blocks firmware-without-tests commits
(`DIGGER_NO_TEST_CHANGE=1` escape for test-neutral edits; `--no-verify` is the
human emergency bypass); (2) `.claude/hooks/test-gate.sh` (PreToolUse[Bash])
makes it agent-proof — refuses `--no-verify` and unactivated `core.hooksPath`;
(3) CI `unit-tests` job (ubuntu, `make -C tests run`) as required status
check. Local runner on the Windows dev box is WSL Ubuntu g++ 11.4 (no native
toolchain). tests/vendor excluded from cpplint/lizard; tests/ excluded from
cppcheck (doctest macros defeat its parser — the unit-tests job gates that
tree).
