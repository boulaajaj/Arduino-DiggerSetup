# Excavator Track Controller — Project Memory

## What This Is

Arduino **UNO R4 WiFi**-based tank-style track controller for a ride-on
excavator. Two FOC brushless motors drive rubber tracks via FOC ESCs. Dual
input: RC transmitter (Jason) and joystick (Malaki/rider). 3-position override
switch selects who has authority. X.BUS telemetry is streamed to a Wi-Fi
dashboard (monitoring only).

## Working Agreement (#53 — the Karpathy method, adapted)

Behavioral guardrails for every agent working in this repo. The
**behavior-preservation covenant** (`.claude/rules/behavior-preservation.md`)
sits above all four: during epic #116, organization changes NOTHING observable
— firmware flashed after a refactor must drive exactly as before.

1. **Think before coding — never guess on safety-relevant ambiguity.**
   Ambiguity on pin assignments, ESC parameters, safety caps/thresholds, or
   timing → stop and ask. Before any XC-Link / ESC parameter suggestion, run
   the **`safety-review` skill** — its command-path checklist IS this
   principle for our domain. State assumptions explicitly; push back on
   needless complexity instead of silently building it.
2. **Simplicity first.** Senior-engineer default: no speculative features, no
   abstraction for one caller, 100 lines over 1000. The file policy (150
   soft / 250 hard, `.claude/rules/architecture.md`) is the mechanical
   backstop, not the goal.
3. **Surgical changes.** Touch only what the task needs: no reformatting, no
   renaming, no drive-by "improvements" to code the diff didn't have to
   visit (naming fixes only when a file is extracted/moved, per
   `.claude/rules/naming.md`). A reviewer should read the diff and see
   exactly one intent.
4. **Goal-driven execution — hand the agent the goal + the gate, not steps.**
   The standing success criteria for any firmware-touching change:
   - host suite green: `wsl -e make -C tests run` (characterization +
     invariants — compiles the REAL `dual_track_control.ino`)
   - compiles clean locally:
     `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/dual_track_control`
   - CI green: `arduino-ci`, `lint`, `static-analysis`, `code-quality`,
     `structure-check`, `unit-tests`, `architecture-fitness` (#129),
     `wiki-lint` (#145), `hooks-selftest` (#156/#193)
   - no blocking calls in `loop()` (`delay()`, `pulseIn()`, unbounded `while`)
   - zero behavior change unless the task's own issue explicitly authorizes it

   An agent that satisfies the gate is done; one that can't must say why —
   worked examples: `docs/AGENT-EXAMPLES.md`.

## Workflow — Board, PRs, Flashes

- [Project #1](https://github.com/users/boulaajaj/projects/1) is the **single
  source of truth**: anything being worked on shows as **In Progress** there.
- **Issue first, always; starting work = draft PR** with `Closes #N` — one
  PR ↔ one issue, one open PR at a time. Never leave active work as a bare
  branch (invisible to the board). Board automation moves linked PRs to
  In Review and merged work to Done; setting an issue In Progress is manual.
- The full lifecycle (branch naming, review-round protocol, autonomous merge
  protocol) is the **`prepare-pull-request` skill**; standing rules:
  `.claude/rules/workflow.md`.
- **Every firmware flash gets a row in `docs/FIRMWARE-UPLOAD-LOG.md`
  immediately** — an unlogged flash is treated as not done. Procedure:
  **`flash-and-log` skill**.

## People

**Jason** — RC operator (safety supervisor) · **Malaki** — rider on the
joystick · **boulaajaj** — GitHub owner / builder

## Hardware Stack

| Component | Model |
| --- | --- |
| Controller | Arduino UNO R4 WiFi (RA4M1 48MHz, 14-bit ADC, 5V tolerant; ESP32-S3 Wi-Fi coprocessor) |
| ESC ×2 | XC GL10 80A FOC (50 Hz servo PWM in, internal FOC) |
| Motor ×2 | XC GL540L sensored brushless |
| Battery ×2 | OVONIC 3S LiPo 11.1V 15000mAh (EC5) |
| RC | Radiolink RC6GS V3 + R7FG (gun-style; Arduino does the tank mixing) |
| Joystick | Genie 101174GT dual-axis hall-effect, 0–5V analog |

## Control Strategy (PWM out, FOC inside the ESC)

- Arduino: read RC + joystick → expo/deadband → curvatureDrive mix →
  override → gear cap → servo PWM (1000–2000 µs) per track on D9/D10.
  The GL10's internal FOC owns command smoothing (no Arduino-side filtering
  — V7.2 removed it; smoothness is the top priority, `docs/MISSION.md`).
- **Open-loop, permanent**: no RPM feedback, no PID.
- **Telemetry is monitoring-only, permanent**: X.BUS func 0x10 (read
  register — never 0x50, which would seize control authority) polled
  read-only on Serial1; no path from Wi-Fi or telemetry to motor output.
- **Never `SoftwareSerial` for telemetry** — hardware UARTs only.

## Pins, Wiring, Protocols

Canonical reference: `docs/WIRING-GUIDE-V8.md` (pin map, UART table, S.BUS
inverter, interface board). Quick summary: A0/A1 joystick (14-bit ADC),
D12 S.BUS RX via NPN inverter (`sbusUart` on SCI0, D11 TX unused), D9/D10
track PWM, D0/D1 Serial1 X.BUS telemetry to both ESCs, D8 active piezo,
USB-C debug/upload. X.BUS frames, registers, and scaling:
`docs/XBUS-PROTOCOL.md`.

## Architecture Summary

`sketches/dual_track_control/dual_track_control.ino` is a composition root
ONLY (#189) — it includes `src/application/FirmwareApp.h` and delegates
`setup()`/`loop()`. **Every `src/...` path in this file is relative to
`sketches/dual_track_control/`.** Layers under `src/` (rules:
`.claude/rules/architecture.md`; spec:
`docs/architecture/ARCHITECTURE-TARGET.md`):

- `domain/` pure logic (battery, thermal, operator_input, drive, safety)
- `application/` orchestration + state (the former `[MODULE]` sections;
  banners moved with the code — search `[NAME]` to jump)
- `ports/` hardware contracts · `infrastructure/` the ONLY layer with
  hardware includes · `telemetry/` + `alerts/` observers and alert policy
- `config/` tunables per domain — values are law; `FIRMWARE_VERSION` SSOT
  in `src/config/BuildInfo.h` (#124); no doc carries a live copy

Signal pipeline: S.BUS (16ch) + joystick ADC → deadband/expo →
curvatureDrive (symmetric add + desaturate, smoothstep pivot blend, pivot
cap, throttle taper #114) → override mode (CH5: RC only / RC overrides /
50-50) → gear cap (CH4 Eco/Normal/Boost — caps AVERAGE track speed, turn
headroom by design; per-gear reverse cap #113) → servo PWM out. Loop
~20 kHz non-blocking, `micros()`-based; per-input-path failsafe (S.BUS is
ONE path — one frame-freshness timeout covers all 16 channels).

Full annotated inventory: **`docs/architecture/FILE-MAP.md`**.

## Dashboard & Alerts

- Wi-Fi AP `Digger-Telemetry` at `192.168.4.1`, SSE ~5 Hz — **monitoring
  only, permanent**. Source of truth `dashboard/index.html`; `web_page.h` is
  GENERATED (`scripts/generate_web_page.py`, CI-enforced #120) — never
  hand-edit.
- Active piezo on D8 (`src/alerts/`): ready beep, horn, inactivity alarm,
  latched low-battery chirps — sound-only. Motor-affecting protection is the
  separate `[SAFETY]` staged ladder (#65). Beep table: `OPERATOR-GUIDE.md`;
  thresholds: `src/config/`.

## Key Design Decisions

GL10 FOC replaced the custom PID stack (2026-04-25); UNO R4 WiFi replaced
Nano R4 (onboard Wi-Fi); open-loop and monitoring-only telemetry are
permanent scope decisions. Rationale + dated history (hardware swaps, beeper
lineage): `docs/DECISION-LOG.md`.

## Coding Rules

Path-scoped rules in `.claude/rules/` and the architecture spec WIN over any
summary; which artifact is authoritative for what is stated once in
[docs/wiki/authority-matrix.md](docs/wiki/authority-matrix.md) (#198).
Quick reference:

- **Architecture**: production sketch only — layers per the table above;
  one file = one concept (150/250); structs shared with the `.ino` in
  `types.h` (interim bridge); tunables in `src/config/` (adapter-owned stay
  single-homed in `src/infrastructure/`)
- **Testing (#47, `docs/TESTING.md`)**: host suite is the behavior-parity
  gate; commits hard-blocked by `.githooks/pre-commit` (+ PreToolUse
  test-gate, #193); a changed test EXPECTATION is a behavior change — own
  issue + operator sign-off; new mutable global ⇒ `resetFirmwareState()`
- **Real-time**: no blocking calls in `loop()`; `micros()` fresh at point of
  use; ISRs < 10 µs; `constrain()` at every servo boundary
- **Style**: `float` with `f` suffix (`double` is software-emulated);
  commit messages `V{major}.{minor}: {imperative} {what}`; README describes
  behavior, never tunable numbers (#124)

### ESC / motor configuration changes

Every ESC parameter (XC-Link Bluetooth app, X.BUS register, programming
card) must be evaluated against **what PWM commands the Arduino code
actually sends** before recommending a value — per-direction caps,
asymmetric scalings, and brake limits can silently clip legitimate commands
and break features (e.g. the GL10 default `Max Reverse Force = 50%`
collapses the pivot differential). The full command-path checklist is owned
by the **`safety-review` skill** (`.claude/skills/safety-review/SKILL.md`) —
run it before ANY parameter suggestion. Per-parameter code-context analysis:
`docs/GL10-PARAMETERS.md`.

## Skills & Reviewer Agents (#127)

Repeatable procedures are packaged as project skills in `.claude/skills/`
(each owns its procedure — this file only points):

- **safety-review** — command-path checklist + invariants walk before any
  control-path change or ESC parameter suggestion
- **flash-and-log** — upload procedure + the mandatory FIRMWARE-UPLOAD-LOG row
- **new-module** — layer placement + conventions for new firmware files
- **field-tune** — drive-feel constant changes (covenant-compliant)
- **wiki-impact-review** — diff → affected docs/wiki walk + receipt
- **prepare-pull-request** — issue-first lifecycle: draft PR, review rounds,
  autonomous merge protocol

Read-only reviewer subagents live in `.claude/agents/`:
`architecture-reviewer`, `safety-reviewer`, `tests-reviewer`,
`documentation-reviewer` — run the relevant ones before marking a PR ready.

## Build & Upload

- First checkout: `cp sketches/dual_track_control/arduino_secrets.h.example
  sketches/dual_track_control/arduino_secrets.h` and fill in the real Wi-Fi
  credentials (gitignored, #125). CI copies the template automatically.
- Board: Arduino UNO R4 WiFi · FQBN `arduino:renesas_uno:unor4wifi` ·
  Port COM7 · Serial 115200 baud
- Upload: `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/dual_track_control && arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi -p COM7 sketches/dual_track_control`
  (then log it — `flash-and-log` skill)
- VS Code: Ctrl+Shift+B → "Live Plot" for real-time monitoring
