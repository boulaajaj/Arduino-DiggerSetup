# Target Architecture — Dual Track Control

**Status:** adopted 2026-07-05 (epic #116, issue #128, ADR-0001)
**Applies to:** the production firmware, migrating from `sketches/rc_test/` to
`sketches/dual_track_control/` (rename tracked in #118)

This document is the contract for the Phase D refactor (#117, #130, #132) and
the rulebook the CI architecture fitness functions (#129) enforce. Bench
sketches (`sketches/*_test/`, `hw_diagnostic`, …) are exempt — they are
throwaway bring-up tools by design.

---

## 1. Why this architecture

The firmware has two jobs: drive a real machine safely, and demonstrate how a
growing, safety-aware, multi-domain embedded system is engineered. The current
single 1606-line `.ino` works, but its `[MODULE]` comment sections are
navigation, not boundaries — nothing stops a Wi-Fi function from reading a
safety latch directly, and none of the control math can run off-target.

The target: **domain logic that could run on a laptop, hardware access behind
explicit ports, one owner per piece of state, and a composition root that reads
like the signal-flow diagram.**

## 2. Target tree

```text
sketches/dual_track_control/
├── dual_track_control.ino        ← composition root ONLY (~20-30 lines)
└── src/
    ├── application/              ← orchestration: wires domains + ports, owns the cycle
    │   ├── FirmwareApp.h/.cpp    ← setup()/tick() entry, owns module instances
    │   ├── ControlCycle.h/.cpp   ← one pass: read → decide → gate → write → observe
    │   └── SystemSnapshot.h      ← immutable per-cycle state for observers (#132)
    │
    ├── domain/                   ← pure logic. NO Arduino/hardware includes. Host-compilable.
    │   ├── drive/
    │   │   ├── DriveTypes.h              (DriveIntent, TrackCommand, WheelSpeeds)
    │   │   ├── CurvatureDrive.h/.cpp     (curvature mix + desaturation)
    │   │   ├── PivotPolicy.h/.cpp        (pivot cap, blend band, throttle taper)
    │   │   ├── GearPolicy.h/.cpp         (gear scale, per-gear caps, reverse cap)
    │   │   └── CommandMixer.h/.cpp       (max/oppose RC+joystick combine)
    │   ├── operator_input/
    │   │   ├── InputTypes.h              (OperatorInput, normalized axes)
    │   │   ├── DeadbandPolicy.h/.cpp
    │   │   ├── ExpoCurve.h/.cpp
    │   │   └── InputNormalization.h/.cpp (raw→[-1,1], gains, polarity)
    │   ├── battery/
    │   │   ├── BatteryTypes.h            (BatteryReading, BatteryStatus)
    │   │   ├── VoltagePlausibility.h/.cpp(validity + worst-of-two — ONE implementation)
    │   │   └── BatteryLadder.h/.cpp      (Eco lock, low-V alarm level, cutoff, boot gate)
    │   ├── thermal/
    │   │   ├── ThermalTypes.h            (ThermalReading, ThermalStage)
    │   │   ├── ThermalHysteresis.h/.cpp  (one stage: on/off thresholds + trip debounce)
    │   │   └── ThermalDerating.h/.cpp    (warn / eco / cut staging off hottest sensor)
    │   └── safety/
    │       ├── SafetyTypes.h             (SafetyInputs, SafetyDecision, FailsafeReason)
    │       ├── SafetySupervisor.h/.cpp   (battery + thermal + RC freshness → decision)
    │       └── OutputGate.h/.cpp         (ACTIVE/HOLD/CUT state machine, ease-out ramp)
    │
    ├── ports/                    ← tiny headers; hardware contracts consumed by
    │                               application/ — they speak domain types, but
    │                               domain/ itself never includes them
    │   ├── ClockPort.h           (now µs/ms)
    │   ├── RcInputPort.h         (channels + freshness)
    │   ├── JoystickPort.h        (raw ADC pair)
    │   ├── EscOutputPort.h       (µs per track; attach/detach)
    │   ├── TelemetrySource.h     (EscTelem per ESC)
    │   ├── TelemetrySink.h       (SystemSnapshot out)
    │   └── AlertOutputPort.h     (piezo on/off)
    │
    ├── infrastructure/           ← the ONLY layer that includes hardware libraries
    │   ├── arduino/              (ArduinoClock, PwmEscAdapter, PiezoAdapter, AdcJoystickAdapter, WatchdogAdapter)
    │   ├── radiolink/            (SbusReceiverAdapter — sbus lib + inverted-UART detail)
    │   ├── xc/                   (XbusTelemetryAdapter — 0x10 poller, framing, checksum)
    │   └── network/              (WifiService, DashboardServer, SseStream — non-blocking rules live here)
    │
    ├── telemetry/                ← observers: encode SystemSnapshot for sinks
    │   ├── TelemetryScaling.h    (×10 integer scaling — ONE implementation)
    │   ├── JsonEncoder.h/.cpp    (SSE //data payload)
    │   └── CsvEncoder.h/.cpp     (USB debug stream)
    │
    ├── alerts/                   ← beep vocabulary + priority (domain-ish, no hardware)
    │   ├── AlertTypes.h          (AlertView, patterns)
    │   ├── AlertPolicy.h/.cpp    (priority: thermal-cut > low-V > thermal-warn > inactivity)
    │   └── PatternPlayer.h/.cpp  (non-blocking on/off sequencing)
    │
    ├── config/                   ← ALL tunables, grouped by domain (replaces [CONFIG])
    │   ├── DriveConfig.h  SafetyConfig.h  BatteryConfig.h  ThermalConfig.h
    │   ├── InputConfig.h  TelemetryConfig.h  WifiConfig.h  AlertConfig.h
    │   ├── Pins.h                (every pin assignment in one file)
    │   └── BuildInfo.h           (FIRMWARE_VERSION — single source of truth, #124)
    │
    └── generated/                ← machine-written only; never hand-edited (#120)
        └── web_page.h
```

`tests/` (host-side, #47/#131) lives at repo root, mirroring `domain/`
subfolders plus `characterization/` and `invariants/`.

## 3. Layers and dependency rules

| Layer | May depend on | Must NOT depend on |
| --- | --- | --- |
| `domain/` | other `domain/` headers, `config/` | `Arduino.h`, `WiFiS3.h`, `Servo.h`, SBUS lib, X.BUS/GL10 code, `ports/`, `infrastructure/`, `application/` |
| `ports/` | plain types from `domain/` | any hardware library |
| `application/` (incl. its observer submodules `telemetry/`, `alerts/`) | `domain/`, `ports/`, `config/` | `infrastructure/` internals (talks through ports) |
| `infrastructure/` | `ports/`, hardware libraries, `config/` | `domain/` internals (plain types via ports only), `application/` |
| `telemetry/`, `alerts/` (observer ring of the application layer) | `application/SystemSnapshot.h` ONLY from application/, plus `domain/` types, `config/` | hardware libraries, any other `application/` header, domain *internals* (mutable state) |
| `generated/` | — | hand edits |
| `.ino` | `application/` only | everything else |

`telemetry/` and `alerts/` are part of the **application layer** (its observer
ring), not a separate layer — so application orchestration calling them is
within-layer, not a cross-layer cycle. At file level the graph stays acyclic:
`SystemSnapshot.h` includes nothing from `telemetry/` or `alerts/`, and those
folders may include no `application/` header except `SystemSnapshot.h`. The
fitness checks (#129) enforce exactly that file-level rule.

Three non-negotiables, enforced by #129:

1. **`domain/` compiles on the host** with no Arduino includes.
2. **No namespace-scope mutable state in `domain/`.** Modules own state via
   structs passed explicitly (or objects held by `application/`).
3. **Time is a parameter.** Domain code never calls `millis()`/`micros()`;
   `ControlCycle` reads `ClockPort` once per pass and passes `now` down.

## 4. Control flow (one cycle)

```text
   SbusReceiverAdapter   AdcJoystickAdapter        XbusTelemetryAdapter
        │ RcInputPort         │ JoystickPort             │ TelemetrySource
        ▼                     ▼                          ▼
   ┌────────────────────────────────┐            ┌───────────────────┐
   │ operator_input:                │            │ battery:          │
   │ normalize → deadband → expo →  │            │ plausibility →    │
   │ gains → OperatorInput          │            │ ladder            │
   └───────────────┬────────────────┘            │ thermal:          │
                   ▼                             │ hysteresis stages │
   ┌────────────────────────────────┐            └─────────┬─────────┘
   │ drive: CommandMixer →          │                      │
   │ GearPolicy → PivotPolicy →     │                      ▼
   │ CurvatureDrive → TrackCommand  │            ┌───────────────────┐
   └───────────────┬────────────────┘            │ safety:           │
                   │       proposed command      │ SafetySupervisor  │
                   └──────────────┬──────────────┤ → SafetyDecision  │
                                  ▼              └───────────────────┘
                        ┌───────────────────┐
                        │ safety:OutputGate │  ACTIVE / HOLD(ease) / CUT
                        └─────────┬─────────┘
                                  ▼ approved µs
                        ┌───────────────────┐
                        │ EscOutputPort →   │
                        │ PwmEscAdapter     │
                        └───────────────────┘

   ControlCycle then builds ONE immutable SystemSnapshot and hands it read-only to:
      telemetry/JsonEncoder → SSE/dashboard      (monitoring only — permanent rule)
      telemetry/CsvEncoder  → USB debug
      alerts/AlertPolicy    → PatternPlayer → AlertOutputPort (piezo)
```

The permanent safety rules are unchanged by this refactor: **open-loop control**
(telemetry never feeds throttle), **no control input over Wi-Fi, ever**, and the
watchdog refresh stays exactly once per cycle, after the control path.

## 5. State ownership

**One domain owns its state; no other domain writes it.** The concrete sins
this outlaws (all present in `rc_test.ino` today):

- `[SAFETY]` writing `[ALERT]`'s `lowVoltLatched` → instead `SafetyDecision`
  carries `alarmRequested`; `AlertPolicy` reads it and owns its own latch.
- `curvatureDrive()` silently reading the `currentGear` global → gear is a
  field of the input it receives.
- Wi-Fi/debug scraping ~30 globals → observers read `SystemSnapshot` only.

`SystemSnapshot` (#132) is built once per cycle by `application/` and passed
`const&` to every observer. Observers cannot write, and cannot see two
different cycles in one frame.

## 6. File and naming policy

- **Soft limit 150 lines** per human-written source file (one screenful);
  **hard CI fail at 250**; exemptions in `.architecture-allowlist` with a
  written reason (generated files, lookup tables).
- One file = one concept. Splitting `Foo.cpp` into `FooPart2.cpp` to duck the
  limit is a review reject — split by *responsibility* or don't split.
- Sketch/dir names `lowercase_snake_case` (structure-check CI). C++ types
  `PascalCase`, functions `camelCase`, constants `UPPER_SNAKE_CASE`, pins
  `PIN_*` (unchanged from today's rules).
- **Banned file names:** `Utils`, `Helpers`, `Helper`, `Manager`, `Common`,
  `Misc`, `Stuff`, `Data`. A `helpers/` *folder* inside a domain is allowed
  only when every file in it names one precise responsibility
  (e.g. `drive/helpers/Smoothstep.h`) — never a grab-bag file.
- Domain language over mechanism language: `OutputGate`, not `PwmManager`;
  `FailsafeReason`, not `ErrorCode`.

## 7. Ports & adapters strategy

Ports are **small headers** owned by the consuming side; adapters implement
them in `infrastructure/`. Preference order:

1. **Link-time substitution** (default): the firmware build links the Arduino
   adapter, the host test build links a fake. Zero runtime cost, no vtables.
2. **Virtual interfaces** only where implementations genuinely swap at runtime
   (none identified today).

No Arduino type may appear in a port signature — ports speak `domain/` types
and primitives.

## 8. Ubiquitous language

| Term | Meaning |
| --- | --- |
| `OperatorInput` | Normalized post-shaping axis pair from one input source |
| `DriveIntent` | Mixed, gear-capped command before safety review |
| `TrackCommand` | Per-track normalized output (−1..+1) |
| `ServoOutput` | Per-track µs (1000–2000) |
| `SafetyDecision` | Supervisor verdict: allow / ease-to-stop / cut + `FailsafeReason` |
| `FailsafeReason` | Why drive is denied: RC_STALE, BATTERY_CUTOFF, THERMAL_CUT, BOOT_GATE |
| `BatteryLadder` | Staged response: Eco-lock 10.8 V → alarm 10.5 V → cutoff 10.0 V |
| `ThermalStage` | NORMAL / WARN (80°) / ECO (90°) / CUT (95°), hysteresis releases |
| `OutputGate` | ACTIVE / HOLD (ease to neutral) / CUT (no pulses) state machine |
| `SystemSnapshot` | Immutable per-cycle state for observers |
| `Gear` | Eco / Normal / Boost average-speed cap |

## 9. Migration plan (Phase D, #117)

Smallest, purest logic first — the pattern is proven before the propulsion
path moves:

1. `VoltagePlausibility`
2. `BatteryLadder`
3. `ThermalHysteresis` / `ThermalDerating`
4. `TelemetryScaling` + encoders
5. `operator_input/`
6. `CurvatureDrive`
7. `PivotPolicy` / `GearPolicy`
8. `SafetySupervisor`
9. `OutputGate`
10. `infrastructure/` adapters
11. `FirmwareApp` + composition-root `.ino`

**Rules for every migration PR** (operating constraints locked 2026-07-05):

- Behavior-preserving only; characterization tests (#47) green before/after.
- `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi` locally **before
  every push** — structure problems surface at step 1, not step 10.
- **No hardware available:** no flashing, no bench tests. Compile plus host
  tests plus review are the verification. Each PR appends any physical checks it would
  have run to `docs/architecture/BENCH-VERIFICATION-DEFERRED.md`; one bench
  pass executes that whole checklist when the machine returns.
- One extraction step per PR, own issue, draft PR from the first commit
  (board rules unchanged).

## 10. Enforcement

Rules in this document are enforced by the CI architecture fitness functions
(#129, shipped): forbidden includes, layer direction, file-size policy
(allowlist: `.architecture-allowlist`, reason required per entry), banned
names, no mutable domain globals, generated-file drift (activates with #120),
version SSOT (activates with #124). Implementation:
`scripts/check_architecture.py` + `scripts/architecture_rules.py`, run by the
`architecture-fitness` workflow after a self-test
(`scripts/check_architecture_selftest.py`) proves every failure mode fires.
The check is **advisory** (not in branch protection) until the Phase D
extraction lands, then flipped to required. Section 3's table is canonical —
a rule change here must change the script in the same PR.

**Migration window (#150):** the composition-root rule (`.ino` →
`application/` only) activates once `src/application/FirmwareApp.h` exists —
the step-11 marker. During steps 1–10 the production `.ino` is the interim
application shell and may include extracted layers directly (the checker
emits a NOTE instead); extracted `src/` files are fully checked from step 1.

## 11. Testing strategy (summary — details in #47/#131)

| Level | What | Where |
| --- | --- | --- |
| 1 Pure domain | curvature, expo, deadband, gear/reverse caps, plausibility | `tests/<domain>/` |
| 2 State transitions | ladder debounce/latch, thermal hysteresis, output gate, explicit `now` | `tests/<domain>/` |
| 3 Characterization | current firmware behavior captured BEFORE refactor | `tests/characterization/` |
| 4 Invariants + faults | outputs bounded, stale-RC ⇒ neutral, cutoff dominates, monotone derating | `tests/invariants/` (#131) |
| 5 Adapters | S.BUS decode, X.BUS framing/checksum, JSON/CSV encoding | `tests/` |
| 6 Bench (deferred) | physical behavior per BENCH-VERIFICATION-DEFERRED.md | hardware, when available |

Level 4 landed with #131; the invariant registry (verbatim list, constants,
test links) is `docs/SAFETY.md`.
