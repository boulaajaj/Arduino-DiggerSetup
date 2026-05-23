# GL10 ESC — Operational Procedures

**Source:** XC-ESC GL10 official user manual (`docs/GL10-Manual.pdf`,
3-page image-based PDF from <https://www.xc-esc.com/product/rc-brushless-sensored-foc-gl10-esc-80a2-4s/>).

This document transcribes the manual's procedural sections (4, 5, 6, 8, 9)
so they live in the repo as text alongside the parameter reference in
`GL10-PARAMETERS.md`. The original PDF stays the authoritative source for
ambiguous wording — these are working notes, not a legal translation.

## Startup procedure (Manual §4)

Before applying throttle to a newly-powered ESC:

1. Inspect the wiring for shorts, opens, or loose connections.
2. Verify the motor turns freely by hand — nothing mechanically jammed.
3. Plug in the power cable.
4. Turn on the ESC switch.
5. Listen for the **"Normal Startup"** prompt tone (see §9 LED & beep
   reference). When that tone plays, startup is normal and throttle can be
   applied.

If you hear a fault tone instead of the startup tone, do **not** apply
throttle — consult the fault-warning rows in §9.

## Throttle range calibration (Manual §5)

This teaches the ESC the exact PWM pulse widths your signal source uses for
full-reverse, full-forward, and neutral. The procedure is per-ESC, not
per-pair — each ESC stores its own learned range.

**Three-step procedure with the stick (or Arduino signal):**

1. **ESC powered off, signal held at full reverse.** Power on the ESC.
   Wait until the red/green LED indicator goes off — that confirms the ESC
   has entered calibration mode and registered the reverse endpoint.
2. **Quickly move the signal to full forward.** Wait for the green LED to
   go off — forward endpoint stored.
3. **Quickly move the signal to neutral (center).** Wait for the red LED
   to go off — neutral endpoint stored.

**Success indication:** the ESC repeats a prompt four times — red and green
LEDs solid + motor "lo, ro, do" tone.

**Failure indication:** red/green LEDs stay off, no completion tone. Power
the ESC down and retry — common causes are moving between positions too
slowly, or the signal source not actually reaching the documented PWM
extremes.

### Arduino integration note

In this project the Arduino sits between the receiver and the ESCs, so
"stick at full reverse" means **Arduino emitting 1000 µs** on that ESC's
signal lead. Two options when running the procedure:

- **Option A (recommended):** drive the calibration from the Arduino so
  each ESC learns the *exact* µs range the firmware actually outputs (this
  project: 1000/1500/2000). Write a one-off sketch that holds 1000 µs at
  boot, then advances to 2000 µs and 1500 µs on Serial input or fixed delays.
- **Option B:** bypass the Arduino, plug the receiver throttle channel
  straight into the ESC's signal lead, and use the physical transmitter
  stick for the three positions. Faster if you don't mind the temporary
  re-wire — but the ESC ends up calibrated to the transmitter's range, not
  the Arduino's, and any small offset between the two reappears as a
  per-ESC asymmetry.

After calibration, power-cycle the ESC. Repeat on the second ESC.

## Bluetooth pairing & default password (Manual §6)

- **Bluetooth name format:** `<model>-<ESC-code-ID>`, e.g. `KC_010-1CB1`
  where `KC_010` is the product family and `1CB1` is this ESC's hex
  code-ID. Each ESC has a unique code-ID so two ESCs on the bench advertise
  distinct Bluetooth names.
- **Default password:** `1234`.

Use the XC-Link Bluetooth app to read/write parameters. The GL10 has no
programming card or DIP switches — Bluetooth is the only configuration
channel.

## Factory reset (Manual §8)

### Restore Bluetooth password / boot mode

If the Bluetooth password is lost (or you need to force the ESC into Boot
mode to upgrade firmware):

1. Short the ESC's BEC- pin to ground (red wire end of the BEC connector).
2. Turn on the ESC.
3. When the green LED goes off and the red LED comes on, disconnect the
   short.
4. Remove the shorting wire.

Result: Bluetooth name and password reset to factory defaults (password
`1234`). The same procedure activates Boot mode for firmware upgrade if a
hardware error has bricked the normal boot path.

### Restore parameter values

To restore the configurable parameters (the items in `GL10-PARAMETERS.md`)
to factory defaults: open the XC-Link app's parameter page and tap
**Default**. This does not touch the throttle range calibration — that's
stored separately and must be re-run via §5 if needed.

### What factory-reset does **not** clear

- **Throttle range calibration** (§5 procedure) — stays unless you re-run
  the calibration. This is exactly why two ESCs with identical exported
  parameter files can still produce asymmetric throttle readings: their
  learned endpoint ranges differ.
- **Per-unit Hall sensor / motor auto-detect results** (if the GL10 stores
  any — the manual doesn't explicitly document this, but FOC ESCs in this
  class typically do). If swapping in a new motor, expect to re-run any
  motor-pairing or Hall-alignment routine the XC-Link app offers.

## LED & beep reference (Manual §9)

This is a working extract of the prompt table. The manual's column layout
is dense and image-based; verify against `docs/GL10-Manual.pdf` page 3 for
any ambiguous case.

### Normal-operation prompts

| Condition | Light | Sound |
| --- | --- | --- |
| Throttle not zeroed at boot | Red flashing | Long tone "beep1" |
| Throttle zeroed at boot | Red flashing | Long tone "beep2" |
| Voltage normal (in range) | Red/green steady | Detected cell-count beeps |
| Voltage low (warning) | Red flashing | Long tone "beep3" |
| Voltage over-range | Red flashing | Long tone "beep4" |
| Temperature normal | (no indicator) | — |
| Temperature too high (MOS or motor) | Red rapid flash | Short tone repeated |
| Throttle calibration in progress | (varies by step — see §5) | — |
| Throttle calibration success | Red & green solid | "lo, ro, do" motor tone, repeated 4× |

### Brake / startup prompts

| Condition | Light | Sound |
| --- | --- | --- |
| Brake engaged | Red flash | — |
| Normal startup (initial boot tone) | Red & green | Battery cell-count + short confirmation |
| Normal startup (after throttle returns from non-zero) | Green steady | — |

### Fault prompts (ESC self-check abnormality)

When the ESC reports a fault during self-check, the type of fault is
encoded in the beep pattern (1×, 2×, 3×, 4×, or 5× repeats). See the
manual's "Fault warning" rows for the per-pattern meaning — patterns
include MOS over-temp, motor over-temp, over-current, Hall failure, and
under/over-voltage. Reset by power-cycling once the underlying condition
clears; persistent faults indicate hardware damage and require service.

## GL540L motor

We do not currently have a manufacturer manual for the GL540L motor in the
repo. The GL540L is a sensored brushless motor paired with the GL10 ESC;
configuration and tuning happens on the ESC side, not the motor. If a
motor-specific manual surfaces (e.g. Hall sensor pinout, magnetic
alignment procedure, recommended idle current), add it as
`docs/GL540L-Manual.pdf` and create a `GL540L-OPERATION.md` alongside this
file.

## See also

- `docs/GL10-Manual.pdf` — original 3-page manual
- `docs/GL10-PARAMETERS.md` — configurable parameters table + code-context
  analysis (what each parameter does to the Arduino's control commands)
- `docs/XBUS-PROTOCOL.md` — X.BUS protocol reference (the alternate
  digital control path for these ESCs, currently deferred)
