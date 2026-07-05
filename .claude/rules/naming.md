# Naming Rules — self-documenting, full words

Code must read without a decoder ring. **Default: complete English words for
every identifier** — types, functions, variables, files, folders.

- `telemetry`, not `telem`. `hardware`, not `hw`. `joystick`, not `joy`.
  `command`, not `cmd`. `temperature`, not `temp` (ambiguous with
  "temporary"). `previous`, not `prev`. `button`, not `btn`.
- Applies to NEW and MOVED code. Do not mass-rename untouched legacy code —
  names are fixed as each file is extracted during the Phase D migration.

## Permitted exceptions (closed list — extending it requires operator sign-off)

1. **Universal domain acronyms**, uppercase per convention: `PWM`, `ADC`,
   `ESC`, `RC`, `RPM`, `SBUS`, `XBUS`, `JSON`, `HTTP`, `SSE`, `CSV`, `URL`,
   `ID`, `WiFi`, `LiPo`, `AP`, `WDT`. These are clearer than their expansions
   (`pulseWidthModulation` self-documents worse than `PWM`).
2. **Unit suffixes** on quantities: `Ms`, `Us` (microseconds), `V`, `A`,
   `Hz`, `C` (Celsius) — e.g. `lastGoodMs`, `cutoffThresholdV`. The unit IS
   the documentation.
3. **Loop indices** `i`/`j` in scopes shorter than ~5 lines.
4. **Wire-format keys** (SSE/JSON payload, CSV columns): compact keys are a
   deliberate bandwidth decision on the 448-byte SSE frame budget, not
   naming — keep them short and document the mapping next to the encoder.
5. **`config`** as the conventional name for the tunables layer.

## White-label rule — branding is data, not code

This platform is generic: it must be buildable for any vehicle, any rider, any
slogan. Therefore:

- **No person or brand names** in code identifiers, file/folder names, issue
  and board titles, or architecture documents. "Malaki SuperTracks" is one
  *skin* of the dashboard, not the platform's name — the platform is
  Dual Track Control.
- Product branding (display name, subtitle/slogan, logo/badge, accent colors)
  lives in **one configuration block** in the dashboard source so it can be
  swapped without touching logic (issue #136).
- People's names are fine where they document reality (operator guide, wiring
  history, decision log) — that is documentation, not branding.

## Why identifier length is free (and where brevity actually matters)

Identifiers are compile-time only — they cost zero flash and zero RAM in the
binary. Never justify an abbreviation with "Arduino memory limits." The only
places byte-length matters are string literals (debug prints, JSON keys,
HTML), which is exactly why exception 4 exists for wire formats and why the
identifier rule has no space-based escape hatch.
