---
name: flash-and-log
description: Firmware upload procedure — use whenever flashing the digger (production or bench/test), including "upload this", "flash the board", or any arduino-cli upload. Enforces the flight-log rule - an unlogged flash is treated as not done.
---

# Flash and log

## Preconditions (all must hold before upload)

1. Host suite green: `wsl -e make -C tests run`.
2. Compiles clean:
   `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/dual_track_control`
3. The change is on an issue-linked PR branch (see `prepare-pull-request`
   skill) — never flash unreviewed drive-by edits.
4. If NO hardware is connected (epic #116 standing constraint), STOP: no
   flashing, no bench claims. Physical checks go to
   `docs/architecture/BENCH-VERIFICATION-DEFERRED.md` instead.

## Upload

```
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi -p COM7 sketches/dual_track_control
```

## Log the flash — IMMEDIATELY after, no exceptions

Append a row to `docs/FIRMWARE-UPLOAD-LOG.md`:

| Column | Content |
| --- | --- |
| Date | today |
| Version | sketch version string in the build (`FIRMWARE_VERSION`, `src/config/BuildInfo.h`) — not a git tag |
| SHA | short commit SHA actually flashed |
| Branch | branch name + PR number, e.g. `agent/C-Builder/foo (PR #93)` |
| Board/Port | e.g. `UNO R4 WiFi / COM7` |
| Notes | what changed + what to test on the machine |

This applies to bench/test uploads too. The log is the device's flight log:
**an unlogged flash is treated as not done.** If the flash was a tuning
iteration, also record the outcome per the `field-tune` skill.
