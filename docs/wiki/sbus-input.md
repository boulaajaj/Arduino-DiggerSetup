# S.BUS input

All RC channels arrive on one wire: the receiver's S.BUS output — an
electrically **inverted** UART protocol — passes through an NPN inverter on
the [interface board](hardware.md) into a dedicated hardware UART on the
Arduino. From one frame the firmware reads throttle, steering, the
[override mode](override-modes.md) switch, the
[gear](gear-and-reverse-caps.md) switch, and the horn button.

- Current pins, UART instance naming, and frame parameters:
  [CLAUDE.md](../../CLAUDE.md) "UART Architecture" (canonical); wiring:
  [WIRING-GUIDE-V8](../WIRING-GUIDE-V8.md)
- **Per-channel failsafe** — each channel times out independently, and the
  receiver's failsafe flag is honored; stale or failsafed RC never produces
  non-neutral propulsion ([SAFETY](../SAFETY.md) · [testing](testing.md))
- Standing rule: hardware UARTs only, never `SoftwareSerial`
  ([CLAUDE.md](../../CLAUDE.md) coding rules)
- Position in the chain: [control pipeline](control-pipeline.md)
