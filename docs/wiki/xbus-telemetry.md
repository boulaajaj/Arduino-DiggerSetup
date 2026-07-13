# X.BUS telemetry

How the firmware reads voltage, current, RPM, and temperatures from both
ESCs without ever taking control authority: it polls the GL10's **Read
Register** function over a shared half-duplex UART bus. The critical protocol
choice — Read Register is point-to-point service traffic that never puts the
ESC into bus-control mode, so PWM command authority is fully preserved. The
throttle-over-bus function would fight our PWM and is never used.

- Wire protocol, framing, and register map:
  [XBUS-PROTOCOL](../XBUS-PROTOCOL.md) (canonical)
- Current registers polled, cadence, smoothing, and freshness watchdog:
  [CLAUDE.md](../../CLAUDE.md) "Telemetry"
- Code home: the poller (framing, checksum, parse, EMA, poll state machine)
  is `src/infrastructure/xc/XbusTelemetryAdapter` behind
  `src/ports/TelemetrySource.h`; the sketch owns the telemetry array
  ([architecture remediation](architecture-remediation.md), slice 4)
- Consumers: the [battery ladder](battery-ladder.md),
  [thermal derating](thermal-derating.md), and the
  [Wi-Fi dashboard](wifi-dashboard.md) — monitoring-only by permanent
  decision ([telemetry and dashboard](telemetry-and-dashboard.md))
- Plausibility gates reject implausible readings — with one documented
  exception: NaN pack voltage passes the voltage band ([SAFETY](../SAFETY.md)
  "Known gaps"; unreachable today via the integer parse path). Dropout
  freezes the safety ladders rather than releasing them.
- Bus wiring and the X.BUS merge on the interface board:
  [hardware](hardware.md)
