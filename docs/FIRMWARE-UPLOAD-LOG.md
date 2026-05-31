# Firmware Upload Log

Every firmware upload to the digger gets a row here: date, version, commit
SHA, branch, board/port, and notes on what was tested.

| Date | Version | SHA | Branch | Board / Port | Notes |
| --- | --- | --- | --- | --- | --- |
| 2026-05-31 | rc_test V7.6 | af9f84c | claude/v8-uno-r4-wifi-migration | UNO R4 WiFi / COM7 | First UNO R4 WiFi bring-up. Compiles 22% flash / 25% RAM. S.BUS verified live on SCI0/D12 after fixing inverter wiring (collector↔base swap). Throttle, steering, curvatureDrive differential on D9/D10, and Eco reverse cap (1375 µs) all confirmed. FS=0/Lost=0. X.BUS telemetry not polled by this firmware. |
