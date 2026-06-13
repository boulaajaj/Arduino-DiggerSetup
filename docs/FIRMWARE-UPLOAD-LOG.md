# Firmware Upload Log

Every firmware upload to the digger gets a row here: date, version, commit
SHA, branch, board/port, and notes on what was tested.

| Date | Version | SHA | Branch | Board / Port | Notes |
| --- | --- | --- | --- | --- | --- |
| 2026-05-31 | rc_test V7.6 | af9f84c | claude/v8-uno-r4-wifi-migration | UNO R4 WiFi / COM7 | First UNO R4 WiFi bring-up. Compiles 22% flash / 25% RAM. S.BUS verified live on SCI0/D12 after fixing inverter wiring (collector↔base swap). Throttle, steering, curvatureDrive differential on D9/D10, and Eco reverse cap (1375 µs) all confirmed. FS=0/Lost=0. X.BUS telemetry not polled by this firmware. |
| 2026-06-11 | rc_test V7.7 | 181f4eb | claude/restore-xbus-telemetry-36 | UNO R4 WiFi / COM7 | Added non-blocking X.BUS Read Register (0x10) telemetry on Serial1/D0-D1 — telemetry WITH control (0x10 never enters BUS_MODE). 58.9 KB, 22% flash / 26% RAM. Confirmed live: ESC0 reads 11.9V / ESC 28°C / motor 22°C / RPM tracks throttle (7→38 Hz); control intact (FS=0/Lost=0, outputs follow stick). ESC1 silent (OK1=0) — wiring or X.BUS address. Bus current 0.0A at idle (verify under load). CSV format changed (added per-ESC telemetry columns). |
| 2026-06-13 | rc_test V7.8 | 8f92d38 | claude/restore-xbus-telemetry-36 | UNO R4 WiFi / COM7 | Wi-Fi AP telemetry server (WiFiS3) + live dashboard. AP "Digger-Telemetry" (pass digger12345) @ 192.168.4.1; GET /data → telemetry JSON (CORS *), monitoring only. beginAP status 7 (AP_LISTENING) confirmed on boot; both ESCs OK while AP up. Non-blocking server (1 client/loop, 20ms bounded read). dashboard/index.html rewired sim→live /data polling (5Hz) + NO-LINK watchdog. 69.8 KB, 26% flash / 28% RAM. Includes wifiDebug() 3s status print (bring-up; remove later). |
