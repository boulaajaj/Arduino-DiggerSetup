# Dev tools

Human-run development tooling, one home (#123). CI/hook machinery lives in
`scripts/` — the split is deliberate: `scripts/` runs in pipelines,
`tools/` runs on a bench.

## Serial + telemetry (need the board on a COM port)

| Tool | Run | What it does |
| --- | --- | --- |
| `monitor.py` | `python tools/monitor.py` | Plain serial monitor |
| `live_plot.py` | `python tools/live_plot.py` | Real-time matplotlib plot of the debug CSV stream |
| `live_plot_telemetry.py` | `python tools/live_plot_telemetry.py` | Live plot of the X.BUS telemetry values |
| `deploy.py` | `python tools/deploy.py` | Compile + upload the production sketch via arduino-cli |

## Dashboard UI tests (no hardware — mock SSE + Playwright screenshots)

| Tool | Run | What it does |
| --- | --- | --- |
| `ui-test.mjs` | `npm run ui-test` | Renders the dashboard with mock frames, screenshots it |
| `ui-test-battery.mjs` | `npm run ui-test:battery` | Battery gauge states |
| `ui-test-battery-color.mjs` | `npm run ui-test:battery-color` | Battery color thresholds |
| `ui-test-safety.mjs` | `npm run ui-test:safety` | Low-battery banner (eco lock / hard cutoff) |
| `ui-test-thermal.mjs` | `npm run ui-test:thermal` | Thermal warning states |
| `drive-trace.mjs` | `npm run drive-trace` | Mirrors the curvatureDrive math to verify pivot/turn behavior without flashing |

## Hardware drawings (no hardware)

| Tool | Run | What it does |
| --- | --- | --- |
| `draw_perfboard.py` | `python tools/draw_perfboard.py` | Interface-board perfboard layout PNG |
| `draw_singlepad.py` | `python tools/draw_singlepad.py` | Single-pad drawing variant |

Dashboard UI testing policy: mock data + screenshots BEFORE flashing
(standing rule). The dashboard source of truth is `dashboard/index.html`;
`web_page.h` is generated.
