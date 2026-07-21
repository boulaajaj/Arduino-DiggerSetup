---
paths:
  - "dashboard/**"
  - "sketches/**/web_page.h"
  - "tools/ui-test*.mjs"
---

# Dashboard Rules

- **`dashboard/index.html` is the single source of truth; `web_page.h` is
  GENERATED** (#120) — never hand-edit it. After editing the source, run
  `python scripts/generate_web_page.py` and commit both files; CI
  (`dashboard-drift` job in structure-check) fails on any mismatch. Its
  planned home is `src/generated/` per the architecture target (#227).
- **Test UI changes locally before any flash**: run the Playwright harness
  (`tools/ui-test*.mjs`) with mock data and check the screenshots. Never make
  the operator tether to the machine to verify a UI change.
- **Monitoring only — permanent rule.** The dashboard never gains a control
  input; no code path from Wi-Fi to motor output, ever.
- **Serving must never block the control loop**: one bounded modem operation
  per loop pass, incremental page chunks, bounded timeouts (the #69 incident
  — a blocking page send while ESCs held throttle — is why).
- Page budget: served dashboard ≤ 100 KB (epic #81).
