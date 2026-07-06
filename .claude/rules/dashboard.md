---
paths:
  - "dashboard/**"
  - "sketches/**/web_page.h"
  - "tools/ui-test*.mjs"
---

# Dashboard Rules

- **`dashboard/index.html` is the source; `web_page.h` is the mirror.** Any
  change to one must land in the other in the same PR. (Until the generator
  from issue #120 exists, the mirror is maintained by hand — after #120,
  `web_page.h` moves to `src/generated/` and is never hand-edited.)
- **Test UI changes locally before any flash**: run the Playwright harness
  (`tools/ui-test*.mjs`) with mock data and check the screenshots. Never make
  the operator tether to the machine to verify a UI change.
- **Monitoring only — permanent rule.** The dashboard never gains a control
  input; no code path from Wi-Fi to motor output, ever.
- **Serving must never block the control loop**: one bounded modem operation
  per loop pass, incremental page chunks, bounded timeouts (the #69 incident
  — a blocking page send while ESCs held throttle — is why).
- Page budget: served dashboard ≤ 100 KB (epic #81).
