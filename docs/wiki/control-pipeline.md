# Control pipeline

Stick to track, end to end — all shaping happens on the Arduino, all motor
smoothing inside the ESC's FOC. Canonical description with current values:
[CLAUDE.md](../../CLAUDE.md) "Architecture Summary"; research background:
[CONTROL-RESEARCH](../CONTROL-RESEARCH.md).

1. Inputs: [S.BUS input](sbus-input.md) (RC) and the joystick ADC
2. Shaping: deadband + expo curves per axis
3. Mixing: [curvature drive](curvature-drive.md) (tank mix with pivot blend)
4. Authority: [override modes](override-modes.md) select RC / joystick / blend
5. Limits: [gear and reverse caps](gear-and-reverse-caps.md), then the
   [safety system](safety-system.md) may derate or close the
   [output gate](output-gate.md)
6. Output: servo PWM per track to the GL10 ESCs ([hardware](hardware.md))

Two permanent design decisions guard this pipeline: it is **open-loop** (no
RPM feedback — the FOC ESC owns execution) and **nothing on the Wi-Fi side
can command a motor** ([telemetry and dashboard](telemetry-and-dashboard.md)
is monitoring-only). Why smoothness dominates every trade-off:
[MISSION](../MISSION.md).
