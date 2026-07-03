// Local drivetrain trace — mirrors curvatureDrive() math so we can verify the
// pivot↔drive transition and turning behavior WITHOUT flashing.
// Run: node tools/drive-trace.mjs
//
// Mirrors sketches/rc_test/rc_test.ino exactly (V7.34: #86 additive delta,
// #96 outer-track ceiling, #114 pivot-branch throttle taper). If you change
// the firmware math, change it here too (kept deliberately small).
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const smoothstep = (t) => { t = clamp(t, 0, 1); return t * t * (3 - 2 * t); };

// [CONFIG] mirror — keep in sync with rc_test.ino
const PIVOT_BLEND_START = 0.05, PIVOT_BLEND_END = 0.55;
const PIVOT_CAP = 0.60, PIVOT_CAP_ECO = 0.725;
const TURN_TRACK_CAP = 0.70;
const PIVOT_THROTTLE_TAPER = 0.70;   // set 0.0 to reproduce pre-#114 behavior

function curvatureDrive(x, z, gearScale, eco, taper = PIVOT_THROTTLE_TAPER) {
  x = clamp(x, -1, 1); z = clamp(z, -1, 1);
  // Pivot branch — throttle tapered by RAW steering (#114)
  const pc = eco ? PIVOT_CAP_ECO : PIVOT_CAP;
  const cr = clamp(z, -pc, pc);
  const pivotThr = x * (1 - taper * Math.abs(z));
  const pL = (pivotThr - cr) * gearScale, pR = (pivotThr + cr) * gearScale;
  // Curvature branch — additive delta, sign from the steering stick (#86)
  const avg = x * gearScale;
  const delta = Math.abs(avg) * Math.abs(z);
  let cL, cR;
  if (z > 0) { cL = avg - delta; cR = avg + delta; } else { cL = avg + delta; cR = avg - delta; }
  // Outer-track ceiling fades rail → TURN_TRACK_CAP with |z| (#96)
  const ceiling = 1 - (1 - TURN_TRACK_CAP) * Math.abs(z);
  const peak = Math.max(Math.abs(cL), Math.abs(cR));
  if (peak > ceiling) { const k = ceiling / peak; cL *= k; cR *= k; }
  // Smoothstep blend pivot → curvature over |x| (#72)
  const t = smoothstep((Math.abs(x) - PIVOT_BLEND_START) / (PIVOT_BLEND_END - PIVOT_BLEND_START));
  return { L: pL * (1 - t) + cL * t, R: pR * (1 - t) + cR * t, t };
}

const us = (w) => Math.round(clamp(1500 + w * 500, 1000, 2000));
const f = (n) => (n >= 0 ? ' ' : '') + n.toFixed(2);

// ── Trace A: pivot→forward hand-off (#114) — taper 0.70 vs 0.0 (pre-#114)
for (const z of [1.0, 0.4]) {
  console.log(`\n=== A. Pivot right (z=${-z}) + forward throttle (Normal 0.80) ===`);
  console.log('  L=outer should HOLD, R=inner should ease through zero — no surge.');
  console.log('  xSpd | pre-#114 L    R    | V7.34   L    R    (µs L/R)');
  for (let x = 0; x <= 0.601; x += 0.05) {
    const o = curvatureDrive(x, -z, 0.80, false, 0.0);
    const n = curvatureDrive(x, -z, 0.80, false);
    console.log(`  ${x.toFixed(2)} |      ${f(o.L)} ${f(o.R)} |       ${f(n.L)} ${f(n.R)} (${us(n.L)}/${us(n.R)})`);
  }
}

// ── Trace B: invariants — taper must not change straight / pure pivot / at-speed turns
console.log('\n=== B. Invariant check: max |taper0.70 − taper0| where behavior must be identical ===');
let worst = 0;
for (const [g, eco] of [[0.65, true], [0.80, false], [1.00, false]]) {
  for (let i = -100; i <= 100; i++) {
    const x = i / 100;
    for (let j = -100; j <= 100; j++) {
      const z = j / 100;
      if (z !== 0 && x !== 0 && Math.abs(x) < PIVOT_BLEND_END) continue; // only invariant zones
      const a = curvatureDrive(x, z, g, eco, 0.0), b = curvatureDrive(x, z, g, eco);
      worst = Math.max(worst, Math.abs(a.L - b.L), Math.abs(a.R - b.R));
    }
  }
}
console.log(`  max diff = ${worst.toFixed(6)} (must be 0.000000)`);

// ── Trace C: full throttle while steering — outer/inner/avg per gear (headroom + #96 ceiling)
console.log('\n=== C. Full throttle (xSpd=1.0) while steering — per gear ===');
for (const [name, s, eco] of [['Eco', 0.65, true], ['Normal', 0.80, false], ['Boost', 1.00, false]]) {
  console.log(`\n  ${name} (gear ${s})`);
  console.log('   z   | outer inner  avg');
  for (let z = 0; z <= 1.001; z += 0.25) {
    const n = curvatureDrive(1.0, z, s, eco);
    const a = (Math.abs(n.L) + Math.abs(n.R)) / 2;
    console.log(`  ${z.toFixed(2)} | ${f(Math.max(n.L, n.R))} ${f(Math.min(n.L, n.R))} ${f(a)}`);
  }
}
console.log('\n(Boost has no headroom by design; #96 ceiling fades outer toward 0.70 at full steer.)');
