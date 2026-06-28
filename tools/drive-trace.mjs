// Local drivetrain trace — mirrors curvatureDrive() math so we can verify the
// pivot↔drive transition and the Eco/Normal turning headroom WITHOUT flashing.
// Run: node tools/drive-trace.mjs
//
// Mirrors sketches/rc_test/rc_test.ino exactly. If you change the firmware math,
// change it here too (kept deliberately small).
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const smoothstep = (t) => { t = clamp(t, 0, 1); return t * t * (3 - 2 * t); };
const PIVOT_CAP = 0.60, PIVOT_CAP_ECO = 0.725;

// NEW (this change): gear caps the AVERAGE; outer desaturates against ±1.0 → headroom.
function curveNew(x, z, gearScale, eco, bandEnd = 0.55, bandStart = 0.05) {
  const pc = eco ? PIVOT_CAP_ECO : PIVOT_CAP;
  const cr = clamp(z, -pc, pc);
  const pL = (x - cr) * gearScale, pR = (x + cr) * gearScale;
  const avg = x * gearScale, boost = 1 + Math.abs(z), slow = 1 - Math.abs(z);
  let cL, cR;
  if (z > 0) { cL = avg * slow; cR = avg * boost; } else { cL = avg * boost; cR = avg * slow; }
  const peak = Math.max(Math.abs(cL), Math.abs(cR));
  if (peak > 1) { cL /= peak; cR /= peak; }
  const t = smoothstep((Math.abs(x) - bandStart) / (bandEnd - bandStart));
  return { L: pL * (1 - t) + cL * t, R: pR * (1 - t) + cR * t, t };
}

// OLD (before this change): desaturate at full scale, THEN ×gearScale; narrow band 0.30.
function curveOld(x, z, gearScale, eco) {
  const pc = eco ? PIVOT_CAP_ECO : PIVOT_CAP;
  const cr = clamp(z, -pc, pc);
  const pL = x - cr, pR = x + cr;
  const boost = 1 + Math.abs(z), slow = 1 - Math.abs(z);
  let cL, cR;
  if (z > 0) { cL = x * slow; cR = x * boost; } else { cL = x * boost; cR = x * slow; }
  const peak = Math.max(Math.abs(cL), Math.abs(cR));
  if (peak > 1) { cL /= peak; cR /= peak; }
  const t = smoothstep((Math.abs(x) - 0.05) / (0.30 - 0.05));
  return { L: (pL * (1 - t) + cL * t) * gearScale, R: (pR * (1 - t) + cR * t) * gearScale };
}
const us = (w) => Math.round(clamp(1500 + w * 500, 1000, 2000));
const f = (n) => (n >= 0 ? ' ' : '') + n.toFixed(2);

// ── Trace A: pivot→forward hand-off (Normal, gentle steer z=0.3) — OLD vs NEW band
console.log('\n=== A. Pivot→forward transition (Normal gear, steer z=0.3) ===');
console.log('  inner = LEFT track (turning left). Watch it ease, not snap.');
console.log('  xSpd | t_old t_new | innerOLD innerNEW (µs) | outerNEW');
for (let x = 0; x <= 1.001; x += 0.1) {
  const o = curveOld(x, 0.3, 0.70, false);   // old Normal 0.70
  const n = curveNew(x, 0.3, 0.80, false);   // new Normal 0.80
  const tOld = smoothstep((x - 0.05) / (0.30 - 0.05));
  console.log(`  ${x.toFixed(2)} | ${tOld.toFixed(2)}  ${n.t.toFixed(2)} | ${f(o.L)}   ${f(n.L)} (${us(n.L)}) | ${f(n.R)}`);
}

// ── Trace C: full throttle while steering — average speed held? (per gear, OLD vs NEW)
console.log('\n=== C. Full throttle (xSpd=1.0) while steering — does avg speed hold? ===');
for (const g of [['Eco', 0.55, 0.65, true], ['Normal', 0.70, 0.80, false], ['Boost', 1.00, 1.00, false]]) {
  const [name, oldS, newS, eco] = g;
  console.log(`\n  ${name}  (gear ${oldS}→${newS})`);
  console.log('   z   | outOLD inOLD avgOLD | outNEW inNEW avgNEW');
  for (let z = 0; z <= 1.001; z += 0.25) {
    const o = curveOld(1.0, z, oldS, eco), n = curveNew(1.0, z, newS, eco);
    const ao = (Math.abs(o.L) + Math.abs(o.R)) / 2, an = (Math.abs(n.L) + Math.abs(n.R)) / 2;
    const outO = Math.max(o.L, o.R), inO = Math.min(o.L, o.R);
    const outN = Math.max(n.L, n.R), inN = Math.min(n.L, n.R);
    console.log(`  ${z.toFixed(2)} | ${f(outO)}  ${f(inO)} ${f(ao)} | ${f(outN)}  ${f(inN)} ${f(an)}`);
  }
}
console.log('\n(Boost OLD≈NEW = no headroom by design; Eco/Normal NEW avg should stay higher.)');
