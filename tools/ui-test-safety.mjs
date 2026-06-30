// Local UI test for the low-battery safety banner (eco lock / hard cutoff).
// Renders dashboard/index.html, injects mock SSE frames with eco/cut set, and
// screenshots each so the banner can be verified WITHOUT flashing the digger.
//   node tools/ui-test-safety.mjs  → .shots/safety/<case>.png
import { chromium } from 'playwright';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';
import { mkdirSync } from 'fs';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const page_url = 'file://' + resolve(root, 'dashboard/index.html');
const outDir = resolve(root, '.shots/safety');
mkdirSync(outDir, { recursive: true });

const base = { t: 0, seq: 1, gear: 1, mode: 0, fs: 0, lost: 0, outL: 1650, outR: 1650,
  e0: { ok: 1, age: 50, rpm: 12, cur: 50, v: 109, tE: 30, tM: 25 },
  e1: { ok: 1, age: 50, rpm: 12, cur: 50, v: 109, tE: 30, tM: 25 } };

const cases = [
  ['normal',   { eco: 0, cut: 0 }, 'no banner'],
  ['eco_lock', { eco: 1, cut: 0 }, 'AMBER: ECO LOCKED'],
  ['cutoff',   { eco: 1, cut: 1 }, 'RED: MOTORS CUT (cut wins)'],
];

const browser = await chromium.launch({ channel: 'chrome' });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
await page.goto(page_url, { waitUntil: 'domcontentloaded' });
await page.waitForTimeout(300);

for (const [name, flags, expect] of cases) {
  await page.evaluate((d) => window.applyData(d), { ...base, ...flags });
  await page.waitForTimeout(300);
  const banner = await page.evaluate(() => {
    const b = document.getElementById('safetyBanner');
    return b ? { shown: b.style.display !== 'none', text: b.textContent, bg: b.style.background } : { shown: false, text: '(none)' };
  });
  await page.screenshot({ path: resolve(outDir, name + '.png') });
  console.log(`${name}: expect ${expect}  →  shown=${banner.shown} "${banner.text}" ${banner.bg||''}`);
}
await browser.close();
console.log('done →', outDir);
