// Local dashboard UI test — render dashboard/index.html with MOCK telemetry and
// screenshot it, so UI changes can be verified WITHOUT flashing the digger.
//
// Usage:  node tools/ui-test.mjs
// Output: .shots/arrow/<case>.png  (one per mock case) + full_forward.png
//
// Requires Playwright (npx playwright) + system Chrome. It injects data by
// calling the page's own applyData() with a mock JSON frame — the same shape the
// firmware streams over SSE — so we exercise the real dashboard code.
import { chromium } from 'playwright';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';
import { mkdirSync } from 'fs';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const page_url = 'file://' + resolve(root, 'dashboard/index.html');
const outDir = resolve(root, '.shots/arrow');
mkdirSync(outDir, { recursive: true });

// One mock SSE frame. cur is 0.1 A, v is 0.1 V (firmware integer scaling).
const mock = (outL, outR) => ({
  t: 0, seq: 1, gear: 1, mode: 0, fs: 0, lost: 0, outL, outR,
  e0: { ok: 1, age: 50, rpm: 12, cur: 50, v: 118, tE: 30, tM: 25 },
  e1: { ok: 1, age: 50, rpm: 12, cur: 50, v: 118, tE: 30, tM: 25 },
});

// (outL, outR, label, expected heading)
const cases = [
  [1800, 1800, 'fwd_straight',  'UP / green'],
  [1200, 1200, 'rev_straight',  'DOWN / red'],
  [1200, 1800, 'pivot_right',   'RIGHT (→) / orange'],   // right turn = right track faster (R>L)
  [1800, 1200, 'pivot_left',    'LEFT (←) / orange'],
  [1500, 1800, 'fwd_right',     'UP-RIGHT ~45° / green'],
  [1800, 1500, 'fwd_left',      'UP-LEFT ~-45° / green'],
  [1100, 1300, 'rev_right',     'DOWN-RIGHT / red'],
  [1500, 1500, 'neutral',       'dim (deadzone)'],
];

const browser = await chromium.launch({ channel: 'chrome' });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
await page.goto(page_url, { waitUntil: 'domcontentloaded' });
await page.waitForTimeout(300);

for (const [outL, outR, name, expect] of cases) {
  await page.evaluate((d) => window.applyData(d), mock(outL, outR));
  await page.waitForTimeout(450); // let the CSS rotate/colour transition settle
  // Clip a region around the combined direction arrow so the rotated glyph is fully captured.
  const box = await page.locator('#vDir').boundingBox();
  const pad = 70;
  await page.screenshot({
    path: resolve(outDir, name + '.png'),
    clip: { x: box.x - pad, y: box.y - pad, width: box.width + pad * 2, height: box.height + pad * 2 },
  });
  console.log(`shot ${name}  (outL=${outL} outR=${outR})  expect: ${expect}`);
}

// One full-page shot (forward) to sanity-check bars/scale/mode pills overall.
await page.evaluate((d) => window.applyData(d), mock(1750, 1750));
await page.waitForTimeout(450);
await page.screenshot({ path: resolve(outDir, 'full_forward.png') });
console.log('shot full_forward');

await browser.close();
console.log('done →', outDir);
