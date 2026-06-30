// Local test for the battery % curve (bp()). Injects mock pack voltages via the
// dashboard's own applyData() and reads back the displayed %, so the SoC fix can
// be verified WITHOUT flashing.  node tools/ui-test-batt.mjs
import { chromium } from 'playwright';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const page_url = 'file://' + resolve(root, 'dashboard/index.html');

const frame = (volts) => ({ t: 0, seq: 1, gear: 1, mode: 0, fs: 0, lost: 0, outL: 1500, outR: 1500,
  e0: { ok: 1, age: 50, rpm: 0, cur: 0, v: Math.round(volts * 10), tE: 30, tM: 25 },
  e1: { ok: 1, age: 50, rpm: 0, cur: 0, v: Math.round(volts * 10), tE: 30, tM: 25 } });

// [pack V, old linear %, expected curve %]
const cases = [[12.6,'100','100'],[12.0,'83','80'],[11.4,'67','50'],[11.1,'58','40'],[10.6,'44','22'],[10.0,'28','0']];

const browser = await chromium.launch({ channel: 'chrome' });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
await page.goto(page_url, { waitUntil: 'domcontentloaded' });
await page.waitForTimeout(300);

console.log('pack V | old(linear) | expect(curve) | actual');
for (const [v, oldp, exp] of cases) {
  // inject repeatedly so the eased display value settles on the target
  for (let i = 0; i < 30; i++) { await page.evaluate((d) => window.applyData(d), frame(v)); await page.waitForTimeout(20); }
  await page.waitForTimeout(200);
  const pct = await page.evaluate(() => document.getElementById('batLPct').textContent);
  console.log(`${v.toFixed(1)}V |     ${oldp}%     |     ~${exp}%     |   ${pct}`);
}
await browser.close();
