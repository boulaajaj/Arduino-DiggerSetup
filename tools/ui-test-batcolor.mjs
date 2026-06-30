// Verify dashboard battery colour bands directly via bc(p): green >30 / orange
// 20-30 / red <=20.  node tools/ui-test-batcolor.mjs
import { chromium } from 'playwright';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const page_url = 'file://' + resolve(root, 'dashboard/index.html');

// [percent, expected band]
const cases = [[100,'green'],[40,'green'],[31,'green'],[30,'orange'],[25,'orange'],[21,'orange'],[20,'red'],[15,'red'],[0,'red']];
const band = (g) => g.includes('#0c6')?'green' : g.includes('#fa0')?'orange' : 'red';

const browser = await chromium.launch({ channel: 'chrome' });
const page = await browser.newPage();
await page.goto(page_url, { waitUntil: 'domcontentloaded' });
let pass = true;
console.log(' % | got    | expected');
for (const [p, exp] of cases) {
  const grad = await page.evaluate((x) => window.bc(x), p);
  const got = band(grad);
  const ok = got === exp; if (!ok) pass = false;
  console.log(`${String(p).padStart(3)} | ${got.padEnd(6)} | ${exp} ${ok ? 'ok' : '  <-- MISMATCH'}`);
}
await browser.close();
if (!pass) { console.error('FAIL'); process.exit(1); } else console.log('all bands correct');
