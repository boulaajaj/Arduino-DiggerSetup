// Verify dashboard battery colour bands (green >30 / orange 20-30 / red <=20)
// for BOTH the bar gradient bc(p) AND the % text colour batColor(p).
//   node tools/ui-test-batcolor.mjs
import { chromium } from 'playwright';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const page_url = 'file://' + resolve(root, 'dashboard/index.html');

// [percent, expected band]
const cases = [[100,'green'],[40,'green'],[31,'green'],[30,'orange'],[25,'orange'],[21,'orange'],[20,'red'],[15,'red'],[0,'red']];
// bc() gradient uses #0c6/#fa0/#f44; batColor() text uses #0c6/#fc0/#f44.
const bandGrad = (g) => g.includes('#0c6')?'green' : g.includes('#fa0')?'orange' : 'red';
const bandRGB  = (c) => c.includes('0, 204, 102')?'green' : c.includes('255, 204, 0')?'orange' : 'red';
const anyFrame = { t:0,seq:1,gear:1,mode:0,fs:0,lost:0,outL:1500,outR:1500,
  e0:{ok:1,age:50,rpm:0,cur:0,v:115,tE:30,tM:25}, e1:{ok:1,age:50,rpm:0,cur:0,v:115,tE:30,tM:25} };

const browser = await chromium.launch({ channel: 'chrome' });
const page = await browser.newPage();
await page.goto(page_url, { waitUntil: 'domcontentloaded' });
await page.waitForTimeout(200);
let pass = true;

console.log('--- bar gradient bc(p) ---\n  % | got    | expected');
for (const [p, exp] of cases) {
  const got = bandGrad(await page.evaluate((x) => window.bc(x), p));
  const ok = got === exp; if (!ok) pass = false;
  console.log(`${String(p).padStart(3)} | ${got.padEnd(6)} | ${exp} ${ok ? 'ok' : '  <-- MISMATCH'}`);
}

// batColor() is a local const inside applyData(); exercise it by forcing bp() to
// return each percent, running a frame, then reading the rendered #batLPct colour.
console.log('\n--- % text colour batColor(p) ---\n  % | got    | expected');
for (const [p, exp] of cases) {
  await page.evaluate((P) => { window.bp = () => P; }, p);
  await page.evaluate((d) => window.applyData(d), anyFrame);
  await page.waitForTimeout(30);
  const got = bandRGB(await page.evaluate(() => getComputedStyle(document.getElementById('batLPct')).color));
  const ok = got === exp; if (!ok) pass = false;
  console.log(`${String(p).padStart(3)} | ${got.padEnd(6)} | ${exp} ${ok ? 'ok' : '  <-- MISMATCH'}`);
}

await browser.close();
if (!pass) { console.error('FAIL'); process.exit(1); } else console.log('\nall bands correct (bar + text)');
