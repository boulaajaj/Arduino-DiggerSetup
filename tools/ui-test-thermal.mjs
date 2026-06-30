// Verify the dashboard motor-thermal banners (#111) + firmware version badge.
//   node tools/ui-test-thermal.mjs
// Drives applyData() with thermal flags and asserts the safety-banner text/colour
// and priority ordering, plus the discrete top-left version badge.
import { chromium } from 'playwright';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const page_url = 'file://' + resolve(root, 'dashboard/index.html');

// Base frame: both ESCs fresh, cool. Override per-case.
const base = () => ({ t:0,seq:1,gear:1,mode:0,fs:0,lost:0,outL:1500,outR:1500,
  eco:0,cut:0,hot:0,teco:0,tcut:0,
  e0:{ok:1,age:50,rpm:0,cur:0,v:115,tE:30,tM:25},
  e1:{ok:1,age:50,rpm:0,cur:0,v:115,tE:30,tM:25} });

// [label, frame-mutator, expected substring in banner (or null = hidden), expected bg rgb]
const RED='rgb(176, 0, 32)', AMBER='rgb(199, 120, 0)', GREEN='rgb(10, 125, 44)';
const cases = [
  ['cool → hidden',        d=>d,                                              null,            null],
  ['hot warn 82°C',        d=>{d.hot=1; d.e1.tE=82;},                         'MOTOR HOT 82',  AMBER],
  ['thermal eco 91°C',     d=>{d.teco=1; d.hot=1; d.e0.tM=91;},               'ECO LOCKED — MOTOR HOT 91', AMBER],
  ['thermal cut 96°C',     d=>{d.tcut=1; d.teco=1; d.hot=1; d.e1.tM=96;},     'MOTORS CUT — OVERHEAT 96',  RED],
  ['cut>batt-cut priority', d=>{d.tcut=1; d.cut=1; d.e0.tE=97;},              'OVERHEAT 97',   RED],
  ['batt cut (no heat)',   d=>{d.cut=1;},                                     'LOW BATTERY',   RED],
  ['batt eco (no heat)',   d=>{d.eco=1;},                                     'ECO LOCKED — LOW BATTERY', AMBER],
];

const browser = await chromium.launch({ channel: 'chrome' });
const page = await browser.newPage();
await page.goto(page_url, { waitUntil: 'domcontentloaded' });
await page.waitForTimeout(200);
let pass = true;

const readBanner = async () => page.evaluate(() => {
  const b = document.getElementById('safetyBanner');
  if (!b || b.style.display === 'none') return { shown:false, txt:'', bg:'' };
  return { shown:true, txt:b.textContent, bg:getComputedStyle(b).backgroundColor };
});

console.log('--- thermal safety banner ---');
for (const [label, mut, exp, bg] of cases) {
  const d = base(); mut(d);
  await page.evaluate((f) => window.applyData(f), d);
  await page.waitForTimeout(30);
  const b = await readBanner();
  let ok;
  if (exp === null) ok = !b.shown;
  else ok = b.shown && b.txt.includes(exp) && b.bg === bg;
  if (!ok) pass = false;
  console.log(`${ok?'ok ':'XX '} ${label.padEnd(24)} | ${b.shown ? `"${b.txt}" [${b.bg}]` : '(hidden)'}`);
}

// Restored flash: go cut → clear, expect a green "MOTORS RESTORED" for ~3 s.
{
  const cut = base(); cut.tcut=1; cut.e0.tE=97;
  await page.evaluate((f)=>window.applyData(f), cut);
  await page.waitForTimeout(30);
  const clear = base();                       // tcut back to 0
  await page.evaluate((f)=>window.applyData(f), clear);
  await page.waitForTimeout(30);
  const b = await readBanner();
  const ok = b.shown && b.txt.includes('MOTORS RESTORED') && b.bg === GREEN;
  if(!ok) pass=false;
  console.log(`${ok?'ok ':'XX '} restored flash on cut→clear   | ${b.shown ? `"${b.txt}" [${b.bg}]` : '(hidden)'}`);
}

// Version badge present + non-empty.
{
  const v = await page.evaluate(() => {
    const e = document.getElementById('fwVer');
    return e ? e.textContent.trim() : null;
  });
  const ok = !!v && /^V\d+\.\d+$/.test(v);
  if(!ok) pass=false;
  console.log(`${ok?'ok ':'XX '} version badge                 | ${v}`);
}

await browser.close();
if (!pass) { console.error('\nFAIL'); process.exit(1); } else console.log('\nall thermal banners + badge correct');
