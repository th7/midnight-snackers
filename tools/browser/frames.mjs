
import fs from 'node:fs';
import path from 'node:path';
import { chrome } from './bench.mjs';

function argument(name, fallback) {
  const at = process.argv.indexOf('--' + name);
  return at > 0 && at + 1 < process.argv.length ? process.argv[at + 1] : fallback;
}

const bench = argument('bench');
const run = argument('run');
const into = argument('into', 'frames');
const cookie = argument('cookie');
const every = Number(argument('every', '1'));

if (!bench || !run) {
  console.error('usage: node tools/browser/frames.mjs --bench <url> --run <id> [--into <dir>] '
      + '[--cookie <cookie>] [--every <n>]');
  process.exit(2);
}

const browser = await chrome();

let written = 0;
try {
  const context = await browser.newContext();
  if (cookie) {
    const url = new URL(bench);
    await context.addCookies(cookie.split(';').map((one) => {
      const [name, ...rest] = one.trim().split('=');
      return { name, value: rest.join('='), domain: url.hostname, path: '/' };
    }));
  }
  const page = await context.newPage();
  const at = bench.replace(/\/$/, '') + '/field?run=' + encodeURIComponent(run) + '&view=camera';
  await page.goto(at, { waitUntil: 'load', timeout: 120_000 });
  await page.waitForFunction(() => window.fieldPage && window.fieldPage.camera3,
      null, { timeout: 180_000 });
  await page.evaluate(() => window.fieldPage.tagsReady);

  const lens = await page.evaluate(() => window.fieldPage.lens);
  const loops = await page.evaluate(() => window.fieldPage.run && window.fieldPage.run.loops);
  if (!loops) {
    throw new Error('run ' + run + ' has no loops to capture');
  }
  await page.setViewportSize({ width: lens.width, height: lens.height });

  await page.addStyleTag({ content: '#hud, #run { display: none !important; }' });
  fs.mkdirSync(into, { recursive: true });

  const width = String(loops).length;
  for (let loop = 0; loop < loops; loop += Math.max(1, every)) {
    await page.evaluate((i) => window.fieldPage.goTo(i), loop);

    await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
    const name = 'loop-' + String(loop).padStart(width, '0') + '.png';
    await page.screenshot({ path: path.join(into, name) });
    written++;
  }
  console.log(`${written} frames of ${lens.width}x${lens.height} in ${into}`);
  console.log('  the lens and the tag artwork\'s fit are assumptions; see field.html before '
      + 'trusting a pose read off these');
} catch (wrong) {
  console.error('no frames: ' + (wrong && wrong.message ? wrong.message : wrong));
  process.exitCode = 1;
} finally {
  await browser.close();
}
