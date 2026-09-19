/**
 * Write what the robot's webcam would have seen of the goal tags, frame by frame, for one run.
 *
 *     node tools/browser/frames.mjs --bench http://localhost:21986/sim --run 3 --into frames/
 *     node tools/browser/frames.mjs --bench ... --run 3 --into frames/ --cookie "session=..."
 *
 * The bench is where the run and the field model come from; the page and its assets come from
 * there too, so what is captured is the page as served rather than a copy of it. The coding
 * server wants an approved session, which is what `--cookie` carries.
 *
 * Each frame is a PNG at the camera's own resolution, named for the loop it is. What is in one
 * is the tags and nothing else: the field is hidden rather than absent, so a tag still moves
 * with the hive it hangs on as the run plays.
 *
 * The lens and the tag artwork's fit to its plate are assumptions, written down in field.html.
 * Frames from here are geometry, not a measurement, until the webcam has been calibrated.
 */
import fs from 'node:fs';
import path from 'node:path';
import { chromium } from 'playwright';

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

const browser = await chromium.launch({
  executablePath: process.env.CHROME_BIN || undefined,
  ...(process.env.CHROME_BIN ? {} : { channel: 'chromium' }),
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--enable-unsafe-swiftshader',
         '--use-gl=angle', '--use-angle=swiftshader']
});

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
  // The page keeps its heads-up display and its play controls, because it is watched as well as
  // captured. A frame is meant to be what the webcam would have seen, and neither is part of
  // that, so they go for the capture and only for the capture.
  await page.addStyleTag({ content: '#hud, #run { display: none !important; }' });
  fs.mkdirSync(into, { recursive: true });

  const width = String(loops).length;
  for (let loop = 0; loop < loops; loop += Math.max(1, every)) {
    await page.evaluate((i) => window.fieldPage.goTo(i), loop);
    // Two frames: the first carries the move, the second is drawn with it in place.
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
