/**
 * Open the field page in a real browser and see that it draws.
 *
 * Everything else about this page is checked without one. `SimAssetsTest` holds that every
 * asset it names is served; `tools/renderer/check.mjs` holds that three.js loads the model and
 * that the field comes out the right size and the right way round. Neither has a canvas, and
 * so neither can say the page draws at all: a shader that will not compile, a WebGL context
 * that will not start, a camera pointed away from the field -- each leaves a page that loads
 * everything it asks for and shows nothing, and no gate we had could tell that from a working
 * one.
 *
 *     node tools/browser/check.mjs
 *
 * There is no GPU, so Chromium falls back to SwiftShader. That is slower and it is also the
 * only way a page test comes out the same on two machines.
 */
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const here = path.dirname(fileURLToPath(import.meta.url));
const sim = path.join(here, '..', '..', 'TeamCode', 'src', 'test', 'resources',
                      'org', 'firstinspires', 'ftc', 'teamcode', 'sim');

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.glb': 'model/gltf-binary',
  '.png': 'image/png'
};

/**
 * The bench serves the page at /field and its assets under /assets, from one directory. This
 * stands in for that, over the same directory, so the page is exercised exactly as it is
 * served -- the route itself is held by SimAssetsTest, which is Java's to answer for.
 */
function serve() {
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    const relative = asked === '/field' || asked === '/' ? 'field.html'
        : asked.startsWith('/assets/') ? asked.slice('/assets/'.length)
        : null;
    const file = relative && path.resolve(sim, relative);
    if (!file || !file.startsWith(path.resolve(sim)) || !fs.existsSync(file)) {
      response.writeHead(404).end('no');
      return;
    }
    response.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
    response.end(fs.readFileSync(file));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

const problems = [];
const check = (ok, said) => { if (!ok) problems.push(said); };

const server = await serve();
const url = `http://127.0.0.1:${server.address().port}/field`;

let browser = null;
try {
  browser = await chromium.launch({
    executablePath: process.env.CHROME_BIN || undefined,
    args: [
      // The image strips setuid bits and containers run with no-new-privileges, so Chromium's
      // own sandbox cannot start. See doc/browser-tests.md.
      '--no-sandbox',
      // A container's /dev/shm is 64 MB, and Chromium runs out of it part way through a page.
      '--disable-dev-shm-usage',
      // No GPU: WebGL falls back to SwiftShader, which newer Chromium will not do unless asked.
      '--enable-unsafe-swiftshader',
      '--use-gl=angle',
      '--use-angle=swiftshader'
    ]
  });
} catch (wrong) {
  server.close();
  console.error('no browser to test the page with. The image installs one and points CHROME_BIN '
      + 'at it; away from the image, run `npx playwright install chromium` in tools/browser.');
  console.error('  ' + String(wrong && wrong.message ? wrong.message : wrong).split('\n')[0]);
  process.exit(2);
}

const page = await browser.newPage({ viewport: { width: 1200, height: 800 } });

const thrown = [];
const failed = [];
page.on('pageerror', (wrong) => thrown.push(String(wrong && wrong.message ? wrong.message : wrong)));
page.on('requestfailed', (request) => failed.push(`${request.url()} (${request.failure()?.errorText})`));
page.on('response', (response) => {
  if (response.status() >= 400) {
    failed.push(`${response.url()} answered ${response.status()}`);
  }
});

try {
  await page.goto(url, { waitUntil: 'load', timeout: 60_000 });

  // The model is three megabytes and SwiftShader is not quick; wait for the page to say it is
  // loaded rather than for a time that would be a guess on one machine and wrong on another.
  await page.waitForFunction(
      () => window.fieldPage && (window.fieldPage.loaded || window.fieldPage.problem),
      null, { timeout: 120_000 });

  const state = await page.evaluate(() => ({
    loaded: window.fieldPage.loaded,
    problem: window.fieldPage.problem,
    said: document.getElementById('said').textContent
  }));

  check(!state.problem, `the page reported a problem: ${state.problem}`);
  check(state.loaded !== null, 'the page never loaded the model');
  if (state.loaded) {
    check(state.loaded.parts > 100, `only ${state.loaded.parts} parts reached the page`);
    check(state.loaded.triangles > 100000, `only ${state.loaded.triangles} triangles reached the page`);
  }

  // A frame has to have been drawn since the model arrived, so wait for one and then ask how
  // much of it was rasterised. Zero is a field that loaded and was not drawn.
  await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  const drawn = await page.evaluate(() => window.fieldPage.drawnTriangles);
  check(drawn > 1000, `only ${drawn} triangles were drawn in a frame; the field is loaded but not on screen`);

  check(thrown.length === 0, `the page threw: ${thrown.join('; ')}`);
  check(failed.length === 0, `a request the page made failed: ${failed.join('; ')}`);

  if (problems.length) {
    const shot = path.join(here, 'field-as-drawn.png');
    await page.screenshot({ path: shot });
    console.error(`what the browser saw: ${shot}`);
  } else {
    console.log(`${state.said}`);
    console.log(`  ${drawn.toLocaleString()} triangles drawn in a frame, at 1200x800, on SwiftShader`);
  }
} catch (wrong) {
  problems.push(`the page could not be driven: ${wrong && wrong.message}`);
} finally {
  await browser.close();
  server.close();
}

if (problems.length) {
  console.error('the field page did not draw as it should:');
  for (const said of problems) {
    console.error('  ' + said);
  }
  process.exit(1);
}
console.log('the field page draws, in a real browser.');
