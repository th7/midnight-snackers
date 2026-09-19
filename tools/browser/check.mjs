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
    // In the image, apt's chromium, which CHROME_BIN names. Everywhere else, Playwright's own
    // full browser rather than its default: `chrome-headless-shell` is the faster build and it
    // is the one without the graphics stack WebGL needs, so a page that draws would not.
    ...(process.env.CHROME_BIN
        ? { executablePath: process.env.CHROME_BIN }
        : { channel: 'chromium' }),
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

/** The page sets window.fieldPage last, so anything it threw on the way is why it never did. */
function whyItNeverReported() {
  if (thrown.length) {
    return `the page threw before it could report: ${thrown.join('; ')}`;
  }
  if (failed.length) {
    return `a request the page made failed: ${failed.join('; ')}`;
  }
  return 'the page never reported itself loaded, and threw nothing to say why';
}

try {
  await page.goto(url, { waitUntil: 'load', timeout: 60_000 });

  // Before anything about the field: can this browser do WebGL at all? Without it three.js
  // throws when it makes its renderer, the page never reaches the line that reports itself,
  // and every check below would fail as an unexplained timeout.
  const webgl = await page.evaluate(() => {
    const canvas = document.createElement('canvas');
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
    if (!gl) {
      return null;
    }
    const named = gl.getExtension('WEBGL_debug_renderer_info');
    return named ? gl.getParameter(named.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
  });
  if (!webgl) {
    throw new Error('this browser has no WebGL, so the page cannot draw whatever else is right. '
        + 'SwiftShader is what supplies it here; see doc/browser-tests.md for the flags.');
  }

  // The model is three megabytes and SwiftShader is not quick; wait for the page to say it is
  // loaded rather than for a time that would be a guess on one machine and wrong on another.
  try {
    await page.waitForFunction(
        () => window.fieldPage && (window.fieldPage.loaded || window.fieldPage.problem),
        null, { timeout: 90_000 });
  } catch (timedOut) {
    throw new Error(whyItNeverReported());
  }

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

  // The three things that were wrong the first time anyone looked at this page.
  // three.js renames nodes as it loads them: spaces become underscores and repeats get numbers,
  // so these patterns allow for the separator. A pattern that assumed the CAD's spelling would
  // match nothing and every check below would pass by finding nothing to disagree with.
  const drawn2 = await page.evaluate(() => {
    const out = { walls: 0, seeThrough: [], solidSeeThrough: [], tapeTop: null, floorZ: window.fieldPage.floorZ };
    window.fieldPage.scene.traverse((o) => {
      if (!o.isMesh) {
        return;
      }
      if (/field[\s_]panel|ftc[\s_]rail|side[\s_]glass/i.test(o.name)) {
        out.walls++;
      }
      if (/skin|side[\s_]glass/i.test(o.name)) {
        (o.material.transparent ? out.seeThrough : out.solidSeeThrough).push(o.name);
      }
      if (/tape/i.test(o.name)) {
        o.geometry.computeBoundingBox();
        const top = o.geometry.boundingBox.max.z;
        out.tapeTop = out.tapeTop === null ? top : Math.max(out.tapeTop, top);
      }
    });
    return out;
  });

  // The field was a floor with things standing on it: the perimeter is dropped by the collision
  // model, which models the walls itself, and the visual model has to keep it.
  check(drawn2.walls > 50, `only ${drawn2.walls} wall parts are in the scene; the field has no perimeter`);

  // Drawn solid, a hive's skins close the basket over what was scored in it, and the perimeter's
  // side glass is what the whole field is watched through.
  check(drawn2.seeThrough.length > 0, 'nothing in the scene is see-through');
  check(drawn2.solidSeeThrough.length === 0,
      `drawn solid and should not be: ${drawn2.solidSeeThrough.slice(0, 4).join(', ')}`);

  // The tape's top face is at z = 0. A floor drawn there too leaves the two in one plane, and
  // which one a pixel belongs to is decided by rounding -- which is the tape flickering.
  check(drawn2.tapeTop !== null, 'no tape in the scene to stand clear of the floor');
  if (drawn2.tapeTop !== null) {
    check(drawn2.floorZ < drawn2.tapeTop - 0.1,
        `the floor is drawn at z=${drawn2.floorZ} and the tape's top is at z=${drawn2.tapeTop}; `
        + 'that close they fight for the same pixels and the tape flickers');
  }

  check(thrown.length === 0, `the page threw: ${thrown.join('; ')}`);
  check(failed.length === 0, `a request the page made failed: ${failed.join('; ')}`);

  if (!problems.length) {
    console.log(state.said);
    console.log(`  ${drawn.toLocaleString()} triangles drawn in a frame, at 1200x800, on ${webgl}`);
  }
} catch (wrong) {
  problems.push(String(wrong && wrong.message ? wrong.message : wrong));
} finally {
  if (problems.length) {
    // Whatever went wrong, the first question is what the browser actually had on screen.
    const shot = path.join(here, 'field-as-drawn.png');
    await page.screenshot({ path: shot }).catch(() => {});
    console.error(`what the browser saw: ${shot}`);
  }
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
