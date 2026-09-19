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
/**
 * A run for the page to play: the robot driving a few inches and turning, a loose pollen rolling
 * with it, and one hive leaning further than the field was set up at. Small on purpose -- what is
 * being checked is that a tick reaches the scene, not the simulator that made it.
 */
function cannedRun(model) {
  const moved = model.pieces.filter((p) => p.loose)
      .concat(model.pieces.filter((p) => p.cell))
      .concat(model.pieces.filter((p) => p.flower));
  const blue = model.hives.find((h) => h.alliance === 'Blue');
  const places = (n) => moved.map((piece, i) => i === 0 ? [n * 6, n * 3, piece.radius] : piece.centre);
  return {
    outcome: 'done',
    ticks: [0, 1, 2].map((n) => ({
      t: n * 0.25, x: -30 + n * 12, y: -20 + n * 5, heading: n * 0.4, step: 'drive',
      powers: [0, 0, 0, 0], packets: [], pieces: places(n),
      tilt: n === 2 && blue ? { Blue: blue.tilt - 25 } : undefined
    }))
  };
}

function serve() {
  const model = JSON.parse(fs.readFileSync(path.join(sim, 'field.json'), 'utf8'));
  model.robotIn = 18;
  const run = cannedRun(model);
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    if (asked === '/model' || asked === '/runs/1/ticks') {
      response.writeHead(200, { 'Content-Type': 'application/json' });
      response.end(JSON.stringify(asked === '/model' ? model : run));
      return;
    }
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

/**
 * How much of a PNG is not black. Chromium encodes the screenshot, so rather than decode it,
 * the same browser is asked to read it back through a canvas -- which is the one image decoder
 * certainly present and certainly agreeing with what drew it.
 */
async function litPixels(png) {
  const page = await browser.newPage();
  try {
    return await page.evaluate(async (bytes) => {
      const blob = new Blob([new Uint8Array(bytes)], { type: 'image/png' });
      const bitmap = await createImageBitmap(blob);
      const canvas = document.createElement('canvas');
      canvas.width = bitmap.width;
      canvas.height = bitmap.height;
      const context = canvas.getContext('2d');
      context.drawImage(bitmap, 0, 0);
      const data = context.getImageData(0, 0, bitmap.width, bitmap.height).data;
      let lit = 0;
      for (let i = 0; i < data.length; i += 4) {
        if (data[i] > 24 || data[i + 1] > 24 || data[i + 2] > 24) {
          lit++;
        }
      }
      return lit;
    }, Array.from(png));
  } finally {
    await page.close();
  }
}

const problems = [];
const check = (ok, said) => { if (!ok) problems.push(said); };

const server = await serve();
const base = `http://127.0.0.1:${server.address().port}`;
const url = `${base}/field`;

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

  // --- and now the same page with a run to play --------------------------------------------
  const played = await (async () => {
    const replay = await browser.newPage({ viewport: { width: 1000, height: 700 } });
    const wrong = [];
    replay.on('pageerror', (e) => wrong.push(String(e && e.message ? e.message : e)));
    try {
      await replay.goto(`${base}/field?run=1`, { waitUntil: 'load', timeout: 60_000 });
      await replay.waitForFunction(() => window.fieldPage && window.fieldPage.run,
          null, { timeout: 90_000 });
      const first = await replay.evaluate(() => window.fieldPage.run);
      await replay.evaluate(() => window.fieldPage.goTo(2));
      await replay.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
      const last = await replay.evaluate(() => ({
        run: window.fieldPage.run,
        drawn: window.fieldPage.drawnTriangles,
        hiveTurned: (() => {
          let turned = false;
          window.fieldPage.scene.traverse((o) => {
            if (!o.isMesh && /blue[\s_-]*hive/i.test(o.name || '')
                && Math.abs(o.matrix.elements[8]) > 0.01) {
              turned = true;
            }
          });
          return turned;
        })()
      }));
      return { first, last, wrong };
    } catch (stuck) {
      return { first: null, last: null, wrong: wrong.concat(String(stuck && stuck.message)) };
    } finally {
      await replay.close();
    }
  })();

  check(played.wrong.length === 0, `the run page threw: ${played.wrong.join('; ')}`);
  if (played.first && played.last) {
    check(played.first.loops === 3, `the run page read ${played.first.loops} loops of 3`);
    check(played.first.balls > 20, `only ${played.first.balls} game pieces are drawn`);
    check(played.first.hives === 2, `${played.first.hives} hives were found to lean`);
    // The robot has to be where the tick put it, not merely on the field somewhere.
    check(played.first.robot && Math.abs(played.first.robot.x + 30) < 0.01,
        `the robot started at x=${played.first.robot && played.first.robot.x}, not the tick's -30`);
    check(played.last.run.robot && Math.abs(played.last.run.robot.x - -6) < 0.01,
        `the robot ended at x=${played.last.run.robot && played.last.run.robot.x}, not the tick's -6`);
    check(Math.abs(played.last.run.robot.heading - 0.8) < 0.01,
        `the robot ended facing ${played.last.run.robot.heading}, not the tick's 0.8`);
    check(played.last.hiveTurned, 'the blue hive did not lean, though the last tick says it did');
    check(played.last.drawn > 1000, `a run frame drew only ${played.last.drawn} triangles`);
  }

  // --- the camera's view of the tags -------------------------------------------------------
  const camera = await (async () => {
    const view = await browser.newPage({ viewport: { width: 640, height: 480 } });
    const wrong = [];
    view.on('pageerror', (e) => wrong.push(String(e && e.message ? e.message : e)));
    try {
      await view.goto(`${base}/field?run=1&view=camera`, { waitUntil: 'load', timeout: 60_000 });
      await view.waitForFunction(() => window.fieldPage && window.fieldPage.camera3,
          null, { timeout: 90_000 });
      await view.evaluate(() => window.fieldPage.tagsReady);
      await view.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));

      // Where the lens says each tag should land, worked out from the camera matrix rather than
      // from the picture: a tag drawn in the wrong place still looks like a tag.
      const seen = await view.evaluate(() => {
        const page = window.fieldPage;
        const out = { tags: page.tags, lens: page.lens, at: [] };
        page.scene.traverse((o) => {
          if (!/^tag /.test(o.name || '')) {
            return;
          }
          const where = new o.position.constructor();
          o.getWorldPosition(where);
          const ndc = where.clone().project(page.camera3);
          out.at.push({
            name: o.name,
            px: (ndc.x + 1) / 2 * page.lens.width,
            py: (1 - ndc.y) / 2 * page.lens.height,
            inFront: ndc.z > -1 && ndc.z < 1
          });
        });
        return out;
      });
      // Blank but for the tags: anything else left drawing is in every frame that is captured.
      seen.strays = await view.evaluate(() => {
        const out = [];
        window.fieldPage.scene.traverse((o) => {
          if (o.isMesh && o.visible && !o.userData.tag && !/^tag /.test(o.name || '')) {
            let hidden = false;
            for (let up = o.parent; up; up = up.parent) {
              if (!up.visible) {
                hidden = true;
              }
            }
            if (!hidden) {
              out.push(o.name || '(unnamed)');
            }
          }
        });
        return out;
      });
      const shot = await view.screenshot();
      return { seen, shot, wrong };
    } catch (stuck) {
      return { seen: null, shot: null, wrong: wrong.concat(String(stuck && stuck.message)) };
    } finally {
      await view.close();
    }
  })();

  check(camera.wrong.length === 0, `the camera view threw: ${camera.wrong.join('; ')}`);
  if (camera.seen) {
    check(camera.seen.tags.length === 4, `${camera.seen.tags.length} goal tags were made, of 4`);
    const onScreen = camera.seen.at.filter(
        (t) => t.inFront && t.px > 0 && t.px < camera.seen.lens.width
            && t.py > 0 && t.py < camera.seen.lens.height);
    check(camera.seen.strays.length === 0,
        `the camera's view is meant to be blank but for the tags, and ${camera.seen.strays.length} `
        + `other things are drawn in it: ${camera.seen.strays.slice(0, 4).join(', ')}`);
    check(onScreen.length > 0,
        'the lens has no tag in view at all from where the run puts the robot, so nothing '
        + 'about the picture says whether the tags are where they should be');

    // And the picture has to agree: a frame of pure black is what a tag drawn behind the camera,
    // or never textured, or hidden with the field, all look like.
    const lit = await litPixels(camera.shot);
    check(lit > 200, `only ${lit} pixels of the camera's frame are lit; the tags are not drawn`);
    if (onScreen.length) {
      console.log(`  the lens sees ${onScreen.length} of 4 goal tags, `
          + `the nearest at ${onScreen[0].px.toFixed(0)},${onScreen[0].py.toFixed(0)} px of `
          + `${camera.seen.lens.width}x${camera.seen.lens.height}; ${lit} pixels lit`);
    }
  }

  check(thrown.length === 0, `the page threw: ${thrown.join('; ')}`);
  check(failed.length === 0, `a request the page made failed: ${failed.join('; ')}`);

  if (!problems.length) {
    console.log(state.said);
    console.log(`  ${drawn.toLocaleString()} triangles drawn in a frame, at 1200x800, on ${webgl}`);
    if (played.last) {
      console.log(`  a run plays: ${played.first.loops} loops, ${played.first.balls} game pieces, `
          + `${played.first.hives} hives, the robot ending where its last tick puts it`);
    }
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
