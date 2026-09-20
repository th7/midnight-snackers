
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { movedPieces } from './model.mjs';

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

function cannedRun(model) {
  const moved = model.moved;
  const blue = model.hives.find((h) => h.alliance === 'Blue');

  const places = (n) => moved.map((piece, i) => i === 0 ? [n * 6, n * 3, piece.radius] : piece.centre)
      .concat([n < 2 ? null : [18, -9, 26]]);
  return {
    outcome: 'done',
    ticks: [0, 1, 2, 3].map((n) => ({
      t: n * 0.25, x: -30 + n * 12, y: -20 + n * 5, heading: n * 0.4, step: 'drive',
      powers: [0, 0, 0, 0], packets: [], pieces: places(n),
      tilt: n === 2 && blue ? { Blue: blue.tilt - 25 } : undefined
    }))
  };
}


function serve() {
  const model = JSON.parse(fs.readFileSync(path.join(sim, 'field.json'), 'utf8'));
  model.robotIn = 18;
  model.moved = movedPieces(model);
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
    ...(process.env.CHROME_BIN
        ? { executablePath: process.env.CHROME_BIN }
        : { channel: 'chromium' }),
    args: [

      '--no-sandbox',

      '--disable-dev-shm-usage',

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

  await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  const drawn = await page.evaluate(() => window.fieldPage.drawnTriangles);
  check(drawn > 1000, `only ${drawn} triangles were drawn in a frame; the field is loaded but not on screen`);

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

        const box = o.geometry.boundingBox.clone().applyMatrix4(o.matrixWorld);
        out.tapeTop = out.tapeTop === null ? box.max.z : Math.max(out.tapeTop, box.max.z);
      }
    });
    return out;
  });

  check(drawn2.walls > 50, `only ${drawn2.walls} wall parts are in the scene; the field has no perimeter`);

  check(drawn2.seeThrough.length > 0, 'nothing in the scene is see-through');
  check(drawn2.solidSeeThrough.length === 0,
      `drawn solid and should not be: ${drawn2.solidSeeThrough.slice(0, 4).join(', ')}`);

  check(drawn2.tapeTop !== null, 'no tape in the scene to stand clear of the floor');
  if (drawn2.tapeTop !== null) {
    const clear = drawn2.tapeTop - drawn2.floorZ;
    check(clear > 0.001,
        `the tape's top is ${clear} in above the floor at z=${drawn2.floorZ}; in the same plane `
        + 'they fight for the same pixels and the tape flickers');
    check(clear < 0.5, `the tape stands ${clear} in off the floor, which is tape that is floating`);
  }

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
        launched: (() => {
          const page = window.fieldPage;
          const ball = page.launchedBall;
          return ball ? { visible: ball.visible, at: ball.position.toArray() } : null;
        })(),
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
      await replay.evaluate(() => window.fieldPage.goTo(3));
      await replay.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
      const rested = await replay.evaluate(() => {
        let turned = false;
        window.fieldPage.scene.traverse((o) => {
          if (!o.isMesh && /blue[\s_-]*hive/i.test(o.name || '')
              && Math.abs(o.matrix.elements[8]) > 0.01) {
            turned = true;
          }
        });
        return { turned };
      });
      return { first, last, rested, wrong };
    } catch (stuck) {
      return { first: null, last: null, wrong: wrong.concat(String(stuck && stuck.message)) };
    } finally {
      await replay.close();
    }
  })();

  check(played.wrong.length === 0, `the run page threw: ${played.wrong.join('; ')}`);
  if (played.first && played.last) {
    check(played.first.loops === 4, `the run page read ${played.first.loops} loops of 4`);
    check(played.first.balls > 20, `only ${played.first.balls} game pieces are drawn`);
    check(played.first.hives === 2, `${played.first.hives} hives were found to lean`);

    check(played.first.robot && Math.abs(played.first.robot.x + 30) < 0.01,
        `the robot started at x=${played.first.robot && played.first.robot.x}, not the tick's -30`);
    check(played.last.run.robot && Math.abs(played.last.run.robot.x - -6) < 0.01,
        `the robot ended at x=${played.last.run.robot && played.last.run.robot.x}, not the tick's -6`);
    check(Math.abs(played.last.run.robot.heading - 0.8) < 0.01,
        `the robot ended facing ${played.last.run.robot.heading}, not the tick's 0.8`);
    check(played.last.hiveTurned, 'the blue hive did not lean, though its tick says it did');
    check(played.rested && !played.rested.turned,
        'the blue hive is still drawn leaning a tick after it tipped back, so the page is reading '
        + 'the last tick that named it rather than the tick it is drawing');

    check(played.first.run ? played.first.run.launchable > 0 : played.first.launchable > 0,
        'no ball is drawn for what the robot brought with it, so a launched pollen would vanish');
    check(played.last.launched && played.last.launched.visible,
        'the launched pollen is not drawn where the tick puts it');
    if (played.last.launched && played.last.launched.at) {
      check(Math.abs(played.last.launched.at[2] - 26) < 0.01,
          `the launched pollen is at z=${played.last.launched.at[2]}, not the tick's 26`);
    }
    check(played.last.drawn > 1000, `a run frame drew only ${played.last.drawn} triangles`);
  }

  let seenClearance = [];
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

      seenClearance = await view.evaluate(() => {
        const page = window.fieldPage;
        const out = [];
        page.scene.traverse((quad) => {
          if (!/^tag /.test(quad.name || '')) {
            return;
          }
          const side = /scoring/i.test(quad.name) ? /scoring/i : /audience/i;
          const alliance = /blue/i.test(quad.name) ? /blue/i : /red/i;
          let plate = null;
          page.scene.traverse((o) => {
            if (o.isMesh && /april/i.test(o.name || '') && side.test(o.name) && alliance.test(o.name)) {
              plate = o;
            }
          });
          if (!plate) {
            out.push(null);
            return;
          }

          const facing = new quad.position.constructor();
          quad.getWorldDirection(facing);
          const point = new quad.position.constructor();
          const points = plate.geometry.attributes.position;
          let nearest = -Infinity;
          for (let i = 0; i < points.count; i++) {
            point.fromBufferAttribute(points, i).applyMatrix4(plate.matrixWorld).sub(quad.position);
            nearest = Math.max(nearest, point.dot(facing));
          }
          out.push(-nearest);
        });
        return out;
      });
      let placedAgainstGeometry = null;
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

      await view.addStyleTag({ content: '#hud, #run { display: none !important; }' });
      placedAgainstGeometry = await view.evaluate(() => window.fieldPage.tagPlacedAgainstGeometry);
      const shot = await view.screenshot();
      return { seen, shot, wrong, placedAgainstGeometry };
    } catch (stuck) {
      return { seen: null, shot: null, placedAgainstGeometry: null,
               wrong: wrong.concat(String(stuck && stuck.message)) };
    } finally {
      await view.close();
    }
  })();

  check(camera.wrong.length === 0, `the camera view threw: ${camera.wrong.join('; ')}`);
  if (camera.seen) {
    check(camera.seen.tags.length === 4, `${camera.seen.tags.length} goal tags were made, of 4`);
    check(seenClearance.length === 4,
        `${seenClearance.length} of 4 tags were measured against the plate they are printed on`);
    const placed = await (async () => camera.placedAgainstGeometry)();
    check(placed && placed.every(Boolean),
        'a tag was placed against field.json\'s hull of its plate rather than the plate drawn');
    for (const clear of seenClearance) {
      check(clear !== null && clear > 0.001,
          `a tag's artwork stands ${clear} in clear of its plate, so part of the plate is in `
          + 'front of it and the two fight for the same pixels');
      check(clear === null || clear < 0.5,
          `a tag's artwork stands ${clear} in off its plate, which is not printed on it`);
    }
    const onScreen = camera.seen.at.filter(
        (t) => t.inFront && t.px > 0 && t.px < camera.seen.lens.width
            && t.py > 0 && t.py < camera.seen.lens.height);
    check(camera.seen.strays.length === 0,
        `the camera's view is meant to be blank but for the tags, and ${camera.seen.strays.length} `
        + `other things are drawn in it: ${camera.seen.strays.slice(0, 4).join(', ')}`);
    check(onScreen.length > 0,
        'the lens has no tag in view at all from where the run puts the robot, so nothing '
        + 'about the picture says whether the tags are where they should be');

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
