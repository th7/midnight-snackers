import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { ASSETS, benchModel, cannedRun, chrome, servingTheLiveView, sim, theLiveView } from './bench.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

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

const model = benchModel();
const canned = cannedRun(model);
const livePage = theLiveView(
    { name: 'FieldCheckAuto', kind: 'auto', live: false, outcome: 'done', ticks: canned.ticks },
    ASSETS,
    'FieldCheckAuto');
const server = await servingTheLiveView(livePage);
const base = `http://127.0.0.1:${server.address().port}`;
const url = `${base}/runs/1/`;

let browser = null;
try {
  browser = await chrome();
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
// A page draws the resolution it is told to, which here is what the committed field-default.js
// says -- high -- and nothing is built here: so the 404 that sends it back down to the committed
// model is the working case. What must hold is that it drew and that it said which model it drew
// -- both checked below, not waved through.
const UNBUILT = /field-(high|medium)\.glb/;
const faults = () => failed.filter((said) => !UNBUILT.test(said));
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
  if (faults().length) {
    return `a request the page made failed: ${faults().join('; ')}`;
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
        () => window.replayPage && window.replayPage.settled,
        null, { timeout: 90_000 });
  } catch (timedOut) {
    throw new Error(whyItNeverReported());
  }

  const state = await page.evaluate(() => ({
    loaded: window.replayPage.loaded,
    problem: window.replayPage.problem,
    said: document.getElementById('field-msg').textContent
  }));

  const fellBack = /has not been built, so this is the low one/.test(state.problem || '');
  check(!state.problem || fellBack, `the page reported a problem: ${state.problem}`);
  check(fellBack, 'nothing here has built the resolution a page asks for, so the page has to say it '
      + `fell back, and it said "${state.problem}"`);
  check(state.loaded !== null, 'the page never loaded the model');
  if (state.loaded) {
    check(state.loaded.parts > 100, `only ${state.loaded.parts} parts reached the page`);
    check(state.loaded.triangles > 100000, `only ${state.loaded.triangles} triangles reached the page`);
  }

  await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  const drawn = await page.evaluate(() => window.replayPage.drawnTriangles);
  check(drawn > 1000, `only ${drawn} triangles were drawn in a frame; the field is loaded but not on screen`);

  const drawn2 = await page.evaluate(() => {
    const out = { walls: 0, seeThrough: [], solidSeeThrough: [], tapeTop: null, floorZ: window.replayPage.floorZ,
                  fieldPlaneZ: window.replayPage.fieldPlaneZ, orbit: window.replayPage.orbit,
                  guessedNormals: window.replayPage.guessedNormals, pieces: window.replayPage.pieces,
                  normalsFromTheModel: 0, fieldMeshes: 0, inScene: 0 };
    window.replayPage.scene.traverse((o) => {
      if (!o.isMesh) {
        return;
      }
      out.inScene++;
      if (o.geometry.attributes.normal) {
        out.normalsFromTheModel++;
      }

      const names = [o.name].concat(o.userData.from || []);
      out.walls += names.filter((name) => /field[\s_]panel|ftc[\s_]rail|side[\s_]glass/i.test(name)).length;
      if (/skin|side[\s_]glass/i.test(o.name)) {
        (o.material.transparent ? out.seeThrough : out.solidSeeThrough).push(o.name);
      }
      if (/tape/i.test(o.name)) {
        o.geometry.computeBoundingBox();

        const box = o.geometry.boundingBox.clone().applyMatrix4(o.matrixWorld);
        out.tapeTop = out.tapeTop === null ? box.max.z : Math.max(out.tapeTop, box.max.z);
      }
    });
    window.replayPage.field.traverse((o) => {
      if (o.isMesh) {
        out.fieldMeshes++;
      }
    });
    return out;
  });

  check(drawn2.walls > 50, `only ${drawn2.walls} wall parts are in the scene; the field has no perimeter`);

  // This is the cheap model -- the stand-in is the normal detail, and a fetched full one is what CI
  // has not got. What can be held here is that a model carrying neither normals nor the CAD's
  // materials still draws: the page computes the one, paints the other, and says it had to. The full
  // model's own normals and its own pieces are held in the Java tests, where the pipeline is, and by
  // eye against a real build. Every mesh still has to end up with a normal from somewhere, or it
  // shades as a silhouette.
  check(drawn2.guessedNormals === true,
      'the stand-in carries the CAD\'s normals now; this check was written for the model that does not, '
      + 'so what it holds has moved and it should be rewritten rather than flipped');
  check(drawn2.normalsFromTheModel === drawn2.inScene,
      `${drawn2.normalsFromTheModel} of ${drawn2.inScene} meshes have a normal; batching must not lose them`);

  // The simulator says where each piece is, so pieces are drawn as objects rather than batched into
  // the field. Out of the cheap model they are spheres, which is what it can afford.
  check(drawn2.pieces && drawn2.pieces.drawn > 0, 'no game piece is drawn at all');
  if (drawn2.pieces && drawn2.pieces.drawn > 0) {
    check(drawn2.pieces.fromTheModel === 0,
        `${drawn2.pieces.fromTheModel} pieces came out of the cheap model, whose pollen is snapped and `
        + 'flat-shaded; a sphere draws that better and cheaper');
  }

  check(drawn2.fieldMeshes * 4 < state.loaded.parts,
      `the field model has ${state.loaded.parts} parts and the scene draws them as ${drawn2.fieldMeshes} `
      + 'meshes; parts that share a material are not being batched, so a frame makes a draw call for each');
  check(state.loaded.triangles === state.loaded.batchedTriangles,
      `the model loaded ${state.loaded.triangles} triangles and the scene holds `
      + `${state.loaded.batchedTriangles}; batching must lose none of them`);

  check(drawn2.seeThrough.length > 0, 'nothing in the scene is see-through');
  check(drawn2.solidSeeThrough.length === 0,
      `drawn solid and should not be: ${drawn2.solidSeeThrough.slice(0, 4).join(', ')}`);

  check(drawn2.tapeTop !== null, 'no tape in the scene to stand clear of the floor');
  if (drawn2.tapeTop !== null) {
    const clear = drawn2.tapeTop - drawn2.fieldPlaneZ;
    check(clear > 0.001,
        `the tape's top is ${clear} in above the field's floor plane at z=${drawn2.fieldPlaneZ}; in the same `
        + 'plane they fight for the same pixels and the tape flickers');
    check(clear < 0.5, `the tape stands ${clear} in off the floor, which is tape that is floating`);
  }

  // The drawn floor is scenery under the field's own plane, and full detail brings the CAD's soft
  // tiles whose top surface *is* that plane. Two opaque surfaces in one plane fight for the same
  // pixels, which is what the floor did: radial slivers, worse the further out the camera. How far
  // below is not taste. A 24-bit depth buffer resolves about z^2 / (near * 2^24) at distance z, so
  // the test is against that quantum at the far end of the orbit, not against a number somebody
  // liked. CI draws the stand-in, which has no tiles and so cannot show the fight -- what it can
  // hold is the rule that prevents it.
  const drop = drawn2.fieldPlaneZ - drawn2.floorZ;
  const reach = drawn2.orbit.maxDistance + drawn2.orbit.fieldIn / 2;
  const quantum = (reach * reach) / (drawn2.orbit.near * 2 ** 24);
  check(drop > 0,
      `the drawn floor is at z=${drawn2.floorZ} and the field's plane at z=${drawn2.fieldPlaneZ}; in the same `
      + 'plane the CAD\'s tiles and the drawn floor fight for the same pixels');
  check(drop > quantum * 2,
      `the drawn floor is ${drop} in below the field's plane, and the depth buffer cannot resolve better than `
      + `${quantum.toFixed(4)} in at ${reach} in out, so it will fight at full zoom`);
  check(drop < 0.5, `the drawn floor is ${drop} in below the field's plane, which is far enough to read as a gap`);

  // The same page, played rather than read. It was a second one at another size, which meant
  // loading the field and building its scene twice over -- the dearest thing this check does, for
  // nothing: the reads above leave the page as they found it, and only these move it off tick 0.
  const played = await (async () => {
    const before = thrown.length;
    try {
      await page.waitForFunction(
          () => window.replayPage && window.replayPage.settled && window.replayPage.run
              && window.replayPage.run.hives, null, { timeout: 90_000 });
      const first = await page.evaluate(() => window.replayPage.run);
      await page.evaluate(() => window.replayPage.goTo(2));
      await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
      const last = await page.evaluate(() => ({
        run: window.replayPage.run,
        launched: (() => {
          const shown = window.replayPage;
          const ball = shown.launchedBall;
          return ball ? { visible: ball.visible, at: ball.position.toArray() } : null;
        })(),
        drawn: window.replayPage.drawnTriangles,
        hiveTurned: (() => {
          let turned = false;
          window.replayPage.scene.traverse((o) => {
            if (!o.isMesh && /blue[\s_-]*hive/i.test(o.name || '')
                && Math.abs(o.matrix.elements[8]) > 0.01) {
              turned = true;
            }
          });
          return turned;
        })()
      }));
      await page.evaluate(() => window.replayPage.goTo(3));
      await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
      const rested = await page.evaluate(() => {
        let turned = false;
        window.replayPage.scene.traverse((o) => {
          if (!o.isMesh && /blue[\s_-]*hive/i.test(o.name || '')
              && Math.abs(o.matrix.elements[8]) > 0.01) {
            turned = true;
          }
        });
        return { turned };
      });
      return { first, last, rested, wrong: thrown.slice(before) };
    } catch (stuck) {
      return { first: null, last: null, wrong: thrown.slice(before).concat(String(stuck && stuck.message)) };
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
      await view.goto(`${base}/runs/1/?view=camera`, { waitUntil: 'load', timeout: 60_000 });
      await view.waitForFunction(
          () => window.replayPage && window.replayPage.settled && window.replayPage.camera3,
          null, { timeout: 90_000 });
      await view.evaluate(() => window.replayPage.tagsReady);
      await view.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));

      seenClearance = await view.evaluate(() => {
        const page = window.replayPage;
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
        const page = window.replayPage;
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
        window.replayPage.scene.traverse((o) => {
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
      placedAgainstGeometry = await view.evaluate(() => window.replayPage.tagPlacedAgainstGeometry);
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
  check(faults().length === 0, `a request the page made failed: ${faults().join('; ')}`);

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
