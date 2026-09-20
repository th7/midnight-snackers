import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { ASSETS, benchModel, cannedRun, chrome, servingTheLiveView, theLiveView } from './bench.mjs';
import { inSoftware } from '../../TeamCode/src/test/resources/org/firstinspires/ftc/teamcode/sim/framecost.js';

const budgetFile = fileURLToPath(new URL('./scene-budget.json', import.meta.url));
const regenerating = process.argv.includes('--regenerate');
const onAGpu = process.argv.includes('--gpu');
const headed = process.argv.includes('--headed');

const PINNED = ['withShadows.draws', 'withShadows.calls', 'withShadows.triangles',
                'withoutShadows.draws', 'withoutShadows.triangles'];

const wrong = [];
const check = (ok, saying) => { if (!ok) wrong.push(saying); };
const reach = (reading, dotted) => dotted.split('.').reduce((at, key) => at && at[key], reading);

function ms(value) {
  return value === null || value === undefined ? '--' : value.toFixed(3) + ' ms';
}

function say(name, reading) {
  if (!reading.judged) {
    console.log(`  ${name}: could not judge -- ${reading.why}`);
    return;
  }
  console.log(`  ${name}: ${ms(reading.best)} a frame at best, ${ms(reading.median)} median `
      + `(${reading.renders} renders a block); ${ms(reading.bestSubmit)} of that is submitting it`);
  console.log(`    ${reading.draws.toLocaleString()} draws a frame `
      + `(three.js counts ${reading.calls.toLocaleString()} of them, the colour pass), `
      + `${reading.triangles.toLocaleString()} triangles, ${reading.programs} programs, `
      + `${reading.geometries.toLocaleString()} geometries, ${reading.textures} textures`);
}

async function theProbeTimesWhatItSaysItTimes(browser, base) {
  const page = await browser.newPage({ viewport: { width: 400, height: 300 } });
  try {
    // The flat drawing: this holds the probe to what it times, over a renderer and a triangle of
    // its own, so the field scene the page would otherwise build is a few seconds spent on a
    // picture nothing here looks at. ?view=flat serves the same page and the same modules.
    await page.goto(`${base}/runs/1/?view=flat`, { waitUntil: 'load', timeout: 60_000 });
    const measured = await page.evaluate(async () => {
      const THREE = await import('../../assets/vendor/three.module.min.js');
      const { measure } = await import('../../assets/framecost.js');

      const canvas = document.createElement('canvas');
      canvas.width = 64;
      canvas.height = 64;
      const renderer = new THREE.WebGLRenderer({ canvas: canvas });
      const scene = new THREE.Scene();
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(
          new Float32Array([-1, -1, 0, 1, -1, 0, 0, 1, 0]), 3));
      scene.add(new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color: 0xffffff })));
      const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 10);
      camera.position.z = 3;

      const before = renderer.info.render.frame;
      const reading = await measure(renderer, () => renderer.render(scene, camera), { least: 20, blocks: 3 });
      const frames = renderer.info.render.frame - before;

      const refused = await measure(renderer, () => renderer.render(scene, camera),
                                    { least: 1e9, blocks: 3, most: 1 });
      renderer.dispose();
      return { reading, frames, refused };
    });

    const one = measured.reading;
    check(one.judged, `the probe could not time a single triangle: ${one.why}`);
    check(one.renders > 0, `the probe says it timed ${one.renders} renders a block`);
    check(measured.frames >= one.renders * 3,
          `the probe reported ${one.renders} renders over 3 blocks but the renderer drew `
          + `${measured.frames} frames, so it is not timing what it says it is`);
    check(one.best > 0 && one.median > 0, `the probe timed a frame at ${one.best}/${one.median} ms`);
    check(one.best <= one.median + 1e-9, `the probe's best frame (${one.best}) is slower than its median`);
    check(one.calls === 1 && one.triangles === 1,
          `the probe counted ${one.calls} draw calls and ${one.triangles} triangles for one triangle`);
    check(one.draws === 1, `the probe counted ${one.draws} GL draws for a scene drawn in one`);

    check(measured.refused.judged === false,
          'a run too quick to time at this clock reported a number anyway, so a measurement nobody '
          + 'could take cannot be told from one that was');
    check(/\S/.test(measured.refused.why || ''), 'and it did not say why it could not judge');
  } finally {
    await page.close();
  }
}

async function theFieldPageTakesTheReading(browser, base) {
  const page = await browser.newPage({ viewport: { width: 1200, height: 800 } });
  const threw = [];
  page.on('pageerror', (e) => threw.push(String(e && e.message ? e.message : e)));
  try {
    await page.goto(`${base}/runs/1/?cost`, { waitUntil: 'load', timeout: 60_000 });
    await page.waitForFunction(() => window.replayPage && window.replayPage.cost, null, { timeout: 180_000 });
    const reading = await page.evaluate(() => window.replayPage.cost);
    const said = await page.evaluate(() => document.getElementById('field-msg').textContent);

    check(threw.length === 0, `the cost page threw: ${threw.join('; ')}`);
    check(reading.withShadows && reading.withoutShadows,
          'the reading does not say what the frame costs with and without the shadow pass');
    check(/ms/.test(said), `the page shows "${said}", which a reader on a tablet cannot read a cost off`);
    const canvas = await page.evaluate(() => {
      const solid = document.getElementById('solid');
      return { width: solid.width, height: solid.height };
    });
    check(reading.at && reading.at.width === canvas.width && reading.at.height === canvas.height,
          `the reading says ${JSON.stringify(reading.at)} and the view is ${JSON.stringify(canvas)}; a time `
          + 'is a time at a size, so the two must be the same');
    check(/\S/.test(reading.gpu || ''), 'the reading does not say what drew it, so two are not comparable');
    check(reading.withShadows.draws > reading.withShadows.calls,
          `the reading says the frame makes ${reading.withShadows.draws} draws and three.js counts `
          + `${reading.withShadows.calls}; with a shadow pass on they cannot be the same number, so the `
          + 'shadow pass is going uncounted');
    check(reading.withoutShadows.draws < reading.withShadows.draws,
          `turning the shadow pass off left the frame making ${reading.withoutShadows.draws} draws against `
          + `${reading.withShadows.draws} with it, so the reading is not measuring what it says`);
    return reading;
  } finally {
    await page.close();
  }
}

function theSceneIsStillWithinItsBudget(reading) {
  const now = {};
  for (const key of PINNED) {
    now[key] = reach(reading, key);
  }

  if (regenerating) {
    fs.writeFileSync(budgetFile, JSON.stringify(now, null, 2) + '\n');
    wrong.push(`regenerated ${path.relative(process.cwd(), budgetFile)}. A run that wrote the budget has not `
        + 'checked it: read the diff, then re-run without --regenerate to check it.');
    return;
  }

  if (!fs.existsSync(budgetFile)) {
    wrong.push(`no budget at ${path.relative(process.cwd(), budgetFile)}, so nothing is being checked. `
        + 'Write it with: node tools/browser/cost.mjs --regenerate');
    return;
  }

  const was = JSON.parse(fs.readFileSync(budgetFile, 'utf8'));
  for (const key of PINNED) {
    check(was[key] === now[key],
          `${key}: the budget says ${was[key]}, the scene now draws ${now[key]}. `
          + 'These do not vary with the machine, so this is a change in the scene. If it is one you meant, '
          + 'read it and regenerate with: node tools/browser/cost.mjs --regenerate');
  }
}

function softwareIsToldFromHardware() {
  const software = [
    'SwiftShader driver',
    'ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (LLVM 16.0.0) (0x0000C0DE)), SwiftShader driver)',
    'llvmpipe (LLVM 15.0.7, 256 bits)',
    'Mesa/X.org, llvmpipe',
    'Software Rasterizer'
  ];
  const hardware = [
    'Apple M2',
    'ANGLE (Apple, ANGLE Metal Renderer: Apple M1 Pro, Unspecified Version)',
    'ANGLE (NVIDIA, NVIDIA GeForce RTX 3060 Direct3D11 vs_5_0 ps_5_0, D3D11)',
    'Adreno (TM) 650',
    'Mali-G78 MP14',
    'Intel(R) Iris(R) Xe Graphics'
  ];
  for (const name of software) {
    check(inSoftware(name) === true, `"${name}" is software rendering and was not read as it`);
  }
  for (const name of hardware) {
    check(inSoftware(name) === false, `"${name}" is a GPU and was read as software rendering`);
  }
}

async function main() {
  softwareIsToldFromHardware();

  const model = benchModel();
  const run = cannedRun(model);
  const page = theLiveView(
      { name: 'CostCheckAuto', kind: 'auto', live: false, outcome: 'done', ticks: run.ticks },
      ASSETS,
      'CostCheckAuto');
  const server = await servingTheLiveView(page);
  const base = `http://127.0.0.1:${server.address().port}`;
  const browser = await chrome({ gpu: onAGpu, headed: headed });
  let reading = null;
  try {
    await theProbeTimesWhatItSaysItTimes(browser, base);
    reading = await theFieldPageTakesTheReading(browser, base);
  } catch (stuck) {
    wrong.push(String(stuck && stuck.message));
  } finally {
    await browser.close();
    server.close();
  }

  if (reading) {
    const software = inSoftware(reading.gpu);
    console.log(`the field scene costs, at ${reading.at.width}x${reading.at.height} on ${reading.gpu}`
        + `${software ? ' (software, so the times are this machine\'s and not a GPU\'s)' : ''}:`);
    say('with shadows', reading.withShadows);
    say('without shadows', reading.withoutShadows);

    if (onAGpu && software) {
      wrong.push(`--gpu asked for a hardware reading and the browser drew it in software (${reading.gpu}). `
          + 'Times from a software rasteriser are not a GPU\'s and must not be read as one. There may be no '
          + 'GPU here -- a container without /dev/dri and a CI runner both lack one -- or headless Chromium '
          + 'may not be reaching it, which `--gpu --headed` is worth trying against.');
    }
    if (!onAGpu && !software) {
      wrong.push(`without --gpu the browser is asked for SwiftShader and drew with ${reading.gpu} instead. `
          + 'The correctness checks need software rendering to come out the same on two machines; see '
          + 'doc/browser-tests.md.');
    }

    theSceneIsStillWithinItsBudget(reading);
  } else {
    wrong.push('no reading was taken, so nothing is known about what a frame costs');
  }

  if (wrong.length) {
    console.error('the frame cost could not be measured as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
}

await main();
