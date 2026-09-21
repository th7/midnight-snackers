import fs from 'node:fs';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chrome } from './bench.mjs';
import { asTheBenchServesIt } from './model.mjs';

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

const FIELD_IN = 141.17;
const ROBOT_IN = 18;
const WALL_IN = 12.2;
const NAME = 'ReplayCheckAuto';
const ASSETS = '../../assets/';

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

function aRunToPlay(model) {
  const moved = model.moved;
  const blue = model.hives.find((h) => h.alliance === 'Blue');
  const places = (n) => moved.map((piece, i) => i === 0 ? [n * 6, n * 3, piece.radius] : piece.centre);
  const overlay = [{ ops: [
    { type: 'STROKE', color: '#4CAF50' },
    { type: 'POLYLINE', xPoints: [0, 24, 36], yPoints: [0, 24, -12] }
  ] }];
  return {
    name: NAME,
    kind: 'auto',
    live: false,
    outcome: 'done',
    ticks: [0, 1, 2].map((n) => ({
      t: n * 0.25,
      x: -30 + n * 12,
      y: -20 + n * 5,
      heading: n * 0.4,
      step: `${n}. drive`,
      powers: [1, 1, 1, 1],
      packets: n === 0 ? [] : overlay,
      pieces: places(n),
      tilt: n === 1 && blue ? { Blue: blue.tilt - 25 } : undefined
    }))
  };
}

function importMapFor(assets) {
  return '<script type="importmap">\n{"imports": {"three": "' + assets + 'vendor/three.module.min.js", '
      + '"three/addons/": "' + assets + 'vendor/jsm/"}}\n</script>';
}

function filled(template, model, run, assets) {
  return template
      .replace('__IMPORTMAP__', assets === null ? '' : importMapFor(assets))
      .replace('__ASSETS__', JSON.stringify(assets))
      .replace('__TITLE__', NAME)
      .replace('__FIELD_IN__', String(FIELD_IN))
      .replace('__ROBOT_IN__', String(ROBOT_IN))
      .replace('__WALL_IN__', String(WALL_IN))
      .replace('__FIELD__', JSON.stringify(model))
      .replace('__DATA__', JSON.stringify(run));
}

function served(page, live, ticks) {
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    if (asked === '/live/runs/1/ticks') {
      const from = Number(new URL(request.url, 'http://x').searchParams.get('from') || 0);
      response.writeHead(200, { 'Content-Type': 'application/json' });
      response.end(JSON.stringify({ outcome: from >= ticks.length ? 'done' : null, ticks: ticks.slice(from) }));
      return;
    }
    if (asked === '/live/runs/1/') {
      response.writeHead(200, { 'Content-Type': TYPES['.html'] });
      response.end(live);
      return;
    }
    if (asked === '/favicon.ico') {
      response.writeHead(204).end();
      return;
    }
    const withoutModel = asked.startsWith('/nomodel/');
    const relative = withoutModel ? asked.slice('/nomodel'.length)
        : asked.startsWith('/live/') ? asked.slice('/live'.length) : asked;
    if (relative === '/runs/1/') {
      response.writeHead(200, { 'Content-Type': TYPES['.html'] });
      response.end(page);
      return;
    }
    const asset = relative.startsWith('/assets/') ? relative.slice('/assets/'.length) : null;
    const file = asset && path.resolve(sim, asset);
    if (!file || !file.startsWith(path.resolve(sim)) || !fs.existsSync(file)
        || (withoutModel && asset === 'field.glb')) {
      response.writeHead(404).end('no');
      return;
    }
    response.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
    response.end(fs.readFileSync(file));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

// A page asks for the resolution it draws at -- high, unless told otherwise -- and nothing here has
// built one, so the 404 that sends it back down to the committed model is the working case rather
// than a fault. What must not be quiet is the page's own account of it, which is checked separately.
const UNBUILT = /field-(high|medium)\.glb/;
// The console says only "Failed to load resource", without naming it; the response that goes with it
// does name it. So a bare 404 in the console is forgiven only when a model 404 was actually seen.
const BARE_404 = /Failed to load resource.*404/;

function faults(threw) {
  const unbuilt = threw.some((said) => UNBUILT.test(said) && said.includes('404'));
  return threw.filter((said) => {
    if (UNBUILT.test(said) && said.includes('404')) {
      return false;
    }
    return !(unbuilt && BARE_404.test(said));
  });
}

async function opened(browser, url) {
  const open = await browser.newPage({ viewport: { width: 1100, height: 900 } });
  const threw = [];
  open.on('pageerror', (e) => threw.push(String(e && e.message ? e.message : e)));
  open.on('response', (r) => { if (r.status() >= 400) threw.push(`${r.url()} answered ${r.status()}`); });
  open.on('console', (m) => {
    if (m.type() === 'error') {
      threw.push(`the page logged an error: ${m.text()}`);
    }
  });
  await open.goto(url, { waitUntil: 'load', timeout: 60_000 });
  await open.waitForFunction(() => window.replayPage && window.replayPage.settled, null, { timeout: 90_000 });
  return { open, threw };
}

const readPanels = (open) => open.evaluate(() => ({
  title: document.getElementById('title').textContent,
  outcome: document.getElementById('outcome').textContent,
  step: document.getElementById('step').textContent,
  pose: document.getElementById('pose').textContent,
  ticks: Number(document.getElementById('scrub').max),
  drawing: window.replayPage.drawing,
  problem: window.replayPage.problem
}));

const twoFrames = (open) => open.evaluate(
    () => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));

const scrubTo = (open, to) => open.evaluate((i) => {
  const scrub = document.getElementById('scrub');
  scrub.value = String(i);
  scrub.dispatchEvent(new Event('input', { bubbles: true }));
}, to);

async function litPixels(browser, png) {
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

async function theLiveViewDrawsTheFieldModel(browser, base, run) {
  const { open, threw } = await opened(browser, `${base}/runs/1/`);
  try {
    const panels = await readPanels(open);
    check(faults(threw).length === 0, `the live view threw: ${faults(threw).join('; ')}`);
    check(panels.drawing === 'solid',
          `the live view drew the ${panels.drawing} field, not the model: ${panels.problem}`);
    check(panels.title.includes(NAME), `the page is titled "${panels.title}", not the run's name`);
    check(/done/i.test(panels.outcome), `the page says the run ended "${panels.outcome}", not done`);
    check(panels.ticks === run.ticks.length - 1,
          `the scrub covers ${panels.ticks + 1} ticks of ${run.ticks.length}`);
    check(panels.step === '0. drive', `the first tick's step reads "${panels.step}"`);

    await twoFrames(open);
    const first = await open.evaluate(() => ({
      drawn: window.replayPage.drawnTriangles,
      parts: window.replayPage.loaded && window.replayPage.loaded.parts,
      robot: window.replayPage.robot,
      balls: window.replayPage.balls,
      hives: window.replayPage.hives
    }));
    check(first.parts > 100, `only ${first.parts} parts of the field model reached the live view`);
    check(first.drawn > 1000,
          `only ${first.drawn} triangles were drawn in a frame; the model loaded but is not on screen`);
    check(first.balls > 20, `only ${first.balls} game pieces are drawn`);
    check(first.hives === 2, `${first.hives} hives were found to lean`);
    check(first.robot && Math.abs(first.robot.x + 30) < 0.01,
          `the robot starts at x=${first.robot && first.robot.x}, not the tick's -30`);

    await scrubTo(open, run.ticks.length - 1);
    await twoFrames(open);
    const last = await readPanels(open);
    const at = run.ticks[run.ticks.length - 1];
    check(last.step === '2. drive', `after scrubbing to the end the step reads "${last.step}"`);
    const reads = `x ${at.x.toFixed(1)} in, y ${at.y.toFixed(1)} in, `
        + `${(at.heading * 180 / Math.PI).toFixed(1)}°`;
    check(last.pose === reads, `the page shows the pose "${last.pose}", not the last tick's "${reads}"`);

    const ended = await open.evaluate(() => ({
      robot: window.replayPage.robot,
      trail: window.replayPage.trailPoints,
      overlay: window.replayPage.overlayLines,
      leaning: window.replayPage.leaning
    }));
    check(ended.robot && Math.abs(ended.robot.x - at.x) < 0.01,
          `the robot in the scene is at x=${ended.robot && ended.robot.x}, not the tick's ${at.x}`);
    check(Math.abs(ended.robot.heading - at.heading) < 0.01,
          `the robot in the scene faces ${ended.robot.heading}, not the tick's ${at.heading}`);
    check(ended.trail === run.ticks.length,
          `the robot's trail is ${ended.trail} points of ${run.ticks.length}; the live view drew no path`);
    check(ended.overlay > 0, "the dashboard's field overlay is not drawn, so the target and the plan are lost");

    await scrubTo(open, 1);
    await twoFrames(open);
    const tipped = await open.evaluate(() => window.replayPage.leaning);
    check(Math.abs(tipped.Blue - ended.leaning.Blue) > 0.01,
          `the blue hive leans ${tipped.Blue} on the tick that tips it and ${ended.leaning.Blue} at rest, `
          + 'so the live view is not leaning it');
    check(Math.abs(ended.leaning.Blue) < 0.01,
          `the blue hive is still leaning at ${ended.leaning.Blue} a tick after it tipped back`);
    return true;
  } finally {
    await open.close();
  }
}

async function aRunStillAddingTicksIsFollowedInTheModel(browser, base, ticks) {
  const { open, threw } = await opened(browser, `${base}/live/runs/1/`);
  try {
    await open.waitForFunction(
        (many) => Number(document.getElementById('scrub').max) === many - 1, ticks.length, { timeout: 30_000 });
    await twoFrames(open);
    const panels = await readPanels(open);
    const followed = await open.evaluate(() => window.replayPage.robot);
    const last = ticks[ticks.length - 1];
    check(faults(threw).length === 0, `the live run threw: ${faults(threw).join('; ')}`);
    check(panels.drawing === 'solid', `a run still running drew the ${panels.drawing} field: ${panels.problem}`);
    check(followed && Math.abs(followed.x - last.x) < 0.01,
          `the live view followed to x=${followed && followed.x}, not the newest tick's ${last.x}; `
          + 'ticks that arrive after the model loaded do not reach the scene');
    check(/running|done/.test(panels.outcome), `a run being followed says "${panels.outcome}"`);
  } finally {
    await open.close();
  }
}

// High is what a page asks for when it asks for nothing, and CI has fetched nothing -- so this is
// also the check that the default is high, and that a server which has not built it still draws.
async function theResolutionNobodyBuiltFallsBackAndSaysSo(browser, base, asked, file) {
  const { open, threw } = await opened(browser, `${base}/runs/1/${asked}`);
  try {
    const panels = await readPanels(open);
    const whole = new RegExp(`${file}|404`);
    const unexpected = threw.filter((said) => !whole.test(said));
    check(unexpected.length === 0, `${asked || 'the default'} threw: ${unexpected.join('; ')}`);
    check(threw.some((said) => said.includes(file)),
          `the page never asked for ${file}, so falling back proves nothing`);
    check(panels.drawing === 'solid',
          `a page asked for a resolution nobody built drew the ${panels.drawing} field instead of falling back`);
    check(/has not been built, so this is the low one/.test(panels.problem),
          `the page fell back and said "${panels.problem}", which does not say it fell back`);
  } finally {
    await open.close();
  }
}

async function theFlatDrawingIsStillThereToAskFor(browser, base) {
  const { open, threw } = await opened(browser, `${base}/runs/1/?view=flat`);
  try {
    const panels = await readPanels(open);
    check(threw.length === 0, `the flat view threw: ${threw.join('; ')}`);
    check(panels.drawing === 'flat', `?view=flat drew the ${panels.drawing} field`);
    await twoFrames(open);
    const drawn = await litPixels(browser, await open.locator('#field').screenshot());
    check(drawn > 2000, `only ${drawn} pixels of the flat field are drawn`);
  } finally {
    await open.close();
  }
}

async function aModelItCannotFetchFallsBackAndSaysSo(browser, base) {
  const { open } = await opened(browser, `${base}/nomodel/runs/1/`);
  try {
    const panels = await readPanels(open);
    check(panels.drawing === 'flat',
          `with no field model to fetch the page still claims to draw the ${panels.drawing} field`);
    check(/model|field/i.test(panels.problem),
          `the page fell back to the flat drawing and said "${panels.problem}", which does not say why`);
    await twoFrames(open);
    const drawn = await litPixels(browser, await open.locator('#field').screenshot());
    check(drawn > 2000, `the page fell back and then drew only ${drawn} pixels`);
  } finally {
    await open.close();
  }
}

async function theWrittenPageIsSelfContained(browser, page) {
  const where = path.join(fs.mkdtempSync(path.join(os.tmpdir(), 'replay-')), 'replay.html');
  fs.writeFileSync(where, page);
  const open = await browser.newPage({ viewport: { width: 1100, height: 900 } });
  const asked = [];
  const threw = [];
  open.on('pageerror', (e) => threw.push(String(e && e.message ? e.message : e)));
  open.on('request', (r) => asked.push(r.url()));
  try {
    await open.goto('file://' + where);
    await open.waitForFunction(() => window.replayPage && window.replayPage.settled, null, { timeout: 30_000 });
    const panels = await readPanels(open);
    check(threw.length === 0, `the written page threw: ${threw.join('; ')}`);
    check(panels.drawing === 'flat', `the written page drew the ${panels.drawing} field, which it cannot fetch`);
    check(panels.title.includes(NAME), `the written page is titled "${panels.title}"`);
    check(asked.filter((u) => !u.startsWith('file://')).length === 0,
          `the written page fetched ${asked.filter((u) => !u.startsWith('file://')).join(', ')}`);
    await twoFrames(open);
    const drawn = await litPixels(browser, await open.locator('#field').screenshot());
    check(drawn > 2000, `only ${drawn} pixels of the written page's field are drawn`);
  } finally {
    await open.close();
  }
}

async function main() {
  const template = fs.readFileSync(path.join(sim, 'replay.html'), 'utf8');
  const model = asTheBenchServesIt(JSON.parse(fs.readFileSync(path.join(sim, 'field.json'), 'utf8')), ROBOT_IN);
  const run = aRunToPlay(model);
  const page = filled(template, model, run, ASSETS);

  for (const which of [page, filled(template, model, run, null)]) {
    const left = which.match(/__[A-Z_]+__/g);
    check(left === null, `the page still holds ${left && [...new Set(left)].join(', ')}, so it was never filled in`);
  }

  const following = { name: NAME, kind: 'auto', live: true, outcome: null, ticks: [] };
  const server = await served(page, filled(template, model, following, ASSETS), run.ticks);
  const base = `http://127.0.0.1:${server.address().port}`;
  const browser = await chrome();

  let solid = false;
  try {
    solid = await theLiveViewDrawsTheFieldModel(browser, base, run);
    await aRunStillAddingTicksIsFollowedInTheModel(browser, base, run.ticks);
    await theResolutionNobodyBuiltFallsBackAndSaysSo(browser, base, '', 'field-high.glb');
    await theResolutionNobodyBuiltFallsBackAndSaysSo(browser, base, '?resolution=medium', 'field-medium.glb');
    await theFlatDrawingIsStillThereToAskFor(browser, base);
    await aModelItCannotFetchFallsBackAndSaysSo(browser, base);
    await theWrittenPageIsSelfContained(browser, filled(template, model, run, null));
  } catch (stuck) {
    wrong.push(String(stuck && stuck.message));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the replay page did not play as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log(`the live view plays ${run.ticks.length} ticks of the field model and follows a run still `
      + 'adding them, and keeps the flat drawing for ?view=flat, for a model it cannot fetch, and for the '
      + 'page it writes to a file.');
  if (!solid) {
    console.error('the live view never reported itself drawn');
    process.exit(1);
  }
}

await main();
