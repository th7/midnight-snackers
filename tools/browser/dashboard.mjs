import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { chrome, sim } from './bench.mjs';

const FILES = [
  { path: 'TeamCode/src/main/java/org/firstinspires/ftc/reginald/Plans.java', editors: [] },
  { path: 'TeamCode/src/main/java/org/firstinspires/ftc/reginald/Drive.java', editors: ['ana'] }
];
const CONTENT = 'package org.firstinspires.ftc.reginald;\n\npublic final class Plans {\n}\n';
const OPMODE = { name: 'BlueLeftAuto', kind: 'auto', group: 'Autonomous',
                 where: 'org.firstinspires.ftc.reginald.BlueLeftAuto', seed: null };

const ANSWERS = {
  '/me': { state: 'approved', username: 'mia', robot: 'nugget', robots: ['reginald', 'nugget'], branch: 'nugget/mia' },
  '/files': { files: FILES },
  '/git/status': { branch: 'nugget/mia', develop: 'nugget-develop', changed: [], behind: 0, ahead: 0, pushable: false,
                   head: '0123456' },
  '/build': { available: true, ok: true, problems: [] },
  '/sim/catalog': [OPMODE],
  '/sim/status': { running: false, runs: [] }
};

const RUNS_ASKED = [];

const SAVES = [];

const BRANCHES = { reginald: 'coding/mia', nugget: 'nugget/mia' };

function onto(robot, robots) {
  ANSWERS['/me'] = { ...ANSWERS['/me'], robot, robots, branch: BRANCHES[robot] };
}

const LAUNCH_GROUPS = [
  { name: 'launcher', label: 'Launcher', open: true,
    says: 'Where the launcher stands, which way it points and how fast its two wheels spin.' },
  { name: 'pollen', label: 'Pollen and pieces', open: true,
    says: 'The ball that is launched, and the pieces in play where a match starts.' },
  { name: 'field', label: 'Field', open: true, says: 'How the hives stand.' },
  { name: 'air', label: 'Air', open: false, says: 'The air the pollen flies through.' },
  { name: 'flight', label: 'Flight model', open: false, says: 'How the air pushes on a spinning ball.' },
  { name: 'bounce', label: 'Bounce', open: false, says: 'How a pollen bounces off each surface of the field.' },
  { name: 'contact', label: 'Contact', open: false, says: 'What every bounce and roll has in common.' },
  { name: 'simulation', label: 'Simulation', open: false, says: 'How finely the shot is computed, and for how long.' }
];

const SURFACES = [
  { name: 'floor', label: 'Floor', says: 'The foam tiles', bounce: 0.55, friction: 0.5, grip: 0.2 },
  { name: 'wall', label: 'Wall', says: 'The polycarbonate perimeter', bounce: 0.5, friction: 0.3, grip: 0.15 },
  { name: 'frame', label: 'Frame', says: 'The metal of the A-frame, the hive tubes and brackets', bounce: 0.62,
    friction: 0.35, grip: 0.15 },
  { name: 'flower', label: 'Flower', says: 'The flowers\' pipes and plates', bounce: 0.55, friction: 0.35, grip: 0.15 },
  { name: 'cell', label: 'Cell', says: 'The hive cells\' walls and backs', bounce: 0.5, friction: 0.3, grip: 0.15 },
  { name: 'piece', label: 'Piece', says: 'Another ball', bounce: 0.55, friction: 0.3, grip: 0.15 }
];

const PROPERTIES = [
  { name: 'bounce', label: 'bounce', most: 1, says: 'How much of its speed into the surface a pollen keeps at 244 in/s.' },
  { name: 'friction', label: 'friction', most: 1.5, says: 'How hard the surface pulls against a sliding pollen.' },
  { name: 'grip', label: 'grip', most: 1, says: 'How much of its slip a gripping pollen sends back the other way.' }
];

function setting(group, name, label, unit, byDefault, least, most, says, extra = {}) {
  return { name, group, label, unit, says, least, most, byDefault, switch: false, value: byDefault, ...extra };
}

function aSwitch(group, name, label, byDefault, says) {
  return setting(group, name, label, '', byDefault, 0, 1, says, { switch: true });
}

const LAUNCH_SETTINGS = [
  setting('launcher', 'x', 'X', 'in', 55, -70, 70, 'How far toward the far wall the ball leaves the launcher.'),
  setting('launcher', 'y', 'Y', 'in', -12.76, -70, 70, 'How far toward red\'s wall the ball leaves the launcher.'),
  setting('launcher', 'height', 'Height', 'in', 14, 2, 40, 'How high above the floor the ball leaves.'),
  setting('launcher', 'yaw', 'Yaw', 'deg', 180, -180, 180, 'Which way the launcher points, from the far wall.'),
  setting('launcher', 'pitch', 'Pitch', 'deg', 70, -30, 90, 'How far up from level the launcher points.'),
  setting('launcher', 'roll', 'Roll', 'deg', 0, -180, 180, 'How far wheel one is turned from the top.'),
  setting('launcher', 'wheel_one_rpm', 'Wheel one', 'rpm', 900, 0, 6000, 'How fast wheel one, on top, spins.'),
  setting('launcher', 'wheel_two_rpm', 'Wheel two', 'rpm', 1200, 0, 6000, 'How fast wheel two, below, spins.'),
  setting('launcher', 'wheel_one_diameter', 'Wheel one diameter', 'in', 3.78, 1.5, 6, 'Across wheel one.'),
  setting('launcher', 'wheel_two_diameter', 'Wheel two diameter', 'in', 3.78, 1.5, 6, 'Across wheel two.'),
  setting('launcher', 'ticks_per_rev', 'Encoder ticks', 'ticks', 28, 1, 10000,
      'Encoder ticks per turn of a wheel, to show each wheel\'s speed as the robot code sets it.'),
  aSwitch('launcher', 'wheels_slow', 'Wheels slow as they throw', 0,
      'The wheels give up some of their speed to the ball, as real flywheels do.'),
  setting('launcher', 'wheel_inertia', 'Wheel inertia', 'lb in²', 0.95, 0.05, 5,
      'Each wheel with its flywheel, when the wheels slow as they throw.'),
  setting('pollen', 'diameter', 'Diameter', 'in', 2.8, 2.6, 3, 'Across the pollen.'),
  setting('pollen', 'mass', 'Mass', 'g', 24.9, 20, 30, 'What the pollen weighs.'),
  setting('pollen', 'inertia', 'Inertia', 'of m r²', 0.634, 0.4, 2 / 3, 'How hard the pollen is to spin.'),
  aSwitch('pollen', 'pieces', 'Pieces in play', 1, 'The loose pollen and the nectar where a match starts.'),
  setting('pollen', 'nectar_mass', 'Nectar mass', 'g', 41.3, 30, 55, 'What a nectar weighs.'),
  aSwitch('field', 'blue_hive_tipped', 'Blue hive tipped', 0, 'Blue\'s hive leans the other way.'),
  aSwitch('field', 'red_hive_tipped', 'Red hive tipped', 0, 'Red\'s hive leans the other way.'),
  setting('air', 'temperature', 'Temperature', '°C', 20, -10, 45, 'How warm the air is.', { value: 25 }),
  setting('air', 'humidity', 'Humidity', '%', 0, 0, 100, 'How damp the air is.'),
  setting('air', 'wind_x', 'Wind toward the far wall', 'in/s', 0, -200, 200, 'A draught across the field.'),
  setting('flight', 'drag', 'Drag', '', 0.45, 0, 1, 'The drag coefficient of a ball that does not spin.'),
  setting('flight', 'lift_slope', 'Lift slope', '', 2, 0.5, 5, 'How fast lift grows with spin.'),
  setting('flight', 'spin_down', 'Spin down', '', 0.012, 0, 0.05, 'How fast the air slows the spin.'),
  ...SURFACES.flatMap((surface) => PROPERTIES.map((property) => setting('bounce', surface.name + '_' + property.name,
      surface.label + ' ' + property.label, '', surface[property.name], 0, property.most,
      property.says, { surface: surface.name, property: property.name }))),
  setting('contact', 'bounce_loss', 'Bounce lost', 'per 100 in/s', 0.038, 0, 0.2,
      'How much less a pollen bounces for each 100 in/s faster it hits.'),
  setting('contact', 'bounce_threshold', 'Bounce threshold', 'in/s', 4, 0.5, 25,
      'A pollen slower than this into a surface does not bounce.'),
  setting('simulation', 'step', 'Step', 'ms', 0.1, 0.01, 2, 'How long each step of the computation is.'),
  setting('simulation', 'longest', 'Longest', 's', 8, 0.5, 20, 'How long a shot is followed before it is called.')
];

const FORGOT = 'The launch remembered spin_rate, which is no longer a setting, so it was forgotten.';

ANSWERS['/sim/launch/settings'] = {
  groups: LAUNCH_GROUPS,
  settings: LAUNCH_SETTINGS,
  surfaces: SURFACES.map(({ name, label, says }) => ({ name, label, says })),
  forgot: FORGOT
};

const BLUE_SCORING = 'Blue Cell (Scoring) <1>';
const AIMS = [
  { cell: BLUE_SCORING, alliance: 'Blue', upturned: true, yaw: 176.5, distance: 84.2, height: 32.1 },
  { cell: 'Red Cell (Scoring) <1>', alliance: 'Red', upturned: true, yaw: 92.25, distance: 120.4, height: 32.1 },
  { cell: 'Blue Cell (Holding) <2>', alliance: 'Blue', upturned: false, yaw: 170, distance: 90, height: 20 }
];

const TOO_STEEP = 89;
const SIMULATING_MS = 400;
const TOO_STEEP_RULE = 'Pitch is from -30 to 90 deg, not 95';

const PREVIEWS_ASKED = [];
const LAUNCHES_ASKED = [];
const LAUNCHES = [];
const LAUNCHED = [];
let viewerLoads = 0;

function freshLaunches() {
  PREVIEWS_ASKED.length = 0;
  LAUNCHES_ASKED.length = 0;
  LAUNCHES.length = 0;
  LAUNCHED.length = 0;
}

function valueIn(body, name) {
  const found = LAUNCH_SETTINGS.find((s) => s.name === name);
  return Object.prototype.hasOwnProperty.call(body, name) ? body[name] : found.byDefault;
}

function previewOf(body) {
  const surface = (n) => Math.PI * valueIn(body, 'wheel_' + n + '_diameter') * valueIn(body, 'wheel_' + n + '_rpm') / 60;
  const one = surface('one');
  const two = surface('two');
  const spin = (one - two) / valueIn(body, 'diameter');
  const yaw = valueIn(body, 'yaw') * Math.PI / 180;
  const pitch = valueIn(body, 'pitch') * Math.PI / 180;
  const ticks = (n) => valueIn(body, 'wheel_' + n + '_rpm') * valueIn(body, 'ticks_per_rev') / 60;
  return {
    speed: (one + two) / 2,
    spinRpm: Math.abs(spin) * 60 / (2 * Math.PI),
    spinKind: one === two ? 'none' : one > two ? 'topspin' : 'backspin',
    spinParameter: Math.abs(spin) * valueIn(body, 'diameter') / 2 / Math.max((one + two) / 2, 1e-9),
    surfaces: [one, two],
    after: [one, two],
    ticksPerSecond: [ticks('one'), ticks('two')],
    at: [valueIn(body, 'x'), valueIn(body, 'y'), valueIn(body, 'height')],
    direction: [Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)],
    aims: AIMS
  };
}

function changedIn(body) {
  const changed = {};
  for (const s of LAUNCH_SETTINGS) {
    if (valueIn(body, s.name) !== s.byDefault) {
      changed[s.name] = valueIn(body, s.name);
    }
  }
  return changed;
}

function summaryOf(id, body, before) {
  const preview = previewOf(body);
  const sinceBefore = {};
  for (const s of LAUNCH_SETTINGS) {
    const was = before ? valueIn(before, s.name) : s.byDefault;
    if (was !== valueIn(body, s.name)) {
      sinceBefore[s.name] = [was, valueIn(body, s.name)];
    }
  }
  return {
    id, at: Date.now(), changed: changedIn(body), sinceBefore,
    ending: { kind: 'scored', says: 'came to rest in ' + BLUE_SCORING + ', which is upturned, so it scores' },
    seconds: 2.1, computeSeconds: 0.012, realTime: 0.0057, impacts: 3,
    speed: preview.speed, spinRpm: preview.spinRpm, spinKind: preview.spinKind,
    apex: { t: 0.5, at: [40, -12, 61.2] }, firstBounce: null, crossings: 1
  };
}

function refused(response, body) {
  if (valueIn(body, 'pitch') > TOO_STEEP) {
    response.writeHead(400, { 'Content-Type': 'application/json; charset=utf-8' });
    response.end(JSON.stringify({ setting: 'pitch', rule: TOO_STEEP_RULE }));
    return true;
  }
  return false;
}

function json(response, answer) {
  response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
  response.end(JSON.stringify(answer));
}

function bodyOf(request, then) {
  let body = '';
  request.on('data', (chunk) => { body += chunk; });
  request.on('end', () => then(JSON.parse(body)));
}

const VIEWER = '<!doctype html><meta charset="utf-8"><title>Launch Sim</title><body><script>'
    + 'window.viewer = { token: Math.random(), previews: [] };'
    + 'window.addEventListener("message", function (e) { window.viewer.previews.push(e.data); });'
    + '</script></body>';

function servedTheLaunchSim(request, response, route) {
  if (request.method === 'POST' && route === '/sim/launch/preview') {
    bodyOf(request, (body) => {
      PREVIEWS_ASKED.push(body);
      if (!refused(response, body)) {
        json(response, previewOf(body));
      }
    });
    return true;
  }
  if (request.method === 'POST' && route === '/sim/launch') {
    bodyOf(request, (body) => {
      LAUNCHES_ASKED.push(body);
      if (refused(response, body)) {
        return;
      }
      setTimeout(() => {
        const id = LAUNCHED.length + 1;
        LAUNCHES.unshift(summaryOf(id, body, LAUNCHED.length ? LAUNCHED[LAUNCHED.length - 1] : null));
        LAUNCHED.push(body);
        json(response, { id });
      }, SIMULATING_MS);
    });
    return true;
  }
  if (request.method === 'GET' && route === '/sim/launches') {
    json(response, { launches: LAUNCHES });
    return true;
  }
  if (request.method === 'GET' && route === '/sim/launches/') {
    viewerLoads += 1;
    response.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    response.end(VIEWER);
    return true;
  }
  return false;
}

function serve() {
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    if (request.method === 'POST' && asked === '/robot') {
      const robot = new URL(request.url, 'http://x').searchParams.get('robot');
      if (!ANSWERS['/me'].robots.includes(robot)) {
        response.writeHead(403, { 'Content-Type': 'text/plain; charset=utf-8' }).end('the admin has not let you work on ' + robot);
        return;
      }
      onto(robot, ANSWERS['/me'].robots);
      response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
      response.end(JSON.stringify(ANSWERS['/me']));
      return;
    }
    if (request.method === 'PUT' && asked.startsWith('/files/')) {
      let body = '';
      request.on('data', (chunk) => { body += chunk; });
      request.on('end', () => {
        SAVES.push(JSON.parse(body));
        response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
        response.end(JSON.stringify({ path: asked.slice('/files/'.length), version: 'v' + (SAVES.length + 1) }));
      });
      return;
    }
    if (servedTheLaunchSim(request, response, asked)) {
      return;
    }
    if (request.method === 'POST' && asked === '/sim/run') {
      const query = new URL(request.url, 'http://x').searchParams;
      RUNS_ASKED.push({ opmode: query.get('opmode'), mode: query.get('mode'), begin: query.get('begin') });
      response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
      response.end(JSON.stringify({ id: RUNS_ASKED.length }));
      return;
    }
    const answer = Object.prototype.hasOwnProperty.call(ANSWERS, asked) ? ANSWERS[asked]
        : asked.startsWith('/files/') ? { path: asked.slice('/files/'.length), version: 'v1', content: CONTENT,
                                          robot: ANSWERS['/me'].robot }
        : null;
    if (answer) {
      response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
      response.end(JSON.stringify(answer));
      return;
    }
    const file = asked === '/' ? path.join(sim, 'dashboard.html')
        : asked === '/static/codemirror.js' ? path.join(sim, 'codemirror.js')
        : null;
    if (!file || !fs.existsSync(file)) {
      response.writeHead(404).end('no route: ' + asked);
      return;
    }
    response.writeHead(200, {
      'Content-Type': asked === '/' ? 'text/html; charset=utf-8' : 'application/javascript; charset=utf-8'
    });
    response.end(fs.readFileSync(file));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

const asAnOlderIpad = () => {
  const realMatchMedia = window.matchMedia.bind(window);
  window.matchMedia = (query) => {
    const live = realMatchMedia(query);
    return {
      media: live.media,
      get matches() { return live.matches; },
      addListener: (fn) => live.addListener(fn),
      removeListener: (fn) => live.removeListener(fn)
    };
  };
  delete window.ResizeObserver;
  delete window.requestIdleCallback;
  delete window.structuredClone;
  delete Intl.Segmenter;
  delete Element.prototype.replaceChildren;
};

const withNoEditorBundle = () => {
  Object.defineProperty(window, 'CM', { get: () => undefined, set: () => {}, configurable: true });
};

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

async function opened(browser, base, asked = {}) {
  const page = await browser.newPage({ viewport: asked.viewport || { width: 1024, height: 768 } });
  const threw = [];
  page.on('pageerror', (e) => threw.push(String((e && e.message) || e)));
  if (asked.engine) {
    await page.addInitScript(asked.engine);
  }
  await page.goto(base + '/', { waitUntil: 'load', timeout: 60_000 });
  const started = await page
      .waitForFunction(() => window.codingPage && window.codingPage.started, null, { timeout: 15_000 })
      .then(() => true)
      .catch(() => false);
  return { page, threw, started };
}

async function itWorks(page, engine) {
  await page.waitForFunction(() => document.querySelectorAll('#files li').length > 0, null, { timeout: 10_000 })
      .catch(() => {});
  const listed = await page.locator('#files li').count();
  check(listed === FILES.length, `${engine}: ${listed} of ${FILES.length} editable files are listed`);

  const who = await page.locator('#who').textContent();
  check(who === 'mia', `${engine}: the header says "${who}" rather than who is signed in`);
  await page.waitForFunction(() => document.getElementById('robot').value === 'nugget', null, { timeout: 10_000 })
      .catch(() => {});
  const robot = await page.locator('#robot').inputValue();
  check(robot === 'nugget', `${engine}: the header says "${robot}" rather than the robot they are on`);
  const offered = await page.locator('#robot option').allInnerTexts();
  check(JSON.stringify(offered) === JSON.stringify(['Reginald', 'Nugget']),
      `${engine}: the header offers ${JSON.stringify(offered)} rather than both robots mia works on`);
  await page.waitForFunction(() => /nugget-develop/.test(document.getElementById('push').title), null,
      { timeout: 10_000 }).catch(() => {});
  const push = await page.locator('#push').getAttribute('title');
  check(/nugget-develop/.test(push), `${engine}: Push says "${push}" rather than the line it lands on`);

  const opened = await page.locator('#files li').first().click({ timeout: 5_000 })
      .then(() => true)
      .catch(() => false);
  check(opened, `${engine}: no file in the list could be opened`);
  await page.waitForFunction(() => /class Plans/.test(document.getElementById('editor').textContent), null,
      { timeout: 10_000 }).catch(() => {});
  const shown = await page.locator('#editor').textContent();
  check(/class Plans/.test(shown), `${engine}: the editor does not hold the file that was opened`);

  await page.locator('nav [data-tab="simulate"]').click({ timeout: 5_000 });
  check(await page.locator('#simulate').isVisible(), `${engine}: tapping Simulate does not open the tab`);
  check(!(await page.locator('#edit').isVisible()), `${engine}: the Edit tab is still shown under Simulate`);
  await page.waitForFunction((name) => document.getElementById('opmodes').textContent.includes(name),
      OPMODE.name, { timeout: 10_000 }).catch(() => {});
  const opmodes = await page.locator('#opmodes').textContent();
  check(opmodes.includes(OPMODE.name), `${engine}: the Simulate tab lists no op modes`);
}

async function aGameIsAskedForByTheBox(page, engine) {
  const box = page.locator('#game-mode');
  check(await box.count() === 1, `${engine}: the Simulate tab offers no game mode`);
  if (await box.count() !== 1) {
    return;
  }
  check(!(await box.isChecked()), `${engine}: game mode is on before anybody asked for it`);
  const run = page.locator('#opmodes button.run').first();

  const before = RUNS_ASKED.length;
  await run.click({ timeout: 5_000 });
  await page.waitForFunction(() => document.getElementById('stage-title').textContent.startsWith('Run '), null,
      { timeout: 10_000 }).catch(() => {});
  await box.check();
  await run.click({ timeout: 5_000 });
  await page.waitForFunction((n) => document.getElementById('stage-title').textContent.startsWith('Run ' + n),
      before + 2, { timeout: 10_000 }).catch(() => {});

  const asked = RUNS_ASKED.slice(before).map((r) => r.mode);
  check(asked.join(',') === 'free,game',
      `${engine}: pressing Run with the box clear and then ticked asked for ${asked.join(', ') || 'nothing'}`);
  const begins = RUNS_ASKED.slice(before).map((r) => r.begin || 'unsaid');
  check(begins.length === 2 && begins.every((b) => b === 'ready'),
      `${engine}: pressing Run asked for runs that begin ${begins.join(', ') || 'never'}, not once the live view `
      + 'under it has drawn the field, so the view loads while the run is already going');
  await box.uncheck();
}

async function aSaveSaysWhichRobotTheFileWasReadOn(page, engine) {
  await page.locator('nav [data-tab="edit"]').click({ timeout: 5_000 });
  const before = SAVES.length;
  await page.locator('#editor .cm-content').click({ timeout: 5_000 });
  await page.keyboard.type('x');
  await page.waitForFunction(() => /saved/.test(document.getElementById('status').textContent), null,
      { timeout: 10_000 }).catch(() => {});
  const saved = SAVES.slice(before);
  check(saved.length > 0 && saved.every((save) => save.robot === 'nugget'),
      `${engine}: typing saved ${JSON.stringify(saved.map((save) => save.robot))} rather than on the robot it was read on`);
}

async function aSwitchComesBackOnTheOtherRobot(page, engine) {
  const [asked] = await Promise.all([
    page.waitForRequest((request) => request.url().includes('/robot?'), { timeout: 10_000 }),
    page.selectOption('#robot', 'reginald')
  ]);
  const url = new URL(asked.url());
  check(asked.method() === 'POST' && url.search === '?robot=reginald',
      `${engine}: switching to Reginald asked ${asked.method()} ${url.pathname + url.search}`);
  await page.waitForFunction(() => window.codingPage && window.codingPage.started
      && document.getElementById('robot').value === 'reginald'
      && document.getElementById('branch').textContent === 'coding/mia', null, { timeout: 15_000 }).catch(() => {});
  check(await page.locator('#robot').inputValue() === 'reginald'
      && await page.locator('#branch').textContent() === 'coding/mia',
      `${engine}: after switching, the page is not back on Reginald's branch`);

  onto('nugget', ['nugget']);
  await page.waitForFunction(() => document.getElementById('robot').value === 'nugget'
      && document.getElementById('robot').disabled, null, { timeout: 15_000 }).catch(() => {});
  check(await page.locator('#robot').inputValue() === 'nugget',
      `${engine}: moved onto Nugget by the admin, the page stays on ${await page.locator('#robot').inputValue()}`);
  check(await page.locator('#robot').isDisabled(),
      `${engine}: on one robot alone, the header still offers a switch`);
  onto('nugget', ['reginald', 'nugget']);
}

async function nothingBroke(page, threw, started, engine) {
  check(started, `${engine}: the page never reported itself started, so it stopped before it finished`);
  const broke = started ? await page.evaluate(() => window.codingPage.broke) : [];
  check(broke.length === 0,
      `${engine}: the page reported ${broke.map((b) => `${b.part}: ${b.message}`).join('; ')}`);
  check(threw.length === 0, `${engine}: the page threw ${threw.join('; ')}`);
}

async function theEditorFollowsTheSystemTheme(page, engine) {
  const background = () => page.evaluate(() =>
      getComputedStyle(document.querySelector('#editor .cm-editor')).backgroundColor);
  await page.emulateMedia({ colorScheme: 'light' });
  const light = await background();
  await page.emulateMedia({ colorScheme: 'dark' });
  await page.waitForFunction((was) =>
      getComputedStyle(document.querySelector('#editor .cm-editor')).backgroundColor !== was,
  light, { timeout: 5_000 }).catch(() => {});
  const dark = await background();
  check(light !== dark, `${engine}: the editor stays ${light} when the system turns dark`);
}

async function itSaysWhatBroke(page) {
  const seen = await page.locator('#broke').isVisible().catch(() => false);
  check(seen, 'with no editor bundle the page says nothing at all');
  if (!seen) {
    return;
  }
  const said = await page.locator('#broke').textContent();
  check(/CM|editor/i.test(said), `with no editor bundle the page says "${said}", which does not name the editor`);
}

const input = (page, name) => page.locator('#launch-setting-' + name);

const viewerSource = (page) => page.locator('#launch-stage').evaluate((frame) => frame.src);

const viewerToken = (page) => page.locator('#launch-stage')
    .evaluate((frame) => frame.contentWindow && frame.contentWindow.viewer ? frame.contentWindow.viewer.token : null);

async function untilTrue(page, fn, arg) {
  return page.waitForFunction(fn, arg, { timeout: 10_000 }).then(() => true).catch(() => false);
}

async function untilAsked(list, from, ok) {
  for (let waited = 0; waited < 10_000; waited += 50) {
    const found = list.slice(from).find(ok);
    if (found) {
      return found;
    }
    await new Promise((resolve) => setTimeout(resolve, 50));
  }
  return null;
}

async function theLaunchSimSetsItsSettingsOutBesideTheView(page, engine) {
  await page.locator('nav [data-tab="launch"]').click({ timeout: 5_000 });
  check(await page.locator('#launch').isVisible(), `${engine}: tapping Launch Sim does not open the tab`);
  check(!(await page.locator('#simulate').isVisible()), `${engine}: the Simulate tab is still shown under Launch Sim`);
  const listed = await untilTrue(page, (n) => document.querySelectorAll('#launch details.launch-group').length === n,
      LAUNCH_GROUPS.length);
  check(listed, `${engine}: the Launch Sim tab sets out ${await page.locator('#launch details.launch-group').count()}`
      + ` groups of settings rather than ${LAUNCH_GROUPS.length}`);
  if (!listed) {
    return false;
  }

  const view = await page.locator('#launch-stage').boundingBox();
  const settings = await page.locator('#launch .launch-settings').boundingBox();
  check(view && view.width >= 480, `${engine}: the launch view is ${view && view.width} px wide, too narrow to watch`);
  check(view && settings && settings.x >= view.x + view.width - 0.5,
      `${engine}: the settings start at ${settings && settings.x} px, not right of the view that ends at `
      + `${view && view.x + view.width} px`);
  const screen = page.viewportSize();
  const go = await page.locator('#launch-go').boundingBox();
  check(go && go.x >= 0 && go.y >= 0 && go.x + go.width <= screen.width && go.y + go.height <= screen.height,
      `${engine}: Launch is at ${JSON.stringify(go)}, out of sight in a ${screen.width}x${screen.height} screen`);

  const open = await page.locator('#launch details.launch-group[open]')
      .evaluateAll((groups) => groups.map((group) => group.dataset.group));
  const meantOpen = LAUNCH_GROUPS.filter((group) => group.open).map((group) => group.name);
  check(open.join(',') === meantOpen.join(','),
      `${engine}: the groups shown open are ${open.join(', ')}, not ${meantOpen.join(', ')}`);
  const air = await page.locator('#launch details.launch-group[data-group="air"] > summary').textContent();
  check(/1 changed/.test(air), `${engine}: the closed Air group says "${air}", not that one of its settings is changed`);
  check(await input(page, 'temperature').inputValue() === '25',
      `${engine}: the temperature reads ${await input(page, 'temperature').inputValue()} rather than the 25 remembered`);
  const pitch = input(page, 'pitch');
  check(await pitch.getAttribute('min') === '-30' && await pitch.getAttribute('max') === '90',
      `${engine}: pitch's input runs ${await pitch.getAttribute('min')} to ${await pitch.getAttribute('max')}`);
  const says = page.locator('#launch [data-setting="pitch"] .says');
  check(await says.isVisible() && /up from level/.test(await says.textContent()),
      `${engine}: what pitch means is not shown under it`);
  check(await page.locator('#launch-setting-wheels_slow').getAttribute('type') === 'checkbox',
      `${engine}: a switch is not a checkbox`);

  const rows = await page.locator('#launch table.bounce-grid tbody tr')
      .evaluateAll((trs) => trs.map((tr) => tr.querySelectorAll('input[type="number"]').length));
  check(rows.length === SURFACES.length && rows.every((n) => n === PROPERTIES.length),
      `${engine}: the bounce grid has rows of ${JSON.stringify(rows)} inputs, not ${SURFACES.length} of `
      + `${PROPERTIES.length}`);
  const forgot = page.locator('#launch-forgot');
  check(await forgot.isVisible() && (await forgot.textContent()).includes(FORGOT),
      `${engine}: the settings forgotten since the last launch are not said`);
  return true;
}

async function aChangeIsPreviewedAndAnAimSetsTheYaw(page, engine) {
  const from = PREVIEWS_ASKED.length;
  await input(page, 'wheel_one_rpm').fill('1000');
  const previewed = await untilAsked(PREVIEWS_ASKED, from, (body) => body.wheel_one_rpm === 1000);
  check(previewed, `${engine}: changing wheel one asked for no preview of it`);
  const shown = await untilTrue(page, () => /218 in\/s/.test(document.getElementById('launch-preview').textContent));
  check(shown, `${engine}: the preview says "${await page.locator('#launch-preview').textContent()}", not the `
      + '218 in/s two wheels at 1000 and 1200 rpm throw at');
  check(/1 changed/.test(await page.locator('#launch details.launch-group[data-group="launcher"] > summary').textContent()),
      `${engine}: the Launcher group does not count the setting changed in it`);
  check(await page.locator('#launch [data-setting="wheel_one_rpm"] .changed-mark').isVisible(),
      `${engine}: wheel one is not marked changed`);
  const told = await untilTrue(page, () => {
    const frame = document.getElementById('launch-stage').contentWindow;
    const said = frame && frame.viewer ? frame.viewer.previews : [];
    return said.some((m) => m && m.type === 'launch-preview' && m.at && m.at.length === 3
        && m.direction && m.direction.length === 3);
  });
  check(told, `${engine}: the view was not sent the preview to draw`);

  const aim = page.locator('#launch-preview button.aim', { hasText: 'Aim at ' + BLUE_SCORING });
  check(await aim.count() === 1, `${engine}: there is no button to aim at ${BLUE_SCORING}`);
  check(await page.locator('#launch-preview button.aim', { hasText: 'Holding' }).count() === 0,
      `${engine}: a cell that is not upturned is offered to aim at`);
  if (await aim.count() !== 1) {
    return;
  }
  const aimed = PREVIEWS_ASKED.length;
  await aim.click({ timeout: 5_000 });
  check(await input(page, 'yaw').inputValue() === '176.5',
      `${engine}: aiming at ${BLUE_SCORING} set the yaw to ${await input(page, 'yaw').inputValue()}, not 176.5`);
  check(await untilAsked(PREVIEWS_ASKED, aimed, (body) => body.yaw === 176.5),
      `${engine}: aiming asked for no preview of the new yaw`);
}

async function aLaunchIsShownComparedAndUsedAgain(page, engine) {
  const loads = viewerLoads;
  const token = await viewerToken(page);
  check(token !== null, `${engine}: the launch view was not opened when the tab was`);

  await page.locator('#launch-go').click({ timeout: 5_000 });
  const busy = await untilTrue(page, () => {
    const go = document.getElementById('launch-go');
    return go.disabled && go.textContent === 'Simulating…';
  });
  check(busy, `${engine}: while the launch computes, Launch neither waits nor says so`);
  const first = await untilAsked(LAUNCHES_ASKED, 0, () => true);
  check(first && first.wheel_one_rpm === 1000 && first.yaw === 176.5
      && Object.keys(first).length === LAUNCH_SETTINGS.length,
      `${engine}: Launch asked for ${JSON.stringify(first)}, not the form as it stands`);
  await untilTrue(page, () => /\/sim\/launches\/#1$/.test(document.getElementById('launch-stage').src));
  check(/\/sim\/launches\/#1$/.test(await viewerSource(page)),
      `${engine}: after the first launch the view is on ${await viewerSource(page)}`);
  await untilTrue(page, () => !document.getElementById('launch-go').disabled);
  check(await page.locator('#launch-go').textContent() === 'Launch', `${engine}: Launch does not come back`);

  await input(page, 'wheel_two_rpm').fill('1300');
  await page.locator('#launch-go').click({ timeout: 5_000 });
  await untilTrue(page, () => /\/sim\/launches\/#2$/.test(document.getElementById('launch-stage').src));
  check(/\/sim\/launches\/#2$/.test(await viewerSource(page)),
      `${engine}: after the second launch the view is on ${await viewerSource(page)}`);
  check(viewerLoads === loads && await viewerToken(page) === token,
      `${engine}: showing another launch loaded the view again rather than changing its fragment`);

  await untilTrue(page, () => document.querySelectorAll('#launches tbody tr[data-id]').length === 2);
  const ids = await page.locator('#launches tbody tr[data-id]').evaluateAll((trs) => trs.map((tr) => tr.dataset.id));
  check(ids.join(',') === '2,1', `${engine}: the launches are listed ${ids.join(', ')}, not newest first`);
  const newest = page.locator('#launches tbody tr[data-id="2"]');
  const since = await newest.locator('.since').textContent().catch(() => '');
  check(/Wheel two/.test(since) && /1200/.test(since) && /1300/.test(since),
      `${engine}: the second launch says "${since}" changed since the first`);
  check(/scored/.test(await newest.locator('.pill').textContent().catch(() => '')),
      `${engine}: the second launch does not say it scored`);

  await page.locator('#launches tbody tr[data-id="1"] input.compare').check({ timeout: 5_000 });
  await untilTrue(page, () => /\/sim\/launches\/#2,1$/.test(document.getElementById('launch-stage').src));
  check(/\/sim\/launches\/#2,1$/.test(await viewerSource(page)),
      `${engine}: ticking the first launch to compare shows ${await viewerSource(page)}`);
  check(viewerLoads === loads, `${engine}: comparing loaded the view again`);

  await page.locator('#launches tbody tr[data-id="1"] button.use').click({ timeout: 5_000 });
  check(await input(page, 'wheel_two_rpm').inputValue() === '1200'
      && await input(page, 'wheel_one_rpm').inputValue() === '1000'
      && await input(page, 'temperature').inputValue() === '25',
      `${engine}: using the first launch's settings left wheel one at ${await input(page, 'wheel_one_rpm').inputValue()},`
      + ` wheel two at ${await input(page, 'wheel_two_rpm').inputValue()} and the air at `
      + `${await input(page, 'temperature').inputValue()} °C`);

  await page.locator('#launch-defaults').click({ timeout: 5_000 });
  check(await input(page, 'wheel_one_rpm').inputValue() === '900' && await input(page, 'yaw').inputValue() === '180'
      && await input(page, 'temperature').inputValue() === '20',
      `${engine}: Defaults left wheel one at ${await input(page, 'wheel_one_rpm').inputValue()} and the air at `
      + `${await input(page, 'temperature').inputValue()} °C`);
  check(!(await page.locator('#launch [data-setting="wheel_one_rpm"] .changed-mark').isVisible()),
      `${engine}: a setting back at its default is still marked changed`);
}

async function aRefusedSettingIsMarkedWhereItIs(page, engine) {
  const from = LAUNCHES_ASKED.length;
  await input(page, 'pitch').fill('95');
  await page.locator('#launch details.launch-group[data-group="launcher"]')
      .evaluate((group) => { group.open = false; });
  await page.locator('#launch-go').click({ timeout: 5_000 });
  check(await untilAsked(LAUNCHES_ASKED, from, (body) => body.pitch === 95),
      `${engine}: a pitch of 95 was not asked of the server`);
  const marked = await untilTrue(page,
      () => document.getElementById('launch-setting-pitch').getAttribute('aria-invalid') === 'true');
  check(marked, `${engine}: the pitch the launch was refused for is not marked`);
  check(await page.locator('#launch details.launch-group[data-group="launcher"]').evaluate((group) => group.open),
      `${engine}: the group holding the refused pitch stays closed`);
  check(await input(page, 'pitch').isVisible(), `${engine}: the refused pitch is out of sight`);
  check((await page.locator('#launch-message').textContent()).includes(TOO_STEEP_RULE),
      `${engine}: the message says "${await page.locator('#launch-message').textContent()}", not the rule`);
  await untilTrue(page, () => !document.getElementById('launch-go').disabled);

  await page.locator('#launch [data-setting="pitch"] button.reset').click({ timeout: 5_000 });
  check(await input(page, 'pitch').inputValue() === '70', `${engine}: resetting pitch did not put it back at 70`);
  const cleared = await untilTrue(page,
      () => document.getElementById('launch-setting-pitch').getAttribute('aria-invalid') !== 'true');
  check(cleared, `${engine}: the pitch stays marked once it is back in range`);
}

async function theLaunchSimWorks(page, engine) {
  freshLaunches();
  if (!(await theLaunchSimSetsItsSettingsOutBesideTheView(page, engine))) {
    return;
  }
  await aChangeIsPreviewedAndAnAimSetsTheYaw(page, engine);
  await aLaunchIsShownComparedAndUsedAgain(page, engine);
  await aRefusedSettingIsMarkedWhereItIs(page, engine);
}

async function main() {
  const server = await serve();
  const base = `http://127.0.0.1:${server.address().port}`;
  const browser = await chrome();
  try {
    const current = await opened(browser, base);
    await itWorks(current.page, 'this engine');
    await aGameIsAskedForByTheBox(current.page, 'this engine');
    await theEditorFollowsTheSystemTheme(current.page, 'this engine');
    await aSaveSaysWhichRobotTheFileWasReadOn(current.page, 'this engine');
    await theLaunchSimWorks(current.page, 'this engine');
    await nothingBroke(current.page, current.threw, current.started, 'this engine');

    const older = await opened(browser, base, { engine: asAnOlderIpad, viewport: { width: 810, height: 1080 } });
    await itWorks(older.page, 'an older iPad');
    await aGameIsAskedForByTheBox(older.page, 'an older iPad');
    await theEditorFollowsTheSystemTheme(older.page, 'an older iPad');
    await theLaunchSimWorks(older.page, 'an older iPad');
    await nothingBroke(older.page, older.threw, older.started, 'an older iPad');

    await aSwitchComesBackOnTheOtherRobot(current.page, 'this engine');
    await nothingBroke(current.page, current.threw, true, 'this engine, after switching');

    const bare = await opened(browser, base, { engine: withNoEditorBundle });
    await itSaysWhatBroke(bare.page);
    const listed = await bare.page.locator('#files li').count();
    check(listed === FILES.length, `with no editor bundle ${listed} of ${FILES.length} files are listed`);
    await bare.page.locator('nav [data-tab="simulate"]').click({ timeout: 5_000 });
    check(await bare.page.locator('#simulate').isVisible(),
        'with no editor bundle the Simulate tab cannot be reached');
    await bare.page.locator('nav [data-tab="launch"]').click({ timeout: 5_000 });
    check(await bare.page.locator('#launch').isVisible(),
        'with no editor bundle the Launch Sim tab cannot be reached');
    check(await untilTrue(bare.page, (n) => document.querySelectorAll('#launch details.launch-group').length === n,
        LAUNCH_GROUPS.length), 'with no editor bundle the Launch Sim tab sets out no settings');
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the coding server\'s dashboard did not start as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log('the dashboard says which robot it is on, switches to another the user works on and comes back on '
      + 'it, follows the admin moving it, saves on the robot a file was read on, '
      + 'lists its files, opens one in the editor and reaches the Simulate tab, where a '
      + 'run is a game only when the box says so and begins once the view under it has drawn the field, '
      + 'and the Launch Sim tab, which sets its settings out in groups to the right of a view at least 480 px '
      + 'wide with Launch in sight, previews each change, aims at an upturned cell, launches what the form says '
      + 'and shows each launch, and the launches ticked to compare, by changing the view\'s fragment alone, '
      + 'takes a launch\'s settings back, and marks the setting a launch was refused for, on '
      + 'this engine and on one shaped like an older iPad\'s, and '
      + 'says so on the page when the editor cannot start, where both tabs are still reached.');
}

await main();
