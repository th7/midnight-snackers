import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
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

const here = path.dirname(fileURLToPath(import.meta.url));
const ANSWERED = JSON.parse(fs.readFileSync(path.join(here, 'launch-answers.json'), 'utf8'));
const LAUNCH_GROUPS = ANSWERED.settings.groups;
const LAUNCH_SETTINGS = ANSWERED.settings.settings;
const SURFACES = ANSWERED.settings.surfaces;
const PROPERTIES = [...new Set(LAUNCH_SETTINGS.filter((s) => s.property).map((s) => s.property))];
const PREVIEW = ANSWERED.preview;
const SUMMARIES = ANSWERED.launches.launches.slice().reverse();
const TOO_STEEP_RULE = ANSWERED.refused.rule;

const REMEMBERED = { temperature: 25 };
const FORGOT = 'Went back to the default for what was kept of spin_rate (there is no such setting now).';
const INSIDE_A_PIECE = { x: -69, y: 60.6, height: 2 };
const INSIDE_RULE = 'the pollen would start inside Pollen <1>, at Vec3[x=-69.0, y=60.6, z=2.0]';
const TOO_STEEP = 89;
const SIMULATING_MS = 400;

const PREVIEWS_ASKED = [];
const LAUNCHES_ASKED = [];
const LAUNCHES = [];
const LAUNCHED = [];
const UNKNOWN_TO_THE_BENCH = new Set();
let settingsAsked = 0;
let viewerLoads = 0;
let previewsHeld = null;

function freshLaunches() {
  PREVIEWS_ASKED.length = 0;
  LAUNCHES_ASKED.length = 0;
  LAUNCHES.length = 0;
  LAUNCHED.length = 0;
  UNKNOWN_TO_THE_BENCH.clear();
  previewsHeld = null;
}

function holdPreviews() {
  previewsHeld = [];
}

function releasePreviews() {
  const held = previewsHeld || [];
  previewsHeld = null;
  held.forEach((answer) => answer());
}

const settingNamed = (name) => LAUNCH_SETTINGS.find((s) => s.name === name);
const known = () => LAUNCH_SETTINGS.filter((s) => !UNKNOWN_TO_THE_BENCH.has(s.name));

function listingAsRemembered() {
  return Object.assign({}, ANSWERED.settings, {
    settings: known().map((s) => Object.assign({}, s,
        Object.prototype.hasOwnProperty.call(REMEMBERED, s.name) ? { value: REMEMBERED[s.name] } : {})),
    forgot: FORGOT
  });
}

function valueIn(body, name) {
  return Object.prototype.hasOwnProperty.call(body, name) ? body[name] : settingNamed(name).byDefault;
}

const startsInsideAPiece = (body) => Object.keys(INSIDE_A_PIECE)
    .every((name) => valueIn(body, name) === INSIDE_A_PIECE[name]);

function refusalOf(body, launching) {
  const names = known().map((s) => s.name);
  const stranger = Object.keys(body).find((name) => !names.includes(name));
  if (stranger !== undefined) {
    return { setting: stranger, rule: `a launch setting is ${names.join(', ')}, not '${stranger}'` };
  }
  if (valueIn(body, 'pitch') > TOO_STEEP) {
    return ANSWERED.refused;
  }
  if (launching && startsInsideAPiece(body)) {
    return { setting: null, rule: INSIDE_RULE };
  }
  return null;
}

function summaryOf(id) {
  return Object.assign({}, SUMMARIES[(id - 1) % SUMMARIES.length], { id });
}

function json(response, answer, status = 200) {
  response.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8' });
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
  if (request.method === 'GET' && route === '/sim/launch/settings') {
    settingsAsked += 1;
    json(response, listingAsRemembered());
    return true;
  }
  if (request.method === 'POST' && route === '/sim/launch/preview') {
    bodyOf(request, (body) => {
      PREVIEWS_ASKED.push(body);
      const refusal = refusalOf(body, false);
      const answer = () => (refusal ? json(response, refusal, 400) : json(response, PREVIEW));
      if (previewsHeld) {
        previewsHeld.push(answer);
      } else {
        answer();
      }
    });
    return true;
  }
  if (request.method === 'POST' && route === '/sim/launch') {
    bodyOf(request, (body) => {
      LAUNCHES_ASKED.push(body);
      const refusal = refusalOf(body, true);
      if (refusal) {
        json(response, refusal, 400);
        return;
      }
      setTimeout(() => {
        LAUNCHED.push(body);
        const id = LAUNCHED.length;
        LAUNCHES.unshift(summaryOf(id));
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

const message = (page) => page.locator('#launch-message').textContent();

const viewerAt = (page) => page.locator('#launch-stage').evaluate((frame) => frame.contentWindow.location.href);

const viewerToken = (page) => page.locator('#launch-stage')
    .evaluate((frame) => frame.contentWindow && frame.contentWindow.viewer ? frame.contentWindow.viewer.token : null);

const theViewIsAt = (page, ending) => untilTrue(page,
    (wanted) => document.getElementById('launch-stage').contentWindow.location.href.endsWith(wanted), ending);

const shownAs = (name, value) => (settingNamed(name).switch ? (value === 1 ? 'on' : 'off') : String(value));

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

const settle = (page) => untilTrue(page, () => !document.getElementById('launch-go').disabled);

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
  const inputs = await page.locator('#launch input[id^="launch-setting-"]').count();
  check(inputs === LAUNCH_SETTINGS.length,
      `${engine}: the Launch Sim tab offers ${inputs} inputs for the bench's ${LAUNCH_SETTINGS.length} settings`);

  const view = await page.locator('#launch-stage').boundingBox();
  const settings = await page.locator('#launch .launch-settings').boundingBox();
  check(view && view.width >= 480, `${engine}: the launch view is ${view && view.width} px wide, too narrow to watch`);
  check(view && view.height >= 300, `${engine}: the launch view is ${view && view.height} px high, too low to watch`);
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
  check(await input(page, 'temperature').inputValue() === String(REMEMBERED.temperature),
      `${engine}: the temperature reads ${await input(page, 'temperature').inputValue()} rather than the `
      + `${REMEMBERED.temperature} remembered`);
  const pitch = input(page, 'pitch');
  check(await pitch.getAttribute('min') === String(settingNamed('pitch').least)
      && await pitch.getAttribute('max') === String(settingNamed('pitch').most),
      `${engine}: pitch's input runs ${await pitch.getAttribute('min')} to ${await pitch.getAttribute('max')}`);
  const says = page.locator('#launch [data-setting="pitch"] .says');
  check(await says.isVisible() && (await says.textContent()).includes(settingNamed('pitch').says),
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
  const said = [`${Math.round(PREVIEW.speed)} in/s`, `${Math.round(PREVIEW.spinRpm)} rpm ${PREVIEW.spinKind}`,
                ...PREVIEW.ticksPerSecond.map((ticks) => `${Math.round(ticks)} ticks/s`)];
  const shown = await untilTrue(page, (all) => all.every((one) => document.getElementById('launch-preview')
      .textContent.includes(one)), said);
  check(shown, `${engine}: the preview says "${await page.locator('#launch-preview').textContent()}", not `
      + `${said.join(', ')} as the bench answered`);
  check(/1 changed/.test(await page.locator('#launch details.launch-group[data-group="launcher"] > summary').textContent()),
      `${engine}: the Launcher group does not count the setting changed in it`);
  check(await page.locator('#launch [data-setting="wheel_one_rpm"] .changed-mark').isVisible(),
      `${engine}: wheel one is not marked changed`);
  const told = await untilTrue(page, () => {
    const frame = document.getElementById('launch-stage').contentWindow;
    const previews = frame && frame.viewer ? frame.viewer.previews : [];
    return previews.some((m) => m && m.type === 'launch-preview' && m.at && m.at.length === 3
        && m.direction && m.direction.length === 3);
  });
  check(told, `${engine}: the view was not sent the preview to draw`);

  for (const aim of PREVIEW.aims.filter((one) => !one.upturned)) {
    check(await page.locator('#launch-preview button.aim', { hasText: aim.cell }).count() === 0,
        `${engine}: ${aim.cell}, which is not upturned, is offered to aim at`);
  }
  const target = PREVIEW.aims.find((one) => one.upturned && one.alliance === 'Blue');
  const aim = page.locator('#launch-preview button.aim', { hasText: 'Aim at ' + target.cell });
  check(await aim.count() === 1, `${engine}: there is no button to aim at ${target.cell}`);
  if (await aim.count() !== 1) {
    return;
  }
  const yaw = Math.round(target.yaw * 100) / 100;
  const aimed = PREVIEWS_ASKED.length;
  await aim.click({ timeout: 5_000 });
  check(await input(page, 'yaw').inputValue() === String(yaw),
      `${engine}: aiming at ${target.cell} set the yaw to ${await input(page, 'yaw').inputValue()}, not ${yaw}`);
  check(await untilAsked(PREVIEWS_ASKED, aimed, (body) => body.yaw === yaw),
      `${engine}: aiming asked for no preview of the new yaw`);
  return yaw;
}

async function aLaunchIsShownComparedAndUsedAgain(page, engine, yaw) {
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
  check(first && first.wheel_one_rpm === 1000 && first.yaw === yaw
      && Object.keys(first).length === LAUNCH_SETTINGS.length,
      `${engine}: Launch asked for ${JSON.stringify(first)}, not the form as it stands`);
  await theViewIsAt(page, '/sim/launches/#1');
  check((await viewerAt(page)).endsWith('/sim/launches/#1'),
      `${engine}: after the first launch the view is on ${await viewerAt(page)}`);
  await settle(page);
  check(await page.locator('#launch-go').textContent() === 'Launch', `${engine}: Launch does not come back`);

  await input(page, 'wheel_two_rpm').fill('1300');
  await page.locator('#launch-go').click({ timeout: 5_000 });
  await theViewIsAt(page, '/sim/launches/#2');
  check((await viewerAt(page)).endsWith('/sim/launches/#2'),
      `${engine}: after the second launch the view is on ${await viewerAt(page)}`);
  check(viewerLoads === loads && await viewerToken(page) === token,
      `${engine}: showing another launch loaded the view again rather than changing its fragment`);

  await untilTrue(page, () => document.querySelectorAll('#launches tbody tr[data-id]').length === 2);
  const ids = await page.locator('#launches tbody tr[data-id]').evaluateAll((trs) => trs.map((tr) => tr.dataset.id));
  check(ids.join(',') === '2,1', `${engine}: the launches are listed ${ids.join(', ')}, not newest first`);
  const second = summaryOf(2);
  const newest = page.locator('#launches tbody tr[data-id="2"]');
  const since = await newest.locator('.since').textContent().catch(() => '');
  for (const [name, [was, now]] of Object.entries(second.sinceBefore)) {
    const setting = settingNamed(name);
    const meant = `${setting.label} ${shownAs(name, was)} → ${shownAs(name, now)} ${setting.unit}`.trim();
    check(since.includes(meant), `${engine}: the second launch says "${since}" changed since the first, not "${meant}"`);
  }
  const pill = await newest.locator('.pill').textContent().catch(() => '');
  check(pill === second.ending.kind, `${engine}: the second launch's ending reads "${pill}", not ${second.ending.kind}`);
  const apex = `${second.apex.toFixed(1)} in`;
  check((await newest.textContent()).includes(apex), `${engine}: the second launch does not give its apex, ${apex}`);

  await page.locator('#launches tbody tr[data-id="1"] input.compare').check({ timeout: 5_000 });
  await theViewIsAt(page, '/sim/launches/#2,1');
  check((await viewerAt(page)).endsWith('/sim/launches/#2,1'),
      `${engine}: ticking the first launch to compare shows ${await viewerAt(page)}`);
  check(viewerLoads === loads, `${engine}: comparing loaded the view again`);

  const used = summaryOf(1).changed;
  await page.locator('#launches tbody tr[data-id="1"] button.use').click({ timeout: 5_000 });
  for (const setting of LAUNCH_SETTINGS) {
    const meant = Object.prototype.hasOwnProperty.call(used, setting.name) ? used[setting.name] : setting.byDefault;
    const field = input(page, setting.name);
    const holds = setting.switch ? ((await field.isChecked()) ? 1 : 0) : Number(await field.inputValue());
    check(holds === meant, `${engine}: using the first launch's settings left ${setting.name} at ${holds}, not ${meant}`);
  }

  await page.locator('#launch-defaults').click({ timeout: 5_000 });
  check(await input(page, 'wheel_one_rpm').inputValue() === '900' && await input(page, 'yaw').inputValue() === '180'
      && await input(page, 'temperature').inputValue() === '20' && await input(page, 'longest').inputValue() === '8',
      `${engine}: Defaults left wheel one at ${await input(page, 'wheel_one_rpm').inputValue()}, the air at `
      + `${await input(page, 'temperature').inputValue()} °C and the longest shot at `
      + `${await input(page, 'longest').inputValue()} s`);
  check(!(await page.locator('#launch [data-setting="longest"] .changed-mark').isVisible()),
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
  check((await message(page)).includes(TOO_STEEP_RULE),
      `${engine}: the message says "${await message(page)}", not the rule`);
  await settle(page);

  await page.locator('#launch [data-setting="pitch"] button.reset').click({ timeout: 5_000 });
  check(await input(page, 'pitch').inputValue() === '70', `${engine}: resetting pitch did not put it back at 70`);
  const cleared = await untilTrue(page,
      () => document.getElementById('launch-setting-pitch').getAttribute('aria-invalid') !== 'true');
  check(cleared, `${engine}: the pitch stays marked once it is back in range`);
}

async function aLaunchsRefusalStaysUntilTheFormChanges(page, engine) {
  await settle(page);
  const previewsFrom = PREVIEWS_ASKED.length;
  const launchesFrom = LAUNCHES_ASKED.length;
  holdPreviews();
  for (const [name, value] of Object.entries(INSIDE_A_PIECE)) {
    await input(page, name).fill(String(value));
  }
  await page.locator('#launch-go').click({ timeout: 5_000 });
  check(await untilAsked(LAUNCHES_ASKED, launchesFrom, startsInsideAPiece),
      `${engine}: a launch from inside a piece was not asked of the server`);
  const said = await untilTrue(page,
      (rule) => document.getElementById('launch-message').textContent.includes(rule), INSIDE_RULE);
  check(said, `${engine}: a launch refused for starting inside a piece says "${await message(page)}", not why`);
  const previewed = await untilAsked(PREVIEWS_ASKED, previewsFrom, startsInsideAPiece);
  check(previewed, `${engine}: the edit that put the launch inside a piece asked for no preview`);
  await page.evaluate(() => { window.previewShownBefore = document.querySelector('#launch-preview dl'); });
  releasePreviews();
  const answered = await untilTrue(page,
      () => document.querySelector('#launch-preview dl') !== window.previewShownBefore);
  check(answered, `${engine}: the preview the same edit asked for was never shown`);
  check((await message(page)).includes(INSIDE_RULE),
      `${engine}: once the preview the same edit asked for came back, the launch's refusal was gone and the `
      + `message says "${await message(page)}", so Launch seems to have done nothing`);

  await input(page, 'height').fill(String(settingNamed('height').byDefault));
  const gone = await untilTrue(page,
      (rule) => !document.getElementById('launch-message').textContent.includes(rule), INSIDE_RULE);
  check(gone, `${engine}: once the form changed, the launch's refusal "${await message(page)}" still stands`);
  for (const name of ['x', 'y']) {
    await input(page, name).fill(String(settingNamed(name).byDefault));
  }
}

async function theViewKeepsHowItIsDrawn(page, engine) {
  await settle(page);
  await page.locator('#launch-stage').evaluate((frame) => { frame.contentWindow.location.search = '?view=flat'; });
  await untilTrue(page, () => {
    const shown = document.getElementById('launch-stage').contentWindow;
    return shown.location.search === '?view=flat' && !!shown.viewer && shown.document.readyState === 'complete';
  });
  const loads = viewerLoads;
  const token = await viewerToken(page);
  const keptAt = async (doing, ending) => {
    await theViewIsAt(page, ending);
    const at = await viewerAt(page);
    check(at.endsWith(ending), `${engine}: ${doing} put the view on ${at}, not on ${ending} drawn as the user chose`);
    check(viewerLoads === loads && await viewerToken(page) === token,
        `${engine}: ${doing} loaded the view again rather than changing its fragment alone`);
  };

  await page.locator('#launches tbody tr[data-id="1"] a.show').click({ timeout: 5_000 });
  await keptAt('showing launch 1', '/sim/launches/?view=flat#1');
  await page.locator('#launches tbody tr[data-id="2"] input.compare').check({ timeout: 5_000 });
  await keptAt('ticking launch 2 to compare', '/sim/launches/?view=flat#1,2');
  await page.locator('#launch-go').click({ timeout: 5_000 });
  await keptAt('launching again', '/sim/launches/?view=flat#3,2,1');
  await settle(page);
}

async function anUnreadableNumberIsNeitherPreviewedNorLaunched(page, engine) {
  await settle(page);
  const field = input(page, 'wheel_one_rpm');
  await field.click({ timeout: 5_000 });
  await page.keyboard.press('ControlOrMeta+A');
  const previewsFrom = PREVIEWS_ASKED.length;
  await page.keyboard.type('1e');
  const marked = await untilTrue(page,
      () => document.getElementById('launch-setting-wheel_one_rpm').getAttribute('aria-invalid') === 'true');
  check(marked, `${engine}: wheel one holds text that is not a number and is not marked`);
  check(/not a number/.test(await message(page)) && (await message(page)).includes(settingNamed('wheel_one_rpm').label),
      `${engine}: with text in wheel one that is not a number the message says "${await message(page)}"`);
  await page.waitForTimeout(700);
  const previewed = PREVIEWS_ASKED.slice(previewsFrom).filter((body) => body.wheel_one_rpm !== 1);
  check(previewed.length === 0, `${engine}: with text in wheel one that is not a number, a preview was asked for `
      + `wheel one at ${previewed.map((body) => body.wheel_one_rpm).join(', ')}`);

  const launchesFrom = LAUNCHES_ASKED.length;
  await page.locator('#launch-go').click({ timeout: 5_000 });
  await page.waitForTimeout(700);
  check(LAUNCHES_ASKED.length === launchesFrom, `${engine}: with text in wheel one that is not a number, Launch asked `
      + `for wheel one at ${LAUNCHES_ASKED.slice(launchesFrom).map((body) => body.wheel_one_rpm).join(', ')}`);
  check(/not a number/.test(await message(page)),
      `${engine}: pressing Launch with text in wheel one that is not a number says "${await message(page)}"`);

  const blankFrom = PREVIEWS_ASKED.length;
  await field.fill('');
  const cleared = await untilTrue(page,
      () => document.getElementById('launch-setting-wheel_one_rpm').getAttribute('aria-invalid') !== 'true');
  check(cleared, `${engine}: wheel one stays marked once it is blank`);
  check(await untilAsked(PREVIEWS_ASKED, blankFrom,
      (body) => body.wheel_one_rpm === settingNamed('wheel_one_rpm').byDefault),
      `${engine}: a blank wheel one was not previewed at its default`);
  check(!/not a number/.test(await message(page)), `${engine}: once wheel one is blank the message still says `
      + `"${await message(page)}"`);
}

async function aSettingTheBenchNoLongerKnowsReloadsTheSettings(page, engine) {
  await settle(page);
  const asked = settingsAsked;
  const previewsFrom = PREVIEWS_ASKED.length;
  UNKNOWN_TO_THE_BENCH.add('roll');
  await input(page, 'wheel_two_rpm').fill('1250');
  const reloaded = await untilTrue(page, () => document.getElementById('launch-setting-roll') === null);
  check(reloaded && settingsAsked > asked,
      `${engine}: the bench no longer knows roll, and the settings were asked for ${settingsAsked - asked} times since`);
  check(/reloaded/.test(await message(page)),
      `${engine}: after the bench refused roll, the message says "${await message(page)}", not that the settings were `
      + 'reloaded');
  check(await untilAsked(PREVIEWS_ASKED, previewsFrom, (body) => !('roll' in body) && body.wheel_two_rpm === 1250),
      `${engine}: once the settings were reloaded, no preview was asked of the form as it stood`);
  check(await input(page, 'wheel_two_rpm').inputValue() === '1250',
      `${engine}: reloading the settings threw away wheel two's ${await input(page, 'wheel_two_rpm').inputValue()}`);
  UNKNOWN_TO_THE_BENCH.clear();
}

async function theLaunchSimWorks(page, engine) {
  freshLaunches();
  if (!(await theLaunchSimSetsItsSettingsOutBesideTheView(page, engine))) {
    return;
  }
  const yaw = await aChangeIsPreviewedAndAnAimSetsTheYaw(page, engine);
  await aLaunchIsShownComparedAndUsedAgain(page, engine, yaw);
  await aRefusedSettingIsMarkedWhereItIs(page, engine);
  await aLaunchsRefusalStaysUntilTheFormChanges(page, engine);
  await theViewKeepsHowItIsDrawn(page, engine);
  await anUnreadableNumberIsNeitherPreviewedNorLaunched(page, engine);
  await aSettingTheBenchNoLongerKnowsReloadsTheSettings(page, engine);
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
      + `and the Launch Sim tab, which sets the bench's ${LAUNCH_SETTINGS.length} settings out in its `
      + `${LAUNCH_GROUPS.length} groups to the right of a view at least 480 px wide and 300 px high with Launch in `
      + 'sight, previews each change, aims at an upturned cell, launches what the form says and shows each launch, '
      + 'and the launches ticked to compare, by changing the view\'s fragment alone and keeping how the view is '
      + 'drawn, takes a launch\'s settings back, marks the setting a launch was refused for, keeps a launch\'s '
      + 'refusal in sight until the form changes, sends nothing while a number cannot be read, and reloads the '
      + 'settings when the bench no longer knows one, on '
      + 'this engine and on one shaped like an older iPad\'s, and '
      + 'says so on the page when the editor cannot start, where both tabs are still reached.');
}

await main();
