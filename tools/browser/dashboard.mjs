// Opens the coding server's dashboard page in a real browser and checks that it starts: the file
// list, the tabs and the editor. Nothing else opens this page, so nothing else can tell a page that
// started from one that threw on its first line and left the shell it was served as.
//
// It opens the page three times: on the engine this machine has, on one shaped like the WebKit an
// older iPad carries, and on one where the editor bundle never arrived. The last two are the ones
// worth having. A page that only ever runs on a current Chromium is a page whose first user on a
// tablet finds out what it needs.
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

// What the page asks the user listener for, answered as the server answers it.
const ANSWERS = {
  '/me': { state: 'approved', username: 'mia', robot: 'nugget', robots: ['reginald', 'nugget'], branch: 'nugget/mia' },
  '/files': { files: FILES },
  '/git/status': { branch: 'nugget/mia', develop: 'nugget-develop', changed: [], behind: 0, ahead: 0, pushable: false,
                   head: '0123456' },
  '/build': { available: true, ok: true, problems: [] },
  '/sim/catalog': [OPMODE],
  '/sim/status': { running: false, runs: [] }
};

// Every run the page asked for, by how it asked: which op mode, and whether it was a game.
const RUNS_ASKED = [];

const SAVES = [];

const BRANCHES = { reginald: 'coding/mia', nugget: 'nugget/mia' };

function onto(robot, robots) {
  ANSWERS['/me'] = { ...ANSWERS['/me'], robot, robots, branch: BRANCHES[robot] };
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

// An iPadOS 13 MediaQueryList is not an EventTarget: it carries addListener and nothing else. The
// rest are the globals that WebKit of that age has not got, every one of which CodeMirror itself
// feature-tests -- so a page that needs them is a page of ours that forgot to.
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

// What the page is for: the files are listed, the header says who you are, a file opens in the
// editor, and the Simulate tab is a tab you can reach.
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

// A run is a game when the box says so and free play when it does not, and the page says which it
// asked for rather than leaving it to what the server does with a run that says nothing.
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

// The theme follows the system's, which is the one thing the page needs a media query listener for.
// Proving it changes is what tells a listener that was wired from one that was quietly dropped.
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

// A page that cannot start a piece of itself has to say so. The shell it is served as -- ellipses in
// the header, an empty file list -- is what "nothing to show you" looks like too, so silence here
// reads as a verdict rather than as a failure.
async function itSaysWhatBroke(page) {
  const seen = await page.locator('#broke').isVisible().catch(() => false);
  check(seen, 'with no editor bundle the page says nothing at all');
  if (!seen) {
    return;
  }
  const said = await page.locator('#broke').textContent();
  check(/CM|editor/i.test(said), `with no editor bundle the page says "${said}", which does not name the editor`);
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
    await nothingBroke(current.page, current.threw, current.started, 'this engine');

    const older = await opened(browser, base, { engine: asAnOlderIpad, viewport: { width: 810, height: 1080 } });
    await itWorks(older.page, 'an older iPad');
    await aGameIsAskedForByTheBox(older.page, 'an older iPad');
    await theEditorFollowsTheSystemTheme(older.page, 'an older iPad');
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
      + 'run is a game only when the box says so and begins once the view under it has drawn the field, on '
      + 'this engine and on one shaped like an older iPad\'s, and '
      + 'says so on the page when the editor cannot start.');
}

await main();
