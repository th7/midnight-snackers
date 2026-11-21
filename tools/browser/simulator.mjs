import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chrome, sim } from './bench.mjs';

const OPMODE = { name: 'driveForward', kind: 'auto', group: 'Autonomous',
                 where: 'org.firstinspires.ftc.reginald.opmode.Autos', seed: null };
const DOES_NOT_COMPILE = 'the sources do not compile:\n'
    + 'org/firstinspires/ftc/reginald/Plans.java:161: illegal start of expression';
const AS_THE_CODING_SERVER_SERVES_IT = '<html lang="en" data-host="coding-server">';
const AS_THE_SIMULATOR_SERVES_IT = '<html lang="en" data-host="simulator">';
const THE_EDITOR = '<script src="/static/codemirror.js"></script>\n';
const LIVE_VIEW = '<!doctype html><meta charset="utf-8"><title>Live view</title><body></body>';
const VIEWER = '<!doctype html><meta charset="utf-8"><title>Viewer</title><body></body>';

const here = path.dirname(fileURLToPath(import.meta.url));
const LAUNCH = JSON.parse(fs.readFileSync(path.join(here, 'launch-answers.json'), 'utf8'));
const FEEDER = JSON.parse(fs.readFileSync(path.join(here, 'feeder-answers.json'), 'utf8'));

const ASKED = [];
const RUNS = [];
let compiles = false;

function json(response, answer, status = 200) {
  response.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8' });
  response.end(JSON.stringify(answer));
}

function html(response, page) {
  response.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
  response.end(page);
}

function once(page, asTheCodingServerServesIt, asTheSimulatorServesIt) {
  if (page.split(asTheCodingServerServesIt).length !== 2) {
    throw new Error('dashboard.html does not hold ' + asTheCodingServerServesIt + ' exactly once, '
        + 'which LocalSimulator serves as "' + asTheSimulatorServesIt + '"');
  }
  return page.replace(asTheCodingServerServesIt, asTheSimulatorServesIt);
}

function thePage() {
  const page = fs.readFileSync(path.join(sim, 'dashboard.html'), 'utf8');
  return once(once(page, AS_THE_CODING_SERVER_SERVES_IT, AS_THE_SIMULATOR_SERVES_IT), THE_EDITOR, '');
}

function serve() {
  const page = thePage();
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    ASKED.push(asked);
    const route = request.method + ' ' + asked;
    if (route === 'GET /') {
      html(response, page);
    } else if (route === 'GET /sim/catalog') {
      if (compiles) {
        json(response, [OPMODE]);
      } else {
        response.writeHead(500, { 'Content-Type': 'text/plain; charset=utf-8' }).end(DOES_NOT_COMPILE);
      }
    } else if (route === 'GET /sim/status') {
      json(response, { running: false, runs: [] });
    } else if (route === 'POST /sim/run') {
      RUNS.push(new URL(request.url, 'http://x').searchParams.get('opmode'));
      json(response, { id: RUNS.length });
    } else if (request.method === 'GET' && /^\/sim\/runs\/\d+\/$/.test(asked)) {
      html(response, LIVE_VIEW);
    } else if (route === 'GET /sim/launch/settings') {
      json(response, LAUNCH.settings);
    } else if (route === 'GET /sim/launches') {
      json(response, { launches: [] });
    } else if (route === 'GET /sim/feed/settings') {
      json(response, FEEDER.settings);
    } else if (route === 'GET /sim/feeds') {
      json(response, { feeds: [] });
    } else if (route === 'GET /sim/launches/' || route === 'GET /sim/feeds/') {
      html(response, VIEWER);
    } else {
      response.writeHead(404).end('no route: ' + route);
    }
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
  delete Element.prototype.replaceChildren;
};

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

async function untilTrue(page, fn, arg) {
  return page.waitForFunction(fn, arg, { timeout: 10_000 }).then(() => true).catch(() => false);
}

async function opened(browser, base, engine) {
  const page = await browser.newPage({ viewport: { width: 1024, height: 768 } });
  const threw = [];
  page.on('pageerror', (e) => threw.push(String((e && e.message) || e)));
  if (engine) {
    await page.addInitScript(engine);
  }
  await page.goto(base + '/#edit', { waitUntil: 'load', timeout: 60_000 });
  const started = await untilTrue(page, () => window.codingPage && window.codingPage.started);
  return { page, threw, started };
}

async function itCarriesTheSimulatorAlone(page, engine) {
  check(await page.title() === 'Simulator', `${engine}: the page is titled "${await page.title()}"`);
  const heading = await page.locator('header h1').textContent();
  check(heading === 'Simulator', `${engine}: the page is headed "${heading}"`);
  const tabs = await page.locator('nav [data-tab]').evaluateAll((all) => all.map((tab) => tab.dataset.tab));
  check(tabs.join(',') === 'simulate,launch,feeder', `${engine}: the page offers the tabs ${tabs.join(', ')}`);
  check(await page.locator('#edit, #editor, #files, #git, #who, #robot, #adb').count() === 0,
      `${engine}: the page still carries the editor, the branch panel or who is signed in`);
  check(await page.locator('#simulate').isVisible(), `${engine}: the page does not open on Simulate`);
  check(page.url().endsWith('#simulate'), `${engine}: the page is at ${page.url()}, not #simulate`);
}

async function aCompileErrorSaysWhereAndAFixIsListed(page, engine) {
  check(await untilTrue(page, () => /Plans\.java:161/.test(document.getElementById('msg').textContent)),
      `${engine}: a checkout that does not compile shows "${await page.locator('#msg').textContent()}", `
      + 'not the file and line');
  compiles = true;
  await page.locator('nav [data-tab="simulate"]').click({ timeout: 5_000 });
  check(await untilTrue(page, (name) => document.getElementById('opmodes').textContent.includes(name), OPMODE.name),
      `${engine}: once the checkout compiles the Simulate tab lists no op modes`);
  check(await page.locator('#msg').textContent() === '', `${engine}: the compile error is still shown once fixed`);
  compiles = false;
}

async function aRunIsAskedForAndShown(page, engine) {
  const before = RUNS.length;
  await page.locator('#opmodes button.run').first().click({ timeout: 5_000 });
  check(await untilTrue(page, () => /^\/sim\/runs\/\d+\/$/.test(
      new URL(document.getElementById('stage').src, location.href).pathname)),
      `${engine}: pressing Run does not show the run's live view`);
  check(RUNS.slice(before).join(',') === OPMODE.name,
      `${engine}: pressing Run asked for ${RUNS.slice(before).join(', ') || 'nothing'}`);
}

async function theLaunchAndFeederSimsSetOutTheirSettings(page, engine) {
  await page.locator('nav [data-tab="launch"]').click({ timeout: 5_000 });
  check(await untilTrue(page, (n) => document.querySelectorAll('#launch details.launch-group').length === n,
      LAUNCH.settings.groups.length), `${engine}: the Launch Sim tab sets out no settings`);
  await page.locator('nav [data-tab="feeder"]').click({ timeout: 5_000 });
  check(await untilTrue(page, (n) => document.querySelectorAll('#feeder details.launch-group').length === n,
      FEEDER.settings.groups.length), `${engine}: the Feeder Sim tab sets out no settings`);
}

function itAskedForNothingButTheSimulator(engine, from) {
  const itsOwn = (asked) => asked === '/' || asked === '/favicon.ico' || asked.startsWith('/sim/');
  const elsewhere = [...new Set(ASKED.slice(from).filter((asked) => !itsOwn(asked)))];
  check(elsewhere.length === 0, `${engine}: the page asked for ${elsewhere.join(', ')}, which the simulator does `
      + 'not serve');
}

function nothingBroke(page, threw, started, engine) {
  check(started, `${engine}: the page never said it had started`);
  check(threw.length === 0, `${engine}: the page threw ${threw.join('; ')}`);
  return page.evaluate(() => window.codingPage.broke).then((broke) => check(broke.length === 0,
      `${engine}: parts broke: ${JSON.stringify(broke)}`));
}

async function main() {
  const server = await serve();
  const base = `http://127.0.0.1:${server.address().port}`;
  const browser = await chrome();
  try {
    for (const [engine, shaped] of [['this engine', null], ['an older iPad', asAnOlderIpad]]) {
      const from = ASKED.length;
      const { page, threw, started } = await opened(browser, base, shaped);
      await itCarriesTheSimulatorAlone(page, engine);
      await aCompileErrorSaysWhereAndAFixIsListed(page, engine);
      await aRunIsAskedForAndShown(page, engine);
      await theLaunchAndFeederSimsSetOutTheirSettings(page, engine);
      itAskedForNothingButTheSimulator(engine, from);
      await nothingBroke(page, threw, started, engine);
      await page.close();
    }
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the dashboard as the local simulator serves it did not start as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log('the dashboard as the local simulator serves it is titled Simulator, offers Simulate, Launch Sim '
      + 'and Feeder Sim alone and opens on Simulate even when asked for #edit, shows a compile error with its '
      + 'file and line and lists the op modes once it is fixed, asks for a run and shows its live view, sets out '
      + 'the Launch Sim\'s and the Feeder Sim\'s settings, and asks for nothing outside /sim, on this engine and '
      + 'on one shaped like an older iPad\'s.');
}

await main();
