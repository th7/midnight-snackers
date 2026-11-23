import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chrome, importMapFor, sim, TYPES, WHILE_A_VIEW_DRAWS } from './bench.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

const ASSETS = '../assets/';
const TITLE = 'Spindexer Sim';
const PLACEHOLDERS = ['__ASSETS__', '__IMPORTMAP__', '__TITLE__'];
const SIMULATING_MS = 300;
const REFUSED_SILL = 10;
const SAME_PLACE_IN = 1e-6;
const SAME_DEGREES = 1e-6;
const WIDE = { width: 1280, height: 900 };

const ANSWERED = JSON.parse(fs.readFileSync(path.join(here, 'spindexer-answers.json'), 'utf8'));
const FED_ID = ANSWERED.fed.id;
const FRAMES = ANSWERED.frames.flatMap((chunk) => chunk.frames);
const THROWN = ANSWERED.feed.events.filter((e) => e.kind === 'thrown').length;
const END = FRAMES[FRAMES.length - 1].t;

const ANSWERS = {
  '/sim/catalog': [],
  '/sim/status': { running: false, runs: [] },
  '/sim/launches': { launches: [] }
};

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

const feedsAsked = [];
let fed = false;

function json(response, answer, status = 200) {
  response.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8' });
  response.end(JSON.stringify(answer));
}

function bodyOf(request, then) {
  let body = '';
  request.on('data', (chunk) => { body += chunk; });
  request.on('end', () => then(JSON.parse(body || '{}')));
}

function filled(template) {
  return template
      .replace('__IMPORTMAP__', importMapFor(ASSETS))
      .replace('__ASSETS__', JSON.stringify(ASSETS))
      .replace('__TITLE__', TITLE);
}

function servedTheSpindexerSim(request, response, asked, viewer) {
  if (request.method === 'GET' && asked === '/sim/feed/settings') {
    json(response, ANSWERED.settings);
    return true;
  }
  if (request.method === 'POST' && asked === '/sim/feed') {
    bodyOf(request, (body) => {
      feedsAsked.push(body);
      if (body.back_sill === REFUSED_SILL) {
        json(response, ANSWERED.refused, 400);
        return;
      }
      setTimeout(() => {
        fed = true;
        json(response, ANSWERED.fed);
      }, SIMULATING_MS);
    });
    return true;
  }
  if (request.method === 'GET' && asked === '/sim/feeds') {
    json(response, fed ? ANSWERED.listing : { feeds: [] });
    return true;
  }
  if (request.method === 'GET' && asked === '/sim/feeds/') {
    response.writeHead(200, { 'Content-Type': TYPES['.html'] });
    response.end(viewer);
    return true;
  }
  if (request.method === 'GET' && asked === `/sim/feeds/${FED_ID}/feed`) {
    json(response, ANSWERED.feed);
    return true;
  }
  const chunk = asked.match(new RegExp(`^/sim/feeds/${FED_ID}/frames/([0-9]+)$`));
  if (request.method === 'GET' && chunk && ANSWERED.frames[Number(chunk[1])]) {
    json(response, ANSWERED.frames[Number(chunk[1])]);
    return true;
  }
  return false;
}

function serve(viewer) {
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    if (servedTheSpindexerSim(request, response, asked, viewer)) {
      return;
    }
    if (request.method === 'GET' && Object.prototype.hasOwnProperty.call(ANSWERS, asked)) {
      json(response, ANSWERS[asked]);
      return;
    }
    const asset = asked.startsWith('/sim/assets/') ? asked.slice('/sim/assets/'.length) : null;
    const file = asked === '/' ? path.join(sim, 'dashboard.html')
        : asset ? path.resolve(sim, asset)
        : null;
    if (!file || !file.startsWith(path.resolve(sim)) || !fs.existsSync(file)) {
      response.writeHead(404).end('no route: ' + asked);
      return;
    }
    response.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
    response.end(fs.readFileSync(file));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

const theViewer = (page) => page.frame({ url: /\/sim\/feeds\// });

async function untilTrue(page, fn, arg, timeout = 15_000) {
  await page.waitForFunction(fn, arg, { timeout });
}

async function viewerOf(page) {
  await untilTrue(page, (id) => {
    const frame = document.getElementById('spindexer-stage');
    const at = frame && frame.contentWindow ? frame.contentWindow.location : null;
    return !!at && at.pathname === '/sim/feeds/' && at.hash === '#' + id;
  }, FED_ID);
  const viewer = theViewer(page);
  await viewer.waitForFunction((id) => window.spindexerPage && window.spindexerPage.settled
      && window.spindexerPage.shown === id, FED_ID, { timeout: 30_000 });
  return viewer;
}

async function theTabSetsOutTheDefaults(page) {
  await page.locator('nav [data-tab="spindexer"]').click(WHILE_A_VIEW_DRAWS);
  await untilTrue(page, (count) => document.querySelectorAll('#spindexer-groups input').length === count,
                  ANSWERED.settings.settings.length);
  const sill = ANSWERED.settings.settings.find((s) => s.name === 'back_sill');
  const shown = await page.locator('#spindexer-setting-back_sill').inputValue();
  check(Number(shown) === sill.value, `the back sill shows ${shown}, not the bench's ${sill.value}`);
}

async function aFeedRunsAndShowsItsPollenThrown(page) {
  await page.locator('#spindexer-go').click(WHILE_A_VIEW_DRAWS);
  const viewer = await viewerOf(page);
  check(feedsAsked.length === 1, `pressing Feed asked for ${feedsAsked.length} feeds, not 1`);
  await untilTrue(page, (id) => !!document.querySelector(`#feeds tbody tr[data-id="${id}"]`), FED_ID);
  const row = await page.locator(`#feeds tbody tr[data-id="${FED_ID}"]`).textContent();
  check(row.includes(`${THROWN} thrown`), `the feed's row says "${row}", not that ${THROWN} were thrown`);
  const title = await page.locator('#spindexer-title').textContent();
  check(title === `Feed ${FED_ID}`, `the tab is titled "${title}", not "Feed ${FED_ID}"`);
  const thrown = await viewer.evaluate(() => window.spindexerPage.thrown);
  check(thrown === THROWN, `the viewer says ${thrown} pollen were thrown, not ${THROWN}`);
  const summary = await viewer.locator('#summary').textContent();
  check(summary.includes(`${THROWN} of 2 thrown`), `the viewer's summary says "${summary}"`);
  return viewer;
}

async function itReplaysToTheEndAndBack(viewer) {
  const first = FRAMES[0];
  const atStart = await viewer.evaluate(() => window.spindexerPage.pollen);
  check(atStart.length === first.pollen.length && atStart.every((p, i) => p.visible
      && p.position.every((v, k) => Math.abs(v - first.pollen[i].at[k]) <= SAME_PLACE_IN)),
        `the viewer opened with its pollen at ${JSON.stringify(atStart)}, not the feed's first frame`);
  await viewer.locator('#speed').selectOption('2');
  await viewer.locator('#play').click(WHILE_A_VIEW_DRAWS);
  await viewer.waitForFunction((end) => !window.spindexerPage.playing && window.spindexerPage.time === end, END,
                               { timeout: 30_000 });
  const atEnd = await viewer.evaluate(() => window.spindexerPage.pollen.filter((p) => p.visible).length);
  check(atEnd === 0, `${atEnd} pollen are drawn at the end, after every one has left`);
  const middle = FRAMES[Math.floor(FRAMES.length / 2)];
  await viewer.evaluate((t) => window.spindexerPage.goTo(t), middle.t);
  const drawn = await viewer.evaluate(() => ({ spinner: window.spindexerPage.spinnerDegrees,
                                               ramp: window.spindexerPage.rampDegrees }));
  check(Math.abs(drawn.spinner - middle.spinner[0]) <= SAME_DEGREES
        && Math.abs(drawn.ramp - middle.ramp[0]) <= SAME_DEGREES,
        `at ${middle.t} s the spinner and ramp are drawn at ${JSON.stringify(drawn)}, not ${middle.spinner[0]}`
        + ` and ${middle.ramp[0]} degrees`);
  await viewer.locator('#play').press('Home');
  const back = await viewer.evaluate(() => ({ time: window.spindexerPage.time,
                                              shown: window.spindexerPage.pollen.filter((p) => p.visible).length }));
  check(back.time === 0 && back.shown === first.pollen.length,
        `back at the start the viewer is at ${back.time} s with ${back.shown} pollen drawn`);
}

async function aRefusedSettingShowsItsRuleBesideTheField(page) {
  const asked = feedsAsked.length;
  await page.locator('#spindexer-setting-back_sill').fill(String(REFUSED_SILL));
  await page.locator('#spindexer-go').click(WHILE_A_VIEW_DRAWS);
  await untilTrue(page, () => {
    const rule = document.querySelector('#spindexer [data-setting="back_sill"] .rule');
    return !!rule && !rule.hidden && rule.textContent.length > 0;
  });
  check(feedsAsked.length === asked + 1, 'pressing Feed with the sill at 10 did not ask the bench');
  const rule = await page.locator('#spindexer [data-setting="back_sill"] .rule').textContent();
  check(rule === ANSWERED.refused.rule, `beside the back sill it says "${rule}", not "${ANSWERED.refused.rule}"`);
  const invalid = await page.locator('#spindexer-setting-back_sill').getAttribute('aria-invalid');
  check(invalid === 'true', 'the refused back sill is not marked invalid');
  const others = await page.locator('#spindexer .rule:not([hidden])').count();
  check(others === 1, `${others} rules show, not only the back sill's`);
  await page.locator('#spindexer-setting-back_sill').fill('30');
  const after = await page.locator('#spindexer [data-setting="back_sill"] .rule').isHidden();
  check(after, 'the rule stays beside the back sill after it is changed');
}

async function main() {
  const template = fs.readFileSync(path.join(sim, 'spindexer.html'), 'utf8');
  const holes = [...new Set(template.match(/__[A-Z_]+__/g) || [])].sort();
  check(holes.join(',') === PLACEHOLDERS.join(','),
        `the page holds the placeholders ${holes.join(', ')}, not the ones the bench fills: ${PLACEHOLDERS.join(', ')}`);
  const viewer = filled(template);
  check(THROWN === 2, `the bench's default feed threw ${THROWN} pollen, which this check cannot show as two`);

  const server = await serve(viewer);
  const base = `http://127.0.0.1:${server.address().port}`;
  let browser = null;
  const threw = [];
  try {
    browser = await chrome();
    const page = await browser.newPage({ viewport: WIDE });
    page.on('pageerror', (e) => threw.push(String((e && e.message) || e)));
    await page.goto(base + '/', { waitUntil: 'load', timeout: 60_000 });
    await untilTrue(page, () => window.simulatorPage && window.simulatorPage.started);
    await theTabSetsOutTheDefaults(page);
    const shown = await aFeedRunsAndShowsItsPollenThrown(page);
    await itReplaysToTheEndAndBack(shown);
    const problem = await shown.evaluate(() => window.spindexerPage.problem);
    check(problem === '', `the viewer says "${problem}"`);
    await aRefusedSettingShowsItsRuleBesideTheField(page);
    const broke = await page.evaluate(() => window.simulatorPage.broke);
    check(broke.length === 0, `the dashboard broke: ${JSON.stringify(broke)}`);
  } catch (stuck) {
    wrong.push(String(stuck && stuck.message));
  } finally {
    if (browser) {
      await browser.close();
    }
    server.close();
  }
  check(threw.length === 0, `the page threw: ${threw.join('; ')}`);

  if (wrong.length) {
    console.error('the Spindexer Sim did not run and replay its feed as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log(`the Spindexer Sim tab sets out ${ANSWERED.settings.settings.length} settings, runs the bench's default`
      + ` feed, shows its ${THROWN} pollen thrown, replays its ${FRAMES.length} frames to ${END} s and back, and`
      + ` shows "${ANSWERED.refused.rule}" beside the back sill.`);
}

await main();
