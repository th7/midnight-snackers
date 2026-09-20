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
  { path: 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Plans.java', editors: [] },
  { path: 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Drive.java', editors: ['ana'] }
];
const CONTENT = 'package org.firstinspires.ftc.teamcode;\n\npublic final class Plans {\n}\n';
const OPMODE = { name: 'BlueLeftAuto', kind: 'auto', group: 'Autonomous',
                 where: 'org.firstinspires.ftc.teamcode.BlueLeftAuto', seed: null };

// What the page asks the user listener for, answered as the server answers it.
const ANSWERS = {
  '/me': { state: 'approved', username: 'mia', branch: 'mia' },
  '/files': { files: FILES },
  '/git/status': { changed: [], behind: 0, ahead: 0, pushable: false, head: '0123456' },
  '/build': { available: true, ok: true, problems: [] },
  '/sim/catalog': [OPMODE],
  '/sim/status': { running: false, runs: [] }
};

function serve() {
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    const answer = Object.prototype.hasOwnProperty.call(ANSWERS, asked) ? ANSWERS[asked]
        : asked.startsWith('/files/') ? { path: asked.slice('/files/'.length), version: 'v1', content: CONTENT }
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
    await theEditorFollowsTheSystemTheme(current.page, 'this engine');
    await nothingBroke(current.page, current.threw, current.started, 'this engine');

    const older = await opened(browser, base, { engine: asAnOlderIpad, viewport: { width: 810, height: 1080 } });
    await itWorks(older.page, 'an older iPad');
    await theEditorFollowsTheSystemTheme(older.page, 'an older iPad');
    await nothingBroke(older.page, older.threw, older.started, 'an older iPad');

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
  console.log('the dashboard lists its files, opens one in the editor and reaches the Simulate tab, on this '
      + 'engine and on one shaped like an older iPad\'s, and says so on the page when the editor cannot start.');
}

await main();
