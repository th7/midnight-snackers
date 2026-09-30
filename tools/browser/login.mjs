import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { chrome, sim } from './bench.mjs';

const LOGINS_ASKED = [];

function answer(response, body) {
  response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
  response.end(JSON.stringify(body));
}

function serve() {
  const server = http.createServer((request, response) => {
    const url = new URL(request.url, 'http://x');
    if (request.method === 'POST' && url.pathname === '/login') {
      const asked = { username: url.searchParams.get('username'), robot: url.searchParams.get('robot') };
      LOGINS_ASKED.push(asked);
      answer(response, { state: 'pending', username: asked.username, robot: asked.robot });
      return;
    }
    if (url.pathname === '/me') {
      const last = LOGINS_ASKED[LOGINS_ASKED.length - 1];
      answer(response, last ? { state: 'pending', username: last.username, robot: last.robot } : { state: 'none' });
      return;
    }
    if (url.pathname === '/') {
      response.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      response.end(fs.readFileSync(path.join(sim, 'login.html')));
      return;
    }
    response.writeHead(404).end('no route: ' + url.pathname);
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

const wrong = [];

function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

async function main() {
  const server = await serve();
  const base = `http://127.0.0.1:${server.address().port}`;
  const browser = await chrome();
  try {
    const page = await browser.newPage();
    const threw = [];
    page.on('pageerror', (e) => threw.push(String((e && e.message) || e)));
    await page.goto(base + '/', { waitUntil: 'load', timeout: 60_000 });

    for (const name of ['Reginald', 'Nugget']) {
      check(await page.getByLabel(name, { exact: true }).isVisible(), `the login page offers no ${name}`);
    }
    await page.fill('#username', 'mia');
    await page.click('#login button[type="submit"]');
    check(!(await page.evaluate(() => document.getElementById('login').checkValidity())),
        'the form would go without a robot picked');
    check(LOGINS_ASKED.length === 0,
        `a login went without a robot picked: ${JSON.stringify(LOGINS_ASKED)}`);

    await page.getByLabel('Nugget', { exact: true }).check();
    await page.click('#login button[type="submit"]');
    await page.waitForFunction(() => !document.getElementById('waiting').hidden, null, { timeout: 10_000 })
        .catch(() => {});
    check(await page.locator('#waiting').isVisible(), 'asking to join with a robot picked did not wait for the admin');
    check(JSON.stringify(LOGINS_ASKED) === JSON.stringify([{ username: 'mia', robot: 'nugget' }]),
        `the login asked for ${JSON.stringify(LOGINS_ASKED)} rather than mia on nugget`);
    check(threw.length === 0, `the page threw ${threw.join('; ')}`);
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the coding server\'s login page did not ask as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log('the login page will not ask to join until a robot is picked, and asks for the one that was.');
}

main();
