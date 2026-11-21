import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { chrome, gateway } from './bench.mjs';

const USERS = {
  users: [
    {
      username: 'ada',
      relayed: true,
      sessions: [
        { id: 1, address: '10.0.0.5', state: 'approved', ageSeconds: 5 },
        { id: 2, address: '10.0.0.6', state: 'approved', ageSeconds: 9 }
      ]
    },
    {
      username: 'bob',
      relayed: false,
      sessions: [
        { id: 3, address: '10.0.0.7', state: 'approved', ageSeconds: 20 },
        { id: 4, address: '10.0.0.7', state: 'pending', ageSeconds: 2 }
      ]
    }
  ],
  relay: { port: 5555, hub: '192.168.43.1:5555', hubState: 'unreachable', hubWhy: 'Connect timed out',
           hubCheckedSecondsAgo: 4, connect: ['adb connect 10.0.0.2:5555'] }
};

const ASSETS = { under: '/state/assets', fetched: {}, complete: false, resolution: {}, downloaded: true };

const NO_LONGER_ASKED = ['/admin/files', '/admin/tree', '/admin/constants', '/admin/users/robots',
  '/admin/users/pull', '/admin/editable'];

const ASKED = [];

function answer(response, body) {
  response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
  response.end(JSON.stringify(body));
}

function serve() {
  const server = http.createServer((request, response) => {
    const url = new URL(request.url, 'http://x');
    ASKED.push(request.method + ' ' + url.pathname + url.search);
    if (url.pathname === '/admin/info') {
      answer(response, { userPort: 21986, addresses: ['10.0.0.2'] });
    } else if (url.pathname === '/admin/users') {
      answer(response, USERS);
    } else if (url.pathname === '/admin/assets') {
      answer(response, ASSETS);
    } else if (request.method === 'POST' && url.pathname === '/admin/assets/build') {
      answer(response, ASSETS);
    } else if (request.method === 'POST' && url.pathname === '/admin/users/delete') {
      answer(response, { deleted: true, username: url.searchParams.get('username'), logins: 2 });
    } else if (request.method === 'POST' && /^\/admin\/logins\/\d+\/(approve|deny|revoke)$/.test(url.pathname)) {
      answer(response, { ok: true });
    } else if (url.pathname === '/' || url.pathname === '/admin') {
      response.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      response.end(fs.readFileSync(path.join(gateway, 'admin.html')));
    } else {
      response.writeHead(404).end('no route: ' + url.pathname);
    }
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
    await page.goto(base + '/admin', { waitUntil: 'load', timeout: 60_000 });

    check(await page.title() === 'Robot Gateway Admin', `the admin page is titled "${await page.title()}"`);
    await page.waitForFunction(() => document.querySelectorAll('#users .user').length === 2, null,
        { timeout: 10_000 }).catch(() => {});
    const rows = await page.locator('#users .user .user-head').allInnerTexts();
    check(rows.length === 2, `the admin page lists ${rows.length} rows rather than one for ada and one for bob`);
    const ada = page.locator('#users .user').filter({ hasText: 'ada' });
    const bob = page.locator('#users .user').filter({ hasText: 'bob' });
    check((await ada.locator('.user-head').innerText()).includes('relaying'),
        'ada, whose adb connection is relayed, is not shown relaying');
    check(!(await bob.locator('.user-head').innerText()).includes('relaying'),
        'bob, who has no relayed connection, is shown relaying');
    await page.waitForFunction(() => document.getElementById('relay').textContent.includes('192.168.43.1'), null,
        { timeout: 10_000 }).catch(() => {});
    const relay = await page.locator('#relay').innerText();
    for (const saying of ['192.168.43.1:5555', 'unreachable', 'Connect timed out', 'checked 4s ago',
      'adb connect 10.0.0.2:5555']) {
      check(relay.includes(saying), `the relay panel does not say ${saying}: ${JSON.stringify(relay)}`);
    }
    await page.waitForFunction(() => document.getElementById('urls').textContent.includes('10.0.0.2'), null,
        { timeout: 10_000 }).catch(() => {});
    check((await page.locator('#urls').innerText()).includes('http://10.0.0.2:21986/'),
        'the admin page does not say what teammates open');

    const bobsLogins = await bob.locator('.sessions .row').allInnerTexts();
    check(bobsLogins.length === 2 && bobsLogins.every((login) => login.includes('10.0.0.7')),
        `bob's logins do not say where each came from: ${JSON.stringify(bobsLogins)}`);
    const heads = [...rows, ...bobsLogins].join('\n');
    check(!/robot|reginald|for /i.test(heads), `the admin page names a robot for a user or a login: ${heads}`);

    const [approving] = await Promise.all([
      page.waitForRequest((request) => request.url().includes('/admin/logins/'), { timeout: 10_000 }),
      bob.locator('.sessions .row').filter({ hasText: 'login 4' }).getByRole('button', { name: 'Approve' }).click()
    ]);
    check(new URL(approving.url()).pathname === '/admin/logins/4/approve',
        `approving bob's login asked ${new URL(approving.url()).pathname}`);

    const [deleting] = await Promise.all([
      page.waitForRequest((request) => request.url().includes('/admin/users/delete'), { timeout: 10_000 }),
      ada.getByRole('button', { name: 'Delete', exact: true }).click()
    ]);
    const asked = new URL(deleting.url());
    check(asked.search === '?username=ada', `deleting ada asked ${asked.pathname + asked.search}`);

    await page.waitForFunction(() => !document.getElementById('build-assets').disabled, null,
        { timeout: 10_000 }).catch(() => {});
    const [building] = await Promise.all([
      page.waitForRequest((request) => request.url().includes('/admin/assets/build'), { timeout: 10_000 }),
      page.click('#build-assets')
    ]);
    const built = new URL(building.url());
    check(built.search === '?resolution=high', `building the field models asked ${built.pathname + built.search}`);

    check(await page.locator('#constants, #editable, #tree, #editable-robot, .work').count() === 0,
        'the admin page still carries the simulation constants, the file picker or the users\' work');
    const stale = ASKED.filter((one) => NO_LONGER_ASKED.some((gone) => one.split(' ')[1].startsWith(gone)));
    check(stale.length === 0, `the admin page asked for ${stale.join(', ')}, which the gateway no longer serves`);
    check(threw.length === 0, `the page threw ${threw.join('; ')}`);
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the robot gateway\'s admin page fell short:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log('the admin page lists each user\'s logins and where each came from, and no robot, approves a login, deletes a '
      + 'user, says whose adb connection is relayed, whether the hub could be reached and what teammates open, '
      + 'builds the field models at the resolution picked, and asks for nothing the gateway no longer serves.');
}

main();
