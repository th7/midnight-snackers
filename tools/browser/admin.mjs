import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { chrome, sim } from './bench.mjs';

function ownBy(robot) {
  return ['main', 'test'].map((set) => 'TeamCode/src/' + set + '/java/org/firstinspires/ftc/' + robot + '/');
}

const ROBOTS = [
  { robot: 'reginald', name: 'Reginald', develop: 'develop', own: ownBy('reginald') },
  { robot: 'nugget', name: 'Nugget', develop: 'nugget-develop', own: ownBy('nugget') }
];

const REGINALDS = 'TeamCode/src/main/java/org/firstinspires/ftc/reginald/Plans.java';
const NUGGETS = 'TeamCode/src/main/java/org/firstinspires/ftc/nugget/TankDrive.java';
const SHARED = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/planrunner/Plan.java';

const TREE = {
  dir: 'TeamCode/src/main/java/org/firstinspires/ftc',
  entries: [
    { name: 'nugget', type: 'dir', path: 'TeamCode/src/main/java/org/firstinspires/ftc/nugget' },
    { name: 'Plans.java', type: 'file', path: REGINALDS },
    { name: 'TankDrive.java', type: 'file', path: NUGGETS },
    { name: 'Plan.java', type: 'file', path: SHARED }
  ]
};

const STATUS = { branch: '', develop: '', changed: [], ahead: 0, behind: 0, head: '0123456', pushable: false };

function userOn(robot, branch) {
  return {
    username: 'ada',
    robot,
    worktree: '/state/worktrees/' + branch,
    branch,
    status: { ...STATUS, branch, develop: ROBOTS.find((r) => r.robot === robot).develop },
    statusError: null,
    deletable: true,
    lastMerge: null,
    sessions: [{ id: robot === 'reginald' ? 1 : 2, address: '10.0.0.5', state: 'approved', ageSeconds: 5, file: null }]
  };
}

const USERS = { users: [userOn('reginald', 'coding/ada'), userOn('nugget', 'nugget/ada')] };

const FILES = {
  reginald: { files: [{ path: 'TeamCode/Plans.java' }] },
  nugget: { files: [{ path: 'TeamCode/Nugget.java' }] }
};

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
      answer(response, { userPort: 21986, root: '/project', worktreesDir: '/state/worktrees', addresses: [],
                         robots: ROBOTS });
    } else if (url.pathname === '/admin/users') {
      answer(response, USERS);
    } else if (url.pathname === '/admin/files') {
      answer(response, FILES[url.searchParams.get('robot')] || { files: [] });
    } else if (url.pathname === '/admin/tree') {
      answer(response, TREE);
    } else if (url.pathname === '/admin/assets') {
      answer(response, { under: '/state/assets', fetched: {}, complete: false, drawing: 'the model committed for tests',
                         resolution: {}, defaultResolution: 'high', downloaded: false });
    } else if (request.method === 'POST' && url.pathname === '/admin/users/delete') {
      answer(response, { deleted: true });
    } else if (url.pathname === '/' || url.pathname === '/admin') {
      response.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      response.end(fs.readFileSync(path.join(sim, 'admin.html')));
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

    await page.waitForFunction(() => document.querySelectorAll('#users .user').length === 2, null,
        { timeout: 10_000 }).catch(() => {});
    const rows = await page.locator('#users .user .user-head').allInnerTexts();
    check(rows.length === 2, `the admin page lists ${rows.length} users rather than ada on each robot`);
    check(rows.some((row) => row.includes('Reginald')) && rows.some((row) => row.includes('Nugget')),
        `the users do not say which robot each is on: ${JSON.stringify(rows)}`);

    await page.waitForFunction(() => document.getElementById('editable-robot').options.length === 2, null,
        { timeout: 10_000 }).catch(() => {});
    const offered = await page.locator('#editable-robot option').allInnerTexts();
    check(JSON.stringify(offered) === JSON.stringify(['Reginald (develop)', 'Nugget (nugget-develop)']),
        `the editable files are offered for ${JSON.stringify(offered)}`);
    await page.waitForFunction(() => document.getElementById('editable').textContent.includes('Plans.java'), null,
        { timeout: 10_000 }).catch(() => {});
    check((await page.locator('#editable').innerText()).includes('Plans.java'),
        'the editable files shown first are not Reginald\'s');

    const reginaldsPanel = await page.locator('#editable').innerText();
    for (const own of ownBy('reginald')) {
      check(reginaldsPanel.includes(own), `Reginald's panel does not say ${own} is always editable: `
          + JSON.stringify(reginaldsPanel));
    }
    check(!reginaldsPanel.includes(ownBy('nugget')[0]), 'Reginald\'s panel shows Nugget\'s package');
    const alwaysRows = page.locator('#editable .row').filter({ hasText: ownBy('reginald')[0] });
    check(await alwaysRows.count() === 1, 'Reginald\'s own package is not one row of the panel');
    check(await alwaysRows.getByRole('button').count() === 0, 'Reginald\'s own package can be removed');
    check(await page.locator('#editable .row').filter({ hasText: 'Plans.java' }).getByRole('button',
        { name: 'Remove' }).count() === 1, 'a picked file cannot be removed');

    await page.waitForFunction(() => document.getElementById('tree').textContent.includes('TankDrive.java'), null,
        { timeout: 10_000 }).catch(() => {});
    const treeRow = (name) => page.locator('#tree .row').filter({ hasText: name });
    check(await treeRow('Plans.java').getByRole('button', { name: 'Add' }).count() === 0,
        'Reginald\'s own file is offered to Reginald as if it needed picking');
    check((await treeRow('Plans.java').innerText()).includes('always editable'),
        'Reginald\'s own file does not say it is always editable for Reginald');
    check(await treeRow('TankDrive.java').getByRole('button', { name: 'Add' }).count() === 0,
        'Nugget\'s own file is offered to Reginald');
    check((await treeRow('TankDrive.java').innerText()).includes('Nugget\'s'),
        'Nugget\'s own file does not say whose it is when picking for Reginald');
    check(await treeRow('Plan.java').getByRole('button', { name: 'Add' }).count() === 1,
        'a shared file cannot be picked for Reginald');
    check((await page.locator('#tree .row.dir').innerText()).includes('Nugget\'s'),
        'Nugget\'s package does not say whose it is');

    await page.selectOption('#editable-robot', 'nugget');
    await page.waitForFunction(() => document.getElementById('editable').textContent.includes('Nugget.java'), null,
        { timeout: 10_000 }).catch(() => {});
    const nuggets = await page.locator('#editable').innerText();
    check(nuggets.includes('Nugget.java') && !nuggets.includes('Plans.java'),
        `choosing Nugget shows ${JSON.stringify(nuggets)} rather than Nugget's files`);
    check(ASKED.includes('GET /admin/files?robot=nugget'), 'choosing Nugget never asked for Nugget\'s files');
    check(nuggets.includes(ownBy('nugget')[0]) && !nuggets.includes(ownBy('reginald')[0]),
        `choosing Nugget does not show Nugget's own package alone as always editable: ${JSON.stringify(nuggets)}`);
    check((await treeRow('TankDrive.java').innerText()).includes('always editable')
        && await treeRow('TankDrive.java').getByRole('button', { name: 'Add' }).count() === 0,
        'Nugget\'s own file is offered to Nugget as if it needed picking');
    check((await treeRow('Plans.java').innerText()).includes('Reginald\'s')
        && await treeRow('Plans.java').getByRole('button', { name: 'Add' }).count() === 0,
        'Reginald\'s own file is offered to Nugget');

    const nuggetsAda = page.locator('#users .user').filter({ hasText: 'Nugget' });
    const [deleting] = await Promise.all([
      page.waitForRequest((request) => request.url().includes('/admin/users/delete'), { timeout: 10_000 }),
      nuggetsAda.getByRole('button', { name: 'Delete', exact: true }).click()
    ]);
    const asked = new URL(deleting.url());
    check(asked.search === '?robot=nugget&username=ada',
        `deleting ada on Nugget asked ${asked.pathname + asked.search}`);
    check(threw.length === 0, `the page threw ${threw.join('; ')}`);
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the coding server\'s admin page did not keep the robots apart:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log('the admin page says which robot each user is on, picks files for one robot at a time, '
      + 'shows each robot\'s own package as always editable and never offers it, '
      + 'and deletes a user from the robot they are on.');
}

main();
