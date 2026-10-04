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

const GROUPS = [
  { name: 'noise', label: 'Noise', says: 'What a seed draws a run\'s robot from.' },
  { name: 'mechanisms', label: 'Mechanisms', says: 'Guesses until measured on the robot.' }
];

const BUILT = [
  { name: 'motor_spread', group: 'noise', label: 'Motor spread', unit: 'of the tuned value',
    says: 'How far each drive motor may be drawn from its tuning.', least: 0, most: 0.5, byDefault: 0.1 },
  { name: 'hiccup_chance', group: 'noise', label: 'Hiccup chance', unit: 'per loop',
    says: 'The chance that a loop is a hiccup.', least: 0, most: 1, byDefault: 0.02 },
  { name: 'launch_throw', group: 'mechanisms', label: 'Launcher throw', unit: 'in/s per tick/s',
    says: 'How fast a ball leaves the launcher.', least: 0.01, most: 1, byDefault: 0.189 }
];

let set = { launch_throw: 0.25 };

const PUTS = [];

function constantsListed() {
  return {
    groups: GROUPS,
    constants: BUILT.map((c) => ({ ...c, value: c.name in set ? set[c.name] : c.byDefault }))
  };
}

function constantsAsked(request, response) {
  let body = '';
  request.on('data', (chunk) => { body += chunk; });
  request.on('end', () => {
    PUTS.push(body);
    const asked = JSON.parse(body);
    if (asked.launch_throw > 1) {
      response.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
      response.end('Launcher throw is from 0.01 to 1.0 in/s per tick/s, not ' + asked.launch_throw.toFixed(1));
      return;
    }
    set = asked;
    answer(response, constantsListed());
  });
}

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
    } else if (request.method === 'GET' && url.pathname === '/admin/constants') {
      answer(response, constantsListed());
    } else if (request.method === 'PUT' && url.pathname === '/admin/constants') {
      constantsAsked(request, response);
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
    await page.waitForFunction(() => document.querySelectorAll('#constants input').length === 3, null,
        { timeout: 10_000 }).catch(() => {});
    const panel = await page.locator('#constants').innerText();
    for (const saying of ['Noise', 'Mechanisms', 'Motor spread', 'Hiccup chance', 'Launcher throw', 'in/s per tick/s',
      'built 0.189']) {
      check(panel.includes(saying), `the constants panel does not say ${saying}: ${JSON.stringify(panel)}`);
    }
    const throwInput = page.locator('#constants input[data-name="launch_throw"]');
    check(await throwInput.inputValue() === '0.25', `the launcher throw shows ${await throwInput.inputValue()}, not 0.25`);
    const changedRows = await page.locator('#constants .constant.changed').allInnerTexts();
    check(changedRows.length === 1 && changedRows[0].includes('Launcher throw'),
        `the constants marked as changed are ${JSON.stringify(changedRows)} rather than the launcher throw alone`);

    await page.fill('#constants input[data-name="hiccup_chance"]', '0');
    check(await page.locator('#constants .constant.changed').count() === 2,
        'a constant typed away from how it was built is not marked as changed');
    await Promise.all([
      page.waitForResponse((r) => r.url().endsWith('/admin/constants') && r.request().method() === 'PUT',
          { timeout: 10_000 }),
      page.click('#save-constants')
    ]);
    check(JSON.stringify(JSON.parse(PUTS[PUTS.length - 1])) === JSON.stringify({ hiccup_chance: 0, launch_throw: 0.25 }),
        `Save sent ${PUTS[PUTS.length - 1]} rather than every constant that differs from how it was built`);
    await page.waitForFunction(() => document.getElementById('constants-said').textContent.length > 0, null,
        { timeout: 10_000 }).catch(() => {});
    check((await page.locator('#constants-said').innerText()).includes('next run'),
        'saving does not say when the constants take effect');

    await page.fill('#constants input[data-name="launch_throw"]', '5');
    await Promise.all([
      page.waitForResponse((r) => r.url().endsWith('/admin/constants') && r.request().method() === 'PUT',
          { timeout: 10_000 }),
      page.click('#save-constants')
    ]);
    await page.waitForFunction(() => document.getElementById('constants-said').textContent.includes('from 0.01'), null,
        { timeout: 10_000 }).catch(() => {});
    check((await page.locator('#constants-said').innerText()).includes('Launcher throw is from 0.01 to 1.0'),
        `a refused set does not show why: ${await page.locator('#constants-said').innerText()}`);

    await Promise.all([
      page.waitForResponse((r) => r.url().endsWith('/admin/constants') && r.request().method() === 'PUT',
          { timeout: 10_000 }),
      page.click('#reset-constants')
    ]);
    check(PUTS[PUTS.length - 1] === '{}', `putting them back as built sent ${PUTS[PUTS.length - 1]} rather than {}`);
    await page.waitForFunction(() => document.querySelectorAll('#constants .constant.changed').length === 0, null,
        { timeout: 10_000 }).catch(() => {});
    check(await page.locator('#constants .constant.changed').count() === 0,
        'after putting them back as built, some are still marked as changed');
    check(await page.locator('#constants input[data-name="launch_throw"]').inputValue() === '0.189',
        'after putting them back as built, the launcher throw is not 0.189');

    check(threw.length === 0, `the page threw ${threw.join('; ')}`);
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    await browser.close();
    server.close();
  }

  if (wrong.length) {
    console.error('the coding server\'s admin page fell short:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log('the admin page says which robot each user is on, picks files for one robot at a time, '
      + 'shows each robot\'s own package as always editable and never offers it, '
      + 'deletes a user from the robot they are on, '
      + 'and shows the simulation constants, saves those that differ from how they were built, '
      + 'says why a set is refused, and puts them back as built.');
}

main();
