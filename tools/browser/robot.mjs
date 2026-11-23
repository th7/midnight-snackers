import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chrome, sim, TYPES } from './bench.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

const ANSWERED = JSON.parse(fs.readFileSync(path.join(here, 'robot-answers.json'), 'utf8'));
const MOVED_WHEEL = '{"constants":{"parallel_dead_wheel_y_in":-1.5}}';
const TOO_FLAT = '{"constants":{"parallel_dead_wheel_y_in":-1.5,"flattest_volts":14}}';
const REWIRED = '{"configuration":{"rightBack":{"encoder":"nothing"}}}';
const REVERSED = '{"directions":{"intake":"pushes_out"}}';
const KEPT_BY_AN_OLDER_BUILD = '{"constants":{"mass_kg":40,"launch_throw":1.2}}';
const WIDE = { width: 1280, height: 900 };
const PHONE = { width: 390, height: 844 };

const OTHER_ANSWERS = {
  '/sim/catalog': [],
  '/sim/status': { running: false, runs: [] },
  '/sim/launches': { launches: [] },
  '/sim/feeds': { feeds: [] }
};

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

const sent = [];
const asked = [];
let current = ANSWERED.listing;

function json(response, answer, status = 200) {
  response.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8' });
  response.end(JSON.stringify(answer));
}

function canonical(text) {
  const sorted = (value) => value === null || typeof value !== 'object' ? value
    : Object.keys(value).sort().reduce((into, key) => ({ ...into, [key]: sorted(value[key]) }), {});
  try {
    return JSON.stringify(sorted(JSON.parse(text)));
  } catch (e) {
    return text;
  }
}

const SAVES = { [canonical(MOVED_WHEEL)]: 'saved', '{}': 'reset', [canonical(REWIRED)]: 'rewired', [canonical(REVERSED)]: 'reversed' };

function servedTheRobotTab(request, response, path) {
  if (request.method === 'GET' && path === '/sim/robot') {
    json(response, current);
    return true;
  }
  if (request.method === 'PUT' && path === '/sim/robot') {
    let body = '';
    request.on('data', (chunk) => { body += chunk; });
    request.on('end', () => {
      sent.push(canonical(body));
      if (canonical(body) === canonical(TOO_FLAT)) {
        json(response, ANSWERED.refused, 400);
        return;
      }
      const saved = SAVES[canonical(body)];
      if (!saved) {
        response.writeHead(500).end('this check has no answer for ' + body);
        return;
      }
      current = ANSWERED[saved];
      json(response, current);
    });
    return true;
  }
  return false;
}

function serve() {
  const server = http.createServer((request, response) => {
    const path_ = decodeURIComponent(request.url.split('?')[0]);
    asked.push(path_);
    if (servedTheRobotTab(request, response, path_)) {
      return;
    }
    if (request.method === 'GET' && Object.prototype.hasOwnProperty.call(OTHER_ANSWERS, path_)) {
      json(response, OTHER_ANSWERS[path_]);
      return;
    }
    if (path_ !== '/') {
      response.writeHead(404).end('no route: ' + path_);
      return;
    }
    response.writeHead(200, { 'Content-Type': TYPES['.html'] });
    response.end(fs.readFileSync(path.join(sim, 'dashboard.html')));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

async function untilTrue(page, fn, arg, timeout = 15_000) {
  try {
    await page.waitForFunction(fn, arg, { timeout });
    return true;
  } catch (e) {
    return false;
  }
}

async function opened(browser, base, viewport = WIDE) {
  const page = await browser.newPage({ viewport });
  const threw = [];
  page.on('pageerror', (e) => threw.push(String((e && e.message) || e)));
  await page.goto(base + '/#robot', { waitUntil: 'load', timeout: 60_000 });
  check(await untilTrue(page, () => window.simulatorPage && window.simulatorPage.started),
        'the page never said it had started');
  check(await untilTrue(page, (n) => document.querySelectorAll('#robot-groups input').length === n,
                        ANSWERED.listing.constants.length),
        `the Robot tab sets out ${await page.locator('#robot-groups input').count()} values, not the bench's `
        + `${ANSWERED.listing.constants.length}`);
  return { page, threw };
}

const constantOf = (listing, name) => listing.constants.find((c) => c.name === name);
const valueField = (page, name) => page.locator(`#robot-value-${name}`);
const ruleOf = (page, name) => page.locator(`#robot [data-setting="${name}"] .rule`);

async function theTabComesFirstAndListsTheDevicesAndEveryValue(page) {
  const tabs = await page.locator('nav [data-tab]').evaluateAll((all) => all.map((tab) => tab.dataset.tab));
  check(tabs[0] === 'robot', `the first tab is ${tabs[0]}, not Robot`);
  check(await page.locator('#robot').isVisible(), 'opening #robot does not show the Robot tab');
  const devices = await page.locator('#robot-devices tbody tr').evaluateAll((rows) => rows.map((row) => ({
    device: row.dataset.device,
    selects: row.querySelectorAll('select').length
  })));
  const motors = ANSWERED.listing.configuration.filter((d) => d.kind === 'motor').length;
  check(devices.map((d) => d.device).join(',') === ANSWERED.listing.configuration.map((d) => d.device).join(','),
        `the configuration lists ${devices.map((d) => d.device).join(', ')}`);
  check(devices.filter((d) => d.selects === 2).length === motors,
        `${devices.filter((d) => d.selects === 2).length} devices offer a motor type and an encoder port, not the `
        + `${motors} motors`);
  const encoder = await page.locator('#robot-devices tr[data-device="rightBack"] select').nth(1).inputValue();
  check(encoder === 'parallel_dead_wheel', `rightBack's encoder port reads ${encoder}, not the parallel dead wheel`);
  const wheel = constantOf(ANSWERED.listing, 'parallel_dead_wheel_y_in');
  const says = await page.locator('#robot [data-setting="parallel_dead_wheel_y_in"] .says').textContent();
  check(says.startsWith(wheel.says) && says.includes('-9 to 9 in') && says.includes('-0.25 as committed'),
        `the parallel dead wheel says "${says}", not its sentence, its range and its committed value`);
  const groups = await page.locator('#robot-groups details').count();
  check(groups === ANSWERED.listing.groups.length,
        `the values are set out in ${groups} groups, not the bench's ${ANSWERED.listing.groups.length}`);
  const described = await page.locator('#robot-described').textContent();
  check(described.endsWith('as built'), `with nothing changed the tab says "${described}"`);
  check(await page.locator('#robot-notices').isHidden(), 'with nothing kept the tab shows notices');
  check(await page.locator('#robot-reset-all').isDisabled(), 'Reset all is offered with nothing to reset');
}

async function anEditIsSavedMarkedAndNamedForTheNextRun(page) {
  const from = sent.length;
  await valueField(page, 'parallel_dead_wheel_y_in').fill('-1.5');
  await valueField(page, 'parallel_dead_wheel_y_in').press('Tab');
  check(await untilTrue(page, (said) => document.getElementById('robot-described').textContent.endsWith(said),
                        ANSWERED.saved.described),
        `after the edit the tab says "${await page.locator('#robot-described').textContent()}"`);
  check(sent.slice(from).join(' ') === canonical(MOVED_WHEEL), `moving the wheel sent ${sent.slice(from).join(' ')}`);
  const marked = await page.locator('#robot [data-setting="parallel_dead_wheel_y_in"]').getAttribute('class');
  check(marked.split(' ').includes('changed'), 'the moved wheel is not marked as changed');
  const count = await page.locator('#robot details[data-group="dead_wheels"] summary .changes').textContent();
  check(count === '1 changed', `the dead wheels group says "${count}", not that one value changed`);
  check(await page.locator('#robot [data-setting="parallel_dead_wheel_y_in"] button.reset:not(.unneeded)').count()
        === 1, 'the moved wheel offers no reset');
  check(await page.locator('#robot-reset-all').isEnabled(), 'Reset all is not offered once a value changed');
}

async function aSavedEditOutlivesTheTab(browser, base) {
  const { page, threw } = await opened(browser, base);
  const shown = await valueField(page, 'parallel_dead_wheel_y_in').inputValue();
  check(Number(shown) === -1.5, `opened again the wheel shows ${shown}, not the saved -1.5`);
  check(threw.length === 0, `the page threw on opening again: ${threw.join('; ')}`);
  await page.close();
}

async function aRefusedSetShowsItsRuleAndSavesNothing(page) {
  const from = sent.length;
  await valueField(page, 'flattest_volts').fill('14');
  await valueField(page, 'flattest_volts').press('Tab');
  check(await untilTrue(page, () => {
    const rule = document.querySelector('#robot [data-setting="flattest_volts"] .rule');
    return !!rule && !rule.hidden;
  }), 'a flattest battery above the freshest shows no rule');
  check(sent.slice(from).join(' ') === canonical(TOO_FLAT), `the too-flat battery sent ${sent.slice(from).join(' ')}`);
  const rule = await ruleOf(page, 'flattest_volts').textContent();
  check(rule === ANSWERED.refused.rule, `beside the flattest battery it says "${rule}"`);
  const said = await page.locator('#robot-message').textContent();
  check(said.includes('Nothing was saved') && said.includes(ANSWERED.refused.rule),
        `the tab says "${said}", not that nothing was saved and why`);
  const described = await page.locator('#robot-described').textContent();
  check(described.endsWith(ANSWERED.saved.described), `after the refusal the tab says "${described}"`);
  await page.locator('#robot [data-setting="flattest_volts"] button.reset').click({ timeout: 5_000 });
  check(await untilTrue(page, () => document.querySelector('#robot [data-setting="flattest_volts"] .rule').hidden),
        'the rule stays once the flattest battery is reset');
  check(sent[sent.length - 1] === canonical(MOVED_WHEEL), `resetting the battery sent ${sent[sent.length - 1]}`);
}

async function aMotorIsRewiredAndResetAllPutsEverythingBack(page) {
  await page.locator('#robot [data-setting="parallel_dead_wheel_y_in"] button.reset').click({ timeout: 5_000 });
  check(await untilTrue(page, () => document.getElementById('robot-described').textContent.endsWith('as built')),
        'resetting the wheel does not make the next run as built');
  check(sent[sent.length - 1] === '{}', `resetting the wheel sent ${sent[sent.length - 1]}`);
  await page.locator('#robot-devices tr[data-device="rightBack"] select').nth(1).selectOption('nothing');
  check(await untilTrue(page, (said) => document.getElementById('robot-described').textContent.endsWith(said),
                        ANSWERED.rewired.described), 'rewiring rightBack is not what the next run is made on');
  check(sent[sent.length - 1] === canonical(REWIRED), `rewiring rightBack sent ${sent[sent.length - 1]}`);
  const count = await page.locator('#robot details[data-group="configuration"] summary .changes').textContent();
  check(count === '1 changed', `the configuration says "${count}", not that one value changed`);
  await page.locator('#robot-reset-all').click({ timeout: 5_000 });
  check(await untilTrue(page, () => document.getElementById('robot-described').textContent.endsWith('as built')),
        'Reset all does not make the next run as built');
  check(sent[sent.length - 1] === '{}', `Reset all sent ${sent[sent.length - 1]}`);
  const encoder = await page.locator('#robot-devices tr[data-device="rightBack"] select').nth(1).inputValue();
  check(encoder === 'parallel_dead_wheel', `after Reset all rightBack's encoder reads ${encoder}`);
  check(await page.locator('#robot .changed').count() === 0, 'values are still marked changed after Reset all');
}

async function aMechanismIsReversedByWhatPositivePowerDoes(page) {
  const listed = ANSWERED.listing.directions;
  check(['left_front', 'right_front', 'left_back', 'right_back'].every((name) => listed.some((one) => one.name === name)),
        `the drive motors are not all among the directions: ${listed.map((one) => one.name).join(', ')}`);
  for (const one of listed) {
    const ways = await page.locator(`#robot [data-direction="${one.name}"] select option`).allTextContents();
    check(ways.join('|') === one.ways.map((way) => way.says).join('|'),
          `${one.label} offers ${JSON.stringify(ways)}, not what positive power does each way`);
  }
  const intake = listed.find((one) => one.name === 'intake');
  const select = page.locator('#robot [data-direction="intake"] select');
  const offered = await select.locator('option').allTextContents();
  check(offered.join('|') === intake.ways.map((way) => way.says).join('|'),
        `the intake offers ${JSON.stringify(offered)}, not what positive power does each way`);
  check(await select.inputValue() === intake.byDefault, `the intake starts as ${await select.inputValue()}`);
  await select.selectOption('pushes_out');
  check(await untilTrue(page, (said) => document.getElementById('robot-described').textContent.endsWith(said),
                        ANSWERED.reversed.described), 'reversing the intake is not what the next run is made on');
  check(sent[sent.length - 1] === canonical(REVERSED), `reversing the intake sent ${sent[sent.length - 1]}`);
  const count = await page.locator('#robot details[data-group="directions"] summary .changes').textContent();
  check(count === '1 changed', `the directions say "${count}", not that one changed`);
  await page.locator('#robot [data-direction="intake"] button.reset').click({ timeout: 5_000 });
  check(await untilTrue(page, () => document.getElementById('robot-described').textContent.endsWith('as built')),
        'putting the intake back does not make the next run as built');
  check(sent[sent.length - 1] === '{}', `putting the intake back sent ${sent[sent.length - 1]}`);
}

async function whatAnOlderBuildKeptIsShownAsTheTabRecalledIt(browser, base) {
  current = ANSWERED.recalled;
  const { page, threw } = await opened(browser, base);
  const notices = await page.locator('#robot-notices li').allTextContents();
  check(notices.join('|') === ANSWERED.recalled.notices.join('|'),
        `the tab shows the notices ${JSON.stringify(notices)}, not ${JSON.stringify(ANSWERED.recalled.notices)}`);
  check(ANSWERED.recalled.notices.length === 2, `the bench recalled ${KEPT_BY_AN_OLDER_BUILD} with `
        + `${ANSWERED.recalled.notices.length} notices, not one for each edit it could not keep`);
  const mass = await valueField(page, 'mass_kg').inputValue();
  check(Number(mass) === constantOf(ANSWERED.recalled, 'mass_kg').byDefault,
        `a mass past its range shows ${mass}, not its committed value`);
  check(threw.length === 0, `the page threw on recalling: ${threw.join('; ')}`);
  await page.close();
}

async function itFitsAPhone(browser, base) {
  current = ANSWERED.listing;
  const { page } = await opened(browser, base, PHONE);
  const wide = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
  check(wide <= 0, `on a phone the page scrolls ${wide}px sideways`);
  await page.close();
}

async function main() {
  const server = await serve();
  const base = `http://127.0.0.1:${server.address().port}`;
  let browser = null;
  const threw = [];
  try {
    browser = await chrome();
    const { page, threw: thrown } = await opened(browser, base);
    await theTabComesFirstAndListsTheDevicesAndEveryValue(page);
    await anEditIsSavedMarkedAndNamedForTheNextRun(page);
    await aSavedEditOutlivesTheTab(browser, base);
    await aRefusedSetShowsItsRuleAndSavesNothing(page);
    await aMotorIsRewiredAndResetAllPutsEverythingBack(page);
    await aMechanismIsReversedByWhatPositivePowerDoes(page);
    const broke = await page.evaluate(() => window.simulatorPage.broke);
    check(broke.length === 0, `the dashboard broke: ${JSON.stringify(broke)}`);
    threw.push(...thrown);
    await page.close();
    await whatAnOlderBuildKeptIsShownAsTheTabRecalledIt(browser, base);
    await itFitsAPhone(browser, base);
  } catch (stuck) {
    wrong.push(String((stuck && stuck.message) || stuck));
  } finally {
    if (browser) {
      await browser.close();
    }
    server.close();
  }
  check(threw.length === 0, `the page threw: ${threw.join('; ')}`);
  const elsewhere = [...new Set(asked.filter((one) => one !== '/' && one !== '/favicon.ico'
      && !one.startsWith('/sim/')))];
  check(elsewhere.length === 0, `the page asked for ${elsewhere.join(', ')}, which the simulator does not serve`);

  if (wrong.length) {
    console.error('the Robot tab did not keep the robot description as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log(`the Robot tab comes first, lists the ${ANSWERED.listing.configuration.length} configured devices and`
      + ` the ${ANSWERED.listing.constants.length} values with their ranges and sentences, saves a moved wheel and`
      + ' keeps it, refuses a flattest battery above the freshest saying why and saving nothing, rewires a motor,'
      + ' puts everything back with Reset all, offers each mechanism\'s direction, the drive motors\' among them, reverses the intake by what positive power does, shows what it could not keep from an older build\'s file, and fits a'
      + ' phone.');
}

await main();
