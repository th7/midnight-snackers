import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { asTheBenchServesIt } from './model.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const sim = path.join(here, '..', '..', 'TeamCode', 'src', 'test', 'resources',
                      'org', 'firstinspires', 'ftc', 'teamcode', 'sim');

const FIELD_IN = 141.17;
const ROBOT_IN = 18;
const WALL_IN = 12.2;
const NAME = 'ReplayCheckAuto';

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

function aRunToPlay(model) {
  const moved = model.moved;
  const places = (n) => moved.map((piece, i) => i === 0 ? [n * 6, n * 3, piece.radius] : piece.centre);
  return {
    name: NAME,
    kind: 'auto',
    live: false,
    outcome: 'done',
    ticks: [0, 1, 2].map((n) => ({
      t: n * 0.25,
      x: -30 + n * 12,
      y: -20 + n * 5,
      heading: n * 0.4,
      step: `${n}. drive`,
      powers: [1, 1, 1, 1],
      packets: [],
      pieces: places(n)
    }))
  };
}

function filled(template, model, run) {
  return template
      .replace('__TITLE__', NAME)
      .replace('__FIELD_IN__', String(FIELD_IN))
      .replace('__ROBOT_IN__', String(ROBOT_IN))
      .replace('__WALL_IN__', String(WALL_IN))
      .replace('__FIELD__', JSON.stringify(model))
      .replace('__DATA__', JSON.stringify(run));
}

async function main() {
  const template = fs.readFileSync(path.join(sim, 'replay.html'), 'utf8');
  const model = asTheBenchServesIt(JSON.parse(fs.readFileSync(path.join(sim, 'field.json'), 'utf8')), ROBOT_IN);
  const run = aRunToPlay(model);
  const page = filled(template, model, run);

  const left = page.match(/__[A-Z_]+__/g);
  check(left === null, `the page still holds ${left && [...new Set(left)].join(', ')}, so it was never filled in`);

  const where = path.join(fs.mkdtempSync(path.join(os.tmpdir(), 'replay-')), 'replay.html');
  fs.writeFileSync(where, page);

  const browser = await chromium.launch({ channel: 'chromium' });
  const open = await browser.newPage();
  open.on('pageerror', (e) => wrong.push(`the page threw: ${e.message}`));
  open.on('console', (m) => {
    if (m.type() === 'error') {
      wrong.push(`the page logged an error: ${m.text()}`);
    }
  });

  try {
    await open.goto('file://' + where);
    await open.waitForFunction(() => document.getElementById('title').textContent.length > 0, null,
                               { timeout: 30_000 });

    const read = () => open.evaluate(() => ({
      title: document.getElementById('title').textContent,
      outcome: document.getElementById('outcome').textContent,
      step: document.getElementById('step').textContent,
      pose: document.getElementById('pose').textContent,
      ticks: Number(document.getElementById('scrub').max)
    }));

    const first = await read();
    check(first.title.includes(NAME), `the page is titled "${first.title}", not the run's name`);
    check(/done/i.test(first.outcome), `the page says the run ended "${first.outcome}", not done`);
    check(first.ticks === run.ticks.length - 1,
          `the scrub covers ${first.ticks + 1} ticks of ${run.ticks.length}`);
    check(first.step === '0. drive', `the first tick's step reads "${first.step}"`);

    await open.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
    const shot = await open.locator('#field').screenshot();
    const drawn = await litPixels(browser, shot);
    check(drawn > 2000, `only ${drawn} pixels of the field are drawn; the page loaded but did not draw`);

    await open.evaluate((to) => {
      const scrub = document.getElementById('scrub');
      scrub.value = String(to);
      scrub.dispatchEvent(new Event('input', { bubbles: true }));
    }, run.ticks.length - 1);
    await open.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));

    const last = await read();
    const at = run.ticks[run.ticks.length - 1];
    check(last.step === '2. drive', `after scrubbing to the end the step reads "${last.step}"`);
    const reads = `x ${at.x.toFixed(1)} in, y ${at.y.toFixed(1)} in, `
        + `${(at.heading * 180 / Math.PI).toFixed(1)}\u00b0`;
    check(last.pose === reads, `the page shows the pose "${last.pose}", not the last tick's "${reads}"`);
    check(last.pose !== first.pose, 'the pose readout never changed, so scrubbing did nothing');
  } catch (stuck) {
    wrong.push(String(stuck && stuck.message));
  } finally {
    await browser.close();
  }

  if (wrong.length) {
    console.error('the replay page did not play as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  console.log(`the replay page plays ${run.ticks.length} ticks, in a real browser.`);
}

async function litPixels(browser, png) {
  const page = await browser.newPage();
  try {
    return await page.evaluate(async (bytes) => {
      const blob = new Blob([new Uint8Array(bytes)], { type: 'image/png' });
      const bitmap = await createImageBitmap(blob);
      const canvas = document.createElement('canvas');
      canvas.width = bitmap.width;
      canvas.height = bitmap.height;
      const context = canvas.getContext('2d');
      context.drawImage(bitmap, 0, 0);
      const data = context.getImageData(0, 0, bitmap.width, bitmap.height).data;
      let lit = 0;
      for (let i = 0; i < data.length; i += 4) {
        if (data[i] > 24 || data[i + 1] > 24 || data[i + 2] > 24) {
          lit++;
        }
      }
      return lit;
    }, Array.from(png));
  } finally {
    await page.close();
  }
}

await main();
