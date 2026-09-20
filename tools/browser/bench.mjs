import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { movedPieces } from './model.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

export const sim = path.join(here, '..', '..', 'TeamCode', 'src', 'test', 'resources',
                             'org', 'firstinspires', 'ftc', 'teamcode', 'sim');

export const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.glb': 'model/gltf-binary',
  '.png': 'image/png'
};

const ANYWHERE = [

  '--no-sandbox',

  '--disable-dev-shm-usage'
];

const IN_SOFTWARE = [

  '--enable-unsafe-swiftshader',
  '--use-gl=angle',
  '--use-angle=swiftshader'
];

export const FLAGS = ANYWHERE.concat(IN_SOFTWARE);

export function chrome(asked = {}) {
  return chromium.launch({
    ...(process.env.CHROME_BIN ? { executablePath: process.env.CHROME_BIN } : { channel: 'chromium' }),
    ...(asked.headed ? { headless: false } : {}),
    args: asked.gpu ? ANYWHERE : FLAGS
  });
}

export function benchModel() {
  const model = JSON.parse(fs.readFileSync(path.join(sim, 'field.json'), 'utf8'));
  model.robotIn = 18;
  model.moved = movedPieces(model);
  return model;
}

export function cannedRun(model) {
  const moved = model.moved;
  const blue = model.hives.find((h) => h.alliance === 'Blue');

  const places = (n) => moved.map((piece, i) => i === 0 ? [n * 6, n * 3, piece.radius] : piece.centre)
      .concat([n < 2 ? null : [18, -9, 26]]);
  return {
    outcome: 'done',
    ticks: [0, 1, 2, 3].map((n) => ({
      t: n * 0.25, x: -30 + n * 12, y: -20 + n * 5, heading: n * 0.4, step: 'drive',
      powers: [0, 0, 0, 0], packets: [], pieces: places(n),
      tilt: n === 2 && blue ? { Blue: blue.tilt - 25 } : undefined
    }))
  };
}

export function serve() {
  const model = benchModel();
  const run = cannedRun(model);
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    if (asked === '/model' || asked === '/runs/1/ticks') {
      response.writeHead(200, { 'Content-Type': 'application/json' });
      response.end(JSON.stringify(asked === '/model' ? model : run));
      return;
    }
    const relative = asked === '/field' || asked === '/' ? 'field.html'
        : asked.startsWith('/assets/') ? asked.slice('/assets/'.length)
        : null;
    const file = relative && path.resolve(sim, relative);
    if (!file || !file.startsWith(path.resolve(sim)) || !fs.existsSync(file)) {
      response.writeHead(404).end('no');
      return;
    }
    response.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
    response.end(fs.readFileSync(file));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}
