import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chrome, importMapFor, sim, TYPES } from './bench.mjs';
import { movedPieces } from './model.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));

const ASSETS = '../assets/';
const TITLE = 'Launch Sim';
const PLACEHOLDERS = ['__ASSETS__', '__DATA__', '__FIELD__', '__IMPORTMAP__', '__TITLE__'];
const GRAVITY_IN_PER_S2 = 386.09;
const FRAME_EVERY_S = 0.004;
const POLLEN = { radius: 1.4, colour: '#ffef3f' };
const WALL_THICKNESS_IN = 1;
const METRES_PER_INCH = 0.0254;
const RPM_PER_RAD_S = 60 / (2 * Math.PI);
const WIDE = { width: 1100, height: 900 };
const NARROW = { width: 480, height: 680 };
const ONE_MS = 0.001;

const UNBUILT = /field-(high|medium)\.glb/;
const BARE_404 = /Failed to load resource.*404/;
const MODEL = /\.glb(\?|$)/;

const wrong = [];
function check(ok, saying) {
  if (!ok) {
    wrong.push(saying);
  }
}

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => scale(a, 1 / norm(a));
const centroid = (points) => scale(points.reduce(add, [0, 0, 0]), 1 / points.length);
const rounded = (v, digits) => Math.round(v * 10 ** digits) / 10 ** digits;
const roundedAll = (a, digits) => a.map((v) => rounded(v, digits));
const near = (a, b, within) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) <= within);

function quaternionTimes(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
  ];
}

function turned(q, spin, seconds) {
  const rate = norm(spin);
  if (!(rate > 0) || seconds === 0) {
    return q;
  }
  const half = rate * seconds / 2;
  const s = Math.sin(half) / rate;
  const out = quaternionTimes([Math.cos(half), spin[0] * s, spin[1] * s, spin[2] * s], q);
  const length = Math.hypot(...out);
  return out.map((v) => v / length);
}

function crossingTime(f, from, to) {
  let lo = from;
  let hi = to;
  for (let i = 0; i < 60; i++) {
    const middle = (lo + hi) / 2;
    if (f(middle) > 0) {
      lo = middle;
    } else {
      hi = middle;
    }
  }
  return lo;
}

function firstFallBelow(f, from, to) {
  for (let t = from; t <= to; t += ONE_MS) {
    if (f(t) <= 0) {
      return crossingTime(f, Math.max(from, t - ONE_MS), t);
    }
  }
  throw new Error('the canned shot never reaches what it was built to reach');
}

function along(leg, from, tau) {
  if (leg.kind === 'fly') {
    return {
      p: [from.p[0] + from.v[0] * tau, from.p[1] + from.v[1] * tau,
          from.p[2] + from.v[2] * tau - GRAVITY_IN_PER_S2 * tau * tau / 2],
      v: [from.v[0], from.v[1], from.v[2] - GRAVITY_IN_PER_S2 * tau],
      w: from.w,
      q: turned(from.q, from.w, tau)
    };
  }
  if (leg.kind === 'roll') {
    const travelled = tau - tau * tau / (2 * leg.duration);
    const left = 1 - tau / leg.duration;
    return { p: add(from.p, scale(from.v, travelled)), v: scale(from.v, left), w: scale(from.w, left),
             q: turned(from.q, from.w, travelled) };
  }
  return { p: from.p, v: [0, 0, 0], w: [0, 0, 0], q: from.q };
}

function frameRow(t, state, phase) {
  return [rounded(t, 5), ...roundedAll(state.p, 4), ...roundedAll(state.v, 4), ...roundedAll(state.w, 4),
          ...roundedAll(state.q, 5), phase];
}

function flown(start, legs) {
  let state = { p: start.at, v: start.velocity, w: start.spin, q: [1, 0, 0, 0] };
  let t = 0;
  let k = 0;
  let lastT = -1;
  let phase = 'f';
  const frames = [];
  const impacts = [];
  const push = (time, s, saying) => {
    frames.push(frameRow(time, s, saying));
    lastT = time;
  };
  for (const leg of legs) {
    if (leg.kind === 'impact') {
      const after = { p: state.p, v: leg.velocity, w: leg.spin, q: state.q };
      const approach = -dot(state.v, leg.normal);
      const slipping = sub(state.v, scale(leg.normal, dot(state.v, leg.normal)));
      push(t, state, phase);
      push(t, after, leg.phase);
      impacts.push({
        t: rounded(t, 5), at: roundedAll(state.p, 4), normal: roundedAll(leg.normal, 4),
        against: leg.against, surface: leg.surface, approach: rounded(approach, 4),
        slip: rounded(norm(slipping), 4), restitution: leg.restitution, gripped: leg.gripped,
        before: { velocity: roundedAll(state.v, 4), spin: roundedAll(state.w, 4) },
        after: { velocity: roundedAll(after.v, 4), spin: roundedAll(after.w, 4) }
      });
      state = after;
      phase = leg.phase;
      continue;
    }
    while (k * FRAME_EVERY_S < t + leg.duration - 1e-9) {
      const time = k * FRAME_EVERY_S;
      if (time > lastT + 1e-9) {
        push(time, along(leg, state, time - t), leg.phase);
      }
      k++;
    }
    state = along(leg, state, leg.duration);
    t += leg.duration;
    phase = leg.phase;
  }
  if (t > lastT + 1e-9) {
    push(t, state, phase);
  }
  return { frames, impacts, end: t, state };
}

function tiltedAbout(hive, degrees) {
  const radians = degrees * Math.PI / 180;
  const c = Math.cos(radians);
  const s = Math.sin(radians);
  const [px, py, pz] = hive.pivot;
  return (p) => [px + p[0] * c - p[2] * s, py + p[1], pz + p[0] * s + p[2] * c];
}

function boxFaces(least, most) {
  const [x0, y0, z0] = least;
  const [x1, y1, z1] = most;
  return [
    [[x0, y0, z0], [x0, y1, z0], [x1, y1, z0], [x1, y0, z0]],
    [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
    [[x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]],
    [[x0, y1, z0], [x0, y1, z1], [x1, y1, z1], [x1, y1, z0]],
    [[x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0]],
    [[x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]]
  ];
}

function collidersOf(field, tilts) {
  const half = field.size / 2;
  const high = field.wallHeight;
  const thick = WALL_THICKNESS_IN;
  const ring = (points) => points.map((p) => roundedAll(p, 4));
  const out = [{ name: 'Floor', surface: 'floor', kind: 'floor', faces: [] }];
  const walls = [
    ['Audience wall', [-half - thick, -half - thick, 0], [-half, half + thick, high]],
    ['Far wall', [half, -half - thick, 0], [half + thick, half + thick, high]],
    ["Blue's wall", [-half - thick, -half - thick, 0], [half + thick, -half, high]],
    ["Red's wall", [-half - thick, half, 0], [half + thick, half + thick, high]]
  ];
  for (const [name, least, most] of walls) {
    out.push({ name, surface: 'wall', kind: 'solid', faces: boxFaces(least, most).map(ring) });
  }
  for (const element of field.elements) {
    out.push({
      name: element.name,
      surface: /^Flower/.test(element.group) ? 'flower' : 'frame',
      kind: 'solid',
      faces: element.faces.map((face) => ring(face.map((i) => element.vertices[i])))
    });
  }
  for (const hive of field.hives) {
    const at = tiltedAbout(hive, tilts[hive.alliance]);
    for (const part of hive.parts) {
      out.push({ name: part.name, surface: 'frame', kind: 'solid',
                 faces: part.faces.map((face) => ring(face.map((i) => at(part.vertices[i])))) });
    }
    for (const cell of hive.cells) {
      cell.walls.forEach((wall, i) => out.push(
          { name: `${cell.name} wall ${i + 1}`, surface: 'cell', kind: 'plate', faces: [ring(wall.map(at))] }));
      out.push({ name: `${cell.name} back`, surface: 'cell', kind: 'plate', faces: [ring(cell.back.map(at))] });
    }
  }
  return out;
}

function piecesOf(field, tipped) {
  return field.moved
      .filter((piece) => !(piece.cell && tipped.some((alliance) => piece.cell.startsWith(alliance))))
      .map((piece) => ({
        name: piece.name, kind: piece.kind, colour: piece.colour, radius: piece.radius,
        fixed: piece.flower !== undefined, start: piece.centre, track: []
      }));
}

function settingsOf(changed) {
  const all = [
    ['x', 'launcher', 'X', 'in', 55], ['y', 'launcher', 'Y', 'in', -12.76],
    ['height', 'launcher', 'Height', 'in', 14], ['yaw', 'launcher', 'Yaw', 'deg', 180],
    ['pitch', 'launcher', 'Pitch', 'deg', 70], ['wheel_one_rpm', 'launcher', 'Wheel one', 'rpm', 900],
    ['wheel_two_rpm', 'launcher', 'Wheel two', 'rpm', 1200], ['wind_x', 'air', 'Wind x', 'in/s', 0],
    ['wind_y', 'air', 'Wind y', 'in/s', 0], ['pieces', 'pollen', 'Pieces', '', 1]
  ];
  return all.map(([name, group, label, unit, byDefault]) => ({
    name, group, label, unit, byDefault, value: name in changed ? changed[name] : byDefault
  }));
}

function spinParameterOf(velocity, spin) {
  const speed = norm(velocity);
  return speed > 0 ? POLLEN.radius * norm(cross(spin, velocity)) / (speed * speed) : 0;
}

function shotJson(id, field, made) {
  const { start, legs, tilts, pieces, ending, kind, surfaces, crossingsOf, changed } = made;
  const flight = flown(start, legs);
  let apex = flight.frames[0];
  for (const frame of flight.frames) {
    if (frame[3] > apex[3]) {
      apex = frame;
    }
  }
  const floorHit = flight.impacts.find((impact) => impact.surface === 'floor');
  const last = flight.frames[flight.frames.length - 1];
  const seconds = 0.0123;
  return {
    id,
    at: 1760000000000 + id * 60000,
    pollen: POLLEN,
    settings: settingsOf(changed),
    launch: {
      at: start.at, velocity: roundedAll(start.velocity, 4), spin: roundedAll(start.spin, 4),
      speed: rounded(norm(start.velocity), 4), spinRpm: rounded(norm(start.spin) * RPM_PER_RAD_S, 4),
      spinKind: kind, spinParameter: rounded(spinParameterOf(start.velocity, start.spin), 4),
      surfaces: surfaces[0], after: surfaces[1]
    },
    air: { density: 1.2041 },
    tilts,
    colliders: collidersOf(field, tilts),
    frames: flight.frames,
    pieces: pieces(flight),
    impacts: flight.impacts,
    crossings: crossingsOf ? crossingsOf(flight) : [],
    apex: { t: apex[0], at: apex.slice(1, 4) },
    firstBounce: floorHit
      ? { t: floorHit.t, range: rounded(Math.hypot(floorHit.at[0] - start.at[0], floorHit.at[1] - start.at[1]), 4),
          at: floorHit.at }
      : null,
    ending: Object.assign({ t: last[0], at: last.slice(1, 4) }, ending),
    computed: { seconds, simulated: rounded(flight.end, 5), realTime: rounded(seconds / flight.end, 5),
                work: flight.frames.length * 40 }
  };
}

function aLobIntoTheBlueScoringCell(field) {
  const hive = field.hives.find((h) => h.alliance === 'Blue');
  const at = tiltedAbout(hive, hive.tilt);
  const cell = hive.cells.find((c) => c.side === 'Scoring');
  const mouth = cell.mouth.map(at);
  const back = cell.back.map(at);
  const middle = centroid(mouth.concat(back));
  const inward = (normal, onIt) => (dot(normal, sub(middle, onIt)) < 0 ? scale(normal, -1) : normal);
  const outward = unit(cross(sub(back[1], back[0]), sub(back[2], back[0])));
  const axis = dot(outward, sub(centroid(mouth), centroid(back))) < 0 ? scale(outward, -1) : outward;
  const backOffset = dot(axis, back[0]);
  const mouthOffset = dot(axis, mouth[0]);
  const walls = cell.walls.map((wall) => wall.map(at));
  const floorOfTheCell = walls
      .map((wall) => ({ wall, normal: inward(unit(cross(sub(wall[1], wall[0]), sub(wall[2], wall[0]))), wall[0]) }))
      .reduce((best, one) => (one.normal[2] > best.normal[2] ? one : best));
  const up = floorOfTheCell.normal;
  const upOffset = dot(up, floorOfTheCell.wall[0]);

  const from = [55, -8, 14];
  const backMiddle = centroid(back);
  const hit = add([backMiddle[0], from[1], backMiddle[2]], scale(axis, POLLEN.radius));
  const toHit = 0.75;
  const velocity = [(hit[0] - from[0]) / toHit, 0, (hit[2] - from[2]) / toHit + GRAVITY_IN_PER_S2 * toHit / 2];
  const spin = [0, 21.3, 0];
  const start = { at: from, velocity, spin };

  const firstLeg = { kind: 'fly', duration: toHit, phase: 'f' };
  const atHit = along(firstLeg, { p: from, v: velocity, w: spin, q: [1, 0, 0, 0] }, toHit);
  const intoBack = dot(atHit.v, axis);
  const alongBack = sub(atHit.v, scale(axis, intoBack));
  const offTheBack = add(scale(alongBack, 0.7), scale(axis, -0.5 * intoBack));
  const offTheBackSpin = [0, -12, 0];

  const rebound = { p: atHit.p, v: offTheBack, w: offTheBackSpin, q: [1, 0, 0, 0] };
  const toFloor = firstFallBelow(
      (tau) => dot(up, along({ kind: 'fly' }, rebound, tau).p) - (upOffset + POLLEN.radius), ONE_MS, 1);
  const onTheFloor = along({ kind: 'fly' }, rebound, toFloor);
  const downhill = unit(sub([0, 0, -1], scale(up, dot([0, 0, -1], up))));
  const rolling = scale(downhill, 20);
  const rollingSpin = scale(cross(up, rolling), 1 / POLLEN.radius);
  const rollFor = (dot(axis, onTheFloor.p) - (backOffset + POLLEN.radius)) / -dot(downhill, axis);

  return {
    start,
    tilts: { Blue: hive.tilt, Red: field.hives.find((h) => h.alliance === 'Red').tilt },
    kind: 'backspin',
    surfaces: [[178.12, 237.5], [178.12, 237.5]],
    changed: { y: -8 },
    legs: [
      firstLeg,
      { kind: 'impact', against: `${cell.name} back`, surface: 'cell', normal: axis, velocity: offTheBack,
        spin: offTheBackSpin, restitution: 0.5, gripped: false, phase: 'f' },
      { kind: 'fly', duration: toFloor, phase: 'f' },
      { kind: 'impact', against: `${cell.name} wall 2`, surface: 'cell', normal: up, velocity: rolling,
        spin: rollingSpin, restitution: 0.42, gripped: true, phase: 'r' },
      { kind: 'roll', duration: 2 * rollFor / 20, phase: 'r' },
      { kind: 'rest', duration: 0.3, phase: 'x' }
    ],
    pieces: () => piecesOf(field, []),
    crossingsOf: () => {
      const inFlight = (tau) => along(firstLeg, { p: from, v: velocity, w: spin, q: [1, 0, 0, 0] }, tau);
      const t = firstFallBelow((tau) => dot(axis, inFlight(tau).p) - mouthOffset, 0, toHit);
      const there = inFlight(t);
      const fromMiddle = sub(there.p, centroid(mouth));
      const across = sub(fromMiddle, scale(axis, dot(fromMiddle, axis)));
      return [{
        t: rounded(t, 5), cell: cell.name, alliance: 'Blue', upturned: true, into: true,
        at: roundedAll(there.p, 4), offset: rounded(norm(across), 4), rim: 3.1,
        angle: rounded(Math.acos(-dot(unit(there.v), axis)) * 180 / Math.PI, 4), speed: rounded(norm(there.v), 4)
      }];
    },
    ending: { kind: 'scored', says: `came to rest in ${cell.name}, which is upturned, so it scores` }
  };
}

function aHighSpinBounceAcrossTheFloor(field) {
  const from = [-30, 30, 14];
  const velocity = [113.1, 0, 113.1];
  const spin = [0, -900, 0];
  const r = POLLEN.radius;
  const toFloor = (state) => firstFallBelow((tau) => along({ kind: 'fly' }, state, tau).p[2] - r, ONE_MS, 2);
  const launched = { p: from, v: velocity, w: spin, q: [1, 0, 0, 0] };
  const first = toFloor(launched);
  const bounced = along({ kind: 'fly' }, launched, first);
  const afterFirst = [-40, 0, -0.55 * bounced.v[2]];
  const second = toFloor({ p: bounced.p, v: afterFirst, w: [0, -500, 0], q: [1, 0, 0, 0] });
  const secondState = along({ kind: 'fly' }, { p: bounced.p, v: afterFirst, w: [0, -500, 0], q: [1, 0, 0, 0] }, second);
  const afterSecond = [-30, 0, -0.55 * secondState.v[2]];
  const third = toFloor({ p: secondState.p, v: afterSecond, w: [0, -200, 0], q: [1, 0, 0, 0] });
  const rolling = [-25, 0, 0];
  const rollingSpin = scale(cross([0, 0, 1], rolling), 1 / r);
  const red = field.hives.find((h) => h.alliance === 'Red');
  return {
    start: { at: from, velocity, spin },
    tilts: { Blue: field.hives.find((h) => h.alliance === 'Blue').tilt, Red: -red.tilt },
    kind: 'backspin',
    surfaces: [[1520.4, 260.3], [1520.4, 260.3]],
    changed: { x: -30, y: 30, wheel_one_rpm: 5600, wheel_two_rpm: 960 },
    legs: [
      { kind: 'fly', duration: first, phase: 'f' },
      { kind: 'impact', against: 'Floor', surface: 'floor', normal: [0, 0, 1], velocity: afterFirst,
        spin: [0, -500, 0], restitution: 0.55, gripped: false, phase: 'f' },
      { kind: 'fly', duration: second, phase: 'f' },
      { kind: 'impact', against: 'Floor', surface: 'floor', normal: [0, 0, 1], velocity: afterSecond,
        spin: [0, -200, 0], restitution: 0.55, gripped: false, phase: 'f' },
      { kind: 'fly', duration: third, phase: 'f' },
      { kind: 'impact', against: 'Floor', surface: 'floor', normal: [0, 0, 1], velocity: rolling,
        spin: rollingSpin, restitution: 0, gripped: true, phase: 'r' },
      { kind: 'roll', duration: 25 / 15.4, phase: 'r' },
      { kind: 'rest', duration: 0.3, phase: 'x' }
    ],
    pieces: () => {
      const all = piecesOf(field, ['Red']);
      const knocked = all.find((piece) => !piece.fixed && !piece.kind.startsWith('Nectar'));
      for (let i = 0; i <= 125; i++) {
        const t = 1 + i * FRAME_EVERY_S;
        const q = turned([1, 0, 0, 0], [-20 / knocked.radius, 0, 0], i * FRAME_EVERY_S);
        knocked.track.push([rounded(t, 5), ...roundedAll(add(knocked.start, [0, 20 * i * FRAME_EVERY_S, 0]), 4),
                            ...roundedAll(q, 5)]);
      }
      return all;
    },
    ending: { kind: 'floor', says: 'came to rest on the floor' }
  };
}

function aShotOverRedsWall(field) {
  const from = [0, 20, 14];
  const velocity = [0, 153.2, 128.6];
  const spin = [0, 0, 50];
  const beyond = field.size / 2 + POLLEN.radius + 0.05;
  const launched = { p: from, v: velocity, w: spin, q: [1, 0, 0, 0] };
  const out = firstFallBelow((tau) => beyond - along({ kind: 'fly' }, launched, tau).p[1], ONE_MS, 2);
  return {
    start: { at: from, velocity, spin },
    tilts: Object.fromEntries(field.hives.map((h) => [h.alliance, h.tilt])),
    kind: 'sidespin',
    surfaces: [[206.1, 194.4], [206.1, 194.4]],
    changed: { x: 0, y: 20, yaw: 90, pieces: 0 },
    legs: [{ kind: 'fly', duration: out, phase: 'f' }],
    pieces: () => [],
    ending: { kind: 'out', says: "left the field over red's wall" }
  };
}

function filled(template, field) {
  return template
      .replace('__IMPORTMAP__', () => importMapFor(ASSETS))
      .replace('__ASSETS__', () => JSON.stringify(ASSETS))
      .replace('__TITLE__', () => TITLE)
      .replace('__FIELD__', () => JSON.stringify(field))
      .replace('__DATA__', () => '{}');
}

const STAGE = (src) => '<!doctype html><html lang="en"><head><meta charset="utf-8"><title>Stage</title></head>'
    + `<body style="margin:0"><iframe id="stage" src="${src}" style="width:600px;height:540px;border:0"></iframe>`
    + '</body></html>';

function served(page, shots) {
  const server = http.createServer((request, response) => {
    const asked = decodeURIComponent(request.url.split('?')[0]);
    if (asked === '/favicon.ico') {
      response.writeHead(204).end();
      return;
    }
    if (asked === '/launches/') {
      response.writeHead(200, { 'Content-Type': TYPES['.html'] });
      response.end(page);
      return;
    }
    if (asked === '/stage/') {
      response.writeHead(200, { 'Content-Type': TYPES['.html'] });
      response.end(STAGE('/launches/?view=flat#1'));
      return;
    }
    const wanted = /^\/launches\/(\d+)\/shot$/.exec(asked);
    if (wanted) {
      const shot = shots.get(Number(wanted[1]));
      if (!shot) {
        response.writeHead(404, { 'Content-Type': 'text/plain' }).end('no such launch');
        return;
      }
      response.writeHead(200, { 'Content-Type': 'application/json' });
      response.end(JSON.stringify(shot));
      return;
    }
    const asset = asked.startsWith('/assets/') ? asked.slice('/assets/'.length) : null;
    const file = asset && path.resolve(sim, asset);
    if (!file || !file.startsWith(path.resolve(sim)) || !fs.existsSync(file) || !fs.statSync(file).isFile()) {
      response.writeHead(404).end('no');
      return;
    }
    response.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream' });
    response.end(fs.readFileSync(file));
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

function servedElsewhere(launchPage) {
  const server = http.createServer((request, response) => {
    if (request.url.split('?')[0] === '/foreign/') {
      response.writeHead(200, { 'Content-Type': TYPES['.html'] });
      response.end(STAGE(launchPage));
      return;
    }
    response.writeHead(404).end('no');
  });
  return new Promise((resolve) => server.listen(0, '127.0.0.1', () => resolve(server)));
}

function faults(threw, forgiven) {
  const excused = (said) => (UNBUILT.test(said) && said.includes('404')) || forgiven.some((one) => said.includes(one));
  const anyExcused = threw.some(excused);
  return threw.filter((said) => !excused(said) && !(anyExcused && BARE_404.test(said)));
}

function watched(open) {
  const threw = [];
  const requested = [];
  open.on('pageerror', (e) => threw.push(String(e && e.message ? e.message : e)));
  open.on('request', (r) => requested.push(r.url()));
  open.on('requestfailed', (r) => {
    const why = r.failure() && r.failure().errorText;
    if (why !== 'net::ERR_ABORTED') {
      threw.push(`${r.url()} failed: ${why}`);
    }
  });
  open.on('response', (r) => { if (r.status() >= 400) threw.push(`${r.url()} answered ${r.status()}`); });
  open.on('console', (m) => { if (m.type() === 'error') threw.push(`the page logged an error: ${m.text()}`); });
  return { threw, requested };
}

const settledIn = (where) => where.waitForFunction(
    () => window.launchPage && window.launchPage.settled, null, { timeout: 120_000 });

const showing = (where, ids) => where.waitForFunction(
    (wanted) => window.launchPage && window.launchPage.settled
        && JSON.stringify(window.launchPage.shown) === JSON.stringify(wanted), ids, { timeout: 60_000 });

const twoFrames = (where) => where.evaluate(
    () => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));

const hook = (where) => where.evaluate(() => {
  const page = window.launchPage;
  return {
    settled: page.settled, problem: page.problem, drawing: page.drawing, shown: page.shown,
    time: page.time, frame: page.frame, ball: page.ball, ghosts: page.ghosts,
    colliderMeshes: page.colliderMeshes, pieces: page.pieces, impactMarkers: page.impactMarkers,
    preview: page.preview, leaning: page.leaning, ownPieces: page.ownPieces, robotShown: page.robotShown,
    view: page.view, following: page.following, playing: page.playing, events: page.events,
    spin: page.spin
  };
});

const goTo = (where, t) => where.evaluate((seconds) => window.launchPage.goTo(seconds), t);

const press = async (open, key) => {
  await open.evaluate(() => document.activeElement && document.activeElement.blur && document.activeElement.blur());
  await open.keyboard.press(key);
};

const positionOf = (frame) => frame.slice(1, 4);
const facedCount = (shot) => shot.colliders.filter((c) => c.faces.length > 0).length;

function relativeTurn(from, to) {
  const inverse = [from.w, -from.x, -from.y, -from.z];
  const r = quaternionTimes([to.w, to.x, to.y, to.z], inverse);
  const sign = r[0] < 0 ? -1 : 1;
  const half = Math.acos(Math.min(1, Math.abs(r[0])));
  const s = Math.sin(half);
  return { angle: 2 * half, axis: s > 0 ? [sign * r[1] / s, sign * r[2] / s, sign * r[3] / s] : [0, 0, 0] };
}

async function theShotIsDrawnWhereItsFirstFrameIs(open, shot, state) {
  const first = shot.frames[0];
  check(state.drawing === 'low', `the page drew ${state.drawing}, not the low model it falls back to`);
  check(/has not been built/.test(state.problem), `the page did not say why it drew low: "${state.problem}"`);
  check(JSON.stringify(state.shown) === '[1]', `the page shows ${JSON.stringify(state.shown)} for #1`);
  check(state.ownPieces === 0, `the field drew ${state.ownPieces} pieces of its own besides the shot's`);
  check(state.robotShown === false, 'a robot is drawn on the launch page');
  check(state.ball && state.ball.visible, 'the launched pollen is not drawn');
  check(state.ball && near(state.ball.position, positionOf(first), 1e-3),
        `the pollen starts at ${state.ball && state.ball.position}, not the first frame's ${positionOf(first)}`);
  check(state.ball && state.ball.holes === 26, `the pollen has ${state.ball && state.ball.holes} holes, not 26`);
  check(state.time === first[0] && state.frame === 0, `the page starts at t=${state.time}, frame ${state.frame}`);
  check(state.impactMarkers === shot.impacts.length,
        `${state.impactMarkers} impact markers are drawn for ${shot.impacts.length} impacts`);
  check(state.pieces.length === shot.pieces.length,
        `${state.pieces.length} of the shot's ${shot.pieces.length} pieces are drawn`);
  const fixed = shot.pieces.find((piece) => piece.fixed);
  const drawn = fixed && state.pieces.find((piece) => piece.name === fixed.name && near(piece.position, fixed.start, 1e-3));
  check(!!drawn, 'a fixed piece is not drawn at its start');
  const marks = await open.locator('#marks .mark').count();
  check(marks === shot.impacts.length + 1, `the scrub has ${marks} marks for ${shot.impacts.length} impacts and the apex`);
  const line = await open.locator('#now-line').textContent();
  check(/in\/s/.test(line) && /m\/s/.test(line) && /rpm/.test(line),
        `the live readout "${line}" does not give the speed in in/s and m/s and the spin in rpm`);
  const launch = await open.locator('#launch').textContent();
  check(launch.includes(shot.launch.speed.toFixed(1)) && /backspin/.test(launch),
        `the launch readout "${launch}" does not give the exit speed ${shot.launch.speed.toFixed(1)} and its backspin`);
  const ending = await open.locator('#ending').textContent();
  check(ending.includes(shot.ending.says), `the ending reads "${ending}"`);
  const computed = await open.locator('#computed-line').textContent();
  check(/computed in .* ms/.test(computed) && /real time/.test(computed), `the cost reads "${computed}"`);
}

async function playbackGoesWhereItIsAsked(open, shot) {
  const frames = shot.frames;
  const last = frames[frames.length - 1];
  await goTo(open, last[0]);
  let state = await hook(open);
  check(near(state.ball.position, positionOf(last), 1e-3),
        `goTo(end) put the pollen at ${state.ball.position}, not the last frame's ${positionOf(last)}`);
  check(state.frame === frames.length - 1, `goTo(end) shows frame ${state.frame} of ${frames.length}`);

  await press(open, 'Home');
  state = await hook(open);
  check(state.time === frames[0][0], `Home went to t=${state.time}`);
  await press(open, 'ArrowRight');
  state = await hook(open);
  check(state.frame === 1 && state.time === frames[1][0], `the right arrow went to frame ${state.frame}, t=${state.time}`);
  await press(open, 'Home');
  await press(open, 'Shift+ArrowRight');
  state = await hook(open);
  const next = state.events.find((e) => e.t > frames[0][0]);
  check(next && state.time === next.t, `Shift+right went to t=${state.time}, not the next event at ${next && next.t}`);
  await press(open, 'End');
  state = await hook(open);
  check(state.time === last[0], `End went to t=${state.time}, not ${last[0]}`);

  const impact = shot.impacts[1];
  await open.locator('#impacts tbody tr').nth(1).click();
  state = await hook(open);
  check(state.time === impact.t, `clicking the second impact went to t=${state.time}, not its ${impact.t}`);

  await press(open, 'Home');
  await open.locator('#play').click();
  const moved = await open.waitForFunction((t0) => window.launchPage.time > t0, frames[0][0], { timeout: 20_000 })
      .then(() => true, () => false);
  state = await hook(open);
  check(moved && state.playing, `pressing Play left the time at ${state.time}, playing ${state.playing}`);
  await open.locator('#play').click();
  state = await hook(open);
  check(!state.playing, 'pressing Pause did not pause');

  await press(open, 'Space');
  const spaced = await hook(open);
  await press(open, 'Space');
  state = await hook(open);
  check(spaced.playing && !state.playing, `Space played ${spaced.playing} and then paused ${!state.playing}`);
}

async function theCollidersAreDrawnOverTheField(open, shot) {
  const box = open.locator('#show-colliders');
  await box.check();
  await twoFrames(open);
  let state = await hook(open);
  check(state.colliderMeshes === facedCount(shot),
        `showing the colliders drew ${state.colliderMeshes} meshes for ${facedCount(shot)} colliders with faces`);
  const legend = await open.locator('#surface-legend').textContent();
  check(/cell/.test(legend) && /wall/.test(legend), `the surfaces are not named in a legend: "${legend}"`);
  await box.uncheck();
  state = await hook(open);
  check(state.colliderMeshes === 0, `hiding the colliders left ${state.colliderMeshes} meshes`);
}

async function theCameraFramesAndFollows(open, shot) {
  await open.locator('#field').dblclick();
  await twoFrames(open);
  let state = await hook(open);
  const xs = shot.frames.map((f) => f[1]);
  const zs = shot.frames.map((f) => f[3]);
  const target = state.view.target;
  check(target[0] >= Math.min(...xs) - 1 && target[0] <= Math.max(...xs) + 1
        && target[2] >= Math.min(...zs) - 1 && target[2] <= Math.max(...zs) + 1,
        `double-clicking aimed the camera at ${target}, not the middle of the path`);
  await open.locator('#follow').click();
  const middle = shot.frames[Math.floor(shot.frames.length / 3)];
  await goTo(open, middle[0]);
  await twoFrames(open);
  await twoFrames(open);
  state = await hook(open);
  check(state.following, 'the follow button did not start following the pollen');
  check(near(state.view.target, state.ball.position, 0.05),
        `following, the camera looks at ${state.view.target} while the pollen is at ${state.ball.position}`);
  await open.locator('#follow').click();
}

async function anotherLaunchIsSwappedInWithoutReloading(open, shots, requested) {
  const shot = shots.get(2);
  const before = await hook(open);
  await open.evaluate(() => { window.stillThisPage = window.launchPage; });
  const modelsBefore = requested.filter((u) => MODEL.test(u)).length;
  await open.evaluate(() => { location.hash = '#2'; });
  await showing(open, [2]);
  const same = await open.evaluate(() => window.stillThisPage === window.launchPage);
  check(same, 'showing launch 2 reloaded the page');
  check(requested.filter((u) => MODEL.test(u)).length === modelsBefore, 'showing launch 2 fetched the field model again');
  const state = await hook(open);
  check(near(state.ball.position, positionOf(shot.frames[0]), 1e-3),
        `after the swap the pollen is at ${state.ball.position}, not launch 2's start`);
  check(state.pieces.length === shot.pieces.length, `${state.pieces.length} of launch 2's ${shot.pieces.length} pieces drawn`);
  check(Math.abs(state.leaning.Red - before.leaning.Red) > 0.01,
        `the red hive leans ${state.leaning.Red} in a launch where it is tipped and ${before.leaning.Red} at rest`);
  const launch = await open.locator('#launch').textContent();
  check(launch.includes(String(Math.round(shot.launch.spinRpm))), `launch 2's readout "${launch}" lacks its rpm`);
  const marks = await open.locator('#marks .mark').count();
  check(marks === shot.impacts.length + 1, `launch 2's scrub has ${marks} marks for ${shot.impacts.length} impacts and the apex`);

  const knocked = shot.pieces.find((piece) => piece.track.length);
  await goTo(open, shot.frames[shot.frames.length - 1][0]);
  const ended = await hook(open);
  const lastTrack = knocked.track[knocked.track.length - 1].slice(1, 4);
  const drawn = ended.pieces.find((piece) => piece.name === knocked.name && near(piece.position, lastTrack, 1e-3));
  check(!!drawn, `the knocked ${knocked.name} is not at the end of its track ${lastTrack}`);
}

async function theSpinTurnsThePollenTheRightWay(open, shot) {
  const k = 10;
  const frame = shot.frames[k];
  const spin = frame.slice(7, 10);
  check(shot.frames[k + 1][0] - frame[0] > ONE_MS && frame[14] === 'f',
        'the canned high-spin shot has no frame in flight to turn from');
  await goTo(open, frame[0]);
  const from = (await hook(open)).ball.quaternion;
  await goTo(open, frame[0] + ONE_MS);
  const to = (await hook(open)).ball.quaternion;
  const turn = relativeTurn(from, to);
  const expected = norm(spin) * ONE_MS;
  check(Math.abs(turn.angle - expected) < 2e-3,
        `in 1 ms the pollen turned ${turn.angle.toFixed(4)} rad, not ${expected.toFixed(4)} at ${norm(spin)} rad/s`);
  check(dot(turn.axis, unit(spin)) > 0.999,
        `the pollen turned about ${turn.axis.map((v) => v.toFixed(3))}, not its spin axis ${unit(spin)}`);
  const state = await hook(open);
  check(state.spin && state.spin.visible && near(state.spin.direction, unit(spin), 1e-3),
        `the spin arrow points ${state.spin && state.spin.direction}, not along ${unit(spin)}`);
  check(state.spin && /rpm/.test(state.spin.label) && /backspin/.test(state.spin.label),
        `the spin arrow is labelled "${state.spin && state.spin.label}"`);
}

function onItsFlight(frame, tau) {
  return [frame[1] + frame[4] * tau, frame[2] + frame[5] * tau,
          frame[3] + frame[6] * tau - GRAVITY_IN_PER_S2 * tau * tau / 2];
}

async function betweenFramesThePollenIsWhereItsFlightPutsIt(open, shot) {
  const k = 10;
  const frame = shot.frames[k];
  const tau = 0.0013;
  await goTo(open, frame[0] + tau);
  const between = (await hook(open)).ball.position;
  check(near(between, onItsFlight(frame, tau), 2e-3),
        `${tau * 1000} ms after frame ${k} the pollen is at ${between}, not on its flight at ${onItsFlight(frame, tau)}`);

  const impact = shot.impacts[0];
  const before = shot.frames.filter((f) => f[0] < impact.t).pop();
  const ahead = impact.t - before[0];
  await goTo(open, impact.t - ahead / 2);
  const coming = (await hook(open)).ball.position;
  check(near(coming, onItsFlight(before, ahead / 2), 2e-3),
        `just before the first impact the pollen is at ${coming}, not on the flight into it at `
        + `${onItsFlight(before, ahead / 2)}: the drawing reached across the impact`);
}

async function ghostsAreDrawnBesideTheShownLaunch(open, shots) {
  await open.evaluate(() => { location.hash = '#1,2,3'; });
  await showing(open, [1, 2, 3]);
  let state = await hook(open);
  check(state.ghosts.map((g) => g.id).join(',') === '2,3', `#1,2,3 drew ghosts ${state.ghosts.map((g) => g.id)}`);
  for (const ghost of state.ghosts) {
    check(ghost.points === shots.get(ghost.id).frames.length,
          `ghost ${ghost.id} has ${ghost.points} points of ${shots.get(ghost.id).frames.length}`);
    check(ghost.label === `#${ghost.id}`, `ghost ${ghost.id} is labelled "${ghost.label}"`);
  }
  await open.evaluate(() => { location.hash = '#1,2'; });
  await showing(open, [1, 2]);
  state = await hook(open);
  check(state.ghosts.length === 1 && state.ghosts[0].id === 2, `#1,2 drew ghosts ${state.ghosts.map((g) => g.id)}`);
  check(near(state.ball.position, positionOf(shots.get(1).frames[0]), 1e-3), 'with ghosts the pollen is not launch 1\'s');
}

async function theCollidersAloneCanBeTheField(open, shots, requested) {
  const before = requested.length;
  await Promise.all([open.waitForURL(/view=flat/, { timeout: 30_000 }),
                     open.locator('#resolution').selectOption('colliders')]);
  await settledIn(open);
  await showing(open, [1, 2]);
  check(/#1,2$/.test(open.url()), `choosing colliders lost the launches shown: ${open.url()}`);
  const state = await hook(open);
  const since = requested.slice(before);
  check(state.drawing === 'colliders', `choosing colliders drew ${state.drawing}`);
  check(!since.some((u) => MODEL.test(u)), `the colliders fetched a field model: ${since.filter((u) => MODEL.test(u))}`);
  check(state.colliderMeshes === facedCount(shots.get(1)),
        `the colliders drew ${state.colliderMeshes} meshes for ${facedCount(shots.get(1))} colliders with faces`);
  check(await open.locator('#resolution').inputValue() === 'colliders', 'the control does not say colliders are drawn');
  check(near(state.ball.position, positionOf(shots.get(1).frames[0]), 1e-3), 'the colliders drawing lost the pollen');
}

async function theWideViewWorks(browser, base, shots) {
  const open = await browser.newPage({ viewport: WIDE });
  const { threw, requested } = watched(open);
  try {
    await open.goto(`${base}/launches/#1`, { waitUntil: 'load', timeout: 60_000 });
    await settledIn(open);
    await showing(open, [1]);
    await twoFrames(open);
    const shot = shots.get(1);
    await theShotIsDrawnWhereItsFirstFrameIs(open, shot, await hook(open));
    await playbackGoesWhereItIsAsked(open, shot);
    await theCollidersAreDrawnOverTheField(open, shot);
    await theCameraFramesAndFollows(open, shot);
    await anotherLaunchIsSwappedInWithoutReloading(open, shots, requested);
    await theSpinTurnsThePollenTheRightWay(open, shots.get(2));
    await betweenFramesThePollenIsWhereItsFlightPutsIt(open, shots.get(2));
    await ghostsAreDrawnBesideTheShownLaunch(open, shots);
    await theCollidersAloneCanBeTheField(open, shots, requested);
    const left = faults(threw, []);
    check(left.length === 0, `the launch page threw or failed a request: ${left.join('; ')}`);
  } catch (stuck) {
    wrong.push(`the wide page got stuck: ${stuck && stuck.message}`);
    await open.screenshot({ path: path.join(here, 'launch-as-drawn.png') }).catch(() => {});
  } finally {
    await open.close();
  }
}

async function theNarrowViewFitsAndSaysWhatIsMissing(browser, base) {
  const open = await browser.newPage({ viewport: NARROW });
  const { threw } = watched(open);
  try {
    await open.goto(`${base}/launches/?view=flat`, { waitUntil: 'load', timeout: 60_000 });
    await settledIn(open);
    let state = await hook(open);
    check(state.shown.length === 0 && state.ball === null, 'with no launch asked for, the page drew one');
    await open.evaluate(() => { location.hash = '#99'; });
    await open.waitForFunction(() => /99/.test(window.launchPage.problem), null, { timeout: 30_000 });
    state = await hook(open);
    check(state.shown.length === 0, `a launch the bench does not have drew ${state.shown}`);
    check(/no launch 99/i.test(state.problem), `asking for a launch that is not there says "${state.problem}"`);
    await open.evaluate(() => { location.hash = '#1'; });
    await showing(open, [1]);
    await twoFrames(open);
    for (const [what, selector] of [['canvas', '#field'], ['play button', '#play'], ['live readout', '#now-line']]) {
      const box = await open.locator(selector).boundingBox();
      check(box && box.y >= 0 && box.x >= 0 && box.y + box.height <= NARROW.height && box.x + box.width <= NARROW.width,
            `at ${NARROW.width}x${NARROW.height} the ${what} is at ${JSON.stringify(box)}, not in view without scrolling`);
    }
    const page = await open.evaluate(() => ({ y: scrollY, wide: document.documentElement.scrollWidth }));
    check(page.y === 0 && page.wide <= NARROW.width, `the narrow page scrolls: ${JSON.stringify(page)}`);
    const left = faults(threw, ['/launches/99/shot']);
    check(left.length === 0, `the narrow page threw or failed a request: ${left.join('; ')}`);
  } catch (stuck) {
    wrong.push(`the narrow page got stuck: ${stuck && stuck.message}`);
  } finally {
    await open.close();
  }
}

async function theStageSendsThePreviewArrow(browser, base, elsewhere) {
  const open = await browser.newPage({ viewport: { width: 640, height: 600 } });
  const { threw } = watched(open);
  try {
    await open.goto(`${base}/stage/`, { waitUntil: 'load', timeout: 60_000 });
    const frame = open.frames().find((f) => f.url().includes('/launches/'));
    await settledIn(frame);
    const before = await hook(frame);
    check(before.preview === null, 'a preview arrow is drawn before anybody sent one');
    const at = [50, -12.76, 14];
    const direction = [-0.342, 0, 0.94];
    await open.evaluate(([from, toward]) => document.getElementById('stage').contentWindow.postMessage(
        { type: 'launch-preview', at: from, direction: toward }, location.origin), [at, direction]);
    await frame.waitForFunction(() => window.launchPage.preview !== null, null, { timeout: 10_000 });
    const after = await hook(frame);
    check(after.preview.visible && near(after.preview.at, at, 1e-6) && near(after.preview.direction, unit(direction), 1e-6),
          `the preview arrow is ${JSON.stringify(after.preview)}, not at ${at} toward ${direction}`);
    const left = faults(threw, []);
    check(left.length === 0, `the staged page threw or failed a request: ${left.join('; ')}`);
  } catch (stuck) {
    wrong.push(`the staged page got stuck: ${stuck && stuck.message}`);
  } finally {
    await open.close();
  }

  const foreign = await browser.newPage({ viewport: { width: 640, height: 600 } });
  try {
    await foreign.goto(`${elsewhere}/foreign/`, { waitUntil: 'load', timeout: 60_000 });
    const frame = foreign.frames().find((f) => f.url().includes('/launches/'));
    await settledIn(frame);
    await foreign.evaluate(() => document.getElementById('stage').contentWindow.postMessage(
        { type: 'launch-preview', at: [0, 0, 10], direction: [1, 0, 0] }, '*'));
    await foreign.waitForTimeout(300);
    const state = await hook(frame);
    check(state.preview === null, 'a page of another origin drew a preview arrow on the launch page');
  } catch (stuck) {
    wrong.push(`the foreign stage got stuck: ${stuck && stuck.message}`);
  } finally {
    await foreign.close();
  }
}

async function main() {
  const template = fs.readFileSync(path.join(sim, 'launch.html'), 'utf8');
  const holes = [...new Set(template.match(/__[A-Z_]+__/g) || [])].sort();
  check(holes.join(',') === PLACEHOLDERS.join(','),
        `the page holds the placeholders ${holes.join(', ')}, not the ones the bench fills: ${PLACEHOLDERS.join(', ')}`);
  const raw = JSON.parse(fs.readFileSync(path.join(sim, 'field.json'), 'utf8'));
  const field = Object.assign({}, raw, { moved: movedPieces(raw) });
  const page = filled(template, field);
  const left = page.match(/__[A-Z_]+__/g);
  check(left === null, `the filled page still holds ${left && [...new Set(left)].join(', ')}`);

  const shots = new Map([
    [1, shotJson(1, field, aLobIntoTheBlueScoringCell(field))],
    [2, shotJson(2, field, aHighSpinBounceAcrossTheFloor(field))],
    [3, shotJson(3, field, aShotOverRedsWall(field))]
  ]);
  check(shots.get(1).crossings.length === 1 && shots.get(1).impacts.length === 2,
        'the canned lob does not cross the cell mouth once and hit twice');

  const server = await served(page, shots);
  const base = `http://127.0.0.1:${server.address().port}`;
  const other = await servedElsewhere(`${base}/launches/?view=flat#1`);
  const elsewhere = `http://127.0.0.1:${other.address().port}`;
  let browser = null;
  try {
    browser = await chrome();
    await theWideViewWorks(browser, base, shots);
    await theNarrowViewFitsAndSaysWhatIsMissing(browser, base);
    await theStageSendsThePreviewArrow(browser, base, elsewhere);
  } catch (stuck) {
    wrong.push(String(stuck && stuck.message));
  } finally {
    if (browser) {
      await browser.close();
    }
    server.close();
    other.close();
  }

  if (wrong.length) {
    console.error('the launch page did not show its shots as it should:');
    for (const saying of wrong) {
      console.error('  ' + saying);
    }
    process.exit(1);
  }
  const lob = shots.get(1);
  console.log(`the launch page shows a lob of ${lob.frames.length} frames into ${lob.crossings[0].cell} with `
      + `${lob.impacts.length} impacts, turns a ${Math.round(shots.get(2).launch.spinRpm)} rpm pollen the way it `
      + 'spins, swaps launches and ghosts by its fragment without reloading, draws the colliders over the field '
      + `or as the field, fits ${NARROW.width}x${NARROW.height}, and draws the preview its own stage sends it.`);
}

await main();
