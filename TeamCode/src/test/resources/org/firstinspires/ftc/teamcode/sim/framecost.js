const DRAW_CALLS = ['drawElements', 'drawArrays', 'drawElementsInstanced', 'drawArraysInstanced'];

const LEAST_MS = 50;
const BLOCKS = 9;
const MOST_RENDERS = 4096;

export function drawnBy(renderer) {
  const gl = renderer.getContext();
  if (!gl) {
    return 'nothing: there is no WebGL context';
  }
  const named = gl.getExtension('WEBGL_debug_renderer_info');
  return String(named ? gl.getParameter(named.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER));
}

export async function measure(renderer, draw, asked = {}) {
  const least = asked.least === undefined ? LEAST_MS : asked.least;
  const blocks = asked.blocks === undefined ? BLOCKS : asked.blocks;
  const most = asked.most === undefined ? MOST_RENDERS : asked.most;

  const gl = renderer.getContext();
  if (!gl) {
    return { judged: false, why: 'there is no WebGL context to time' };
  }

  const pixel = new Uint8Array(4);
  const settle = () => gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, pixel);
  const breathe = () => new Promise((go) => setTimeout(go, 0));

  function block(renders) {
    const started = performance.now();
    for (let i = 0; i < renders; i++) {
      draw();
    }
    const submitted = performance.now();
    settle();
    const finished = performance.now();
    return { renders: renders, submit: submitted - started, finish: finished - started };
  }

  block(1);
  block(1);

  let renders = 1;
  let taken = block(renders);
  while (taken.finish < least && renders < most) {
    renders = Math.min(most, renders * 2);
    await breathe();
    taken = block(renders);
  }

  if (taken.finish < least) {
    return {
      judged: false,
      renders: renders,
      why: renders + ' renders finished in ' + taken.finish.toFixed(3) + ' ms, short of the ' + least
          + ' ms it takes to tell one frame from another on this clock, so any number here would be '
          + 'the clock’s resolution rather than the frame’s cost'
    };
  }

  const finishes = [];
  const submits = [];
  for (let i = 0; i < blocks; i++) {
    await breathe();
    const one = block(renders);
    finishes.push(one.finish / renders);
    submits.push(one.submit / renders);
  }

  return Object.assign({
    judged: true,
    renders: renders,
    best: quickest(finishes),
    median: middle(finishes),
    bestSubmit: quickest(submits),
    medianSubmit: middle(submits),
    programs: renderer.info.programs ? renderer.info.programs.length : 0,
    geometries: renderer.info.memory.geometries,
    textures: renderer.info.memory.textures
  }, whatOneFrameDraws(gl, renderer, draw));
}

function whatOneFrameDraws(gl, renderer, draw) {
  const was = {};
  let drawn = 0;
  for (const name of DRAW_CALLS) {
    if (typeof gl[name] === 'function') {
      was[name] = gl[name];
      gl[name] = function () {
        drawn++;
        return was[name].apply(gl, arguments);
      };
    }
  }
  try {
    draw();
  } finally {
    for (const name of Object.keys(was)) {
      gl[name] = was[name];
    }
  }
  return {
    draws: drawn,
    calls: renderer.info.render.calls,
    triangles: renderer.info.render.triangles
  };
}

function quickest(values) {
  return Math.min.apply(null, values);
}

function middle(values) {
  const sorted = values.slice().sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)];
}
