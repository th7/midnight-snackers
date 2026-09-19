/**
 * Load the field's visual model with the three.js the page actually uses.
 *
 * The writer that makes field.glb is checked by reading it back with the reader that made it,
 * which says the two agree with each other. This says something else: that the library the
 * browser will hand it to accepts it. It is the same distinction as signing a request and having
 * Onshape accept the signature -- only the other side can say.
 *
 * It runs in node, where there is no WebGL, so nothing here draws. What it proves is that the
 * file parses, that the scene has the meshes and materials it should, that the assembly's names
 * survive, and that the field comes out the size and the way round it is meant to be. A page that
 * got any of that wrong would show a blank canvas or a field inside out, and neither says why.
 *
 *     node tools/renderer/check.mjs
 */
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const sim = path.join(here, '..', '..', 'TeamCode', 'src', 'test', 'resources',
                      'org', 'firstinspires', 'ftc', 'teamcode', 'sim');
const vendor = path.join(sim, 'vendor');
// A model given on the command line rather than the committed one, so that this check can be
// pointed at a deliberately wrong field and shown to catch it.
const model = process.argv[2] || path.join(sim, 'field.glb');

const problems = [];
const check = (ok, said) => { if (!ok) problems.push(said); };

/**
 * The addons import the bare specifier 'three', which a browser resolves through the page's
 * import map and node does not resolve at all. The whole addon tree is copied to a temporary
 * one with that specifier rewritten to the vendored file, so the addons' own relative imports
 * -- GLTFLoader reaches for ../utils/BufferGeometryUtils.js -- still resolve between them.
 *
 * The copy is of the real vendored source, so what is loaded here is what the browser loads.
 */
function mirroredAddons() {
  const three = JSON.stringify(path.join(vendor, 'three.module.min.js'));
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'renderer-addons-'));
  const copy = (from, to) => {
    fs.mkdirSync(to, { recursive: true });
    for (const entry of fs.readdirSync(from, { withFileTypes: true })) {
      const source = path.join(from, entry.name);
      const target = path.join(to, entry.name);
      if (entry.isDirectory()) {
        copy(source, target);
      } else if (entry.name.endsWith('.js')) {
        fs.writeFileSync(target, fs.readFileSync(source, 'utf8').replaceAll("from 'three'", `from ${three}`));
      }
    }
  };
  copy(path.join(vendor, 'jsm'), root);
  return root;
}

const THREE = await import(path.join(vendor, 'three.module.min.js'));
check(THREE.REVISION === '169', `three.js is revision ${THREE.REVISION}, and the page expects 169`);

const addons = mirroredAddons();
let scene = null;
try {
  const { GLTFLoader } = await import(path.join(addons, 'loaders', 'GLTFLoader.js'));
  await import(path.join(addons, 'controls', 'OrbitControls.js')); // it must at least load
  const bytes = fs.readFileSync(model);
  const buffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
  scene = await new Promise((resolve, reject) =>
      new GLTFLoader().parse(buffer, '', (gltf) => resolve(gltf.scene), reject));
} catch (wrong) {
  problems.push(`three.js could not load ${path.basename(model)}: ${wrong && wrong.message}`);
} finally {
  fs.rmSync(addons, { recursive: true, force: true });
}

if (scene) {
  let meshes = 0;
  let triangles = 0;
  const materials = new Set();
  const named = [];
  scene.traverse((object) => {
    if (object.name) named.push(object.name);
    if (object.isMesh) {
      meshes++;
      triangles += object.geometry.index.count / 3;
      materials.add(object.material.name);
    }
  });
  const box = new THREE.Box3().setFromObject(scene);

  check(meshes > 100, `only ${meshes} meshes; the field has a part for each`);
  check(triangles > 100000, `only ${triangles} triangles; the detail is the point of this model`);
  check(materials.size > 1, `only ${materials.size} material; parts carry the CAD's colours`);
  check(named.some((name) => /Flower/i.test(name)), 'no flower is named; the assembly names are gone');
  check(named.some((name) => /Hive/i.test(name)), 'no hive is named; the assembly names are gone');

  // Inches, Road Runner's frame: the walls stand about 70 in out, and the field reaches further
  // where the trays and the rows of game pieces are set out beyond them.
  check(box.max.x > 60 && box.max.x < 200, `the field reaches ${box.max.x.toFixed(1)} in along x`);
  check(box.min.z > -2 && box.min.z < 1, `the floor is at z = ${box.min.z.toFixed(1)} in, not 0`);
  check(box.max.z > 30 && box.max.z < 100, `the field stands ${box.max.z.toFixed(1)} in tall`);

  // Which way round it is, the one thing the field is not symmetric about.
  const middle = new THREE.Vector3();
  let blue = null;
  scene.traverse((object) => {
    if (blue === null && object.isMesh && /blue/i.test(object.name)) {
      new THREE.Box3().setFromObject(object).getCenter(middle);
      blue = middle.y;
    }
  });
  check(blue !== null, 'no blue part to say which way round the field is');
  check(blue === null || blue < 0, `a blue part sits at y = ${blue}, and blue belongs at negative y`);

  console.log(`${path.basename(model)}: ${meshes} meshes, ${triangles} triangles, ${materials.size} materials`);
  console.log(`  bbox  x ${box.min.x.toFixed(1)}..${box.max.x.toFixed(1)}`
      + `  y ${box.min.y.toFixed(1)}..${box.max.y.toFixed(1)}`
      + `  z ${box.min.z.toFixed(1)}..${box.max.z.toFixed(1)}  (inches)`);
}

if (problems.length) {
  console.error('the renderer would not draw the field as it should:');
  for (const said of problems) {
    console.error('  ' + said);
  }
  process.exit(1);
}
console.log('three.js loads the field, the right way round and the right size.');
