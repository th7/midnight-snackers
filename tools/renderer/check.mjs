
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const sim = path.join(here, '..', '..', 'TeamCode', 'src', 'test', 'resources',
                      'org', 'firstinspires', 'ftc', 'teamcode', 'sim');
const vendor = path.join(sim, 'vendor');

const model = process.argv[2] || path.join(sim, 'field.glb');

const problems = [];
const check = (ok, said) => { if (!ok) problems.push(said); };

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
  await import(path.join(addons, 'controls', 'OrbitControls.js'));
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

  check(box.max.x > 60 && box.max.x < 200, `the field reaches ${box.max.x.toFixed(1)} in along x`);
  check(box.min.z > -2 && box.min.z < 1, `the floor is at z = ${box.min.z.toFixed(1)} in, not 0`);
  check(box.max.z > 30 && box.max.z < 100, `the field stands ${box.max.z.toFixed(1)} in tall`);

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
