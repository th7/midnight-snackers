import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export const FIELD_IN = 141.17;
export const FLOOR_Z = 0;
export const TAPE_Z = 0.01;
export const OVERLAY_Z = 0.05;

const SEE_THROUGH = /skin|side[\s_]glass/i;
const GAME_PIECE = /pollen|nectar/i;

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export class FieldScene {
  constructor({ canvas, assets, background }) {
    this.assets = assets;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, canvas: canvas });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(background);

    this.camera = new THREE.PerspectiveCamera(38, 1, 1, 4000);
    this.camera.position.set(-170, -190, 130);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 18);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.02;
    this.controls.minDistance = 30;
    this.controls.maxDistance = 900;

    const key = new THREE.DirectionalLight(0xffffff, 2.1);
    key.position.set(-120, -160, 260);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 40;
    key.shadow.camera.far = 700;
    const reach = FIELD_IN * 1.1;
    Object.assign(key.shadow.camera, { left: -reach, right: reach, top: reach, bottom: -reach });
    key.shadow.bias = -0.0015;
    this.scene.add(key);
    this.scene.add(new THREE.HemisphereLight(0xbfd4e8, 0x2a2f36, 1.0));

    this.floor = new THREE.Mesh(
        new THREE.PlaneGeometry(FIELD_IN, FIELD_IN),
        new THREE.MeshStandardMaterial({ color: 0x20262e, roughness: 0.95, metalness: 0 }));
    this.floor.position.z = FLOOR_Z;
    this.floor.receiveShadow = true;
    this.scene.add(this.floor);

    this.tiles = new THREE.GridHelper(FIELD_IN, 6, 0x4a5a68, 0x33414d);
    this.tiles.rotation.x = Math.PI / 2;
    this.tiles.position.z = FLOOR_Z + 0.02;
    this.scene.add(this.tiles);

    this.field = null;
    this.loaded = null;
    this.model = null;
    this.movingBalls = [];
    this.staticBalls = [];
    this.hives = [];
    this.robot = null;
    this.trail = null;
    this.overlay = new THREE.Group();
    this.scene.add(this.overlay);
  }

  load() {
    return new Promise((resolve, reject) => {
      new GLTFLoader().load(this.assets + 'field.glb', (gltf) => {
        let meshes = 0;
        let triangles = 0;
        gltf.scene.traverse((object) => {
          if (!object.isMesh) {
            return;
          }
          meshes++;
          triangles += object.geometry.index ? object.geometry.index.count / 3
                                             : object.geometry.attributes.position.count / 3;
          object.castShadow = true;
          object.receiveShadow = true;

          if (!object.geometry.attributes.normal) {
            object.geometry.computeVertexNormals();
          }
          object.material.roughness = 0.62;
          object.material.metalness = 0.05;

          if (/tape/i.test(object.name)) {
            object.position.z += TAPE_Z;
          }

          if (SEE_THROUGH.test(object.name)) {
            object.material = object.material.clone();
            object.material.transparent = true;
            object.material.opacity = 0.22;
            object.material.side = THREE.DoubleSide;
            object.material.depthWrite = false;
            object.castShadow = false;
          }
        });
        this.scene.add(gltf.scene);
        this.field = gltf.scene;
        this.loaded = { parts: meshes, triangles: triangles };
        resolve(this.loaded);
      }, undefined, (wrong) => reject(
          wrong instanceof Error ? wrong : new Error(String(wrong && wrong.message ? wrong.message : wrong))));
    });
  }

  place(model) {
    this.model = model;
    const moved = model.moved;

    this.field.traverse((o) => {
      if (o.isMesh && GAME_PIECE.test(o.name)) {
        o.visible = false;
      }
    });

    this.movingBalls = moved.map((piece) => {
      const ball = this.#ball(piece);
      ball.position.set(piece.centre[0], piece.centre[1], piece.centre[2]);
      return ball;
    });

    this.staticBalls = model.pieces
        .filter((piece) => !piece.loose && piece.cell === undefined && piece.flower === undefined)
        .map((piece) => {
          const ball = this.#ball(piece);
          ball.position.set(piece.centre[0], piece.centre[1], piece.centre[2]);
          return ball;
        });

    this.hives = model.hives.map((hive) => ({
      group: this.#groupNamedFor(hive.alliance),
      pivot: hive.pivot,
      rest: hive.tilt,
      alliance: hive.alliance
    })).filter((h) => h.group);

    this.robot = this.#robotOf(model.robotIn || 18);
    this.scene.add(this.robot);
  }

  showRobot(pose) {
    if (!this.robot) {
      return;
    }
    this.robot.visible = true;
    this.robot.position.set(pose.x, pose.y, 0);
    this.robot.rotation.z = pose.heading;
  }

  showPieces(where) {
    if (!where) {
      return;
    }
    const spare = this.model.moved[0] || { colour: '#ffef3f', radius: 1.4 };
    while (this.movingBalls.length < where.length) {
      const ball = this.#ball(spare);
      ball.visible = false;
      this.movingBalls.push(ball);
    }
    for (let i = 0; i < this.movingBalls.length; i++) {
      const ball = this.movingBalls[i];
      const to = where[i];
      if (to) {
        ball.visible = true;
        ball.position.set(to[0], to[1], to.length > 2 ? to[2] : ball.userData.radius);
      } else {
        ball.visible = false;
      }
    }
  }

  showTilts(leaning) {
    for (const hive of this.hives) {
      const said = leaning && leaning[hive.alliance];
      this.#lean(hive, said === undefined || said === null ? hive.rest : said);
    }
  }

  showTrail(points) {
    if (this.trail) {
      this.scene.remove(this.trail);
      this.trail.geometry.dispose();
      this.trail.material.dispose();
      this.trail = null;
    }
    if (points.length < 2) {
      return;
    }
    const line = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
      line[i * 3] = p[0];
      line[i * 3 + 1] = p[1];
      line[i * 3 + 2] = OVERLAY_Z;
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(line, 3));
    this.trail = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0xd84a05, transparent: true, opacity: 0.7 }));
    this.scene.add(this.trail);
  }

  showOverlay(ops) {
    for (const drawn of this.overlay.children.slice()) {
      this.overlay.remove(drawn);
      drawn.geometry.dispose();
      drawn.material.dispose();
    }
    let colour = 0x4caf50;
    for (const op of ops) {
      if (op.type === 'STROKE' || op.type === 'FILL') {
        colour = new THREE.Color(op.color).getHex();
      } else {
        for (const ring of ringsOf(op)) {
          this.overlay.add(this.#floorLine(ring, colour, op.type === 'POLYGON' || op.type === 'CIRCLE'));
        }
      }
    }
  }

  drawnFrom(camera) {
    this.renderer.render(this.scene, camera || this.camera);
  }

  fit(width, height) {
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  background(colour) {
    this.scene.background = new THREE.Color(colour);
  }

  #ball(piece) {
    const ball = new THREE.Mesh(
        new THREE.SphereGeometry(piece.radius, 20, 14),
        new THREE.MeshStandardMaterial({ color: new THREE.Color(piece.colour), roughness: 0.55 }));
    ball.castShadow = true;
    ball.userData.radius = piece.radius;
    this.scene.add(ball);
    return ball;
  }

  #robotOf(size) {
    const group = new THREE.Group();
    const body = new THREE.Mesh(
        new THREE.BoxGeometry(size, size, size),
        new THREE.MeshStandardMaterial({ color: 0x2f6fb5, roughness: 0.45, metalness: 0.1 }));
    body.position.z = size / 2;
    body.castShadow = true;
    group.add(body);

    const arrow = new THREE.Group();
    const paint = new THREE.MeshStandardMaterial({ color: 0xffc24a, roughness: 0.5 });
    const shaft = new THREE.Mesh(new THREE.BoxGeometry(size * 0.42, size * 0.12, 0.2), paint);
    shaft.position.set(-size * 0.08, 0, 0);
    const head = new THREE.Mesh(new THREE.ConeGeometry(size * 0.16, size * 0.26, 3), paint);
    head.rotation.z = -Math.PI / 2;
    head.position.set(size * 0.26, 0, 0);
    arrow.add(shaft);
    arrow.add(head);
    arrow.position.z = size + 0.1;
    group.add(arrow);
    return group;
  }

  #groupNamedFor(alliance) {
    let found = null;
    this.field.traverse((o) => {
      if (!found && !o.isMesh && o.name && new RegExp(alliance + '[\\s_-]*Hive', 'i').test(o.name)) {
        found = o;
      }
    });
    return found;
  }

  #lean(hive, degrees) {
    const turn = THREE.MathUtils.degToRad(hive.rest - degrees);
    const [px, py, pz] = hive.pivot;
    hive.group.matrixAutoUpdate = false;
    hive.group.matrix
        .makeTranslation(px, py, pz)
        .multiply(new THREE.Matrix4().makeRotationY(turn))
        .multiply(new THREE.Matrix4().makeTranslation(-px, -py, -pz));
    hive.group.updateMatrixWorld(true);
  }

  #floorLine(ring, colour, close) {
    const points = close ? ring.concat([ring[0]]) : ring;
    const line = new Float32Array(points.length * 3);
    points.forEach((p, i) => {
      line[i * 3] = p[0];
      line[i * 3 + 1] = p[1];
      line[i * 3 + 2] = OVERLAY_Z;
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(line, 3));
    return new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: colour }));
  }
}

export function ringsOf(op) {
  switch (op.type) {
    case 'POLYLINE':
    case 'POLYGON':
      return op.xPoints.length ? [op.xPoints.map((x, i) => [x, op.yPoints[i]])] : [];
    case 'CIRCLE': {
      const rim = [];
      for (let i = 0; i < 36; i++) {
        const a = i / 36 * 2 * Math.PI;
        rim.push([op.x + op.radius * Math.cos(a), op.y + op.radius * Math.sin(a)]);
      }
      return [rim];
    }
    case 'SPLINE': {
      const curve = [];
      for (let i = 0; i <= 24; i++) {
        const t = i / 24;
        curve.push([((((op.ax * t + op.bx) * t + op.cx) * t + op.dx) * t + op.ex) * t + op.fx,
                    ((((op.ay * t + op.by) * t + op.cy) * t + op.dy) * t + op.ey) * t + op.fy]);
      }
      return [curve];
    }
    default:
      return [];
  }
}
