import * as THREE from 'three';

export const LENS = {
  width: 640,
  height: 480,
  fx: 622.0,
  fy: 622.0,
  cx: 320.0,
  cy: 240.0,

  rightIn: 0.75,
  forwardIn: 4,
  upIn: 16,
  pitchDegrees: -68
};

const TAG_ARTWORK = {
  blueaudience: 'textures/GoalAprilTag_blueaudience.png',
  bluescoring: 'textures/GoalAprilTag_bluescoring.png',
  redaudience: 'textures/GoalAprilTag_redaudience.png',
  redscoring: 'textures/GoalAprilTag_redscoring.png'
};

const GOAL_TAG = /april[\s_-]*tag/i;

function lensProjection(lens, near, far) {
  const m = new THREE.Matrix4();
  m.set(
      2 * lens.fx / lens.width, 0, (lens.width - 2 * lens.cx) / lens.width, 0,
      0, 2 * lens.fy / lens.height, -(lens.height - 2 * lens.cy) / lens.height, 0,
      0, 0, -(far + near) / (far - near), -2 * far * near / (far - near),
      0, 0, -1, 0);
  return m;
}

function toHive(hive, point) {
  const rest = THREE.MathUtils.degToRad(hive.tilt);
  const c = Math.cos(rest);
  const sn = Math.sin(rest);
  const dx = point.x - hive.pivot[0];
  const dy = point.y - hive.pivot[1];
  const dz = point.z - hive.pivot[2];
  return new THREE.Vector3(dx * c + dz * sn, dy, -dx * sn + dz * c);
}

function fromHive(hive, point) {
  const rest = THREE.MathUtils.degToRad(hive.tilt);
  const c = Math.cos(rest);
  const sn = Math.sin(rest);
  return new THREE.Vector3(
      hive.pivot[0] + point[0] * c - point[2] * sn,
      hive.pivot[1] + point[1],
      hive.pivot[2] + point[0] * sn + point[2] * c);
}

export function theGoalTags(view, model, assets, onProblem) {
  const said = onProblem || (() => {});
  const quads = [];
  let pending = 0;
  let drawable = null;
  const ready = new Promise((resolve) => {
    drawable = resolve;
  });

  function plateBottom(hive, group, side) {
    let found = null;
    (group || view.field).traverse((o) => {
      if (o.isMesh && GOAL_TAG.test(o.name) && new RegExp(side, 'i').test(o.name)) {
        found = o;
      }
    });
    if (!found) {
      return null;
    }
    const points = found.geometry.attributes.position;
    const point = new THREE.Vector3();
    let lowest = Infinity;
    for (let i = 0; i < points.count; i++) {
      point.fromBufferAttribute(points, i).applyMatrix4(found.matrixWorld);
      lowest = Math.min(lowest, toHive(hive, point).z);
    }
    return lowest;
  }

  function makeTags(loader) {
    for (const hive of model.hives) {
      const group = view.hives.find((h) => h.alliance === hive.alliance);
      for (const part of hive.parts) {
        if (!GOAL_TAG.test(part.name)) {
          continue;
        }
        const side = /scoring/i.test(part.name) ? 'scoring' : 'audience';
        const image = TAG_ARTWORK[hive.alliance.toLowerCase() + side];
        if (!image) {
          said('No artwork for ' + hive.alliance + ' ' + side + '.');
          continue;
        }

        const xs = part.vertices.map((v) => v[0]);
        const ys = part.vertices.map((v) => v[1]);
        const zs = part.vertices.map((v) => v[2]);
        const acrossBeam = Math.max(...ys) - Math.min(...ys);
        const alongBeam = Math.max(...xs) - Math.min(...xs);

        const drawn = plateBottom(hive, group ? group.group : null, side);
        const under = (drawn === null ? Math.min(...zs) : drawn) - 0.01;

        pending++;
        const texture = loader.load(assets + image, () => {
          if (--pending === 0) {
            drawable();
          }
        }, undefined, () => {
          said('The tag artwork ' + image + ' did not load.');
          if (--pending === 0) {
            drawable();
          }
        });
        texture.colorSpace = THREE.SRGBColorSpace;
        texture.anisotropy = view.renderer.capabilities.getMaxAnisotropy();

        const long = Math.min(acrossBeam, alongBeam * 3);
        const quad = new THREE.Mesh(
            new THREE.PlaneGeometry(long, alongBeam),
            new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide, toneMapped: false }));

        const facing = new THREE.Matrix4().makeBasis(
            new THREE.Vector3(0, 1, 0), new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, -1));
        quad.quaternion.setFromRotationMatrix(
            new THREE.Matrix4().makeRotationY(-THREE.MathUtils.degToRad(hive.tilt)).multiply(facing));
        quad.position.copy(fromHive(hive, [
          (Math.max(...xs) + Math.min(...xs)) / 2,
          (Math.max(...ys) + Math.min(...ys)) / 2,
          under
        ]));
        quad.name = 'tag ' + hive.alliance + ' ' + side;

        quad.userData.tag = true;
        quad.userData.plate = drawn;
        quad.renderOrder = 2;
        (group ? group.group : view.scene).add(quad);
        quads.push(quad);
      }
    }
  }

  function hideEverythingElse() {
    view.field.traverse((o) => {
      if (o.isMesh && !o.userData.tag) {
        o.visible = false;
      }
    });
    for (const ball of view.movingBalls.concat(view.staticBalls)) {
      ball.visible = false;
    }
    if (view.robot) {
      view.robot.visible = false;
    }
    view.floor.visible = false;
    view.tiles.visible = false;
    if (view.trail) {
      view.trail.visible = false;
    }
    view.overlay.visible = false;
  }

  makeTags(new THREE.TextureLoader());
  if (pending === 0) {
    drawable();
  }

  return {
    ready: ready,
    hideEverythingElse: hideEverythingElse,
    get tags() {
      return quads.map((quad) => quad.name);
    },
    get placedAgainstGeometry() {
      return quads.map((quad) => quad.userData.plate !== null && quad.userData.plate !== undefined);
    }
  };
}

export function theWebcamsView(view, tags) {
  const camera = new THREE.PerspectiveCamera();
  camera.projectionMatrix.copy(lensProjection(LENS, 1, 4000));
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
  let pose = { x: 0, y: 0, heading: 0 };

  function aim(tick) {
    pose = tick || pose;
    const h = pose.heading;
    const forward = new THREE.Vector3(Math.cos(h), Math.sin(h), 0);
    const right = new THREE.Vector3(Math.sin(h), -Math.cos(h), 0);
    camera.position.set(
        pose.x + right.x * LENS.rightIn + forward.x * LENS.forwardIn,
        pose.y + right.y * LENS.rightIn + forward.y * LENS.forwardIn,
        LENS.upIn);

    const up = THREE.MathUtils.degToRad(LENS.pitchDegrees + 90);
    const look = new THREE.Vector3(
        forward.x * Math.cos(up), forward.y * Math.cos(up), Math.sin(up));
    camera.up.set(0, 0, 1);
    camera.lookAt(camera.position.clone().add(look));
  }

  view.background(0x000000);
  tags.hideEverythingElse();
  aim(null);

  return { camera: camera, lens: LENS, aim: aim, hideEverythingElse: tags.hideEverythingElse };
}
