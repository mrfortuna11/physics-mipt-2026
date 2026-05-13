// Multi-contact SAT for OBB-OBB + body-vs-plane.

function _boxAxesArr(body) {
  const R = quatToMat3(body.q).elements;
  return [
    new THREE.Vector3(R[0], R[3], R[6]),
    new THREE.Vector3(R[1], R[4], R[7]),
    new THREE.Vector3(R[2], R[5], R[8]),
  ];
}

function _projectBoxAxis(body, axis, axes) {
  const h = body.size;
  return Math.abs(axes[0].dot(axis)) * (h.x * 0.5) +
         Math.abs(axes[1].dot(axis)) * (h.y * 0.5) +
         Math.abs(axes[2].dot(axis)) * (h.z * 0.5);
}

function boxVerticesLocalWorld(body) {
  const h = body.size;
  const hx = h.x * 0.5, hy = h.y * 0.5, hz = h.z * 0.5;
  const out = [];
  for (let sx = -1; sx <= 1; sx += 2)
  for (let sy = -1; sy <= 1; sy += 2)
  for (let sz = -1; sz <= 1; sz += 2) {
    const local = new THREE.Vector3(sx*hx, sy*hy, sz*hz);
    out.push({ local, world: body.worldPoint(local) });
  }
  return out;
}

function _pointInsideBox(p, body) {
  const Rt = quatToMat3(body.q).clone().transpose();
  const local = mat3MulVec(Rt, p.clone().sub(body.x));
  const h = body.size;
  return Math.abs(local.x) <= h.x * 0.5 + 1e-4 &&
         Math.abs(local.y) <= h.y * 0.5 + 1e-4 &&
         Math.abs(local.z) <= h.z * 0.5 + 1e-4;
}

function satBoxBoxMulti(A, B) {
  const axesA = _boxAxesArr(A);
  const axesB = _boxAxesArr(B);
  const t = B.x.clone().sub(A.x);

  let bestDepth = Infinity;
  let bestAxis = null;

  const tryAxis = (axis) => {
    const len2 = axis.lengthSq();
    if (len2 < 1e-9) return true;
    const ax = axis.clone().multiplyScalar(1 / Math.sqrt(len2));
    const rA = _projectBoxAxis(A, ax, axesA);
    const rB = _projectBoxAxis(B, ax, axesB);
    const dist = Math.abs(t.dot(ax));
    const overlap = rA + rB - dist;
    if (overlap < 0) return false;
    if (overlap < bestDepth) {
      bestDepth = overlap;
      const dir = t.dot(ax) >= 0 ? 1 : -1;
      bestAxis = ax.clone().multiplyScalar(dir);
    }
    return true;
  };

  for (let i = 0; i < 3; i++) if (!tryAxis(axesA[i].clone())) return null;
  for (let i = 0; i < 3; i++) if (!tryAxis(axesB[i].clone())) return null;
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      const c = new THREE.Vector3().crossVectors(axesA[i], axesB[j]);
      if (!tryAxis(c)) return null;
    }

  const n_AtoB = bestAxis;
  const n_escape = bestAxis.clone().multiplyScalar(-1);
  const Rat = quatToMat3(A.q).clone().transpose();
  const Rbt = quatToMat3(B.q).clone().transpose();
  const vertsA = boxVerticesLocalWorld(A);
  const vertsB = boxVerticesLocalWorld(B);

  const raw = [];
  for (const v of vertsA) {
    if (_pointInsideBox(v.world, B)) {
      const pBworld = v.world.clone().addScaledVector(n_AtoB, -bestDepth);
      raw.push({
        rAlocal: v.local.clone(),
        rBlocal: mat3MulVec(Rbt, pBworld.clone().sub(B.x)),
        normal: n_escape.clone(),
        depth: bestDepth,
      });
    }
  }
  for (const v of vertsB) {
    if (_pointInsideBox(v.world, A)) {
      const pAworld = v.world.clone().addScaledVector(n_AtoB, bestDepth);
      raw.push({
        rAlocal: mat3MulVec(Rat, pAworld.clone().sub(A.x)),
        rBlocal: v.local.clone(),
        normal: n_escape.clone(),
        depth: bestDepth,
      });
    }
  }

  if (raw.length === 0) {
    raw.push({
      rAlocal: mat3MulVec(Rat, bestAxis.clone().multiplyScalar( A.size.x * 0.5)),
      rBlocal: mat3MulVec(Rbt, bestAxis.clone().multiplyScalar(-B.size.x * 0.5)),
      normal: n_escape.clone(),
      depth: bestDepth,
    });
  }

  const Ra = quatToMat3(A.q);
  const final = [];
  for (const c of raw) {
    const pA = mat3MulVec(Ra, c.rAlocal);
    let dup = false;
    for (const e of final) {
      const eA = mat3MulVec(Ra, e.rAlocal);
      if (pA.distanceTo(eA) < 1e-3) { dup = true; break; }
    }
    if (!dup) final.push(c);
    if (final.length >= 4) break;
  }
  return final;
}

function detectBodyVsPlane(body, planeNormal, planeD) {
  const contacts = [];
  const verts = boxVerticesLocalWorld(body);
  for (const v of verts) {
    const dist = v.world.dot(planeNormal) - planeD;
    if (dist < 0) {
      const depth = -dist;
      contacts.push({
        bodyA: body, bodyB: null,
        rAlocal: v.local.clone(), rBlocal: null,
        normal: planeNormal.clone(), depth, lambda: 0,
        _planeNormal: planeNormal, _planeD: planeD,
        _pBworldFixed: v.world.clone(),
      });
    }
  }
  return contacts;
}
