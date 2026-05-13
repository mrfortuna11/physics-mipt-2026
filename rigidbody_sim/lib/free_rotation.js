// Part 1 — integrators of free rigid-body rotation.

function stepGlobalConstL(body, dt) {
  integrateQuat(body.q, body.omega, dt);
  const { Iinv } = inertiaWorld(body);
  body.omega.copy(mat3MulVec(Iinv, body.L0));
}

function stepBodyNoGyro(body, dt) {
  const R = quatToMat3(body.q);
  const Rt = R.clone().transpose();
  const omegaBody = mat3MulVec(Rt, body.omega);
  body.omega.copy(mat3MulVec(R, omegaBody));
  integrateQuat(body.q, body.omega, dt);
}

function stepBodyExplicitGyro(body, dt) {
  const R = quatToMat3(body.q);
  const Rt = R.clone().transpose();
  const omegaBody = mat3MulVec(Rt, body.omega);
  const Iw = mat3MulVec(body.I_body, omegaBody);
  const cross = new THREE.Vector3().crossVectors(omegaBody, Iw);
  const domega = mat3MulVec(body.I_body_inv, cross.multiplyScalar(-1));
  omegaBody.addScaledVector(domega, dt);
  body.omega.copy(mat3MulVec(R, omegaBody));
  integrateQuat(body.q, body.omega, dt);
}

function stepBodyImplicitGyro(body, dt) {
  const R = quatToMat3(body.q);
  const Rt = R.clone().transpose();
  const omegaBody = mat3MulVec(Rt, body.omega);
  const Ib = body.I_body;
  const w = omegaBody.clone();

  for (let it = 0; it < 3; it++) {
    const Iw = mat3MulVec(Ib, w);
    const cross = new THREE.Vector3().crossVectors(w, Iw);
    const f = mat3MulVec(Ib, new THREE.Vector3().subVectors(w, omegaBody))
                .add(cross.clone().multiplyScalar(dt));
    if (f.length() < 1e-10) break;
    const Sw  = skew(w);
    const SIw = skew(Iw);
    const SwI = new THREE.Matrix3().multiplyMatrices(Sw, Ib);
    const J = new THREE.Matrix3();
    for (let i = 0; i < 9; i++) {
      J.elements[i] = Ib.elements[i] + dt * (SwI.elements[i] - SIw.elements[i]);
    }
    const Jinv = mat3Inverse(J);
    const dw = mat3MulVec(Jinv, f).multiplyScalar(-1);
    w.add(dw);
  }
  body.omega.copy(mat3MulVec(R, w));
  integrateQuat(body.q, body.omega, dt);
}
