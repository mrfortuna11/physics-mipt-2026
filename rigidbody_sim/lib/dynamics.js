// Part 2 dynamics: predict step (symplectic Euler + implicit gyro + damping).

function predictVelocity(body, dt, externalFn, gravity) {
  if (body.mass === 0) return;

  const ext = externalFn ? externalFn(body) : { force: new THREE.Vector3(), torque: new THREE.Vector3() };
  ext.force.addScaledVector(gravity, body.mass);

  body.v.addScaledVector(ext.force, dt * body.invMass);

  if (body.useDamping) {
    const ld = globalParams.linDamp;
    const ad = globalParams.angDamp;
    if (ld > 0) body.v.multiplyScalar(Math.exp(-ld * dt));
    if (ad > 0) body.omega.multiplyScalar(Math.exp(-ad * dt));
  }

  const R  = quatToMat3(body.q);
  const Rt = R.clone().transpose();
  const omegaBody = mat3MulVec(Rt, body.omega);
  const tauBody   = mat3MulVec(Rt, ext.torque);

  const Ib = body.I_body;
  const w  = omegaBody.clone();
  for (let it = 0; it < 3; it++) {
    const Iw = mat3MulVec(Ib, w);
    const cross = new THREE.Vector3().crossVectors(w, Iw);
    const f = mat3MulVec(Ib, new THREE.Vector3().subVectors(w, omegaBody))
                .add(cross.clone().multiplyScalar(dt))
                .addScaledVector(tauBody, -dt);
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
}

function integratePositions(body, dt) {
  if (body.mass === 0) return;
  body.x.addScaledVector(body.v, dt);
  integrateQuat(body.q, body.omega, dt);
}

function stepSI_implicit(body, dt, externalFn, gravity) {
  predictVelocity(body, dt, externalFn, gravity);
  integratePositions(body, dt);
}
