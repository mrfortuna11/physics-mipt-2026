// math + quat helpers (built on global THREE).

function boxInertia(m, a, b, c) {
  const I = new THREE.Matrix3();
  const ix = (m / 12) * (b * b + c * c);
  const iy = (m / 12) * (a * a + c * c);
  const iz = (m / 12) * (a * a + b * b);
  I.set(ix, 0, 0,  0, iy, 0,  0, 0, iz);
  return I;
}

function mat3Inverse(M) {
  const e = M.elements;
  const a = e[0], b = e[3], c = e[6];
  const d = e[1], ee = e[4], f = e[7];
  const g = e[2], h = e[5], i = e[8];
  const A =  (ee * i - f * h);
  const B = -(d  * i - f * g);
  const C =  (d  * h - ee* g);
  const det = a * A + b * B + c * C;
  const inv = 1 / det;
  const R = new THREE.Matrix3();
  R.set(
    A * inv,                  -(b*i - c*h) * inv,    (b*f - c*ee) * inv,
    B * inv,                   (a*i - c*g) * inv,   -(a*f - c*d ) * inv,
    C * inv,                  -(a*h - b*g) * inv,    (a*ee - b*d) * inv
  );
  return R;
}

function mat3MulVec(M, v) {
  const e = M.elements;
  return new THREE.Vector3(
    e[0]*v.x + e[3]*v.y + e[6]*v.z,
    e[1]*v.x + e[4]*v.y + e[7]*v.z,
    e[2]*v.x + e[5]*v.y + e[8]*v.z
  );
}

function rotateInertia(I_body, R3) {
  const tmp = new THREE.Matrix3().multiplyMatrices(R3, I_body);
  const RT = R3.clone().transpose();
  return new THREE.Matrix3().multiplyMatrices(tmp, RT);
}

function quatToMat3(q) {
  const m4 = new THREE.Matrix4().makeRotationFromQuaternion(q);
  const me = m4.elements;
  const m3 = new THREE.Matrix3();
  m3.set(
    me[0], me[4], me[8],
    me[1], me[5], me[9],
    me[2], me[6], me[10]
  );
  return m3;
}

function integrateQuat(q, omegaWorld, dt) {
  const dq = new THREE.Quaternion(omegaWorld.x, omegaWorld.y, omegaWorld.z, 0);
  dq.multiply(q);
  q.x += 0.5 * dt * dq.x;
  q.y += 0.5 * dt * dq.y;
  q.z += 0.5 * dt * dq.z;
  q.w += 0.5 * dt * dq.w;
  q.normalize();
}

function skew(v) {
  const m = new THREE.Matrix3();
  m.set(   0, -v.z,  v.y,
         v.z,    0, -v.x,
        -v.y,  v.x,    0);
  return m;
}

function inertiaWorld(body) {
  const R = quatToMat3(body.q);
  return {
    I:    rotateInertia(body.I_body,    R),
    Iinv: rotateInertia(body.I_body_inv, R),
    R
  };
}

function softParams(hz, zeta, dt) {
  const omega = 2 * Math.PI * hz;
  const a1 = 2 * zeta + omega * dt;
  const a2 = dt * omega * a1;
  const a3 = 1 / (1 + a2);
  return {
    biasRate:     omega / a1,
    massCoeff:    a2 * a3,
    impulseCoeff: a3
  };
}

function applyQuatDelta(q, dq_vec) {
  const qq = new THREE.Quaternion(dq_vec.x, dq_vec.y, dq_vec.z, 0).multiply(q);
  q.x += 0.5 * qq.x;
  q.y += 0.5 * qq.y;
  q.z += 0.5 * qq.z;
  q.w += 0.5 * qq.w;
  q.normalize();
}
