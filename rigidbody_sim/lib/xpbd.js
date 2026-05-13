// XPBD position-level contact + friction (Müller 2020).
// Approach: absolute penetration depth as the constraint magnitude. Solver
// converges body to depth = SLOP (not depth = 0) by using Ceff = C − SLOP in
// the lambda formula. This avoids the resting-body bouncing that "lift to 0"
// would cause, while still correcting accumulated penetration.

const XPBD_SLOP = 0.001;          // resting body sits this deep below contact surface
const XPBD_MAX_DL = 0.05;         // safety clamp on a single iter correction

function snapshotContactPrev(ct) {
  const A = ct.bodyA, B = ct.bodyB;
  const Ra = quatToMat3(A.q);
  ct._prevPA = mat3MulVec(Ra, ct.rAlocal).add(A.x);
  if (B) {
    const Rb = quatToMat3(B.q);
    ct._prevPB = mat3MulVec(Rb, ct.rBlocal).add(B.x);
  } else {
    // Plane contact: pB is the live projection of pA onto the plane.
    ct._prevPB = ct._prevPA.clone().addScaledVector(
      ct._planeNormal, -(ct._prevPA.dot(ct._planeNormal) - ct._planeD));
  }
}

// Compute contact point on body B (or live projection on plane).
function _getPB(ct, pA) {
  if (ct.bodyB) return ct.bodyB.worldPoint(ct.rBlocal);
  // Plane: project pA onto the plane along planeNormal.
  const dist = pA.dot(ct._planeNormal) - ct._planeD;
  return pA.clone().addScaledVector(ct._planeNormal, -dist);
}

function xpbdContactNormal(ct, dt, alpha) {
  const A = ct.bodyA, B = ct.bodyB;
  const alphaT = alpha > 0 ? alpha / (dt * dt) : 0;

  const pA = A.worldPoint(ct.rAlocal);
  const pB = _getPB(ct, pA);
  const n  = ct.normal;
  const C  = pB.clone().sub(pA).dot(n);
  // Solver leaves body at depth = SLOP. Effective magnitude = C − SLOP.
  const Ceff = C - XPBD_SLOP;
  if (Ceff <= 0) return 0;

  let w = 0;
  let rA = null, rB = null, IAi = null, IBi = null;
  if (A.mass > 0) {
    IAi = inertiaWorld(A).Iinv;
    rA  = A.worldR(ct.rAlocal);
    const rxn = new THREE.Vector3().crossVectors(rA, n);
    w += A.invMass + rxn.dot(mat3MulVec(IAi, rxn));
  }
  if (B && B.mass > 0) {
    IBi = inertiaWorld(B).Iinv;
    rB  = B.worldR(ct.rBlocal);
    const rxn = new THREE.Vector3().crossVectors(rB, n);
    w += B.invMass + rxn.dot(mat3MulVec(IBi, rxn));
  }
  if (w < 1e-12) return C;

  const lambdaOld = ct.lambda || 0;
  let dLambda = (Ceff - alphaT * lambdaOld) / (w + alphaT);
  let lambdaNew = Math.max(0, lambdaOld + dLambda);
  dLambda = lambdaNew - lambdaOld;
  if (dLambda > XPBD_MAX_DL) {       // safety clamp against runaway corrections
    dLambda = XPBD_MAX_DL;
    lambdaNew = lambdaOld + dLambda;
  }
  ct.lambda = lambdaNew;
  if (dLambda === 0) return C;

  const P = n.clone().multiplyScalar(dLambda);
  if (A.mass > 0) {
    A.x.addScaledVector(P, A.invMass);
    applyQuatDelta(A.q, mat3MulVec(IAi, new THREE.Vector3().crossVectors(rA, P)));
  }
  if (B && B.mass > 0) {
    B.x.addScaledVector(P, -B.invMass);
    applyQuatDelta(B.q, mat3MulVec(IBi, new THREE.Vector3().crossVectors(rB, P)).multiplyScalar(-1));
  }
  return C;
}

function xpbdContactFriction(ct, muS, muD) {
  if ((ct.lambda || 0) <= 0) return;
  if (!ct._prevPA) return;

  const A = ct.bodyA, B = ct.bodyB;
  const rA = A.worldR(ct.rAlocal);
  const pA = rA.clone().add(A.x);
  let rB = null, pB;
  if (B) {
    rB = B.worldR(ct.rBlocal);
    pB = rB.clone().add(B.x);
  } else {
    pB = ct._prevPB;
  }
  const dpA = pA.clone().sub(ct._prevPA);
  const dpB = pB.clone().sub(ct._prevPB);
  const dp  = dpA.sub(dpB);
  const dpN = dp.dot(ct.normal);
  const dpT = dp.clone().addScaledVector(ct.normal, -dpN);
  const dpTlen = dpT.length();
  if (dpTlen < 1e-9) return;

  const d = Math.max(0, ct.depth);
  const limitS = muS * d;
  const limitD = muD * d;

  let corr;
  if (dpTlen < limitS) corr = dpT.clone().multiplyScalar(-1);
  else                 corr = dpT.clone().multiplyScalar(-limitD / dpTlen);
  const cLen = corr.length();
  if (cLen < 1e-9) return;
  const nDir = corr.clone().multiplyScalar(1 / cLen);

  let w = 0;
  let IAi = null, IBi = null;
  let rxA = null, rxB = null;
  if (A.mass > 0) {
    IAi = inertiaWorld(A).Iinv;
    rxA = new THREE.Vector3().crossVectors(rA, nDir);
    w += A.invMass + rxA.dot(mat3MulVec(IAi, rxA));
  }
  if (B && B.mass > 0) {
    IBi = inertiaWorld(B).Iinv;
    rxB = new THREE.Vector3().crossVectors(rB, nDir);
    w += B.invMass + rxB.dot(mat3MulVec(IBi, rxB));
  }
  if (w < 1e-12) return;
  const dLambda = cLen / w;
  const P = nDir.clone().multiplyScalar(dLambda);

  if (A.mass > 0) {
    A.x.addScaledVector(P, A.invMass);
    applyQuatDelta(A.q, mat3MulVec(IAi, new THREE.Vector3().crossVectors(rA, P)));
  }
  if (B && B.mass > 0) {
    B.x.addScaledVector(P, -B.invMass);
    applyQuatDelta(B.q, mat3MulVec(IBi, new THREE.Vector3().crossVectors(rB, P)).multiplyScalar(-1));
  }
}
