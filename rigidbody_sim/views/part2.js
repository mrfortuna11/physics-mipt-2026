// Part 2A — Spring view
class Part2SpringView {
  constructor(name, description, springMode, color, overlaysHost) {
    this.name = name;
    this.description = description;
    this.springMode = springMode;
    this.tab = 2;

    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(3, 5, 2);
    this.scene.add(dir);

    const axes = new THREE.AxesHelper(0.5);
    axes.material.depthTest = false;
    axes.material.transparent = true;
    axes.material.opacity = 0.5;
    this.scene.add(axes);

    this.bodySize = new THREE.Vector3(0.6, 0.4, 0.4);
    this.body = new RigidBody(1.0, this.bodySize);
    this.body.useDamping = true;

    const geom = new THREE.BoxGeometry(this.bodySize.x, this.bodySize.y, this.bodySize.z);
    const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.2 });
    this.mesh = new THREE.Mesh(geom, mat);
    this.scene.add(this.mesh);
    const edges = new THREE.EdgesGeometry(geom);
    this.mesh.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x000000 })));

    this.anchorWorld = new THREE.Vector3(0, 1.5, 0);
    const anchorMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.06, 12, 12),
      new THREE.MeshBasicMaterial({ color: 0x4ade80 })
    );
    anchorMesh.position.copy(this.anchorWorld);
    this.scene.add(anchorMesh);

    this.attachLocal = new THREE.Vector3(this.bodySize.x * 0.5, this.bodySize.y * 0.5, 0);
    this.attachMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.05, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0xfbbf24 })
    );
    this.scene.add(this.attachMesh);

    const lineGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.springLine = new THREE.Line(lineGeom, new THREE.LineBasicMaterial({ color: 0xa78bfa }));
    this.scene.add(this.springLine);

    const grid = new THREE.GridHelper(6, 12, 0x444444, 0x2a2a2a);
    grid.position.y = -2.0;
    this.scene.add(grid);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.1, 50);
    this.camera.position.set(3.5, 1.0, 3.5);
    this.camera.lookAt(0, 0, 0);

    this.label = document.createElement('div');
    this.label.className = 'vp-label';
    overlaysHost.appendChild(this.label);

    this.lambdaAcc = new THREE.Vector3();
  }

  reset() {
    this.body.x.set(0, 0.8, 0);
    this.body.v.set(0, 0, 0);
    this.body.q.identity();
    this.body.omega.set(0, 0, 0);
    this.lambdaAcc.set(0, 0, 0);
    this._E0 = null;
    this.updateMesh();
  }

  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);

    if (this.springMode === 'force') {
      const k = globalParams.springK;
      const c = globalParams.springC;
      const externalFn = (body) => {
        const rW      = body.worldR(this.attachLocal);
        const attachW = body.x.clone().add(rW);
        const vAttach = body.pointVelocity(rW);
        const F = attachW.clone().sub(this.anchorWorld).multiplyScalar(-k)
                     .addScaledVector(vAttach, -c);
        const tau = new THREE.Vector3().crossVectors(rW, F);
        return { force: F, torque: tau };
      };
      stepSI_implicit(this.body, dt, externalFn, g);
    } else {
      const body = this.body;
      const x_prev = body.x.clone();
      const q_prev = body.q.clone();
      this._xpbdLambda = 0;
      predictVelocity(body, dt, null, g);
      integratePositions(body, dt);
      this.applySoftAnchor(dt);
      body.v.copy(body.x).sub(x_prev).multiplyScalar(1 / dt);
      const dq = body.q.clone().multiply(q_prev.invert());
      body.omega.set(2 * dq.x / dt, 2 * dq.y / dt, 2 * dq.z / dt);
      if (dq.w < 0) body.omega.multiplyScalar(-1);
    }
  }

  applySoftAnchor(dt) {
    const body = this.body;
    const omega = 2 * Math.PI * globalParams.softHz;
    const compliance = 1 / (body.mass * omega * omega);
    const alphaTilde = compliance / (dt * dt);

    const { Iinv } = inertiaWorld(body);
    const rW = body.worldR(this.attachLocal);
    const attachW = body.x.clone().add(rW);
    const dx = attachW.clone().sub(this.anchorWorld);
    const c = dx.length();
    if (c < 1e-10) return;
    const n = dx.clone().multiplyScalar(1 / c);
    const rxn = new THREE.Vector3().crossVectors(rW, n);
    const w = body.invMass + rxn.dot(mat3MulVec(Iinv, rxn));
    const lambdaPrev = this._xpbdLambda || 0;
    const dLambda = (-c - alphaTilde * lambdaPrev) / (w + alphaTilde);
    this._xpbdLambda = lambdaPrev + dLambda;
    const p = n.clone().multiplyScalar(dLambda);
    body.x.addScaledVector(p, body.invMass);
    const dq_axis = mat3MulVec(Iinv, new THREE.Vector3().crossVectors(rW, p));
    const dq = new THREE.Quaternion(dq_axis.x, dq_axis.y, dq_axis.z, 0);
    dq.multiply(body.q);
    body.q.x += 0.5 * dq.x;
    body.q.y += 0.5 * dq.y;
    body.q.z += 0.5 * dq.z;
    body.q.w += 0.5 * dq.w;
    body.q.normalize();
  }

  updateMesh() {
    this.mesh.position.copy(this.body.x);
    this.mesh.quaternion.copy(this.body.q);
    const attachW = this.body.worldPoint(this.attachLocal);
    this.attachMesh.position.copy(attachW);
    const pos = this.springLine.geometry.attributes.position;
    pos.setXYZ(0, this.anchorWorld.x, this.anchorWorld.y, this.anchorWorld.z);
    pos.setXYZ(1, attachW.x, attachW.y, attachW.z);
    pos.needsUpdate = true;
  }

  updateLabel() {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);
    const E = this.body.getEnergy(g);
    if (this._E0 === null || this._E0 === undefined) this._E0 = E;
    const attachW = this.body.worldPoint(this.attachLocal);
    const dist = attachW.clone().sub(this.anchorWorld).length();

    this.label.innerHTML = `
      <div class="title">${this.name}</div>
      |attach − anchor|: ${dist.toFixed(3)} м<br>
      y центра: ${this.body.x.y.toFixed(3)} м<br>
      |v|: ${this.body.v.length().toFixed(3)} &nbsp; |ω|: ${this.body.omega.length().toFixed(3)}<br>
      E: ${E.toFixed(3)} &nbsp; ΔE: ${(E - this._E0).toFixed(3)}<br>
      <span class="note">${this.description}</span>
    `;
  }
}

// =========================================================================
//  Part 2B — Distance constraint base + 4 solvers
// =========================================================================
class Part2DistView {
  constructor(name, description, color, overlaysHost) {
    this.name = name;
    this.description = description;
    this.tab = 2;

    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(3, 5, 2);
    this.scene.add(dir);

    const axes = new THREE.AxesHelper(0.4);
    axes.material.depthTest = false;
    axes.material.transparent = true;
    axes.material.opacity = 0.4;
    this.scene.add(axes);

    this.body1Size = new THREE.Vector3(0.5, 0.3, 0.3);
    this.body1 = new RigidBody(0, this.body1Size);
    this.body1.x.set(0, 1.5, 0);

    this.body2Size = new THREE.Vector3(0.5, 0.3, 0.3);
    this.body2 = new RigidBody(1.0, this.body2Size);
    this.body2.useDamping = true;

    this.r1Local = new THREE.Vector3( this.body1Size.x * 0.5, -this.body1Size.y * 0.5, 0);
    this.r2Local = new THREE.Vector3(-this.body2Size.x * 0.5,  this.body2Size.y * 0.5, 0);

    this.restLength = 0.8;

    const mat1 = new THREE.MeshStandardMaterial({ color: 0x888888, roughness: 0.6 });
    this.mesh1 = new THREE.Mesh(new THREE.BoxGeometry(...this.body1Size.toArray()), mat1);
    this.mesh1.position.copy(this.body1.x);
    this.scene.add(this.mesh1);
    this.mesh1.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(this.mesh1.geometry),
      new THREE.LineBasicMaterial({ color: 0x000000 })
    ));

    const mat2 = new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.2 });
    this.mesh2 = new THREE.Mesh(new THREE.BoxGeometry(...this.body2Size.toArray()), mat2);
    this.scene.add(this.mesh2);
    this.mesh2.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(this.mesh2.geometry),
      new THREE.LineBasicMaterial({ color: 0x000000 })
    ));

    this.attach1Mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.04, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0xfbbf24 })
    );
    this.scene.add(this.attach1Mesh);
    this.attach2Mesh = this.attach1Mesh.clone();
    this.scene.add(this.attach2Mesh);

    const lineGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.constraintLine = new THREE.Line(lineGeom, new THREE.LineBasicMaterial({ color: 0xa78bfa }));
    this.scene.add(this.constraintLine);

    const grid = new THREE.GridHelper(6, 12, 0x444444, 0x2a2a2a);
    grid.position.y = -2.0;
    this.scene.add(grid);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.1, 50);
    this.camera.position.set(3.5, 0.5, 3.5);
    this.camera.lookAt(0, 0.5, 0);

    this.label = document.createElement('div');
    this.label.className = 'vp-label';
    overlaysHost.appendChild(this.label);

    this.lambda = 0;
  }

  reset() {
    this.body1.x.set(0, 1.5, 0);
    this.body1.q.identity();
    this.body2.x.set(0.5, 0.4, 0);
    this.body2.v.set(0, 0, 0);
    this.body2.q.identity();
    this.body2.omega.set(0, 0, 0);
    this.lambda = 0;
    this._E0 = null;
    this.updateMesh();
  }

  getConstraintGeom() {
    const r1W = this.body1.worldR(this.r1Local);
    const r2W = this.body2.worldR(this.r2Local);
    const p1 = this.body1.x.clone().add(r1W);
    const p2 = this.body2.x.clone().add(r2W);
    const d = p2.clone().sub(p1);
    const len = d.length();
    const n = len > 1e-10 ? d.clone().multiplyScalar(1 / len) : new THREE.Vector3(0, -1, 0);
    const c = len - this.restLength;
    return { p1, p2, r1W, r2W, n, c, len };
  }

  computeW(r1W, r2W, n) {
    const { Iinv: Iinv1 } = inertiaWorld(this.body1);
    const { Iinv: Iinv2 } = inertiaWorld(this.body2);
    const rxn1 = new THREE.Vector3().crossVectors(r1W, n);
    const rxn2 = new THREE.Vector3().crossVectors(r2W, n);
    const w1 = this.body1.invMass + rxn1.dot(mat3MulVec(Iinv1, rxn1));
    const w2 = this.body2.invMass + rxn2.dot(mat3MulVec(Iinv2, rxn2));
    return { w1, w2, w: w1 + w2, Iinv1, Iinv2 };
  }

  applyPositionImpulse(dLambda, r1W, r2W, n, Iinv1, Iinv2) {
    const p = n.clone().multiplyScalar(dLambda);
    if (this.body1.mass > 0) {
      this.body1.x.addScaledVector(p, -this.body1.invMass);
      const dq_axis = mat3MulVec(Iinv1, new THREE.Vector3().crossVectors(r1W, p)).multiplyScalar(-1);
      const dq = new THREE.Quaternion(dq_axis.x, dq_axis.y, dq_axis.z, 0);
      dq.multiply(this.body1.q);
      this.body1.q.x += 0.5 * dq.x; this.body1.q.y += 0.5 * dq.y;
      this.body1.q.z += 0.5 * dq.z; this.body1.q.w += 0.5 * dq.w;
      this.body1.q.normalize();
    }
    if (this.body2.mass > 0) {
      this.body2.x.addScaledVector(p,  this.body2.invMass);
      const dq_axis = mat3MulVec(Iinv2, new THREE.Vector3().crossVectors(r2W, p));
      const dq = new THREE.Quaternion(dq_axis.x, dq_axis.y, dq_axis.z, 0);
      dq.multiply(this.body2.q);
      this.body2.q.x += 0.5 * dq.x; this.body2.q.y += 0.5 * dq.y;
      this.body2.q.z += 0.5 * dq.z; this.body2.q.w += 0.5 * dq.w;
      this.body2.q.normalize();
    }
  }

  applyVelocityImpulse(dLambda, r1W, r2W, n, Iinv1, Iinv2) {
    const p = n.clone().multiplyScalar(dLambda);
    if (this.body1.mass > 0) {
      this.body1.v.addScaledVector(p, -this.body1.invMass);
      this.body1.omega.add(mat3MulVec(Iinv1, new THREE.Vector3().crossVectors(r1W, p)).multiplyScalar(-1));
    }
    if (this.body2.mass > 0) {
      this.body2.v.addScaledVector(p,  this.body2.invMass);
      this.body2.omega.add(mat3MulVec(Iinv2, new THREE.Vector3().crossVectors(r2W, p)));
    }
  }

  getRelativeNormalVelocity(r1W, r2W, n) {
    const v1p = this.body1.pointVelocity(r1W);
    const v2p = this.body2.pointVelocity(r2W);
    return v2p.sub(v1p).dot(n);
  }

  step(dt) { throw new Error('Part2DistView.step must be overridden'); }

  updateMesh() {
    this.mesh1.position.copy(this.body1.x);
    this.mesh1.quaternion.copy(this.body1.q);
    this.mesh2.position.copy(this.body2.x);
    this.mesh2.quaternion.copy(this.body2.q);
    const { p1, p2 } = this.getConstraintGeom();
    this.attach1Mesh.position.copy(p1);
    this.attach2Mesh.position.copy(p2);
    const pos = this.constraintLine.geometry.attributes.position;
    pos.setXYZ(0, p1.x, p1.y, p1.z);
    pos.setXYZ(1, p2.x, p2.y, p2.z);
    pos.needsUpdate = true;
  }

  updateLabel() {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);
    const E = this.body2.getEnergy(g);
    if (this._E0 === null || this._E0 === undefined) this._E0 = E;
    const { c, len } = this.getConstraintGeom();
    const errRel = this.restLength > 1e-8 ? (c / this.restLength * 100) : 0;
    this.label.innerHTML = `
      <div class="title">${this.name}</div>
      длина: ${len.toFixed(3)} (rest ${this.restLength.toFixed(2)})<br>
      ошибка C: <span style="color:${Math.abs(errRel)>2?'#f87171':'#4ade80'}">${errRel.toFixed(2)}%</span><br>
      |v|: ${this.body2.v.length().toFixed(3)} &nbsp; |ω|: ${this.body2.omega.length().toFixed(3)}<br>
      E: ${E.toFixed(3)} &nbsp; ΔE: ${(E - this._E0).toFixed(3)}<br>
      <span class="note">${this.description}</span>
    `;
  }
}

class Part2DistView_XPBD extends Part2DistView {
  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);
    const body2 = this.body2;
    const x_prev = body2.x.clone();
    const q_prev = body2.q.clone();
    this.lambda = 0;
    predictVelocity(body2, dt, null, g);
    integratePositions(body2, dt);
    const compliance = globalParams.distCompliance;
    const alphaTilde = compliance / (dt * dt);
    const { r1W, r2W, n, c } = this.getConstraintGeom();
    const { w, Iinv1, Iinv2 } = this.computeW(r1W, r2W, n);
    const dLambda = (-c - alphaTilde * this.lambda) / (w + alphaTilde);
    this.lambda += dLambda;
    this.applyPositionImpulse(dLambda, r1W, r2W, n, Iinv1, Iinv2);
    body2.v.copy(body2.x).sub(x_prev).multiplyScalar(1 / dt);
    const dq = body2.q.clone().multiply(q_prev.invert());
    body2.omega.set(2 * dq.x / dt, 2 * dq.y / dt, 2 * dq.z / dt);
    if (dq.w < 0) body2.omega.multiplyScalar(-1);
  }
}

class Part2DistView_SI_Baumgarte extends Part2DistView {
  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);
    predictVelocity(this.body2, dt, null, g);
    const { r1W, r2W, n, c } = this.getConstraintGeom();
    const { w, Iinv1, Iinv2 } = this.computeW(r1W, r2W, n);
    const beta = globalParams.baumgarte;
    this.applyVelocityImpulse(this.lambda, r1W, r2W, n, Iinv1, Iinv2);
    const NUM_ITER = globalParams.siIters;
    for (let iter = 0; iter < NUM_ITER; iter++) {
      const vN = this.getRelativeNormalVelocity(r1W, r2W, n);
      const dLambda = (-vN - beta * c) / w;
      const prevLambda = this.lambda;
      this.lambda += dLambda;
      this.applyVelocityImpulse(this.lambda - prevLambda, r1W, r2W, n, Iinv1, Iinv2);
    }
    integratePositions(this.body2, dt);
  }
}

class Part2DistView_SI_NGS extends Part2DistView {
  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);
    predictVelocity(this.body2, dt, null, g);
    const NUM_ITER = globalParams.siIters;
    for (let iter = 0; iter < NUM_ITER; iter++) {
      const { r1W, r2W, n } = this.getConstraintGeom();
      const { w, Iinv1, Iinv2 } = this.computeW(r1W, r2W, n);
      const vN = this.getRelativeNormalVelocity(r1W, r2W, n);
      const dLambda = -vN / w;
      this.applyVelocityImpulse(dLambda, r1W, r2W, n, Iinv1, Iinv2);
    }
    integratePositions(this.body2, dt);
    const NUM_POS_ITER = globalParams.posIters;
    for (let iter = 0; iter < NUM_POS_ITER; iter++) {
      const { r1W, r2W, n, c } = this.getConstraintGeom();
      if (Math.abs(c) < 1e-6) break;
      const { w, Iinv1, Iinv2 } = this.computeW(r1W, r2W, n);
      const dLambda = -c / w;
      this.applyPositionImpulse(dLambda, r1W, r2W, n, Iinv1, Iinv2);
    }
  }
}

class Part2DistView_SI_Soft extends Part2DistView {
  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);
    predictVelocity(this.body2, dt, null, g);
    const sp = softParams(globalParams.softHz, globalParams.softZeta, dt);
    this.lambda = 0;
    const NUM_ITER = globalParams.siIters;
    for (let iter = 0; iter < NUM_ITER; iter++) {
      const { r1W, r2W, n, c } = this.getConstraintGeom();
      const { w, Iinv1, Iinv2 } = this.computeW(r1W, r2W, n);
      const meff = 1 / w;
      const vN = this.getRelativeNormalVelocity(r1W, r2W, n);
      const dLambda = -sp.massCoeff * meff * (vN + sp.biasRate * c)
                      - sp.impulseCoeff * this.lambda;
      this.lambda += dLambda;
      this.applyVelocityImpulse(dLambda, r1W, r2W, n, Iinv1, Iinv2);
    }
    integratePositions(this.body2, dt);
  }
}
