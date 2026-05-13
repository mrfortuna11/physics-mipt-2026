// Part 3 / Part 4 view classes (uses globals from lib/* scripts).

// -------------------------------------------------------------------------
//  Base class: sets up scene, planes, bodies, broadphase dispatch + contact
//  detection. Sub-classes override solveContacts (SI) or step (XPBD).
// -------------------------------------------------------------------------
class Part3View {
  constructor(name, description, numBodies, useSpatialGrid, opts, overlaysHost) {
    opts = opts || {};
    this.name = name;
    this.description = description;
    this.useSpatialGrid = useSpatialGrid;
    this.broadphase = opts.broadphase || (useSpatialGrid ? 'grid' : 'brute');
    this.useFriction = !!opts.useFriction;
    this.frictionGetter = opts.frictionGetter || (() => ({ mus: globalParams.col3Mus, mud: globalParams.col3Mud }));
    this.numBodiesGetter = opts.numBodiesGetter || (() => numBodies);
    this.varied = !!opts.varied;
    this.arenaSize = opts.arenaSize || (useSpatialGrid ? 12 : 6);
    this.rampAngleGetter = opts.rampAngleGetter || null;
    this.rampAngle = opts.rampAngle || 0;
    this.tab = opts.tab || 3;
    this.sapState = makeSAPState();

    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dir = new THREE.DirectionalLight(0xffffff, 0.5);
    dir.position.set(5, 8, 3);
    this.scene.add(dir);

    const arenaW = this.arenaSize;
    const arenaH = 8;
    this.arenaW = arenaW;
    this.arenaH = arenaH;

    const grid = new THREE.GridHelper(arenaW, arenaW, 0x555555, 0x333333);
    grid.position.y = 0;
    this.scene.add(grid);

    const ang = this.rampAngle;
    const floorN = ang > 0
      ? new THREE.Vector3(Math.sin(ang), Math.cos(ang), 0).normalize()
      : new THREE.Vector3(0, 1, 0);
    this.planes = [
      { n: floorN,                            d: 0          },
      { n: new THREE.Vector3( 1, 0, 0),       d: -arenaW / 2 },
      { n: new THREE.Vector3(-1, 0, 0),       d: -arenaW / 2 },
      { n: new THREE.Vector3(0, 0,  1),       d: -arenaW / 2 },
      { n: new THREE.Vector3(0, 0, -1),       d: -arenaW / 2 },
    ];

    if (ang > 0 || this.rampAngleGetter) {
      const w = arenaW * 1.2, depth = arenaW;
      const rampGeom = new THREE.BoxGeometry(w, 0.1, depth);
      const rampMat  = new THREE.MeshStandardMaterial({ color: 0x44556e, roughness: 0.9 });
      this._rampMesh = new THREE.Mesh(rampGeom, rampMat);
      this._rampMesh.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -ang);
      this.scene.add(this._rampMesh);
    }

    this.bodies = [];
    this.meshes = [];
    this.numBodies = numBodies;

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100);
    this.camera.position.set(0, 6, useSpatialGrid ? 18 : 8);
    this.camera.lookAt(0, 2, 0);

    this.label = document.createElement('div');
    this.label.className = 'vp-label';
    overlaysHost.appendChild(this.label);

    this._frameCount = 0;
    this._contactCount = 0;
  }

  spawnBodies() {
    for (const m of this.meshes) { this.scene.remove(m); m.geometry.dispose(); }
    this.meshes = [];
    this.bodies = [];

    const colors = [0x4a9eff, 0x4ade80, 0xfbbf24, 0xf87171, 0xa78bfa,
                    0xff9966, 0x60a5fa, 0x34d399, 0xfcd34d, 0xf9a8d4];

    const isGrid  = this.useSpatialGrid;
    const isVaried = this.varied;
    const isRamp  = this.rampAngle > 0;
    const N = this.numBodiesGetter ? this.numBodiesGetter() : this.numBodies;
    this.numBodies = N;

    for (let i = 0; i < N; i++) {
      let size;
      if (isVaried) {
        const sx = 0.25 + Math.random() * 0.75;
        const sy = 0.25 + Math.random() * 0.75;
        const sz = 0.25 + Math.random() * 0.75;
        size = new THREE.Vector3(sx, sy, sz);
      } else if (isGrid) {
        size = new THREE.Vector3(0.35, 0.35, 0.35);
      } else if (isRamp) {
        const s = 0.4;
        size = new THREE.Vector3(s, s, s);
      } else {
        const s = 0.45 + (Math.random() - 0.5) * 0.2;
        size = new THREE.Vector3(s, s, s);
      }

      const mass = isVaried ? size.x*size.y*size.z*4 : 1.0;
      const body = new RigidBody(mass, size);
      body.useDamping = true;

      if (isRamp) {
        const tx = -2 + (i % 4) * 1.3;
        const ty = 2.0 + Math.floor(i / 4) * 1.0;
        body.x.set(tx, ty, (Math.random() - 0.5) * 0.6);
        body.q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -this.rampAngle);
      } else if (isVaried) {
        const hw = this.arenaW * 0.4;
        body.x.set(
          (Math.random() - 0.5) * hw * 2,
          1.5 + Math.random() * (this.arenaH - 2),
          (Math.random() - 0.5) * hw * 2
        );
        const axis = new THREE.Vector3(Math.random()-0.5, Math.random()-0.5, Math.random()-0.5).normalize();
        body.q.setFromAxisAngle(axis, Math.random() * Math.PI);
      } else if (isGrid) {
        const cols = Math.ceil(Math.sqrt(N));
        const spacing = this.arenaW / cols;
        const col = i % cols;
        const row = Math.floor(i / cols) % cols;
        const layer = Math.floor(i / (cols * cols));
        body.x.set(
          -this.arenaW/2 + spacing*(col+0.5) + (Math.random()-0.5)*spacing*0.3,
          0.35*0.6 + layer * 0.35 * 1.2,
          -this.arenaW/2 + spacing*(row+0.5) + (Math.random()-0.5)*spacing*0.3
        );
        const angle = (Math.random() - 0.5) * 0.3;
        body.q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), angle);
      } else {
        const hw = this.arenaW * 0.4;
        body.x.set(
          (Math.random() - 0.5) * hw * 2,
          0.5 + Math.floor(i / 3) * 0.8 + Math.random() * 0.2,
          (Math.random() - 0.5) * hw * 2
        );
        const angle = (Math.random() - 0.5) * 0.3;
        body.q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), angle);
      }
      body.v.set(0, 0, 0);
      body.omega.set(0, 0, 0);

      this.bodies.push(body);

      const geom = new THREE.BoxGeometry(size.x, size.y, size.z);
      const mat = new THREE.MeshStandardMaterial({ color: colors[i % colors.length], roughness: 0.6 });
      const mesh = new THREE.Mesh(geom, mat);
      mesh.add(new THREE.LineSegments(
        new THREE.EdgesGeometry(geom),
        new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.5 })
      ));
      this.scene.add(mesh);
      this.meshes.push(mesh);
    }
  }

  reset() {
    if (this.rampAngleGetter) {
      this.rampAngle = this.rampAngleGetter();
      const floorN = this.rampAngle > 0
        ? new THREE.Vector3(Math.sin(this.rampAngle), Math.cos(this.rampAngle), 0).normalize()
        : new THREE.Vector3(0, 1, 0);
      this.planes[0].n = floorN;
      if (this._rampMesh) {
        this._rampMesh.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -this.rampAngle);
      }
    }
    this.spawnBodies();
    this._frameCount = 0;
    this._contactCount = 0;
  }

  getContacts() {
    const contacts = [];

    for (const b of this.bodies) bodyComputeAABB(b);

    let pairs;
    if      (this.broadphase === 'grid') pairs = broadphaseGridArr(this.bodies, 1.0);
    else if (this.broadphase === 'sap')  pairs = broadphaseSAP(this.bodies, this.sapState);
    else if (this.broadphase === 'lbvh') pairs = broadphaseLBVH(this.bodies);
    else                                  pairs = broadphaseBrute(this.bodies);
    this._broadphasePairs = pairs.length;

    for (const [i, j] of pairs) {
      const A = this.bodies[i], B = this.bodies[j];
      const list = satBoxBoxMulti(A, B);
      if (!list) continue;
      for (const c of list) {
        contacts.push({
          bodyA: A, bodyB: B,
          rAlocal: c.rAlocal, rBlocal: c.rBlocal,
          normal: c.normal, depth: c.depth,
          lambda: 0,
        });
      }
    }

    for (const b of this.bodies) {
      for (const { n, d } of this.planes) {
        const cs = detectBodyVsPlane(b, n, d);
        for (const c of cs) contacts.push(c);
      }
    }
    return contacts;
  }

  solveContacts(contacts, dt) {}

  // SI step (Part3View_SI overrides solveContacts; base step is used by SI).
  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);

    // 1. velocity predict (gravity + damping)
    for (const b of this.bodies) {
      if (b.mass === 0) continue;
      b.v.addScaledVector(g, dt);
      if (b.useDamping) {
        b.v.multiplyScalar(Math.exp(-globalParams.linDamp * dt));
        b.omega.multiplyScalar(Math.exp(-globalParams.angDamp * dt));
      }
    }

    // 2. detect contacts (at current x)
    const contacts = this.getContacts();
    this._contactCount = contacts.length;

    // 3. velocity-level solve (SI implementations override this)
    this.solveContacts(contacts, dt);

    // 4. integrate positions
    for (const b of this.bodies) {
      if (b.mass === 0) continue;
      b.x.addScaledVector(b.v, dt);
      integrateQuat(b.q, b.omega, dt);
    }
  }

  updateMesh() {
    for (let i = 0; i < this.bodies.length; i++) {
      const b = this.bodies[i];
      const m = this.meshes[i];
      m.position.copy(b.x);
      m.quaternion.copy(b.q);
    }
  }

  updateLabel() {
    this._frameCount++;
    this.label.innerHTML = `
      <div class="title">${this.name}</div>
      тел: ${this.bodies.length} &nbsp; контактов: ${this._contactCount}<br>
      <span class="note">${this.description}</span>
    `;
  }
}

class Part3View_XPBD extends Part3View {
  step(dt) {
    const g = new THREE.Vector3(0, -globalParams.gravity, 0);

    // 1. Apply gravity + implicit gyro + damping once.
    for (const b of this.bodies) {
      if (b.mass === 0) continue;
      const R = quatToMat3(b.q);
      const Rt = R.clone().transpose();
      const omegaBody = mat3MulVec(Rt, b.omega);
      const Ib = b.I_body;
      const w = omegaBody.clone();
      for (let it = 0; it < 3; it++) {
        const Iw = mat3MulVec(Ib, w);
        const cross = new THREE.Vector3().crossVectors(w, Iw);
        const f = mat3MulVec(Ib, new THREE.Vector3().subVectors(w, omegaBody))
                    .add(cross.clone().multiplyScalar(dt));
        if (f.length() < 1e-10) break;
        const Sw = skew(w), SIw = skew(Iw);
        const SwI = new THREE.Matrix3().multiplyMatrices(Sw, Ib);
        const J = new THREE.Matrix3();
        for (let i = 0; i < 9; i++) J.elements[i] = Ib.elements[i] + dt*(SwI.elements[i]-SIw.elements[i]);
        w.add(mat3MulVec(mat3Inverse(J), f).multiplyScalar(-1));
      }
      b.omega.copy(mat3MulVec(R, w));
      b.v.addScaledVector(g, dt);
      if (b.useDamping) {
        b.v.multiplyScalar(Math.exp(-globalParams.linDamp * dt));
        b.omega.multiplyScalar(Math.exp(-globalParams.angDamp * dt));
      }
    }

    // Müller order: predict positions FIRST, then SAT — depth in c.depth
    // reflects the actual penetration after the symplectic-Euler position
    // step. The XPBD solver then converges back to depth = SLOP.
    const alpha = globalParams.col3Alpha;
    const POS_ITERS = Math.max(4, globalParams.col3PosIters * 2);
    const fric = this.useFriction ? this.frictionGetter() : null;

    // 2a. Snapshot prev (so v derives over the full step, not just the solve)
    for (const b of this.bodies) {
      if (b.mass === 0) continue;
      b.prevX = b.x.clone();
      b.prevQ = b.q.clone();
    }
    // 2b. Predict position
    for (const b of this.bodies) {
      if (b.mass === 0) continue;
      b.x.addScaledVector(b.v, dt);
      integrateQuat(b.q, b.omega, dt);
    }
    // 2c. SAT at predicted positions
    const contacts = this.getContacts();
    this._contactCount = contacts.length;
    // 2d. Snapshot for friction (post-predict, pre-solve)
    if (fric) {
      for (const c of contacts) snapshotContactPrev(c);
    }
    // 2e. Reset lambda
    for (const c of contacts) c.lambda = 0;
    // 3e. Position iters (multi-contact GS — need a few iters to converge on face patches)
    for (let it = 0; it < POS_ITERS; it++) {
      for (const c of contacts) {
        xpbdContactNormal(c, dt, alpha);
        if (fric) xpbdContactFriction(c, fric.mus, fric.mud);
      }
    }
    // 3f. Derive v and ω (Müller XPBD canon: v = (x − prev)/dt)
    for (const b of this.bodies) {
      if (b.mass === 0) continue;
      b.v.copy(b.x).sub(b.prevX).divideScalar(dt);
      const qPrevInv = b.prevQ.clone().invert();
      const dq = b.q.clone().multiply(qPrevInv);
      if (dq.w < 0) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; dq.w = -dq.w; }
      const axisLen = Math.hypot(dq.x, dq.y, dq.z);
      if (axisLen > 1e-9) {
        const ang = 2 * Math.atan2(axisLen, dq.w);
        b.omega.set(dq.x / axisLen, dq.y / axisLen, dq.z / axisLen).multiplyScalar(ang / dt);
      } else {
        b.omega.set(0, 0, 0);
      }
    }
  }
}

// -------------------------------------------------------------------------
//  Sequential Impulses + Baumgarte (velocity-level). Used by 3B subtab.
// -------------------------------------------------------------------------
class Part3View_SI extends Part3View {
  solveContacts(contacts, dt) {
    const beta  = globalParams.baumgarte;
    const ITERS = globalParams.col3VelIters;

    for (let iter = 0; iter < ITERS; iter++) {
      for (const c of contacts) {
        const A = c.bodyA, B = c.bodyB;
        const rA = A.worldR(c.rAlocal);
        const rB = B ? B.worldR(c.rBlocal) : new THREE.Vector3();
        const vA = A.mass > 0
          ? A.v.clone().add(new THREE.Vector3().crossVectors(A.omega, rA))
          : new THREE.Vector3();
        const vB = B && B.mass > 0
          ? B.v.clone().add(new THREE.Vector3().crossVectors(B.omega, rB))
          : new THREE.Vector3();
        const vRel = vB.clone().sub(vA);
        const vN = vRel.dot(c.normal);

        let w = 0;
        let IAi = null, IBi = null;
        if (A.mass > 0) {
          IAi = inertiaWorld(A).Iinv;
          const rxn = new THREE.Vector3().crossVectors(rA, c.normal);
          w += A.invMass + rxn.dot(mat3MulVec(IAi, rxn));
        }
        if (B && B.mass > 0) {
          IBi = inertiaWorld(B).Iinv;
          const rxn = new THREE.Vector3().crossVectors(rB, c.normal);
          w += B.invMass + rxn.dot(mat3MulVec(IBi, rxn));
        }
        if (w < 1e-10) continue;

        const bias = Math.min(beta * c.depth / dt, 2.0);
        const dL = (vN + bias) / w;
        const prevL = c.lambda || 0;
        c.lambda = Math.max(0, prevL + dL);
        const actual = c.lambda - prevL;
        if (actual < 1e-12) continue;

        const p = c.normal.clone().multiplyScalar(actual);
        if (A.mass > 0) {
          A.v.addScaledVector(p, A.invMass);
          A.omega.add(mat3MulVec(IAi, new THREE.Vector3().crossVectors(rA, p)));
        }
        if (B && B.mass > 0) {
          B.v.addScaledVector(p, -B.invMass);
          B.omega.sub(mat3MulVec(IBi, new THREE.Vector3().crossVectors(rB, p)));
        }
      }
    }
    for (const b of this.bodies) {
      b.v.multiplyScalar(0.995);
      b.omega.multiplyScalar(0.995);
    }
  }
}

// 3C — many-body pile. Same physics as XPBD, just different scene.
class Part3View_Pile extends Part3View_XPBD {}

// Part 4 — same physics as XPBD (broadphase/scene configured via opts).
class Part4View_XPBD extends Part3View_XPBD {}
