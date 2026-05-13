class Part1View {
  constructor(name, description, integrator, color, overlaysHost) {
    this.name = name;
    this.description = description;
    this.integrator = integrator;
    this.tab = 1;

    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(3, 5, 2);
    this.scene.add(dir);

    const axes = new THREE.AxesHelper(0.6);
    axes.material.depthTest = false;
    axes.material.transparent = true;
    axes.material.opacity = 0.6;
    this.scene.add(axes);

    this.bodySize = new THREE.Vector3(0.4, 0.8, 1.2);
    this.body = new RigidBody(1.0, this.bodySize);

    const geom = new THREE.BoxGeometry(this.bodySize.x, this.bodySize.y, this.bodySize.z);
    const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.2 });
    this.mesh = new THREE.Mesh(geom, mat);
    this.scene.add(this.mesh);
    const edges = new THREE.EdgesGeometry(geom);
    this.mesh.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x000000 })));

    this.arrL0 = new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(), 1, 0x4ade80, 0.20, 0.12);
    this.arrL  = new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(), 1, 0xf87171, 0.16, 0.10);
    this.arrW  = new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(), 1, 0xfbbf24, 0.13, 0.08);
    for (const arr of [this.arrL0, this.arrL, this.arrW]) {
      arr.line.material.depthTest = false;
      arr.line.material.transparent = true;
      arr.cone.material.depthTest = false;
      arr.cone.material.transparent = true;
      arr.renderOrder = 999;
      arr.line.renderOrder = 999;
      arr.cone.renderOrder = 999;
    }
    this.scene.add(this.arrL0, this.arrL, this.arrW);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.1, 50);
    this.camera.position.set(4, 3, 5);
    this.camera.lookAt(0, 0, 0);

    this.label = document.createElement('div');
    this.label.className = 'vp-label';
    overlaysHost.appendChild(this.label);
  }

  applyInitial(q0, omega0) {
    this.body.resetFree(q0, omega0);
    this._E0_init = null;
    this.updateMesh();
  }

  step(dt) { this.integrator(this.body, dt); }

  updateMesh() {
    this.mesh.quaternion.copy(this.body.q);
    const L = this.body.getL();
    const Lmag = L.length();
    const wMag = this.body.omega.length();
    const L0mag = this.body.L0.length();
    const ARR_LEN = 1.2;

    this.arrL0.position.set(0,0,0);
    if (L0mag > 1e-8) {
      this.arrL0.setDirection(this.body.L0.clone().normalize());
      this.arrL0.setLength(ARR_LEN * 1.15, 0.20, 0.12);
    }
    this.arrL.position.set(0,0,0);
    if (Lmag > 1e-8) {
      this.arrL.setDirection(L.clone().normalize());
      this.arrL.setLength(ARR_LEN * 0.85, 0.16, 0.10);
    }
    this.arrW.position.set(0,0,0);
    if (wMag > 1e-8) {
      this.arrW.setDirection(this.body.omega.clone().normalize());
      this.arrW.setLength(ARR_LEN * 0.6, 0.13, 0.08);
    }
  }

  updateLabel() {
    const L = this.body.getL();
    const E = 0.5 * this.body.omega.dot(L);
    const Lmag = L.length();
    const L0mag = this.body.L0.length();
    const dL = L.clone().sub(this.body.L0).length();
    const dL_rel = L0mag > 1e-8 ? (dL / L0mag * 100) : 0;
    let angleDeg = 0;
    if (Lmag > 1e-8 && L0mag > 1e-8) {
      const cos = L.dot(this.body.L0) / (Lmag * L0mag);
      angleDeg = Math.acos(Math.max(-1, Math.min(1, cos))) * 180 / Math.PI;
    }
    if (this._E0_init === null || this._E0_init === undefined) this._E0_init = E;
    const dE_rel = Math.abs(this._E0_init) > 1e-8 ? ((E - this._E0_init) / this._E0_init * 100) : 0;

    this.label.innerHTML = `
      <div class="title">${this.name}</div>
      |L|: ${Lmag.toFixed(4)}  &nbsp; Δ|L−L₀|: <span style="color:${dL_rel>1?'#f87171':'#4ade80'}">${dL_rel.toFixed(2)}%</span><br>
      ∠(L,L₀): <span style="color:${angleDeg>1?'#f87171':'#4ade80'}">${angleDeg.toFixed(2)}°</span><br>
      E: ${E.toFixed(4)}  &nbsp; ΔE: <span style="color:${dE_rel>0?'#fbbf24':'#60a5fa'}">${dE_rel>0?'+':''}${dE_rel.toFixed(2)}%</span><br>
      <span class="note">${this.description}</span>
    `;
  }
}
