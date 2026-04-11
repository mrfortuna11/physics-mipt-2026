(function(){
const App = window.App = window.App || {};

function choleskyFactor(A, n) {
  for (let j = 0; j < n; j++) {
    let s = A[j*n + j];
    for (let k = 0; k < j; k++) s -= A[k*n + j] * A[k*n + j];
    if (s <= 0) {
      s = Math.max(s, 0) + 1e-9;
    }
    const Ljj = Math.sqrt(s);
    A[j*n + j] = Ljj;
    const invLjj = 1 / Ljj;
    for (let i = j+1; i < n; i++) {
      let sum = A[j*n + i];
      for (let k = 0; k < j; k++) sum -= A[k*n + i] * A[k*n + j];
      A[j*n + i] = sum * invLjj;
    }
  }
}

function choleskySolve(L, n, b) {
  for (let i = 0; i < n; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[k*n + i] * b[k];
    b[i] = s / L[i*n + i];
  }
  for (let i = n-1; i >= 0; i--) {
    let s = b[i];
    for (let k = i+1; k < n; k++) s -= L[i*n + k] * b[k];
    b[i] = s / L[i*n + i];
  }
}

class PDSolver {
  constructor() {
    this.n = 0;
    this.L = null;          
    this.diagBase = null;   
    this.springW = 100;     
    this.pinW = 1e6;        
    this.lastH = 0;         
    this.dirty = true;      
    this.pinIds = new Set();
    // Scratch buffers
    this.bx = null; this.by = null; this.bz = null;
    this.sx = null; this.sy = null; this.sz = null;
    this.px = null; this.py = null; this.pz = null;
  }

  build(sim, h) {
    const n = sim.P.length;
    this.n = n;
    this.lastH = h;

    const A = new Float64Array(n*n);
    const wS = this.springW;
    const invHH = 1 / (h*h);

    for (let i = 0; i < n; i++) A[i*n + i] += 1.0 * invHH;

    for (const c of sim.C) {
      const i = c.i, j = c.j;
      A[i*n + i] += wS;
      A[j*n + j] += wS;
      A[i*n + j] -= wS;
      A[j*n + i] -= wS;
    }

    this.pinIds = new Set();
    for (const i of sim.pinned) { A[i*n + i] += this.pinW; this.pinIds.add(i); }
    if (sim.grabIdx >= 0) { A[sim.grabIdx*n + sim.grabIdx] += this.pinW; this.pinIds.add(sim.grabIdx); }

    choleskyFactor(A, n);
    this.L = A;

    if (!this.bx || this.bx.length !== n) {
      this.bx = new Float64Array(n); this.by = new Float64Array(n); this.bz = new Float64Array(n);
      this.sx = new Float64Array(n); this.sy = new Float64Array(n); this.sz = new Float64Array(n);
    }
    this.dirty = false;
  }

  ensureFactored(sim, h) {
    if (this.dirty || h !== this.lastH || this.n !== sim.P.length) { this.build(sim, h); return; }
    const want = new Set(sim.pinned);
    if (sim.grabIdx >= 0) want.add(sim.grabIdx);
    if (want.size !== this.pinIds.size) { this.build(sim, h); return; }
    for (const i of want) if (!this.pinIds.has(i)) { this.build(sim, h); return; }
  }

  substep(sim, h) {
    const n = sim.P.length;
    this.ensureFactored(sim, h);

    const g = -sim.gravity;
    const invHH = 1 / (h*h);
    const wS = this.springW;
    const wPin = this.pinW;

    for (let i = 0; i < n; i++) {
      const p = sim.P[i];
      p.vel.multiplyScalar(Math.max(0, 1 - sim.damping*h));
      this.sx[i] = p.pos.x + h*p.vel.x;
      this.sy[i] = p.pos.y + h*p.vel.y + h*h*g;
      this.sz[i] = p.pos.z + h*p.vel.z;
      p.prev.copy(p.pos);
    }

    for (let i = 0; i < n; i++) {
      sim.P[i].pos.x = this.sx[i];
      sim.P[i].pos.y = this.sy[i];
      sim.P[i].pos.z = this.sz[i];
    }

    for (let it = 0; it < sim.iters; it++) {
      const bx = this.bx, by = this.by, bz = this.bz;
      for (let i = 0; i < n; i++) {
        bx[i] = invHH * this.sx[i];
        by[i] = invHH * this.sy[i];
        bz[i] = invHH * this.sz[i];
      }
      for (const c of sim.C) {
        const a = sim.P[c.i].pos, b = sim.P[c.j].pos;
        const dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
        const L = Math.sqrt(dx*dx+dy*dy+dz*dz);
        if (L < 1e-12) continue;
        const s = c.rest / L;
        const px = dx*s, py = dy*s, pz = dz*s;
        bx[c.i] += wS*px; bx[c.j] -= wS*px;
        by[c.i] += wS*py; by[c.j] -= wS*py;
        bz[c.i] += wS*pz; bz[c.j] -= wS*pz;
      }
      for (const i of this.pinIds) {
        let tx, ty, tz;
        if (sim.grabIdx === i) {
          tx = sim.grabTarget.x; ty = sim.grabTarget.y; tz = sim.grabTarget.z;
        } else {
          tx = sim.P[i].prev.x; ty = sim.P[i].prev.y; tz = sim.P[i].prev.z;
        }
        bx[i] += wPin * tx;
        by[i] += wPin * ty;
        bz[i] += wPin * tz;
      }

      choleskySolve(this.L, n, bx);
      choleskySolve(this.L, n, by);
      choleskySolve(this.L, n, bz);
      for (let i = 0; i < n; i++) {
        sim.P[i].pos.x = bx[i];
        sim.P[i].pos.y = by[i];
        sim.P[i].pos.z = bz[i];
      }
    }

    sim.solveFloor();
    if (sim.selfColl) sim.solveSelf();
    if (sim.grabIdx >= 0) sim.P[sim.grabIdx].pos.copy(sim.grabTarget);

    for (let i = 0; i < n; i++) {
      const p = sim.P[i];
      if (sim.pinned.has(i)) { p.vel.set(0,0,0); continue; }
      p.vel.subVectors(p.pos, p.prev).multiplyScalar(1/h);
    }
  }
}

App.PD = new PDSolver();
})();
