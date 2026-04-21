(function(){
const App = window.App = window.App || {};

class VBDSolver {
  constructor() {
    this.springW = 200;
    this.dampK = 0.01;
    this.adj = null;
    this.dirty = true;
  }

  buildAdj(sim) {
    const n = sim.P.length;
    this.adj = new Array(n);
    for (let i = 0; i < n; i++) this.adj[i] = [];
    for (let ci = 0; ci < sim.C.length; ci++) {
      this.adj[sim.C[ci].i].push(ci);
      this.adj[sim.C[ci].j].push(ci);
    }
    this.dirty = false;
  }

  substep(sim, h) {
    if (this.dirty || !this.adj || this.adj.length !== sim.P.length) {
      this.buildAdj(sim);
    }

    const n = sim.P.length;
    const g = -sim.gravity;
    const invHH = 1 / (h * h);
    const wS = this.springW;
    const kd = this.dampK;

    // Save prev, apply damping, compute inertial target y
    const yx = new Float64Array(n), yy = new Float64Array(n), yz = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const p = sim.P[i];
      p.vel.multiplyScalar(Math.max(0, 1 - sim.damping * h));
      p.prev.copy(p.pos);

      yx[i] = p.pos.x + h * p.vel.x;
      yy[i] = p.pos.y + h * p.vel.y + h * h * g;
      yz[i] = p.pos.z + h * p.vel.z;

      if (p.w === 0) continue; // pinned

      // Initialize: x = y 
      p.pos.x = yx[i];
      p.pos.y = yy[i];
      p.pos.z = yz[i];
    }

    // Gauss-Seidel iterations
    for (let iter = 0; iter < sim.iters; iter++) {
      for (let i = 0; i < n; i++) {
        const p = sim.P[i];
        if (p.w === 0) continue;

        let fx = -invHH * (p.pos.x - yx[i]);
        let fy = -invHH * (p.pos.y - yy[i]);
        let fz = -invHH * (p.pos.z - yz[i]);
        let h00 = invHH, h01 = 0, h02 = 0;
        let h11 = invHH, h12 = 0;
        let h22 = invHH;

        // Spring contributions
        const adj = this.adj[i];
        for (let ai = 0; ai < adj.length; ai++) {
          const c = sim.C[adj[ai]];
          const ki = c.i === i ? c.j : c.i;
          const pk = sim.P[ki].pos;
          const dx = p.pos.x - pk.x;
          const dy = p.pos.y - pk.y;
          const dz = p.pos.z - pk.z;
          const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
          if (dist < 1e-12) continue;

          const L = c.rest;
          const ratio = L / dist;
          const w = (c.comp !== null) ? Math.max(wS, 1e4) : wS;

          // Gradient
          fx -= w * (1 - ratio) * dx;
          fy -= w * (1 - ratio) * dy;
          fz -= w * (1 - ratio) * dz;

          // Hessian
          const a = w * ratio / (dist * dist);
          const b = w * (1 - ratio);
          h00 += a * dx * dx + b;
          h01 += a * dx * dy;
          h02 += a * dx * dz;
          h11 += a * dy * dy + b;
          h12 += a * dy * dz;
          h22 += a * dz * dz + b;

          // Rayleigh damping 
          if (kd > 0) {
            const dh = kd / h;
            h00 += dh * (a * dx * dx + b);
            h01 += dh * (a * dx * dy);
            h02 += dh * (a * dx * dz);
            h11 += dh * (a * dy * dy + b);
            h12 += dh * (a * dy * dz);
            h22 += dh * (a * dz * dz + b);
            const vx = p.pos.x - p.prev.x;
            const vy = p.pos.y - p.prev.y;
            const vz = p.pos.z - p.prev.z;
            fx -= dh * ((a*dx*dx+b)*vx + a*dx*dy*vy + a*dx*dz*vz);
            fy -= dh * (a*dx*dy*vx + (a*dy*dy+b)*vy + a*dy*dz*vz);
            fz -= dh * (a*dx*dz*vx + a*dy*dz*vy + (a*dz*dz+b)*vz);
          }
        }

        // Floor penalty energy 
        const floorPen = p.r - p.pos.y;
        if (floorPen > 0) {
          const kc = 1e5;
          fy += kc * floorPen;
          h11 += kc;
          if (sim.friction > 0) {
            const tanF = kc * floorPen * sim.friction;
            const vx = p.pos.x - p.prev.x;
            const vz = p.pos.z - p.prev.z;
            const ts = Math.sqrt(vx * vx + vz * vz);
            if (ts > 1e-10) {
              fx -= tanF * vx / ts;
              fz -= tanF * vz / ts;
              h00 += tanF / ts;
              h22 += tanF / ts;
            }
          }
        }

        // H Δx = f (Cramer's rule)
        const det =
          h00 * (h11 * h22 - h12 * h12) -
          h01 * (h01 * h22 - h12 * h02) +
          h02 * (h01 * h12 - h11 * h02);
        if (Math.abs(det) < 1e-20) continue;

        const id = 1 / det;
        p.pos.x += (h11*h22-h12*h12)*id*fx + (h02*h12-h01*h22)*id*fy + (h01*h12-h02*h11)*id*fz;
        p.pos.y += (h02*h12-h01*h22)*id*fx + (h00*h22-h02*h02)*id*fy + (h02*h01-h00*h12)*id*fz;
        p.pos.z += (h01*h12-h02*h11)*id*fx + (h02*h01-h00*h12)*id*fy + (h00*h11-h01*h01)*id*fz;
      }

      if (sim.grabIdx >= 0) sim.P[sim.grabIdx].pos.copy(sim.grabTarget);
    }

    // Post-projection
    sim.solveFloor();
    if (sim.selfColl) sim.solveSelf();
    if (sim.grabIdx >= 0) sim.P[sim.grabIdx].pos.copy(sim.grabTarget);

    // Velocity update
    for (let i = 0; i < n; i++) {
      const p = sim.P[i];
      if (p.w === 0 || sim.pinned.has(i)) { p.vel.set(0, 0, 0); continue; }
      p.vel.subVectors(p.pos, p.prev).multiplyScalar(1 / h);
    }
  }
}

App.VBD = new VBDSolver();
})();