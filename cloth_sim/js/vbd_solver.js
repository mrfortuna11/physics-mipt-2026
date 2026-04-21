(function(){
const App = window.App = window.App || {};

class VBDSolver {
  constructor() {
    this.springW = 200;     
    this.adj = null;        
    this.dirty = true;
    this.prevVel = null;    
    this.frameCount = 0;    
  }

  buildAdj(sim) {
    const n = sim.P.length;
    this.adj = new Array(n);
    for (let i = 0; i < n; i++) this.adj[i] = [];
    for (let ci = 0; ci < sim.C.length; ci++) {
      const c = sim.C[ci];
      this.adj[c.i].push(ci);
      this.adj[c.j].push(ci);
    }
    this.dirty = false;
    this.frameCount = 0;
  }

  // One VBD substep
  substep(sim, h) {
    if (this.dirty || !this.adj || this.adj.length !== sim.P.length) {
      this.buildAdj(sim);
    }

    const n = sim.P.length;
    const g = -sim.gravity;
    const invHH = 1 / (h * h);
    const wS = this.springW;
    const kd = this.dampK;

    if (!this.prevVel || this.prevVel.length !== n * 3) {
      this.prevVel = new Float64Array(n * 3);
    }

    const aExtY = g; 
    const aExtMag = Math.abs(aExtY);

    for (let i = 0; i < n; i++) {
      const p = sim.P[i];

      p.vel.multiplyScalar(Math.max(0, 1 - sim.damping * h));

      p.prev.copy(p.pos);

      let aFactor = 1.0;  
      if (this.frameCount > 5 && aExtMag > 1e-8) {
        const prevVy = this.prevVel[i * 3 + 1];
        const atY = (p.vel.y - prevVy) / h;  
        const aExtDir = aExtY / aExtMag;      
        const atExt = atY * aExtDir;          
        if (atExt > aExtMag) aFactor = 1.0;
        else if (atExt < 0) aFactor = 0.0;
        else aFactor = atExt / aExtMag;
      }

      this.prevVel[i * 3] = p.vel.x;
      this.prevVel[i * 3 + 1] = p.vel.y;
      this.prevVel[i * 3 + 2] = p.vel.z;

      if (p.w === 0) continue; 

      p.pos.x += h * p.vel.x;
      p.pos.y += h * p.vel.y + h * h * aFactor * aExtY;
      p.pos.z += h * p.vel.z;
    }

    const yx = new Float64Array(n), yy = new Float64Array(n), yz = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const p = sim.P[i];
      yx[i] = p.prev.x + h * this.prevVel[i * 3];
      yy[i] = p.prev.y + h * this.prevVel[i * 3 + 1] + h * h * aExtY;
      yz[i] = p.prev.z + h * this.prevVel[i * 3 + 2];
    }

    // Gauss-Seidel iterations 
    for (let iter = 0; iter < sim.iters; iter++) {
      for (let i = 0; i < n; i++) {
        const p = sim.P[i];
        if (p.w === 0) continue; 

        const mi = 1.0;  

        let fx = -mi * invHH * (p.pos.x - yx[i]);
        let fy = -mi * invHH * (p.pos.y - yy[i]);
        let fz = -mi * invHH * (p.pos.z - yz[i]);

        let h00 = mi * invHH, h01 = 0, h02 = 0;
        let h11 = mi * invHH, h12 = 0;
        let h22 = mi * invHH;

        // Spring contributions
        const adj = this.adj[i];
        for (let ai = 0; ai < adj.length; ai++) {
          const c = sim.C[adj[ai]];
          const ki = c.i === i ? c.j : c.i;  
          const sign = c.i === i ? 1 : -1;    

          const pk = sim.P[ki].pos;
          const dx = p.pos.x - pk.x;
          const dy = p.pos.y - pk.y;
          const dz = p.pos.z - pk.z;
          const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
          if (dist < 1e-12) continue;

          const L = c.rest;
          const ratio = L / dist;
          const w = (c.comp !== null ? 1e6 : wS); 

          const gradScale = w * (1 - ratio);
          fx -= gradScale * dx;
          fy -= gradScale * dy;
          fz -= gradScale * dz;


          const a = w * ratio / (dist * dist);  
          const b = w * (1 - ratio);            

          h00 += a * dx * dx + b;
          h01 += a * dx * dy;
          h02 += a * dx * dz;
          h11 += a * dy * dy + b;
          h12 += a * dy * dz;
          h22 += a * dz * dz + b;

          if (kd > 0) {
            const dampH = kd / h;
            const da = dampH * a, db = dampH * b;
            h00 += da * dx * dx + db;
            h01 += da * dx * dy;
            h02 += da * dx * dz;
            h11 += da * dy * dy + db;
            h12 += da * dy * dz;
            h22 += da * dz * dz + db;

            const vx = p.pos.x - p.prev.x;
            const vy = p.pos.y - p.prev.y;
            const vz = p.pos.z - p.prev.z;
            const hsx = (a * dx * dx + b) * vx + (a * dx * dy) * vy + (a * dx * dz) * vz;
            const hsy = (a * dx * dy) * vx + (a * dy * dy + b) * vy + (a * dy * dz) * vz;
            const hsz = (a * dx * dz) * vx + (a * dy * dz) * vy + (a * dz * dz + b) * vz;
            fx -= dampH * hsx;
            fy -= dampH * hsy;
            fz -= dampH * hsz;
          }
        }

        const det =
          h00 * (h11 * h22 - h12 * h12) -
          h01 * (h01 * h22 - h12 * h02) +
          h02 * (h01 * h12 - h11 * h02);

        if (Math.abs(det) < 1e-20) continue; 

        const invDet = 1 / det;
        const i00 = (h11 * h22 - h12 * h12) * invDet;
        const i01 = (h02 * h12 - h01 * h22) * invDet;
        const i02 = (h01 * h12 - h02 * h11) * invDet;
        const i11 = (h00 * h22 - h02 * h02) * invDet;
        const i12 = (h02 * h01 - h00 * h12) * invDet;
        const i22 = (h00 * h11 - h01 * h01) * invDet;

        const ddx = i00 * fx + i01 * fy + i02 * fz;
        const ddy = i01 * fx + i11 * fy + i12 * fz;
        const ddz = i02 * fx + i12 * fy + i22 * fz;

        p.pos.x += ddx;
        p.pos.y += ddy;
        p.pos.z += ddz;
      }

      if (sim.grabIdx >= 0) sim.P[sim.grabIdx].pos.copy(sim.grabTarget);
    }

    sim.solveFloor();
    if (sim.selfColl) sim.solveSelf();
    if (sim.grabIdx >= 0) sim.P[sim.grabIdx].pos.copy(sim.grabTarget);

    for (let i = 0; i < n; i++) {
      const p = sim.P[i];
      if (p.w === 0 || sim.pinned.has(i)) { p.vel.set(0, 0, 0); continue; }
      p.vel.subVectors(p.pos, p.prev).multiplyScalar(1 / h);
    }
    this.frameCount++;
  }
}

App.VBD = new VBDSolver();
})();