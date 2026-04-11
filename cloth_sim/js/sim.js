(function(){
const V3 = THREE.Vector3;
const _t1 = new V3(), _t2 = new V3();

class Sim {
  constructor() {
    this.P = []; this.C = []; this.pinned = new Set();
    this.solver='XPBD'; this.iters=10; this.substeps=1;
    this.stiffness=1; this.compliance=0;
    this.friction=0.3; this.restitution=0.15; this.damping=0.05;
    this.gravity=9.8; this.particleRadius=0.05; this.selfColl=true;
    this.grabIdx=-1; this.grabTarget=new V3(); this.grabStoreW=0;
  }
  addParticle(p,m=1){this.P.push({pos:p.clone(),prev:p.clone(),vel:new V3(),w:m>0?1/m:0,r:this.particleRadius}); if(window.App&&window.App.PD)window.App.PD.dirty=true; return this.P.length-1;}
  addConstraint(i,j,c=null){const r=this.P[i].pos.distanceTo(this.P[j].pos);this.C.push({i,j,rest:r,lambda:0,comp:c}); if(window.App&&window.App.PD)window.App.PD.dirty=true;}
  pin(i){this.P[i].w=0;this.pinned.add(i); if(window.App&&window.App.PD)window.App.PD.dirty=true;}
  updateRadii(){for(const p of this.P)p.r=this.particleRadius;}
  step(dt){
    const h = dt/this.substeps;
    for (let s = 0; s < this.substeps; s++) {
      if (this.solver === 'PD' && window.App && window.App.PD) {
        window.App.PD.substep(this, h);
      } else {
        this.substep(h);
      }
    }
  }
  substep(h){
    const g=-this.gravity;
    for(const p of this.P){
      if(p.w===0){p.prev.copy(p.pos);continue;}
      p.prev.copy(p.pos);
      p.vel.y+=g*h;
      p.vel.multiplyScalar(Math.max(0,1-this.damping*h));
      p.pos.addScaledVector(p.vel,h);
    }
    if(this.solver==='XPBD')for(const c of this.C)c.lambda=0;
    for(let k=0;k<this.iters;k++){
      this.solveDistance(h);
      this.solveFloor();
      if(this.selfColl)this.solveSelf();
      this.applyGrab();
    }
    for(const p of this.P){
      if(p.w===0){p.vel.set(0,0,0);continue;}
      p.vel.subVectors(p.pos,p.prev).multiplyScalar(1/h);
    }
  }
  solveDistance(h){
    const d=_t1;
    if(this.solver==='JAK'){
      const k=1-Math.pow(1-this.stiffness,1/this.iters);
      for(const c of this.C){
        const a=this.P[c.i],b=this.P[c.j];const w=a.w+b.w;if(w===0)continue;
        d.subVectors(b.pos,a.pos);const L=d.length();if(L<1e-9)continue;
        const f=k*(L-c.rest)/L/w;
        a.pos.addScaledVector(d,f*a.w);b.pos.addScaledVector(d,-f*b.w);
      }
    }else if(this.solver==='PBD'){
      const k=1-Math.pow(1-this.stiffness,1/this.iters);
      for(const c of this.C){
        const a=this.P[c.i],b=this.P[c.j];const w=a.w+b.w;if(w===0)continue;
        d.subVectors(a.pos,b.pos);const L=d.length();if(L<1e-9)continue;
        const s=-(L-c.rest)/w*k,nx=d.x/L,ny=d.y/L,nz=d.z/L;
        a.pos.x+=s*a.w*nx;a.pos.y+=s*a.w*ny;a.pos.z+=s*a.w*nz;
        b.pos.x-=s*b.w*nx;b.pos.y-=s*b.w*ny;b.pos.z-=s*b.w*nz;
      }
    }else{
      const hh=h*h;
      for(const c of this.C){
        const a=this.P[c.i],b=this.P[c.j];const w=a.w+b.w;if(w===0)continue;
        const comp=(c.comp!==null?c.comp:this.compliance),alpha=comp/hh;
        d.subVectors(a.pos,b.pos);const L=d.length();if(L<1e-9)continue;
        const C=L-c.rest,dL=(-C-alpha*c.lambda)/(w+alpha);c.lambda+=dL;
        const nx=d.x/L,ny=d.y/L,nz=d.z/L;
        a.pos.x+=dL*a.w*nx;a.pos.y+=dL*a.w*ny;a.pos.z+=dL*a.w*nz;
        b.pos.x-=dL*b.w*nx;b.pos.y-=dL*b.w*ny;b.pos.z-=dL*b.w*nz;
      }
    }
  }
  solveFloor(){
    const fr=this.friction,e=this.restitution;
    for(const p of this.P){
      if(p.w===0)continue;
      if(p.r-p.pos.y>0){
        const vy=p.pos.y-p.prev.y;
        p.pos.y=p.r;
        p.pos.x-=(p.pos.x-p.prev.x)*fr;
        p.pos.z-=(p.pos.z-p.prev.z)*fr;
        if(vy<0)p.prev.y=p.r+e*vy;
      }
    }
  }
  solveSelf(){
    const cell=Math.max(2*this.particleRadius,1e-4),inv=1/cell;
    const hash=new Map();
    const hk=(x,y,z)=>(x*73856093)^(y*19349663)^(z*83492791);
    for(let i=0;i<this.P.length;i++){
      const p=this.P[i].pos;
      const k=hk(Math.floor(p.x*inv),Math.floor(p.y*inv),Math.floor(p.z*inv));
      let a=hash.get(k);if(!a){a=[];hash.set(k,a);}a.push(i);
    }
    const fr=this.friction,d=_t2;
    for(let i=0;i<this.P.length;i++){
      const pi=this.P[i],pp=pi.pos;
      const cx=Math.floor(pp.x*inv),cy=Math.floor(pp.y*inv),cz=Math.floor(pp.z*inv);
      for(let ox=-1;ox<=1;ox++)for(let oy=-1;oy<=1;oy++)for(let oz=-1;oz<=1;oz++){
        const arr=hash.get(hk(cx+ox,cy+oy,cz+oz));if(!arr)continue;
        for(const j of arr){
          if(j<=i)continue;
          const pj=this.P[j],w=pi.w+pj.w;if(w===0)continue;
          const minD=pi.r+pj.r;
          d.subVectors(pj.pos,pi.pos);
          const L2=d.x*d.x+d.y*d.y+d.z*d.z;
          if(L2>=minD*minD||L2<1e-12)continue;
          const L=Math.sqrt(L2),corr=(minD-L)/w;
          const nx=d.x/L,ny=d.y/L,nz=d.z/L;
          pi.pos.x-=corr*pi.w*nx;pi.pos.y-=corr*pi.w*ny;pi.pos.z-=corr*pi.w*nz;
          pj.pos.x+=corr*pj.w*nx;pj.pos.y+=corr*pj.w*ny;pj.pos.z+=corr*pj.w*nz;
          if(fr>0){
            let tx=(pj.pos.x-pj.prev.x)-(pi.pos.x-pi.prev.x);
            let ty=(pj.pos.y-pj.prev.y)-(pi.pos.y-pi.prev.y);
            let tz=(pj.pos.z-pj.prev.z)-(pi.pos.z-pi.prev.z);
            const tn=tx*nx+ty*ny+tz*nz;
            tx-=tn*nx;ty-=tn*ny;tz-=tn*nz;
            const f=fr*0.5;
            pi.pos.x+=f*pi.w/w*tx;pi.pos.y+=f*pi.w/w*ty;pi.pos.z+=f*pi.w/w*tz;
            pj.pos.x-=f*pj.w/w*tx;pj.pos.y-=f*pj.w/w*ty;pj.pos.z-=f*pj.w/w*tz;
          }
        }
      }
    }
  }
  applyGrab(){if(this.grabIdx>=0)this.P[this.grabIdx].pos.copy(this.grabTarget);}
  startGrab(i,t){if(i<0)return;this.grabIdx=i;this.grabStoreW=this.P[i].w;this.P[i].w=0;this.grabTarget.copy(t); if(window.App&&window.App.PD)window.App.PD.dirty=true;}
  moveGrab(t){this.grabTarget.copy(t);}
  endGrab(){if(this.grabIdx<0)return;if(!this.pinned.has(this.grabIdx))this.P[this.grabIdx].w=this.grabStoreW;this.grabIdx=-1; if(window.App&&window.App.PD)window.App.PD.dirty=true;}
}

window.App = window.App || {};
window.App.Sim = Sim;
})();
