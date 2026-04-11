(function(){
const V3 = THREE.Vector3;
const App = window.App;

function buildCloth(sim, N, spacing, origin, pins) {
  pins = pins || [];
  const base = sim.P.length;
  for (let y=0;y<N;y++) for (let x=0;x<N;x++) {
    sim.addParticle(new V3(
      origin.x+(x-(N-1)/2)*spacing, origin.y, origin.z+(y-(N-1)/2)*spacing));
  }
  const idx=(x,y)=>base+y*N+x;
  for (let y=0;y<N;y++) for (let x=0;x<N;x++) {
    if (x+1<N) sim.addConstraint(idx(x,y),idx(x+1,y));
    if (y+1<N) sim.addConstraint(idx(x,y),idx(x,y+1));
  }
  for (let y=0;y<N-1;y++) for (let x=0;x<N-1;x++) {
    sim.addConstraint(idx(x,y),idx(x+1,y+1));
    sim.addConstraint(idx(x+1,y),idx(x,y+1));
  }
  for (let y=0;y<N;y++) for (let x=0;x<N;x++) {
    if (x+2<N) sim.addConstraint(idx(x,y),idx(x+2,y));
    if (y+2<N) sim.addConstraint(idx(x,y),idx(x,y+2));
  }
  for (const [px,py] of pins) sim.pin(idx(px,py));
  const tris=[];
  for (let y=0;y<N-1;y++) for (let x=0;x<N-1;x++) {
    const a=idx(x,y),b=idx(x+1,y),c=idx(x,y+1),d=idx(x+1,y+1);
    tris.push(a,b,c,b,d,c);
  }
  return {base, N, tris};
}

function buildBall(sim, origin, radius, rings) {
  rings = rings || 8;
  const base=sim.P.length, seg=rings*2, surface=[];
  surface.push(sim.addParticle(new V3(origin.x,origin.y+radius,origin.z)));
  for (let r=1;r<rings;r++) {
    const phi=Math.PI*r/rings, y=radius*Math.cos(phi), rr=radius*Math.sin(phi);
    for (let s=0;s<seg;s++) {
      const th=2*Math.PI*s/seg;
      surface.push(sim.addParticle(new V3(
        origin.x+rr*Math.cos(th), origin.y+y, origin.z+rr*Math.sin(th))));
    }
  }
  surface.push(sim.addParticle(new V3(origin.x,origin.y-radius,origin.z)));
  const center=sim.addParticle(new V3(origin.x,origin.y,origin.z));
  const maxD=(2*Math.PI*radius/seg)*2.4;
  for (let i=0;i<surface.length;i++) {
    for (let j=i+1;j<surface.length;j++) {
      const a=sim.P[surface[i]].pos, b=sim.P[surface[j]].pos;
      if (a.distanceTo(b)<=maxD) sim.addConstraint(surface[i],surface[j],0);
    }
    sim.addConstraint(surface[i],center,0);
  }
  const tris=[];
  for (let s=0;s<seg;s++) tris.push(base+0, base+1+s, base+1+((s+1)%seg));
  for (let r=0;r<rings-2;r++) for (let s=0;s<seg;s++) {
    const a=base+1+r*seg+s, b=base+1+r*seg+((s+1)%seg);
    const c=base+1+(r+1)*seg+s, d=base+1+(r+1)*seg+((s+1)%seg);
    tris.push(a,c,b,b,c,d);
  }
  const botIdx=base+1+(rings-1)*seg;
  for (let s=0;s<seg;s++)
    tris.push(botIdx, base+1+(rings-2)*seg+((s+1)%seg), base+1+(rings-2)*seg+s);
  return {tris};
}

function loadScene(name) {
  App.render.clearRenderables();
  App.state.sim = new App.Sim();
  App.ui.applyParamsToSim(App.state.sim);
  const sim = App.state.sim;

  if (name === 'hang') {
    const N=22;
    const r=buildCloth(sim, N, 0.12, new V3(0,3.2,0),
      [[0,0],[N-1,0],[0,N-1],[N-1,N-1]]);
    App.render.makeRenderable(r.base, r.base+N*N, r.tris, 0xe06060);
  }
  else if (name === 'fall') {
    const N=20;
    const r=buildCloth(sim, N, 0.13, new V3(0,2.0,0), []);
    for (let i=r.base;i<r.base+N*N;i++) {
      sim.P[i].pos.y += (Math.random()-0.5)*0.1;
      sim.P[i].pos.x += (Math.random()-0.5)*0.03;
      sim.P[i].prev.copy(sim.P[i].pos);
    }
    App.render.makeRenderable(r.base, r.base+N*N, r.tris, 0x60b060);
    const b=buildBall(sim, new V3(0,4.5,0.2), 0.55, 8);
    const bFirst=r.base+N*N, bCount=sim.P.length-bFirst;
    App.render.makeRenderable(bFirst, bFirst+bCount, b.tris, 0x8080e0);
  }
  else if (name === 'stretch') {
    const N=18;
    const r=buildCloth(sim, N, 0.12, new V3(0,2.5,0), [[0,0],[N-1,N-1]]);
    const natural=(N-1)*0.12*Math.SQRT2, target=natural*2.2;
    const a=r.base, b=r.base+(N-1)*N+(N-1);
    sim.P[a].pos.set(-target/2,3.0,-target/2); sim.P[a].prev.copy(sim.P[a].pos);
    sim.P[b].pos.set(target/2,3.0,target/2);   sim.P[b].prev.copy(sim.P[b].pos);
    App.render.makeRenderable(r.base, r.base+N*N, r.tris, 0xe0a040);
  }
  else if (name === 'ball') {
    const b=buildBall(sim, new V3(0,3.0,0), 0.8, 10);
    App.render.makeRenderable(0, sim.P.length, b.tris, 0x8080e0);
    const N=14;
    const r=buildCloth(sim, N, 0.14, new V3(0,5.0,0), []);
    App.render.makeRenderable(r.base, r.base+N*N, r.tris, 0xe06060);
  }
  App.render.buildEdgeBuffers(sim);
  App.render.updateRenderables(sim);
  App.ui.updateVisibility();
}

App.scenes = { buildCloth, buildBall, loadScene };
})();
