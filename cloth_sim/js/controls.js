(function(){
const App = window.App;
const V3 = THREE.Vector3;
const _t = new V3();

const camState = {
  target: new V3(0,1.2,0),
  radius: 7, theta: -Math.PI/6, phi: Math.PI/3,
};
function updateCam() {
  const cam = App.render.camera;
  cam.position.set(
    camState.target.x + camState.radius*Math.sin(camState.phi)*Math.sin(camState.theta),
    camState.target.y + camState.radius*Math.cos(camState.phi),
    camState.target.z + camState.radius*Math.sin(camState.phi)*Math.cos(camState.theta)
  );
  cam.lookAt(camState.target);
}

const ray = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let mode = null, last = {x:0,y:0};
const grabPlane = new THREE.Plane();

function pick(cx, cy) {
  const sim = App.state.sim;
  const cam = App.render.camera;
  const canvas = App.render.canvas;
  const rect = canvas.getBoundingClientRect();
  ndc.x = ((cx-rect.left)/rect.width)*2-1;
  ndc.y = -((cy-rect.top)/rect.height)*2+1;
  ray.setFromCamera(ndc, cam);
  let best=-1, bestT=Infinity;
  for (let i=0;i<sim.P.length;i++) {
    _t.copy(sim.P[i].pos).sub(ray.ray.origin);
    const t = _t.dot(ray.ray.direction);
    if (t<0) continue;
    const perp2 = _t.lengthSq()-t*t;
    const r = sim.P[i].r*1.5;
    if (perp2<r*r && t<bestT) { bestT=t; best=i; }
  }
  return best;
}

function init() {
  updateCam();
  const canvas = App.render.canvas;
  const cam = App.render.camera;
  canvas.addEventListener('contextmenu', e=>e.preventDefault());
  canvas.addEventListener('mousedown', e=>{
    last.x=e.clientX; last.y=e.clientY;
    if (e.button===2) { mode='orbit'; return; }
    if (e.button===0) {
      const i = pick(e.clientX, e.clientY);
      if (i>=0) {
        mode='grab';
        const sim = App.state.sim;
        const n = new V3().subVectors(cam.position, sim.P[i].pos).normalize();
        grabPlane.setFromNormalAndCoplanarPoint(n, sim.P[i].pos);
        sim.startGrab(i, sim.P[i].pos);
      } else mode='orbit';
    }
  });
  canvas.addEventListener('mousemove', e=>{
    const dx=e.clientX-last.x, dy=e.clientY-last.y;
    last.x=e.clientX; last.y=e.clientY;
    if (mode==='orbit') {
      camState.theta -= dx*0.008;
      camState.phi = Math.max(0.12, Math.min(Math.PI-0.12, camState.phi-dy*0.008));
      updateCam();
    } else if (mode==='grab') {
      const rect=canvas.getBoundingClientRect();
      ndc.x=((e.clientX-rect.left)/rect.width)*2-1;
      ndc.y=-((e.clientY-rect.top)/rect.height)*2+1;
      ray.setFromCamera(ndc, cam);
      const hit=new V3();
      if (ray.ray.intersectPlane(grabPlane, hit)) App.state.sim.moveGrab(hit);
    }
  });
  window.addEventListener('mouseup', ()=>{
    if (mode==='grab') App.state.sim.endGrab();
    mode=null;
  });
  canvas.addEventListener('wheel', e=>{
    e.preventDefault();
    camState.radius *= (1+e.deltaY*0.001);
    camState.radius = Math.max(1.5, Math.min(30, camState.radius));
    updateCam();
  }, {passive:false});
}

App.controls = { init, updateCam, camState };
})();
