(function(){
const App = window.App;

const ui_state = { paused: false };

function bind(id, fn, isFloat) {
  if (isFloat === undefined) isFloat = true;
  const el = document.getElementById(id);
  const vel = document.getElementById(id+'Val');
  let dec=2;
  if (isFloat && el.step) {
    const s=parseFloat(el.step);
    if (s>0) dec=Math.max(0, Math.min(5, -Math.floor(Math.log10(s))));
  }
  const upd=()=>{
    const v = isFloat ? parseFloat(el.value) : parseInt(el.value);
    if (vel) vel.textContent = isFloat ? v.toFixed(dec) : v;
    fn(v);
  };
  el.addEventListener('input', upd); upd();
}

function applyParamsToSim(sim) {
  sim.iters = parseInt(document.getElementById('iters').value);
  sim.substeps = parseInt(document.getElementById('subs').value);
  sim.stiffness = parseFloat(document.getElementById('stiff').value);
  sim.compliance = parseFloat(document.getElementById('comp').value);
  sim.friction = parseFloat(document.getElementById('fric').value);
  sim.restitution = parseFloat(document.getElementById('rest').value);
  sim.damping = parseFloat(document.getElementById('damp').value);
  sim.gravity = parseFloat(document.getElementById('grav').value);
  sim.particleRadius = parseFloat(document.getElementById('prad').value);
  sim.solver = document.getElementById('solver').value;
  sim.selfColl = document.getElementById('selfColl').checked;
  sim.updateRadii();
}

function updateVisibility() {
  const edges = document.getElementById('showEdges').checked;
  const pts = document.getElementById('showPts').checked;
  for (const r of App.render.renderables) {
    r.meshLine.visible = edges; r.ptPoints.visible = pts;
  }
}

function init() {
  bind('iters', v=>App.state.sim.iters=v, false);
  bind('subs',  v=>App.state.sim.substeps=v, false);
  bind('stiff', v=>App.state.sim.stiffness=v);
  bind('comp',  v=>App.state.sim.compliance=v);
  bind('fric',  v=>App.state.sim.friction=v);
  bind('rest',  v=>App.state.sim.restitution=v);
  bind('damp',  v=>App.state.sim.damping=v);
  bind('grav',  v=>App.state.sim.gravity=v);
  bind('prad',  v=>{App.state.sim.particleRadius=v; App.state.sim.updateRadii();});
  bind('pdW',   v=>{ if(App.PD){ App.PD.springW=v; App.PD.dirty=true; } });
  bind('vbdW',  v=>{ if(App.VBD){ App.VBD.springW=v; App.VBD.dirty=true; } });
  bind('vbdD',  v=>{ if(App.VBD){ App.VBD.dampK=v; App.VBD.dirty=true; } });

  document.getElementById('solver').addEventListener('change', e=>App.state.sim.solver=e.target.value);
  document.getElementById('scene').addEventListener('change', e=>App.scenes.loadScene(e.target.value));
  document.getElementById('restart').addEventListener('click', ()=>App.scenes.loadScene(document.getElementById('scene').value));
  document.getElementById('pauseBtn').addEventListener('click', e=>{
    ui_state.paused = !ui_state.paused;
    e.target.textContent = ui_state.paused ? 'Resume' : 'Pause';
  });
  document.getElementById('selfColl').addEventListener('change', e=>App.state.sim.selfColl=e.target.checked);
  document.getElementById('showEdges').addEventListener('change', updateVisibility);
  document.getElementById('showPts').addEventListener('change', updateVisibility);
}

App.ui = { init, applyParamsToSim, updateVisibility, ui_state };
})();
