(function(){
const App = window.App;
App.state = { sim: new App.Sim() };

function resize() {
  const canvas = App.render.canvas;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  App.render.renderer.setSize(w, h, false);
  App.render.camera.aspect = w/h;
  App.render.camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

App.ui.init();
App.controls.init();
App.scenes.loadScene('hang');

const statsEl = document.getElementById('stats');
const frameTimes = [];
function loop() {
  const t0 = performance.now();
  if (!App.ui.ui_state.paused) App.state.sim.step(1/60);
  App.render.updateRenderables(App.state.sim);
  App.render.renderer.render(App.render.scene, App.render.camera);
  const dt = performance.now()-t0;
  frameTimes.push(dt);
  if (frameTimes.length>30) frameTimes.shift();
  const avg = frameTimes.reduce((a,b)=>a+b,0)/frameTimes.length;
  statsEl.textContent = App.state.sim.solver + ' \u00b7 ' +
    App.state.sim.P.length + ' pts \u00b7 ' +
    App.state.sim.C.length + ' constraints \u00b7 ' +
    avg.toFixed(1) + ' ms/frame';
  requestAnimationFrame(loop);
}
loop();
})();
