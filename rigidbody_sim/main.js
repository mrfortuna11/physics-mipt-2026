// Entry point: registers scenes, wires UI controls, runs the animation loop.
// THREE is loaded as a global UMD script (three.min.js + OrbitControls.js).
const OrbitControls = THREE.OrbitControls;

// =========================================================================
//  Renderer / scene host
// =========================================================================
const canvas = document.getElementById('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x1a1a1a, 1);
renderer.autoClear = false;
renderer.setScissorTest(true);

const overlaysHost = document.getElementById('overlays');

// =========================================================================
//  Scene registry
// =========================================================================
const partViews = {
  1: [
    new Part1View('(a) Global frame',  'L = const в межкадре, ω = I⁻¹(t)·L. Должно сохраняться лучше всех.', stepGlobalConstL,       0x6b9aff, overlaysHost),
    new Part1View('(b) Body — без гироскопики', 'ω̇_body = 0. Неверно: L уплывает быстро.',                     stepBodyNoGyro,          0xff9966, overlaysHost),
    new Part1View('(c) Body — explicit gyro',   'Явный Эйлер с −ω×Iω. Эффект Джанибекова, E растёт.',          stepBodyExplicitGyro,    0xffcc44, overlaysHost),
    new Part1View('(d) Body — implicit gyro',   'Неявный Эйлер, Ньютон 3 итерации. Эффект Джанибекова, E падает.', stepBodyImplicitGyro, 0x4ade80, overlaysHost),
  ],
  2: {
    'spring': [
      new Part2SpringView('Spring (external force)',
                          'F = -k(x_a - anchor) - c·v_a, применённое как внешняя сила и момент.',
                          'force', 0x6b9aff, overlaysHost),
      new Part2SpringView('Spring (soft constraint, XPBD)',
                          'XPBD позиционная коррекция: Δλ = (-c - α̃·λ)/(w + α̃), compliance из Hz.',
                          'soft',  0xa78bfa, overlaysHost),
    ],
    'dist': [
      new Part2DistView_XPBD       ('(a) XPBD',           'Position-level: Δλ = (-c - α̃·λ)/(w + α̃). α̃ = compliance/dt².', 0x4ade80, overlaysHost),
      new Part2DistView_SI_Baumgarte('(b) SI + Baumgarte', 'Velocity-level PGS: Δλ = (-vN - β·c/dt)/w. Энергия может расти.', 0x6b9aff, overlaysHost),
      new Part2DistView_SI_NGS     ('(c) SI + NGS',        'Velocity solve без bias, потом отдельный position solve.', 0xfbbf24, overlaysHost),
      new Part2DistView_SI_Soft    ('(d) SI + Soft',       'Soft (Catto): Δλ = -massCoeff·meff·(vN+biasRate·c) - impulseCoeff·λ.', 0xa78bfa, overlaysHost),
    ],
  },
  3: {
    'xpbd': [ new Part3View_XPBD(
                '(3A) XPBD + friction',
                'Position-level XPBD contact constraint + статическое и динамическое трение.',
                10, false,
                { useFriction: true,
                  frictionGetter: () => ({ mus: globalParams.col3Mus, mud: globalParams.col3Mud }) },
                overlaysHost) ],
    'si':   [ new Part3View_SI(
                '(3B) SI коллизии',
                'Velocity-level PGS + Baumgarte stabilization.',
                10, false, {}, overlaysHost) ],
    'grid': [ new Part3View_Pile(
                '(3C) Spatial Grid (pile)',
                'XPBD + friction, broadphase = Spatial Hash Grid. N задаётся слайдером.',
                400, true,
                { useFriction: true,
                  broadphase: 'grid',
                  numBodiesGetter: () => globalParams.col3N,
                  frictionGetter: () => ({ mus: globalParams.col3Mus, mud: globalParams.col3Mud }) },
                overlaysHost) ],
  },
  4: {
    'sap':  [ new Part4View_XPBD(
                '(4A) varied + SAP',
                'Разнокалиберные тела, broadphase = Sweep and Prune.',
                250, false,
                { useFriction: true, varied: true, broadphase: 'sap',
                  arenaSize: 14, tab: 4,
                  numBodiesGetter: () => globalParams.col4N,
                  frictionGetter: () => ({ mus: globalParams.col4Mus, mud: globalParams.col4Mud }) },
                overlaysHost) ],
    'lbvh': [ new Part4View_XPBD(
                '(4B) varied + LBVH',
                'Разнокалиберные тела, broadphase = LBVH (Morton-sorted BVH).',
                250, false,
                { useFriction: true, varied: true, broadphase: 'lbvh',
                  arenaSize: 14, tab: 4,
                  numBodiesGetter: () => globalParams.col4N,
                  frictionGetter: () => ({ mus: globalParams.col4Mus, mud: globalParams.col4Mud }) },
                overlaysHost) ],
    'ramp': [ new Part4View_XPBD(
                '(4C) Friction ramp',
                'Наклонная плоскость. Если μ_s > tan(angle) — тела стоят; иначе скользят с μ_d.',
                8, false,
                { useFriction: true, broadphase: 'brute',
                  arenaSize: 10, tab: 4,
                  rampAngle: 25 * Math.PI / 180,
                  rampAngleGetter: () => globalParams.col4Ramp * Math.PI / 180,
                  numBodiesGetter: () => 8,
                  frictionGetter: () => ({ mus: globalParams.col4Mus, mud: globalParams.col4Mud }) },
                overlaysHost) ],
  },
};

// =========================================================================
//  Tabs / subtabs
// =========================================================================
const subtabsHost = document.getElementById('subtabs');
let currentTab = 1;
let currentSubtab = null;
let activeViews = [];

function buildSubtabs(tab) {
  subtabsHost.innerHTML = '';
  const subtabConfigs = {
    2: [['spring','2A · Spring'], ['dist','2B · Distance constraint']],
    3: [['xpbd','3A · XPBD'], ['si','3B · SI'], ['grid','3C · Spatial Grid']],
    4: [['sap','4A · SAP'], ['lbvh','4B · LBVH'], ['ramp','4C · Friction ramp']],
  };
  const config = subtabConfigs[tab];
  if (!config) {
    subtabsHost.style.display = 'none';
    currentSubtab = null;
    return;
  }
  subtabsHost.style.display = 'flex';
  config.forEach(([sub, label]) => {
    const el = document.createElement('div');
    el.className = 'subtab' + (currentSubtab === sub ? ' active' : '');
    el.textContent = label;
    el.dataset.sub = sub;
    subtabsHost.appendChild(el);
  });
  const validSubs = config.map(c => c[0]);
  if (!validSubs.includes(currentSubtab)) currentSubtab = validSubs[0];

  subtabsHost.querySelectorAll('.subtab').forEach(s => {
    s.addEventListener('click', () => {
      subtabsHost.querySelectorAll('.subtab').forEach(x => x.classList.remove('active'));
      s.classList.add('active');
      currentSubtab = s.dataset.sub;
      updateControlVisibility();
      activateViews();
    });
  });
}

function updateControlVisibility() {
  const isSpring = currentTab === 2 && currentSubtab === 'spring';
  const isDist   = currentTab === 2 && currentSubtab === 'dist';
  document.getElementById('ctrl-part2-spring').classList.toggle('hidden', !isSpring);
  document.getElementById('ctrl-part2-dist').classList.toggle('hidden', !isDist);
  document.getElementById('ctrl-part3').classList.toggle('hidden', currentTab !== 3);
  document.getElementById('ctrl-part4').classList.toggle('hidden', currentTab !== 4);
}

function activateViews() {
  document.querySelectorAll('.vp-label').forEach(l => l.style.display = 'none');

  if (currentTab === 1)                            activeViews = partViews[1];
  else if (currentTab === 2)                       activeViews = partViews[2][currentSubtab];
  else if (currentTab === 3 || currentTab === 4) {
    activeViews = partViews[currentTab][currentSubtab];
    if (substeps < 8) {
      substeps = 8;
      document.getElementById('sl-sub').value = 8;
      document.getElementById('v-sub').textContent = '8';
    }
  } else activeViews = [];

  activeViews.forEach(v => v.label.style.display = '');

  document.getElementById('ctrl-part1').classList.toggle('hidden', currentTab !== 1);
  updateControlVisibility();
  document.getElementById('legend-part1').classList.toggle('hidden', currentTab !== 1);
  document.getElementById('legend-part2').classList.toggle('hidden', currentTab !== 2);
  document.getElementById('legend-part34').classList.toggle('hidden', currentTab !== 3 && currentTab !== 4);

  resetAll();
  layout();
}

function switchTab(tab) {
  if (tab === currentTab) return;
  currentTab = tab;
  document.querySelectorAll('#tabs .tab').forEach(t => {
    t.classList.toggle('active', parseInt(t.dataset.tab) === tab);
  });
  buildSubtabs(tab);
  activateViews();
}

// =========================================================================
//  Camera / orbit + viewport layout
// =========================================================================
const orbit = new OrbitControls(new THREE.PerspectiveCamera(), canvas);
orbit.enableDamping = true;
orbit.dampingFactor = 0.08;
orbit.target.set(0, 0, 0);

function syncCameras() {
  if (activeViews.length === 0) return;
  const master = activeViews[0].camera;
  master.position.copy(orbit.object.position);
  master.quaternion.copy(orbit.object.quaternion);
  master.zoom = orbit.object.zoom;
  master.updateProjectionMatrix();
  for (let i = 1; i < activeViews.length; i++) {
    activeViews[i].camera.position.copy(master.position);
    activeViews[i].camera.quaternion.copy(master.quaternion);
    activeViews[i].camera.zoom = master.zoom;
    activeViews[i].camera.updateProjectionMatrix();
  }
}
function bindOrbitTo(camera) { orbit.object = camera; orbit.update(); }

function layout() {
  const host = document.getElementById('scene-host');
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h, false);
  const n = activeViews.length;
  if (n === 0) return;

  let cols, rows;
  if (n === 1)      { cols = 1; rows = 1; }
  else if (n === 2) { cols = 2; rows = 1; }
  else if (n <= 4)  { cols = 2; rows = 2; }
  else              { cols = 3; rows = Math.ceil(n / 3); }

  const cw = w / cols, ch = h / rows;
  activeViews.forEach((v, i) => {
    const col = i % cols, row = Math.floor(i / cols);
    v._vp = { x: col * cw, y: h - (row + 1) * ch, w: cw, h: ch };
    v.camera.aspect = cw / ch;
    v.camera.updateProjectionMatrix();
    v.label.style.left = (col * cw + 8) + 'px';
    v.label.style.top  = (row * ch + 8) + 'px';
  });
  bindOrbitTo(activeViews[0].camera);
}
window.addEventListener('resize', layout);

// =========================================================================
//  Reset / sliders / buttons
// =========================================================================
function resetAll() {
  if (currentTab === 1) {
    const pert = parseFloat(document.getElementById('sl-pert').value);
    const omega0 = new THREE.Vector3(pert, 3.0, pert * 0.7);
    const q0 = new THREE.Quaternion();
    activeViews.forEach(v => v.applyInitial(q0, omega0));
  } else {
    activeViews.forEach(v => v.reset && v.reset());
  }
}

function bindSlider(id, vid, setter, fmt) {
  const el = document.getElementById(id);
  const vel = document.getElementById(vid);
  const update = () => {
    const x = parseFloat(el.value);
    setter(x);
    vel.textContent = fmt(x);
  };
  el.addEventListener('input', update);
  update();
}

let dt = 0.002, substeps = 4;
let paused = false, stepOnce = false;

bindSlider('sl-dt',   'v-dt',   v => dt = v,                    v => v.toFixed(4));
bindSlider('sl-sub',  'v-sub',  v => substeps = parseInt(v),    v => String(parseInt(v)));
bindSlider('sl-pert', 'v-pert', () => {},                       v => v.toFixed(3));
bindSlider('sl-g',    'v-g',    v => globalParams.gravity  = v, v => v.toFixed(2));
bindSlider('sl-k',    'v-k',    v => globalParams.springK  = v, v => v.toFixed(0));
bindSlider('sl-c',    'v-c',    v => globalParams.springC  = v, v => v.toFixed(2));
bindSlider('sl-hz',   'v-hz',   v => globalParams.softHz   = v, v => v.toFixed(1));
bindSlider('sl-zeta', 'v-zeta', v => globalParams.softZeta = v, v => v.toFixed(2));

bindSlider('sl-g2',    'v-g2',    v => globalParams.gravity        = v, v => v.toFixed(2));
bindSlider('sl-alpha', 'v-alpha', v => globalParams.distCompliance = v, v => v.toFixed(4));
bindSlider('sl-beta',  'v-beta',  v => globalParams.baumgarte      = v, v => v.toFixed(2));
bindSlider('sl-hz2',   'v-hz2',   v => globalParams.softHz         = v, v => v.toFixed(1));
bindSlider('sl-zeta2', 'v-zeta2', v => globalParams.softZeta       = v, v => v.toFixed(2));
bindSlider('sl-vi',    'v-vi',    v => globalParams.siIters        = parseInt(v), v => String(parseInt(v)));
bindSlider('sl-pi',    'v-pi',    v => globalParams.posIters       = parseInt(v), v => String(parseInt(v)));
bindSlider('sl-ld',    'v-ld',    v => globalParams.linDamp        = v,           v => v.toFixed(1));
bindSlider('sl-ad',    'v-ad',    v => globalParams.angDamp        = v,           v => v.toFixed(1));

bindSlider('sl-rest',  'v-rest',  v => globalParams.restitution    = v,           v => v.toFixed(2));
bindSlider('sl-c3a',   'v-c3a',   v => globalParams.col3Alpha      = v,           v => v.toFixed(4));
bindSlider('sl-c3b',   'v-c3b',   v => globalParams.baumgarte      = v,           v => v.toFixed(2));
bindSlider('sl-c3pi',  'v-c3pi',  v => globalParams.col3PosIters   = parseInt(v), v => String(parseInt(v)));
bindSlider('sl-c3vi',  'v-c3vi',  v => globalParams.col3VelIters   = parseInt(v), v => String(parseInt(v)));
bindSlider('sl-c3mus', 'v-c3mus', v => globalParams.col3Mus        = v,           v => v.toFixed(2));
bindSlider('sl-c3mud', 'v-c3mud', v => globalParams.col3Mud        = v,           v => v.toFixed(2));
bindSlider('sl-c3n',   'v-c3n',   v => globalParams.col3N          = parseInt(v), v => String(parseInt(v)));

bindSlider('sl-c4rest','v-c4rest',v => globalParams.restitution    = v,           v => v.toFixed(2));
bindSlider('sl-c4mus', 'v-c4mus', v => globalParams.col4Mus        = v,           v => v.toFixed(2));
bindSlider('sl-c4mud', 'v-c4mud', v => globalParams.col4Mud        = v,           v => v.toFixed(2));
bindSlider('sl-c4n',   'v-c4n',   v => globalParams.col4N          = parseInt(v), v => String(parseInt(v)));
bindSlider('sl-c4ramp','v-c4ramp',v => globalParams.col4Ramp       = parseInt(v), v => String(parseInt(v)));

document.getElementById('btn-pause').addEventListener('click', e => {
  paused = !paused;
  e.target.textContent = paused ? '▶ Старт' : '⏸ Пауза';
});
document.getElementById('btn-reset').addEventListener('click', resetAll);
document.getElementById('btn-step').addEventListener('click', () => {
  paused = true;
  document.getElementById('btn-pause').textContent = '▶ Старт';
  stepOnce = true;
});
document.querySelectorAll('#tabs .tab').forEach(t => {
  t.addEventListener('click', () => {
    if (t.classList.contains('disabled')) return;
    switchTab(parseInt(t.dataset.tab));
  });
});

// =========================================================================
//  Animation loop
// =========================================================================
function animate() {
  requestAnimationFrame(animate);
  orbit.update();
  syncCameras();

  if (!paused || stepOnce) {
    for (let s = 0; s < substeps; s++) {
      activeViews.forEach(v => v.step(dt));
    }
    stepOnce = false;
  }
  activeViews.forEach(v => { v.updateMesh(); v.updateLabel(); });

  renderer.clear();
  activeViews.forEach(v => {
    const vp = v._vp;
    if (!vp) return;
    renderer.setViewport(vp.x, vp.y, vp.w, vp.h);
    renderer.setScissor(vp.x, vp.y, vp.w, vp.h);
    renderer.render(v.scene, v.camera);
  });
}

buildSubtabs(1);
activateViews();
animate();
