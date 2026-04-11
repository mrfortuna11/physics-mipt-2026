(function(){
const App = window.App = window.App || {};

const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({canvas, antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1f);
scene.fog = new THREE.Fog(0x1a1a1f, 8, 25);

const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 200);
camera.position.set(0,3,6); camera.lookAt(0,1.2,0);

scene.add(new THREE.AmbientLight(0xffffff, 0.35));
const dir = new THREE.DirectionalLight(0xffffff, 1.0);
dir.position.set(3,8,4); scene.add(dir);
const dir2 = new THREE.DirectionalLight(0x88aaff, 0.4);
dir2.position.set(-4,3,-3); scene.add(dir2);

const floorGeo = new THREE.PlaneGeometry(30,30,30,30);
floorGeo.rotateX(-Math.PI/2);
scene.add(new THREE.Mesh(floorGeo, new THREE.MeshStandardMaterial({color:0x25252e, roughness:0.9})));
const grid = new THREE.GridHelper(30,30,0x444455,0x333340);
grid.position.y=0.002; scene.add(grid);

const renderables = [];

function clearRenderables() {
  for (const r of renderables) {
    scene.remove(r.meshTri); r.meshTri.geometry.dispose();
    scene.remove(r.meshLine); r.meshLine.geometry.dispose();
    scene.remove(r.ptPoints); r.ptPoints.geometry.dispose();
  }
  renderables.length = 0;
}

function makeRenderable(firstIdx, lastIdx, tris, color) {
  const n = lastIdx-firstIdx;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(n*3), 3));
  geo.setIndex(tris.map(i=>i-firstIdx));
  const meshTri = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
    color, roughness:0.7, side:THREE.DoubleSide
  }));
  scene.add(meshTri);

  const meshLine = new THREE.LineSegments(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({color:0x88ccff, transparent:true, opacity:0.5}));
  meshLine.visible = false; scene.add(meshLine);

  const ptGeo = new THREE.BufferGeometry();
  ptGeo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(n*3), 3));
  const ptPoints = new THREE.Points(ptGeo, new THREE.PointsMaterial({color:0xffaa44, size:0.1, sizeAttenuation:true}));
  ptPoints.visible = false; scene.add(ptPoints);

  const r = {meshTri, meshLine, ptPoints, tris, firstIdx, lastIdx};
  renderables.push(r);
  return r;
}

function buildEdgeBuffers(sim) {
  for (const r of renderables) {
    let count=0;
    for (const c of sim.C) if (c.i>=r.firstIdx && c.i<r.lastIdx) count++;
    r.meshLine.geometry.setAttribute('position',
      new THREE.BufferAttribute(new Float32Array(count*6), 3));
  }
}

function updateRenderables(sim) {
  for (const r of renderables) {
    const pa = r.meshTri.geometry.attributes.position.array;
    let k=0;
    for (let i=r.firstIdx;i<r.lastIdx;i++) {
      const p=sim.P[i].pos; pa[k++]=p.x; pa[k++]=p.y; pa[k++]=p.z;
    }
    r.meshTri.geometry.attributes.position.needsUpdate = true;
    r.meshTri.geometry.computeVertexNormals();
    r.meshTri.geometry.computeBoundingSphere();

    if (r.ptPoints.visible) {
      const pp = r.ptPoints.geometry.attributes.position.array;
      let j=0;
      for (let i=r.firstIdx;i<r.lastIdx;i++) {
        const p=sim.P[i].pos; pp[j++]=p.x; pp[j++]=p.y; pp[j++]=p.z;
      }
      r.ptPoints.geometry.attributes.position.needsUpdate = true;
      r.ptPoints.material.size = sim.particleRadius*2;
    }
    if (r.meshLine.visible) {
      const la = r.meshLine.geometry.attributes.position;
      const arr = la.array; let m=0;
      for (const c of sim.C) {
        if (c.i<r.firstIdx||c.i>=r.lastIdx) continue;
        const a=sim.P[c.i].pos, b=sim.P[c.j].pos;
        arr[m++]=a.x;arr[m++]=a.y;arr[m++]=a.z;
        arr[m++]=b.x;arr[m++]=b.y;arr[m++]=b.z;
      }
      la.needsUpdate = true;
    }
  }
}

App.render = {
  scene, camera, renderer, canvas, renderables,
  clearRenderables, makeRenderable, buildEdgeBuffers, updateRenderables
};
})();
