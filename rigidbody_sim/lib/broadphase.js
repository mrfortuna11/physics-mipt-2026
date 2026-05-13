// Broadphase: AABB compute + brute / spatial grid / SAP / LBVH.

function bodyComputeAABB(body) {
  const h = body.size;
  const hx = h.x * 0.5, hy = h.y * 0.5, hz = h.z * 0.5;
  const R = quatToMat3(body.q).elements;
  const ex = Math.abs(R[0])*hx + Math.abs(R[3])*hy + Math.abs(R[6])*hz;
  const ey = Math.abs(R[1])*hx + Math.abs(R[4])*hy + Math.abs(R[7])*hz;
  const ez = Math.abs(R[2])*hx + Math.abs(R[5])*hy + Math.abs(R[8])*hz;
  body.aabbMin = [body.x.x - ex, body.x.y - ey, body.x.z - ez];
  body.aabbMax = [body.x.x + ex, body.x.y + ey, body.x.z + ez];
}

function aabbOverlap(a, b) {
  return a.aabbMin[0] <= b.aabbMax[0] && a.aabbMax[0] >= b.aabbMin[0] &&
         a.aabbMin[1] <= b.aabbMax[1] && a.aabbMax[1] >= b.aabbMin[1] &&
         a.aabbMin[2] <= b.aabbMax[2] && a.aabbMax[2] >= b.aabbMin[2];
}

function broadphaseBrute(bodies) {
  const pairs = [];
  for (let i = 0; i < bodies.length; i++) {
    const A = bodies[i];
    for (let j = i + 1; j < bodies.length; j++) {
      const B = bodies[j];
      if (A.mass === 0 && B.mass === 0) continue;
      if (aabbOverlap(A, B)) pairs.push([i, j]);
    }
  }
  return pairs;
}

function broadphaseGridArr(bodies, cellSize) {
  const inv = 1 / cellSize;
  const table = new Map();
  for (let bi = 0; bi < bodies.length; bi++) {
    const b = bodies[bi];
    const i0 = Math.floor(b.aabbMin[0] * inv);
    const j0 = Math.floor(b.aabbMin[1] * inv);
    const k0 = Math.floor(b.aabbMin[2] * inv);
    const i1 = Math.floor(b.aabbMax[0] * inv);
    const j1 = Math.floor(b.aabbMax[1] * inv);
    const k1 = Math.floor(b.aabbMax[2] * inv);
    for (let k = k0; k <= k1; k++)
      for (let j = j0; j <= j1; j++)
        for (let i = i0; i <= i1; i++) {
          const key = (((i*73856093) ^ (j*19349663) ^ (k*83492791)) >>> 0);
          let arr = table.get(key);
          if (!arr) { arr = []; table.set(key, arr); }
          arr.push(bi);
        }
  }
  const seen = new Set();
  const pairs = [];
  for (const arr of table.values()) {
    const m = arr.length;
    if (m < 2) continue;
    for (let a = 0; a < m; a++)
      for (let b = a + 1; b < m; b++) {
        const ia = arr[a], ib = arr[b];
        const lo = Math.min(ia, ib), hi = Math.max(ia, ib);
        const k = lo * 1000003 + hi;
        if (seen.has(k)) continue;
        seen.add(k);
        const A = bodies[lo], B = bodies[hi];
        if (A.mass === 0 && B.mass === 0) continue;
        if (aabbOverlap(A, B)) pairs.push([lo, hi]);
      }
  }
  return pairs;
}

function makeSAPState() { return { axes: [[], [], []] }; }

function broadphaseSAP(bodies, sap) {
  if (sap.axes[0].length !== bodies.length * 2) {
    for (let a = 0; a < 3; a++) {
      sap.axes[a].length = 0;
      for (let i = 0; i < bodies.length; i++) {
        sap.axes[a].push({ pos: bodies[i].aabbMin[a], isMax: false, body: i });
        sap.axes[a].push({ pos: bodies[i].aabbMax[a], isMax: true,  body: i });
      }
      sap.axes[a].sort((x, y) => x.pos - y.pos);
    }
  } else {
    for (let a = 0; a < 3; a++) {
      const arr = sap.axes[a];
      for (const e of arr) {
        e.pos = e.isMax ? bodies[e.body].aabbMax[a] : bodies[e.body].aabbMin[a];
      }
      for (let i = 1; i < arr.length; i++) {
        const cur = arr[i];
        let j = i - 1;
        while (j >= 0 && arr[j].pos > cur.pos) {
          arr[j + 1] = arr[j]; j--;
        }
        arr[j + 1] = cur;
      }
    }
  }

  const overlaps = [new Set(), new Set(), new Set()];
  for (let a = 0; a < 3; a++) {
    const active = new Set();
    for (const e of sap.axes[a]) {
      if (e.isMax) {
        active.delete(e.body);
      } else {
        for (const other of active) {
          const lo = Math.min(other, e.body), hi = Math.max(other, e.body);
          overlaps[a].add(lo * 1000003 + hi);
        }
        active.add(e.body);
      }
    }
  }
  const pairs = [];
  for (const key of overlaps[0]) {
    if (overlaps[1].has(key) && overlaps[2].has(key)) {
      const hi = key % 1000003;
      const lo = (key - hi) / 1000003;
      if (bodies[lo].mass === 0 && bodies[hi].mass === 0) continue;
      pairs.push([lo, hi]);
    }
  }
  return pairs;
}

function _part1by2(v) {
  v &= 0x3ff;
  v = (v | (v << 16)) & 0xff0000ff;
  v = (v | (v << 8))  & 0x0300f00f;
  v = (v | (v << 4))  & 0x030c30c3;
  v = (v | (v << 2))  & 0x09249249;
  return v >>> 0;
}
function _morton30(x, y, z) {
  return _part1by2(x) | (_part1by2(y) << 1) | (_part1by2(z) << 2);
}

function broadphaseLBVH(bodies) {
  const n = bodies.length;
  if (n < 2) return [];

  let mn = [Infinity, Infinity, Infinity];
  let mx = [-Infinity, -Infinity, -Infinity];
  for (const b of bodies) {
    if (b.aabbMin[0] < mn[0]) mn[0] = b.aabbMin[0];
    if (b.aabbMin[1] < mn[1]) mn[1] = b.aabbMin[1];
    if (b.aabbMin[2] < mn[2]) mn[2] = b.aabbMin[2];
    if (b.aabbMax[0] > mx[0]) mx[0] = b.aabbMax[0];
    if (b.aabbMax[1] > mx[1]) mx[1] = b.aabbMax[1];
    if (b.aabbMax[2] > mx[2]) mx[2] = b.aabbMax[2];
  }
  const ext = [mx[0]-mn[0] || 1, mx[1]-mn[1] || 1, mx[2]-mn[2] || 1];

  const entries = new Array(n);
  for (let i = 0; i < n; i++) {
    const b = bodies[i];
    const cx = ((b.aabbMin[0] + b.aabbMax[0]) * 0.5 - mn[0]) / ext[0];
    const cy = ((b.aabbMin[1] + b.aabbMax[1]) * 0.5 - mn[1]) / ext[1];
    const cz = ((b.aabbMin[2] + b.aabbMax[2]) * 0.5 - mn[2]) / ext[2];
    const xi = Math.max(0, Math.min(1023, Math.floor(cx * 1023)));
    const yi = Math.max(0, Math.min(1023, Math.floor(cy * 1023)));
    const zi = Math.max(0, Math.min(1023, Math.floor(cz * 1023)));
    entries[i] = { code: _morton30(xi, yi, zi), bi: i };
  }
  entries.sort((a, b) => a.code - b.code);

  function buildNode(lo, hi) {
    if (lo === hi) {
      const body = bodies[entries[lo].bi];
      return { leaf: true, bi: entries[lo].bi, aabbMin: body.aabbMin, aabbMax: body.aabbMax };
    }
    const mid = (lo + hi) >> 1;
    const L = buildNode(lo, mid);
    const R = buildNode(mid + 1, hi);
    return {
      leaf: false, left: L, right: R,
      aabbMin: [
        Math.min(L.aabbMin[0], R.aabbMin[0]),
        Math.min(L.aabbMin[1], R.aabbMin[1]),
        Math.min(L.aabbMin[2], R.aabbMin[2]),
      ],
      aabbMax: [
        Math.max(L.aabbMax[0], R.aabbMax[0]),
        Math.max(L.aabbMax[1], R.aabbMax[1]),
        Math.max(L.aabbMax[2], R.aabbMax[2]),
      ],
    };
  }
  const root = buildNode(0, n - 1);

  const pairs = [];
  function nodesOverlap(A, B) {
    return A.aabbMin[0] <= B.aabbMax[0] && A.aabbMax[0] >= B.aabbMin[0] &&
           A.aabbMin[1] <= B.aabbMax[1] && A.aabbMax[1] >= B.aabbMin[1] &&
           A.aabbMin[2] <= B.aabbMax[2] && A.aabbMax[2] >= B.aabbMin[2];
  }
  function emitPair(i, j) {
    if (i === j) return;
    const a = bodies[i], b = bodies[j];
    if (a.mass === 0 && b.mass === 0) return;
    if (i < j) pairs.push([i, j]); else pairs.push([j, i]);
  }
  function descend(A, B) {
    if (!nodesOverlap(A, B)) return;
    if (A.leaf && B.leaf) { emitPair(A.bi, B.bi); return; }
    if (A.leaf)            { descend(A, B.left); descend(A, B.right); return; }
    if (B.leaf)            { descend(A.left, B); descend(A.right, B); return; }
    descend(A.left,  B.left);
    descend(A.left,  B.right);
    descend(A.right, B.left);
    descend(A.right, B.right);
  }
  function selfPairs(node) {
    if (node.leaf) return;
    selfPairs(node.left);
    selfPairs(node.right);
    descend(node.left, node.right);
  }
  selfPairs(root);
  return pairs;
}
