import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------- Semantic model ----------
// Environment: store floor (x<8) + loading zone outside the door (x>8), on a nav grid.
// Obstacles: shelves, walls, parked car (static) and People (dynamic).
// Items: lumber etc. on shelves. Client: car with listed type (app) and confirmed type (on site).
const CELL = 0.4, X0 = -16, Z0 = -10, GW = 85, GH = 50;
const RR = 0.7, PR = 0.3, PERSON_BLOCK = 1.0;
const DOCK = { x: -14.4, z: 0 }, WAIT = { x: 9.8, z: 8 };
const REAR_X = 11;
const BAYS = [{ name: 'A', z: -5.5 }, { name: 'B', z: 3.5 }];
const CARS = {
  truck: { label: 'Pickup truck', L: 5.2, W: 2.0, h: 1.1, maxLen: 3.0, gap: 1.7, reach: 0.9 },
  sedan: { label: 'Sedan (trunk)', L: 4.6, W: 1.9, h: 0.9, maxLen: 1.3, gap: 1.5, reach: 0.6 },
  suv:   { label: 'SUV (hatch)', L: 4.8, W: 2.0, h: 1.0, maxLen: 2.5, gap: 1.9, reach: 0.8 },
};
const TYPES = {
  stud:  { label: '2x4x8 Stud', len: 2.4, h: 0.09, w: 0.14, color: 0xd9b27c },
  ply:   { label: 'Plywood 4x8', len: 2.4, h: 0.05, w: 0.9, color: 0xe9d3a3 },
  board: { label: '1x4x4 Board', len: 1.2, h: 0.04, w: 0.2, color: 0xc89a5b },
  dowel: { label: 'Dowel 36"', len: 0.9, h: 0.06, w: 0.06, color: 0xb98550 },
  pvc:   { label: 'PVC Pipe 4ft', len: 1.2, h: 0.08, w: 0.08, color: 0xf2f2f2 },
};
const MAX_CARRY = 3;

// ---------- Static geometry & grids ----------
const shelves = [];
for (const cx of [-9, 2]) for (const z of [-6, -2, 2, 6]) shelves.push({ x: cx, z, hx: 4.5, hz: 0.45, face: z < 0 ? (z === -6 ? 1 : -1) : (z === 2 ? 1 : -1) });
const walls = [{ x: 8, z: -6, hx: 0.15, hz: 4 }, { x: 8, z: 6, hx: 0.15, hz: 4 }];
const aabbs = [...shelves, ...walls];
const gi = (cx, cz) => cz * GW + cx;
const toCell = (x, z) => [Math.floor((x - X0) / CELL), Math.floor((z - Z0) / CELL)];
const toWorld = (cx, cz) => [X0 + (cx + 0.5) * CELL, Z0 + (cz + 0.5) * CELL];

function buildGrid(r, extra = []) {
  const g = new Uint8Array(GW * GH), boxes = aabbs.concat(extra);
  for (let cz = 0; cz < GH; cz++) for (let cx = 0; cx < GW; cx++) {
    const [wx, wz] = toWorld(cx, cz);
    if (wx < X0 + r || wx > X0 + GW * CELL - r || wz < Z0 + r || wz > Z0 + GH * CELL - r) { g[gi(cx, cz)] = 1; continue; }
    for (const b of boxes) {
      const dx = Math.max(Math.abs(wx - b.x) - b.hx, 0), dz = Math.max(Math.abs(wz - b.z) - b.hz, 0);
      if (dx * dx + dz * dz < r * r) { g[gi(cx, cz)] = 1; break; }
    }
  }
  return g;
}
function nearestFree(g, wx, wz) {
  let [cx, cz] = toCell(wx, wz);
  cx = Math.min(GW - 1, Math.max(0, cx)); cz = Math.min(GH - 1, Math.max(0, cz));
  if (!g[gi(cx, cz)]) return [cx, cz];
  for (let r = 1; r < 20; r++) for (let dz = -r; dz <= r; dz++) for (let dx = -r; dx <= r; dx++) {
    if (Math.max(Math.abs(dx), Math.abs(dz)) !== r) continue;
    const x = cx + dx, z = cz + dz;
    if (x >= 0 && z >= 0 && x < GW && z < GH && !g[gi(x, z)]) return [x, z];
  }
  return [cx, cz];
}
function stamp(g, wx, wz, r) {
  const [c0, c1] = toCell(wx - r, wz - r), [d0, d1] = toCell(wx + r, wz + r);
  for (let cz = Math.max(0, c1); cz <= Math.min(GH - 1, d1); cz++) for (let cx = Math.max(0, c0); cx <= Math.min(GW - 1, d0); cx++) {
    const [px, pz] = toWorld(cx, cz);
    if ((px - wx) ** 2 + (pz - wz) ** 2 < r * r) g[gi(cx, cz)] = 1;
  }
}
function los(g, a, b) {
  const [ax, az] = toWorld(...a), [bx, bz] = toWorld(...b);
  const n = Math.ceil(Math.hypot(bx - ax, bz - az) / (CELL * 0.4));
  for (let i = 0; i <= n; i++) {
    const [cx, cz] = toCell(ax + (bx - ax) * i / n, az + (bz - az) * i / n);
    if (g[gi(cx, cz)]) return false;
  }
  return true;
}
function astar(g, s, t) {
  const N = GW * GH, gs = new Float32Array(N).fill(Infinity), from = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
  const heap = []; // [f, idx]
  const push = it => { heap.push(it); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { let l = 2 * i + 1, r = l + 1, m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
  const h = (x, z) => { const dx = Math.abs(x - t[0]), dz = Math.abs(z - t[1]); return Math.max(dx, dz) + 0.414 * Math.min(dx, dz); };
  const si = gi(...s), ti = gi(...t); gs[si] = 0; push([h(...s), si]);
  while (heap.length) {
    const [, i] = pop(); if (closed[i]) continue; closed[i] = 1;
    if (i === ti) { const out = []; for (let k = i; k !== -1; k = from[k]) out.push([k % GW, (k / GW) | 0]); return out.reverse(); }
    const cx = i % GW, cz = (i / GW) | 0;
    for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) {
      if (!dx && !dz) continue;
      const x = cx + dx, z = cz + dz; if (x < 0 || z < 0 || x >= GW || z >= GH || g[gi(x, z)]) continue;
      if (dx && dz && (g[gi(cx + dx, cz)] || g[gi(cx, cz + dz)])) continue;
      const j = gi(x, z), ng = gs[i] + (dx && dz ? 1.414 : 1);
      if (ng < gs[j]) { gs[j] = ng; from[j] = i; push([ng + h(x, z), j]); }
    }
  }
  return null;
}
function smoothPath(g, pts) {
  const out = [pts[0]]; let i = 0;
  while (i < pts.length - 1) { let j = pts.length - 1; while (j > i + 1 && !los(g, pts[i], pts[j])) j--; out.push(pts[j]); i = j; }
  return out;
}

// ---------- Scene ----------
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(devicePixelRatio); renderer.shadowMap.enabled = true;
const scene = new THREE.Scene(); scene.background = new THREE.Color(0xb9d3e6);
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 300);
camera.position.set(-2, 24, 20);
const controls = new OrbitControls(camera, canvas); controls.target.set(1, 0, 0); controls.maxPolarAngle = Math.PI / 2.05;
scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 0.9); sun.position.set(8, 22, 10); sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -22, right: 22, top: 14, bottom: -14 }); sun.shadow.mapSize.set(2048, 2048); scene.add(sun);

const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, ...o });
function box(w, h, d, c, x = 0, y = 0, z = 0, parent) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat(c)); m.position.set(x, y, z); m.castShadow = m.receiveShadow = true; if (parent) parent.add(m); return m;
}
function label(text, x, z, color = '#223', size = 1.6) {
  const c = document.createElement('canvas'); c.width = 256; c.height = 64; const g = c.getContext('2d');
  g.font = 'bold 34px system-ui'; g.fillStyle = color; g.textAlign = 'center'; g.fillText(text, 128, 44);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  s.scale.set(size * 4, size, 1); s.position.set(x, 0.3, z); s.renderOrder = 5; scene.add(s); return s;
}
const flat = (w, d, c, x, z, y = 0.01) => { const m = new THREE.Mesh(new THREE.PlaneGeometry(w, d), mat(c)); m.rotation.x = -Math.PI / 2; m.position.set(x, y, z); m.receiveShadow = true; scene.add(m); return m; };

flat(60, 40, 0x93b779, 0, 0, -0.02);
const floor = flat(24, 20, 0xd8d8d4, -4, 0, 0);       // store floor
flat(10, 20, 0x4b4f55, 13, 0, 0);                      // asphalt loading zone
const pickGround = new THREE.Mesh(new THREE.PlaneGeometry(60, 40), new THREE.MeshBasicMaterial({ visible: false }));
pickGround.rotation.x = -Math.PI / 2; scene.add(pickGround);
for (const s of shelves) {
  box(s.hx * 2, 1.6, s.hz * 2, 0xe8751a, s.x, 0.8, s.z, scene);
  box(s.hx * 2, 0.06, s.hz * 2 + 0.1, 0x555b66, s.x, 1.63, s.z, scene);
}
for (const w of walls) box(w.hx * 2, 1.8, w.hz * 2, 0xb8b2a6, w.x, 0.9, w.z, scene);
box(24, 0.8, 0.3, 0xb8b2a6, -4, 0.4, -10.15, scene); box(24, 0.8, 0.3, 0xb8b2a6, -4, 0.4, 10.15, scene); box(0.3, 0.8, 20, 0xb8b2a6, -16.15, 0.4, 0, scene);
flat(2.4, 2.4, 0x2e8b3d, DOCK.x, DOCK.z, 0.02); label('DOCK', DOCK.x, DOCK.z + 1.8, '#2e8b3d', 1.1);
flat(3, 3, 0xf2c200, WAIT.x, WAIT.z, 0.02); label('WAITING AREA', WAIT.x, WAIT.z + 2.2, '#a07d00', 1.1);
label('LOADING ZONE', 13, -9, '#fff', 1.4); label('DOOR', 8, -0.1, '#223', 0.9);
const bayMarks = BAYS.map(b => {
  const m = flat(6, 2.8, 0x4b4f55, REAR_X + 3, b.z, 0.015);
  for (const s of [-1, 1]) flat(6, 0.08, 0xffffff, REAR_X + 3, b.z + s * 1.4, 0.03);
  label('SPOT ' + b.name, REAR_X + 3, b.z, '#fff', 1.2); return m;
});

// ---------- Items ----------
const items = []; let itemGroup = new THREE.Group(); scene.add(itemGroup);
function makeStock() {
  items.length = 0; itemGroup.clear();
  const keys = Object.keys(TYPES); let n = 0;
  for (const s of shelves) for (const dx of [-3, 0, 3]) {
    const type = keys[(n * 2 + (n >> 2)) % keys.length], T = TYPES[type];
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(T.len, T.h, T.w), mat(T.color, { emissive: 0x000000 }));
    mesh.castShadow = true; mesh.position.set(s.x + dx, 1.7, s.z); itemGroup.add(mesh);
    const tag = new THREE.Mesh(new THREE.ConeGeometry(0.2, 0.45, 8), new THREE.MeshBasicMaterial({ color: 0x22cc44 }));
    tag.rotation.x = Math.PI; tag.position.y = 0.6; tag.visible = false; mesh.add(tag);
    items.push({ id: n++, type, len: T.len, mesh, tag, x: s.x + dx, z: s.z, face: s.face, state: 'shelf' });
  }
}
const pickPoint = it => ({ x: it.x, z: it.z + it.face * 1.3 });

// ---------- Robot ----------
const robot = new THREE.Group(); scene.add(robot);
box(1.2, 0.35, 0.9, 0x2255aa, 0, 0.3, 0, robot); box(0.5, 0.5, 0.7, 0xf96302, -0.3, 0.7, 0, robot);
box(0.12, 2.3, 0.12, 0x333a44, 0.45, 1.3, 0.35, robot); box(0.12, 2.3, 0.12, 0x333a44, 0.45, 1.3, -0.35, robot);
for (const [x, z] of [[-0.4, 0.5], [0.4, 0.5], [-0.4, -0.5], [0.4, -0.5]]) { const w = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.12, 14), mat(0x111)); w.rotation.x = Math.PI / 2; w.position.set(x, 0.18, z); robot.add(w); }
const platform = new THREE.Group(); robot.add(platform);
box(0.8, 0.06, 0.8, 0xf2c200, 0, 0, 0, platform);
const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 8), new THREE.MeshBasicMaterial({ color: 0x00ff66 })); beacon.position.set(-0.3, 1.1, 0); robot.add(beacon);

const R = { x: DOCK.x, z: DOCK.z, a: 0, lift: 0.35, ext: 0, phase: 'idle', t: 0, goal: null, path: [], blocked: false, replan: 0, onArrive: null, carried: [], queue: [], speed: 2.2, started: false, hold: false };
let robotGrid = buildGrid(RR), peopleGrid = buildGrid(PR);
function applyRobot() { robot.position.set(R.x, 0, R.z); robot.rotation.y = -R.a; platform.position.set(0.55 + R.ext, R.lift, 0); }

// ---------- People (dynamic obstacles) ----------
const people = []; let freeze = false;
function addPerson(x, z) {
  const g = new THREE.Group(), c = [0xd33, 0x38c, 0x6a3, 0xa5a, 0xe90][people.length % 5];
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.3, 1.1, 14), mat(c)); body.position.y = 0.65; body.castShadow = true; g.add(body);
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.2, 14, 10), mat(0xf0c8a0)); head.position.y = 1.4; head.castShadow = true; g.add(head);
  scene.add(g);
  if (x === undefined) { do { x = X0 + 1 + Math.random() * 32; z = Z0 + 1 + Math.random() * 18; } while (peopleGrid[gi(...toCell(x, z))] || Math.hypot(x - R.x, z - R.z) < 3); }
  const p = { x, z, mesh: g, path: [], wait: 0 }; people.push(p); g.position.set(x, 0, z);
}
function personPlan(p) {
  for (let k = 0; k < 8; k++) {
    const tx = X0 + 1 + Math.random() * 32, tz = Z0 + 1 + Math.random() * 18, t = toCell(tx, tz);
    if (peopleGrid[gi(...t)]) continue;
    const s = nearestFree(peopleGrid, p.x, p.z), path = astar(peopleGrid, s, t);
    if (path) { p.path = smoothPath(peopleGrid, path).map(c => toWorld(...c)).slice(1); return; }
  }
  p.wait = 1;
}

// ---------- Cars (the Client) ----------
let car = null;
function buildCar(type) {
  const S = CARS[type], g = new THREE.Group(), body = { truck: 0x2a5d9f, sedan: 0xb22, suv: 0x3a7a4a }[type], parts = {};
  const wheel = (x, z) => { const w = new THREE.Mesh(new THREE.CylinderGeometry(0.45, 0.45, 0.3, 16), mat(0x111)); w.rotation.x = Math.PI / 2; w.position.set(x, 0.45, z); w.castShadow = true; g.add(w); };
  [-1, 1].forEach(s => { wheel(-S.L / 2 + 0.9, s * (S.W / 2 - 0.1)); wheel(S.L / 2 - 0.9, s * (S.W / 2 - 0.1)); });
  const glass = 0x9ec9e8;
  if (type === 'truck') {
    box(S.L, 0.15, S.W, body, 0, 0.9, 0, g); box(1.9, 1.1, S.W, body, S.L / 2 - 0.95, 1.5, 0, g); box(0.1, 0.5, S.W - 0.3, glass, S.L / 2 - 0.05, 1.7, 0, g);
    box(S.L - 1.9, 0.12, S.W, 0x333a44, -0.95, S.h - 0.06, 0, g);
    for (const s of [-1, 1]) box(S.L - 1.9, 0.5, 0.1, body, -0.95, S.h + 0.25, s * (S.W / 2 - 0.05), g);
    box(0.1, 0.5, S.W, body, S.L / 2 - 1.9, S.h + 0.25, 0, g);
    const lid = new THREE.Group(); lid.position.set(-S.L / 2, S.h, 0); box(0.08, 0.5, S.W, body, 0, 0.25, 0, lid); g.add(lid); parts.lid = lid; parts.max = Math.PI / 2;
  } else if (type === 'sedan') {
    box(S.L, 0.6, S.W, body, 0, 0.65, 0, g); box(2.2, 0.6, S.W - 0.2, glass, 0.2, 1.25, 0, g);
    const lid = new THREE.Group(); lid.position.set(-S.L / 2 + 1.0, 0.96, 0); box(1.0, 0.06, S.W - 0.1, body, -0.5, 0, 0, lid); g.add(lid); parts.lid = lid; parts.max = -1.1;
  } else {
    box(S.L, 1.0, S.W, body, 0, 0.9, 0, g); box(S.L - 1.0, 0.7, S.W - 0.1, glass, -0.2, 1.75, 0, g);
    const lid = new THREE.Group(); lid.position.set(-S.L / 2, 2.1, 0); box(0.08, 1.0, S.W - 0.2, body, 0, -0.5, 0, lid); g.add(lid); parts.lid = lid; parts.max = -1.7;
  }
  g.userData = parts; return g;
}
const carBox = c => ({ x: REAR_X + CARS[c.actual].L / 2, z: BAYS[c.bay].z, hx: CARS[c.actual].L / 2, hz: CARS[c.actual].W / 2 });
function rebuildGrids() { const ex = car && car.state !== 'departing' ? [carBox(car)] : []; robotGrid = buildGrid(RR, ex); peopleGrid = buildGrid(PR, ex); }

// ---------- Mission logic ----------
const logEl = document.getElementById('log'), logLines = [];
function log(m, err) { logLines.unshift({ m, err }); logLines.length = Math.min(logLines.length, 6); logEl.innerHTML = logLines.map(l => `<div class="${l.err ? 'err' : ''}">${l.err ? '⚠ ' : '• '}${l.m}</div>`).join(''); }
const ordered = () => items.filter(i => i.state === 'ordered' || i.state === 'carried');

function navTo(x, z, cb) {
  const c = nearestFree(robotGrid, x, z); [R.goal] = [toWorld(...c)]; R.onArrive = cb; R.phase = 'nav'; R.replan = 0; R.path = []; R.blocked = false;
}
function plan() {
  const dyn = robotGrid.slice(); for (const p of people) stamp(dyn, p.x, p.z, PERSON_BLOCK);
  const s = toCell(R.x, R.z); for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) if (s[0] + dx >= 0 && s[1] + dz >= 0 && s[0] + dx < GW && s[1] + dz < GH) dyn[gi(s[0] + dx, s[1] + dz)] = robotGrid[gi(s[0] + dx, s[1] + dz)];
  const t = toCell(...R.goal);
  const path = dyn[gi(...t)] ? null : astar(dyn, s, t);
  if (!path) { R.blocked = true; R.path = []; return; }
  R.blocked = false;
  const pts = smoothPath(dyn, path).map(c => toWorld(...c)).slice(1); pts[pts.length - 1] = R.goal.slice(); R.path = pts;
}
const angDiff = (a, b) => Math.atan2(Math.sin(b - a), Math.cos(b - a));
function turnTo(angle, cb) { R.faceAngle = angle; R.afterTurn = cb; R.phase = 'turn'; }
function setPhase(p) { R.phase = p; R.t = 0; }
const lerpTo = (v, to, rate, dt) => v + Math.max(-rate * dt, Math.min(rate * dt, to - v));

function startMission() {
  if (R.phase !== 'idle') return log('Mission already in progress.', true);
  const list = ordered(); if (!list.length) return log('Order is empty. Add items first.', true);
  R.started = true; let p = { x: R.x, z: R.z }; R.queue = [];
  while (list.length) { list.sort((a, b) => Math.hypot(a.x - p.x, a.z - p.z) - Math.hypot(b.x - p.x, b.z - p.z)); const it = list.shift(); R.queue.push(it); p = it; }
  log(`Mission started: ${R.queue.length} item(s).`); nextItem();
}
function nextItem() {
  if (!R.queue.length) { log('All items picked. Heading to waiting area.'); return navTo(WAIT.x, WAIT.z, () => { setPhase('waiting'); }); }
  const it = R.queue.shift(), pp = pickPoint(it);
  navTo(pp.x, pp.z, () => turnTo(-it.face * Math.PI / 2, () => { R.target = it; setPhase('pick'); }));
}
function fits() { const S = CARS[car.confirmed]; return ordered().every(i => i.len <= S.maxLen); }
function stopPoint(type) { return { x: REAR_X - CARS[type].gap, z: BAYS[car.bay].z }; }

function updateRobot(dt) {
  R.t += dt;
  switch (R.phase) {
    case 'nav': {
      R.replan -= dt; if (R.replan <= 0) { plan(); R.replan = 0.3; }
      R.hold = false; if (R.blocked || !R.path.length) { R.hold = R.blocked; if (!R.blocked && !R.path.length) { R.x = R.goal[0]; R.z = R.goal[1]; const cb = R.onArrive; setPhase('idle'); cb && cb(); } break; }
      const [wx, wz] = R.path[0], dx = wx - R.x, dz = wz - R.z, d = Math.hypot(dx, dz), want = Math.atan2(dz, dx);
      const diff = angDiff(R.a, want); R.a += Math.max(-4 * dt, Math.min(4 * dt, diff));
      for (const p of people) { const px = p.x - R.x, pz = p.z - R.z; if (Math.hypot(px, pz) < 1.05 && px * dx + pz * dz > 0) R.hold = true; }
      if (!R.hold && Math.abs(diff) < 0.7) { const step = Math.min(d, R.speed * dt); R.x += dx / d * step; R.z += dz / d * step; }
      if (d < 0.08) R.path.shift();
      break;
    }
    case 'turn': { const diff = angDiff(R.a, R.faceAngle); R.a += Math.max(-3 * dt, Math.min(3 * dt, diff)); if (Math.abs(diff) < 0.03) { const cb = R.afterTurn; R.afterTurn = null; setPhase('idle'); cb && cb(); } break; }
    case 'pick': {
      const t = R.t;
      if (t < 0.7) R.lift = lerpTo(R.lift, 1.8, 3, dt);
      else if (t < 1.4) R.ext = lerpTo(R.ext, 0.75, 1.5, dt);
      else {
        if (R.target) { const it = R.target; R.target = null; it.state = 'carried'; it.tag.visible = false; platform.add(it.mesh); it.mesh.position.set(0, 0.1 + R.carried.length * 0.2, 0); it.mesh.rotation.set(0, 0, 0); R.carried.push(it); log(`Picked ${TYPES[it.type].label}.`); }
        if (t < 2.1) R.ext = lerpTo(R.ext, 0, 1.5, dt); else if (t < 2.8) R.lift = lerpTo(R.lift, 0.35, 2.5, dt); else nextItem();
      }
      break;
    }
    case 'waiting': {
      if (!car) break;
      if (car.state === 'confirmed') {
        const S = CARS[car.confirmed];
        if (!fits()) { setPhase('assist'); log(`Cargo too long for ${S.label} (max ${S.maxLen} m). Staff assist requested.`, true); break; }
        const sp = stopPoint(car.confirmed); log(`Approaching ${S.label} at spot ${BAYS[car.bay].name}.`);
        navTo(sp.x, sp.z, () => turnTo(0, () => setPhase('load')));
      }
      break;
    }
    case 'load': {
      const t = R.t, S = CARS[car.confirmed], lid = car.mesh.userData;
      if (t < 1.3) { R.lift = lerpTo(R.lift, S.h + 0.1, 1.2, dt); lid.lid.rotation.z = lerpTo(lid.lid.rotation.z, lid.max, 1.4, dt); }
      else if (t < 2.4) R.ext = lerpTo(R.ext, S.reach, 1.2, dt);
      else {
        if (R.carried.length) {
          R.carried.forEach((it, i) => { it.state = 'loaded'; car.mesh.add(it.mesh); it.mesh.position.set(-S.L / 2 + 0.25 + it.len / 2, S.h - 0.05 + 0.08 + i * 0.12, (i - 1) * 0.45); it.mesh.rotation.set(0, 0, 0); });
          log(`Loaded ${R.carried.length} item(s) into ${S.label}.`); R.carried = [];
        }
        if (t < 3.5) R.ext = lerpTo(R.ext, 0, 1.2, dt);
        else if (t < 4.6) { R.lift = lerpTo(R.lift, 0.35, 1.2, dt); if (car.confirmed !== 'truck') lid.lid.rotation.z = lerpTo(lid.lid.rotation.z, 0, 1.4, dt); }
        else {
          car.state = 'loaded'; log('Client loaded. Robot returning to dock.');
          navTo(DOCK.x, DOCK.z, () => { R.started = false; turnTo(0, () => log('Mission complete.')); });
          setTimeout(() => { if (car) { car.state = 'departing'; rebuildGrids(); } }, 1200);
        }
      }
      break;
    }
  }
}

function updateCar(dt) {
  if (!car) return;
  const S = CARS[car.actual], fx = REAR_X + S.L / 2;
  if (car.state === 'arriving') { const dx = car.mesh.position.x - fx; car.mesh.position.x -= Math.min(dx, Math.max(0.6, dx * 0.9) * 3.2 * dt * 0.5 + 0.02); if (car.mesh.position.x - fx < 0.02) { car.mesh.position.x = fx; car.state = 'parked'; log(`Car parked at spot ${BAYS[car.bay].name}. Waiting for client to confirm location.`); } }
  else if (car.state === 'departing') { car.mesh.position.x += 4 * dt; if (car.mesh.position.x > 36) { scene.remove(car.mesh); car = null; rebuildGrids(); } }
}
function updatePeople(dt) {
  if (freeze) return;
  for (const p of people) {
    if (p.wait > 0) { p.wait -= dt; continue; }
    if (!p.path.length) { personPlan(p); continue; }
    const [wx, wz] = p.path[0], dx = wx - p.x, dz = wz - p.z, d = Math.hypot(dx, dz), step = Math.min(d, 1.0 * dt);
    p.x += dx / d * step; p.z += dz / d * step; if (d < 0.05) p.path.shift();
    p.mesh.position.set(p.x, 0, p.z); p.mesh.rotation.y = -Math.atan2(dz, dx);
  }
}

// ---------- Toolbar ----------
const bar = document.getElementById('toolbar'), ctl = {};
function group(title) { const g = document.createElement('div'); g.className = 'grp'; g.innerHTML = `<span class="t">${title}</span>`; bar.appendChild(g); return g; }
function button(g, text, fn, cls = '') { const b = document.createElement('button'); b.textContent = text; b.className = cls; b.onclick = fn; g.appendChild(b); return b; }
function select(g, opts, val) { const s = document.createElement('select'); for (const [k, v] of Object.entries(opts)) s.add(new Option(v, k)); if (val) s.value = val; g.appendChild(s); return s; }
let g = group('Order'); ctl.item = select(g, Object.fromEntries(Object.entries(TYPES).map(([k, v]) => [k, v.label])));
button(g, 'Add item', () => {
  if (R.phase !== 'idle' || R.started) return log('Cannot change the order during a mission.', true);
  if (ordered().length >= MAX_CARRY) return log(`Rule 1: robot carries at most ${MAX_CARRY} items.`, true);
  const cand = items.filter(i => i.type === ctl.item.value && i.state === 'shelf').sort((a, b) => Math.hypot(a.x - DOCK.x, a.z - DOCK.z) - Math.hypot(b.x - DOCK.x, b.z - DOCK.z))[0];
  if (!cand) return log('Out of stock.', true);
  cand.state = 'ordered'; cand.tag.visible = true; log(`Ordered ${TYPES[cand.type].label}.`);
});
button(g, 'Clear order', () => { if (R.started) return log('Cannot change the order during a mission.', true); items.forEach(i => { if (i.state === 'ordered') { i.state = 'shelf'; i.tag.visible = false; } }); log('Order cleared.'); });
g = group('Client'); const carOpts = Object.fromEntries(Object.entries(CARS).map(([k, v]) => [k, v.label]));
const lbl = t => { const s = document.createElement('span'); s.className = 't'; s.textContent = t; g.appendChild(s); };
lbl('app lists'); ctl.listed = select(g, carOpts, 'sedan'); lbl('actually'); ctl.actual = select(g, carOpts, 'sedan');
button(g, 'Car arrives', () => {
  if (car) return log('A client car is already here.', true);
  const bay = Math.floor(Math.random() * BAYS.length), actual = ctl.actual.value;
  const mesh = buildCar(actual); mesh.position.set(REAR_X + CARS[actual].L / 2 + 16, 0, BAYS[bay].z); scene.add(mesh);
  car = { listed: ctl.listed.value, actual, confirmed: null, bay, mesh, state: 'arriving' };
  bayMarks.forEach((m, i) => m.material.color.set(i === bay ? 0x2e6b3a : 0x4b4f55)); rebuildGrids();
  log(`Client (app lists ${CARS[car.listed].label}) is heading to spot ${BAYS[bay].name}.`);
});
button(g, 'Client confirms', () => {
  if (!car || car.state !== 'parked') return log('Rule 3: client can confirm only once parked at the spot.', true);
  car.confirmed = car.actual; car.state = 'confirmed';
  log(car.confirmed === car.listed ? `Confirmed: ${CARS[car.confirmed].label} (matches app).` : `Confirmed on site: ${CARS[car.confirmed].label} — differs from app listing (${CARS[car.listed].label}). Using confirmed type.`, car.confirmed !== car.listed);
});
g = group('Obstacles'); button(g, 'Add person', () => addPerson());
ctl.place = button(g, 'Place person (click floor)', () => { placing = !placing; ctl.place.classList.toggle('on', placing); });
button(g, 'Remove person', () => { const p = people.pop(); if (p) scene.remove(p.mesh); });
ctl.freeze = button(g, 'Freeze people', () => { freeze = !freeze; ctl.freeze.classList.toggle('on', freeze); ctl.freeze.textContent = freeze ? 'Unfreeze people' : 'Freeze people'; });
g = group('Mission'); button(g, 'Start', startMission, 'go');
ctl.pause = button(g, 'Pause', () => { paused = !paused; ctl.pause.textContent = paused ? 'Resume' : 'Pause'; ctl.pause.classList.toggle('on', paused); });
button(g, 'Reset', reset);
let placing = false, paused = false;

function reset() {
  if (car) scene.remove(car.mesh); car = null; logLines.length = 0; logEl.innerHTML = '';
  Object.assign(R, { x: DOCK.x, z: DOCK.z, a: 0, lift: 0.35, ext: 0, phase: 'idle', t: 0, goal: null, path: [], blocked: false, onArrive: null, carried: [], queue: [], started: false, hold: false });
  makeStock(); bayMarks.forEach(m => m.material.color.set(0x4b4f55)); rebuildGrids(); log('Reset.');
}

canvas.addEventListener('pointermove', () => {});
let down = null; canvas.addEventListener('pointerdown', e => { down = [e.clientX, e.clientY]; });
const ray = new THREE.Raycaster(), ptr = new THREE.Vector2();
canvas.addEventListener('pointerup', e => {
  if (!placing || !down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
  ptr.set(e.clientX / innerWidth * 2 - 1, -(e.clientY / innerHeight) * 2 + 1); ray.setFromCamera(ptr, camera);
  const h = ray.intersectObject(pickGround)[0]; if (!h) return;
  const c = toCell(h.point.x, h.point.z); if (c[0] < 0 || c[1] < 0 || c[0] >= GW || c[1] >= GH || peopleGrid[gi(...c)]) return log('Cannot place a person there.', true);
  addPerson(h.point.x, h.point.z);
});

// ---------- HUD ----------
const PHASE = { idle: 'Idle at dock', nav: 'Navigating', turn: 'Aligning', pick: 'Picking item', waiting: 'At waiting area', load: 'Loading vehicle', assist: 'Staff assist needed' };
function hud() {
  let s = PHASE[R.phase] || R.phase;
  if (R.phase === 'nav') s = R.blocked ? 'Path blocked by people — waiting / re-planning' : R.hold ? 'Person ahead — holding' : 'Navigating';
  if (R.phase === 'waiting') s = !car ? 'Waiting for client car to arrive at spot' : car.state === 'confirmed' ? 'Client confirmed' : car.state === 'parked' ? 'Car parked — awaiting client confirmation' : 'Waiting for car to park';
  document.getElementById('state').textContent = 'Robot: ' + s + (paused ? ' (paused)' : '');
  const ord = items.filter(i => i.state !== 'shelf').map(i => `${TYPES[i.type].label} <small>(${i.len} m, ${i.state})</small>`);
  document.getElementById('info').innerHTML = `Order (${ordered().length}/${MAX_CARRY}): ${ord.length ? ord.join(', ') : '<i>none</i>'}<br>` +
    (car ? `Car: app says <b>${CARS[car.listed].label}</b>${car.confirmed ? `, confirmed <b>${CARS[car.confirmed].label}</b>` : ', <i>unconfirmed</i>'} · spot ${BAYS[car.bay].name} · ${car.state}` : 'Car: none') + `<br>People: ${people.length}${freeze ? ' (frozen)' : ''}`;
}

function resize() { renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); }
addEventListener('resize', resize); resize();
let last = performance.now(), hudT = 0;
function loop(now) {
  requestAnimationFrame(loop); const dt = Math.min(0.05, (now - last) / 1000); last = now;
  if (!paused) { updatePeople(dt); updateCar(dt); updateRobot(dt); }
  const tick = ordered().length; items.forEach(i => { if (i.tag.visible) i.tag.position.y = 0.6 + Math.sin(now / 250) * 0.08; });
  beacon.material.color.set(R.blocked || R.hold ? 0xff3322 : 0x00ff66);
  applyRobot(); controls.update(); renderer.render(scene, camera);
  hudT -= dt; if (hudT <= 0) { hud(); hudT = 0.2; }
}
makeStock(); for (let i = 0; i < 3; i++) addPerson(); applyRobot(); log('Add items, then Start. Spawn a car and have the client confirm.');
requestAnimationFrame(loop);
