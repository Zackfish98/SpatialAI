import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------- Semantic model ----------
// Entity: Cell {x,z} holds at most one Thing.
// Thing: {kind: road|house|tower|shop|park, level}
const N = 12;
const RADIUS = 3;
const KINDS = {
  road:  { label: 'Road',  needsRoad: false },
  house: { label: 'House', needsRoad: true, perLevel: 4 },
  tower: { label: 'Tower', needsRoad: true, perLevel: 10 },
  shop:  { label: 'Shop',  needsRoad: true, capacity: 16 },
  park:  { label: 'Park',  needsRoad: false },
};
const grid = Array.from({ length: N }, () => Array(N).fill(null));
const inB = (x, z) => x >= 0 && z >= 0 && x < N && z < N;
const at = (x, z) => (inB(x, z) ? grid[x][z] : null);
const DIRS = [[1, 0], [-1, 0], [0, 1], [0, -1]];
const touchesRoad = (x, z, ignore) =>
  DIRS.some(([dx, dz]) => { const t = at(x + dx, z + dz); return t && t.kind === 'road' && !(ignore && ignore[0] === x + dx && ignore[1] === z + dz); });
const each = fn => { for (let x = 0; x < N; x++) for (let z = 0; z < N; z++) if (grid[x][z]) fn(grid[x][z], x, z); };
const near = (x, z, kind, r = RADIUS) => {
  let found = 0;
  each((t, tx, tz) => { if (t.kind === kind && Math.max(Math.abs(tx - x), Math.abs(tz - z)) <= r && !(tx === x && tz === z)) found++; });
  return found;
};
const isResidential = t => t.kind === 'house' || t.kind === 'tower';
const residents = t => KINDS[t.kind].perLevel * t.level;

function canPlace(kind, x, z) {
  if (!inB(x, z)) return 'Out of bounds.';
  if (grid[x][z]) return 'Rule 1: cell is occupied.';
  if (KINDS[kind].needsRoad && !touchesRoad(x, z)) return 'Rule 2: ' + KINDS[kind].label + ' must touch a road.';
  if (kind === 'shop' && near(x, z, 'shop', 1)) return 'Rule 7: shops cannot be adjacent.';
  if (kind === 'road') {
    // a road is always allowed; nothing to check
  }
  return null;
}
function canRemove(x, z) {
  const t = at(x, z);
  if (!t) return 'Nothing to demolish.';
  if (t.kind === 'road') {
    for (const [dx, dz] of DIRS) {
      const n = at(x + dx, z + dz);
      if (n && KINDS[n.kind].needsRoad && !touchesRoad(x + dx, z + dz, [x, z])) return 'Rule 3: removing this road would cut off a ' + KINDS[n.kind].label + '.';
    }
  }
  return null;
}
function canUpgrade(x, z) {
  const t = at(x, z);
  if (!t || !isResidential(t)) return 'Only Houses and Towers can be upgraded.';
  if (t.level >= 3) return 'Already at max level.';
  const next = t.level + 1;
  if (next >= 2 && !near(x, z, 'shop')) return 'Rule 5: level 2+ needs a Shop within ' + RADIUS + ' cells.';
  if (next >= 3 && !near(x, z, 'park')) return 'Rule 5: level 3 needs a Park within ' + RADIUS + ' cells.';
  return null;
}
function stats() {
  let pop = 0, houses = 0, shops = 0, parks = 0, roads = 0;
  const served = new Map();
  const shopsList = [];
  each((t, x, z) => { if (t.kind === 'shop') shopsList.push([x, z, 0]); });
  const res = [];
  each((t, x, z) => { if (isResidential(t)) res.push([x, z, residents(t)]); });
  // Rule 6: assign residents to nearest shop with spare capacity
  let happy = 0;
  res.sort((a, b) => a[2] - b[2]);
  for (const [x, z, r] of res) {
    const cand = shopsList.filter(s => Math.max(Math.abs(s[0] - x), Math.abs(s[1] - z)) <= RADIUS && s[2] + r <= KINDS.shop.capacity)
      .sort((a, b) => (Math.abs(a[0] - x) + Math.abs(a[1] - z)) - (Math.abs(b[0] - x) + Math.abs(b[1] - z)));
    if (cand[0]) { cand[0][2] += r; happy += r; served.set(x + ',' + z, true); }
  }
  each(t => { if (isResidential(t)) { pop += residents(t); houses++; } else if (t.kind === 'shop') shops++; else if (t.kind === 'park') parks++; else roads++; });
  return { pop, houses, shops, parks, roads, happy, served };
}

// ---------- Rendering ----------
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(devicePixelRatio);
renderer.shadowMap.enabled = true;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xcfe8f5);
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 200);
camera.position.set(N * 0.9, N * 1.0, N * 1.3);
const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0, 0);
controls.maxPolarAngle = Math.PI / 2.1;
scene.add(new THREE.HemisphereLight(0xffffff, 0x88aa88, 0.8));
const sun = new THREE.DirectionalLight(0xffffff, 1.0);
sun.position.set(10, 20, 8);
sun.castShadow = true;
sun.shadow.camera.left = sun.shadow.camera.bottom = -N;
sun.shadow.camera.right = sun.shadow.camera.top = N;
sun.shadow.mapSize.set(2048, 2048);
scene.add(sun);

const ground = new THREE.Mesh(new THREE.BoxGeometry(N, 0.2, N), new THREE.MeshStandardMaterial({ color: 0x7fbf6a }));
ground.position.y = -0.1; ground.receiveShadow = true;
scene.add(ground);
const gridHelper = new THREE.GridHelper(N, N, 0x4a8a3a, 0x4a8a3a);
gridHelper.position.y = 0.01;
scene.add(gridHelper);
const world = (x, z) => [x - N / 2 + 0.5, z - N / 2 + 0.5];

const things = new THREE.Group(); scene.add(things);
const hover = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ color: 0xffff00, transparent: true, opacity: 0.5 }));
hover.rotation.x = -Math.PI / 2; hover.position.y = 0.03; hover.visible = false; scene.add(hover);

const mat = c => new THREE.MeshStandardMaterial({ color: c });
function box(w, h, d, c, y) { const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat(c)); m.position.y = y; m.castShadow = m.receiveShadow = true; return m; }

function build(t, x, z, served) {
  const g = new THREE.Group();
  const [wx, wz] = world(x, z);
  g.position.set(wx, 0, wz);
  if (t.kind === 'road') {
    g.add(box(1, 0.05, 1, 0x444a52, 0.025));
    for (const [dx, dz] of DIRS) {
      const n = at(x + dx, z + dz);
      if (n && n.kind === 'road') { const l = box(dx ? 0.5 : 0.08, 0.02, dz ? 0.5 : 0.08, 0xe8e8e8, 0.06); l.position.x = dx * 0.25; l.position.z = dz * 0.25; g.add(l); }
    }
  } else if (t.kind === 'house') {
    const h = 0.5 + t.level * 0.35;
    g.add(box(0.7, h, 0.7, [0xf2d6a2, 0xeab676, 0xd9904a][t.level - 1], h / 2));
    const roof = new THREE.Mesh(new THREE.ConeGeometry(0.6, 0.4, 4), mat(0xb5412f));
    roof.position.y = h + 0.2; roof.rotation.y = Math.PI / 4; roof.castShadow = true; g.add(roof);
  } else if (t.kind === 'tower') {
    const h = 0.8 + t.level * 0.9;
    g.add(box(0.75, h, 0.75, [0xb8c4d6, 0x93a6c4, 0x6f88b0][t.level - 1], h / 2));
  } else if (t.kind === 'shop') {
    g.add(box(0.8, 0.5, 0.8, 0x4aa3c9, 0.25));
    g.add(box(0.9, 0.08, 0.9, 0xff8a3d, 0.55));
  } else if (t.kind === 'park') {
    g.add(box(1, 0.04, 1, 0x58b34a, 0.02));
    for (const [tx, tz] of [[-0.25, -0.2], [0.25, 0.2]]) {
      const trunk = box(0.08, 0.3, 0.08, 0x6b4a2b, 0.15); trunk.position.x = tx; trunk.position.z = tz; g.add(trunk);
      const crown = new THREE.Mesh(new THREE.SphereGeometry(0.22, 12, 10), mat(0x2f8f3a)); crown.position.set(tx, 0.45, tz); crown.castShadow = true; g.add(crown);
    }
  }
  if (isResidential(t) && !served) { // unhappy marker
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.1, 10, 8), new THREE.MeshBasicMaterial({ color: 0xe03030 }));
    m.position.y = (t.kind === 'house' ? 0.5 + t.level * 0.35 + 0.6 : 0.8 + t.level * 0.9 + 0.25); g.add(m);
  }
  return g;
}

let tool = 'road';
function refresh() {
  things.clear();
  const s = stats();
  each((t, x, z) => things.add(build(t, x, z, s.served.has(x + ',' + z))));
  document.getElementById('stats').innerHTML =
    `Population: <b>${s.pop}</b> (served/happy: <b>${s.happy}</b>)<br>` +
    `Residences ${s.houses} · Shops ${s.shops} · Parks ${s.parks} · Roads ${s.roads}<br>` +
    `<small>Red dot = residents without a shop in range.</small>`;
}
const msg = document.getElementById('msg');
function say(text, err) { msg.textContent = text; msg.className = err ? 'err' : ''; }

// ---------- Toolbar ----------
const tools = [['road', 'Road'], ['house', 'House'], ['tower', 'Tower'], ['shop', 'Shop'], ['park', 'Park'], ['upgrade', 'Upgrade'], ['demolish', 'Demolish']];
const bar = document.getElementById('toolbar');
const btns = {};
for (const [id, label] of tools) {
  const b = document.createElement('button'); b.textContent = label; b.onclick = () => select(id); bar.appendChild(b); btns[id] = b;
}
bar.appendChild(Object.assign(document.createElement('div'), { className: 'sep' }));
for (const [label, fn] of [['Sample', loadSample], ['Clear', () => { clear(); refresh(); say('Cleared.'); }]]) {
  const b = document.createElement('button'); b.textContent = label; b.onclick = fn; bar.appendChild(b);
}
function select(id) { tool = id; for (const k in btns) btns[k].classList.toggle('active', k === id); say('Tool: ' + id + '. Click a cell.'); }

function clear() { for (let x = 0; x < N; x++) for (let z = 0; z < N; z++) grid[x][z] = null; }
function loadSample() {
  clear();
  for (let x = 1; x <= 10; x++) grid[x][5] = { kind: 'road' };
  for (let z = 1; z <= 9; z++) grid[5][z] = { kind: 'road' };
  const put = (k, x, z, level = 1) => { grid[x][z] = { kind: k, level }; };
  put('house', 2, 4); put('house', 3, 4); put('house', 4, 4, 2); put('house', 2, 6); put('house', 3, 6);
  put('tower', 6, 4); put('house', 7, 6); put('house', 8, 4);
  put('shop', 4, 6); put('shop', 8, 6); put('park', 6, 6); put('park', 9, 3); put('house', 4, 3);
  refresh(); say('Sample neighborhood loaded.');
}

// ---------- Interaction ----------
const ray = new THREE.Raycaster(), ptr = new THREE.Vector2();
function pick(e) {
  ptr.set((e.clientX / innerWidth) * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
  ray.setFromCamera(ptr, camera);
  const hit = ray.intersectObject(ground)[0];
  if (!hit) return null;
  const x = Math.floor(hit.point.x + N / 2), z = Math.floor(hit.point.z + N / 2);
  return inB(x, z) ? [x, z] : null;
}
canvas.addEventListener('pointermove', e => {
  const c = pick(e);
  hover.visible = !!c;
  if (c) { const [wx, wz] = world(...c); hover.position.x = wx; hover.position.z = wz; }
});
let down = null;
canvas.addEventListener('pointerdown', e => { down = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return; // was a camera drag
  const c = pick(e); if (!c) return;
  const [x, z] = c;
  let err;
  if (tool === 'demolish') {
    if (!(err = canRemove(x, z))) { grid[x][z] = null; say('Demolished.'); }
  } else if (tool === 'upgrade') {
    if (!(err = canUpgrade(x, z))) { grid[x][z].level++; say('Upgraded to level ' + grid[x][z].level + '.'); }
  } else if (!(err = canPlace(tool, x, z))) {
    grid[x][z] = { kind: tool, level: 1 }; say('Placed ' + KINDS[tool].label + '.');
  }
  if (err) say(err, true);
  refresh();
});

function resize() { renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); }
addEventListener('resize', resize); resize();
(function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();

select('road');
loadSample();
