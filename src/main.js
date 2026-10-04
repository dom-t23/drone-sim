import * as THREE from 'three';
import { makeSim, step, GATE_INNER, v3, DIFFICULTIES } from './sim.js';

const DT = 1 / 120;
const $ = (id) => document.getElementById(id);

// ---------- renderer ----------
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
} catch (e) {
  $('err').style.display = 'grid';
  throw e;
}
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
const SKY = new THREE.Color('#1a2a44');
scene.background = SKY;
scene.fog = new THREE.Fog(SKY, 60, 220);

scene.add(new THREE.HemisphereLight('#9fc3ff', '#2a2218', 0.9));
const sun = new THREE.DirectionalLight('#ffd7b0', 1.6);
sun.position.set(-60, 90, 40);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -90, right: 90, top: 90, bottom: -90, near: 10, far: 250 });
scene.add(sun);

// ground
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(600, 600),
  new THREE.MeshStandardMaterial({ color: '#1d2b22', roughness: 1 })
);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(600, 120, '#3a5246', '#26372e');
grid.position.y = 0.01;
scene.add(grid);

// ---------- cameras ----------
const chaseCam = new THREE.PerspectiveCamera(62, innerWidth / innerHeight, 0.1, 600);
const fpvCam = new THREE.PerspectiveCamera(85, 16 / 9, 0.05, 400);
let camMode = 'chase'; // chase | orbit | fpv
const CAM_MODES = ['chase', 'orbit', 'fpv'];

// ---------- world objects (rebuilt per course) ----------
let sim, seed = 1, difficulty = 'normal', pilot = 'racing', world = new THREE.Group(), racingLine = null;
scene.add(world);

const gateMatIdle = new THREE.MeshStandardMaterial({ color: '#d9e2ef', roughness: 0.5, metalness: 0.1 });
const gateMatNext = new THREE.MeshStandardMaterial({ color: '#ff8a3d', emissive: '#ff6a10', emissiveIntensity: 0.6 });
const gateMatDone = new THREE.MeshStandardMaterial({ color: '#3ddc97', emissive: '#12a46a', emissiveIntensity: 0.35 });
let gateMeshes = [];

function makeGate(gate) {
  const g = new THREE.Group();
  const t = 0.28, s = GATE_INNER;
  const bars = [
    [s + 2 * t, t, 0, (s + t) / 2],
    [s + 2 * t, t, 0, -(s + t) / 2],
    [t, s, (s + t) / 2, 0],
    [t, s, -(s + t) / 2, 0],
  ];
  for (const [w, h, x, y] of bars) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, t), gateMatIdle);
    m.position.set(x, y, 0);
    m.castShadow = true;
    g.add(m);
  }
  g.position.set(...gate.pos);
  g.lookAt(gate.pos[0] + gate.normal[0], gate.pos[1] + gate.normal[1], gate.pos[2] + gate.normal[2]);
  return g;
}

// Vertical support legs from the ground to the gate's two bottom corners, in world space
// so angled and dive gates still stand on straight legs.
const legMat = new THREE.MeshStandardMaterial({ color: '#56657a' });
function makeLegs(gate) {
  const s = GATE_INNER, t = 0.28, legs = [];
  for (const side of [-1, 1]) {
    const c = v3.sub(v3.add(gate.pos, v3.scale(gate.right, side * (s + t) / 2)), v3.scale(gate.up, s / 2 + t));
    if (c[1] < 0.1) continue;
    const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, c[1], 8), legMat);
    leg.position.set(c[0], c[1] / 2, c[2]);
    legs.push(leg);
  }
  return legs;
}

function setGateMat(i, mat) {
  gateMeshes[i].children.forEach((c, k) => { if (k < 4) c.material = mat; });
}

function buildWorld() {
  scene.remove(world);
  world.traverse((o) => o.geometry?.dispose?.());
  world = new THREE.Group();
  scene.add(world);

  sim = makeSim({ seed, pilot, difficulty });
  writeHash();
  gateMeshes = sim.course.gates.map((g) => {
    const m = makeGate(g);
    world.add(m, ...makeLegs(g));
    return m;
  });
  setGateMat(0, gateMatNext);

  // planned racing line, coloured by planned speed (blue = braking for a turn, orange = flat out)
  const plan = sim.plan;
  const linePos = new Float32Array(plan.pts.length * 3);
  const lineCol = new Float32Array(plan.pts.length * 3);
  const vLo = Math.min(...plan.pts.map((q) => q.v)), vHi = Math.max(...plan.pts.map((q) => q.v));
  const slow = new THREE.Color('#3d8bff'), fast = new THREE.Color('#ff8a3d'), c = new THREE.Color();
  plan.pts.forEach((q, i) => {
    linePos.set(q.p, i * 3);
    c.copy(slow).lerp(fast, (q.v - vLo) / Math.max(1e-6, vHi - vLo));
    lineCol.set([c.r, c.g, c.b], i * 3);
  });
  const lineGeo = new THREE.BufferGeometry();
  lineGeo.setAttribute('position', new THREE.BufferAttribute(linePos, 3));
  lineGeo.setAttribute('color', new THREE.BufferAttribute(lineCol, 3));
  racingLine = new THREE.LineLoop(lineGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.75 }));
  racingLine.visible = pilot === 'racing';
  world.add(racingLine);

  // scenery for parallax: scattered pillars away from the course
  const rand = mulberry(seed * 7919);
  const pillarGeo = new THREE.BoxGeometry(1, 1, 1);
  const pillarMat = new THREE.MeshStandardMaterial({ color: '#2e3d55', roughness: 0.9 });
  const pillars = new THREE.InstancedMesh(pillarGeo, pillarMat, 90);
  const m4 = new THREE.Matrix4();
  for (let i = 0; i < 90; i++) {
    const a = rand() * Math.PI * 2;
    const r = rand() < 0.5 ? 8 + rand() * 18 : 75 + rand() * 90;
    const h = 2 + rand() * (r > 60 ? 22 : 6);
    const w = 1 + rand() * 3;
    m4.compose(
      new THREE.Vector3(Math.cos(a) * r, h / 2, Math.sin(a) * r),
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), rand() * Math.PI),
      new THREE.Vector3(w, h, w)
    );
    pillars.setMatrixAt(i, m4);
  }
  pillars.castShadow = pillars.receiveShadow = true;
  world.add(pillars);

  trailCount = 0;
  lastEvents = 0;
  updateHud(true);
}

// ---------- drone model ----------
const drone = new THREE.Group();
{
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.34, 0.1, 0.42),
    new THREE.MeshStandardMaterial({ color: '#20242c', metalness: 0.4, roughness: 0.5 })
  );
  body.castShadow = true;
  drone.add(body);
  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(0.12, 0.08, 0.1),
    new THREE.MeshStandardMaterial({ color: '#ff8a3d', emissive: '#ff6a10', emissiveIntensity: 0.5 })
  );
  nose.position.set(0, 0.02, 0.24);
  drone.add(nose);
  const armMat = new THREE.MeshStandardMaterial({ color: '#2f3640' });
  const propMat = new THREE.MeshBasicMaterial({ color: '#cfd8e3', transparent: true, opacity: 0.35, side: THREE.DoubleSide });
  drone.userData.props = [];
  for (const [x, z] of [[1, 1], [-1, 1], [1, -1], [-1, -1]]) {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.03, 0.42), armMat);
    arm.position.set(x * 0.14, 0, z * 0.14);
    arm.rotation.y = Math.atan2(x, z);
    arm.castShadow = true;
    drone.add(arm);
    const prop = new THREE.Mesh(new THREE.CircleGeometry(0.13, 20), propMat);
    prop.rotation.x = -Math.PI / 2;
    prop.position.set(x * 0.29, 0.05, z * 0.29);
    drone.add(prop);
  }
  drone.scale.setScalar(2.2); // a little larger than life so it reads on screen
}
scene.add(drone);

// onboard camera mounted on the nose, tilted up like a real FPV cam
const camMount = new THREE.Object3D();
camMount.position.set(0, 0.06, 0.2);
camMount.rotation.x = THREE.MathUtils.degToRad(-18);
drone.add(camMount);

// ---------- trail ----------
const TRAIL_N = 900;
const trailPos = new Float32Array(TRAIL_N * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: '#ffb27a', transparent: true, opacity: 0.55 }));
trail.frustumCulled = false;
scene.add(trail);
let trailCount = 0;
function pushTrail(p) {
  if (trailCount < TRAIL_N) trailCount++;
  else trailPos.copyWithin(0, 3);
  trailPos.set(p, (trailCount - 1) * 3);
  trailGeo.setDrawRange(0, trailCount);
  trailGeo.attributes.position.needsUpdate = true;
}

// ---------- HUD ----------
let lastEvents = 0;
function fmt(t) { return t == null ? '–' : `${t.toFixed(2)} s`; }
function updateHud(force) {
  const s = sim.state;
  $('s-lap').textContent = s.lap || '–';
  $('s-last').textContent = fmt(s.laps.at(-1));
  $('s-best').textContent = fmt(s.laps.length ? Math.min(...s.laps) : null);
  $('s-speed').textContent = `${(v3.len(sim.drone.vel) * 3.6).toFixed(0)} km/h`;
  $('s-gates').textContent = s.gatesPassed;
  $('s-plan').textContent = pilot === 'racing' ? fmt(sim.plan.lapTime) : '–';
  $('s-course').textContent = `#${seed} ${difficulty}`;
  if (force) return;
  for (; lastEvents < sim.events.length; lastEvents++) {
    const e = sim.events[lastEvents];
    if (e.type === 'lap') toast(`Lap ${fmt(e.time)}`);
    if (e.type === 'gate') onGate(e.gate);
  }
}
function onGate(id) {
  const n = sim.course.gates.length;
  setGateMat(id, gateMatDone);
  setGateMat((id + 1) % n, gateMatNext);
  if (id === n - 1) for (let i = 1; i < n - 1; i++) setGateMat(i, gateMatIdle);
  if (id === 0) setGateMat(n - 1, gateMatIdle);
}
let toastTimer;
function toast(msg) {
  const el = $('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 1300);
}

// ---------- controls ----------
let timeScale = 1;
$('b-cam').onclick = () => {
  camMode = CAM_MODES[(CAM_MODES.indexOf(camMode) + 1) % CAM_MODES.length];
  $('b-cam').textContent = `Camera: ${camMode === 'fpv' ? 'onboard' : camMode}`;
};
$('b-speed').onclick = () => {
  timeScale = timeScale === 1 ? 2 : timeScale === 2 ? 4 : 1;
  $('b-speed').textContent = `Speed: ${timeScale}×`;
};
$('b-course').onclick = () => { seed++; buildWorld(); };
$('b-diff').onclick = () => {
  difficulty = DIFFICULTIES[(DIFFICULTIES.indexOf(difficulty) + 1) % DIFFICULTIES.length];
  syncButtons();
  buildWorld();
};
$('b-share').onclick = () => {
  const url = location.href;
  if (navigator.clipboard?.writeText) {
    navigator.clipboard.writeText(url).then(() => toast('Link copied'), () => toast(`#seed=${seed}&d=${difficulty}`));
  } else toast(`#seed=${seed}&d=${difficulty}`);
};
function syncButtons() {
  $('b-diff').textContent = `Course: ${difficulty}`;
}

// ---------- shareable course in the URL: #seed=12&d=hard ----------
function readHash() {
  const p = new URLSearchParams(location.hash.slice(1));
  const n = parseInt(p.get('seed'), 10);
  seed = Number.isInteger(n) && n > 0 && n < 1e9 ? n : 1;
  difficulty = DIFFICULTIES.includes(p.get('d')) ? p.get('d') : 'normal';
  syncButtons();
}
function writeHash() {
  const h = `#seed=${seed}&d=${difficulty}`;
  if (location.hash !== h) history.replaceState(null, '', h);
}
addEventListener('hashchange', () => {
  const before = `${seed}/${difficulty}`;
  readHash();
  if (`${seed}/${difficulty}` !== before) buildWorld();
});
$('b-pilot').onclick = () => {
  pilot = pilot === 'racing' ? 'pursuit' : 'racing';
  $('b-pilot').textContent = `Pilot: ${pilot === 'racing' ? 'racing line' : 'pursuit'}`;
  buildWorld();
};
addEventListener('keydown', (e) => {
  if (e.key === 'c') $('b-cam').click();
  if (e.key === 'n') $('b-course').click();
  if (e.key === 'p') $('b-pilot').click();
  if (e.key === 'd') $('b-diff').click();
});

// ---------- main loop ----------
const tmpV = new THREE.Vector3(), tmpQ = new THREE.Quaternion(), yawQ = new THREE.Quaternion();
const UP = new THREE.Vector3(0, 1, 0);
let acc = 0, last = performance.now(), propSpin = 0;

function syncDrone() {
  const d = sim.drone;
  drone.position.set(...d.pos);
  yawQ.setFromAxisAngle(UP, d.yaw);
  tmpV.set(...v3.norm(d.thrust));
  tmpQ.setFromUnitVectors(UP, tmpV);
  drone.quaternion.copy(tmpQ).multiply(yawQ);
}

function updateCameras(dtFrame) {
  const d = sim.drone;
  const k = 1 - Math.exp(-dtFrame * 3.5);
  if (camMode === 'chase') {
    const back = new THREE.Vector3(-Math.sin(d.yaw), 0, -Math.cos(d.yaw));
    const want = new THREE.Vector3(...d.pos).addScaledVector(back, 9).add(new THREE.Vector3(0, 3.2, 0));
    chaseCam.position.lerp(want, k);
    chaseCam.lookAt(d.pos[0], d.pos[1] + 0.6, d.pos[2]);
  } else if (camMode === 'orbit') {
    const a = performance.now() * 0.00006;
    chaseCam.position.lerp(new THREE.Vector3(Math.cos(a) * 95, 55, Math.sin(a) * 95), k * 0.5);
    chaseCam.lookAt(0, 4, 0);
  }
  camMount.getWorldPosition(fpvCam.position);
  camMount.getWorldQuaternion(fpvCam.quaternion);
  fpvCam.rotateY(Math.PI); // three.js cameras look down -Z; the drone's nose is +Z
}

function frame(now) {
  const dtFrame = Math.min(0.1, (now - last) / 1000);
  last = now;
  acc += dtFrame * timeScale;
  let steps = 0;
  while (acc >= DT && steps < 1200) {
    step(sim, DT);
    acc -= DT;
    if (++steps % 6 === 0) pushTrail(sim.drone.pos);
  }

  syncDrone();
  propSpin += dtFrame * 60;
  updateCameras(dtFrame);
  updateHud();

  const W = innerWidth, H = innerHeight;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, W, H);
  if (camMode === 'fpv') {
    fpvCam.aspect = W / H;
    fpvCam.updateProjectionMatrix();
    drone.visible = false;
    renderer.render(scene, fpvCam);
    drone.visible = true;
    $('fpv-label').style.display = 'none';
  } else {
    renderer.render(scene, chaseCam);
    // picture-in-picture onboard view
    const w = Math.round(Math.min(W * 0.3, 360)), h = Math.round(w * 9 / 16);
    const x = W - w - 14, y = Math.max(78, H * 0.12);
    fpvCam.aspect = w / h;
    fpvCam.updateProjectionMatrix();
    renderer.setScissorTest(true);
    renderer.setScissor(x, y, w, h);
    renderer.setViewport(x, y, w, h);
    drone.visible = false;
    renderer.render(scene, fpvCam);
    drone.visible = true;
    const lbl = $('fpv-label');
    lbl.style.display = 'block';
    lbl.style.bottom = `${y + h + 4}px`;
  }
  requestAnimationFrame(frame);
}

addEventListener('resize', () => {
  renderer.setSize(innerWidth, innerHeight);
  chaseCam.aspect = innerWidth / innerHeight;
  chaseCam.updateProjectionMatrix();
});

function mulberry(a) {
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

readHash();
buildWorld();
chaseCam.position.set(sim.drone.pos[0], 6, sim.drone.pos[2] - 12);
requestAnimationFrame(frame);
