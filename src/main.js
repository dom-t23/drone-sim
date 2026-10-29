import * as THREE from 'three';
import { makeSim, step, GATE_INNER, v3, bodyAxes, DIFFICULTIES, NAV_MODES, WIND_MODES, PILOTS, DRONE, FRAME, navError, shareTraffic, bumpAll, cameraBlackout, cameraDark, EYES } from './sim.js';
import { buildScene } from './eye.js';
import { TV, makeDirector, subjectsOf, direct, pinSubject, replayShot, endReplay, makeReplay, recordReplay, replayPose, replayWindow } from './director.js';
import { makeRace, syncRace, raceGap } from './race.js';
import { makeGhostRecorder, recordGhost, ghostPose, encodeGhost, decodeGhost } from './ghost.js';
import { makeTelemetry, resetTelemetry, sampleTelemetry, drawTelemetry } from './telemetry.js';
import { ROSTER, rosterFor, makeField, syncField, standings, gapText, results, RACE } from './field.js';
import { motorTone, makeWash, emitWash, stepWash, WASH } from './fx.js';
import { makeAudio } from './audio.js';
import { makeLanding } from './landing.js';
import { learnSummary, learnNote, confLabel, drawLearnChart } from './learn.js';

const DT = 1 / 120;
const initialHash = location.hash; // before the app writes its own (a plain visit gets the intro)
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

const hemi = new THREE.HemisphereLight('#9fc3ff', '#2a2218', 0.9);
scene.add(hemi);
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
let camMode = 'chase'; // chase | orbit | fpv | tv
const CAM_MODES = ['chase', 'orbit', 'fpv', 'tv'];
const CAM_LABELS = { chase: 'chase', orbit: 'orbit', fpv: 'onboard', tv: 'TV' };
const CHASE_FOV = 62;

// ---------- world objects (rebuilt per course) ----------
let sim, seed = 1, difficulty = 'normal', pilot = 'racing', nav = 'truth', world = new THREE.Group(), racingLine = null;
let ghosts = [], lineVersion = -1, windMode = 'off';
// how the camera finds gates: 'model' (a modelled corner detector) or 'pixels' (vision step 3)
let eyes = 'model';
// learn: the racing pilot learns from its own laps (iterative learning control, sim.js)
let learnOn = false;
const canLearn = () => pilot === 'racing' && nav === 'truth' && !fieldOn;
const hasCamera = () => nav === 'vision' || nav === 'vio';
// manual flight: you fly `sim`, the racing-line autopilot flies `rival` on the same course
let rival = null, race = null, pb = null;
// ghost replay: a gold drone flying the best lap so far (yours are kept per course)
let ghostRec = makeGhostRecorder(), ghostOn = true;
const COUNTDOWN = 3; // s on the pad before a race starts
// multi-drone race: a field of autopilots with different tunings, plus a timing tower.
// field.entries[0] is always the hero (sim, the drone the HUD describes); followId picks
// which drone the cameras follow.
let fieldOn = false, field = null, followId = 'hero', lastLeader = null;
let fieldSims = [], flagShown = false; // all the field's sims, for traffic and contact
const HERO_COLOR = '#ff8a3d';
// TV mode: a race director cutting between trackside posts, flybys, chase, heli and onboard
// shots, with slow-motion replays of crashes, lead changes, laps and the chequered flag.
// During a replay the race is paused and every drone flies its recorded poses again.
let director = null, replayBuf = makeReplay(), replay = null, replayQueue = null, lastReplay = -Infinity;
let tvSeen = new Map(), tvShot = null, labelZoom = 1;
const tvOn = () => camMode === 'tv';
const telemetry = makeTelemetry($('telemetry'));
scene.add(world);

const gateMatIdle = new THREE.MeshStandardMaterial({ color: '#d9e2ef', roughness: 0.5, metalness: 0.1 });
const gateMatNext = new THREE.MeshStandardMaterial({ color: '#ff8a3d', emissive: '#ff6a10', emissiveIntensity: 0.6 });
const gateMatDone = new THREE.MeshStandardMaterial({ color: '#3ddc97', emissive: '#12a46a', emissiveIntensity: 0.35 });
let gateMeshes = [];

// ---------- night mode: LED gates, light pools, stars, a moon and a headlight ----------
let night = false;
function softTexture(draw) {
  const cv = document.createElement('canvas');
  cv.width = cv.height = 128;
  draw(cv.getContext('2d'));
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}
// a soft square ring the shape of the gate frame (bar centres at 30..98 px of 128)
const FRAME_GLOW_SIZE = ((GATE_INNER + 0.28) / 2) * (128 / 34);
const frameGlowTex = softTexture((c) => {
  c.strokeStyle = '#fff';
  c.shadowColor = '#fff';
  c.lineWidth = 6;
  for (const blur of [22, 14, 6]) {
    c.shadowBlur = blur;
    c.strokeRect(30, 30, 68, 68);
  }
});
const radialTex = softTexture((c) => {
  const g = c.createRadialGradient(64, 64, 0, 64, 64, 64);
  g.addColorStop(0, 'rgba(255,255,255,1)');
  g.addColorStop(0.35, 'rgba(255,255,255,.45)');
  g.addColorStop(1, 'rgba(255,255,255,0)');
  c.fillStyle = g;
  c.fillRect(0, 0, 128, 128);
});
const glowMat = (map, color, opacity) => new THREE.MeshBasicMaterial({
  map, color, opacity, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.DoubleSide,
});
const LED = { idle: '#6fa8ff', next: '#ff7a1a', done: '#22e08f' };
const glowMats = { idle: glowMat(frameGlowTex, LED.idle, 0.7), next: glowMat(frameGlowTex, LED.next, 1), done: glowMat(frameGlowTex, LED.done, 0.85) };
const poolMats = { idle: glowMat(radialTex, LED.idle, 0.22), next: glowMat(radialTex, LED.next, 0.5), done: glowMat(radialTex, LED.done, 0.3) };
// how the gate frames themselves look by day and lit up at night: [colour, emissive, intensity]
const GATE_LOOK = {
  idle: [gateMatIdle, ['#d9e2ef', '#000000', 0], ['#202a3a', LED.idle, 1.5]],
  next: [gateMatNext, ['#ff8a3d', '#ff6a10', 0.6], ['#ff8a3d', LED.next, 2.4]],
  done: [gateMatDone, ['#3ddc97', '#12a46a', 0.35], ['#3ddc97', LED.done, 1.7]],
};
const LOOK = {
  day: { sky: '#1a2a44', fog: [60, 220], hemi: 0.9, sun: ['#ffd7b0', 1.6], ground: '#1d2b22', grid: 1 },
  night: { sky: '#03060d', fog: [45, 200], hemi: 0.1, sun: ['#9db8ff', 0.3], ground: '#0c140f', grid: 0.4 },
};
// the next gate lights the ground and the drone around it
const gateLight = new THREE.PointLight(LED.next, 60, 28, 2);
gateLight.visible = false;
scene.add(gateLight);
// FPV headlight on the drone being watched
const headlight = new THREE.SpotLight('#e4edff', 120, 60, 0.5, 0.55, 1.6);
headlight.visible = false;
scene.add(headlight, headlight.target);
// stars and a moon, beyond the fog
const stars = (() => {
  const n = 900, pos = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const a = Math.random() * Math.PI * 2, y = 0.04 + 0.96 * Math.random() ** 1.6, r = Math.sqrt(1 - y * y);
    pos.set([Math.cos(a) * r * 380, y * 380, Math.sin(a) * r * 380], i * 3);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const pts = new THREE.Points(geo, new THREE.PointsMaterial({ color: '#dfe8ff', size: 1.6, sizeAttenuation: false, fog: false, transparent: true, opacity: 0.85 }));
  pts.visible = false;
  return pts;
})();
const moon = new THREE.Sprite(new THREE.SpriteMaterial({ map: radialTex, color: '#dfe8ff', fog: false, depthWrite: false }));
moon.position.set(-60, 90, 40).setLength(360);
moon.scale.setScalar(34);
moon.visible = false;
scene.add(stars, moon);

function applyLook() {
  const L = LOOK[night ? 'night' : 'day'];
  scene.background.set(L.sky);
  scene.fog.color.set(L.sky);
  [scene.fog.near, scene.fog.far] = L.fog;
  hemi.intensity = L.hemi;
  sun.color.set(L.sun[0]);
  sun.intensity = L.sun[1];
  ground.material.color.set(L.ground);
  grid.material.color.setScalar(L.grid);
  for (const [mat, day, nite] of Object.values(GATE_LOOK)) {
    const [c, e, k] = night ? nite : day;
    mat.color.set(c);
    mat.emissive.set(e);
    mat.emissiveIntensity = k;
  }
  for (const g of gateMeshes) g.userData.glow.visible = g.userData.pool.visible = night;
  gateLight.visible = headlight.visible = stars.visible = moon.visible = night;
  ledMat.forEach((m) => (m.opacity = night ? 1 : 0.8));
  washMat.blending = night ? THREE.AdditiveBlending : THREE.NormalBlending;
  washMat.needsUpdate = true;
  $('b-night').classList.toggle('on', night);
}

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
  // night: a soft LED glow around the frame, and a pool of its light on the ground
  const glow = new THREE.Mesh(new THREE.PlaneGeometry(FRAME_GLOW_SIZE, FRAME_GLOW_SIZE), glowMats.idle);
  glow.visible = night;
  g.add(glow);
  const pool = new THREE.Mesh(new THREE.PlaneGeometry(10, 10), poolMats.idle);
  pool.rotation.x = -Math.PI / 2;
  pool.position.set(gate.pos[0], 0.03, gate.pos[2]);
  pool.visible = night;
  g.userData = { glow, pool };
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

// "Ghost" gates: where the drone believes each gate is (vision and map-only modes).
// Thin cyan frames drawn on top of everything, like an AR overlay.
const ghostMat = new THREE.MeshBasicMaterial({ color: '#4dd2ff', transparent: true, opacity: 0.85, depthTest: false });
function makeGhost(b) {
  const g = new THREE.Group(), s = GATE_INNER, t = 0.07;
  for (const [w, h, x, y] of [[s + t, t, 0, s / 2], [s + t, t, 0, -s / 2], [t, s + t, s / 2, 0], [t, s + t, -s / 2, 0]]) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, t), ghostMat);
    m.position.set(x, y, 0);
    m.renderOrder = 10;
    g.add(m);
  }
  g.position.set(...b.pos);
  g.lookAt(b.pos[0] + b.normal[0], b.pos[1] + b.normal[1], b.pos[2] + b.normal[2]);
  return g;
}

function setGateMat(i, mat) {
  const g = gateMeshes[i];
  g.children.forEach((c, k) => { if (k < 4) c.material = mat; });
  const key = mat === gateMatNext ? 'next' : mat === gateMatDone ? 'done' : 'idle';
  g.userData.glow.material = glowMats[key];
  g.userData.pool.material = poolMats[key];
  if (key === 'next') gateLight.position.copy(g.position);
}

function buildWorld() {
  scene.remove(world);
  world.traverse((o) => o.geometry?.dispose?.());
  world = new THREE.Group();
  scene.add(world);

  const manual = pilot === 'manual';
  // you fly by eye, so your sim knows the true course; the rival uses the chosen nav mode
  sim = makeSim({ seed, pilot, difficulty, nav: manual ? 'truth' : nav, wind: windMode, launchAt: manual || fieldOn ? COUNTDOWN : null, optimise: !manual, eyes, learn: learnOn && canLearn() });
  // a race against one rival, unless the whole field is racing
  rival = manual && !fieldOn ? makeSim({ seed, pilot: 'racing', difficulty, nav, wind: windMode, launchAt: COUNTDOWN, eyes }) : null;
  race = rival ? makeRace() : null;
  pb = manual ? loadPb() : null;
  ghostRec = makeGhostRecorder(manual ? loadGhost() : null);
  ghostMesh.visible = false;
  rivalMesh.visible = !!rival;
  buildField();
  $('countdown').hidden = true;
  $('pads').hidden = !(manual && coarse);
  writeHash();
  gateMeshes = sim.course.gates.map((g) => {
    const m = makeGate(g);
    world.add(m, m.userData.pool, ...makeLegs(g));
    return m;
  });
  setGateMat(0, gateMatNext);
  ghosts = (sim.belief?.gates ?? []).map((b) => {
    const ghost = makeGhost(b);
    world.add(ghost);
    return ghost;
  });
  racingLine = null;
  drawRacingLine();
  resetTelemetry(telemetry);
  showNavNote();
  learnDrawn = -1;
  updateLearn();

  // scenery for parallax: scattered pillars away from the course (solid: sim.js owns them)
  const list = sim.scenery.pillars;
  const pillarGeo = new THREE.BoxGeometry(1, 1, 1);
  const pillarMat = new THREE.MeshStandardMaterial({ color: '#2e3d55', roughness: 0.9 });
  const pillars = new THREE.InstancedMesh(pillarGeo, pillarMat, Math.max(1, list.length));
  pillars.count = list.length;
  const m4 = new THREE.Matrix4();
  list.forEach((q, i) => {
    m4.compose(
      new THREE.Vector3(...q.pos),
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), q.yaw),
      new THREE.Vector3(...q.size)
    );
    pillars.setMatrixAt(i, m4);
  });
  pillars.castShadow = pillars.receiveShadow = true;
  world.add(pillars);

  trailCount = 0;
  lastEvents = 0;
  darkWatch = null;
  director = null;
  stopReplay();
  updateHud(true);
}

// ---------- TV mode ----------
function tvEntries() {
  return field ? field.entries : [{ id: 'hero', name: heroName(), color: HERO_COLOR, sim, mesh: drone }];
}
function buildDirector() {
  const boxes = buildScene(sim.course.gates, sim.scenery.pillars, { inner: GATE_INNER, bar: FRAME.bar, legRadius: FRAME.legRadius });
  director = makeDirector({ course: sim.course, plan: (rival ?? sim).plan, boxes, seed });
  replayBuf = makeReplay();
  replayQueue = null;
  lastReplay = -Infinity;
  tvSeen = new Map();
}
// Ask for a replay of drone id around sim time te. Higher prio wins; the flag always does.
function queueReplay(id, te, title, prio) {
  if (!tvOn() || !director || replay || pilot === 'manual') return;
  if (prio < 4 && sim.t - lastReplay < TV.replay.cooldown) return;
  if (replayQueue && replayQueue.prio >= prio) return;
  replayQueue = { id, te, title, prio, at: te + TV.replay.after };
}
// New crashes (any drone), split-S manoeuvres and, flying solo, every second lap.
function tvWatch() {
  for (const e of tvEntries()) {
    const ev = e.sim.events;
    if (!tvSeen.has(e.id)) tvSeen.set(e.id, ev.length);
    for (let i = tvSeen.get(e.id); i < ev.length; i++) {
      const x = ev[i];
      if (x.type === 'crash') queueReplay(e.id, x.t, `${e.id === 'hero' && !field ? 'Crash' : `${e.name} crashes`}`, 3);
      // a split-S (pro courses), slowed right down: the roll to inverted and the pull through
      if (x.type === 'splits') queueReplay(e.id, x.from + 0.5, `${e.id === 'hero' && !field ? 'Split-S' : `${e.name}: split-S`} · ${x.inverted.toFixed(2)} s upside down`, 2);
      if (x.type === 'lap' && !field && e.sim.state.laps.length % 2 === 0) {
        // flying solo: every second lap, the fastest gate of that lap that is still in the buffer
        let best = null;
        for (const g of ev) {
          if (g.type !== 'gate' || g.t < x.t - x.time || g.t < sim.t - TV.replay.keep + TV.replay.before) continue;
          const p = replayPose(replayBuf, e.id, g.t), v = p ? v3.len(p.vel) : 0;
          if (!best || v > best.v) best = { g, v };
        }
        if (best) queueReplay(e.id, best.g.t, `Lap ${e.sim.state.laps.length} · ${x.time.toFixed(2)} s · fastest gate ${best.g.gate + 1}, ${(best.v * 3.6).toFixed(0)} km/h`, 1);
      }
    }
    tvSeen.set(e.id, ev.length);
  }
  if (replayQueue && sim.t >= replayQueue.at) startReplay();
}
let wipeTimer;
function startReplay() {
  const q = replayQueue;
  replayQueue = null;
  const w = replayWindow(replayBuf, q.te), pose = w && replayPose(replayBuf, q.id, q.te);
  if (!pose) return;
  replayShot(director, q.id, pose.pos, pose.vel);
  replay = { ...w, t: w.from, id: q.id, title: q.title };
  lastReplay = sim.t;
  document.body.classList.add('replaying');
  $('tv-skip').hidden = false;
  const wipe = $('tv-wipe');
  wipe.classList.remove('go');
  void wipe.offsetWidth; // restart the animation
  wipe.classList.add('go');
  clearTimeout(wipeTimer);
  wipeTimer = setTimeout(() => wipe.classList.remove('go'), 1000);
}
function stopReplay() {
  if (!replay) return;
  replay = null;
  if (director) endReplay(director);
  document.body.classList.remove('replaying');
  $('tv-skip').hidden = true;
  trailCount = 0; // the trail restarts from where the drone is now
}
$('tv-skip').onclick = () => stopReplay();
// Fly every drone's recorded pose at the replay clock.
function applyReplay(dtFrame) {
  replay.t += dtFrame * TV.replay.speed;
  if (replay.t >= replay.to) { stopReplay(); return; }
  for (const e of tvEntries()) {
    const p = replayPose(replayBuf, e.id, replay.t);
    if (!p) continue;
    syncMesh(e.mesh, p);
    e.label?.position.set(p.pos[0], p.pos[1] + 1.1, p.pos[2]);
  }
  ghostMesh.visible = false;
  estMesh.visible = drMesh.visible = estLine.visible = false;
}
function tvSubjects() {
  if (!replay) return subjectsOf(tvEntries());
  return tvEntries().map((e) => {
    const p = replayPose(replayBuf, e.id, replay.t);
    return p && { id: e.id, name: e.name, pos: p.pos, vel: p.vel, yaw: p.yaw, flying: true, crashed: false, progress: 0, leg: 1 };
  }).filter(Boolean);
}
// The TV graphics: LIVE / REPLAY bug with the shot name, and a lower third for the drone in shot.
let tvHtml = '';
function updateTvGraphics(H) {
  const el = $('tv');
  el.hidden = !tvOn() || !tvShot;
  if (el.hidden) return;
  el.style.bottom = `${H - controlsTop + 10}px`;
  if (frameNo % 3 !== 0 && !tvShot.cut) return;
  const e = tvEntries().find((x) => x.id === tvShot.subject) ?? tvEntries()[0];
  const p = replay ? replayPose(replayBuf, e.id, replay.t) : e.sim.drone;
  const kmh = p ? (v3.len(p.vel) * 3.6).toFixed(0) : '–';
  let info;
  if (replay) info = `${esc(replay.title)} · ${TV.replay.speed}× · ${kmh} km/h`;
  else if (field) {
    const r = standings(field, sim.course.gates.length).find((x) => x.id === e.id);
    info = `P${r.pos} · lap ${Math.min(field.laps, Math.max(1, r.lap))}/${field.laps} · ${kmh} km/h${e.sim.crash ? ' · <em>crashed</em>' : ''}`;
  } else info = `lap ${Math.max(1, e.sim.state.lap)} · ${kmh} km/h${e.sim.crash ? ' · <em>crashed</em>' : ''}`;
  // the bug and the lower third are rebuilt (and slide in) only when the shot or the drone
  // changes; the live numbers just update in place
  const head = `<div class="tv-bug"><span class="tv-live${replay ? ' rp' : ''}">${replay ? 'Replay' : 'Live'}</span><span>${esc(tvShot.label)}</span>${director?.pin ? '<span class="tv-pin">pinned</span>' : ''}</div>`
    + `<div class="tv-l3" style="border-color:${e.color}"><i style="background:${e.color}"></i><b>${esc(e.name)}</b><span id="tv-info"></span></div>`;
  if (head !== tvHtml) $('tv-body').innerHTML = tvHtml = head;
  const inf = $('tv-info');
  if (inf.innerHTML !== info) inf.innerHTML = info;
  $('tv-skip').style.bottom = `${H - controlsTop + 16 + el.offsetHeight}px`;
}

// Planned racing line, coloured by planned speed (blue = braking for a turn, orange = flat
// out). Redrawn whenever vision re-plans it.
const lineMat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.75 });
function drawRacingLine() {
  if (racingLine) {
    world.remove(racingLine);
    racingLine.geometry.dispose();
  }
  const src = rival ?? sim; // in a race, show the rival's line as a guide
  const plan = src.plan;
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
  racingLine = new THREE.LineLoop(lineGeo, lineMat);
  racingLine.visible = pilot !== 'pursuit';
  world.add(racingLine);
  lineVersion = src.planVersion;
}

// ---------- learning: what it has learned from its own laps ----------
// Cyan arrows along the line show the learned correction: the extra push (feedforward
// acceleration) it now gives at each point because that is where it drifted off before.
const LEARN_ARROW = 0.18; // m drawn per m/s^2 of correction
const learnArrows = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#4dd2ff', transparent: true, opacity: 0.8 }));
learnArrows.frustumCulled = false;
scene.add(learnArrows);
let learnDrawn = -1; // laps learned when last drawn
function updateLearn() {
  const L = sim.learn, panel = $('learn');
  panel.hidden = !L;
  learnArrows.visible = !!L && !replay;
  if (!L) { learnDrawn = -1; return; }
  const done = L.laps.length + 1000 * sim.planVersion;
  if (done === learnDrawn) return;
  learnDrawn = done;
  // arrows: every 6th sample of the line (1.5 m apart)
  const pts = sim.plan.pts, N = pts.length, stride = 6, pos = new Float32Array(Math.ceil(N / stride) * 6);
  for (let k = 0, j = 0; k < N; k += stride, j += 6) {
    const p = pts[k].p, c = L.c;
    pos.set(p, j);
    pos.set([p[0] + c[3 * k] * LEARN_ARROW, p[1] + c[3 * k + 1] * LEARN_ARROW, p[2] + c[3 * k + 2] * LEARN_ARROW], j + 3);
  }
  learnArrows.geometry.dispose();
  learnArrows.geometry = new THREE.BufferGeometry();
  learnArrows.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  // the panel
  const sum = learnSummary(sim), cv = $('learn-chart'), dpr = Math.min(devicePixelRatio, 2);
  const w = cv.clientWidth || 236, h = cv.clientHeight || 52;
  if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawLearnChart(ctx, w, h, sum);
  $('learn-conf').textContent = confLabel(sum.conf);
  $('learn-gain').textContent = sum.gain > 0.005 ? `−${sum.gain.toFixed(2)} s` : '–';
  $('learn-gain').className = sum.gain > 0.005 ? 'ahead' : '';
  $('learn-err').textContent = sum.rmsLast == null ? '–'
    : sum.laps.length > 1 ? `${Math.round(sum.rmsFirst * 100)} → ${Math.round(sum.rmsLast * 100)} cm` : `${Math.round(sum.rmsLast * 100)} cm`;
}

// ---------- drone model ----------
const drone = new THREE.Group();
const ledMat = [];
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
  // LEDs: red and green at the front, blue at the back. In a Group so the field's
  // repaint (which picks meshes by child index) leaves them alone.
  const leds = new THREE.Group();
  for (const [x, z, c] of [[0.29, 0.29, '#ff3b3b'], [-0.29, 0.29, '#3bff7a'], [0.29, -0.29, '#5ab0ff'], [-0.29, -0.29, '#5ab0ff']]) {
    const m = new THREE.MeshBasicMaterial({ color: c, transparent: true, opacity: 0.8 });
    ledMat.push(m);
    const led = new THREE.Mesh(new THREE.SphereGeometry(0.03, 8, 6), m);
    led.position.set(x, -0.02, z);
    leds.add(led);
  }
  drone.add(leds);
  drone.scale.setScalar(2.2); // a little larger than life so it reads on screen
}
scene.add(drone);

// the rival in a race: a translucent cyan copy of the drone, flown by the autopilot
const ghostDroneMat = new THREE.MeshBasicMaterial({ color: '#4dd2ff', transparent: true, opacity: 0.5, depthWrite: false });
const rivalMesh = drone.clone(true);
rivalMesh.traverse((o) => {
  if (o.isMesh) { o.material = ghostDroneMat; o.castShadow = false; }
});
rivalMesh.visible = false;
scene.add(rivalMesh);

// the ghost of the best lap: the same drone again, in translucent gold
const bestGhostMat = new THREE.MeshBasicMaterial({ color: '#ffc94d', transparent: true, opacity: 0.45, depthWrite: false });
const ghostMesh = drone.clone(true);
ghostMesh.traverse((o) => {
  if (o.isMesh) { o.material = bestGhostMat; o.castShadow = false; }
});
ghostMesh.visible = false;
scene.add(ghostMesh);

// ---------- the field: one solid drone per entrant, nose and arms in its colour ----------
const racerMeshes = new Map(); // roster id -> mesh, made on first use and reused
function racerMesh(r) {
  if (racerMeshes.has(r.id)) return racerMeshes.get(r.id);
  const mesh = drone.clone(true);
  const paint = new THREE.MeshStandardMaterial({ color: r.color, emissive: r.color, emissiveIntensity: 0.45 });
  // children: body, nose, then (arm, prop) per rotor
  mesh.children.forEach((c, i) => {
    if (c.isMesh && (i === 1 || (i >= 2 && i % 2 === 0))) c.material = paint;
    if (c.isMesh) c.castShadow = false;
  });
  mesh.visible = false;
  scene.add(mesh);
  racerMeshes.set(r.id, mesh);
  return mesh;
}
// name tag floating over a drone
function makeLabel(text, color) {
  const cv = document.createElement('canvas');
  cv.width = 256; cv.height = 64;
  const c = cv.getContext('2d');
  c.font = '600 30px ui-sans-serif, system-ui, sans-serif';
  const w = Math.min(248, c.measureText(text).width + 52);
  c.fillStyle = 'rgba(11, 18, 32, 0.72)';
  c.beginPath();
  if (c.roundRect) c.roundRect((256 - w) / 2, 8, w, 48, 24);
  else c.rect((256 - w) / 2, 8, w, 48);
  c.fill();
  c.fillStyle = color;
  c.beginPath();
  c.arc((256 - w) / 2 + 24, 32, 9, 0, Math.PI * 2);
  c.fill();
  c.fillStyle = '#e8edf5';
  c.textBaseline = 'middle';
  c.fillText(text, (256 - w) / 2 + 42, 33, w - 50);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  // fixed size on screen (no size attenuation), so tags stay readable far away and small up close
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true, sizeAttenuation: false }));
  sprite.scale.set(0.15, 0.0375, 1);
  sprite.renderOrder = 20;
  return sprite;
}
function heroName() {
  if (pilot === 'manual') return 'You';
  const twin = ROSTER.find((r) => r.opts.pilot === pilot && r.opts.nav === nav && !r.opts.tune && r.opts.optimise !== false);
  if (twin) return twin.name;
  if (pilot === 'racing' && nav === 'blind') return 'Map only';
  return PILOT_LABELS[pilot][0].toUpperCase() + PILOT_LABELS[pilot].slice(1) + (nav === 'truth' ? '' : ` · ${NAV_LABELS[nav]}`);
}
function clearField() {
  for (const e of field?.entries ?? []) {
    if (e.id !== 'hero') e.mesh.visible = false;
    scene.remove(e.label);
    e.label.material.map.dispose();
    e.label.material.dispose();
  }
  field = null;
}
function buildField() {
  clearField();
  setFollow('hero');
  lastLeader = null;
  $('board').hidden = !fieldOn;
  $('results').hidden = true;
  fieldSims = [];
  resultsClosed = false;
  if (!fieldOn) return;
  const entries = [{ id: 'hero', name: heroName(), color: HERO_COLOR, sim, mesh: drone }];
  // the hero takes pole, on the line; the rest line up abreast, alternately left and right
  rosterFor({ pilot, nav }).forEach((r, k) => {
    const s = makeSim({ seed, difficulty, wind: windMode, ...r.opts, slot: k + 1, launchAt: COUNTDOWN });
    entries.push({ id: r.id, name: r.name, color: r.color, sim: s, mesh: racerMesh(r) });
  });
  for (const e of entries) {
    e.label = makeLabel(e.name, e.color);
    scene.add(e.label);
    if (e.id !== 'hero') {
      e.mesh.visible = true;
      syncMesh(e.mesh, e.sim.drone);
    }
  }
  field = makeField(entries, { laps: RACE.laps, start: COUNTDOWN });
  fieldSims = entries.map((e) => e.sim);
  flagShown = false;
  $('results').hidden = true;
  boardHtml = '';
}
const viewEntry = () => field?.entries.find((e) => e.id === followId) ?? null;
const viewSim = () => viewEntry()?.sim ?? sim;
const viewMesh = () => viewEntry()?.mesh ?? drone;
// the cameras (chase and onboard) follow the chosen drone
function setFollow(id) {
  followId = id;
  viewMesh().add(camMount);
}
function updateField() {
  syncField(field, sim.course.gates.length);
  if (field.flag != null && !flagShown) {
    flagShown = true;
    const win = field.entries.find((e) => e.id === field.order[0]);
    toast(`Chequered flag: ${win.id === 'hero' && pilot === 'manual' ? 'you win!' : `${win.name} wins`}`);
    queueReplay(win.id, field.flag, `${win.name} takes the flag`, 4);
    audio.cue('lap');
  }
  if (field.over && $('results').hidden && !resultsClosed) showResults();
  for (const e of field.entries) {
    if (e.id !== 'hero') syncMesh(e.mesh, e.sim.drone);
    const p = e.sim.drone.pos;
    e.label.position.set(p[0], p[1] + 1.1, p[2]);
    e.label.scale.set(0.15 * labelZoom, 0.0375 * labelZoom, 1);
    e.label.visible = e.id !== followId || camMode === 'orbit' || (tvOn() && tvShot?.kind !== 'onboard' && tvShot?.kind !== 'chase');
  }
  if (frameNo % 6 === 0) updateBoard();
}
// the timing tower, F1 style: position, name, gap to the leader, best lap (fastest in purple)
let boardHtml = '';
const esc = (t) => t.replace(/[&<>"]/g, (ch) => `&#${ch.charCodeAt(0)};`);
function updateBoard() {
  const n = sim.course.gates.length;
  const rows = standings(field, n);
  const lead = rows[0];
  if (lead.gates && lead.id !== lastLeader) {
    if (lastLeader !== null) {
      toast(`${lead.name} takes the lead`);
      queueReplay(lead.id, sim.t, `${lead.name} takes the lead`, 2);
    }
    lastLeader = lead.id;
  }
  const color = Object.fromEntries(field.entries.map((e) => [e.id, e.color]));
  const lap = Math.min(field.laps, Math.max(1, ...rows.map((r) => r.lap)));
  // once the flag is out, the finishers are frozen in the order they took it
  if (field.flag != null) {
    const fin = new Map(results(field).filter((r) => r.finished).map((r) => [r.id, r]));
    const done = [...fin.values()], running = rows.filter((r) => !fin.has(r.id));
    rows.splice(0, rows.length, ...done.map((d) => ({ ...rows.find((r) => r.id === d.id), fin: d })), ...running);
    rows.forEach((r, i) => { r.pos = i + 1; });
  }
  const head = field.over ? 'Result' : field.flag != null ? 'Chequered flag' : `Race · lap ${lap}/${field.laps}`;
  const html = `<div class="board-head"><span>${head}</span><small>tap to follow</small></div>` + rows.map((r) => {
    const cls = [r.id === followId && 'cam', r.id === 'hero' && 'hero'].filter(Boolean).join(' ');
    const gap = r.fin ? finGap(r.fin) : field.flag != null ? (field.over ? 'DNF' : '…') : r.pos === 1 ? (r.gates ? 'Leader' : '') : gapText(r);
    return `<div class="row ${cls}" data-id="${r.id}"><i>${r.pos}</i><span class="dot" style="background:${color[r.id]}"></span>`
      + `<span class="nm">${r.fin ? '<span class="chq"></span>' : ''}${esc(r.name)}${r.crashed ? ' <em class="out">crash</em>' : ''}</span><b>${gap}</b>`
      + `<em class="${r.fastest ? 'fl' : ''}">${r.best == null ? '' : r.best.toFixed(2)}</em></div>`;
  }).join('');
  if (html !== boardHtml) $('board').innerHTML = boardHtml = html;
}
$('board').addEventListener('click', (e) => {
  const row = e.target.closest('.row');
  if (!row || !field) return;
  if (tvOn() && director) {
    // TV: keep cutting between cameras, but all on this drone (tap it again: director's choice)
    const id = director.pin === row.dataset.id ? null : row.dataset.id;
    pinSubject(director, id);
    toast(id ? `TV: following ${field.entries.find((e) => e.id === id).name}` : "TV: director's choice");
    return;
  }
  setFollow(row.dataset.id);
  boardHtml = '';
  updateBoard();
  const who = viewEntry();
  if (who) toast(who.id === 'hero' ? 'Following you' : `Following ${who.name}`);
});
// A finisher's line in the tower and the results: winner's race time, else the gap.
function finGap(f) {
  if (f.pos === 1) return fmtRace(f.time);
  return f.down ? `+${f.down} lap${f.down > 1 ? 's' : ''}` : `+${f.gap.toFixed(2)}`;
}
const fmtRace = (t) => `${Math.floor(t / 60)}:${(t % 60).toFixed(2).padStart(5, '0')}`;
// Drone-to-drone contact: sparks (a crash already throws its own debris) and a word.
let lastContactToast = -Infinity;
function onContact(h) {
  if (h.crash || h.closing < 1.5) return;
  burst(h.pos, [0, 0, 0]);
  if (sim.t - lastContactToast > 3) {
    lastContactToast = sim.t;
    toast('Contact!');
  }
}
// The results card, once everyone has taken the flag (or the race has timed out).
let resultsClosed = false;
function showResults() {
  const color = Object.fromEntries(field.entries.map((e) => [e.id, e.color]));
  const rows = results(field);
  const contacts = field.entries.reduce((a, e) => a + e.bumps, 0) / 2;
  const crashes = field.entries.reduce((a, e) => a + e.crashes, 0);
  const win = rows[0];
  const title = '<span class="chq"></span>' + (!win.finished ? 'Race over' : win.id === 'hero' && pilot === 'manual' ? 'You win!' : `${esc(win.name)} wins`);
  $('results-body').innerHTML = `<div class="res-title">${title}<small>${field.laps} laps · ${difficulty} course ${seed}${windMode === 'off' ? '' : ` · ${windMode}`}</small></div>`
    + '<div class="res-row res-hd"><i></i><span></span><span>Drone</span><b>Time</b><em>Best lap</em></div>'
    + rows.map((r) => `<div class="res-row${r.id === 'hero' ? ' hero' : ''}"><i>${r.finished ? r.pos : '–'}</i><span class="dot" style="background:${color[r.id]}"></span>`
      + `<span class="nm">${esc(field.entries.find((e) => e.id === r.id).name)}${r.crashes ? ` <em class="out">${r.crashes} crash${r.crashes > 1 ? 'es' : ''}</em>` : ''}</span>`
      + `<b>${r.finished ? finGap(r) : 'DNF'}</b><em>${r.best == null ? '–' : r.best.toFixed(2)}</em></div>`).join('')
    + `<div class="res-foot">${Math.round(contacts)} contact${Math.round(contacts) === 1 ? '' : 's'} · ${crashes} crash${crashes === 1 ? '' : 'es'}</div>`;
  $('results').hidden = false;
}
$('res-again').onclick = () => { resultsClosed = false; buildWorld(); };
$('res-next').onclick = () => { resultsClosed = false; $('b-course').click(); };
$('res-close').onclick = () => { resultsClosed = true; $('results').hidden = true; };

// Fly the ghost to where the best lap was at this point in the current lap. Hidden when
// it sits right on top of the drone (a repeatable autopilot lap matches itself).
function updateGhost() {
  const s = sim.state;
  const pose = ghostOn && s.lapStart != null ? ghostPose(ghostRec.best, sim.t - s.lapStart) : null;
  ghostMesh.visible = !!pose && v3.len(v3.sub(pose.pos, sim.drone.pos)) > 0.8;
  if (ghostMesh.visible) syncMesh(ghostMesh, pose);
}

// Visual-inertial nav: where the drone thinks it is (a magenta wire ball, radius two sigma,
// joined to the real drone by a line) and where the IMU alone would put it (red, joined to
// the drone by a dashed line). Chase view only (layer 1), so they never fill the onboard view.
const estMat = new THREE.MeshBasicMaterial({ color: '#ff4fd8', wireframe: true, transparent: true, opacity: 0.8, depthTest: false });
const estMesh = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 8), estMat);
const drMesh = new THREE.Mesh(new THREE.SphereGeometry(0.35, 12, 8), new THREE.MeshBasicMaterial({ color: '#ff5c5c', transparent: true, opacity: 0.9, depthTest: false }));
const estLinePos = new Float32Array(12);
const estLineGeo = new THREE.BufferGeometry();
estLineGeo.setAttribute('position', new THREE.BufferAttribute(estLinePos, 3));
const estLine = new THREE.LineSegments(estLineGeo, new THREE.LineBasicMaterial({ color: '#ff8fe6', transparent: true, opacity: 0.8, depthTest: false }));
estLine.frustumCulled = false;
for (const o of [estMesh, drMesh, estLine]) {
  o.layers.set(1);
  o.renderOrder = 11;
  o.visible = false;
  scene.add(o);
}
chaseCam.layers.enable(1);
function updateEstimate() {
  const vs = viewSim(), E = vs.est, on = !!E && !vs.crash;
  estMesh.visible = drMesh.visible = estLine.visible = on;
  if (!on) return;
  const n = navError(vs), p = vs.drone.pos, x = E.x;
  estMesh.position.set(x[0], x[1], x[2]);
  estMesh.scale.setScalar(Math.max(0.3, 2 * n.sigma));
  drMesh.position.set(...E.dr.p);
  estLinePos.set(p, 0);
  estLinePos.set([x[0], x[1], x[2]], 3);
  estLinePos.set(p, 6);
  estLinePos.set(E.dr.p, 9);
  estLineGeo.attributes.position.needsUpdate = true;
}

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

// ---------- crash debris: a burst of sparks and bits that fall and fade ----------
const DEBRIS = 48, DEBRIS_LIFE = 1.4;
const debrisPos = new Float32Array(DEBRIS * 3), debrisVel = new Float32Array(DEBRIS * 3);
const debrisGeo = new THREE.BufferGeometry();
debrisGeo.setAttribute('position', new THREE.BufferAttribute(debrisPos, 3));
const debrisMat = new THREE.PointsMaterial({ color: '#ffb057', size: 0.14, transparent: true, opacity: 0, depthWrite: false });
const debris = new THREE.Points(debrisGeo, debrisMat);
debris.frustumCulled = false;
debris.visible = false;
scene.add(debris);
let debrisAge = Infinity;
function burst(p, vel) {
  for (let i = 0; i < DEBRIS; i++) {
    const a = Math.random() * Math.PI * 2, up = Math.random() * 2 - 0.4, sp = 2 + Math.random() * 6;
    debrisPos.set(p, i * 3);
    debrisVel.set([vel[0] * 0.5 + Math.cos(a) * sp, vel[1] * 0.5 + up * sp, vel[2] * 0.5 + Math.sin(a) * sp], i * 3);
  }
  debrisAge = 0;
  debris.visible = true;
  debrisGeo.attributes.position.needsUpdate = true;
}
function updateDebris(dt) {
  if (!debris.visible) return;
  debrisAge += dt;
  if (debrisAge > DEBRIS_LIFE) { debris.visible = false; return; }
  for (let i = 0; i < DEBRIS * 3; i += 3) {
    debrisVel[i + 1] -= 9.81 * dt;
    for (let k = 0; k < 3; k++) debrisPos[i + k] += debrisVel[i + k] * dt;
    if (debrisPos[i + 1] < 0.05) {
      debrisPos[i + 1] = 0.05;
      debrisVel[i + 1] *= -0.3;
      debrisVel[i] *= 0.6; debrisVel[i + 2] *= 0.6;
    }
  }
  debrisMat.opacity = 1 - debrisAge / DEBRIS_LIFE;
  debrisGeo.attributes.position.needsUpdate = true;
}

// ---------- prop wash: air blown down through the props, kicking up dust near the ground ----------
const wash = makeWash();
const washPos = new THREE.BufferAttribute(wash.pos, 3);
const washCol = new THREE.BufferAttribute(new Float32Array(wash.n * 4), 4);
const washGeo = new THREE.BufferGeometry();
washGeo.setAttribute('position', washPos);
washGeo.setAttribute('color', washCol);
const washMat = new THREE.PointsMaterial({ size: 0.45, map: radialTex, vertexColors: true, transparent: true, depthWrite: false });
const washPts = new THREE.Points(washGeo, washMat);
washPts.frustumCulled = false;
scene.add(washPts);
const WASH_TINT = { day: { air: [1, 1, 1, 0.05], dust: [0.66, 0.6, 0.48, 0.55] }, night: { air: [0.45, 0.6, 1, 0.22], dust: [0.75, 0.7, 0.6, 0.35] } };
function updateWash(dtSim) {
  const vs = viewSim();
  if (dtSim > 0) {
    emitWash(wash, vs.drone, dtSim, { crashed: !!vs.crash, scale: 2.2, wind: vs.wind?.now });
    stepWash(wash, dtSim);
  }
  const tint = WASH_TINT[night ? 'night' : 'day'], c = washCol.array;
  for (let i = 0; i < wash.n; i++) {
    const t = wash.age[i] / WASH.life, src = wash.dust[i] ? tint.dust : tint.air;
    const a = t >= 1 ? 0 : src[3] * (1 - t) * Math.min(1, t * 8);
    c[i * 4] = src[0]; c[i * 4 + 1] = src[1]; c[i * 4 + 2] = src[2]; c[i * 4 + 3] = a;
  }
  washPos.needsUpdate = washCol.needsUpdate = true;
}
function updateHeadlight() {
  if (!night) return;
  const d = viewSim().drone, f = [Math.sin(d.yaw), 0, Math.cos(d.yaw)];
  headlight.position.set(d.pos[0] + f[0] * 0.6, d.pos[1] + 0.1, d.pos[2] + f[2] * 0.6);
  headlight.target.position.set(d.pos[0] + f[0] * 14, d.pos[1] - 3, d.pos[2] + f[2] * 14);
}

// ---------- wind streaks: short dashes drifting with the wind around the drone ----------
const STREAKS = 160, STREAK_BOX = 36;
const streakPos = new Float32Array(STREAKS * 6), streakSeed = [];
const streakGeo = new THREE.BufferGeometry();
streakGeo.setAttribute('position', new THREE.BufferAttribute(streakPos, 3));
const streaks = new THREE.LineSegments(streakGeo, new THREE.LineBasicMaterial({ color: '#cfe3ff', transparent: true, opacity: 0.35 }));
streaks.frustumCulled = false;
scene.add(streaks);
for (let i = 0; i < STREAKS; i++) streakSeed.push([Math.random(), Math.random(), Math.random()].map((r) => (r - 0.5) * STREAK_BOX));
function updateStreaks(dtSim) {
  streaks.visible = !!sim.wind;
  if (!sim.wind) return;
  const w = sim.wind.now, c = viewSim().drone.pos, half = STREAK_BOX / 2;
  const len = 0.12; // s of travel each dash shows
  for (let i = 0; i < STREAKS; i++) {
    const p = streakSeed[i];
    for (let k = 0; k < 3; k++) {
      p[k] += w[k] * dtSim;
      // keep each dash in a box that travels with the drone
      const rel = p[k] - c[k];
      if (rel > half) p[k] -= STREAK_BOX; else if (rel < -half) p[k] += STREAK_BOX;
    }
    if (p[1] < 0.2) p[1] += STREAK_BOX / 2;
    streakPos.set([p[0], p[1], p[2], p[0] - w[0] * len, p[1] - w[1] * len, p[2] - w[2] * len], i * 6);
  }
  streakGeo.attributes.position.needsUpdate = true;
}

// ---------- HUD ----------
let lastEvents = 0;
// camera blackout: the drone whose camera was cut, and how far through its events we are
let darkWatch = null;
let darkBtnShown = false;
function updateBlackout() {
  const vs = viewSim(), btn = $('b-dark');
  if (darkBtnShown !== !!vs.vision) {
    darkBtnShown = !!vs.vision;
    btn.hidden = !darkBtnShown;
  }
  btn.classList.toggle('on', cameraDark(vs));
  if (!darkWatch) return;
  const ds = darkWatch.sim;
  for (; darkWatch.i < ds.events.length; darkWatch.i++) {
    const e = ds.events[darkWatch.i];
    if (e.type === 'blackout') toast('Camera dark', true);
    if (e.type === 'hold') toast(`Can't see gate ${e.gate + 1}: stopping short`, true);
    if (e.type === 'resume') toast(`Found gate ${e.gate + 1}: go!`);
  }
}
function fmt(t) { return t == null ? '–' : `${t.toFixed(2)} s`; }
function updateHud(force) {
  const s = sim.state;
  $('s-lap').textContent = s.lap || '–';
  $('s-last').textContent = fmt(s.laps.at(-1));
  $('s-best').textContent = fmt(s.laps.length ? Math.min(...s.laps) : null);
  $('s-speed').textContent = `${(v3.len(sim.drone.vel) * 3.6).toFixed(0)} km/h`;
  const extra = [s.misses && `${s.misses} missed`, s.crashes && `${s.crashes} crashed`].filter(Boolean);
  $('s-gates').textContent = [s.gatesPassed, ...extra].join(' · ');
  $('s-plan').textContent = pilot === 'racing' ? fmt(sim.plan.lapTime) : rival ? fmt(rival.plan.lapTime) : '–';
  // race: gap to the rival at the last gate you've both passed (+ behind, − ahead)
  $('r-rival').hidden = $('s-rival').hidden = !race;
  if (race) {
    const { gap, you, rival: theirs } = raceGap(race), el = $('s-rival');
    if (gap != null) {
      el.textContent = `${gap >= 0 ? '+' : '−'}${Math.abs(gap).toFixed(2)} s`;
      el.className = gap > 0 ? 'behind' : 'ahead';
    } else {
      // you haven't made the first gate yet: count how many the rival is up
      const up = theirs - you;
      el.textContent = up > 0 ? `+${up} gate${up === 1 ? '' : 's'}` : '–';
      el.className = up > 0 ? 'behind' : '';
    }
  }
  // ghost: gap to the best lap at the last gate passed (+ slower, − faster)
  $('r-ghost').hidden = $('s-ghost').hidden = !ghostRec.best;
  if (ghostRec.best) {
    const g = ghostRec.gap, el = $('s-ghost');
    el.textContent = !g ? fmt(ghostRec.best.time) : Math.abs(g.delta) < 0.005 ? '0.00 s' : `${g.delta > 0 ? '+' : '−'}${Math.abs(g.delta).toFixed(2)} s`;
    el.className = !g ? '' : g.delta > 0.005 ? 'behind' : g.delta < -0.005 ? 'ahead' : '';
  }
  $('s-course').textContent = `#${seed} ${difficulty}`;
  // vision: how far out the map is for the next gate, and how far out the estimate is now
  $('r-vis').hidden = $('s-vis').hidden = !sim.belief;
  if (sim.belief) {
    const g = sim.course.gates[s.target], b = sim.belief.gates[s.target];
    const mapErr = v3.len(v3.sub(b.mapPos, g.pos)), estErr = v3.len(v3.sub(b.pos, g.pos));
    $('s-vis').textContent = sim.vision ? `${mapErr.toFixed(1)} → ${estErr.toFixed(2)} m` : `${mapErr.toFixed(1)} m out`;
  }
  // visual-inertial nav: how far out the drone's own position estimate is, and how far
  // out the IMU alone would be by now (since the start of the lap)
  const vs = viewSim(), ne = vs.crash ? null : navError(vs);
  $('r-nav').hidden = $('s-nav').hidden = !vs.est;
  if (vs.est) $('s-nav').textContent = ne ? `${ne.err.toFixed(2)} m · IMU ${ne.dr < 10 ? ne.dr.toFixed(1) : ne.dr.toFixed(0)} m` : '–';
  // wind: what is really blowing, and what the drone's observer reckons
  $('r-wind').hidden = $('s-wind').hidden = !sim.wind;
  if (sim.wind) {
    const w = sim.wind.now, est = v3.scale(sim.dist, 1 / DRONE.drag);
    $('s-wind').textContent = `${Math.hypot(w[0], w[2]).toFixed(0)} m/s · est ${Math.hypot(est[0], est[2]).toFixed(0)}`;
  }
  if (force) return;
  updateBlackout();
  for (; lastEvents < sim.events.length; lastEvents++) {
    const e = sim.events[lastEvents];
    if (e.type === 'lap' && !sim.learn) toast(pilot === 'manual' ? lapToast(e.time) : `Lap ${fmt(e.time)}`);
    if (e.type === 'learn') toast(learnNote(e, s.laps.at(-1), s.laps[0]));
    if (e.type === 'splits') toast(`Split-S · ${e.inverted.toFixed(2)} s upside down`);
    if (e.type === 'gate') onGate(e.gate);
    if (e.type === 'gate' || e.type === 'lap' || e.type === 'crash') audio.cue(e.type);
    if (e.type === 'miss') toast(`Missed gate ${e.gate + 1}`, true);
    if (e.type === 'crash') {
      toast(e.what === 'pillar' ? 'Crashed into a pillar' : e.what === 'drone' ? 'Mid-air collision!' : `Crashed into gate ${e.gate + 1}`, true);
      burst(e.pos, sim.drone.vel);
    }
    if (e.type === 'respawn') trailCount = 0; // don't draw a line to the respawn point
  }
}
function onGate(id) {
  const n = sim.course.gates.length;
  setGateMat(id, gateMatDone);
  setGateMat((id + 1) % n, gateMatNext);
  if (id === n - 1) for (let i = 1; i < n - 1; i++) setGateMat(i, gateMatIdle);
  if (id === 0) setGateMat(n - 1, gateMatIdle);
}
// Your lap in a race: against the rival's best, and your personal best on this course.
function lapToast(t) {
  const rb = rival?.state.laps.length ? Math.min(...rival.state.laps) : null;
  const vs = rb == null ? '' : ` · rival ${rb.toFixed(2)}`;
  if (pb == null || t < pb) {
    const first = pb == null;
    pb = t;
    savePb(t);
    return first ? `Lap ${fmt(t)}${vs}` : `New best ${fmt(t)}${vs}`;
  }
  return `Lap ${fmt(t)}${vs}`;
}
// personal bests per course, kept in this browser only
const pbKey = () => `drone-sim:pb:${seed}:${difficulty}:${windMode}`;
function loadPb() {
  try {
    const v = parseFloat(localStorage.getItem(pbKey()));
    return Number.isFinite(v) ? v : null;
  } catch { return null; }
}
function savePb(t) {
  try { localStorage.setItem(pbKey(), String(t)); } catch { /* storage unavailable */ }
}
// your best lap's ghost, per course, in this browser only
const ghostKey = () => `drone-sim:ghost:${seed}:${difficulty}:${windMode}`;
function loadGhost() {
  try { return decodeGhost(localStorage.getItem(ghostKey()) ?? ''); } catch { return null; }
}
function saveGhost(g) {
  try { localStorage.setItem(ghostKey(), encodeGhost(g)); } catch { /* storage unavailable or full */ }
}
let toastTimer;
function toast(msg, bad = false) {
  const el = $('toast');
  el.textContent = msg;
  el.classList.toggle('bad', bad);
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 1300);
}

// A short note on what the nav mode shows, faded out after a while.
const NAV_NOTES = {
  vision: '<b>Vision.</b> The drone\'s map has every gate up to 2.6 m out. Its camera finds the real gates (corner boxes in the onboard view) and it re-plans through its estimates (cyan). Press <b>Blackout</b> (B) to cut its camera.',
  vio: '<b>Vision + IMU.</b> Now the drone doesn\'t know where <i>it</i> is either: just a noisy, biased accelerometer and the camera. A Kalman filter estimates its position (magenta ball, 2σ) and every gate together. The red ball is the IMU alone: it drifts metres off every lap. Press <b>Blackout</b> (B) to cut its camera and watch the ball grow.',
  blind: '<b>Map only.</b> The drone trusts a map with every gate up to 2.6 m out (cyan), so it misses gates. As a ghost run, it flies straight through any frame it meets.',
};
const EYES_NOTE = '<b>Eyes: pixels.</b> It finds the gates in its own 320×180 camera picture (inset): green, found; dashed, where its map says.';
const coarse = matchMedia('(pointer: coarse)').matches;
const MANUAL_CONTROLS = coarse
  ? '<b>You fly.</b> Left stick: speed and turn. Right stick: climb and drift (height is held for you).'
  : '<b>You fly.</b> <b>W</b>/<b>↑</b> speed, <b>A D</b>/<b>← →</b> turn, <b>Space</b>/<b>Shift</b> climb (height is held for you). Gamepads work too.';
const MANUAL_NOTE = `${MANUAL_CONTROLS} Beat the cyan ghost: it's the autopilot. The gold ghost is your best lap.`;
function fieldNote() {
  const names = field.entries.filter((e) => e.id !== 'hero').map((e) => `<b style="color:${e.color}">${esc(e.name)}</b>`);
  const head = pilot === 'manual' ? `${MANUAL_CONTROLS} ` : '<b>Field.</b> ';
  return `${head}You're racing ${names.slice(0, -1).join(', ')} and ${names.at(-1)}: the same drone with different autopilots. Tap a name in the tower to follow it.`;
}
let noteTimer;
function showNavNote() {
  const el = $('nav-note');
  let html = field ? fieldNote() : pilot === 'manual' ? MANUAL_NOTE : NAV_NOTES[nav];
  if (html && !field && pilot !== 'manual' && eyes === 'pixels' && hasCamera()) html += ' ' + EYES_NOTE;
  el.innerHTML = html ?? '';
  el.hidden = !html;
  el.classList.remove('fade');
  clearTimeout(noteTimer);
  noteTimer = setTimeout(() => el.classList.add('fade'), 12000);
}

// ---------- controls ----------
let timeScale = 1;
$('b-cam').onclick = () => {
  camMode = CAM_MODES[(CAM_MODES.indexOf(camMode) + 1) % CAM_MODES.length];
  syncButtons();
  writeHash();
  if (!tvOn()) {
    stopReplay();
    tvShot = null;
    if (field && director?.pin) setFollow(director.pin);
  } else {
    if (director) { director.shot = null; tvSeen = new Map(); }
    toast('TV: cameras and replays');
  }
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
$('b-nav').onclick = () => {
  nav = NAV_MODES[(NAV_MODES.indexOf(nav) + 1) % NAV_MODES.length];
  syncButtons();
  buildWorld();
};
$('b-eyes').onclick = () => {
  eyes = EYES[(EYES.indexOf(eyes) + 1) % EYES.length];
  syncButtons();
  buildWorld();
};
$('b-dark').onclick = () => {
  const vs = viewSim();
  if (!cameraBlackout(vs)) { toast('No camera to cut: try Nav: vision'); return; }
  darkWatch = { sim: vs, i: vs.events.length - 1 };
  updateBlackout();
};
$('b-tm').onclick = () => {
  const el = $('telemetry');
  el.hidden = !el.hidden;
  $('b-tm').classList.toggle('on', !el.hidden);
  if (!el.hidden) drawTelemetry(telemetry, sim);
};
$('b-ghost').onclick = () => {
  ghostOn = !ghostOn;
  $('b-ghost').classList.toggle('on', ghostOn);
  toast(ghostOn ? 'Ghost on' : 'Ghost off');
};
$('b-learn').onclick = () => {
  learnOn = !learnOn;
  syncButtons();
  buildWorld();
  toast(learnOn ? 'Learning from every lap' : 'Learning off');
};
$('b-wind').onclick = () => {
  windMode = WIND_MODES[(WIND_MODES.indexOf(windMode) + 1) % WIND_MODES.length];
  syncButtons();
  buildWorld();
};
$('b-night').onclick = () => {
  night = !night;
  applyLook();
  writeHash();
  toast(night ? 'Night' : 'Day');
};
const audio = makeAudio();
if (!audio.supported) $('b-sound').hidden = true;
$('b-sound').onclick = () => {
  const on = audio.setOn(!audio.on);
  $('b-sound').classList.toggle('on', on);
};
$('b-share').onclick = () => {
  const url = location.href;
  if (navigator.clipboard?.writeText) {
    navigator.clipboard.writeText(url).then(() => toast('Link copied'), () => toast(location.hash));
  } else toast(location.hash);
};
const NAV_LABELS = { truth: 'ground truth', vision: 'vision', vio: 'vision + IMU', blind: 'map only' };
function syncButtons() {
  $('b-cam').textContent = `Camera: ${CAM_LABELS[camMode]}`;
  $('b-cam').classList.toggle('on', camMode === 'tv');
  $('b-diff').textContent = `Course: ${difficulty}`;
  $('b-nav').textContent = `Nav: ${NAV_LABELS[nav]}`;
  $('b-eyes').hidden = !hasCamera();
  $('b-eyes').textContent = `Eyes: ${eyes === 'pixels' ? 'pixels' : 'corner model'}`;
  $('b-eyes').classList.toggle('on', eyes === 'pixels');
  $('b-wind').textContent = `Wind: ${windMode}`;
  $('b-learn').hidden = !canLearn();
  $('b-learn').textContent = `Learn: ${learnOn ? 'on' : 'off'}`;
  $('b-learn').classList.toggle('on', learnOn);
  $('b-pilot').textContent = `Pilot: ${PILOT_LABELS[pilot]}`;
  $('b-field').textContent = fieldOn ? `Field: ${1 + rosterFor({ pilot, nav }).length} drones` : 'Field: solo';
  $('b-field').classList.toggle('on', fieldOn);
}
const PILOT_LABELS = { racing: 'racing line', pursuit: 'pursuit', manual: 'you' };
$('b-field').onclick = () => {
  fieldOn = !fieldOn;
  syncButtons();
  buildWorld();
};

// ---------- shareable course in the URL: #seed=12&d=hard&nav=vision ----------
function readHash() {
  const p = new URLSearchParams(location.hash.slice(1));
  const n = parseInt(p.get('seed'), 10);
  seed = Number.isInteger(n) && n > 0 && n < 1e9 ? n : 1;
  difficulty = DIFFICULTIES.includes(p.get('d')) ? p.get('d') : 'normal';
  nav = NAV_MODES.includes(p.get('nav')) ? p.get('nav') : 'truth';
  eyes = p.get('eyes') === 'px' ? 'pixels' : 'model';
  windMode = WIND_MODES.includes(p.get('wind')) ? p.get('wind') : 'off';
  pilot = PILOTS.includes(p.get('pilot')) ? p.get('pilot') : 'racing';
  fieldOn = p.get('field') === '1';
  night = p.get('night') === '1';
  learnOn = p.get('learn') === '1';
  if (p.get('cam') === 'tv') camMode = 'tv';
  else if (camMode === 'tv') { camMode = 'chase'; stopReplay(); }
  syncButtons();
}
function writeHash() {
  const h = `#seed=${seed}&d=${difficulty}${nav === 'truth' ? '' : `&nav=${nav}`}${eyes === 'pixels' && hasCamera() ? '&eyes=px' : ''}${windMode === 'off' ? '' : `&wind=${windMode}`}${pilot === 'racing' ? '' : `&pilot=${pilot}`}${fieldOn ? '&field=1' : ''}${night ? '&night=1' : ''}${camMode === 'tv' ? '&cam=tv' : ''}${learnOn && canLearn() ? '&learn=1' : ''}`;
  if (location.hash !== h) history.replaceState(null, '', h);
}
addEventListener('hashchange', () => {
  const key = () => `${seed}/${difficulty}/${nav}/${eyes}/${windMode}/${pilot}/${fieldOn}/${learnOn}`;
  const before = key();
  const wasNight = night;
  readHash();
  if (key() !== before) buildWorld();
  if (night !== wasNight) applyLook();
});
$('b-pilot').onclick = () => {
  pilot = PILOTS[(PILOTS.indexOf(pilot) + 1) % PILOTS.length];
  syncButtons();
  buildWorld();
};

// ---------- manual flight inputs: keyboard, gamepad and touch sticks ----------
const FLIGHT_KEYS = new Set(['w', 'a', 's', 'd', 'q', 'e', ' ', 'shift', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright']);
const keys = new Set();
const kb = { x: 0, y: 0, z: 0 }; // keyboard sticks, eased so a tap isn't a jolt
const keyName = (e) => e.key.toLowerCase();
addEventListener('keyup', (e) => {
  if (!FLIGHT_KEYS.has(keyName(e))) return;
  keys.delete(keyName(e));
  if (pilot === 'manual') e.preventDefault(); // stops Space "clicking" a focused button
});
addEventListener('blur', () => keys.clear());

function makePad(el) {
  const st = { x: 0, y: 0, id: null }, knob = el.querySelector('.knob');
  const move = (e) => {
    const r = el.getBoundingClientRect(), R = r.width / 2;
    let dx = (e.clientX - r.left - R) / R, dy = (e.clientY - r.top - R) / R;
    const l = Math.hypot(dx, dy);
    if (l > 1) { dx /= l; dy /= l; }
    st.x = dx; st.y = dy;
    knob.style.transform = `translate(${dx * R * 0.6}px, ${dy * R * 0.6}px)`;
  };
  el.addEventListener('pointerdown', (e) => {
    st.id = e.pointerId;
    el.setPointerCapture(e.pointerId);
    move(e);
    e.preventDefault();
  });
  el.addEventListener('pointermove', (e) => { if (e.pointerId === st.id) move(e); });
  const end = (e) => {
    if (e.pointerId !== st.id) return;
    st.id = null; st.x = st.y = 0;
    knob.style.transform = '';
  };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
  return st;
}
const padL = makePad($('pad-l')), padR = makePad($('pad-r'));

function readSticks(dt) {
  const on = (a, b) => (keys.has(a) || keys.has(b) ? 1 : 0);
  const want = {
    x: on('d', 'arrowright') - on('a', 'arrowleft'),
    y: on('w', 'arrowup') - on('s', 'arrowdown'),
    z: on(' ', 'e') - on('shift', 'q'),
  };
  const r = Math.min(1, dt * 6);
  for (const a in want) kb[a] += (want[a] - kb[a]) * r;
  // left stick (pad or gamepad): speed and turn; right stick: climb and drift
  const st = { x: kb.x + padL.x, y: kb.y - padL.y, z: kb.z - padR.y, s: padR.x };
  const gp = [...(navigator.getGamepads?.() ?? [])].find((g) => g && g.connected);
  if (gp) {
    const ax = (i) => (Math.abs(gp.axes[i] ?? 0) > 0.12 ? gp.axes[i] : 0);
    st.x += ax(0); st.y -= ax(1); st.s += ax(2); st.z -= ax(3);
  }
  for (const a in st) st[a] = Math.max(-1, Math.min(1, st[a]));
  return st;
}

// intro overlay: a pitch, quick starts and what the latest overnight build added
const landing = makeLanding({ onStart: (h) => { if (location.hash !== h) location.hash = h; } });

addEventListener('keydown', (e) => {
  if (landing.isOpen()) {
    if (e.key === 'Escape') landing.close();
    return; // no shortcuts or flying behind the intro
  }
  if (e.key === '?' || e.key === 'i') { landing.show(); return; }
  if (pilot === 'manual' && FLIGHT_KEYS.has(keyName(e))) {
    keys.add(keyName(e));
    e.preventDefault();
    return;
  }
  if (e.key === 'c') $('b-cam').click();
  if (e.key === 'n') $('b-course').click();
  if (e.key === 'p') $('b-pilot').click();
  if (e.key === 'd') $('b-diff').click();
  if (e.key === 'v') $('b-nav').click();
  if (e.key === 'b' && !$('b-dark').hidden) $('b-dark').click();
  if (e.key === 'e' && !$('b-eyes').hidden) $('b-eyes').click();
  if (e.key === 'w') $('b-wind').click();
  if (e.key === 'k' && !$('b-learn').hidden) $('b-learn').click();
  if (e.key === 't') $('b-tm').click();
  if (e.key === 'g') $('b-ghost').click();
  if (e.key === 'f') $('b-field').click();
  if (e.key === 'Escape' && replay) stopReplay();
  else if (e.key === 'Escape' && !$('results').hidden) $('res-close').click();
  if (e.key === 'l') $('b-night').click();
  if (e.key === 'm') $('b-sound').click();
});

// ---------- onboard overlay: what the camera detected this frame ----------
const overlay = $('overlay'), octx = overlay.getContext('2d');
let overlayDpr = 1, overlayDrawn = false;
function sizeOverlay() {
  overlayDpr = Math.min(devicePixelRatio, 2);
  overlay.width = Math.round(innerWidth * overlayDpr);
  overlay.height = Math.round(innerHeight * overlayDpr);
  overlayDrawn = true; // resizing clears the canvas; make sure it is redrawn
}
sizeOverlay();
const ovV = new THREE.Vector3();
// Screen position (CSS px) of a world point in the onboard view drawn in rect, or null.
function toScreen(p, rect) {
  ovV.set(p[0], p[1], p[2]).applyMatrix4(fpvCam.matrixWorldInverse);
  if (ovV.z > -0.2) return null; // behind the camera
  ovV.applyMatrix4(fpvCam.projectionMatrix);
  return [rect.x + ((ovV.x + 1) / 2) * rect.w, rect.top + ((1 - ovV.y) / 2) * rect.h];
}
// Vision step 3: the drone's own camera picture (what the detector really works on), with
// the quads it found (green: fused, red: thrown out, amber: no gate it could put to it)
// and, dashed, where its map said each gate would be.
const eyeCanvas = document.createElement('canvas'), eyeCtx = eyeCanvas.getContext('2d');
let eyeT = -1;
function drawEye(r, frame) {
  const img = frame.img;
  if (!img) return;
  if (eyeCanvas.width !== img.W) { eyeCanvas.width = img.W; eyeCanvas.height = img.H; }
  if (frame.t !== eyeT) {
    eyeT = frame.t;
    eyeCtx.putImageData(new ImageData(img.rgb, img.W, img.H), 0, 0);
  }
  octx.save();
  octx.beginPath();
  octx.rect(r.x, r.top, r.w, r.h);
  octx.clip();
  octx.imageSmoothingEnabled = false;
  octx.drawImage(eyeCanvas, r.x, r.top, r.w, r.h);
  octx.imageSmoothingEnabled = true;
  const sx = r.w / img.W, sy = r.h / img.H;
  const poly = (c) => {
    octx.beginPath();
    c.forEach(([u, v], i) => (i ? octx.lineTo(r.x + u * sx, r.top + v * sy) : octx.moveTo(r.x + u * sx, r.top + v * sy)));
    octx.closePath();
  };
  octx.lineWidth = 1;
  octx.setLineDash([3, 3]);
  octx.strokeStyle = 'rgba(77, 210, 255, .8)';
  for (const p of frame.pred ?? []) { poly(p.corners); octx.stroke(); }
  octx.setLineDash([]);
  octx.lineWidth = 1.5;
  octx.font = '700 11px ui-sans-serif, system-ui, sans-serif';
  for (const q of frame.quads ?? []) {
    const col = q.ok ? '#3ddc97' : q.gate !== undefined ? '#ff5c5c' : '#ffc04d';
    octx.strokeStyle = octx.fillStyle = col;
    poly(q.corners);
    octx.stroke();
    if (q.gate !== undefined) {
      const top = q.corners.reduce((a, b) => (b[1] < a[1] ? b : a));
      octx.fillText(String(q.gate + 1), r.x + top[0] * sx + 3, r.top + top[1] * sy - 3);
    }
  }
  octx.strokeStyle = 'rgba(255, 255, 255, .25)';
  octx.lineWidth = 1;
  octx.strokeRect(r.x + 0.5, r.top + 0.5, r.w - 1, r.h - 1);
  octx.restore();
}
// rect: the onboard view the detections are drawn over (null: none); eyeRect: where the
// drone's own picture goes, when it has one.
function drawOverlay(rect, label, eyeRect = null) {
  const vs = viewSim(), frame = vs.vision?.frame;
  if (!frame && !overlayDrawn) return;
  octx.setTransform(1, 0, 0, 1, 0, 0);
  octx.clearRect(0, 0, overlay.width, overlay.height);
  overlayDrawn = !!frame;
  if (!frame || (!rect && !eyeRect)) { overlayDrawn = false; return; }
  octx.setTransform(overlayDpr, 0, 0, overlayDpr, 0, 0);
  let found = 0;
  const fresh = vs.t - frame.t < 0.2;
  if (eyeRect && fresh && !frame.dark) {
    drawEye(eyeRect, frame);
    if (!rect) found = frame.dets.filter((d) => d.ok).length;
  }
  if (!rect) rect = eyeRect;
  octx.save();
  octx.beginPath();
  octx.rect(rect.x, rect.top, rect.w, rect.h);
  octx.clip();
  octx.lineWidth = 1.5;
  if (fresh && frame.dark) {
    // camera blackout: no picture, just static
    octx.fillStyle = 'rgba(8, 10, 14, .82)';
    octx.fillRect(rect.x, rect.top, rect.w, rect.h);
    for (let i = 0; i < 26; i++) {
      octx.fillStyle = `rgba(200, 210, 225, ${(0.05 + Math.random() * 0.12).toFixed(2)})`;
      octx.fillRect(rect.x, rect.top + Math.random() * rect.h, rect.w, 1 + Math.random() * 2);
    }
    const left = Math.max(0, vs.vision.darkUntil - vs.t);
    octx.fillStyle = '#ff7a7a';
    octx.font = `700 ${Math.round(Math.min(28, rect.w / 9))}px ui-sans-serif, system-ui, sans-serif`;
    octx.textAlign = 'center';
    const cy = rect.top + rect.h * 0.34; // above the toasts, which sit mid-screen
    octx.fillText('NO SIGNAL', rect.x + rect.w / 2, cy);
    octx.font = '600 11px ui-sans-serif, system-ui, sans-serif';
    octx.fillStyle = '#c8d2e1';
    octx.fillText(`camera back in ${left.toFixed(1)} s`, rect.x + rect.w / 2, cy + 18);
    octx.textAlign = 'start';
  }
  for (const det of fresh && rect !== eyeRect ? frame.dets : []) {
    const pts = det.world.map((p) => toScreen(p, rect));
    if (pts.some((q) => !q)) continue;
    if (det.ok) found++;
    const col = det.ok ? '#3ddc97' : '#ff5c5c';
    octx.strokeStyle = col;
    octx.beginPath();
    pts.forEach(([x, y], i) => (i ? octx.lineTo(x, y) : octx.moveTo(x, y)));
    octx.closePath();
    octx.globalAlpha = 0.45;
    octx.stroke();
    octx.globalAlpha = 1;
    for (const [x, y] of pts) octx.strokeRect(x - 3.5, y - 3.5, 7, 7);
  }
  octx.restore();
  if (label) {
    octx.font = '600 11px ui-sans-serif, system-ui, sans-serif';
    octx.fillStyle = 'rgba(11, 18, 32, .6)';
    octx.fillRect(label.x, label.y, 112, 18);
    octx.fillStyle = frame.dark ? '#ff7a7a' : found ? '#3ddc97' : '#8fa0b8';
    octx.fillText(frame.dark ? 'VISION · dark' : `${frame.img ? 'PIXELS' : 'VISION'} · ${found} gate${found === 1 ? '' : 's'}`, label.x + 6, label.y + 13);
  }
}

// ---------- main loop ----------
let acc = 0, last = performance.now(), propSpin = 0, frameNo = 0;
const controls = document.querySelector('.controls');
let controlsTop = controls.getBoundingClientRect().top;

const spinQ = new THREE.Quaternion(), spinAxis = new THREE.Vector3(), bodyM = new THREE.Matrix4();
const bX = new THREE.Vector3(), bY = new THREE.Vector3(), bZ = new THREE.Vector3();
function syncMesh(mesh, d) {
  mesh.position.set(...d.pos);
  // the same body axes the onboard camera uses (in a split-S the body rolls inverted,
  // which a heading plus a tilt onto the thrust can't describe)
  const { bx, by, bz } = bodyAxes(d);
  bodyM.makeBasis(bX.set(...bx), bY.set(...by), bZ.set(...bz));
  mesh.quaternion.setFromRotationMatrix(bodyM);
  if (d.tumble) {
    // crashed: spin about the axis the impact gave it
    spinQ.setFromAxisAngle(spinAxis.set(...d.tumble.axis), d.tumble.angle);
    mesh.quaternion.premultiply(spinQ);
  }
}
function syncDrone() { syncMesh(drone, sim.drone); }

// 3, 2, 1, GO! while both racers sit on the pad
function updateCountdown() {
  const el = $('countdown'), left = sim.launchAt - sim.t;
  el.hidden = left < -0.8;
  if (el.hidden) return;
  const txt = left > 0 ? String(Math.ceil(left)) : 'GO!';
  if (el.textContent !== txt) el.textContent = txt;
  el.classList.toggle('go', left <= 0);
}

function updateCameras(dtFrame) {
  const d = viewSim().drone;
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
  } else if (camMode === 'tv') {
    if (!director) buildDirector();
    tvShot = direct(director, tvSubjects(), dtFrame, replay ? TV.replay.speed : timeScale);
    if (field && tvShot.subject !== followId && field.entries.some((e) => e.id === tvShot.subject)) setFollow(tvShot.subject);
    if (tvShot.pos) {
      chaseCam.position.set(...tvShot.pos);
      chaseCam.lookAt(...tvShot.look);
    }
  }
  // the director frames for a landscape picture: on a portrait screen keep the same width
  // in shot (so a telephoto close-up isn't cropped to the middle of the drone)
  let fov = camMode === 'tv' && tvShot?.pos ? tvShot.fov : CHASE_FOV;
  const aspect = innerWidth / innerHeight;
  if (fov !== CHASE_FOV && aspect < 16 / 9) fov = Math.min(80, 2 * Math.atan(Math.tan(fov * Math.PI / 360) * (16 / 9) / aspect) * 180 / Math.PI);
  if (Math.abs(chaseCam.fov - fov) > 1e-3) {
    chaseCam.fov = fov;
    chaseCam.updateProjectionMatrix();
  }
  // name tags keep their size on screen whatever the zoom
  labelZoom = Math.tan(fov * Math.PI / 360) / Math.tan(CHASE_FOV * Math.PI / 360);
  camMount.getWorldPosition(fpvCam.position);
  camMount.getWorldQuaternion(fpvCam.quaternion);
  fpvCam.rotateY(Math.PI); // three.js cameras look down -Z; the drone's nose is +Z
}

function frame(now) {
  frameNo++;
  const dtFrame = Math.min(0.1, (now - last) / 1000);
  last = now;
  acc += dtFrame * timeScale;
  if (pilot === 'manual') sim.stick = readSticks(dtFrame);
  if (replay) acc = 0; // the race waits while the replay plays
  let steps = 0;
  while (acc >= DT && steps < 1200) {
    if (field) shareTraffic(fieldSims);
    step(sim, DT);
    if (recordGhost(ghostRec, sim) && pilot === 'manual') saveGhost(ghostRec.best);
    if (rival) step(rival, DT);
    if (field) {
      for (const e of field.entries) if (e.sim !== sim) step(e.sim, DT);
      for (const h of bumpAll(fieldSims)) onContact(h);
    }
    sampleTelemetry(telemetry, sim);
    acc -= DT;
    if (++steps % 6 === 0) pushTrail(sim.drone.pos);
  }

  if (tvOn() && !replay && steps) {
    if (!director) buildDirector();
    recordReplay(replayBuf, sim.t, tvEntries());
    tvWatch();
  }
  syncDrone();
  if (rival) {
    syncRace(race, sim, rival);
    syncMesh(rivalMesh, rival.drone);
  }
  if (field) updateField();
  if (rival || field) updateCountdown();
  updateGhost();
  updateEstimate();
  updateStreaks(steps * DT);
  updateDebris(steps * DT);
  updateWash(steps * DT);
  updateHeadlight();
  if (replay) applyReplay(dtFrame);
  trail.visible = !replay;
  { const vs = viewSim(); audio.update(motorTone(vs.drone, { crashed: !!vs.crash || !!replay })); }
  propSpin += dtFrame * 60;
  updateCameras(dtFrame);
  updateHud();
  updateTvGraphics(innerHeight);
  if ((rival ?? sim).planVersion !== lineVersion) drawRacingLine();
  updateLearn();
  if (!$('telemetry').hidden && frameNo % 2 === 0) drawTelemetry(telemetry, sim);
  ghosts.forEach((g, i) => g.position.set(...sim.belief.gates[i].pos));

  const W = innerWidth, H = innerHeight;
  // touch sticks sit just above the buttons; the onboard view goes above them
  if (frameNo % 30 === 0) controlsTop = controls.getBoundingClientRect().top;
  const padsOn = !$('pads').hidden, padsBottom = H - controlsTop + 26;
  if (padsOn) $('pads').style.bottom = `${padsBottom}px`;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, W, H);
  // picture-in-picture onboard view, kept clear of the control buttons; with Eyes: pixels it
  // shows the drone's own camera picture instead
  if (frameNo % 30 === 0) controlsTop = controls.getBoundingClientRect().top;
  const w = Math.round(Math.min(Math.max(W * 0.3, 160), 360)), h = Math.round(w * 9 / 16);
  const x = W - w - 14, y = Math.max(78, H * 0.12, padsOn ? padsBottom + $('pad-r').offsetHeight + 10 : H - controlsTop + 10);
  const pip = { x, top: H - y - h, w, h };
  const eyeOn = !!viewSim().vision?.eye;
  const lbl = $('fpv-label');
  const onboardView = camMode === 'fpv' || (tvOn() && tvShot?.kind === 'onboard' && !replay);
  if (onboardView) {
    fpvCam.aspect = W / H;
    fpvCam.updateProjectionMatrix();
    const vm = viewMesh();
    vm.visible = false;
    renderer.render(scene, fpvCam);
    vm.visible = true;
    drawOverlay({ x: 0, top: 0, w: W, h: H }, W >= 700 ? { x: W / 2 - 56, y: 16 } : null, eyeOn ? pip : null);
  } else {
    renderer.render(scene, chaseCam);
    if (replay) drawOverlay(null, null, null);
    else if (!eyeOn) {
      fpvCam.aspect = w / h;
      fpvCam.updateProjectionMatrix();
      renderer.setScissorTest(true);
      renderer.setScissor(x, y, w, h);
      renderer.setViewport(x, y, w, h);
      const vm = viewMesh();
      vm.visible = false;
      renderer.render(scene, fpvCam);
      vm.visible = true;
    }
    if (!replay) drawOverlay(eyeOn ? null : pip, w >= 180 ? { x: x + 6, y: H - y - h + 6 } : null, eyeOn ? pip : null);
  }
  lbl.style.display = (onboardView && !eyeOn) || replay ? 'none' : 'block';
  lbl.style.bottom = `${y + h + 4}px`;
  const lblText = eyeOn ? (w >= 240 ? "Drone's eye · 320×180 pixels" : "Drone's eye") : 'Onboard camera';
  if (lbl.textContent !== lblText) lbl.textContent = lblText;
  requestAnimationFrame(frame);
}

addEventListener('resize', () => {
  renderer.setSize(innerWidth, innerHeight);
  sizeOverlay();
  controlsTop = controls.getBoundingClientRect().top;
  chaseCam.aspect = innerWidth / innerHeight;
  chaseCam.updateProjectionMatrix();
});

readHash();
buildWorld();
applyLook();
chaseCam.position.set(sim.drone.pos[0], 6, sim.drone.pos[2] - 12);
requestAnimationFrame(frame);
landing.boot(initialHash);
