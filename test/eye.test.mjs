// Vision step 3: the pixel camera. A software-rendered picture, a gate detector that only
// sees pixels, data association from the drone's beliefs, and whole flights on it.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  makeSim, step, v3, GATE_INNER, FRAME, CAMERA, PIXELS, EYES, mulberry32, gateCorners, project,
  cameraPose, navError, DIFFICULTIES,
} from '../src/sim.js';
import { EYE, buildScene, renderView, detectGates, associate, toPixel, toPlane, isOrange } from '../src/eye.js';

const tanV = Math.tan(CAMERA.vfov / 2);
const mean = (xs) => xs.reduce((a, b) => a + b, 0) / xs.length;
const SCENE = { inner: GATE_INNER, bar: FRAME.bar, legRadius: FRAME.legRadius };
// a gate facing +z (the way the camera below looks), and a camera 3 m up at the origin
const gate = (id, pos) => ({ id, pos, normal: [0, 0, 1], right: [1, 0, 0], up: [0, 1, 0] });
const CAM = { pos: [0, 3, 0], fwd: [0, 0, 1], up: [0, 1, 0], right: [-1, 0, 0] };
// a gate's outer outline in pixels: its face nearest the camera, which is what is seen
const outline = (g, cam = CAM) => gateCorners(g, v3.sub(g.pos, v3.scale(g.normal, FRAME.bar / 2)), g.right, g.up, PIXELS.outer)
  .map((p) => toPixel(...project(cam, p), tanV));
// mean corner distance between a quad and an outline, over corner orders
function cornerError(corners, ref) {
  let best = Infinity;
  for (const mirror of [false, true]) for (let r = 0; r < 4; r++) {
    best = Math.min(best, mean(ref.map((p, k) => {
      const c = corners[mirror ? (r - k + 8) % 4 : (r + k) % 4];
      return Math.hypot(c[0] - p[0], c[1] - p[1]);
    })));
  }
  return best;
}
const moved = (o, dx, dy, s = 1) => {
  const cx = mean(o.map((p) => p[0])), cy = mean(o.map((p) => p[1]));
  return o.map(([x, y]) => [cx + (x - cx) * s + dx, cy + (y - cy) * s + dy]);
};

test('eye: pixel and image-plane coordinates round-trip, and the picture is the camera model\'s', () => {
  for (const [x, y] of [[0, 0], [0.5, -0.3], [-1.2, 0.8]]) {
    const [u, v] = toPixel(x, y, tanV), [x2, y2] = toPlane(u, v, tanV);
    assert.ok(Math.abs(x - x2) < 1e-12 && Math.abs(y - y2) < 1e-12);
  }
  // straight ahead is the middle of the picture; the picture has the camera's shape
  assert.deepEqual(toPixel(0, 0, tanV), [EYE.W / 2, EYE.H / 2]);
  assert.ok(Math.abs(EYE.W / EYE.H - CAMERA.aspect) < 1e-9);
  assert.deepEqual(EYES, ['model', 'pixels']);
});

test('eye: a gate straight ahead is drawn in orange where it should be, and found to within a pixel', () => {
  const g = gate(0, [0, 3, 12]);
  const img = renderView(CAM, buildScene([g], [], SCENE), tanV, mulberry32(1));
  let gatePx = 0, gateOrange = 0, rest = 0, restOrange = 0;
  for (let k = 0; k < EYE.W * EYE.H; k++) {
    const orange = isOrange(img.rgb[k * 4], img.rgb[k * 4 + 1], img.rgb[k * 4 + 2]);
    if (img.gate[k] === 0) { gatePx++; if (orange) gateOrange++; } else { rest++; if (orange) restOrange++; }
  }
  assert.ok(gatePx > 300, `only ${gatePx} gate pixels`);
  assert.ok(gateOrange / gatePx > 0.98, `${gatePx - gateOrange} gate pixels don't look orange`);
  assert.equal(restOrange, 0, 'sky, ground or legs look orange');
  const quads = detectGates(img);
  assert.equal(quads.length, 1);
  const err = cornerError(quads[0].corners, outline(g));
  // (edges lined up with the pixel grid can only be placed to the nearest pixel boundary)
  assert.ok(err < 0.8, `corners ${err.toFixed(2)} px out`);
  assert.ok(quads[0].fill > 0.2 && quads[0].fill < 0.4, `fill ${quads[0].fill}`);
  // turned 30 degrees and seen off to one side, it is still found accurately
  const n = [Math.sin(0.5), 0, Math.cos(0.5)];
  const g2 = { id: 0, pos: [3, 4, 15], normal: n, right: [n[2], 0, -n[0]], up: [0, 1, 0] };
  const q2 = detectGates(renderView(CAM, buildScene([g2], [], SCENE), tanV, mulberry32(2)));
  assert.equal(q2.length, 1);
  const err2 = cornerError(q2[0].corners, outline(g2));
  assert.ok(err2 < 0.8, `turned gate's corners ${err2.toFixed(2)} px out`);
});

test('eye: things hide each other, and a gate with a side hidden is not taken for a whole one', () => {
  const g = gate(0, [0, 3, 12]);
  const full = renderView(CAM, buildScene([g], [], SCENE), tanV, mulberry32(1));
  const count = (img) => img.gate.reduce((n, v) => n + (v === 0 ? 1 : 0), 0);
  // a pillar in front of the gate's left-hand bar (image right is -x, so +x is the left)
  const pillar = { pos: [1.6, 3, 9], size: [1, 6, 1], yaw: 0 };
  const hid = renderView(CAM, buildScene([g], [pillar], SCENE), tanV, mulberry32(1));
  assert.ok(count(hid) < count(full) * 0.92, 'the pillar hides part of the frame');
  assert.equal(detectGates(hid).length, 0, 'a frame with a side missing is not a gate');
  // behind a wide pillar, nothing of the gate is drawn at all
  const wall = { pos: [0, 4, 8], size: [8, 8, 0.5], yaw: 0 };
  const gone = renderView(CAM, buildScene([g], [wall], SCENE), tanV, mulberry32(1));
  assert.equal(count(gone), 0);
  assert.equal(detectGates(gone).length, 0);
  // and a gate behind the camera is not drawn either
  const behind = renderView(CAM, buildScene([gate(0, [0, 3, -10])], [], SCENE), tanV, mulberry32(1));
  assert.equal(count(behind), 0);
});

test('eye: a far gate overlapping a near one makes one blob; with what it expects to see, the near gate comes out', () => {
  // the far gate's corner pokes out from behind the near gate's top corner
  const near = gate(0, [0, 3, 12]), far = gate(1, [-6.2, 9.2, 30]);
  const img = renderView(CAM, buildScene([near, far], [], SCENE), tanV, mulberry32(3));
  // on its own, the merged blob is not mistaken for a gate (its hull is no gate)
  for (const q of detectGates(img)) {
    assert.ok(Math.min(cornerError(q.corners, outline(near)), cornerError(q.corners, outline(far))) < 1.5, 'a bogus quad');
  }
  assert.ok(!detectGates(img).some((q) => cornerError(q.corners, outline(near)) < 1.5));
  // told roughly where the gates should be (shifted and resized, as a map error would),
  // it takes the near one out of the blob
  const pred = [{ id: 0, corners: moved(outline(near), 6, -5, 1.05) }, { id: 1, corners: moved(outline(far), -4, 3, 0.95) }];
  const quads = detectGates(img, pred);
  const hit = quads.find((q) => cornerError(q.corners, outline(near)) < 0.8);
  assert.ok(hit, 'the near gate was not found');
  assert.ok(hit.split);
  assert.equal(associate(quads, pred).find((m) => m.quad === hit)?.id, 0);
});

test('associate: size decides between gates in the same place, ambiguity is left out, corners come in the prediction\'s order', () => {
  const sq = (cx, cy, s) => [[cx - s, cy - s], [cx + s, cy - s], [cx + s, cy + s], [cx - s, cy + s]];
  // a near gate and a far one predicted at the same spot: a big quad is the near one
  const pred = [{ id: 3, corners: sq(100, 80, 20) }, { id: 4, corners: sq(102, 78, 8) }];
  const big = { corners: sq(104, 82, 19) }, small = { corners: sq(99, 79, 8.5) };
  const m = associate([big, small], pred);
  assert.equal(m.length, 2);
  assert.equal(m.find((x) => x.quad === big).id, 3);
  assert.equal(m.find((x) => x.quad === small).id, 4);
  // two gates of the same size either side of a quad: it could be either, so neither
  const twins = [{ id: 0, corners: sq(90, 80, 15) }, { id: 1, corners: sq(110, 80, 15) }];
  assert.equal(associate([{ corners: sq(100, 80, 15) }], twins).length, 0);
  // too far from anything predicted, or the wrong size: no gate
  assert.equal(associate([{ corners: sq(200, 80, 15) }], twins).length, 0);
  assert.equal(associate([{ corners: sq(90, 80, 30) }], twins).length, 0);
  // the quad's corners start anywhere and run either way round: they come back in the
  // prediction's order
  const p = sq(100, 80, 15), rolled = { corners: [p[2], p[1], p[0], p[3]].map(([x, y]) => [x + 1, y - 1]) };
  const [r] = associate([rolled], [{ id: 7, corners: p }]);
  assert.equal(r.id, 7);
  r.corners.forEach((c, k) => assert.ok(Math.hypot(c[0] - p[k][0] - 1, c[1] - p[k][1] + 1) < 1e-9));
});

// ---------- flights ----------
const SEEDS = [1, 2, 3, 4, 5, 7, 11, 42];
const flights = new Map();
// A whole flight on the pixel camera, checking every quad it puts to a gate against the
// true outline of every gate (the detector and the association never see these).
function pixelFlight(seed, difficulty, { nav = 'vision', eyes = 'pixels', seconds = difficulty === 'hard' ? 45 : 40, wind = 'off' } = {}) {
  const key = `${seed}/${difficulty}/${nav}/${eyes}/${seconds}/${wind}`;
  if (flights.has(key)) return flights.get(key);
  const sim = makeSim({ seed, difficulty, nav, eyes, wind });
  const r = { sim, estErr: [], quads: 0, split: 0, assigned: 0, wrong: 0, cornerErr: [], navErr: [], rel: [] };
  let target = 0, lastFrame = -1;
  for (let i = 0; i < Math.round(seconds * 120); i++) {
    step(sim, 1 / 120);
    if (sim.state.target !== target) {
      r.estErr.push(v3.len(v3.sub(sim.belief.gates[target].pos, sim.course.gates[target].pos)));
      target = sim.state.target;
    }
    if (sim.est && sim.t > sim.launchAt) {
      r.navErr.push(navError(sim).err);
      const g = sim.course.gates[target], x = sim.est.x;
      if (v3.len(v3.sub(g.pos, sim.drone.pos)) < 15) {
        r.rel.push(v3.len(v3.sub(v3.sub(sim.belief.gates[target].pos, [x[0], x[1], x[2]]), v3.sub(g.pos, sim.drone.pos))));
      }
    }
    const f = sim.vision.frame;
    if (eyes !== 'pixels' || !f?.quads || f.t === lastFrame) continue;
    lastFrame = f.t;
    const cam = cameraPose(sim.drone);
    const fits = (q, g) => {
      const pj = gateCorners(g, g.pos, g.right, g.up, PIXELS.outer).map((p) => project(cam, p));
      return pj.some((p) => !p) ? Infinity : cornerError(q.corners, pj.map((p) => toPixel(...p, tanV)));
    };
    for (const q of f.quads) {
      r.quads++;
      if (q.split) r.split++;
      if (q.gate === undefined) continue;
      r.assigned++;
      const mine = fits(q, sim.course.gates[q.gate]);
      r.cornerErr.push(mine);
      if (sim.course.gates.some((g) => g.id !== q.gate && fits(q, g) < mine)) r.wrong++;
    }
  }
  flights.set(key, r);
  return r;
}

for (const [difficulty, seeds] of [['easy', [1, 42]], ['normal', SEEDS], ['hard', [1, 3, 4, 42]], ['pro', [1, 2]]]) {
  test(`pixels (vision): ${difficulty} courses flown cleanly on what the camera really sees`, () => {
    for (const seed of seeds) {
      const r = pixelFlight(seed, difficulty), s = r.sim.state, tag = `seed ${seed}`;
      assert.equal(s.misses, 0, `${tag}: ${s.misses} missed`);
      assert.equal(s.crashes, 0, `${tag}: ${s.crashes} crashed`);
      assert.ok(s.laps.length >= 2, `${tag}: ${s.laps.length} laps`);
      // the gates it flies through are where it thinks they are
      assert.ok(mean(r.estErr) < (['hard', 'pro'].includes(difficulty) ? 0.12 : 0.08), `${tag}: estimates ${mean(r.estErr).toFixed(3)} m out on average`);
      assert.ok(Math.max(...r.estErr) < 0.6, `${tag}: a gate passed ${Math.max(...r.estErr).toFixed(2)} m from its estimate`);
      // every quad it put to a gate was that gate, found to about half a pixel
      assert.equal(r.wrong, 0, `${tag}: ${r.wrong} of ${r.assigned} quads put to the wrong gate`);
      assert.ok(r.assigned > 300, `${tag}: only ${r.assigned} gates seen`);
      const sorted = r.cornerErr.slice().sort((a, b) => a - b);
      assert.ok(sorted[sorted.length >> 1] < 0.8, `${tag}: median corner error ${sorted[sorted.length >> 1].toFixed(2)} px`);
      assert.ok(r.sim.vision.stats.replans > 0);
    }
  });
}

test('pixels: overlapping gates are common, and the detector takes them apart', () => {
  let split = 0, quads = 0;
  for (const seed of SEEDS) { const r = pixelFlight(seed, 'normal'); split += r.split; quads += r.quads; }
  assert.ok(split > 100 && split / quads > 0.02, `only ${split} of ${quads} quads came out of merged blobs`);
});

test('pixels cost no lap time against the corner model', () => {
  const ratios = SEEDS.map((seed) => {
    const px = pixelFlight(seed, 'normal').sim.state.laps[1];
    const model = pixelFlight(seed, 'normal', { eyes: 'model' }).sim.state.laps[1];
    assert.ok(px < model * 1.02, `seed ${seed}: pixels ${px.toFixed(2)} s vs model ${model.toFixed(2)} s`);
    return px / model;
  });
  assert.ok(mean(ratios) < 1.005, `pixel laps average ${((mean(ratios) - 1) * 100).toFixed(2)}% slower`);
});

for (const difficulty of ['normal', 'hard']) {
  test(`pixels (vision + IMU): ${difficulty} courses flown cleanly, knowing where it is`, () => {
    for (const seed of [1, 4, 42]) {
      const r = pixelFlight(seed, difficulty, { nav: 'vio' }), s = r.sim.state, tag = `seed ${seed}`;
      assert.equal(s.misses + s.crashes, 0, `${tag}: ${s.misses} missed, ${s.crashes} crashed`);
      assert.ok(s.laps.length >= 2, `${tag}: ${s.laps.length} laps`);
      assert.ok(Math.max(...r.navErr) < 1.3, `${tag}: position ${Math.max(...r.navErr).toFixed(2)} m out at worst`);
      assert.ok(mean(r.navErr) < 0.45, `${tag}: position ${mean(r.navErr).toFixed(2)} m out on average`);
      // (on hard courses gates come into sight late, a few metres before the drone is in them)
      assert.ok(mean(r.rel) < (difficulty === 'hard' ? 0.2 : 0.15), `${tag}: ${mean(r.rel).toFixed(3)} m out relative to the next gate on average`);
      assert.equal(r.wrong, 0, `${tag}: ${r.wrong} quads put to the wrong gate`);
      assert.ok(r.sim.est.stats.updates > 500);
    }
  });
}

test('pixels: clean in gusty wind too', () => {
  for (const [seed, difficulty, nav] of [[2, 'normal', 'vision'], [5, 'hard', 'vision'], [7, 'normal', 'vio']]) {
    const s = pixelFlight(seed, difficulty, { nav, wind: 'gusty', seconds: 52 }).sim.state;
    assert.equal(s.misses + s.crashes, 0, `${difficulty} seed ${seed} ${nav}: ${s.misses} missed, ${s.crashes} crashed`);
    assert.ok(s.laps.length >= 2);
  }
});

test('pixels: flights are deterministic, the picture is kept for the overlay, and only camera modes get an eye', () => {
  const fly = () => {
    const sim = makeSim({ seed: 3, difficulty: 'hard', nav: 'vio', eyes: 'pixels' });
    for (let i = 0; i < 120 * 6; i++) step(sim, 1 / 120);
    return sim;
  };
  const a = fly(), b = fly();
  assert.deepEqual(a.drone.pos, b.drone.pos);
  assert.deepEqual(a.est.x, b.est.x);
  const f = a.vision.frame;
  assert.equal(f.img.W, EYE.W);
  assert.equal(f.img.rgb.length, EYE.W * EYE.H * 4);
  assert.ok(Array.isArray(f.quads) && Array.isArray(f.pred));
  assert.equal(a.eyes, 'pixels');
  // ground truth and map-only have no camera, so pixels mean nothing there
  for (const nav of ['truth', 'blind']) {
    const s = makeSim({ seed: 3, nav, eyes: 'pixels' });
    assert.equal(s.eyes, 'model');
    assert.ok(!s.vision?.eye);
  }
  // and the corner model is still the default
  assert.equal(makeSim({ seed: 3, nav: 'vision' }).vision.eye, null);
});
