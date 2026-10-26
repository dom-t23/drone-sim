// Learning from its own laps: iterative learning control in the racing pilot, and the
// confidence that lets the planner push once the line is flown accurately (2026-10-26).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeSim, step, LEARN, learnedOpts, RACING, SHAPING, v3, GATE_INNER, CRASH } from '../src/sim.js';
import { learnSummary, learnNote, confLabel, drawLearnChart } from '../src/learn.js';

const DT = 1 / 120;
function fly(opts, seconds) {
  const sim = makeSim(opts);
  for (let i = 0, n = Math.round(seconds / DT); i < n; i++) step(sim, DT);
  return sim;
}
const cache = new Map();
function learnFlight(seed, difficulty, { wind = 'off', seconds = 200 } = {}) {
  const key = `${seed}/${difficulty}/${wind}/${seconds}`;
  if (!cache.has(key)) cache.set(key, fly({ seed, difficulty, wind, learn: true }, seconds));
  return cache.get(key);
}
// pure ILC: the confidence never steps, so the line and its speeds stay as planned
function ilcOnly(seed, difficulty, seconds) {
  const step0 = LEARN.step;
  LEARN.step = 0;
  try { return fly({ seed, difficulty, learn: true }, seconds); } finally { LEARN.step = step0; }
}

test('learning: only the racing line on ground truth learns, and only when asked', () => {
  assert.ok(makeSim({ seed: 1, learn: true }).learn);
  assert.equal(makeSim({ seed: 1 }).learn, null);
  for (const o of [{ pilot: 'pursuit' }, { pilot: 'manual' }, { nav: 'vision' }, { nav: 'vio' }, { nav: 'blind' }, { optimise: false }]) {
    assert.equal(makeSim({ seed: 1, learn: true, ...o }).learn, null, JSON.stringify(o));
  }
});

test('learning: the first lap is flown exactly as without it', () => {
  const a = fly({ seed: 3 }, 16), b = fly({ seed: 3, learn: true }, 16);
  assert.equal(b.state.laps.length, 0, 'still on the first lap');
  assert.deepEqual(a.drone.pos, b.drone.pos);
});

test('ILC: the repeated tracking error melts away lap by lap (the line itself unchanged)', () => {
  const limits = { easy: 0.15, normal: 0.3, hard: 0.55 };
  for (const difficulty of ['easy', 'normal', 'hard']) {
    for (const seed of [1, 2, 5]) {
      const sim = ilcOnly(seed, difficulty, difficulty === 'hard' ? 130 : 100);
      const laps = sim.learn.laps;
      assert.ok(laps.length >= 5, `${difficulty} ${seed}: ${laps.length} laps`);
      assert.ok(laps.every((l) => l.conf === 0) && sim.planVersion === 0, 'no pushing in this test');
      const first = laps[0].rms, last = laps.at(-1).rms;
      assert.ok(first > 0.2, `${difficulty} ${seed}: lap 1 already ${first.toFixed(2)} m off`);
      assert.ok(last < first * limits[difficulty], `${difficulty} ${seed}: tracking ${first.toFixed(2)} → ${last.toFixed(2)} m`);
      // and it gets there steadily, not by luck on the last lap
      assert.ok(laps[3].rms < laps[0].rms * 0.75, `${difficulty} ${seed}: slow to learn`);
      // so it threads the gates where it planned to
      assert.ok(laps.at(-1).gate < laps[0].gate, `${difficulty} ${seed}: gate crossing ${laps[0].gate.toFixed(2)} → ${laps.at(-1).gate.toFixed(2)} m`);
      assert.equal(sim.state.misses + sim.state.crashes, 0);
    }
  }
});

test('ILC: the learned correction is bounded, smooth along the line and finite', () => {
  const sim = learnFlight(2, 'normal'), L = sim.learn, N = L.N, ds = sim.plan.ds;
  assert.equal(L.c.length, 3 * sim.plan.pts.length, 'one correction per point of the current line');
  let maxStep = 0, maxC = 0;
  for (let k = 0; k < N; k++) {
    const c = [L.c[3 * k], L.c[3 * k + 1], L.c[3 * k + 2]], n = (k + 1) % N;
    assert.ok(c.every(Number.isFinite));
    maxC = Math.max(maxC, v3.len(c));
    maxStep = Math.max(maxStep, v3.len(v3.sub(c, [L.c[3 * n], L.c[3 * n + 1], L.c[3 * n + 2]])));
  }
  assert.ok(maxC <= LEARN.max + 1e-9 && maxC > 0.5, `largest correction ${maxC.toFixed(2)} m/s²`);
  assert.ok(maxStep / ds < 2, `correction changes ${(maxStep / ds).toFixed(2)} m/s² per m`); // Q-filtered
});

test('learning: a clean, accurate lap steps the confidence up, and the planner gets more envelope', () => {
  const o = learnedOpts({}, 1);
  assert.ok(o.turnMargin > RACING.turnMargin && o.comboMargin > RACING.comboMargin && o.aAccel > RACING.aAccel && o.aBrake > RACING.aBrake);
  assert.ok(o.comboMargin < 1 && o.turnMargin < 1, 'never the whole envelope');
  assert.deepEqual(learnedOpts({ turnMargin: 0.7 }, 0), { turnMargin: 0.7, comboMargin: RACING.comboMargin, aAccel: RACING.aAccel, aBrake: RACING.aBrake });
  // crossings may move out towards the frame, but never so far the drone can't clear it
  assert.ok(SHAPING.maxOffset + LEARN.push.offset + LEARN.okGate + CRASH.radius < GATE_INNER / 2, 'room left at the frame');
  const sim = learnFlight(1, 'easy');
  const L = sim.learn;
  for (let i = 1; i < L.laps.length; i++) {
    const a = L.laps[i - 1].conf, l = L.laps[i];
    if (l.conf > a) assert.ok(l.rms < LEARN.okRms && l.gate < LEARN.okGate && !l.bad, `lap ${i + 1} stepped up on a sloppy lap`);
  }
  assert.equal(L.conf, 1, 'easy course: all the way up');
  // the line it flies now crosses gates further out than the shaping alone allows
  assert.ok(Math.max(...sim.shape.map((s) => Math.max(Math.abs(s.dr), Math.abs(s.du)))) > SHAPING.maxOffset + 0.05);
});

test('learning: it gets faster by itself, and never at the cost of a gate', () => {
  // 2026-10-26: over 30 seeds per difficulty the best lap averages 7.8% (easy), 8.3% (normal)
  // and 6.5% (hard) under lap 1; these five seeds give 8.1, 8.0 and 6.4%
  const want = { easy: 0.075, normal: 0.07, hard: 0.055 };
  for (const difficulty of ['easy', 'normal', 'hard']) {
    const gains = [];
    for (const seed of [1, 2, 3, 4, 7]) {
      const sim = learnFlight(seed, difficulty, { seconds: difficulty === 'hard' ? 230 : 200 });
      const laps = sim.state.laps, best = Math.min(...laps);
      assert.equal(sim.state.misses, 0, `${difficulty} ${seed}: missed a gate`);
      assert.equal(sim.state.crashes, 0, `${difficulty} ${seed}: crashed`);
      assert.ok(best < laps[0], `${difficulty} ${seed}: no faster than lap 1`);
      gains.push(1 - best / laps[0]);
    }
    const mean = gains.reduce((a, b) => a + b) / gains.length;
    assert.ok(mean > want[difficulty], `${difficulty}: best lap only ${(mean * 100).toFixed(1)}% under lap 1`);
  }
});

test('learning: clean in gusty wind too', () => {
  for (const [seed, difficulty] of [[1, 'easy'], [3, 'normal'], [5, 'hard']]) {
    const sim = learnFlight(seed, difficulty, { wind: 'gusty', seconds: 200 });
    assert.equal(sim.state.misses + sim.state.crashes, 0, `${difficulty} ${seed}`);
    assert.ok(sim.learn.conf >= 0.5, `${difficulty} ${seed}: confidence only ${sim.learn.conf}`);
    assert.ok(Math.min(...sim.state.laps) < sim.state.laps[0] * 0.97);
  }
});

test('learning: a crash backs it off at once, and it never pushes that hard again on that course', () => {
  // hard course 27 is the one in 180 sweep flights where full confidence was too much
  const sim = learnFlight(27, 'hard', { seconds: 300 });
  const L = sim.learn, crashes = sim.events.filter((e) => e.type === 'crash');
  assert.equal(crashes.length, 1, `${crashes.length} crashes`);
  const back = sim.events.find((e) => e.type === 'learn' && e.rms == null);
  assert.ok(back && back.t > crashes[0].t && back.step === -1, 'backed off at the respawn');
  assert.ok(L.ceil < 1 && L.conf <= L.ceil, `ceiling ${L.ceil}, confidence ${L.conf}`);
  assert.ok(sim.state.laps.length >= 3 + L.laps.findIndex((l) => l.bad), 'and kept on lapping');
  assert.equal(sim.state.misses, 0);
});

test('learning: the correction follows a re-planned line', () => {
  const sim = learnFlight(1, 'easy'), L = sim.learn;
  assert.ok(sim.planVersion >= 4, 're-planned as it stepped up');
  assert.equal(L.c.length, 3 * sim.plan.pts.length);
  assert.equal(L.N, sim.plan.pts.length);
  // and it is still learning on the latest line: tracking stays tight after the last step
  assert.ok(L.laps.at(-1).rms < 0.1, `tracking ${L.laps.at(-1).rms.toFixed(2)} m on the final line`);
});

test('learning: deterministic', () => {
  const a = fly({ seed: 4, learn: true }, 90), b = fly({ seed: 4, learn: true }, 90);
  assert.deepEqual(a.state.laps, b.state.laps);
  assert.deepEqual(a.drone.pos, b.drone.pos);
  assert.deepEqual(Array.from(a.learn.c), Array.from(b.learn.c));
});

test('learn panel: summary, notes, labels and chart', () => {
  assert.equal(learnSummary(makeSim({ seed: 1 })), null);
  const sim = learnFlight(1, 'easy'), sum = learnSummary(sim);
  assert.equal(sum.laps.length, Math.min(sim.state.laps.length, sim.learn.laps.length));
  assert.equal(sum.first, sim.state.laps[0]);
  assert.ok(sum.gain > 0.5 && Math.abs(sum.gain - (sum.first - sum.best)) < 1e-9);
  assert.ok(sum.rmsLast < sum.rmsFirst);
  assert.equal(confLabel(0), 'learning');
  assert.equal(confLabel(0.5), '50%');
  assert.equal(confLabel(1), 'flat out');
  assert.equal(learnNote({ rms: 0.12, conf: 0.25, step: 1 }, 13.2, 13.7), 'Lap 13.20 s (−0.50) · pushing harder');
  assert.equal(learnNote({ rms: 0.3, conf: 0, step: 0 }, 13.75, 13.7), 'Lap 13.75 s · 30 cm off line');
  assert.equal(learnNote({ rms: 0.05, conf: 1, step: 0 }, 12.6, 13.7), 'Lap 12.60 s (−1.10) · flat out');
  assert.equal(learnNote({ rms: null, conf: 0.5, step: -1 }), 'Too fast: backing off');
  // the chart draws onto any 2D context
  const calls = [];
  const ctx = new Proxy({}, { get: (_, k) => (k in ctx2 ? ctx2[k] : (...a) => calls.push([k, ...a])), set: () => true });
  const ctx2 = {};
  assert.equal(drawLearnChart(ctx, 236, 52, sum), Math.min(12, sum.laps.length));
  const bars = calls.filter((c) => c[0] === 'fillRect');
  assert.equal(bars.length, Math.min(12, sum.laps.length));
  for (const [, x, y, w, h] of bars) assert.ok(x >= 0 && x + w <= 236 && y >= 0 && y + h <= 52 + 1e-9 && h > 0);
  drawLearnChart(ctx, 236, 52, { laps: [] });
  assert.ok(calls.some((c) => c[0] === 'fillText'));
});
