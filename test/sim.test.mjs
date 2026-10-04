// Headless checks for the simulation core. Run with: node --test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeSim, step, planRacingLine, makeCourse, v3, GATE_INNER, DRONE, G, DIFFICULTIES } from '../src/sim.js';

const SEEDS = [1, 2, 3, 4, 5, 7, 11, 42];

function run(seed, seconds, { dt = 1 / 120, pilot = 'racing', difficulty = 'normal', onStep } = {}) {
  const sim = makeSim({ seed, pilot, difficulty });
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) {
    step(sim, dt);
    onStep?.(sim);
  }
  return sim;
}
const mean = (xs) => xs.reduce((a, b) => a + b, 0) / xs.length;

for (const pilot of ['racing', 'pursuit']) {
  test(`${pilot} pilot completes laps on several course seeds without missing gates`, () => {
    for (const seed of SEEDS) {
      const sim = run(seed, 150, { pilot });
      const { laps, misses } = sim.state;
      assert.ok(laps.length >= 2, `seed ${seed}: only ${laps.length} laps`);
      assert.equal(misses, 0, `seed ${seed}: ${misses} missed gates`);
    }
  });
}

test('racing line is clearly faster than axis pursuit, and the gain is locked in', () => {
  // 2026-10-03: racing line averages ~17.6 s a lap against ~27.3 s for pursuit.
  // 2026-10-04: friction-circle speed profile, ~17.2 s (slowest seed 17.7 s).
  for (const seed of SEEDS) {
    const racing = mean(run(seed, 80).state.laps.slice(1));
    const pursuit = mean(run(seed, 120, { pilot: 'pursuit' }).state.laps.slice(1));
    assert.ok(racing < 17.9, `seed ${seed}: racing lap ${racing.toFixed(2)} s`);
    assert.ok(racing < pursuit * 0.75, `seed ${seed}: racing ${racing.toFixed(2)} s vs pursuit ${pursuit.toFixed(2)} s`);
  }
});

test('racing pilot threads gates near the centre and stays on its planned line', () => {
  for (const seed of SEEDS) {
    let worstGate = 0, worstTrack = 0, target = 0;
    run(seed, 60, {
      onStep(sim) {
        if (sim.t > 5) worstTrack = Math.max(worstTrack, v3.len(v3.sub(sim.plan.pts[sim.track.idx].p, sim.drone.pos)));
        if (sim.state.target !== target) {
          const g = sim.course.gates[target];
          const off = v3.sub(sim.drone.pos, g.pos);
          worstGate = Math.max(worstGate, v3.len(v3.sub(off, v3.scale(g.normal, v3.dot(off, g.normal)))));
          target = sim.state.target;
        }
      },
    });
    assert.ok(worstGate < GATE_INNER / 2 - 0.8, `seed ${seed}: passed ${worstGate.toFixed(2)} m off centre`);
    assert.ok(worstTrack < 1.2, `seed ${seed}: strayed ${worstTrack.toFixed(2)} m from the racing line`);
  }
});

test('racing line passes through every gate centre and respects the airframe envelope', () => {
  for (const seed of SEEDS) {
    const course = makeCourse({ seed });
    const plan = planRacingLine(course);
    for (const g of course.gates) {
      const d = Math.min(...plan.pts.map((q) => v3.len(v3.sub(q.p, g.pos))));
      assert.ok(d < plan.ds, `seed ${seed}: line misses gate ${g.id} by ${d.toFixed(2)} m`);
    }
    for (const q of plan.pts) {
      assert.ok(Number.isFinite(q.v) && q.v > 0 && q.v <= plan.params.vMax);
      const a = v3.add(v3.scale(q.kv, q.v * q.v), [0, G, 0]); // turning + hover
      const tilt = Math.acos(a[1] / v3.len(a));
      assert.ok(tilt <= DRONE.maxTiltRad + 1e-6, `seed ${seed}: planned tilt ${(tilt * 57.3).toFixed(1)}°`);
      assert.ok(v3.len(a) <= DRONE.maxThrustAcc);
    }
    assert.ok(plan.lapTime > 10 && plan.lapTime < 25, `seed ${seed}: planned lap ${plan.lapTime}`);
  }
});

test('state stays finite and above the ground', () => {
  for (const pilot of ['racing', 'pursuit']) {
    const sim = makeSim({ seed: 3, pilot });
    for (let i = 0; i < 120 * 60; i++) {
      step(sim, 1 / 120);
      for (const x of [...sim.drone.pos, ...sim.drone.vel]) assert.ok(Number.isFinite(x));
      assert.ok(sim.drone.pos[1] >= 0.15);
    }
  }
});

test('simulation is deterministic for a given seed', () => {
  for (const pilot of ['racing', 'pursuit']) {
    const a = run(9, 30, { pilot });
    const b = run(9, 30, { pilot });
    assert.deepEqual(a.drone.pos, b.drone.pos);
  }
});

// ---------- difficulties (2026-10-04) ----------

// In-plane offset of the drone from a gate centre, in the gate's own right/up axes.
function gateOffset(g, pos) {
  const off = v3.sub(pos, g.pos);
  return Math.max(Math.abs(v3.dot(off, g.right)), Math.abs(v3.dot(off, g.up)));
}

test('normal courses keep their original layout, so old seeds fly the same course', () => {
  for (const seed of SEEDS) {
    const c = makeCourse({ seed });
    assert.equal(c.gates.length, 10);
    // the original generator, inlined: one random draw for the phase
    let a = seed | 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    const phase = (((t ^ (t >>> 14)) >>> 0) / 4294967296) * Math.PI * 2;
    c.gates.forEach((g, i) => {
      const u = (i / 10) * Math.PI * 2, r = 45 + 14 * Math.sin(2 * u + phase);
      const want = [r * Math.cos(u), 5 + 3 * Math.sin(3 * u + phase), r * Math.sin(u)];
      assert.ok(v3.len(v3.sub(g.pos, want)) < 1e-9, `seed ${seed}: gate ${i} moved`);
      assert.equal(g.normal[1], 0, `seed ${seed}: gate ${i} is no longer level`);
    });
  }
});

test('hard courses really are harder: big climbs, angled gates and dive gates', () => {
  for (const seed of SEEDS) {
    const { gates } = makeCourse({ seed, difficulty: 'hard' });
    const ys = gates.map((g) => g.pos[1]);
    const pitch = gates.map((g) => Math.asin(g.normal[1]) * 180 / Math.PI);
    assert.ok(gates.length >= 12);
    assert.ok(Math.max(...ys) - Math.min(...ys) > 5, `seed ${seed}: height range only ${(Math.max(...ys) - Math.min(...ys)).toFixed(1)} m`);
    assert.ok(Math.min(...pitch) < -10, `seed ${seed}: no dive gate`);
    assert.ok(pitch.every((p) => Math.abs(p) <= 30 + 1e-9), `seed ${seed}: a gate is steeper than 30°`);
    assert.equal(gates[0].normal[1], 0, `seed ${seed}: start gate should be level`);
    assert.ok(Math.min(...ys) - GATE_INNER / 2 > 1, `seed ${seed}: a gate opening is too close to the ground`);
  }
});

for (const difficulty of ['easy', 'hard']) {
  test(`racing pilot flies ${difficulty} courses cleanly: every gate, near the centre, on its line`, () => {
    for (const seed of SEEDS) {
      let worstGate = 0, worstTrack = 0, target = 0;
      const sim = run(seed, 70, {
        difficulty,
        onStep(sim) {
          if (sim.t > 5) worstTrack = Math.max(worstTrack, v3.len(v3.sub(sim.plan.pts[sim.track.idx].p, sim.drone.pos)));
          if (sim.state.target !== target) {
            worstGate = Math.max(worstGate, gateOffset(sim.course.gates[target], sim.drone.pos));
            target = sim.state.target;
          }
        },
      });
      assert.ok(sim.state.laps.length >= 2, `seed ${seed}: only ${sim.state.laps.length} laps`);
      assert.equal(sim.state.misses, 0, `seed ${seed}: ${sim.state.misses} missed gates`);
      assert.ok(worstGate < GATE_INNER / 2 - 0.4, `seed ${seed}: passed ${worstGate.toFixed(2)} m off centre`);
      assert.ok(worstTrack < 1.0, `seed ${seed}: strayed ${worstTrack.toFixed(2)} m from the racing line`);
    }
  });
}

test('lap times rise with difficulty', () => {
  const lap = (d) => mean(SEEDS.map((seed) => run(seed, 70, { difficulty: d }).state.laps.slice(1)).map(mean));
  const [easy, normal, hard] = DIFFICULTIES.map(lap);
  assert.ok(easy < normal && normal < hard, `easy ${easy.toFixed(2)}, normal ${normal.toFixed(2)}, hard ${hard.toFixed(2)}`);
  assert.ok(hard < 22.5, `hard laps averaged ${hard.toFixed(2)} s`);
});

test('planned line stays clear of the ground and its speed changes fit the envelope with the turns', () => {
  for (const difficulty of DIFFICULTIES) {
    for (const seed of SEEDS) {
      const plan = planRacingLine(makeCourse({ seed, difficulty }));
      const N = plan.pts.length;
      for (let k = 0; k < N; k++) {
        const q = plan.pts[k];
        assert.ok(q.p[1] > 1.5, `${difficulty} seed ${seed}: line dips to ${q.p[1].toFixed(2)} m`);
        // speed change between samples, as a constant acceleration over ds
        const vn = plan.pts[(k + 1) % N].v;
        const at = (vn * vn - q.v * q.v) / (2 * plan.ds);
        const a = v3.add(v3.add(v3.scale(q.kv, q.v * q.v), v3.scale(q.t, at + DRONE.drag * q.v)), [0, G, 0]);
        const tilt = Math.acos(a[1] / v3.len(a));
        assert.ok(tilt <= DRONE.maxTiltRad + 0.02, `${difficulty} seed ${seed}: needs ${(tilt * 57.3).toFixed(1)}° tilt`);
        assert.ok(v3.len(a) <= DRONE.maxThrustAcc + 0.5, `${difficulty} seed ${seed}: needs ${v3.len(a).toFixed(1)} m/s² thrust`);
      }
    }
  }
});

test('pursuit pilot recovers from a missed gate instead of flying off', () => {
  for (const seed of [...SEEDS, 13, 43, 46]) {
    const sim = run(seed, 150, { pilot: 'pursuit', difficulty: 'hard' });
    assert.ok(sim.state.laps.length >= 3, `seed ${seed}: only ${sim.state.laps.length} laps`);
    assert.ok(v3.len(sim.drone.pos) < 150, `seed ${seed}: drone wandered ${v3.len(sim.drone.pos).toFixed(0)} m away`);
  }
});

test('unknown difficulty falls back to normal', () => {
  assert.equal(makeCourse({ seed: 3, difficulty: 'nope' }).difficulty, 'normal');
  assert.deepEqual(makeCourse({ seed: 3, difficulty: 'nope' }).gates[4].pos, makeCourse({ seed: 3 }).gates[4].pos);
});
