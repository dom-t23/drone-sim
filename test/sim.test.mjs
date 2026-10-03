// Headless checks for the simulation core. Run with: node --test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeSim, step, planRacingLine, makeCourse, v3, GATE_INNER, DRONE, G } from '../src/sim.js';

const SEEDS = [1, 2, 3, 4, 5, 7, 11, 42];

function run(seed, seconds, { dt = 1 / 120, pilot = 'racing', onStep } = {}) {
  const sim = makeSim({ seed, pilot });
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
  for (const seed of SEEDS) {
    const racing = mean(run(seed, 80).state.laps.slice(1));
    const pursuit = mean(run(seed, 120, { pilot: 'pursuit' }).state.laps.slice(1));
    assert.ok(racing < 18.6, `seed ${seed}: racing lap ${racing.toFixed(2)} s`);
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
