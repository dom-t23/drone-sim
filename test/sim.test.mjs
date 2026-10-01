// Headless checks for the simulation core. Run with: node --test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeSim, step } from '../src/sim.js';

function run(seed, seconds, dt = 1 / 120) {
  const sim = makeSim({ seed });
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) step(sim, dt);
  return sim;
}

test('drone completes laps on several course seeds without missing gates', () => {
  for (const seed of [1, 2, 3, 4, 5, 7, 11, 42]) {
    const sim = run(seed, 150);
    const { laps, misses } = sim.state;
    assert.ok(laps.length >= 2, `seed ${seed}: only ${laps.length} laps`);
    assert.equal(misses, 0, `seed ${seed}: ${misses} missed gates`);
  }
});

test('state stays finite and above the ground', () => {
  const sim = makeSim({ seed: 3 });
  for (let i = 0; i < 120 * 60; i++) {
    step(sim, 1 / 120);
    for (const x of [...sim.drone.pos, ...sim.drone.vel]) assert.ok(Number.isFinite(x));
    assert.ok(sim.drone.pos[1] >= 0.15);
  }
});

test('simulation is deterministic for a given seed', () => {
  const a = run(9, 30);
  const b = run(9, 30);
  assert.deepEqual(a.drone.pos, b.drone.pos);
});
