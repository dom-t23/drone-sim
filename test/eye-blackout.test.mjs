// Camera blackouts on the pixel camera (vision step 3): the caution built for the corner
// model has to hold up when the camera sees less far and finds gates in real pixels.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeSim, step, v3, cameraBlackout } from '../src/sim.js';

for (const nav of ['vision', 'vio']) {
  test(`pixels + blackout (${nav}): on the first lap it stops short of gates it hasn't seen, finds them in the pixels and races on`, () => {
    let holds = 0, resumes = 0;
    for (const difficulty of ['normal', 'hard']) {
      for (const seed of [1, 42]) {
        for (const at of [1, 4, 7]) {
          const sim = makeSim({ seed, difficulty, nav, eyes: 'pixels' }), tag = `${difficulty} seed ${seed} at ${at}`;
          let t0 = null, holdGap = Infinity;
          for (let i = 0; i < 120 * 34; i++) {
            step(sim, 1 / 120);
            if (t0 === null && sim.state.gatesPassed >= at) { t0 = sim.t; assert.ok(cameraBlackout(sim)); }
            if (sim.vision.held !== null) {
              const g = sim.course.gates[sim.vision.held];
              holdGap = Math.min(holdGap, -v3.dot(v3.sub(sim.drone.pos, g.pos), g.normal));
            }
          }
          const s = sim.state, events = sim.events.filter((e) => e.t >= t0).map((e) => e.type);
          assert.equal(s.misses, 0, `${tag}: missed a gate`);
          assert.equal(s.crashes, 0, `${tag}: crashed`);
          assert.ok(s.laps.length >= 1, `${tag}: no lap`);
          if (events.includes('hold')) {
            holds++;
            assert.ok(holdGap > 4, `${tag}: held ${holdGap.toFixed(1)} m from the gate`);
          }
          if (events.includes('resume')) resumes++;
        }
      }
    }
    assert.ok(holds >= 10 && resumes >= 10, `${holds} holds and ${resumes} resumes out of 12`);
  });
}
