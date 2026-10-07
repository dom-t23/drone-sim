// Headless checks for the simulation core. Run with: node --test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  makeSim, step, planRacingLine, makeCourse, v3, GATE_INNER, DRONE, G, DIFFICULTIES,
  cameraPose, project, gateCorners, solveGatePose, CAMERA, optimiseLine, SHAPING, WIND,
  MANUAL, headingAxes, CRASH, FRAME, collide, makeScenery,
} from '../src/sim.js';
import { makeRace, syncRace, raceGap } from '../src/race.js';
import { makeSplits, splitsOnEvent } from '../src/splits.js';

const SEEDS = [1, 2, 3, 4, 5, 7, 11, 42];

function run(seed, seconds, { dt = 1 / 120, pilot = 'racing', difficulty = 'normal', nav = 'truth', wind = 'off', observer = true, onStep } = {}) {
  const sim = makeSim({ seed, pilot, difficulty, nav, wind, observer });
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
      assert.equal(sim.state.crashes, 0, `seed ${seed}: ${sim.state.crashes} crashes`);
    }
  });
}

test('racing line is clearly faster than axis pursuit, and the gain is locked in', () => {
  // 2026-10-03: racing line averages ~17.6 s a lap against ~27.3 s for pursuit.
  // 2026-10-04: friction-circle speed profile, ~17.2 s (slowest seed 17.7 s).
  // 2026-10-04 (third run): shaped line (crossing points and tangents), ~16.3 s (slowest 16.7 s).
  for (const seed of SEEDS) {
    const racing = mean(run(seed, 80).state.laps.slice(1));
    const pursuit = mean(run(seed, 120, { pilot: 'pursuit' }).state.laps.slice(1));
    assert.ok(racing < 17.1, `seed ${seed}: racing lap ${racing.toFixed(2)} s`);
    assert.ok(racing < pursuit * 0.75, `seed ${seed}: racing ${racing.toFixed(2)} s vs pursuit ${pursuit.toFixed(2)} s`);
  }
});

test('racing pilot threads each gate where it planned to and stays on its line', () => {
  for (const seed of SEEDS) {
    let worstGate = 0, worstTrack = 0, target = 0;
    run(seed, 60, {
      onStep(sim) {
        if (sim.t > 5) worstTrack = Math.max(worstTrack, v3.len(v3.sub(sim.plan.pts[sim.track.idx].p, sim.drone.pos)));
        if (sim.state.target !== target) {
          const g = sim.course.gates[target];
          const off = v3.sub(sim.drone.pos, sim.plan.cross[target]); // from the planned crossing point
          worstGate = Math.max(worstGate, v3.len(v3.sub(off, v3.scale(g.normal, v3.dot(off, g.normal)))));
          target = sim.state.target;
        }
      },
    });
    assert.ok(worstGate < 0.75, `seed ${seed}: passed ${worstGate.toFixed(2)} m from the planned crossing`);
    assert.ok(worstTrack < 1.2, `seed ${seed}: strayed ${worstTrack.toFixed(2)} m from the racing line`);
  }
});

test('racing line passes through every gate opening and respects the airframe envelope', () => {
  for (const seed of SEEDS) {
    const course = makeCourse({ seed });
    const plan = makeSim({ seed }).plan;
    for (const g of course.gates) {
      const c = plan.cross[g.id], off = v3.sub(c, g.pos);
      assert.ok(Math.abs(v3.dot(off, g.normal)) < 1e-9, 'crossing point lies in the gate plane');
      assert.ok(Math.max(Math.abs(v3.dot(off, g.right)), Math.abs(v3.dot(off, g.up))) <= SHAPING.maxOffset + 1e-9);
      const d = Math.min(...plan.pts.map((q) => v3.len(v3.sub(q.p, c))));
      assert.ok(d < plan.ds, `seed ${seed}: line misses its crossing at gate ${g.id} by ${d.toFixed(2)} m`);
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
      assert.ok(worstGate < GATE_INNER / 2 - 0.35, `seed ${seed}: passed ${worstGate.toFixed(2)} m off centre`);
      assert.ok(worstTrack < 1.0, `seed ${seed}: strayed ${worstTrack.toFixed(2)} m from the racing line`);
    }
  });
}

test('lap times rise with difficulty', () => {
  const lap = (d) => mean(SEEDS.map((seed) => run(seed, 70, { difficulty: d }).state.laps.slice(1)).map(mean));
  const [easy, normal, hard] = DIFFICULTIES.map(lap);
  assert.ok(easy < normal && normal < hard, `easy ${easy.toFixed(2)}, normal ${normal.toFixed(2)}, hard ${hard.toFixed(2)}`);
  assert.ok(hard < 21, `hard laps averaged ${hard.toFixed(2)} s`);
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

// ---------- vision (2026-10-04) ----------

test('onboard camera model: orthonormal axes, tilted up 18°, projects straight ahead to the centre', () => {
  const sim = makeSim({ seed: 2 });
  const cam = cameraPose(sim.drone);
  for (const [a, b] of [[cam.fwd, cam.up], [cam.fwd, cam.right], [cam.up, cam.right]]) assert.ok(Math.abs(v3.dot(a, b)) < 1e-12);
  for (const a of [cam.fwd, cam.up, cam.right]) assert.ok(Math.abs(v3.len(a) - 1) < 1e-12);
  assert.ok(Math.abs(Math.asin(cam.fwd[1]) - CAMERA.tilt) < 1e-9, 'level drone: camera looks 18° up');
  const q = project(cam, v3.add(cam.pos, v3.scale(cam.fwd, 25)));
  assert.ok(Math.abs(q[0]) < 1e-12 && Math.abs(q[1]) < 1e-12);
  assert.equal(project(cam, v3.sub(cam.pos, v3.scale(cam.fwd, 5))), null, 'points behind the camera are not seen');
  // a hard banked turn: the camera rolls with the drone
  sim.drone.thrust = [12, G, 0];
  const banked = cameraPose(sim.drone);
  assert.ok(Math.abs(v3.dot(banked.fwd, banked.up)) < 1e-12 && Math.abs(banked.right[1]) > 0.3);
});

test('PnP recovers a gate from its corners, starting from the wrong map position', () => {
  for (const difficulty of DIFFICULTIES) {
    for (const seed of SEEDS) {
      const sim = makeSim({ seed, difficulty, nav: 'vision' });
      const cam = cameraPose(sim.drone), g = sim.course.gates[0], start = sim.belief.gates[0];
      const uv = gateCorners(g).map((p) => project(cam, p));
      const fit = solveGatePose(cam, uv, start, 0.004);
      const err = v3.len(v3.sub(fit.pos, g.pos));
      // exact apart from the weak orientation prior pulling towards the mapped yaw, which
      // is up to 5° wrong: that costs a centimetre or two at 18 m (a bug would cost metres)
      assert.ok(err < 0.06, `${difficulty} seed ${seed}: noiseless fit is ${err.toFixed(3)} m out`);
      assert.ok(fit.cov[0] > 0 && fit.cov[4] > 0 && fit.cov[8] > 0);
    }
  }
});

// Vision flights are the slowest tests, so each one is flown once and shared.
const flights = new Map();
function visionFlight(seed, difficulty, nav = 'vision') {
  const key = `${seed}/${difficulty}/${nav}`;
  if (!flights.has(key)) {
    let target = 0, worstGate = 0;
    const estErr = [];
    const sim = run(seed, 50, {
      difficulty, nav,
      onStep(sim) {
        if (sim.state.target === target) return;
        const g = sim.course.gates[target];
        worstGate = Math.max(worstGate, gateOffset(g, sim.drone.pos));
        estErr.push(v3.len(v3.sub(sim.belief.gates[target].pos, g.pos)));
        target = sim.state.target;
      },
    });
    flights.set(key, { sim, worstGate, estErr });
  }
  return flights.get(key);
}

test('flying from the map alone misses gates: the map is wrong', () => {
  let seedsWithMisses = 0;
  for (const seed of SEEDS) {
    const sim = run(seed, 50, { nav: 'blind' });
    const err = mean(sim.belief.gates.map((b, i) => v3.len(v3.sub(b.pos, sim.course.gates[i].pos))));
    assert.ok(err > 1, `seed ${seed}: map is only ${err.toFixed(2)} m out on average`);
    if (sim.state.misses > 0) seedsWithMisses++;
  }
  assert.ok(seedsWithMisses >= 6, `only ${seedsWithMisses} of ${SEEDS.length} map-only flights missed a gate`);
});

for (const difficulty of DIFFICULTIES) {
  test(`vision fixes the map in flight: ${difficulty} courses flown cleanly`, () => {
    for (const seed of SEEDS) {
      const { sim, worstGate, estErr } = visionFlight(seed, difficulty);
      assert.equal(sim.state.misses, 0, `seed ${seed}: ${sim.state.misses} missed gates`);
      assert.equal(sim.state.crashes, 0, `seed ${seed}: ${sim.state.crashes} crashes`);
      assert.ok(sim.state.laps.length >= 2, `seed ${seed}: only ${sim.state.laps.length} laps`);
      assert.ok(worstGate < GATE_INNER / 2 - 0.35, `seed ${seed}: passed ${worstGate.toFixed(2)} m off centre`);
      assert.ok(Math.max(...estErr) < 0.5, `seed ${seed}: a gate was passed with its estimate ${Math.max(...estErr).toFixed(2)} m out`);
      assert.ok(mean(estErr) < 0.06, `seed ${seed}: estimates averaged ${mean(estErr).toFixed(3)} m out at the gate`);
      assert.ok(sim.vision.stats.replans > 0);
    }
  });
}

test('vision throws out wild detections and keeps the clean ones', () => {
  const total = { accepted: 0, rejected: 0, wild: 0, wildRejected: 0 };
  for (const seed of SEEDS) {
    for (const k of Object.keys(total)) total[k] += visionFlight(seed, 'normal').sim.vision.stats[k];
  }
  assert.ok(total.wild > 50, 'the detector should produce some wild corners');
  assert.ok(total.wildRejected / total.wild > 0.95, `only ${total.wildRejected}/${total.wild} wild detections rejected`);
  const clean = total.accepted + total.rejected - total.wild;
  const cleanRejected = total.rejected - total.wildRejected;
  assert.ok(cleanRejected / clean < 0.03, `${cleanRejected} of ${clean} clean detections rejected`);
});

test('vision costs almost no lap time once it has seen the course', () => {
  // after lap 1 vision re-shapes the line for the gates it has found
  const ratios = SEEDS.map((seed) => {
    const vision = mean(run(seed, 70, { nav: 'vision' }).state.laps.slice(2));
    const truth = mean(run(seed, 70).state.laps.slice(2));
    assert.ok(vision < truth * 1.05, `seed ${seed}: vision ${vision.toFixed(2)} s vs truth ${truth.toFixed(2)} s`);
    return vision / truth;
  });
  assert.ok(mean(ratios) < 1.025, `vision laps average ${((mean(ratios) - 1) * 100).toFixed(1)}% slower`);
});

test('line shaping: faster plans, crossings inside their limits, start gate centred', () => {
  for (const difficulty of DIFFICULTIES) {
    const gains = SEEDS.map((seed) => {
      const course = makeCourse({ seed, difficulty });
      const shape = optimiseLine(course);
      assert.deepEqual(shape[0], { dr: 0, du: 0, k: 1 });
      for (const sh of shape) {
        assert.ok(Math.abs(sh.dr) <= SHAPING.maxOffset && Math.abs(sh.du) <= SHAPING.maxOffset);
        assert.ok(sh.k >= SHAPING.k[0] && sh.k <= SHAPING.k[1]);
      }
      const shaped = planRacingLine(course, { shape }), plain = planRacingLine(course);
      assert.ok(shaped.minY >= SHAPING.minY - 0.2, `${difficulty} seed ${seed}: shaped line dips to ${shaped.minY.toFixed(2)} m`);
      return 1 - shaped.lapTime / plain.lapTime;
    });
    assert.ok(Math.min(...gains) >= 0, `${difficulty}: shaping made a plan slower`);
    assert.ok(mean(gains) > (difficulty === 'normal' ? 0.045 : 0.025), `${difficulty}: shaping gained only ${(mean(gains) * 100).toFixed(1)}%`);
  }
});

test('vision flights are deterministic, and ground-truth flights carry no vision state', () => {
  const a = run(5, 20, { nav: 'vision', difficulty: 'hard' }), b = run(5, 20, { nav: 'vision', difficulty: 'hard' });
  assert.deepEqual(a.drone.pos, b.drone.pos);
  assert.deepEqual(a.vision.stats, b.vision.stats);
  const t = makeSim({ seed: 5 });
  assert.equal(t.belief, null);
  assert.equal(t.vision, null);
  assert.equal(t.launchAt, 0);
  assert.equal(makeSim({ seed: 5, nav: 'bogus' }).nav, 'truth');
});

// ---------- wind (2026-10-04) ----------

for (const difficulty of DIFFICULTIES) {
  test(`gusty wind: the racing pilot still flies ${difficulty} courses cleanly`, () => {
    for (const seed of SEEDS) {
      let worstGate = 0, target = 0;
      const sim = run(seed, 50, {
        difficulty, wind: 'gusty',
        onStep(sim) {
          if (sim.state.target === target) return;
          worstGate = Math.max(worstGate, gateOffset(sim.course.gates[target], sim.drone.pos));
          target = sim.state.target;
        },
      });
      assert.equal(sim.state.misses, 0, `seed ${seed}: ${sim.state.misses} missed gates`);
      assert.equal(sim.state.crashes, 0, `seed ${seed}: ${sim.state.crashes} crashes`);
      assert.ok(sim.state.laps.length >= 2, `seed ${seed}: only ${sim.state.laps.length} laps`);
      assert.ok(worstGate < GATE_INNER / 2 - 0.2, `seed ${seed}: passed ${worstGate.toFixed(2)} m off centre`);
    }
  });
}

test('the disturbance observer measures the wind and cuts the worst tracking errors', () => {
  let withObs = 0, without = 0;
  for (const seed of SEEDS) {
    const worst = (observer) => {
      let w = 0, est = [0, 0, 0], real = [0, 0, 0], n = 0;
      run(seed, 50, {
        wind: 'gusty', observer,
        onStep(sim) {
          if (sim.t < 5) return;
          w = Math.max(w, v3.len(v3.sub(sim.plan.pts[sim.track.idx].p, sim.drone.pos)));
          est = v3.add(est, v3.scale(sim.dist, 1 / DRONE.drag));
          real = v3.add(real, sim.wind.now);
          n++;
        },
      });
      // averaged over the flight, the estimate matches the wind actually flown through
      const err = v3.len(v3.sub(est, real)) / n;
      assert.ok(err < 0.5, `seed ${seed}: average wind estimate ${err.toFixed(2)} m/s out`);
      return w;
    };
    withObs += worst(true);
    without += worst(false);
  }
  assert.ok(withObs < without * 0.85, `worst tracking errors: ${withObs.toFixed(2)} m with the observer vs ${without.toFixed(2)} m without`);
});

test('gusts stay within two sigma of the forecast, and calm runs carry no wind', () => {
  const sim = makeSim({ seed: 9, wind: 'gusty' });
  for (let i = 0; i < 120 * 60; i++) {
    step(sim, 1 / 120);
    assert.ok(Math.hypot(sim.wind.gust[0], sim.wind.gust[2]) <= 2 * WIND.gusty.gust + 1e-9);
    assert.ok(Math.abs(sim.wind.gust[1]) <= 2 * WIND.gusty.gust * WIND.vertical + 1e-9);
  }
  assert.equal(makeSim({ seed: 9 }).wind, null);
  assert.equal(makeSim({ seed: 9, wind: 'hurricane' }).wind, null);
});

// ---------- splits (2026-10-04) ----------

test('gate splits compare each gate with the best lap so far', () => {
  const S = makeSplits();
  const feed = (gate, t) => splitsOnEvent(S, { type: 'gate', gate, t });
  feed(0, 2); feed(1, 5); feed(2, 9); feed(0, 14); // lap 1: 12 s
  assert.equal(S.bestLap, 12);
  assert.deepEqual(S.rows.at(-1), { label: 'Lap', time: 12, delta: null });
  feed(1, 16.5); // 2.5 s into lap 2 vs 3 s on the best lap
  assert.ok(Math.abs(S.rows.at(-1).delta + 0.5) < 1e-9);
  feed(2, 21.5); feed(0, 26.5); // lap 2: 12.5 s, slower
  assert.ok(Math.abs(S.rows.at(-1).delta - 0.5) < 1e-9);
  assert.equal(S.bestLap, 12, 'a slower lap does not replace the best');
  splitsOnEvent(S, { type: 'miss', gate: 1, t: 30 });
  assert.equal(S.rows.length, 5, 'only the last five rows are kept; misses are ignored');
});

test('splits from a real flight: one row per gate, deltas once a lap is done', () => {
  const S = makeSplits(), sim = run(3, 40);
  sim.events.forEach((e) => splitsOnEvent(S, e, 100));
  const gates = sim.events.filter((e) => e.type === 'gate').length;
  assert.equal(S.rows.length, gates - 1, 'every gate after the first start line gives a row');
  assert.ok(Math.abs(S.bestLap - Math.min(...sim.state.laps)) < 1e-9);
  assert.ok(S.rows.slice(10).every((r) => r.delta !== null));
});

// ---------- manual flight ----------
// A stand-in for a human: looks at the next gate, turns towards a point on its axis and
// eases off the throttle while the nose is off target. It only touches the sticks.
function botSticks(sim, cruise = 0.5) {
  const d = sim.drone, gate = sim.course.gates[sim.state.target];
  const rel = v3.sub(d.pos, gate.pos), along = v3.dot(rel, gate.normal);
  const lat = v3.len(v3.sub(rel, v3.scale(gate.normal, along)));
  const aim = along > 1 || (along > -1 && lat > GATE_INNER / 2)
    ? v3.sub(gate.pos, v3.scale(gate.normal, 10))
    : v3.add(gate.pos, v3.scale(gate.normal, Math.min(along + 7, 6)));
  let err = Math.atan2(aim[0] - d.pos[0], aim[2] - d.pos[2]) - d.yaw;
  err = Math.atan2(Math.sin(err), Math.cos(err));
  return { x: Math.max(-1, Math.min(1, -2.5 * err)), y: cruise * Math.max(0.15, 1 - 1.2 * Math.abs(err)), z: 0, s: 0 };
}
const tiltDeg = (sim) => (Math.acos(v3.norm(sim.drone.thrust)[1]) * 180) / Math.PI;

test('manual: centred sticks hover in place, and altitude assist climbs to the next gate', () => {
  const sim = makeSim({ seed: 3, pilot: 'manual' });
  sim.assist = false;
  const start = sim.drone.pos.slice();
  for (let i = 0; i < 6 * 120; i++) step(sim, 1 / 120);
  assert.ok(v3.len(v3.sub(sim.drone.pos, start)) < 0.3, `drifted ${v3.len(v3.sub(sim.drone.pos, start)).toFixed(2)} m`);
  sim.assist = true;
  for (let i = 0; i < 8 * 120; i++) step(sim, 1 / 120);
  const gy = sim.course.gates[0].pos[1];
  assert.ok(Math.abs(sim.drone.pos[1] - gy) < 0.5, `at ${sim.drone.pos[1].toFixed(2)} m, gate at ${gy.toFixed(2)} m`);
  assert.ok(Math.hypot(sim.drone.pos[0] - start[0], sim.drone.pos[2] - start[2]) < 0.3, 'wandered sideways while climbing');
});

test('manual: full stick reaches top speed along the nose, and turns follow the nose within the envelope', () => {
  const sim = makeSim({ seed: 3, pilot: 'manual' });
  sim.stick = { x: 0, y: 1, z: 0, s: 0 };
  for (let i = 0; i < 10 * 120; i++) step(sim, 1 / 120);
  const { fwd } = headingAxes(sim.drone.yaw);
  const speed = v3.len(sim.drone.vel);
  assert.ok(Math.abs(speed - MANUAL.vMax) < 1, `speed ${speed.toFixed(1)} m/s`);
  assert.ok(v3.dot(v3.norm(sim.drone.vel), fwd) > 0.99, 'not flying along the nose');
  const yaw0 = sim.drone.yaw;
  sim.stick = { x: 1, y: 1, z: 0, s: 0 };
  let worstSlip = 0, worstTilt = 0;
  for (let i = 0; i < 6 * 120; i++) {
    step(sim, 1 / 120);
    worstTilt = Math.max(worstTilt, tiltDeg(sim));
    if (i > 120) {
      const f = headingAxes(sim.drone.yaw).fwd, v = v3.norm([sim.drone.vel[0], 0, sim.drone.vel[2]]);
      worstSlip = Math.max(worstSlip, (Math.acos(Math.min(1, v3.dot(f, v))) * 180) / Math.PI);
    }
  }
  assert.ok(sim.drone.yaw < yaw0 - 2, 'full right stick should turn right (yaw decreasing)');
  assert.ok(worstTilt <= (DRONE.maxTiltRad * 180) / Math.PI + 1e-6, `tilt ${worstTilt.toFixed(1)}°`);
  assert.ok(worstSlip < 12, `velocity lags the nose by ${worstSlip.toFixed(1)}°`);
});

test('manual: full descend stops at the floor, and nothing moves before the start countdown ends', () => {
  // collisions off: on seed 1 this low, straight run meets the first gate's bottom bar
  const sim = makeSim({ seed: 1, pilot: 'manual', launchAt: 3, crashes: false });
  const start = sim.drone.pos.slice();
  sim.stick = { x: 1, y: 1, z: 1, s: 1 };
  for (let i = 0; i < 2.9 * 120; i++) step(sim, 1 / 120);
  assert.ok(v3.len(v3.sub(sim.drone.pos, start)) < 1e-9, 'moved before the countdown ended');
  sim.stick = { x: 0, y: 0.3, z: -1, s: 0 };
  let lowest = Infinity;
  for (let i = 0; i < 8 * 120; i++) {
    step(sim, 1 / 120);
    lowest = Math.min(lowest, sim.drone.pos[1]);
  }
  assert.ok(lowest > MANUAL.floor - 0.25, `sank to ${lowest.toFixed(2)} m`);
});

for (const difficulty of ['easy', 'normal']) {
  test(`manual: a stick-only pilot can fly ${difficulty} courses without missing gates`, () => {
    for (const seed of SEEDS) {
      const sim = makeSim({ seed, pilot: 'manual', difficulty, optimise: false });
      for (let i = 0; i < 100 * 120; i++) {
        sim.stick = botSticks(sim);
        step(sim, 1 / 120);
        assert.ok(tiltDeg(sim) <= (DRONE.maxTiltRad * 180) / Math.PI + 1e-6);
      }
      assert.ok(sim.state.laps.length >= 2, `seed ${seed}: only ${sim.state.laps.length} laps`);
      assert.equal(sim.state.misses, 0, `seed ${seed}: ${sim.state.misses} missed gates`);
      assert.equal(sim.state.crashes, 0, `seed ${seed}: ${sim.state.crashes} crashes`);
    }
  });
}

test('race: gap at the last gate both passed, and who is leading', () => {
  const ev = (ts) => ({ events: ts.map((t, i) => ({ t, type: i === 1 ? 'lap' : 'gate' })) });
  const race = makeRace();
  assert.equal(raceGap(race).gap, null);
  syncRace(race, ev([1, 0, 3]), ev([0.5]));
  // you: gates at 1 and 3 (the 'lap' event is skipped); rival: gate at 0.5
  assert.deepEqual(race.you, [1, 3]);
  let g = raceGap(race);
  assert.equal(g.gap, 0.5);
  assert.equal(g.leading, true);
  const r2 = makeRace();
  syncRace(r2, ev([2]), ev([1, 9, 2.5]));
  g = raceGap(r2);
  assert.equal(g.gap, 1);
  assert.equal(g.leading, false);
  // syncing again adds nothing new
  syncRace(r2, ev([2]), ev([1, 9, 2.5]));
  assert.deepEqual(r2.rival, [1, 2.5]);
});

test('race: a cautious stick pilot trails the racing-line rival, which keeps lapping cleanly', () => {
  const you = makeSim({ seed: 2, pilot: 'manual', launchAt: 3, optimise: false });
  const rival = makeSim({ seed: 2, pilot: 'racing', launchAt: 3 });
  const race = makeRace();
  for (let i = 0; i < 60 * 120; i++) {
    you.stick = botSticks(you);
    step(you, 1 / 120);
    step(rival, 1 / 120);
    syncRace(race, you, rival);
  }
  const g = raceGap(race);
  assert.equal(g.leading, false);
  assert.ok(g.gap > 5, `gap ${g.gap?.toFixed(2)} s`);
  assert.ok(rival.state.laps.length >= 2 && rival.state.misses === 0);
  // the countdown delays the rival's first gate by the launch time, nothing more
  assert.ok(race.rival[0] > 3);
});

// ---------- crashes (2026-10-07) ----------

const S = GATE_INNER, T = FRAME.bar;
// a point on a gate in its own right / up / normal axes
const onGate = (g, r, u, n = 0) => v3.add(v3.add(v3.add(g.pos, v3.scale(g.right, r)), v3.scale(g.up, u)), v3.scale(g.normal, n));

test('collision shapes: the opening is clear, frame bars, legs and pillars are solid', () => {
  for (const difficulty of DIFFICULTIES) {
    const sim = makeSim({ seed: 4, difficulty, pilot: 'pursuit' });
    for (const g of sim.course.gates) {
      assert.equal(collide(sim, g.pos), null, 'gate centre is clear');
      // anywhere a drone fits through the opening is clear
      const edge = S / 2 - CRASH.radius - 0.01;
      for (const [r, u] of [[edge, 0], [-edge, 0], [0, edge], [0, -edge], [edge * 0.7, edge * 0.7]]) {
        assert.equal(collide(sim, onGate(g, r, u)), null, `gate ${g.id} (${r.toFixed(2)}, ${u.toFixed(2)}) should be clear`);
      }
      // the middle of each bar, from just in front: hit, pushed back towards the front
      for (const [r, u] of [[0, (S + T) / 2], [0, -(S + T) / 2], [(S + T) / 2, 0], [-(S + T) / 2, 0]]) {
        const hit = collide(sim, onGate(g, r, u, -(T / 2 + CRASH.radius - 0.05)));
        assert.ok(hit && hit.what === 'gate' && hit.gate === g.id, `gate ${g.id}: bar at (${r}, ${u}) not solid`);
        assert.ok(v3.dot(hit.n, g.normal) < -0.99, 'pushed out of the front face');
        assert.ok(Math.abs(hit.depth - 0.05) < 1e-9);
      }
      // a leg, half way down, from the side
      const foot = v3.sub(v3.add(g.pos, v3.scale(g.right, (S + T) / 2)), v3.scale(g.up, S / 2 + T));
      if (foot[1] > 1) {
        const p = [foot[0] + 0.2, foot[1] / 2, foot[2]];
        const hit = collide(sim, p);
        assert.ok(hit && hit.what === 'gate', `gate ${g.id}: leg not solid`);
        assert.ok(hit.n[0] > 0.99, 'pushed off the leg sideways');
      }
    }
    for (const q of sim.scenery.pillars.slice(0, 10)) {
      assert.equal(collide(sim, [q.pos[0], q.size[1] + CRASH.radius + 0.01, q.pos[2]]), null, 'clear above a pillar');
      const hit = collide(sim, [q.pos[0], q.size[1] + CRASH.radius - 0.1, q.pos[2]]);
      assert.ok(hit?.what === 'pillar' && hit.n[1] > 0.99, 'pillar tops are solid');
      const side = collide(sim, [q.pos[0] + q.size[0] * 0.5 * Math.cos(q.yaw), 1, q.pos[2] - q.size[0] * 0.5 * Math.sin(q.yaw)]);
      assert.ok(side?.what === 'pillar', 'pillar sides are solid');
    }
  }
});

test('scenery: same pillars every time, none within reach of any racing line', () => {
  for (const difficulty of DIFFICULTIES) {
    for (const seed of SEEDS) {
      const sim = makeSim({ seed, difficulty });
      assert.deepEqual(makeScenery(sim.course).pillars, sim.scenery.pillars);
      assert.ok(sim.scenery.pillars.length > 45, `${difficulty} ${seed}: only ${sim.scenery.pillars.length} pillars`);
      let closest = Infinity;
      for (const q of sim.scenery.pillars) {
        for (const pt of sim.plan.pts) {
          closest = Math.min(closest, Math.hypot(pt.p[0] - q.pos[0], pt.p[2] - q.pos[2]) - q.size[0] * Math.SQRT1_2);
        }
      }
      assert.ok(closest > 4, `${difficulty} ${seed}: racing line passes ${closest.toFixed(1)} m from a pillar`);
    }
  }
});

// Put a manual drone just in front of a gate's top bar, nose on, and floor it.
function ramTopBar(sim, gate) {
  sim.assist = false;
  sim.drone.pos = onGate(gate, 0, (S + T) / 2, -8);
  sim.drone.vel = [0, 0, 0];
  sim.drone.yaw = Math.atan2(gate.normal[0], gate.normal[2]);
  sim.stick = { x: 0, y: 1, z: 0, s: 0 };
}

test('crash: a head-on hit tumbles to the ground, then respawns at the start and hovers', () => {
  const sim = makeSim({ seed: 2, pilot: 'manual', optimise: false });
  ramTopBar(sim, sim.course.gates[0]);
  let crashedAt = null, high = 0, low = Infinity;
  for (let i = 0; i < 3 * 120 && !crashedAt; i++) {
    step(sim, 1 / 120);
    if (sim.crash) crashedAt = sim.t;
  }
  assert.ok(crashedAt, 'never hit the bar');
  assert.equal(sim.state.crashes, 1);
  const e = sim.events.find((x) => x.type === 'crash');
  assert.ok(e.what === 'gate' && e.gate === 0 && e.speed > 5, JSON.stringify(e));
  assert.ok(sim.drone.tumble.rate > 4);
  high = sim.drone.pos[1];
  while (sim.crash) {
    step(sim, 1 / 120);
    low = Math.min(low, sim.drone.pos[1]);
    assert.ok(sim.state.gatesPassed === 0, 'a tumble through a gate does not count');
  }
  assert.ok(sim.t - crashedAt >= CRASH.tumble - 1e-6);
  assert.ok(low < 0.2 && high > 2, `fell from ${high.toFixed(1)} m to ${low.toFixed(2)} m`);
  assert.equal(sim.events.at(-1).type, 'respawn');
  assert.deepEqual(sim.drone.pos, sim.start.pos);
  assert.equal(sim.drone.tumble, undefined);
  // it hovers for a moment before the sticks come back
  const at = sim.drone.pos.slice();
  for (let i = 0; i < (CRASH.hold - 0.05) * 120; i++) step(sim, 1 / 120);
  assert.ok(v3.len(v3.sub(sim.drone.pos, at)) < 0.05, 'moved during the respawn hold');
  for (let i = 0; i < 120; i++) step(sim, 1 / 120);
  assert.ok(v3.len(sim.drone.vel) > 5, 'flies again after the hold');
});

test('crash: you respawn just past the last gate you passed, never further on', () => {
  const sim = makeSim({ seed: 3, pilot: 'manual', optimise: false });
  for (let i = 0; i < 40 * 120 && sim.state.gatesPassed < 3; i++) {
    sim.stick = botSticks(sim);
    step(sim, 1 / 120);
  }
  assert.equal(sim.state.gatesPassed, 3);
  ramTopBar(sim, sim.course.gates[5]); // a gate well ahead of the next one
  for (let i = 0; i < 6 * 120 && sim.events.at(-1).type !== 'respawn'; i++) step(sim, 1 / 120);
  assert.equal(sim.events.at(-1).type, 'respawn');
  assert.equal(sim.state.target, 3);
  const { plan } = sim, N = plan.pts.length;
  const q = plan.pts[Math.round((plan.gateS[2] + CRASH.after) / plan.ds) % N];
  assert.ok(Math.hypot(sim.drone.pos[0] - q.p[0], sim.drone.pos[2] - q.p[2]) < 1e-9);
  // and it can carry on and finish the lap from there
  sim.assist = true;
  for (let i = 0; i < 40 * 120 && sim.state.laps.length < 1; i++) {
    sim.stick = botSticks(sim);
    step(sim, 1 / 120);
  }
  assert.equal(sim.state.laps.length, 1);
});

test('crash: a gentle touch slides along instead of crashing', () => {
  const sim = makeSim({ seed: 5, pilot: 'manual', optimise: false });
  const g = sim.course.gates.reduce((a, b) => (b.pos[1] > a.pos[1] ? b : a));
  const barBottom = onGate(g, 0, -(S / 2 + T));
  assert.ok(barBottom[1] > 3, 'need a high gate');
  sim.drone.pos = [barBottom[0], barBottom[1] - 1.5, barBottom[2]];
  sim.stick = { x: 0, y: 0, z: 0.15, s: 0 }; // climb at about 1 m/s into the bottom bar
  for (let i = 0; i < 4 * 120; i++) step(sim, 1 / 120);
  assert.equal(sim.state.crashes, 0);
  assert.equal(sim.crash, null);
  assert.ok(Math.abs(sim.drone.pos[1] - (barBottom[1] - CRASH.radius)) < 0.05, `pressed up at ${sim.drone.pos[1].toFixed(2)} m`);
});

test('crashes: map-only flies through frames, and crash flights are deterministic', () => {
  assert.equal(makeSim({ seed: 1, nav: 'blind' }).crashes, false);
  assert.equal(makeSim({ seed: 1, nav: 'vision' }).crashes, true);
  const fly = () => {
    const sim = makeSim({ seed: 2, pilot: 'manual', optimise: false });
    ramTopBar(sim, sim.course.gates[0]);
    for (let i = 0; i < 5 * 120; i++) step(sim, 1 / 120);
    return sim;
  };
  const a = fly(), b = fly();
  assert.equal(a.state.crashes, 1);
  assert.deepEqual(a.drone.pos, b.drone.pos);
});
