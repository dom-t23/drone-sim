// Pro courses and the split-S (2026-10-29): an out-and-back course whose lap ends with a
// two-storey gate tower. The drone goes through the top gate, rolls inverted, pulls
// through and comes back through the gate beneath, heading the other way.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  makeSim, step, makeCourse, planRacingLine, v3, bodyAxes, limitThrust, GATE_INNER, DRONE, G,
  DIFFICULTIES, PRO, SPLIT_S,
} from '../src/sim.js';
import { GHOST, makeGhostRecorder, recordGhost, ghostPose } from '../src/ghost.js';
import { TV, makeReplay, recordReplay, replayPose } from '../src/director.js';

const SEEDS = [1, 2, 3, 4, 5, 7, 11, 42];
const DT = 1 / 120;
const tiltOf = (t) => Math.acos(Math.max(-1, Math.min(1, t[1] / (v3.len(t) || 1))));
const axesJump = (a, b) => Math.max(v3.len(v3.sub(a.bx, b.bx)), v3.len(v3.sub(a.by, b.by)), v3.len(v3.sub(a.bz, b.bz)));

// One flight, watched step by step: the split-S stats and anything that breaks the rules.
const cache = new Map();
function proFlight(seed, { pilot = 'racing', nav = 'truth', seconds = 50 } = {}) {
  const key = `${seed}/${pilot}/${nav}/${seconds}`;
  if (cache.has(key)) return cache.get(key);
  const sim = makeSim({ seed, difficulty: 'pro', pilot, nav });
  const [top] = sim.course.splitS, n = sim.course.gates[top].normal;
  const r = { sim, worstTiltOutside: 0, maxJump: 0, minUp: 1, minThrustY: Infinity, exits: [] };
  let prev = bodyAxes(sim.drone), wasFlip = false;
  for (let i = 0, N = Math.round(seconds / DT); i < N; i++) {
    step(sim, DT);
    const d = sim.drone, acro = !!sim.plan.pts[sim.track.idx].acro && sim.pilot === 'racing';
    const B = bodyAxes(d);
    if (!sim.crash) {
      if (!acro && sim.t > sim.launchAt) r.worstTiltOutside = Math.max(r.worstTiltOutside, tiltOf(d.thrust));
      if (d.flip || wasFlip) r.maxJump = Math.max(r.maxJump, axesJump(B, prev)); // through the split-S
      r.minUp = Math.min(r.minUp, B.by[1]);
      if (acro) r.minThrustY = Math.min(r.minThrustY, d.thrust[1]);
    }
    // just out of the split-S: upright, nose pointing back down the course
    if (wasFlip && !d.flip) r.exits.push({ up: B.by[1], nose: v3.dot(B.bz, n) });
    wasFlip = !!d.flip;
    prev = B;
  }
  cache.set(key, r);
  return r;
}

test('pro: a new difficulty, with the old ones (and their seeds) untouched', () => {
  assert.deepEqual(DIFFICULTIES, ['easy', 'normal', 'hard', 'pro']);
  for (const d of ['easy', 'normal', 'hard']) assert.equal(makeCourse({ seed: 3, difficulty: d }).splitS, undefined);
  assert.deepEqual(makeCourse({ seed: 5, difficulty: 'pro' }).gates, makeCourse({ seed: 5, difficulty: 'pro' }).gates);
});

test('pro: every lap ends with a split-S tower, two gates stacked and facing opposite ways', () => {
  for (let seed = 1; seed <= 40; seed++) {
    const { gates, splitS } = makeCourse({ seed, difficulty: 'pro' }), n = gates.length, tag = `seed ${seed}`;
    assert.deepEqual(splitS, [n - 2, n - 1], tag);
    const [top, bot] = splitS.map((i) => gates[i]);
    assert.ok(Math.hypot(top.pos[0] - bot.pos[0], top.pos[2] - bot.pos[2]) < 1e-9, `${tag}: stacked`);
    assert.ok(top.pos[1] - bot.pos[1] >= PRO.stack[0] - PRO.stack[1] - 0.4, `${tag}: a full storey apart`);
    assert.ok(top.pos[1] - bot.pos[1] > GATE_INNER + 2 * 0.28 + 2, `${tag}: frames well clear of each other`);
    assert.ok(v3.dot(top.normal, bot.normal) < -0.999 && Math.abs(top.normal[1]) < 1e-9, `${tag}: level, facing opposite ways`);
    assert.equal(gates[0].normal[1], 0, `${tag}: start gate level`);
    assert.ok(Math.min(...gates.map((g) => g.pos[1])) - GATE_INNER / 2 > 1, `${tag}: an opening too near the ground`);
    assert.ok(gates.every((g) => Math.abs(Math.asin(g.normal[1])) <= 30 * Math.PI / 180 + 1e-9), `${tag}: a gate steeper than 30°`);
    // gates are spread out along the course (only the tower's pair share a spot)
    for (let i = 0; i < n - 2; i++) {
      const d = v3.len(v3.sub(gates[i + 1].pos, gates[i].pos));
      assert.ok(d > 10, `${tag}: gates ${i + 1} and ${i + 2} only ${d.toFixed(1)} m apart`);
    }
  }
});

test('pro: acro only inside the split-S; everywhere else the 55° tilt limit still holds', () => {
  assert.ok(v3.len(limitThrust([20, -15, 0], true)) <= DRONE.maxThrustAcc + 1e-9);
  assert.ok(limitThrust([20, -15, 0], true)[1] < 0, 'acro: may thrust downwards');
  assert.ok(tiltOf(limitThrust([20, -15, 0])) <= DRONE.maxTiltRad + 1e-9, 'not acro: tilt limited');
  for (const d of ['easy', 'normal', 'hard']) assert.ok(!makeSim({ seed: 2, difficulty: d }).plan.pts.some((q) => q.acro), d);
  for (const seed of SEEDS) {
    const plan = makeSim({ seed, difficulty: 'pro' }).plan, [a, b] = makeCourse({ seed, difficulty: 'pro' }).splitS;
    const acro = plan.pts.filter((q) => q.acro);
    assert.ok(acro.length > 20, `seed ${seed}`);
    assert.ok(acro.every((q) => q.s > plan.gateS[a] && q.s < plan.gateS[b]), `seed ${seed}: acro outside the tower`);
    // the plan really asks for inverted flight: thrust pointing down over the top
    let downs = 0;
    for (const q of acro) {
      const thrust = v3.add(v3.scale(q.kv, q.v * q.v), [0, G, 0]);
      if (thrust[1] < -3) downs++;
      assert.ok(v3.len(thrust) <= DRONE.maxThrustAcc + 0.5, `seed ${seed}: needs ${v3.len(thrust).toFixed(1)} m/s²`);
    }
    assert.ok(downs > 3, `seed ${seed}: the plan never flies inverted`);
    const r = proFlight(seed);
    assert.ok(r.worstTiltOutside <= DRONE.maxTiltRad + 1e-6, `seed ${seed}: ${(r.worstTiltOutside * 57.3).toFixed(1)}° tilt outside the split-S`);
  }
});

test('pro: the racing pilot flies split-S laps cleanly, really inverted, and comes out heading back', () => {
  for (const seed of SEEDS) {
    const r = proFlight(seed), s = r.sim.state, tag = `seed ${seed}`;
    assert.equal(s.misses, 0, `${tag}: ${s.misses} missed`);
    assert.equal(s.crashes, 0, `${tag}: ${s.crashes} crashed`);
    assert.ok(s.laps.length >= 2, `${tag}: only ${s.laps.length} laps`);
    const splits = r.sim.events.filter((e) => e.type === 'splits');
    assert.ok(splits.length >= s.laps.length, `${tag}: ${splits.length} split-S for ${s.laps.length} laps`);
    for (const e of splits) {
      assert.ok(e.inverted > 0.2 && e.inverted < 1, `${tag}: upside down for ${e.inverted.toFixed(2)} s`);
      assert.ok(e.t - e.from < 2.5 && e.vIn > 4, `${tag}: a ${(e.t - e.from).toFixed(2)} s split-S, in at ${e.vIn.toFixed(1)} m/s`);
    }
    assert.ok(r.minUp < -0.95, `${tag}: never fully inverted (${r.minUp.toFixed(2)})`);
    assert.ok(r.minThrustY < -G, `${tag}: thrust never pointed firmly down`);
    // through the split-S the attitude moves smoothly: no axis jumps more than the roll rate
    // allows in a step
    assert.ok(r.maxJump < 2 * Math.sin(SPLIT_S.rollRate * DT) + 0.12, `${tag}: body axes jumped ${r.maxJump.toFixed(2)} in one step`);
    for (const x of r.exits) assert.ok(x.up > 0.75 && x.nose < -0.6, `${tag}: came out at up ${x.up.toFixed(2)}, nose ${x.nose.toFixed(2)}`);
    assert.ok(s.laps.slice(1).every((t) => t < 16.5), `${tag}: laps ${s.laps.map((t) => t.toFixed(2))}`);
  }
});

test('pro: vision, vision + IMU and the old pursuit pilot get round too', () => {
  for (const seed of [1, 2, 4, 7]) {
    for (const nav of ['vision', 'vio']) {
      const s = proFlight(seed, { nav }).sim.state;
      assert.equal(s.misses + s.crashes, 0, `${nav} seed ${seed}: ${s.misses} missed, ${s.crashes} crashed`);
      assert.ok(s.laps.length >= 2);
    }
    // pursuit can't roll inverted: it stalls over the top and drops through, slowly
    const p = proFlight(seed, { pilot: 'pursuit', seconds: 90 });
    assert.equal(p.sim.state.misses + p.sim.state.crashes, 0, `pursuit seed ${seed}`);
    assert.ok(p.sim.state.laps.length >= 2 && p.minUp > 0, `pursuit seed ${seed}`);
  }
});

test('pro: a recorded split-S replays smoothly (ghost and TV replay)', () => {
  const sim = makeSim({ seed: 3, difficulty: 'pro' });
  const rec = makeGhostRecorder(), rep = makeReplay(), entries = [{ id: 'hero', sim }];
  // fly a couple of laps, and stop soon after a split-S so it's still in the replay buffer
  for (let i = 0, stop = Infinity; i < stop; i++) {
    step(sim, DT);
    if (stop === Infinity && sim.t > 25 && sim.events.at(-1).type === 'splits') stop = i + 120;
    recordGhost(rec, sim);
    recordReplay(rep, sim.t, entries);
  }
  const ghost = rec.best;
  assert.ok(ghost, 'a best lap was kept');
  const flip = sim.events.filter((e) => e.type === 'splits').at(-1);
  // ghost: through the lap's split-S (its last second or so), at 120 Hz
  let prev = null, worst = 0, inverted = false;
  for (let t = ghost.time - 4; t < ghost.time; t += DT) {
    const p = ghostPose(ghost, t);
    if (!p) continue;
    const B = bodyAxes(p);
    if (B.by[1] < -0.8) inverted = true;
    if (prev) worst = Math.max(worst, axesJump(B, prev));
    prev = B;
  }
  assert.ok(inverted, 'the ghost rolls inverted too');
  // (no more than it turns between two of the ghost's samples: it doesn't sweep the long way)
  const frameTurn = (rate) => 2 * Math.sin(SPLIT_S.rollRate / rate / 2) + 0.15;
  assert.ok(worst < frameTurn(GHOST.rate), `ghost axes jumped ${worst.toFixed(2)}`);
  // TV replay, slowed down, over the last split-S
  prev = null; worst = 0; inverted = false;
  for (let t = flip.from - 0.5; t < flip.t + 0.5; t += DT / 3) {
    const B = bodyAxes(replayPose(rep, 'hero', t));
    if (B.by[1] < -0.8) inverted = true;
    if (prev) worst = Math.max(worst, axesJump(B, prev));
    prev = B;
  }
  assert.ok(inverted, 'the replay rolls inverted too');
  assert.ok(worst < frameTurn(TV.replay.rate), `replay axes jumped ${worst.toFixed(2)}`);
});

test('pro: a crash in the middle of the split-S respawns upright', () => {
  const sim = makeSim({ seed: 2, difficulty: 'pro' });
  while (!sim.drone.flip || sim.drone.flip.phase !== 'pull') step(sim, DT);
  // knocked into the bottom gate's top bar while upside down
  const g = sim.course.gates[sim.course.splitS[1]], bar = v3.add(g.pos, v3.scale(g.up, (GATE_INNER + 0.28) / 2));
  sim.drone.pos = v3.sub(bar, v3.scale(g.normal, 0.6));
  sim.drone.vel = v3.scale(g.normal, 8);
  for (let i = 0; i < 30 && !sim.crash; i++) step(sim, DT);
  assert.ok(sim.crash, 'crashed');
  while (sim.crash) step(sim, DT);
  assert.ok(sim.state.crashes >= 1);
  assert.equal(sim.drone.flip, undefined);
  assert.equal(sim.drone.up, undefined);
  // back in front of the bottom gate, upright and facing it, and it flies on through
  const g2 = sim.course.gates[sim.course.splitS[1]];
  assert.ok(bodyAxes(sim.drone).by[1] > 0.95);
  assert.ok(v3.dot(bodyAxes(sim.drone).bz, g2.normal) > 0.95);
  const passed = sim.state.gatesPassed;
  for (let i = 0; i < 30 * 120; i++) step(sim, DT);
  assert.ok(sim.state.gatesPassed > passed + 3 && sim.state.misses === 0, 'through the bottom gate and on');
  assert.ok(sim.state.laps.length >= 1, 'and races on');
});

test('pro: the planner keeps the line clear of the ground through the dive', () => {
  for (const seed of SEEDS) {
    const plan = planRacingLine(makeCourse({ seed, difficulty: 'pro' }));
    assert.ok(plan.minY > 3, `seed ${seed}: line dips to ${plan.minY.toFixed(2)} m`);
  }
});
