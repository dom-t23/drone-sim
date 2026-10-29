// TV mode: trackside camera posts, the race director and the replay buffer.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeSim, step, shareTraffic, bumpAll, GATE_INNER, FRAME, v3 } from '../src/sim.js';
import { buildScene } from '../src/eye.js';
import { rosterFor } from '../src/field.js';
import {
  TV, blocked, clearance, placePosts, makeDirector, subjectsOf, direct, pinSubject,
  replayShot, makeReplay, recordReplay, replayPose, replayWindow,
} from '../src/director.js';

const boxesOf = (s) => buildScene(s.course.gates, s.scenery.pillars, { inner: GATE_INNER, bar: FRAME.bar, legRadius: FRAME.legRadius });
const DEG = Math.PI / 180;

test('line of sight: blocked by a bar, clear through a gate opening and past a pillar', () => {
  const s = makeSim({ seed: 2, difficulty: 'normal' });
  const boxes = boxesOf(s), g = s.course.gates[3];
  const before = v3.sub(g.pos, v3.scale(g.normal, 10)), after = v3.add(g.pos, v3.scale(g.normal, 10));
  assert.equal(blocked(before, after, boxes), false, 'straight through the opening');
  // through the top bar
  const bar = v3.scale(g.up, GATE_INNER / 2 + FRAME.bar / 2);
  assert.equal(blocked(v3.add(before, bar), v3.add(after, bar), boxes), true, 'through the top bar');
  const q = s.scenery.pillars[0];
  const a = [q.pos[0] - 20, 0.5, q.pos[2]], b = [q.pos[0] + 20, 0.5, q.pos[2]];
  assert.equal(blocked(a, b, boxes), true, 'through a pillar');
  assert.equal(blocked([a[0], 80, a[2]], [b[0], 80, b[2]], boxes), false, 'high above everything');
  assert.ok(clearance(q.pos, boxes) === 0 && clearance([q.pos[0], 200, q.pos[2]], boxes) > 100);
});

test('every gate gets a trackside post that is clear of everything and sees its stretch of line', () => {
  for (const difficulty of ['easy', 'normal', 'hard', 'pro']) {
    for (const seed of [1, 4, 7, 9]) {
      const s = makeSim({ seed, difficulty });
      const boxes = boxesOf(s), posts = placePosts(s.course, s.plan, boxes);
      assert.equal(posts.length, s.course.gates.length, `${difficulty} ${seed}`);
      assert.deepEqual(posts.map((p) => p.gate), s.course.gates.map((g) => g.id));
      for (const p of posts) {
        assert.ok(clearance(p.pos, boxes) >= TV.postClear, 'clear of frames, legs and pillars');
        for (const q of s.plan.pts) assert.ok(Math.hypot(q.p[0] - p.pos[0], q.p[2] - p.pos[2]) >= TV.lineClear, 'off the racing line');
        assert.ok(p.pos[1] > 0.5, 'above the ground');
        assert.ok(p.sees >= 0.8, `${difficulty} ${seed} gate ${p.gate} sees ${p.sees}`);
      }
    }
  }
});

// A field race (the hero plus the roster), directed for `secs` of viewing.
function directedRace({ seed, difficulty, secs = 50, fps = 60, rate = 1, pin = null }) {
  const hero = makeSim({ seed, difficulty, launchAt: 3 });
  const entries = [{ id: 'hero', name: 'Hero', sim: hero }, ...rosterFor({ pilot: 'racing', nav: 'truth' })
    .map((r, k) => ({ id: r.id, name: r.name, sim: makeSim({ seed, difficulty, ...r.opts, slot: k + 1, launchAt: 3 }) }))];
  const sims = entries.map((e) => e.sim);
  const dir = makeDirector({ course: hero.course, plan: hero.plan, boxes: boxesOf(hero), seed });
  if (pin) pinSubject(dir, pin);
  const dt = 1 / fps, per = Math.round(120 * dt * rate);
  const shots = [], subjects = new Set();
  let cur = null, frames = 0, bad = 0;
  for (let f = 0; f < secs * fps; f++) {
    for (let k = 0; k < per; k++) {
      shareTraffic(sims);
      for (const s of sims) step(s, 1 / 120);
      bumpAll(sims);
    }
    const out = direct(dir, subjectsOf(entries), dt, rate);
    if (out.cut) {
      if (cur) shots.push(cur);
      cur = { kind: out.kind, subject: out.subject, len: 0, lost: false };
      subjects.add(out.subject);
    }
    cur.len += dt;
    if (out.blocked || out.inFrame === false) cur.lost = true;
    frames++;
    if (!Number.isFinite(out.fov) || (out.pos && !out.pos.every(Number.isFinite))) bad++;
  }
  return { dir, shots, subjects, frames, bad };
}

test('the director cuts between all its shots, keeps the drone in frame and mostly in clear view', () => {
  for (const [difficulty, seed] of [['normal', 1], ['hard', 3]]) {
    const { dir, shots, subjects, bad } = directedRace({ seed, difficulty, secs: 90 });
    assert.equal(bad, 0, 'finite camera');
    for (const k of Object.keys(TV.weights)) assert.ok(dir.stats.kinds[k] > 0, `${difficulty}: uses ${k}`);
    for (const s of shots) {
      assert.ok(s.len <= TV.maxShot + 0.05, `shot of ${s.len.toFixed(2)} s`);
      if (s.len < TV.minShot - 0.05) assert.ok(s.lost || s.kind === 'flyby', `short ${s.kind} shot without losing the view`);
    }
    assert.ok(subjects.size >= 3, `films several drones (${[...subjects]})`);
    const st = dir.stats;
    assert.ok(st.postInFrame / st.postFrames > 0.99, 'trackside: drone in frame');
    assert.ok(st.postVisible / st.postFrames > 0.9, `trackside: clear line of sight ${(st.postVisible / st.postFrames).toFixed(3)}`);
  }
});

test('trackside cameras keep up at 4x speed on a slow device (10 fps)', () => {
  const { dir } = directedRace({ seed: 2, difficulty: 'normal', fps: 10, rate: 4, secs: 30 });
  assert.ok(dir.stats.postFrames > 20);
  assert.ok(dir.stats.postInFrame / dir.stats.postFrames > 0.98);
});

test('pinning a drone keeps every shot on it; the same seed directs the same race', () => {
  const pinned = directedRace({ seed: 5, difficulty: 'normal', pin: 'steady', secs: 25 });
  assert.deepEqual([...pinned.subjects], ['steady']);
  const a = directedRace({ seed: 6, difficulty: 'easy', secs: 20 }), b = directedRace({ seed: 6, difficulty: 'easy', secs: 20 });
  assert.deepEqual(a.shots, b.shots);
});

test('replay buffer: 30 poses a second, the last 12 s kept, interpolated, windowed around an event', () => {
  const sim = { drone: { pos: [0, 2, 0], vel: [10, 0, 0], yaw: 0, thrust: [0, 9.81, 0] } };
  const rep = makeReplay();
  for (let t = 0; t <= 20.0001; t += 1 / 120) {
    sim.drone.pos = [10 * t, 2, 0];
    sim.drone.yaw = t; // wraps past pi: interpolation must take the short way round
    sim.drone.tumble = t > 15 ? { axis: [1, 0, 0], angle: 3 * (t - 15) } : undefined;
    recordReplay(rep, t, [{ id: 'a', sim }]);
  }
  const span = rep.frames.at(-1).t - rep.frames[0].t;
  assert.ok(Math.abs(span - TV.replay.keep) < 0.05, `keeps ${span}`);
  assert.ok(Math.abs(rep.frames.length / span - TV.replay.rate) < 1);
  const p = replayPose(rep, 'a', 12.345);
  assert.ok(Math.abs(p.pos[0] - 123.45) < 1e-6, 'linear motion interpolates exactly');
  assert.ok(Math.cos(p.yaw - 12.345) > 1 - 1e-9, 'heading interpolates');
  assert.ok(Math.abs(replayPose(rep, 'a', 16).tumble.angle - 3) < 0.05, 'tumble replays');
  assert.equal(replayPose(rep, 'a', 10).tumble, null);
  assert.deepEqual(replayPose(rep, 'a', 0).pos, rep.frames[0].poses.a.pos, 'clamped to what was kept');
  assert.equal(replayPose(rep, 'b', 12), null);
  const w = replayWindow(rep, 15);
  assert.ok(Math.abs(w.from - (15 - TV.replay.before)) < 1e-9 && Math.abs(w.to - (15 + TV.replay.after)) < 1e-9);
  const late = replayWindow(rep, 19.9);
  assert.ok(late.to <= 20 + 1e-9, 'never past the newest pose');
  assert.equal(replayWindow(rep, 2), null, 'too old to show');
});

test('a replay is filmed from the nearest post with a clear view of the moment', () => {
  const s = makeSim({ seed: 3, difficulty: 'hard' });
  const dir = makeDirector({ course: s.course, plan: s.plan, boxes: boxesOf(s), seed: 3 });
  for (const g of s.course.gates) {
    const p = v3.add(g.pos, v3.scale(g.normal, 2));
    const shot = replayShot(dir, 'hero', p, v3.scale(g.normal, 20));
    assert.equal(shot.kind, 'replay');
    assert.ok(clearance(shot.pos, dir.boxes) >= 1, 'camera stands clear');
    assert.equal(blocked(shot.pos, p, dir.boxes), false, `clear view of gate ${g.id}`);
    // it stays on that camera until the replay is over, panning with the drone
    const subs = [{ id: 'hero', pos: p, vel: v3.scale(g.normal, 20), yaw: 0, flying: true }];
    let out;
    for (let k = 0; k < 120; k++) out = direct(dir, subs, 1 / 60, TV.replay.speed);
    assert.equal(out.kind, 'replay');
    assert.ok(out.inFrame);
    const ax = v3.norm(v3.sub(out.look, out.pos)), to = v3.norm(v3.sub(p, out.pos));
    assert.ok(Math.acos(Math.min(1, v3.dot(ax, to))) < out.fov * DEG * 0.5);
  }
});
