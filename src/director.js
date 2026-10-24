// TV mode: an automatic race director with trackside cameras and slow-motion replays.
// Pure logic with no DOM or rendering, so it is tested in Node.
//
// - Trackside camera posts are placed once per course, one beside each gate, where they
//   stand clear of every frame, leg, pillar and the racing line, and can see the most of
//   the line through that gate (line of sight is ray-cast against the same boxes the pixel
//   camera draws).
// - Each frame the director knows where every drone is and how far round the lap it has
//   got. At each cut it picks the most interesting drone (a close battle, a crash, the
//   leader) and the best shot of it: a trackside post it is flying towards, a low flyby
//   camera dropped on the line ahead, a tight chase, a helicopter or the onboard camera.
//   A trackside shot pans like a camera operator (a critically damped aim that leads the
//   drone) and zooms to keep the drone the same size in frame; it cuts away early if a
//   frame or pillar gets between the lens and the drone.
// - A replay buffer keeps the last few seconds of every drone's pose, so a crash, a lead
//   change or the chequered flag can be shown again in slow motion from a trackside post.

import { v3, mulberry32 } from './sim.js';

export const TV = {
  minShot: 2.4, // s: shortest shot (unless the view is lost)
  maxShot: 6.5, // s: longest shot
  frameWidth: 6, // m of scene across the frame on a trackside shot (zoom target)
  fovMin: 7, fovMax: 55, // degrees (vertical) for trackside zoom
  postClear: 1.2, // m from any solid box
  lineClear: 4.5, // m (horizontal) from the racing line
  postRange: 75, // m: furthest a post films from
  lost: 0.35, // s of blocked view before a trackside shot cuts away
  battle: 14, // m: two drones closer than this along the line are a battle
  replay: { keep: 12, rate: 30, before: 2.8, after: 1.2, speed: 0.4, cooldown: 18 },
  weights: { post: 0.42, flyby: 0.18, chase: 0.16, heli: 0.14, onboard: 0.1 },
};
const DEG = Math.PI / 180;

// ---------- geometry ----------
// Does the segment a→b pass through any box? Boxes are { c, a: [3 unit axes], h, r }
// as built by eye.js buildScene. The ends are trimmed a little (the drone itself sits at b).
export function blocked(a, b, boxes, trim = 0.4) {
  const d = v3.sub(b, a), L = v3.len(d);
  if (L < 1e-6) return false;
  const t0 = Math.min(0.5, trim / L), t1 = 1 - t0;
  for (const bx of boxes) {
    // quick reject: distance from the box centre to the segment's line vs bounding radius
    const o = v3.sub(bx.c, a);
    const u = Math.max(0, Math.min(1, v3.dot(o, d) / (L * L)));
    const q = v3.sub(o, v3.scale(d, u));
    if (v3.dot(q, q) > bx.r * bx.r) continue;
    let lo = t0, hi = t1, hit = true;
    for (let k = 0; k < 3 && hit; k++) {
      const ax = bx.a[k], e = v3.dot(o, ax), f = v3.dot(d, ax), h = bx.h[k];
      if (Math.abs(f) < 1e-9) {
        if (Math.abs(e) > h) hit = false;
        continue;
      }
      let ta = (e - h) / f, tb = (e + h) / f;
      if (ta > tb) [ta, tb] = [tb, ta];
      lo = Math.max(lo, ta);
      hi = Math.min(hi, tb);
      if (lo > hi) hit = false;
    }
    if (hit) return true;
  }
  return false;
}

// Distance from p to the surface of a box (0 inside).
function boxDist(p, bx) {
  const o = v3.sub(p, bx.c);
  let s = 0;
  for (let k = 0; k < 3; k++) {
    const e = Math.abs(v3.dot(o, bx.a[k])) - bx.h[k];
    if (e > 0) s += e * e;
  }
  return Math.sqrt(s);
}
export function clearance(p, boxes) {
  let m = Infinity;
  for (const bx of boxes) {
    const o = v3.sub(p, bx.c);
    if (v3.dot(o, o) > (bx.r + m) ** 2) continue;
    m = Math.min(m, boxDist(p, bx));
  }
  return m;
}
function lineDist(p, plan) {
  let m = Infinity;
  for (const q of plan.pts) m = Math.min(m, Math.hypot(q.p[0] - p[0], q.p[2] - p[2]));
  return m;
}

// ---------- trackside posts ----------
// One post per gate: the candidate beside it (before or after, either side, low or high)
// that is clear of everything and sees the most of the line from 30 m before the gate to
// 10 m after it. Returns [{ pos, gate, sees }] (sees: fraction of that stretch in view).
export function placePosts(course, plan, boxes) {
  const posts = [];
  const N = plan.pts.length;
  course.gates.forEach((g, i) => {
    const hz = v3.norm([g.normal[0], 0, g.normal[2]]), side = [hz[2], 0, -hz[0]];
    const samples = [];
    for (let s = -30; s <= 10; s += 2.5) samples.push(plan.pts[((Math.round((plan.gateS[i] + s) / plan.ds) % N) + N) % N].p);
    let best = null;
    for (const along of [-10, -4, 5, 11, 17]) for (const off of [6.5, 9, 13]) for (const sgn of [-1, 1]) for (const hy of [1.1, 2.6, g.pos[1] + 2.5]) {
      const p = v3.add(v3.add(g.pos, v3.scale(hz, along)), v3.scale(side, off * sgn));
      p[1] = Math.max(0.8, Math.min(22, hy));
      if (clearance(p, boxes) < TV.postClear || lineDist(p, plan) < TV.lineClear) continue;
      let seen = 0;
      for (const q of samples) if (v3.len(v3.sub(q, p)) < TV.postRange && !blocked(p, q, boxes)) seen++;
      const sees = seen / samples.length;
      // see the stretch; the gate itself in shot; not too far; a gentle preference for the exit
      // side (the drone comes through the frame towards the lens)
      const score = sees + (blocked(p, g.pos, boxes) ? 0 : 0.25) - v3.len(v3.sub(p, g.pos)) * 0.004 + (along > 0 ? 0.05 : 0) - (hy > 2 && hy < 3 ? 0.02 : 0);
      if (!best || score > best.score) best = { pos: p, gate: i, sees, score };
    }
    if (best) posts.push({ pos: best.pos, gate: best.gate, sees: best.sees });
  });
  return posts;
}

// ---------- subjects ----------
// What the director knows about each drone. entries: [{ id, name, sim }].
// progress: gates passed plus the fraction of the current leg flown (comparable across drones).
export function subjectsOf(entries) {
  return entries.map((e) => {
    const s = e.sim, gates = s.course.gates, n = gates.length, st = s.state;
    const tgt = gates[st.target], prev = gates[(st.target - 1 + n) % n];
    const leg = Math.max(1, v3.len(v3.sub(tgt.pos, prev.pos)));
    const frac = st.gatesPassed ? Math.max(0, Math.min(1, 1 - v3.len(v3.sub(tgt.pos, s.drone.pos)) / leg)) : 0;
    return {
      id: e.id, name: e.name, color: e.color,
      pos: s.drone.pos, vel: s.drone.vel, yaw: s.drone.yaw,
      crashed: !!s.crash, flying: s.t >= s.launchAt,
      progress: st.gatesPassed + frac, leg, target: st.target, lap: st.lap,
    };
  });
}

// ---------- the director ----------
export function makeDirector({ course, plan, boxes, seed = 1 }) {
  return {
    course, plan, boxes,
    posts: placePosts(course, plan, boxes),
    rand: mulberry32((seed * 2654435761) >>> 0),
    shot: null, // { kind, subject, label, t (age), len, post?, pos?, aim?, aimVel?, fov }
    pin: null, // a drone the viewer asked to follow
    crashSeen: new Map(), // id -> clock when it was last seen crashed
    clock: 0,
    cuts: 0,
    stats: { frames: 0, postFrames: 0, postVisible: 0, kinds: {} },
  };
}

// How interesting each drone is right now, for picking who to film.
function interest(dir, subs) {
  const order = subs.filter((s) => s.flying).sort((a, b) => b.progress - a.progress);
  const score = new Map(subs.map((s) => [s.id, 0.2]));
  if (!order.length) return score;
  score.set(order[0].id, 1);
  for (let i = 0; i + 1 < order.length; i++) {
    const a = order[i], b = order[i + 1];
    const gap = (a.progress - b.progress) * (a.leg + b.leg) / 2; // metres along the course
    if (gap < TV.battle && Math.floor(a.progress) - Math.floor(b.progress) < 3) {
      const w = 2.2 / (1 + gap / 4) + (i === 0 ? 0.5 : 0);
      score.set(a.id, score.get(a.id) + w);
      score.set(b.id, score.get(b.id) + w * 1.1); // the chaser: the one making the move
    }
  }
  for (const s of subs) {
    if (s.crashed) dir.crashSeen.set(s.id, dir.clock);
    const since = dir.clock - (dir.crashSeen.get(s.id) ?? -Infinity);
    if (since < 2.5) score.set(s.id, score.get(s.id) + 3);
  }
  return score;
}

// Point on the racing line `ahead` metres past the one nearest p.
function lineAhead(plan, p, ahead) {
  const N = plan.pts.length;
  let bi = 0, bd = Infinity;
  for (let i = 0; i < N; i += 2) {
    const q = plan.pts[i].p, d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2;
    if (d < bd) { bd = d; bi = i; }
  }
  return plan.pts[(bi + Math.round(ahead / plan.ds)) % N];
}

// The trackside post best placed to film subject s: in range, with a clear view now and
// of where it will be in a second and a half, and that it is flying towards.
function bestPost(dir, s, exclude = -1) {
  let best = null;
  const fut = v3.add(s.pos, v3.scale(s.vel, 1.5));
  for (const post of dir.posts) {
    if (post.gate === exclude) continue;
    const to = v3.sub(post.pos, s.pos), d = v3.len(to);
    if (d > TV.postRange || d < 5) continue;
    if (blocked(post.pos, s.pos, dir.boxes) || blocked(post.pos, fut, dir.boxes)) continue;
    const sp = v3.len(s.vel);
    const closing = sp > 1 ? v3.dot(to, s.vel) / (d * sp) : 0; // +1: flying straight at it
    const score = closing * 1.2 - d / TV.postRange + post.sees * 0.5;
    if (!best || score > best.score) best = { post, score };
  }
  return best && best.score > -0.4 ? best.post : null;
}

// A low camera dropped beside the line ahead of the drone, so it whips past the lens.
function flybySpot(dir, s) {
  const sp = Math.max(6, v3.len(s.vel));
  const q = lineAhead(dir.plan, s.pos, sp * 1.4);
  const tz = v3.norm([q.t[0], 0, q.t[2]]), side = [tz[2], 0, -tz[0]];
  for (const off of [3.2, -3.2, 4.5, -4.5]) {
    const p = v3.add(q.p, v3.scale(side, off));
    p[1] = Math.max(0.5, q.p[1] - 1.6);
    if (clearance(p, dir.boxes) < 1 || blocked(p, q.p, dir.boxes) || blocked(p, s.pos, dir.boxes)) continue;
    return p;
  }
  return null;
}

const SHOT_LABEL = { post: 'Trackside', flyby: 'Trackside', chase: 'Chase cam', heli: 'Heli cam', onboard: 'Onboard', replay: 'Replay' };

function pick(dir, subs) {
  const score = interest(dir, subs);
  let subject = dir.pin && subs.find((s) => s.id === dir.pin);
  if (!subject) {
    // weighted towards the most interesting, but not always the same drone
    const live = subs.filter((s) => s.flying || s.crashed);
    const pool = live.length ? live : subs;
    const w = pool.map((s) => score.get(s.id) ** 2);
    let r = dir.rand() * w.reduce((a, b) => a + b, 0);
    subject = pool[pool.length - 1];
    for (let i = 0; i < pool.length; i++) if ((r -= w[i]) <= 0) { subject = pool[i]; break; }
  }
  const prev = dir.shot;
  // a drone on the pad or tumbling is best seen from outside
  const kinds = Object.entries(TV.weights).filter(([k]) => {
    if (!subject.flying || subject.crashed) return k === 'post' || k === 'heli';
    return !(prev && prev.kind === k && k !== 'post');
  });
  for (let tries = 0; tries < 6; tries++) {
    let r = dir.rand() * kinds.reduce((a, [, w]) => a + w, 0), kind = kinds[0][0];
    for (const [k, w] of kinds) if ((r -= w) <= 0) { kind = k; break; }
    const shot = { kind, subject: subject.id, t: 0, label: SHOT_LABEL[kind], fov: 50, blockedFor: 0 };
    if (kind === 'post') {
      const post = bestPost(dir, subject, prev?.kind === 'post' ? prev.post.gate : -1);
      if (!post) continue;
      shot.post = post;
      shot.pos = post.pos.slice();
      shot.label = `Gate ${post.gate + 1} camera`;
      shot.len = TV.minShot + dir.rand() * (TV.maxShot - TV.minShot);
    } else if (kind === 'flyby') {
      const p = flybySpot(dir, subject);
      if (!p) continue;
      shot.pos = p;
      shot.len = 2.9;
    } else {
      shot.len = kind === 'onboard' ? 2.6 + dir.rand() * 1.2 : TV.minShot + dir.rand() * 2.5;
    }
    return shot;
  }
  return { kind: 'heli', subject: subject.id, t: 0, label: SHOT_LABEL.heli, fov: 40, len: TV.minShot + 1, blockedFor: 0 };
}

// Critically damped spring of x towards target (per axis), natural frequency w.
function spring(x, v, target, w, dt) {
  for (let k = 0; k < 3; k++) {
    const a = w * w * (target[k] - x[k]) - 2 * w * v[k];
    v[k] += a * dt;
    x[k] += v[k] * dt;
  }
}

// Advance the director by dt (seconds of viewing) and return the camera:
// { kind, subject, label, pos, look, fov, cut } (kind 'onboard' has no pos: use the drone's
// own camera). cut is true on the frame a new shot starts.
export function direct(dir, subs, dt, rate = 1) {
  dir.clock += dt;
  let shot = dir.shot, cut = false;
  let s = shot && subs.find((x) => x.id === shot.subject);
  const pinMoved = dir.pin && shot && shot.subject !== dir.pin;
  if (!shot || !s || (shot.kind !== 'replay' && (shot.t >= shot.len || pinMoved || (shot.blockedFor > TV.lost && shot.t > 0.8)))) {
    shot = dir.shot = pick(dir, subs);
    s = subs.find((x) => x.id === shot.subject);
    dir.cuts++;
    cut = true;
    dir.stats.kinds[shot.kind] = (dir.stats.kinds[shot.kind] ?? 0) + 1;
  }
  shot.t += dt;
  const out = frameShot(dir, shot, s, dt, cut, rate);
  dir.stats.frames++;
  if (shot.kind === 'post') {
    dir.stats.postFrames++;
    if (!out.blocked) dir.stats.postVisible++;
    if (out.inFrame) dir.stats.postInFrame = (dir.stats.postInFrame ?? 0) + 1;
  }
  return out;
}

// Where the camera is and what it looks at for this shot, this frame.
// rate: simulated seconds per second of viewing (the Speed setting), so the pan keeps up.
function frameShot(dir, shot, s, dt, cut, rate) {
  const sp = v3.len(s.vel);
  const fwd = sp > 1.5 ? v3.norm([s.vel[0], 0, s.vel[2]]) : [Math.sin(s.yaw), 0, Math.cos(s.yaw)];
  const side = [fwd[2], 0, -fwd[0]];
  const out = { kind: shot.kind, subject: s.id, label: shot.label, cut, fov: shot.fov, blocked: false };
  if (shot.kind === 'onboard') return out;
  if (shot.kind === 'post' || shot.kind === 'flyby' || shot.kind === 'replay') {
    // a camera operator on a fixed tripod: pan with the drone, leaving it room to fly into
    // (a lead offset that eases in and out, so the framing breathes as it speeds up, brakes
    // and turns), and zoom so it stays about the same size in frame
    let room = v3.scale(s.vel, 0.09);
    if (v3.len(room) > 1.3) room = v3.scale(room, 1.3 / v3.len(room));
    if (cut || !shot.room) { shot.room = room.slice(); shot.roomVel = [0, 0, 0]; }
    spring(shot.room, shot.roomVel, room, 3, dt);
    shot.aim = v3.add(s.pos, shot.room);
    const d = v3.len(v3.sub(shot.aim, shot.pos));
    const width = shot.kind === 'flyby' ? TV.frameWidth * 2.2 : shot.kind === 'replay' ? TV.frameWidth * 0.8 : TV.frameWidth;
    const want = Math.max(TV.fovMin, Math.min(TV.fovMax, 2 * Math.atan(width / 2 / Math.max(1, d)) / DEG));
    shot.fov = cut || shot.fov == null ? want : shot.fov + (want - shot.fov) * Math.min(1, dt * 3);
    out.blocked = blocked(shot.pos, s.pos, dir.boxes);
    // is the drone inside the picture? (angle off the lens axis against the half field of view)
    const ax = v3.norm(v3.sub(shot.aim, shot.pos)), to = v3.norm(v3.sub(s.pos, shot.pos));
    out.inFrame = Math.acos(Math.max(-1, Math.min(1, v3.dot(ax, to)))) < shot.fov * DEG * 0.5;
    shot.blockedFor = out.blocked || !out.inFrame ? shot.blockedFor + dt : 0;
    return Object.assign(out, { pos: shot.pos.slice(), look: shot.aim.slice(), fov: shot.fov });
  }
  // moving cameras: chase (low and tight, a little off to one side) and helicopter (high, wide)
  const want = shot.kind === 'chase'
    ? v3.add(v3.add(v3.sub(s.pos, v3.scale(fwd, 5.5)), v3.scale(side, 1.4)), [0, 1.3, 0])
    : v3.add(v3.add(v3.sub(s.pos, v3.scale(fwd, 16)), v3.scale(side, 14)), [0, 15, 0]);
  if (want[1] < 0.6) want[1] = 0.6;
  const wp = shot.kind === 'chase' ? 7 : 2.5;
  if (cut || !shot.pos) { shot.pos = want.slice(); shot.posVel = v3.scale(s.vel, rate); }
  spring(shot.pos, shot.posVel, v3.add(want, v3.scale(s.vel, rate * (2 / wp))), wp, dt);
  const look = v3.add(s.pos, v3.scale(fwd, shot.kind === 'chase' ? 3 : 1));
  shot.fov = shot.kind === 'chase' ? 58 : 34;
  return Object.assign(out, { pos: shot.pos.slice(), look, fov: shot.fov });
}

// Pin the director to one drone (null: let it choose).
export function pinSubject(dir, id) {
  dir.pin = id;
}

// Start a replay shot of subject id around point p: the trackside post with the best
// view of it, else a low camera beside it.
export function replayShot(dir, subjectId, p, vel = [0, 0, 0]) {
  let best = null;
  for (const post of dir.posts) {
    const d = v3.len(v3.sub(post.pos, p));
    if (d > TV.postRange || d < 4 || blocked(post.pos, p, dir.boxes)) continue;
    if (!best || d < best.d) best = { post, d };
  }
  let pos = best?.post.pos.slice();
  if (!pos) {
    const fwd = v3.len(vel) > 1 ? v3.norm([vel[0], 0, vel[2]]) : [1, 0, 0];
    pos = v3.add(v3.add(p, v3.scale(fwd, 8)), [fwd[2] * 6, 2, -fwd[0] * 6]);
  }
  dir.shot = { kind: 'replay', subject: subjectId, t: 0, len: Infinity, label: best ? `Gate ${best.post.gate + 1} camera` : 'Trackside', pos, fov: 30, blockedFor: 0 };
  return dir.shot;
}
// Back to live after a replay: cut to a fresh shot.
export function endReplay(dir) {
  dir.shot = null;
}

// ---------- replay buffer ----------
// Every drone's pose, TV.replay.rate times a second, for the last TV.replay.keep seconds.
export function makeReplay() {
  return { frames: [], last: -Infinity };
}
export function recordReplay(rep, t, entries) {
  if (t - rep.last < 1 / TV.replay.rate - 1e-9) return;
  rep.last = t;
  const poses = {};
  for (const e of entries) {
    const d = e.sim.drone;
    poses[e.id] = {
      pos: d.pos.slice(), vel: d.vel.slice(), yaw: d.yaw, thrust: d.thrust.slice(),
      tumble: d.tumble ? { axis: d.tumble.axis.slice(), angle: d.tumble.angle } : null,
    };
  }
  rep.frames.push({ t, poses });
  while (rep.frames.length && rep.frames[0].t < t - TV.replay.keep) rep.frames.shift();
}
const lerp = (a, b, u) => a.map((x, k) => x + (b[k] - x) * u);
function lerpAngle(a, b, u) {
  let d = b - a;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return a + d * u;
}
// The pose of drone id at time t, interpolated (clamped to what was recorded), or null.
export function replayPose(rep, id, t) {
  const F = rep.frames;
  if (!F.length) return null;
  if (t <= F[0].t) return F[0].poses[id] ?? null;
  if (t >= F[F.length - 1].t) return F[F.length - 1].poses[id] ?? null;
  let lo = 0, hi = F.length - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (F[m].t <= t) lo = m; else hi = m;
  }
  const a = F[lo].poses[id], b = F[hi].poses[id];
  if (!a || !b) return a ?? b ?? null;
  const u = (t - F[lo].t) / (F[hi].t - F[lo].t);
  return {
    pos: lerp(a.pos, b.pos, u), vel: lerp(a.vel, b.vel, u), yaw: lerpAngle(a.yaw, b.yaw, u),
    thrust: lerp(a.thrust, b.thrust, u),
    tumble: a.tumble && b.tumble ? { axis: a.tumble.axis, angle: a.tumble.angle + (b.tumble.angle - a.tumble.angle) * u } : (u < 0.5 ? a.tumble : b.tumble),
  };
}
// The window to show for an event at time te: [from, to] in sim seconds, within what was kept.
export function replayWindow(rep, te) {
  const F = rep.frames;
  if (!F.length) return null;
  const from = Math.max(F[0].t, te - TV.replay.before), to = Math.min(F[F.length - 1].t, te + TV.replay.after);
  return to - from > 0.8 ? { from, to } : null;
}
