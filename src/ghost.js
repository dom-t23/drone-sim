// Ghost replay: records every lap as it is flown and keeps the fastest one, so a gold
// ghost can fly your best lap alongside you. Pure logic with no DOM, tested in Node.
//
// A ghost is { time, gates, frames } where time is the lap time (s), gates[i] is the time
// into the lap the ghost passed gate i, and frames is a flat array of samples:
// [t, x, y, z, yaw, ux, uy, uz, rx, ry, rz] with u the unit thrust direction (the body's up
// axis) and r the crash tumble as axis * angle (zero when flying normally).

export const GHOST = {
  rate: 20, // samples per second
  maxLap: 240, // s: stop recording a lap longer than this (and never keep it)
  version: 1, // bump if the saved format changes
};
const F = 11; // floats per frame

export function makeGhostRecorder(best = null) {
  // gap: { gate, delta } at the latest gate passed, against the ghost as it was then
  return { cur: null, best, seen: 0, improved: 0, gap: null };
}

// Call after every sim step. Returns true when a new best ghost was just kept.
export function recordGhost(rec, sim) {
  const s = sim.state;
  let kept = false;
  for (; rec.seen < sim.events.length; rec.seen++) {
    const e = sim.events[rec.seen];
    if (!rec.cur) continue;
    if (e.type === 'gate' && e.gate !== 0) {
      const t = e.t - rec.cur.start, d = ghostGap(rec.best, e.gate, t);
      rec.cur.gates[e.gate] = t;
      if (d != null) rec.gap = { gate: e.gate, delta: d };
    }
    if (e.type === 'lap') {
      const c = rec.cur;
      if (rec.best) rec.gap = { gate: 0, delta: e.time - rec.best.time };
      if (!c.overflow && (!rec.best || e.time < rec.best.time)) {
        c.gates[0] = e.time; // gate 0 closes the lap
        rec.best = { time: e.time, gates: c.gates, frames: c.frames };
        rec.improved++;
        kept = true;
      }
      rec.cur = null;
    }
  }
  if (s.lapStart == null) return kept;
  if (!rec.cur || rec.cur.start !== s.lapStart) {
    rec.cur = { start: s.lapStart, gates: [], frames: [], overflow: false };
    rec.seen = sim.events.length; // this lap's own gate-0 event is already behind us
  }
  const c = rec.cur, t = sim.t - c.start;
  if (c.overflow) return kept;
  if (t > GHOST.maxLap) {
    c.overflow = true;
    c.frames = [];
    return kept;
  }
  const n = c.frames.length / F;
  if (n === 0 || t >= n / GHOST.rate) c.frames.push(t, ...sampleOf(sim.drone));
  return kept;
}

function sampleOf(d) {
  const T = d.thrust, l = Math.hypot(T[0], T[1], T[2]) || 1;
  const tb = d.tumble, a = tb ? tb.angle : 0;
  const ax = tb ? tb.axis : [0, 0, 0];
  return [d.pos[0], d.pos[1], d.pos[2], d.yaw, T[0] / l, T[1] / l, T[2] / l, ax[0] * a, ax[1] * a, ax[2] * a];
}

// Where the ghost is t seconds into its lap: { pos, yaw, thrust, tumble? } in the same
// shape as a drone, so the renderer can draw it like one. null outside the lap.
export function ghostPose(g, t) {
  const fr = g?.frames, n = fr ? fr.length / F : 0;
  if (!n || !(t >= 0) || t > g.time) return null;
  // binary search for the last sample at or before t
  let lo = 0, hi = n - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (fr[mid * F] <= t) lo = mid; else hi = mid - 1;
  }
  const i = lo, j = Math.min(n - 1, i + 1);
  const t0 = fr[i * F], t1 = fr[j * F];
  const f = j > i && t1 > t0 ? Math.max(0, Math.min(1, (t - t0) / (t1 - t0))) : 0;
  const at = (k) => fr[i * F + k] + (fr[j * F + k] - fr[i * F + k]) * f;
  let dy = fr[j * F + 4] - fr[i * F + 4];
  dy = Math.atan2(Math.sin(dy), Math.cos(dy)); // shortest way round
  const pose = {
    pos: [at(1), at(2), at(3)],
    yaw: fr[i * F + 4] + dy * f,
    thrust: [at(5), at(6), at(7)],
  };
  const l = Math.hypot(...pose.thrust) || 1;
  pose.thrust = pose.thrust.map((x) => x / l);
  const r = [at(8), at(9), at(10)], angle = Math.hypot(r[0], r[1], r[2]);
  if (angle > 1e-6) pose.tumble = { axis: r.map((x) => x / angle), angle };
  return pose;
}

// Gap to the ghost at the last gate passed this lap: positive means slower than the ghost.
// current: { gate, t } of the latest gate this lap, t measured from the lap start.
export function ghostGap(g, gate, t) {
  const ref = g?.gates?.[gate];
  return ref == null ? null : t - ref;
}

// Compact text form for localStorage: centimetres and milliradians as integers.
export function encodeGhost(g) {
  const q = g.frames.map((x, k) => {
    const c = k % F;
    return c === 0 ? Math.round(x * 1000) : c <= 3 ? Math.round(x * 100) : Math.round(x * 1000);
  });
  return JSON.stringify({ v: GHOST.version, time: g.time, gates: g.gates, q });
}

export function decodeGhost(text) {
  try {
    const o = JSON.parse(text);
    if (!o || o.v !== GHOST.version || !Number.isFinite(o.time) || !Array.isArray(o.q) || o.q.length % F || !o.q.length) return null;
    const frames = o.q.map((x, k) => {
      const c = k % F;
      return c === 0 ? x / 1000 : c <= 3 ? x / 100 : x / 1000;
    });
    if (!frames.every(Number.isFinite)) return null;
    return { time: o.time, gates: Array.isArray(o.gates) ? o.gates : [], frames };
  } catch {
    return null;
  }
}
