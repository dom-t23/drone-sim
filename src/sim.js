// Simulation core: course, quadrotor dynamics and autopilot.
// Pure JavaScript with no rendering dependencies, so it runs in the browser and in Node tests.
// Coordinates: metres, seconds; Y is up.

export const G = 9.81;

// ---------- small vector helpers (arrays [x, y, z]) ----------
export const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
  norm: (a) => {
    const l = Math.hypot(a[0], a[1], a[2]) || 1;
    return [a[0] / l, a[1] / l, a[2] / l];
  },
  clampLen: (a, max) => {
    const l = Math.hypot(a[0], a[1], a[2]);
    return l > max ? [a[0] * max / l, a[1] * max / l, a[2] * max / l] : a;
  },
};

// ---------- course ----------
export const GATE_INNER = 3.2; // inner opening width/height, metres

// Course presets. 'normal' is the original layout and must stay byte-for-byte the same
// (tests and shared links depend on it); the others draw extra random numbers after it.
export const DIFFICULTIES = ['easy', 'normal', 'hard'];
const DEG = Math.PI / 180;
const MAX_PITCH = 30 * DEG; // steepest gate a course may contain

// A closed loop of gates. Each gate stores its facing (normal) plus the in-plane
// right/up axes used for the "did it go through the opening" check, so gates may be
// yawed off the direction of travel or pitched (dive gates).
export function makeCourse({ gates, seed = 1, difficulty = 'normal' } = {}) {
  if (!DIFFICULTIES.includes(difficulty)) difficulty = 'normal';
  const rand = mulberry32(seed);
  const phase = rand() * Math.PI * 2;
  let curve, count, angle = () => ({ yaw: 0, pitch: 0 }), follow3d = false;
  if (difficulty === 'easy') {
    count = gates ?? 8;
    curve = (t) => {
      const r = 48 + 7 * Math.sin(2 * t + phase);
      return [r * Math.cos(t), 4.5 + 1.2 * Math.sin(2 * t + phase), r * Math.sin(t)];
    };
  } else if (difficulty === 'hard') {
    count = gates ?? 12;
    const p2 = rand() * Math.PI * 2, p3 = rand() * Math.PI * 2;
    curve = (t) => {
      const r = 40 + 12 * Math.sin(2 * t + phase) + 6 * Math.sin(3 * t + p2);
      const y = 8.5 + 4 * Math.sin(2 * t + p3) + 1.4 * Math.sin(5 * t + phase);
      return [r * Math.cos(t), Math.max(3.4, y), r * Math.sin(t)];
    };
    follow3d = true; // gates pitch with the climbs and drops
    // every gate is set at an angle to the line of flight; descending gates tip into a dive
    angle = (i, slope) => ({
      yaw: (rand() < 0.5 ? -1 : 1) * (10 + 15 * rand()) * DEG,
      pitch: slope < -0.08 ? -(8 + 10 * rand()) * DEG : 0,
    });
  } else {
    count = gates ?? 10;
    curve = (t) => {
      const r = 45 + 14 * Math.sin(2 * t + phase);
      return [r * Math.cos(t), 5 + 3 * Math.sin(3 * t + phase), r * Math.sin(t)];
    };
  }
  const list = [];
  for (let i = 0; i < count; i++) {
    const t = (i / count) * Math.PI * 2;
    const p = curve(t);
    const ahead = curve(t + 0.01);
    const dir = v3.sub(ahead, p);
    let n = v3.norm([dir[0], 0, dir[2]]); // horizontal facing
    const slope = dir[1] / Math.hypot(dir[0], dir[2]);
    const { yaw, pitch } = angle(i, slope);
    if (yaw || pitch || follow3d) {
      const c = Math.cos(yaw), s = Math.sin(yaw);
      const h = [n[0] * c + n[2] * s, 0, -n[0] * s + n[2] * c];
      const th = Math.max(-MAX_PITCH, Math.min(MAX_PITCH, (follow3d ? Math.atan(slope) : 0) + pitch));
      n = [h[0] * Math.cos(th), Math.sin(th), h[2] * Math.cos(th)];
    }
    list.push({ id: i, pos: p, normal: n });
  }
  if (follow3d) {
    // start the lap at the lowest gate, set level and square-on, so the launch from
    // the ground can always make it
    const lo = list.reduce((b, g, i) => (g.pos[1] < list[b].pos[1] ? i : b), 0);
    list.push(...list.splice(0, lo));
    list.forEach((g, i) => (g.id = i));
    const t0 = (lo / count) * Math.PI * 2, a = curve(t0 + 0.01), p = list[0].pos;
    list[0].normal = v3.norm([a[0] - p[0], 0, a[2] - p[2]]);
  }
  return { gates: list.map(gateFrame), curve, seed, difficulty };
}

// In-plane axes of a gate: right is horizontal, up completes the frame.
function gateFrame(g) {
  const n = g.normal;
  g.right = v3.norm([n[2], 0, -n[0]]);
  g.up = [
    n[1] * g.right[2] - n[2] * g.right[1],
    n[2] * g.right[0] - n[0] * g.right[2],
    n[0] * g.right[1] - n[1] * g.right[0],
  ];
  return g;
}

// ---------- drone ----------
export const DRONE = {
  maxThrustAcc: 26, // m/s^2 (~2.6 g)
  maxTiltRad: (55 * Math.PI) / 180,
  drag: 0.12, // linear drag coefficient, 1/s
  thrustLag: 0.07, // first-order response time of the thrust vector, s
};

export function makeDrone(course) {
  const g0 = course.gates[0];
  // start a little behind the first gate, on the ground-ish
  const start = v3.sub(g0.pos, v3.scale(g0.normal, 18));
  start[1] = 1.5;
  return {
    pos: start,
    vel: [0, 0, 0],
    thrust: [0, G, 0], // current specific thrust vector (acc), world frame
    yaw: Math.atan2(g0.normal[0], g0.normal[2]),
  };
}

// Limit a desired specific-thrust vector to what the airframe can do.
export function limitThrust(t) {
  let mag = v3.len(t);
  const up = mag > 1e-6 ? t[1] / mag : 1;
  let tilt = Math.acos(Math.max(-1, Math.min(1, up)));
  if (tilt > DRONE.maxTiltRad) {
    // keep the horizontal direction, pull the vector back to the tilt limit
    const h = Math.hypot(t[0], t[2]) || 1e-6;
    const vert = Math.max(t[1], 0.3 * G);
    const hMax = vert * Math.tan(DRONE.maxTiltRad);
    t = [(t[0] / h) * hMax, vert, (t[2] / h) * hMax];
    mag = v3.len(t);
  }
  return v3.clampLen(t, DRONE.maxThrustAcc);
}

// ---------- autopilot ----------
// Pure pursuit along each gate's axis: chase a "carrot" point that slides along
// the line through the gate centre, so the drone lines up and flies straight through.
export const AUTOPILOT = {
  cruise: 13, // m/s
  lookahead: 7, // m
  kVel: 3.2, // 1/s, velocity tracking gain
  turnSlowdown: 0.55, // fraction of speed shed when the next gate needs a sharp turn
};

export function autopilot(drone, course, state) {
  const gate = course.gates[state.target];
  const rel = v3.sub(drone.pos, gate.pos);
  const along = v3.dot(rel, gate.normal); // negative = in front of the gate
  const lateral = v3.sub(rel, v3.scale(gate.normal, along));
  const latDist = v3.len(lateral);

  let carrot;
  if (along > 1.0 || (along > -1.0 && latDist > GATE_INNER * 0.5)) {
    // past the plane without going through (or about to clip the frame): go round to
    // the front for another approach rather than chasing the axis off into the distance
    carrot = v3.sub(gate.pos, v3.scale(gate.normal, 10));
  } else {
    carrot = v3.add(gate.pos, v3.scale(gate.normal, along + AUTOPILOT.lookahead));
  }
  // a dive gate's axis runs into the ground beyond it: never chase a point below 2 m
  if (carrot[1] < 2) carrot = [carrot[0], 2, carrot[2]];

  // slow down for sharp direction changes coming up
  const next = course.gates[(state.target + 1) % course.gates.length];
  const turn = 1 - v3.dot(gate.normal, next.normal); // 0 = straight, 2 = U-turn
  const near = Math.max(0, 1 - Math.abs(along) / 15);
  const speed = AUTOPILOT.cruise * (1 - AUTOPILOT.turnSlowdown * Math.min(1, turn) * near);

  const vDes = v3.scale(v3.norm(v3.sub(carrot, drone.pos)), speed);
  const aDes = v3.scale(v3.sub(vDes, drone.vel), AUTOPILOT.kVel);
  return v3.add(aDes, [0, G, 0]); // desired specific thrust
}

// ---------- racing line ----------
// Plan a smooth closed path through every gate centre, crossing each gate square-on,
// then give it a speed profile the airframe can actually fly.
export const RACING = {
  ds: 0.25, // path sample spacing, m
  tangentScale: 1.15, // Hermite tangent length as a fraction of the gate-to-gate distance
  turnMargin: 0.8, // fraction of the tilt/thrust envelope the planned turns may use
  aAccel: 10.0, // m/s^2 budget for speeding up along the path
  aBrake: 14.0, // m/s^2 budget for braking along the path
  comboMargin: 0.9, // fraction of the envelope turn + along-track acceleration may use together
  vMax: 32, // m/s top speed
  kp: 7.0, // 1/s^2, position error gain
  kd: 4.5, // 1/s, velocity error gain
  launchRate: 9, // m/s per second the speed cap rises at from a standstill
};

// Cubic Hermite point and derivatives on [0, 1].
function hermite(p0, m0, p1, m1, u) {
  const u2 = u * u, u3 = u2 * u;
  const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
  const d00 = 6 * u2 - 6 * u, d10 = 3 * u2 - 4 * u + 1, d01 = -6 * u2 + 6 * u, d11 = 3 * u2 - 2 * u;
  const e00 = 12 * u - 6, e10 = 6 * u - 4, e01 = -12 * u + 6, e11 = 6 * u - 2;
  const mix = (a, b, c, d) => [0, 1, 2].map((k) => a * p0[k] + b * m0[k] + c * p1[k] + d * m1[k]);
  return { p: mix(h00, h10, h01, h11), d1: mix(d00, d10, d01, d11), d2: mix(e00, e10, e01, e11) };
}

export function planRacingLine(course, opts = {}) {
  const P = { ...RACING, ...opts };
  const gates = course.gates;
  const n = gates.length;
  const pts = []; // {p, s, gate} — gate is set on the sample at each gate centre
  // dense parameter sampling per segment, then resample to even arc length
  const raw = [];
  for (let i = 0; i < n; i++) {
    const a = gates[i], b = gates[(i + 1) % n];
    const L = v3.len(v3.sub(b.pos, a.pos)) * P.tangentScale;
    const m0 = v3.scale(a.normal, L), m1 = v3.scale(b.normal, L);
    const steps = 400;
    for (let k = 0; k < steps; k++) raw.push({ p: hermite(a.pos, m0, b.pos, m1, k / steps).p, gate: k === 0 ? i : -1 });
  }
  // cumulative length of the raw polyline
  const rawS = [0];
  for (let i = 1; i <= raw.length; i++) rawS.push(rawS[i - 1] + v3.len(v3.sub(raw[i % raw.length].p, raw[i - 1].p)));
  const total = rawS[raw.length];
  const N = Math.max(8, Math.round(total / P.ds));
  const ds = total / N;
  let j = 0;
  const gateS = gates.map((_, i) => rawS[raw.findIndex((r) => r.gate === i)]);
  for (let k = 0; k < N; k++) {
    const s = k * ds;
    while (rawS[j + 1] < s) j++;
    const f = (s - rawS[j]) / (rawS[j + 1] - rawS[j] || 1);
    const q0 = raw[j].p, q1 = raw[(j + 1) % raw.length].p;
    pts.push({ p: v3.add(q0, v3.scale(v3.sub(q1, q0), f)), s });
  }
  // tangent and curvature vector from central differences (closed loop)
  for (let k = 0; k < N; k++) {
    const a = pts[(k - 1 + N) % N].p, b = pts[k].p, c = pts[(k + 1) % N].p;
    pts[k].t = v3.norm(v3.sub(c, a));
    // second difference / ds^2 = curvature vector (points towards the turn centre)
    pts[k].kv = v3.scale(v3.add(v3.sub(a, v3.scale(b, 2)), c), 1 / (ds * ds));
  }
  // speed profile: turn limit, then forward (accel) and backward (brake) passes, twice round the loop
  // Along-track acceleration and braking share the envelope with the turn (a
  // friction-circle style limit), so a crest or tight corner leaves less for braking.
  const v = pts.map((q) => turnSpeedLimit(q.kv, P));
  for (let pass = 0; pass < 2; pass++) {
    for (let k = 1; k <= N; k++) {
      const i = k % N, h = k - 1;
      const acc = alongLimit(pts[h], v[h], P.aAccel, 1, P);
      v[i] = Math.min(v[i], Math.sqrt(v[h] * v[h] + 2 * acc * ds));
    }
    for (let k = N - 1; k >= 0; k--) {
      const i = k, nx = (k + 1) % N;
      const brk = alongLimit(pts[i], v[nx], P.aBrake, -1, P);
      v[i] = Math.min(v[i], Math.sqrt(v[nx] * v[nx] + 2 * brk * ds));
    }
  }
  // along-track acceleration implied by the profile: dv/dt = v dv/ds
  for (let k = 0; k < N; k++) {
    pts[k].v = v[k];
    const vn = v[(k + 1) % N], vp = v[(k - 1 + N) % N];
    pts[k].at = (v[k] * (vn - vp)) / (2 * ds);
  }
  const lapTime = v.reduce((sum, x) => sum + ds / x, 0);
  return { pts, ds, length: total, gateS, lapTime, params: P };
}

// Fastest speed at which the turn at curvature vector kv fits inside the airframe's
// envelope (tilt limit and max thrust), scaled by a safety margin. Diving turns are
// limited hardest because less vertical thrust means less sideways force at max tilt.
function turnFeasible(kv, v, P) {
  const a = v3.scale(kv, v * v);
  const ty = (a[1] + G) * P.turnMargin; // vertical thrust available
  const th = Math.hypot(a[0], a[2]);
  if (ty <= 0) return th < 1e-6 && a[1] + G >= 0;
  return th <= ty * Math.tan(DRONE.maxTiltRad) && Math.hypot(th, a[1] + G) <= DRONE.maxThrustAcc * P.turnMargin;
}
// Largest along-track acceleration (dir 1) or braking (dir -1), up to budget, that
// fits the envelope together with the turn at speed v. Coasting (drag alone, no thrust
// along the path) is the cheapest way to slow down, so braking searches upwards from it.
function alongLimit(q, v, budget, dir, P) {
  const fits = (x) => {
    const at = dir * x + DRONE.drag * v; // thrust along the path: the speed change plus beating drag
    return thrustFits(v3.add(v3.scale(q.kv, v * v), v3.scale(q.t, at)), P.comboMargin);
  };
  let lo = dir > 0 ? 0 : Math.min(budget, DRONE.drag * v), hi = budget;
  if (fits(hi)) return hi;
  if (!fits(lo)) return dir > 0 ? -DRONE.drag * v : lo; // can't even hold speed: coast
  for (let i = 0; i < 16; i++) {
    const mid = (lo + hi) / 2;
    if (fits(mid)) lo = mid; else hi = mid;
  }
  return lo;
}
// Does path acceleration a (excluding gravity) fit inside a fraction of the envelope?
function thrustFits(a, margin) {
  const ty = a[1] + G, th = Math.hypot(a[0], a[2]);
  if (ty <= 0) return false;
  return th <= ty * Math.tan(DRONE.maxTiltRad * margin) && Math.hypot(th, ty) <= DRONE.maxThrustAcc * margin;
}
function turnSpeedLimit(kv, P) {
  if (turnFeasible(kv, P.vMax, P)) return P.vMax;
  let lo = 0, hi = P.vMax;
  for (let i = 0; i < 24; i++) {
    const mid = (lo + hi) / 2;
    if (turnFeasible(kv, mid, P)) lo = mid; else hi = mid;
  }
  return lo;
}

// Find the closest path sample, searching a window around the last one so the
// tracker never jumps to a different part of the course.
function nearestIndex(plan, pos, from, back = 20, fwd = 160) {
  const N = plan.pts.length;
  let best = from, bestD = Infinity;
  for (let o = -back; o <= fwd; o++) {
    const k = (from + o + N) % N;
    const q = plan.pts[k].p;
    const d = (q[0] - pos[0]) ** 2 + (q[1] - pos[1]) ** 2 + (q[2] - pos[2]) ** 2;
    if (d < bestD) { bestD = d; best = k; }
  }
  return best;
}

// Track the racing line: feedforward from the path (centripetal + along-track
// acceleration + drag), plus PD on position and velocity error.
export function racingPilot(sim) {
  const { drone, plan, track } = sim;
  const P = plan.params;
  const N = plan.pts.length;
  track.idx = nearestIndex(plan, drone.pos, track.idx);
  // look a little ahead to make up for the thrust lag
  const speed = v3.len(drone.vel);
  const ahead = Math.round((speed * DRONE.thrustLag) / plan.ds);
  const q = plan.pts[track.idx];
  const qa = plan.pts[(track.idx + ahead) % N];
  const cap = 3 + P.launchRate * sim.t; // gentle launch from a standstill
  const vRef = Math.min(qa.v, cap);
  const atRef = vRef < qa.v ? Math.min(P.aAccel, P.launchRate) : qa.at;
  const velRef = v3.scale(qa.t, vRef);
  let a = v3.add(v3.scale(qa.kv, vRef * vRef), v3.scale(qa.t, atRef));
  a = v3.add(a, v3.scale(velRef, DRONE.drag));
  a = v3.add(a, v3.scale(v3.sub(q.p, drone.pos), P.kp));
  a = v3.add(a, v3.scale(v3.sub(velRef, drone.vel), P.kd));
  return v3.add(a, [0, G, 0]);
}

// ---------- simulation ----------
export const PILOTS = ['racing', 'pursuit'];

export function makeSim({ seed = 1, gates, pilot = 'racing', difficulty = 'normal' } = {}) {
  const course = makeCourse({ seed, gates, difficulty });
  const drone = makeDrone(course);
  const plan = planRacingLine(course);
  if (course.difficulty !== 'normal') {
    // take off from the ground under the racing line, 18 m before the start gate, so the
    // first gate is a straight climb along the line even when the course curves into it
    const N = plan.pts.length;
    const q = plan.pts[(Math.round((plan.gateS[0] - 18) / plan.ds) + N) % N];
    drone.pos = [q.p[0], 1.5, q.p[2]];
    drone.yaw = Math.atan2(q.t[0], q.t[2]);
  }
  return {
    course,
    drone,
    pilot,
    plan,
    track: { idx: nearestIndex(plan, drone.pos, 0, 0, plan.pts.length) },
    t: 0,
    state: { target: 0, gatesPassed: 0, misses: 0, lap: 0, lapStart: null, laps: [] },
    events: [],
  };
}

export function step(sim, dt) {
  const { drone, course, state } = sim;
  const want = sim.pilot === 'pursuit' ? autopilot(drone, course, state) : racingPilot(sim);
  const cmd = limitThrust(want);

  // first-order lag towards the commanded thrust
  const k = Math.min(1, dt / DRONE.thrustLag);
  drone.thrust = v3.add(drone.thrust, v3.scale(v3.sub(cmd, drone.thrust), k));

  const acc = v3.sub(v3.sub(drone.thrust, [0, G, 0]), v3.scale(drone.vel, DRONE.drag));
  const prev = drone.pos;
  drone.vel = v3.add(drone.vel, v3.scale(acc, dt));
  drone.pos = v3.add(drone.pos, v3.scale(drone.vel, dt));

  if (drone.pos[1] < 0.15) {
    drone.pos[1] = 0.15;
    if (drone.vel[1] < 0) drone.vel[1] = 0;
  }
  const hs = Math.hypot(drone.vel[0], drone.vel[2]);
  if (hs > 0.5) drone.yaw = Math.atan2(drone.vel[0], drone.vel[2]);

  // gate crossing check (segment prev -> pos against the target gate's plane)
  const gate = course.gates[state.target];
  const a0 = v3.dot(v3.sub(prev, gate.pos), gate.normal);
  const a1 = v3.dot(v3.sub(drone.pos, gate.pos), gate.normal);
  if (a0 < 0 && a1 >= 0) {
    const f = a0 / (a0 - a1);
    const hit = v3.add(prev, v3.scale(v3.sub(drone.pos, prev), f));
    const off = v3.sub(hit, gate.pos);
    const inside =
      Math.abs(v3.dot(off, gate.right)) < GATE_INNER / 2 && Math.abs(v3.dot(off, gate.up)) < GATE_INNER / 2;
    if (inside) {
      state.gatesPassed++;
      sim.events.push({ t: sim.t, type: 'gate', gate: gate.id });
      if (gate.id === 0) {
        if (state.lapStart !== null) {
          const lt = sim.t - state.lapStart;
          state.laps.push(lt);
          sim.events.push({ t: sim.t, type: 'lap', time: lt });
        }
        state.lapStart = sim.t;
        state.lap++;
      }
      state.target = (state.target + 1) % course.gates.length;
    } else if (Math.abs(a0) < 3) {
      state.misses++;
      sim.events.push({ t: sim.t, type: 'miss', gate: gate.id });
    }
  }
  sim.t += dt;
}

// Roll/pitch for rendering: tilt the body so its up axis matches the thrust vector.
export function bodyTilt(drone) {
  return v3.norm(drone.thrust);
}

// small deterministic RNG
export function mulberry32(a) {
  return function () {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
