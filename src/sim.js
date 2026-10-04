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

// Cubic Hermite point on [0, 1].
function hermite(p0, m0, p1, m1, u) {
  const u2 = u * u, u3 = u2 * u;
  const a = 2 * u3 - 3 * u2 + 1, b = u3 - 2 * u2 + u, c = -2 * u3 + 3 * u2, d = u3 - u2;
  return [
    a * p0[0] + b * m0[0] + c * p1[0] + d * m1[0],
    a * p0[1] + b * m0[1] + c * p1[1] + d * m1[1],
    a * p0[2] + b * m0[2] + c * p1[2] + d * m1[2],
  ];
}

export function planRacingLine(course, opts = {}) {
  const P = { ...RACING, ...opts };
  const gates = course.gates;
  const n = gates.length;
  const pts = []; // {p, s, gate} — gate is set on the sample at each gate centre
  // dense parameter sampling per segment, then resample to even arc length
  // optional shape per gate: where the line crosses the opening (dr right, du up, m from
  // the centre) and how long its tangent is there (k, a multiple of tangentScale)
  const shape = P.shape;
  const cross = gates.map((g, i) => {
    const sh = shape?.[i];
    return sh ? v3.add(g.pos, v3.add(v3.scale(g.right, sh.dr), v3.scale(g.up, sh.du))) : g.pos;
  });
  const raw = [];
  for (let i = 0; i < n; i++) {
    const a = gates[i], b = gates[(i + 1) % n], pa = cross[i], pb = cross[(i + 1) % n];
    const L = v3.len(v3.sub(pb, pa)) * P.tangentScale;
    const m0 = v3.scale(a.normal, L * (shape?.[i]?.k ?? 1)), m1 = v3.scale(b.normal, L * (shape?.[(i + 1) % n]?.k ?? 1));
    const steps = P.steps ?? 400;
    for (let k = 0; k < steps; k++) raw.push({ p: hermite(pa, m0, pb, m1, k / steps), gate: k === 0 ? i : -1 });
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
  const E = envelope(P);
  const v = pts.map((q) => turnSpeedLimit(q.kv, P, E));
  for (let pass = 0; pass < 2; pass++) {
    for (let k = 1; k <= N; k++) {
      const i = k % N, h = k - 1;
      const acc = alongLimit(pts[h], v[h], P.aAccel, 1, E);
      v[i] = Math.min(v[i], Math.sqrt(v[h] * v[h] + 2 * acc * ds));
    }
    for (let k = N - 1; k >= 0; k--) {
      const i = k, nx = (k + 1) % N;
      const brk = alongLimit(pts[i], v[nx], P.aBrake, -1, E);
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
  const minY = Math.min(...pts.map((q) => q.p[1]));
  return { pts, ds, length: total, gateS, lapTime, minY, cross, params: P };
}

// Shape the line for speed: for each gate, where it crosses the opening (up to maxOffset
// from the centre, leaving room for tracking error) and how hard it swings in (tangent
// length; the start gate keeps the centre). Coordinate descent on the planned lap time, using quick coarse plans, with
// shrinking steps; candidates that bring the line within 1.8 m of the ground are refused.
export const SHAPING = { maxOffset: 0.6, rounds: 3, k: [0.6, 1.8], minY: 1.8, coarse: { steps: 80, ds: 0.6 } };
const shapeCache = new Map();
// start/rounds/stepScale allow a quick warm-started refinement of an existing shape.
export function optimiseLine(course, key, { start = null, rounds = SHAPING.rounds, stepScale = 1, plan = {} } = {}) {
  if (key && shapeCache.has(key)) return shapeCache.get(key).map((sh) => ({ ...sh }));
  const n = course.gates.length, S = SHAPING;
  const shape = course.gates.map((_, i) => ({ dr: 0, du: 0, k: 1, ...start?.[i] }));
  const cost = () => {
    const p = planRacingLine(course, { ...plan, ...S.coarse, shape });
    return p.minY < S.minY ? Infinity : p.lapTime;
  };
  const step = { dr: 0.3 * stepScale, du: 0.3 * stepScale, k: 0.2 * stepScale };
  const lim = { dr: [-S.maxOffset, S.maxOffset], du: [-S.maxOffset, S.maxOffset], k: S.k };
  let best = cost();
  for (let r = 0; r < rounds; r++) {
    // the start gate is crossed mid-launch, so it keeps the centre line
    for (let i = 1; i < n; i++) {
      for (const key of ['dr', 'du', 'k']) {
        for (const sign of [1, -1]) {
          const old = shape[i][key];
          const v = Math.max(lim[key][0], Math.min(lim[key][1], old + sign * step[key]));
          if (v === old) continue;
          shape[i][key] = v;
          const c = cost();
          if (c < best) { best = c; break; }
          shape[i][key] = old;
        }
      }
    }
    for (const key in step) step[key] *= 0.6;
  }
  if (key) {
    if (shapeCache.size > 64) shapeCache.delete(shapeCache.keys().next().value);
    shapeCache.set(key, shape.map((sh) => ({ ...sh })));
  }
  return shape;
}

// The envelope checks below are plain scalar code: the planner runs them a few hundred
// thousand times per plan, and vision mode re-plans in flight.
function envelope(P) {
  return {
    tanTilt: Math.tan(DRONE.maxTiltRad),
    tanCombo: Math.tan(DRONE.maxTiltRad * P.comboMargin),
    maxTurn: DRONE.maxThrustAcc * P.turnMargin,
    maxCombo: DRONE.maxThrustAcc * P.comboMargin,
    turnMargin: P.turnMargin,
  };
}

// Fastest speed at which the turn at curvature vector kv fits inside the airframe's
// envelope (tilt limit and max thrust), scaled by a safety margin. Diving turns are
// limited hardest because less vertical thrust means less sideways force at max tilt.
function turnFeasible(kv, v, E) {
  const v2 = v * v, ax = kv[0] * v2, az = kv[2] * v2, lift = kv[1] * v2 + G;
  const ty = lift * E.turnMargin; // vertical thrust available
  const th = Math.sqrt(ax * ax + az * az);
  if (ty <= 0) return th < 1e-6 && lift >= 0;
  return th <= ty * E.tanTilt && Math.sqrt(th * th + lift * lift) <= E.maxTurn;
}
// Largest along-track acceleration (dir 1) or braking (dir -1), up to budget, that
// fits the envelope together with the turn at speed v. Coasting (drag alone, no thrust
// along the path) is the cheapest way to slow down, so braking searches upwards from it.
function alongLimit(q, v, budget, dir, E) {
  const v2 = v * v, kx = q.kv[0] * v2, ky = q.kv[1] * v2 + G, kz = q.kv[2] * v2;
  const tx = q.t[0], ty = q.t[1], tz = q.t[2], drag = DRONE.drag * v;
  const fits = (x) => {
    const at = dir * x + drag; // thrust along the path: the speed change plus beating drag
    const ax = kx + tx * at, ay = ky + ty * at, az = kz + tz * at; // gravity included in ay
    if (ay <= 0) return false;
    const th = Math.sqrt(ax * ax + az * az);
    return th <= ay * E.tanCombo && Math.sqrt(th * th + ay * ay) <= E.maxCombo;
  };
  let lo = dir > 0 ? 0 : Math.min(budget, drag), hi = budget;
  if (fits(hi)) return hi;
  if (!fits(lo)) return dir > 0 ? -drag : lo; // can't even hold speed: coast
  for (let i = 0; i < 16; i++) {
    const mid = (lo + hi) / 2;
    if (fits(mid)) lo = mid; else hi = mid;
  }
  return lo;
}
function turnSpeedLimit(kv, P, E) {
  if (turnFeasible(kv, P.vMax, E)) return P.vMax;
  let lo = 0, hi = P.vMax;
  for (let i = 0; i < 24; i++) {
    const mid = (lo + hi) / 2;
    if (turnFeasible(kv, mid, E)) lo = mid; else hi = mid;
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
  const cap = 3 + P.launchRate * (sim.t - sim.launchAt); // gentle launch from a standstill
  const vRef = Math.min(qa.v, cap);
  const atRef = vRef < qa.v ? Math.min(P.aAccel, P.launchRate) : qa.at;
  const velRef = v3.scale(qa.t, vRef);
  let a = v3.add(v3.scale(qa.kv, vRef * vRef), v3.scale(qa.t, atRef));
  a = v3.add(a, v3.scale(velRef, DRONE.drag));
  a = v3.add(a, v3.scale(v3.sub(q.p, drone.pos), P.kp));
  a = v3.add(a, v3.scale(v3.sub(velRef, drone.vel), P.kd));
  return v3.add(a, [0, G, 0]);
}

// ---------- vision ----------
// Vision mode, step 1. The drone no longer knows exactly where the gates are: it has a
// survey map whose gates have since been knocked out of place. A synthetic onboard camera
// (the same one the onboard view renders) finds the inner corners of each gate in view,
// with pixel noise, lost corners and the odd wildly wrong corner. Each detection becomes a
// gate pose by PnP (Gauss-Newton on the reprojection error), is checked against what the
// drone already believes (innovation gating) and is fused into a per-gate estimate. When
// the estimates move, the racing line is re-planned through them in flight. The drone's
// own position and attitude are still taken as known: estimating those is step 2.

export const NAV_MODES = ['truth', 'vision', 'blind'];

export const CAMERA = {
  vfov: 85 * DEG, // vertical field of view, same as the rendered onboard camera
  aspect: 16 / 9,
  heightPx: 480, // image height the pixel figures below refer to
  tilt: 18 * DEG, // tilted up from the body's forward axis, like a real FPV camera
  mount: [0, 0.132, 0.44], // camera position in the body frame, m (matches the 3D model)
  rate: 30, // frames per second
  noisePx: 1.0, // corner noise, pixels, 1 sigma
  dropout: 0.06, // chance any one corner is lost (stands in for occlusion and blur)
  outlierRate: 0.03, // chance a detection has one corner badly wrong
  outlierPx: 30, // how wrong, pixels
  maxRange: 50, // m; further away a gate is too small to find
  minFacing: 0.25, // gates seen more edge-on than this are skipped
};

export const MAP_ERROR = {
  shift: [1.0, 2.6], // m: every gate has moved this far within its own plane...
  vertical: 0.6, // ...with vertical moves scaled down...
  along: 1.0, // ...up to this far forwards or backwards (m)...
  yaw: 5 * DEG, // ...and turned by up to this much
  sigma: 2.0, // m: how far the drone trusts the map (1 sigma)
};

const VISION = {
  rotSigma: 0.15, // rad: weak prior keeping each PnP fit near the mapped orientation
  chi2Fit: 40, // reprojection misfit above which a detection is thrown out
  chi2Gate: 16.3, // innovation gate (chi-square, 3 dof, 99.9%)
  inflate: 2, // measurement covariance inflation, for model mismatch
  resetAfter: 12, // consecutive rejections before a gate's estimate is restarted
  replanShift: 0.1, // m: re-plan once any gate estimate has moved this far...
  replanEvery: 0.25, // s: ...but no more often than this
  lookFirst: 0.5, // s on the pad watching the first gate before launch
};

// Pose of the onboard camera: forward, up and image-right axes plus position, built the
// same way the renderer orients the drone (yaw, then tilt the body up axis onto the thrust).
export function cameraPose(drone) {
  const t = v3.norm(drone.thrust), c = t[1];
  const tilt = (v) => {
    // rotate v by the minimal rotation taking world up onto t (axis up x t)
    const k = [t[2], 0, -t[0]], kv = k[0] * v[0] + k[2] * v[2];
    const kx = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]];
    const f = kv / (1 + c);
    return [v[0] * c + kx[0] + k[0] * f, v[1] * c + kx[1] + k[1] * f, v[2] * c + kx[2] + k[2] * f];
  };
  const cy = Math.cos(drone.yaw), sy = Math.sin(drone.yaw);
  const bx = tilt([cy, 0, -sy]), by = t, bz = tilt([sy, 0, cy]);
  const ca = Math.cos(CAMERA.tilt), sa = Math.sin(CAMERA.tilt), m = CAMERA.mount;
  const mix = (a, wa, b, wb) => [a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb, a[2] * wa + b[2] * wb];
  return {
    pos: v3.add(drone.pos, v3.add(v3.scale(bx, m[0]), mix(by, m[1], bz, m[2]))),
    fwd: mix(by, sa, bz, ca),
    up: mix(by, ca, bz, -sa),
    right: v3.scale(bx, -1), // the camera looks out of the nose, so image right is body -x
  };
}

// Image-plane coordinates (x/z, y/z) of a world point, or null if it is behind the camera.
export function project(cam, p) {
  const dx = p[0] - cam.pos[0], dy = p[1] - cam.pos[1], dz = p[2] - cam.pos[2];
  const z = dx * cam.fwd[0] + dy * cam.fwd[1] + dz * cam.fwd[2];
  if (z < 0.2) return null;
  return [
    (dx * cam.right[0] + dy * cam.right[1] + dz * cam.right[2]) / z,
    (dx * cam.up[0] + dy * cam.up[1] + dz * cam.up[2]) / z,
  ];
}

// Corners of a gate's inner opening, in order round the frame.
const CORNERS = [[-1, 1], [1, 1], [1, -1], [-1, -1]];
export function gateCorners(g, pos = g.pos, right = g.right, up = g.up) {
  const h = GATE_INNER / 2;
  return CORNERS.map(([a, b]) => [
    pos[0] + (right[0] * a + up[0] * b) * h,
    pos[1] + (right[1] * a + up[1] * b) * h,
    pos[2] + (right[2] * a + up[2] * b) * h,
  ]);
}

// Rotate v by the rotation vector w (Rodrigues).
function rotVec(v, w) {
  const th = Math.hypot(w[0], w[1], w[2]);
  if (th < 1e-12) return v;
  const k = [w[0] / th, w[1] / th, w[2] / th], c = Math.cos(th), s = Math.sin(th);
  const kv = (k[0] * v[0] + k[1] * v[1] + k[2] * v[2]) * (1 - c);
  return [
    v[0] * c + (k[1] * v[2] - k[2] * v[1]) * s + k[0] * kv,
    v[1] * c + (k[2] * v[0] - k[0] * v[2]) * s + k[1] * kv,
    v[2] * c + (k[0] * v[1] - k[1] * v[0]) * s + k[2] * kv,
  ];
}

// Gate pose from its four detected corners (uv, image-plane units) by PnP: Levenberg-
// Marquardt on the reprojection error in units of the pixel noise (sigma), starting
// from the current belief. The unknowns are the gate centre and a small rotation away
// from the mapped orientation; a weak prior on that rotation keeps far-off gates, whose
// orientation the camera can barely see, well posed. Returns the centre, its 3x3
// covariance and the final misfit (chi-square), or null if the fit fails.
export function solveGatePose(cam, uv, start, sigma) {
  const residuals = (x) => {
    const w = [x[3], x[4], x[5]];
    const corners = gateCorners(start, [x[0], x[1], x[2]], rotVec(start.right, w), rotVec(start.up, w));
    const r = new Array(11);
    for (let j = 0; j < 4; j++) {
      const q = project(cam, corners[j]);
      if (!q) return null;
      r[2 * j] = (q[0] - uv[j][0]) / sigma;
      r[2 * j + 1] = (q[1] - uv[j][1]) / sigma;
    }
    r[8] = x[3] / VISION.rotSigma; r[9] = x[4] / VISION.rotSigma; r[10] = x[5] / VISION.rotSigma;
    return r;
  };
  const sumsq = (r) => r.reduce((a, b) => a + b * b, 0);
  const normal = (x, r) => {
    // Jacobian by forward differences, then the normal equations H = J'J, g = J'r
    const J = [];
    for (let p = 0; p < 6; p++) {
      const e = p < 3 ? 1e-4 : 1e-5, xp = x.slice();
      xp[p] += e;
      const rp = residuals(xp);
      if (!rp) return null;
      J.push(rp.map((v, i) => (v - r[i]) / e));
    }
    const H = new Array(36), g = new Array(6);
    for (let a = 0; a < 6; a++) {
      g[a] = J[a].reduce((sum, v, i) => sum + v * r[i], 0);
      for (let b = a; b < 6; b++) H[a * 6 + b] = H[b * 6 + a] = J[a].reduce((sum, v, i) => sum + v * J[b][i], 0);
    }
    return { H, g };
  };
  let x = [start.pos[0], start.pos[1], start.pos[2], 0, 0, 0];
  let r = residuals(x);
  if (!r) return null;
  let cost = sumsq(r), lambda = 1e-3;
  for (let it = 0; it < 15; it++) {
    const ne = normal(x, r);
    if (!ne) return null;
    const A = ne.H.slice();
    for (let a = 0; a < 6; a++) A[a * 7] *= 1 + lambda;
    const step = cholSolve(A, 6, ne.g.map((v) => -v));
    if (!step) { lambda *= 10; continue; }
    const xn = x.map((v, i) => v + step[i]), rn = residuals(xn);
    const cn = rn ? sumsq(rn) : Infinity;
    if (cn < cost) {
      const done = cost - cn < 1e-6 * cost + 1e-9 || Math.hypot(...step) < 1e-6;
      x = xn; r = rn; cost = cn; lambda = Math.max(lambda / 3, 1e-9);
      if (done) break;
    } else if ((lambda *= 4) > 1e8) break;
  }
  const ne = normal(x, r);
  if (!ne) return null;
  const cov = [];
  for (let a = 0; a < 3; a++) {
    const col = cholSolve(ne.H.slice(), 6, [0, 1, 2, 3, 4, 5].map((i) => (i === a ? 1 : 0)));
    if (!col) return null;
    cov.push(col[0], col[1], col[2]);
  }
  // cov was filled column by column; the block is symmetric, so rows = columns
  return { pos: [x[0], x[1], x[2]], cov, chi2: cost };
}

// Solve A x = b for a symmetric positive-definite n x n A (flat, row-major) by Cholesky.
function cholSolve(A, n, b) {
  const L = new Array(n * n).fill(0);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = A[i * n + j];
      for (let k = 0; k < j; k++) sum -= L[i * n + k] * L[j * n + k];
      if (i === j) {
        if (!(sum > 1e-300)) return null;
        L[i * n + i] = Math.sqrt(sum);
      } else L[i * n + j] = sum / L[j * n + j];
    }
  }
  const y = new Array(n);
  for (let i = 0; i < n; i++) {
    let sum = b[i];
    for (let k = 0; k < i; k++) sum -= L[i * n + k] * y[k];
    y[i] = sum / L[i * n + i];
  }
  const x = new Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let sum = y[i];
    for (let k = i + 1; k < n; k++) sum -= L[k * n + i] * x[k];
    x[i] = sum / L[i * n + i];
  }
  return x;
}

// 3x3 helpers (flat, row-major).
const m3 = {
  diag: (d) => [d, 0, 0, 0, d, 0, 0, 0, d],
  add: (a, b) => a.map((x, i) => x + b[i]),
  scale: (a, s) => a.map((x) => x * s),
  mulv: (a, v) => [
    a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
    a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
    a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
  ],
  inv: (a) => {
    const c0 = a[4] * a[8] - a[5] * a[7], c1 = a[5] * a[6] - a[3] * a[8], c2 = a[3] * a[7] - a[4] * a[6];
    const det = a[0] * c0 + a[1] * c1 + a[2] * c2;
    if (!(Math.abs(det) > 1e-300)) return null;
    const k = 1 / det;
    return [
      c0 * k, (a[2] * a[7] - a[1] * a[8]) * k, (a[1] * a[5] - a[2] * a[4]) * k,
      c1 * k, (a[0] * a[8] - a[2] * a[6]) * k, (a[2] * a[3] - a[0] * a[5]) * k,
      c2 * k, (a[1] * a[6] - a[0] * a[7]) * k, (a[0] * a[4] - a[1] * a[3]) * k,
    ];
  },
};

// The drone's map: the true course with every gate knocked out of place.
function makeMap(course, seed) {
  const rand = mulberry32((seed * 7919 + 13) | 0), E = MAP_ERROR;
  return {
    gates: course.gates.map((g) => {
      const ang = rand() * Math.PI * 2, mag = E.shift[0] + (E.shift[1] - E.shift[0]) * rand();
      const along = (rand() * 2 - 1) * E.along, yaw = (rand() * 2 - 1) * E.yaw;
      const pos = v3.add(
        v3.add(g.pos, v3.scale(g.right, mag * Math.cos(ang))),
        v3.add(v3.scale(g.up, mag * Math.sin(ang) * E.vertical), v3.scale(g.normal, along))
      );
      pos[1] = Math.max(pos[1], 2.2);
      const c = Math.cos(yaw), s = Math.sin(yaw), n = g.normal;
      const b = gateFrame({ id: g.id, pos, normal: [n[0] * c + n[2] * s, n[1], -n[0] * s + n[2] * c] });
      b.mapPos = pos.slice();
      b.info = m3.diag(1 / (E.sigma * E.sigma)); // information matrix of the estimate
      b.infoVec = m3.mulv(b.info, pos);
      b.cov = m3.diag(E.sigma * E.sigma);
      b.streak = 0;
      return b;
    }),
  };
}

// Standard normal sample (Box-Muller) from a uniform generator.
function gauss(rand) {
  return Math.sqrt(-2 * Math.log(rand() || 1e-12)) * Math.cos(2 * Math.PI * rand());
}

// Fuse a gate-centre measurement z (covariance C) into the belief b, unless it fails the
// innovation gate. Static gates make this an information filter: add up information.
function fuse(b, z, C) {
  const R = m3.scale(C, VISION.inflate), y = v3.sub(z, b.pos);
  const Si = m3.inv(m3.add(b.cov, R));
  if (!Si) return false;
  if (v3.dot(y, m3.mulv(Si, y)) > VISION.chi2Gate) {
    // a run of rejections means the belief itself has gone wrong: start again from here
    if (++b.streak >= VISION.resetAfter) {
      b.info = m3.diag(1 / (MAP_ERROR.sigma * MAP_ERROR.sigma));
      b.infoVec = m3.mulv(b.info, z);
      b.cov = m3.inv(b.info);
      b.pos = z.slice();
      b.streak = 0;
    }
    return false;
  }
  const Ri = m3.inv(R);
  if (!Ri) return false;
  b.streak = 0;
  b.info = m3.add(b.info, Ri);
  b.infoVec = v3.add(b.infoVec, m3.mulv(Ri, z));
  b.cov = m3.inv(b.info);
  b.pos = m3.mulv(b.cov, b.infoVec);
  return true;
}

// One camera frame: detect, fit, gate, fuse, and re-plan if the picture has changed.
function visionFrame(sim) {
  const V = sim.vision, C = CAMERA, cam = cameraPose(sim.drone);
  const tanV = Math.tan(C.vfov / 2), tanH = tanV * C.aspect, px = (2 * tanV) / C.heightPx;
  const dets = [];
  for (const g of sim.course.gates) {
    // what the camera really sees: the true gates
    const d = v3.sub(g.pos, cam.pos), range = v3.len(d);
    if (range > C.maxRange || range < 2) continue;
    if (Math.abs(v3.dot(g.normal, d)) < C.minFacing * range) continue;
    const uv = [];
    for (const p of gateCorners(g)) {
      const q = project(cam, p);
      if (!q || Math.abs(q[0]) > tanH || Math.abs(q[1]) > tanV) break;
      uv.push(q);
    }
    if (uv.length < 4) continue;
    // the detector: noise on every corner, now and then a lost corner or a wild one
    let lost = false;
    for (const q of uv) {
      q[0] += gauss(V.rand) * C.noisePx * px;
      q[1] += gauss(V.rand) * C.noisePx * px;
      if (V.rand() < C.dropout) lost = true;
    }
    if (lost) { V.stats.lost++; continue; }
    const wild = V.rand() < C.outlierRate;
    if (wild) {
      const q = uv[Math.floor(V.rand() * 4)];
      q[0] += (V.rand() < 0.5 ? -1 : 1) * C.outlierPx * px;
      q[1] += (V.rand() < 0.5 ? -1 : 1) * C.outlierPx * px;
    }
    const b = sim.belief.gates[g.id];
    const fit = solveGatePose(cam, uv, b, C.noisePx * px);
    const ok = !!fit && fit.chi2 < VISION.chi2Fit && fuse(b, fit.pos, fit.cov);
    V.stats[ok ? 'accepted' : 'rejected']++;
    if (wild) V.stats.wild++;
    if (wild && !ok) V.stats.wildRejected++;
    // each measured corner as a world point (its ray, at the true corner's depth), so the
    // onboard overlay can draw it against the live view
    const world = gateCorners(g).map((p, j) => {
      const z = v3.dot(v3.sub(p, cam.pos), cam.fwd);
      return v3.add(cam.pos, v3.scale(v3.add(cam.fwd, v3.add(v3.scale(cam.right, uv[j][0]), v3.scale(cam.up, uv[j][1]))), z));
    });
    dets.push({ gate: g.id, uv, world, ok, wild });
  }
  V.frame = { t: sim.t, dets };
  V.stats.frames++;
  // after the first lap every gate has been seen: re-shape the line for the real course
  if (sim.shape && !V.reshaped && sim.state.laps.length >= 1) {
    sim.shape = optimiseLine(sim.belief, null, { start: sim.shape, rounds: 2, stepScale: 0.5, plan: sim.planOpts });
    V.reshaped = true;
    replan(sim);
  }
  // re-plan once the estimates have moved enough from the ones the plan was built on
  if (sim.pilot !== 'pursuit' && sim.t - V.lastPlan >= VISION.replanEvery) {
    const shift = Math.max(...sim.belief.gates.map((b, i) => v3.len(v3.sub(b.pos, V.planned[i]))));
    if (shift >= VISION.replanShift) replan(sim);
  }
}

function replan(sim) {
  const V = sim.vision, s = sim.plan.pts[sim.track.idx].s;
  const plan = planRacingLine(sim.belief, { ...sim.planOpts, shape: sim.shape });
  sim.plan = plan;
  sim.track.idx = nearestIndex(plan, sim.drone.pos, Math.round(s / plan.ds) % plan.pts.length, 60, 60);
  sim.planVersion++;
  V.planned = sim.belief.gates.map((b) => b.pos.slice());
  V.lastPlan = sim.t;
  V.stats.replans++;
}

// ---------- wind ----------
// A steady wind from a random direction plus gusts: each axis is a first-order Gauss-Markov
// process (vertical gusts weaker), capped at two sigma. Wind acts through drag, on the
// air-relative velocity.
export const WIND_MODES = ['off', 'breezy', 'gusty'];
export const WIND = {
  breezy: { mean: 6, gust: 2.5 },
  gusty: { mean: 12, gust: 6 },
  tau: 1.5, // s, gust correlation time
  vertical: 0.4, // vertical gusts relative to horizontal
  observerTau: 0.25, // s, disturbance observer time constant
};

// Planning for the forecast: keep back the share of the envelope the wind can take
// (drag at the mean wind plus two gust sigmas), so there is thrust left to fight it.
export function windMargins(mode) {
  const W = WIND[mode];
  if (!W) return {};
  const reserve = (DRONE.drag * (W.mean + 2 * W.gust)) / DRONE.maxThrustAcc;
  return { turnMargin: RACING.turnMargin - reserve, comboMargin: RACING.comboMargin - reserve };
}

function makeWind(mode, seed) {
  const W = WIND[mode];
  if (!W) return null;
  const rand = mulberry32((seed * 15485863 + 101) | 0), dir = rand() * Math.PI * 2;
  return { mode, rand, mean: [W.mean * Math.cos(dir), 0, W.mean * Math.sin(dir)], gust: [0, 0, 0], sigma: W.gust, now: [W.mean * Math.cos(dir), 0, W.mean * Math.sin(dir)] };
}

function stepWind(w, dt) {
  const a = dt / WIND.tau, b = Math.sqrt(2 * a);
  for (let i = 0; i < 3; i++) {
    const sd = w.sigma * (i === 1 ? WIND.vertical : 1);
    w.gust[i] += -a * w.gust[i] + b * sd * gauss(w.rand);
  }
  // gusts top out at two sigma, the most the planner keeps in reserve
  const h = Math.hypot(w.gust[0], w.gust[2]), hMax = 2 * w.sigma, vMax = 2 * w.sigma * WIND.vertical;
  if (h > hMax) { w.gust[0] *= hMax / h; w.gust[2] *= hMax / h; }
  w.gust[1] = Math.max(-vMax, Math.min(vMax, w.gust[1]));
  for (let i = 0; i < 3; i++) w.now[i] = w.mean[i] + w.gust[i];
}

// ---------- simulation ----------
export const PILOTS = ['racing', 'pursuit'];

// nav: 'truth' flies from the true gate positions; 'vision' starts from a map with every
// gate moved and corrects it with the camera; 'blind' trusts the map and nothing else.
// wind: 'off', 'breezy' or 'gusty'; observer: whether the pilot estimates and cancels it.
export function makeSim({ seed = 1, gates, pilot = 'racing', difficulty = 'normal', nav = 'truth', optimise = true, wind = 'off', observer = true } = {}) {
  const course = makeCourse({ seed, gates, difficulty });
  if (!NAV_MODES.includes(nav)) nav = 'truth';
  const belief = nav === 'truth' ? null : makeMap(course, seed);
  const drone = makeDrone(course);
  // the racing pilot flies an optimised line; vision keeps its shape through re-plans
  if (!WIND_MODES.includes(wind)) wind = 'off';
  const planOpts = windMargins(wind);
  const shape = pilot === 'racing' && optimise
    ? optimiseLine(belief ?? course, `${difficulty}/${seed}/${gates}/${nav !== 'truth'}/${wind}`, { plan: planOpts })
    : null;
  const plan = planRacingLine(belief ?? course, { ...planOpts, shape });
  if (course.difficulty !== 'normal') {
    // start hovering on the racing line, 18 m before the start gate, so the launch is a
    // level run along the line even when the course climbs or curves into the first gate
    const N = plan.pts.length;
    const q = plan.pts[(Math.round((plan.gateS[0] - 18) / plan.ds) + N) % N];
    drone.pos = [q.p[0], Math.max(1.5, q.p[1]), q.p[2]];
    drone.yaw = Math.atan2(q.t[0], q.t[2]);
  }
  return {
    course,
    drone,
    pilot,
    nav,
    belief, // what the drone believes about the gates (null: it knows the truth)
    vision: nav === 'vision' ? {
      rand: mulberry32((seed * 104729 + 7) | 0),
      clock: 0,
      frame: null, // latest detections, for the onboard overlay
      planned: belief.gates.map((b) => b.pos.slice()),
      lastPlan: 0,
      stats: { frames: 0, accepted: 0, rejected: 0, lost: 0, wild: 0, wildRejected: 0, replans: 0 },
    } : null,
    plan,
    shape,
    planOpts,
    wind: makeWind(wind, seed), // null when calm
    // disturbance observer: the unexplained part of the drone's acceleration (m/s^2)
    dist: [0, 0, 0],
    observer,
    planVersion: 0,
    launchAt: nav === 'vision' ? VISION.lookFirst : 0, // vision takes a look before it goes
    track: { idx: nearestIndex(plan, drone.pos, 0, 0, plan.pts.length) },
    t: 0,
    state: { target: 0, gatesPassed: 0, misses: 0, lap: 0, lapStart: null, laps: [] },
    events: [],
  };
}

export function step(sim, dt) {
  const { drone, course, state } = sim;
  const want =
    sim.t < sim.launchAt ? [0, G, 0] // sitting on the pad
      : sim.pilot === 'pursuit' ? autopilot(drone, sim.belief ?? course, state)
        : racingPilot(sim);
  // cancel the disturbance the observer has measured (wind, mostly)
  const cmd = limitThrust(sim.wind && sim.observer && sim.t >= sim.launchAt ? v3.sub(want, sim.dist) : want);

  // first-order lag towards the commanded thrust
  const k = Math.min(1, dt / DRONE.thrustLag);
  drone.thrust = v3.add(drone.thrust, v3.scale(v3.sub(cmd, drone.thrust), k));

  const air = sim.wind ? v3.sub(drone.vel, sim.wind.now) : drone.vel; // air-relative velocity
  const acc = v3.sub(v3.sub(drone.thrust, [0, G, 0]), v3.scale(air, DRONE.drag));
  const prev = drone.pos, vPrev = drone.vel;
  drone.vel = v3.add(drone.vel, v3.scale(acc, dt));
  drone.pos = v3.add(drone.pos, v3.scale(drone.vel, dt));

  let grounded = false;
  if (drone.pos[1] < 0.15) {
    drone.pos[1] = 0.15;
    if (drone.vel[1] < 0) drone.vel[1] = 0;
    grounded = true;
  }
  if (sim.wind) {
    // observer: measured acceleration minus what the still-air model predicts, low-passed
    if (!grounded) {
      const seen = v3.scale(v3.sub(drone.vel, vPrev), 1 / dt);
      const model = v3.sub(v3.sub(drone.thrust, [0, G, 0]), v3.scale(vPrev, DRONE.drag));
      sim.dist = v3.add(sim.dist, v3.scale(v3.sub(v3.sub(seen, model), sim.dist), Math.min(1, dt / WIND.observerTau)));
    }
    stepWind(sim.wind, dt);
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
  if (sim.vision && (sim.vision.clock += dt) >= 1 / CAMERA.rate - 1e-9) {
    sim.vision.clock -= 1 / CAMERA.rate;
    visionFrame(sim);
  }
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
