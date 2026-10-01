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

// A closed loop of gates whose normals follow the direction of travel.
export function makeCourse({ gates = 10, seed = 1 } = {}) {
  const rand = mulberry32(seed);
  const phase = rand() * Math.PI * 2;
  const curve = (t) => {
    const r = 45 + 14 * Math.sin(2 * t + phase);
    return [r * Math.cos(t), 5 + 3 * Math.sin(3 * t + phase), r * Math.sin(t)];
  };
  const list = [];
  for (let i = 0; i < gates; i++) {
    const t = (i / gates) * Math.PI * 2;
    const p = curve(t);
    const ahead = curve(t + 0.01);
    const n = v3.norm([ahead[0] - p[0], 0, ahead[2] - p[2]]); // horizontal facing
    list.push({ id: i, pos: p, normal: n });
  }
  return { gates: list, curve };
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
  if (along > -1.0 && latDist > GATE_INNER * 0.5) {
    // past the plane but outside the opening: go round to the front
    carrot = v3.sub(gate.pos, v3.scale(gate.normal, 10));
  } else {
    carrot = v3.add(gate.pos, v3.scale(gate.normal, along + AUTOPILOT.lookahead));
  }

  // slow down for sharp direction changes coming up
  const next = course.gates[(state.target + 1) % course.gates.length];
  const turn = 1 - v3.dot(gate.normal, next.normal); // 0 = straight, 2 = U-turn
  const near = Math.max(0, 1 - Math.abs(along) / 15);
  const speed = AUTOPILOT.cruise * (1 - AUTOPILOT.turnSlowdown * Math.min(1, turn) * near);

  const vDes = v3.scale(v3.norm(v3.sub(carrot, drone.pos)), speed);
  const aDes = v3.scale(v3.sub(vDes, drone.vel), AUTOPILOT.kVel);
  return v3.add(aDes, [0, G, 0]); // desired specific thrust
}

// ---------- simulation ----------
export function makeSim({ seed = 1, gates = 10 } = {}) {
  const course = makeCourse({ seed, gates });
  const drone = makeDrone(course);
  return {
    course,
    drone,
    t: 0,
    state: { target: 0, gatesPassed: 0, misses: 0, lap: 0, lapStart: null, laps: [] },
    events: [],
  };
}

export function step(sim, dt) {
  const { drone, course, state } = sim;
  const cmd = limitThrust(autopilot(drone, course, state));

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
    const right = [gate.normal[2], 0, -gate.normal[0]];
    const inside =
      Math.abs(v3.dot(off, right)) < GATE_INNER / 2 && Math.abs(off[1]) < GATE_INNER / 2;
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
