// Effects logic with no rendering or audio code, so it can be tested in Node:
// rotor physics for the motor sound and the prop wash particles.
import { v3, G, DRONE } from './sim.js';

// A 5-inch racing quad, for the momentum-theory numbers below.
export const ROTOR = {
  mass: 0.65, // kg, all-up
  radius: 0.0635, // m (5-inch props)
  rho: 1.225, // air density, kg/m^3
  hoverHz: 180, // motor tone at hover; real 5-inch props hum at a few hundred Hz
};
const DISC = 4 * Math.PI * ROTOR.radius ** 2; // total rotor disc area, m^2

// Rotor speed as a fraction of full power. Thrust goes as rotor speed squared, so
// rotor speed goes as the square root of thrust.
export function rotorSpeed(drone) {
  return Math.sqrt(Math.min(1, v3.len(drone.thrust) / DRONE.maxThrustAcc));
}

// Induced velocity of the air pushed down through the props (actuator-disc momentum
// theory): T = 2 rho A v^2, so v = sqrt(T / (2 rho A)). About 7 m/s at hover.
export function inducedVelocity(drone) {
  const T = ROTOR.mass * Math.min(v3.len(drone.thrust), DRONE.maxThrustAcc);
  return Math.sqrt(T / (2 * ROTOR.rho * DISC));
}

// What the motor sound should do this frame: pitch follows rotor speed (pinned at
// ROTOR.hoverHz in a hover), loudness follows rotor speed, and a rush of air grows with
// airspeed. A crash cuts the motors.
export function motorTone(drone, { crashed = false } = {}) {
  const w = rotorSpeed(drone), wHover = Math.sqrt(G / DRONE.maxThrustAcc);
  const speed = v3.len(drone.vel);
  return {
    freq: ROTOR.hoverHz * (crashed ? 0.25 : w / wHover),
    gain: crashed ? 0 : 0.35 + 0.65 * w,
    air: Math.min(1, speed / 35),
  };
}

// ---------- prop wash: a pool of particles in flat arrays ----------
export const WASH = {
  count: 260,
  life: 0.9, // s
  rate: 110, // particles per second while the motors run
  drag: 2.5, // 1/s: the jet slows as it mixes with still air
  floor: 0.04, // m
  spread: 0.85, // fraction of the downward speed turned outward on hitting the ground
};

export function makeWash(n = WASH.count) {
  return { n, pos: new Float32Array(n * 3), vel: new Float32Array(n * 3), age: new Float32Array(n).fill(Infinity), dust: new Float32Array(n), next: 0, carry: 0 };
}

// Emit this frame's particles under the rotors of `drone` (`scale` is how much larger
// than life the model is drawn, `wind` the air's own velocity). `rand` is a 0..1 random source.
export function emitWash(w, drone, dt, { crashed = false, scale = 1, wind = [0, 0, 0], rand = Math.random } = {}) {
  if (crashed) return 0;
  w.carry += WASH.rate * dt;
  const n = Math.floor(w.carry);
  w.carry -= n;
  const down = v3.scale(v3.norm(drone.thrust), -1);
  const vi = inducedVelocity(drone);
  // body axes, as the renderer draws them: nose along yaw, tilted onto the thrust
  const fwd = [Math.sin(drone.yaw), 0, Math.cos(drone.yaw)], side = [Math.cos(drone.yaw), 0, -Math.sin(drone.yaw)];
  const arm = 0.29 * scale; // rotor hubs sit 0.29 model units out on each axis
  for (let k = 0; k < n; k++) {
    const i = w.next;
    w.next = (w.next + 1) % w.n;
    const sx = rand() < 0.5 ? -1 : 1, sz = rand() < 0.5 ? -1 : 1;
    const p = v3.add(drone.pos, v3.add(v3.scale(side, sx * arm), v3.add(v3.scale(fwd, sz * arm), v3.scale(down, 0.1 * scale))));
    const jit = [rand() - 0.5, rand() - 0.5, rand() - 0.5];
    // the jet is still air (or the wind) pushed down at the induced velocity, so a fast
    // drone leaves its wash behind it
    const v = v3.add(v3.add(wind, v3.scale(down, vi * (0.8 + 0.4 * rand()))), v3.scale(jit, 1.6));
    w.pos.set(p, i * 3);
    w.vel.set(v, i * 3);
    w.age[i] = 0;
    w.dust[i] = 0;
  }
  return n;
}

// Move every live particle on. A particle that reaches the ground turns its downward
// speed outward (the ground-effect ring you see when a drone hovers low over dust) and
// is marked as dust. Returns the number of live particles.
export function stepWash(w, dt) {
  let live = 0;
  const f = Math.exp(-WASH.drag * dt);
  for (let i = 0; i < w.n; i++) {
    if (w.age[i] >= WASH.life) continue;
    w.age[i] += dt;
    live++;
    const j = i * 3;
    for (let k = 0; k < 3; k++) {
      w.vel[j + k] *= f;
      w.pos[j + k] += w.vel[j + k] * dt;
    }
    if (w.pos[j + 1] < WASH.floor) {
      w.pos[j + 1] = WASH.floor;
      const vy = w.vel[j + 1];
      if (vy < 0) {
        const vx = w.vel[j], vz = w.vel[j + 2], h = Math.hypot(vx, vz);
        const out = -vy * WASH.spread;
        // outward from where it hit; straight down picks any direction
        const ux = h > 1e-3 ? vx / h : Math.cos(i), uz = h > 1e-3 ? vz / h : Math.sin(i);
        w.vel[j] = vx + ux * out;
        w.vel[j + 2] = vz + uz * out;
        w.vel[j + 1] = -vy * 0.08;
        w.dust[i] = 1;
      }
    }
  }
  return live;
}
