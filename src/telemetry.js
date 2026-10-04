// Telemetry panel: live strip charts of the last 10 s of flight (small multiples, one axis
// each) and gate split times against the best lap. Rendering only; the numbers come from
// the sim, sampled by main.js.

import { G, DRONE, v3 } from './sim.js';
import { makeSplits, splitsOnEvent } from './splits.js';

const WINDOW = 10; // s shown
const EVERY = 0.05; // s between samples
const GRID = 'rgba(255,255,255,.07)';
const ACTUAL = '#e06d25', PLAN = '#2a98c2', LIMIT = '#8fa0b8'; // validated for the dark panel
const GOOD = '#3ddc97', BAD = '#ff5c5c';

// What each chart shows: value(s) from a sample, fixed y range, optional limit line.
const CHARTS = [
  { key: 'speed', title: 'Speed', unit: 'km/h', max: 120, fmt: (v) => v.toFixed(0), plan: 'plan' },
  { key: 'tilt', title: 'Tilt', unit: '°', max: 60, fmt: (v) => v.toFixed(0), limit: (DRONE.maxTiltRad * 180) / Math.PI },
  { key: 'thrust', title: 'Thrust', unit: 'g', max: 3, fmt: (v) => v.toFixed(2), limit: DRONE.maxThrustAcc / G },
  { key: 'err', title: 'Off the line', unit: 'm', max: 1.5, fmt: (v) => v.toFixed(2) },
];

export function makeTelemetry(root) {
  const T = { root, samples: [], next: 0, splits: makeSplits(), events: 0, canvases: [] };
  root.innerHTML = `
    <div class="tm-legend"><span class="sw" style="background:${ACTUAL}"></span>actual
      <span class="sw dash" style="border-color:${PLAN}"></span>plan
      <span class="sw dash" style="border-color:${LIMIT}"></span>limit</div>
    ${CHARTS.map((c) => `<div class="tm-head"><span>${c.title} <i>0–${c.max} ${c.unit}</i></span><b id="tm-${c.key}">–</b></div><canvas class="tm-chart"></canvas>`).join('')}
    <div class="tm-head tm-splits-head"><span>Splits</span><b id="tm-best">best –</b></div>
    <div class="tm-splits" id="tm-splits"></div>`;
  T.canvases = [...root.querySelectorAll('canvas')];
  return T;
}

export function resetTelemetry(T) {
  T.samples = [];
  T.next = 0;
  T.splits = makeSplits();
  T.events = 0;
}

// Call after every sim step: records a sample every EVERY seconds of sim time.
export function sampleTelemetry(T, sim) {
  if (sim.t < T.next) return;
  T.next = sim.t + EVERY;
  const d = sim.drone, thrust = v3.len(d.thrust);
  const racing = sim.pilot === 'racing';
  const q = sim.plan.pts[sim.track.idx];
  T.samples.push({
    t: sim.t,
    speed: v3.len(d.vel) * 3.6,
    plan: racing ? q.v * 3.6 : null,
    tilt: (Math.acos(Math.min(1, d.thrust[1] / (thrust || 1))) * 180) / Math.PI,
    thrust: thrust / G,
    err: racing ? v3.len(v3.sub(q.p, d.pos)) : null,
  });
  while (T.samples.length && T.samples[0].t < sim.t - WINDOW) T.samples.shift();
}

export function drawTelemetry(T, sim) {
  for (; T.events < sim.events.length; T.events++) splitsOnEvent(T.splits, sim.events[T.events]);
  const last = T.samples.at(-1);
  CHARTS.forEach((c, i) => {
    const el = T.root.querySelector(`#tm-${c.key}`);
    el.textContent = last && last[c.key] != null ? `${c.fmt(last[c.key])} ${c.unit}` : '–';
    drawChart(T.canvases[i], c, T.samples, sim.t);
  });
  const S = T.splits;
  T.root.querySelector('#tm-best').textContent = `best ${Number.isFinite(S.bestLap) ? S.bestLap.toFixed(2) + ' s' : '–'}`;
  T.root.querySelector('#tm-splits').innerHTML = S.rows.length
    ? S.rows.slice().reverse().map((r) => {
      const d = r.delta == null ? null : Math.round(r.delta * 100) / 100;
      const delta = d == null ? '' : d === 0 ? '<em>0.00</em>'
        : `<em style="color:${d < 0 ? GOOD : BAD}">${d < 0 ? '−' : '+'}${Math.abs(d).toFixed(2)}</em>`;
      return `<div><span>${r.label}</span><b>${r.time.toFixed(2)} s</b>${delta}</div>`;
    }).join('')
    : '<div class="tm-empty">Splits appear from the second lap</div>';
}

function drawChart(cv, c, samples, now) {
  const dpr = Math.min(devicePixelRatio, 2), w = cv.clientWidth, h = cv.clientHeight;
  if (!w || !h) return;
  if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const x = (t) => w - ((now - t) / WINDOW) * w;
  const y = (v) => h - 2 - (Math.min(v, c.max) / c.max) * (h - 4);
  // recessive grid: halves of the range
  ctx.strokeStyle = GRID;
  ctx.lineWidth = 1;
  for (const f of [0, 0.5, 1]) {
    ctx.beginPath();
    ctx.moveTo(0, y(c.max * f) + 0.5);
    ctx.lineTo(w, y(c.max * f) + 0.5);
    ctx.stroke();
  }
  const line = (key, color, dash) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash(dash);
    ctx.beginPath();
    let on = false;
    for (const s of samples) {
      if (s[key] == null) { on = false; continue; }
      if (on) ctx.lineTo(x(s.t), y(s[key])); else ctx.moveTo(x(s.t), y(s[key]));
      on = true;
    }
    ctx.stroke();
    ctx.setLineDash([]);
  };
  if (c.limit != null) {
    ctx.strokeStyle = LIMIT;
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(0, y(c.limit));
    ctx.lineTo(w, y(c.limit));
    ctx.stroke();
    ctx.setLineDash([]);
  }
  line(c.key, ACTUAL, []);
  if (c.plan) line(c.plan, PLAN, [4, 3]); // on top: the drone tracks it closely
}
