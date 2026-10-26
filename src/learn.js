// The Learn panel's content, with no DOM: what the drone has learned from its own laps
// (iterative learning control in sim.js), as numbers, a lap note and a small chart drawn
// on whatever 2D context it is given.

// Lap by lap: time, tracking error (RMS, m), worst miss of a planned gate crossing (m),
// and the confidence it finished the lap with (0..1).
export function learnSummary(sim) {
  const L = sim.learn;
  if (!L) return null;
  const times = sim.state.laps;
  const laps = L.laps.map((l, i) => ({ n: i + 1, time: times[i], rms: l.rms, gate: l.gate, conf: l.conf, bad: l.bad }))
    .filter((l) => l.time != null);
  const first = laps[0] ?? null, last = laps.at(-1) ?? null;
  const best = laps.length ? Math.min(...laps.map((l) => l.time)) : null;
  return {
    laps,
    first: first?.time ?? null,
    best,
    gain: first && best != null ? first.time - best : 0, // s saved on the first lap's time
    conf: L.conf,
    rmsFirst: first?.rms ?? null,
    rmsLast: last?.rms ?? null,
  };
}

// A toast for the lap just learned from.
export function learnNote(e, lapTime, first) {
  if (e.rms == null) return 'Too fast: backing off'; // a crash
  const vs = first != null && lapTime != null && lapTime < first - 0.005 ? ` (−${(first - lapTime).toFixed(2)})` : '';
  const what = e.step > 0 ? 'pushing harder' : e.step < 0 ? 'backing off' : e.conf >= 1 ? 'flat out' : `${Math.round(e.rms * 100)} cm off line`;
  return `Lap ${lapTime.toFixed(2)} s${vs} · ${what}`;
}

// How confident it is, as a short label.
export function confLabel(conf) {
  return conf >= 1 ? 'flat out' : conf > 0 ? `${Math.round(conf * 100)}%` : 'learning';
}

// The chart: a bar per lap (its time, against a dashed line at lap 1's), coloured from grey
// (no confidence) to orange (full), and the tracking error as a cyan line on its own scale.
export function drawLearnChart(ctx, w, h, sum, { maxLaps = 12 } = {}) {
  ctx.clearRect(0, 0, w, h);
  if (!sum || !sum.laps.length) {
    ctx.fillStyle = '#8fa0b8';
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText('First lap: watching its own errors…', 2, h / 2 + 4);
    return;
  }
  const laps = sum.laps.slice(-maxLaps), n = laps.length;
  const tMax = Math.max(...laps.map((l) => l.time)), tMin = Math.min(...laps.map((l) => l.time));
  const lo = tMin - Math.max(0.3, (tMax - tMin) * 0.6), hi = tMax + 0.05;
  const y = (t) => h - 2 - ((t - lo) / (hi - lo)) * (h - 6);
  const slot = w / maxLaps, bw = Math.max(4, slot * 0.62);
  laps.forEach((l, i) => {
    const x = i * slot + (slot - bw) / 2, top = y(l.time);
    const c = Math.max(0, Math.min(1, l.conf));
    ctx.fillStyle = l.bad ? '#ff7a7a' : `rgb(${Math.round(110 + 145 * c)},${Math.round(125 + 13 * c)},${Math.round(150 - 89 * c)})`;
    ctx.fillRect(x, top, bw, h - top);
  });
  // lap 1's time, dashed
  if (sum.first != null && sum.first >= lo) {
    ctx.strokeStyle = 'rgba(232,237,245,.55)';
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(0, y(sum.first) + 0.5);
    ctx.lineTo(w, y(sum.first) + 0.5);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  // tracking error, 0 at the bottom, the worst lap shown at the top
  const eMax = Math.max(0.05, ...laps.map((l) => l.rms));
  ctx.strokeStyle = '#4dd2ff';
  ctx.fillStyle = '#4dd2ff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  laps.forEach((l, i) => {
    const x = i * slot + slot / 2, yy = h - 3 - (l.rms / eMax) * (h - 8);
    if (i) ctx.lineTo(x, yy); else ctx.moveTo(x, yy);
  });
  ctx.stroke();
  laps.forEach((l, i) => {
    ctx.beginPath();
    ctx.arc(i * slot + slot / 2, h - 3 - (l.rms / eMax) * (h - 8), 2, 0, Math.PI * 2);
    ctx.fill();
  });
  ctx.lineWidth = 1;
  return n;
}
