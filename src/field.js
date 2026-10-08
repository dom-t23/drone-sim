// Multi-drone races: a field of autopilots with different controllers and tunings, and an
// F1-style live leaderboard. Pure logic, no DOM, so it is tested in Node.

// The entrants. Each one is a makeSim() recipe plus a name and a colour. They differ in
// how they fly, not in the airframe: same drone, same course, same wind.
export const ROSTER = [
  { id: 'line', name: 'Racing line', color: '#ff5c5c', blurb: 'optimised racing line', opts: { pilot: 'racing', nav: 'truth' } },
  { id: 'vision', name: 'Vision', color: '#7aa2ff', blurb: 'racing line, but finds the gates with its camera', opts: { pilot: 'racing', nav: 'vision' } },
  { id: 'smooth', name: 'Smooth', color: '#2ec4b6', blurb: 'unshaped line, gentler on the throttle', opts: { pilot: 'racing', nav: 'truth', optimise: false, tune: { aAccel: 7, aBrake: 10 } } },
  { id: 'steady', name: 'Steady', color: '#b07cff', blurb: 'racing line using 60% of the envelope in turns', opts: { pilot: 'racing', nav: 'truth', tune: { turnMargin: 0.6, comboMargin: 0.7, vMax: 26 } } },
  { id: 'pursuit', name: 'Pursuit', color: '#d6dde8', blurb: 'the original gate-axis chaser', opts: { pilot: 'pursuit', nav: 'truth' } },
];

// The roster minus whichever entrant flies exactly like the hero (the drone you watch or
// fly), so nobody races an identical twin.
export function rosterFor(hero) {
  return ROSTER.filter((r) => !(r.opts.pilot === hero.pilot && r.opts.nav === hero.nav && !r.opts.tune && r.opts.optimise !== false));
}

// entries: [{ id, name, sim }]. Gate times are pulled out of each sim's events.
export function makeField(entries) {
  return { entries: entries.map((e) => ({ ...e, times: [], seen: 0 })) };
}

export function syncField(field) {
  for (const e of field.entries) {
    const ev = e.sim.events;
    for (; e.seen < ev.length; e.seen++) if (ev[e.seen].type === 'gate') e.times.push(ev[e.seen].t);
  }
  return field;
}

// Standings, like a timing tower: whoever has passed more gates is ahead; on the same gate,
// whoever got there first. gap is to the leader at the gate this entrant last passed
// (seconds), or lapped (whole laps down). Before anyone has a gate, all are level.
export function standings(field, gatesPerLap) {
  const rows = field.entries.map((e) => {
    const n = e.times.length, laps = e.sim.state.laps;
    return {
      id: e.id, name: e.name, gates: n, at: n ? e.times[n - 1] : Infinity,
      lap: e.sim.state.lap, best: laps.length ? Math.min(...laps) : null, last: laps.at(-1) ?? null,
      crashed: !!e.sim.crash,
    };
  });
  rows.sort((a, b) => b.gates - a.gates || a.at - b.at);
  const lead = rows[0], leader = field.entries.find((e) => e.id === lead.id);
  const fastest = Math.min(...rows.map((r) => r.best ?? Infinity));
  rows.forEach((r, i) => {
    r.pos = i + 1;
    r.fastest = r.best != null && r.best === fastest;
    r.gap = null;
    r.lapped = 0;
    if (i === 0 || !r.gates) return;
    const down = Math.floor((lead.gates - r.gates) / gatesPerLap);
    // lapped only if the leader had also passed this gate a whole lap earlier
    if (down >= 1) r.lapped = down;
    else r.gap = r.at - leader.times[r.gates - 1];
  });
  return rows;
}

// A gap as the tower shows it: "+1.23", "+1 lap", or "" for the leader / no time yet.
export function gapText(r) {
  if (r.lapped) return `+${r.lapped} lap${r.lapped > 1 ? 's' : ''}`;
  return r.gap == null ? '' : `+${r.gap.toFixed(2)}`;
}
