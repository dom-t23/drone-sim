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
// laps: race length. The race clock starts at the green light (start, in sim seconds).
export const RACE = { laps: 3, timeout: 30 };
export function makeField(entries, { laps = RACE.laps, start = 0 } = {}) {
  return {
    entries: entries.map((e) => ({ ...e, times: [], seen: 0, finish: null, bumps: 0, crashes: 0 })),
    laps, start, flag: null, // flag: sim time the winner took the chequered flag
    order: [], // ids in the order they took the flag
    over: false, // results are final
  };
}

// Pull new events out of each sim. The winner is the first to complete `laps` laps
// (that is, to cross the start gate laps + 1 times: the first crossing starts lap 1).
// Once the flag is out, everyone else finishes the next time they cross the line, with
// however many laps they've done, as in real racing. Results are final when all have
// finished, or RACE.timeout seconds after the flag (anyone still out is "DNF").
export function syncField(field, gatesPerLap) {
  for (const e of field.entries) {
    const ev = e.sim.events;
    for (; e.seen < ev.length; e.seen++) {
      const x = ev[e.seen];
      if (x.type === 'bump') e.bumps++;
      if (x.type === 'crash') e.crashes++;
      if (x.type !== 'gate') continue;
      e.times.push(x.t);
      if (!gatesPerLap || e.finish || field.over || x.gate !== 0) continue;
      const laps = (e.times.length - 1) / gatesPerLap;
      if (laps >= field.laps || (field.flag != null && x.t > field.flag && laps >= 1)) {
        e.finish = { t: x.t, laps, time: x.t - field.start };
        if (field.flag == null) field.flag = x.t;
        field.order.push(e.id);
      }
    }
  }
  if (field.flag != null && !field.over) {
    const now = Math.max(...field.entries.map((e) => e.sim.t));
    field.over = field.entries.every((e) => e.finish) || now - field.flag >= RACE.timeout;
  }
  return field;
}

// The classification: finishers in the order they took the flag (more laps always ahead),
// then anyone who didn't finish, by gates passed. gap is to the winner: seconds on the
// same lap count, else whole laps down.
export function results(field) {
  const byId = Object.fromEntries(field.entries.map((e) => [e.id, e]));
  const done = field.order.map((id) => byId[id]).sort((a, b) => b.finish.laps - a.finish.laps || a.finish.t - b.finish.t);
  const rest = field.entries.filter((e) => !e.finish).sort((a, b) => b.times.length - a.times.length || (a.times.at(-1) ?? Infinity) - (b.times.at(-1) ?? Infinity));
  const win = done[0];
  return [...done, ...rest].map((e, i) => {
    const laps = e.sim.state.laps;
    const row = {
      pos: i + 1, id: e.id, name: e.name, finished: !!e.finish,
      laps: e.finish ? e.finish.laps : Math.max(0, Math.floor((e.times.length - 1) / (e.sim.course.gates.length))),
      time: e.finish ? e.finish.time : null,
      best: laps.length ? Math.min(...laps) : null,
      gap: null, down: 0, crashes: e.crashes, bumps: e.bumps,
    };
    if (e.finish && win && e !== win) {
      row.down = win.finish.laps - e.finish.laps;
      if (!row.down) row.gap = e.finish.time - win.finish.time;
    }
    return row;
  });
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
