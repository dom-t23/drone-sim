// Race against the autopilot: keeps each racer's gate times and works out the gap
// between them, F1 style. Pure logic, no DOM, so it can be tested in Node.

export function makeRace() {
  return { you: [], rival: [], seen: { you: 0, rival: 0 } };
}

// Pull any new gate events out of each sim into the race's lists of gate times.
export function syncRace(race, youSim, rivalSim) {
  for (const [who, sim] of [['you', youSim], ['rival', rivalSim]]) {
    const ev = sim.events;
    for (; race.seen[who] < ev.length; race.seen[who]++) {
      const e = ev[race.seen[who]];
      if (e.type === 'gate') race[who].push(e.t);
    }
  }
  return race;
}

// Gap at the last gate both racers have passed: positive means you are behind the rival
// by that many seconds, negative means you are ahead. null until both have passed a gate.
// leading is true when you have passed more gates, or the same number sooner.
export function raceGap(race) {
  const n = race.you.length, m = race.rival.length, k = Math.min(n, m);
  const leading = n > m || (n === m && n > 0 && race.you[n - 1] < race.rival[n - 1]);
  return { gap: k ? race.you[k - 1] - race.rival[k - 1] : null, leading, you: n, rival: m };
}
