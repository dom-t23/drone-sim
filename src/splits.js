// Gate split times, F1 style: the time from the start of the lap to each gate, compared
// with the same gate on the best lap so far. Pure logic with no DOM, so it is tested in Node.

export function makeSplits() {
  return { lapStart: null, current: [], best: null, bestLap: Infinity, rows: [] };
}

// Feed every sim event through this. Rows (newest last) are { label, time, delta }, where
// delta is against the best lap (negative = faster) or null if there is nothing to compare.
export function splitsOnEvent(S, e, keep = 5) {
  if (e.type !== 'gate') return;
  const push = (row) => {
    S.rows.push(row);
    if (S.rows.length > keep) S.rows.shift();
  };
  if (e.gate === 0) {
    if (S.lapStart !== null) {
      const lap = e.t - S.lapStart;
      push({ label: 'Lap', time: lap, delta: Number.isFinite(S.bestLap) ? lap - S.bestLap : null });
      if (lap < S.bestLap) {
        S.bestLap = lap;
        S.best = S.current.slice();
      }
    }
    S.lapStart = e.t;
    S.current = [];
    return;
  }
  if (S.lapStart === null) return;
  const time = e.t - S.lapStart;
  S.current[e.gate] = time;
  const ref = S.best?.[e.gate];
  push({ label: `Gate ${e.gate + 1}`, time, delta: ref == null ? null : time - ref });
}
