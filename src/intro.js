// The intro overlay's content, with no DOM: reads CHANGELOG.md into nights and
// highlights, works out the build number and date, and lists the quick starts.

// One-tap starting points. Each is a URL hash the app already understands.
export const QUICK_STARTS = [
  { id: 'race', title: 'Watch a race', text: 'Five autopilots, three laps, on TV: trackside cameras and slow-motion replays.', hash: '#seed=1&d=normal&field=1&cam=tv' },
  { id: 'fly', title: 'Fly it yourself', text: 'Race the autopilot and your own gold ghost.', hash: '#seed=2&d=normal&pilot=manual' },
  { id: 'night', title: 'Night race', text: 'A hard course under LED gates, headlights on.', hash: '#seed=3&d=hard&field=1&night=1' },
  { id: 'learn', title: 'Watch it learn', text: 'Every lap it learns where it drifted, flies the line tighter, then pushes harder.', hash: '#seed=2&d=normal&learn=1' },
  { id: 'vio', title: 'Find its way', text: 'No GPS: it spots the gates in its own camera pixels and tracks itself with a Kalman filter.', hash: '#seed=3&d=hard&nav=vio&eyes=px' },
];

const HEAD = /^##\s+(\d{4}-\d{2}-\d{2})(?:\s*\(([^)]*)\))?\s*:\s*(.+?)\s*$/;

// Parse the changelog into entries, newest first as written:
// { date: '2026-10-10', run: 'second run' | null, title, bullets: [markdown, ...] }
export function parseChangelog(md) {
  const entries = [];
  let cur = null;
  for (const line of String(md ?? '').split(/\r?\n/)) {
    const h = HEAD.exec(line);
    if (h) {
      cur = { date: h[1], run: h[2] ?? null, title: h[3], bullets: [] };
      entries.push(cur);
      continue;
    }
    if (/^#/.test(line)) { cur = null; continue; } // any other heading ends an entry
    if (!cur) continue;
    const b = /^[-*]\s+(.*)$/.exec(line);
    if (b) cur.bullets.push(b[1].trim());
    else if (/^\s+\S/.test(line) && cur.bullets.length) cur.bullets[cur.bullets.length - 1] += ` ${line.trim()}`;
  }
  return entries;
}

// The first sentence of a bullet (not fooled by "e.g." or "3.5 m").
export function firstSentence(s) {
  const m = /^(.+?(?<!\b(?:e\.g|i\.e|vs))[.!?])(?:\*\*)?\s+(?=[A-Z*`(“"])/.exec(s);
  if (!m) return s;
  // keep a closing ** with its sentence so bold stays balanced
  return s.slice(0, m[0].trimEnd().length);
}

// The headline features of a night: bullets that open with a bold lead, skipping the
// "How it works" and "Tests" notes. A bold lead that is a whole sentence stands alone;
// otherwise the bullet's first sentence is used, or just the lead if that runs long.
export function highlights(entry, n = 3) {
  const out = [];
  for (const b of entry?.bullets ?? []) {
    if (/^(\*\*)?(How it works|Tests|Honest numbers)\b/i.test(b)) continue;
    const m = /^\*\*(.+?)\*\*/.exec(b);
    if (!m) continue;
    const lead = m[1].trim();
    const sentence = firstSentence(b);
    out.push(/[.!?]$/.test(lead) || sentence.length > 170 ? `**${lead}**` : sentence);
    if (out.length >= n) break;
  }
  return out;
}

const MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
export function dateLabel(iso) {
  const m = /^(\d{4})-(\d{2})-(\d{2})$/.exec(iso ?? '');
  if (!m || +m[2] < 1 || +m[2] > 12) return '';
  return `${+m[3]} ${MONTHS[+m[2] - 1]} ${m[1]}`;
}

// Build number = how many runs are in the changelog; the date is the newest run's.
export function buildInfo(entries) {
  if (!entries?.length) return null;
  const first = entries.at(-1).date, last = entries[0].date;
  const days = Math.round((Date.parse(last) - Date.parse(first)) / 864e5) + 1;
  return {
    build: entries.length,
    date: last,
    dateLabel: dateLabel(last),
    since: dateLabel(first),
    days: Number.isFinite(days) ? days : null,
    latest: entries[0],
  };
}

// Show the intro on a plain visit when this build hasn't been seen yet. Shared links
// (anything in the hash) go straight to the action.
export function shouldShowIntro({ hash, seen, info }) {
  if (hash && hash.length > 1) return false;
  if (!info) return !seen;
  return seen !== introKey(info);
}
export const introKey = (info) => `${info.date}#${info.build}`;

const ESC = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' };
// Safe inline markdown: escapes HTML, then **bold**, `code` and [links](...) as text.
export function mdInline(s) {
  return String(s ?? '')
    .replace(/[&<>"']/g, (c) => ESC[c])
    .replace(/\[([^\]]+)\]\([^)]*\)/g, '$1')
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/\*\*(.+?)\*\*/g, '<b>$1</b>');
}
