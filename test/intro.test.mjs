// Checks for the intro overlay's content (src/intro.js) and its wiring to the page.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as intro from '../src/intro.js';
import { DIFFICULTIES, NAV_MODES, WIND_MODES, PILOTS } from '../src/sim.js';

const {
  QUICK_STARTS, parseChangelog, firstSentence, highlights, dateLabel, buildInfo, shouldShowIntro, introKey, mdInline,
} = intro;
const read = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const CHANGELOG = read('../CHANGELOG.md');

test('the real changelog parses into dated runs, newest first, each with notes', () => {
  const entries = parseChangelog(CHANGELOG);
  assert.ok(entries.length >= 15, `only ${entries.length} runs parsed`);
  assert.equal((CHANGELOG.match(/^## /gm) ?? []).length, entries.length, 'a run heading did not parse');
  for (let i = 0; i < entries.length; i++) {
    const e = entries[i];
    assert.match(e.date, /^\d{4}-\d{2}-\d{2}$/);
    assert.ok(e.title.length > 3, `run ${e.date} has no title`);
    assert.ok(e.bullets.length >= 2, `run ${e.date} has ${e.bullets.length} bullets`);
    if (i > 0) assert.ok(e.date <= entries[i - 1].date, `${e.date} is listed above an older run`);
  }
  assert.equal(entries.at(-1).title, 'First flight');
});

test('the newest run has headline features for the intro', () => {
  const [latest] = parseChangelog(CHANGELOG);
  const hl = highlights(latest, 3);
  assert.ok(hl.length >= 2, 'write the top bullets of tonight\'s entry with a bold lead');
  for (const h of hl) {
    assert.ok(h.length <= 200, `headline too long: ${h}`);
    assert.doesNotMatch(h, /^(How it works|Tests)/);
    assert.equal((h.match(/\*\*/g) ?? []).length % 2, 0, `unbalanced bold: ${h}`);
  }
});

test('headlines: bold sentence alone, else the first sentence, skipping notes', () => {
  const e = parseChangelog([
    '# Changelog', '',
    '## 2026-10-12 (second run): Example',
    '- **A whole sentence.** More detail here.',
    '- **Label**: the rest, e.g. `#seed=1`. Second sentence.',
    '- A plain bullet with no bold lead.',
    '- How it works: **secret** sauce.',
    '- **Tests**: none.',
    '- **Long label** ' + 'word '.repeat(60) + 'end. Next.',
    '  continued on the next line.',
    '## 2026-10-11: Older',
    '- **Older.** Text.',
  ].join('\n'));
  assert.equal(e.length, 2);
  assert.equal(e[0].run, 'second run');
  assert.equal(e[1].run, null);
  assert.deepEqual(highlights(e[0], 5), [
    '**A whole sentence.**',
    '**Label**: the rest, e.g. `#seed=1`.',
    '**Long label**',
  ]);
  assert.match(e[0].bullets.at(-1), /continued on the next line\.$/);
  assert.equal(highlights(e[0], 1).length, 1);
  assert.deepEqual(highlights(null), []);
  assert.equal(firstSentence('One 3.5 m step. Two.'), 'One 3.5 m step.');
  assert.equal(firstSentence('no full stop'), 'no full stop');
});

test('build info, dates and when to show the intro', () => {
  assert.equal(dateLabel('2026-10-01'), '1 Oct 2026');
  assert.equal(dateLabel('2026-13-01'), '');
  assert.equal(dateLabel(undefined), '');
  assert.equal(buildInfo([]), null);
  const entries = parseChangelog(CHANGELOG);
  const info = buildInfo(entries);
  assert.equal(info.build, entries.length);
  assert.equal(info.date, entries[0].date);
  assert.equal(info.since, '1 Oct 2026');
  assert.ok(info.days >= 12);
  const key = introKey(info);
  assert.equal(shouldShowIntro({ hash: '', seen: null, info: null }), true, 'first visit');
  assert.equal(shouldShowIntro({ hash: '', seen: null, info }), true);
  assert.equal(shouldShowIntro({ hash: '', seen: key, info }), false, 'already seen this build');
  assert.equal(shouldShowIntro({ hash: '', seen: key, info: null }), false, 'seen, notes not loaded');
  assert.equal(shouldShowIntro({ hash: '', seen: '2026-10-01#1', info }), true, 'a new build shipped');
  assert.equal(shouldShowIntro({ hash: '#seed=3&d=hard', seen: null, info }), false, 'shared links go straight in');
  assert.equal(shouldShowIntro({ hash: '#', seen: null, info }), true);
});

test('inline markdown is escaped before formatting', () => {
  assert.equal(mdInline('**Bold** and `code` <script>alert("x")</script> & [link](http://x)'),
    '<b>Bold</b> and <code>code</code> &lt;script&gt;alert(&quot;x&quot;)&lt;/script&gt; &amp; link');
  assert.equal(mdInline(null), '');
});

test('every quick start is a valid shareable course', () => {
  assert.ok(QUICK_STARTS.length >= 3);
  const ids = new Set();
  for (const q of QUICK_STARTS) {
    assert.ok(!ids.has(q.id)); ids.add(q.id);
    assert.ok(q.title && q.text);
    const p = new URLSearchParams(q.hash.replace(/^#/, ''));
    const known = new Set(['seed', 'd', 'nav', 'wind', 'pilot', 'field', 'night']);
    for (const k of p.keys()) assert.ok(known.has(k), `${q.id}: unknown key ${k}`);
    assert.ok(Number.isInteger(+p.get('seed')) && +p.get('seed') > 0);
    assert.ok(DIFFICULTIES.includes(p.get('d')));
    if (p.has('nav')) assert.ok(NAV_MODES.includes(p.get('nav')));
    if (p.has('wind')) assert.ok(WIND_MODES.includes(p.get('wind')));
    if (p.has('pilot')) assert.ok(PILOTS.includes(p.get('pilot')));
    if (p.has('field')) assert.equal(p.get('field'), '1');
    if (p.has('night')) assert.equal(p.get('night'), '1');
  }
});

test('the intro is wired to the page: imports exist and every element is there', () => {
  const html = read('../index.html'), landing = read('../src/landing.js'), main = read('../src/main.js');
  const imported = /import \{([^}]+)\} from '\.\/intro\.js'/.exec(landing)[1].split(',').map((s) => s.trim()).filter(Boolean);
  for (const name of imported) assert.ok(name in intro, `intro.js does not export ${name}`);
  const ids = [...landing.matchAll(/\$\('([\w-]+)'\)/g)].map((m) => m[1]);
  assert.ok(ids.length >= 8);
  for (const id of ids) assert.match(html, new RegExp(`id="${id}"`), `index.html has no #${id}`);
  assert.match(main, /import \{ makeLanding \} from '\.\/landing\.js'/);
  assert.match(main, /landing\.boot\(initialHash\)/);
});
