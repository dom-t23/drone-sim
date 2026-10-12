// The intro overlay and build badge (DOM). Content and decisions live in intro.js.
import { QUICK_STARTS, parseChangelog, buildInfo, highlights, shouldShowIntro, introKey, mdInline, dateLabel } from './intro.js';

const SEEN = 'drone-sim:intro-seen';
const store = {
  get() { try { return localStorage.getItem(SEEN); } catch { return null; } },
  set(v) { try { localStorage.setItem(SEEN, v); } catch { /* private mode: show again next time */ } },
};
const short = (iso) => dateLabel(iso).replace(/ \d{4}$/, '');

// onStart(hash) is called when a quick start is picked.
export function makeLanding({ onStart }) {
  const $ = (id) => document.getElementById(id);
  const el = $('intro');
  let info = null, open = false, lastFocus = null, dismissed = false;

  for (const q of QUICK_STARTS) {
    const b = document.createElement('button');
    b.className = 'intro-start';
    b.innerHTML = `<b>${mdInline(q.title)}</b><span>${mdInline(q.text)}</span>`;
    b.onclick = () => { close(); onStart(q.hash); };
    $('intro-starts').append(b);
  }
  $('intro-go').onclick = () => close();
  $('intro-x').onclick = () => close();
  $('b-about').onclick = () => show();
  el.addEventListener('click', (e) => { if (e.target === el) close(); });

  function show() {
    if (open) return;
    lastFocus = document.activeElement;
    el.hidden = false;
    open = true;
    $('intro-go').focus({ preventScroll: true });
  }
  function close() {
    if (!open) return;
    el.hidden = true;
    open = false;
    dismissed = true;
    if (info) store.set(introKey(info));
    if (lastFocus && lastFocus !== document.body) lastFocus.focus?.({ preventScroll: true });
  }

  function render() {
    if (!info) return;
    $('b-about').textContent = `Build ${info.build} · ${short(info.date)} · what's new`;
    $('intro-date').textContent = info.dateLabel;
    $('intro-latest').innerHTML = mdInline(info.latest.title);
    $('intro-hl').innerHTML = highlights(info.latest, 3).map((h) => `<li>${mdInline(h)}</li>`).join('');
    $('intro-recent').innerHTML = info.entries.slice(1, 5)
      .map((e) => `<li><time datetime="${e.date}">${short(e.date)}</time>${mdInline(e.title)}</li>`).join('');
    $('intro-count').textContent = `Build ${info.build}: ${info.build} overnight runs` +
      (info.days ? ` in ${info.days} days since ${info.since}.` : '.');
    $('intro-new').hidden = false;
  }

  // Decide at load: a plain visit shows the intro the first time, and again whenever
  // a new build has shipped since it was last closed. Shared links go straight in.
  function boot(hash) {
    const seen = store.get();
    if (shouldShowIntro({ hash, seen, info: null })) show();
    fetch('CHANGELOG.md', { cache: 'no-cache' })
      .then((r) => (r.ok ? r.text() : Promise.reject(new Error(r.status))))
      .then((md) => {
        const entries = parseChangelog(md);
        const b = buildInfo(entries);
        if (!b) return;
        info = { ...b, entries };
        render();
        if (dismissed) store.set(introKey(info)); // closed before the notes arrived
        else if (shouldShowIntro({ hash, seen, info })) show();
      })
      .catch(() => { /* offline or missing: the intro still works without the build notes */ });
  }

  return { boot, show, close, isOpen: () => open };
}
