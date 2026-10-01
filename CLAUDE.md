# Rules for the overnight build runs

This repo is a showcase of what Claude can build autonomously. A scheduled Claude run works on it three nights a week (Monday, Thursday, Saturday) with no human present. Dom, the owner, wants something fun, impressive and live. He isn't using this to learn, so optimise for a great result, not for teaching. Dom may edit this file; the latest version always wins.

The live demo is served by GitHub Pages from `main` (repo root) at https://dom-t23.github.io/drone-sim/. **Every push to `main` goes live**, so main must always work.

## Each run

1. **Orient.** `git pull`. Read `CHANGELOG.md` (what exists), `ROADMAP.md` (the backlog and any "Next steps" notes) and any `NOTES.md` or GitHub issues from Dom, which take priority.
2. **Pick one meaningful chunk** that can be finished and verified tonight. Choose whatever adds the most: a visible feature, a real technical step up, or fixing anything broken. You may add your own ideas to the roadmap. Prefer finishing a half-done item over starting a new one.
3. **Build it.** Keep `src/sim.js` free of rendering and DOM code so it stays testable in Node. Rendering lives in `src/main.js` and new modules beside it.
4. **Verify. Nothing ships unverified.**
   - `npm test` must pass. Add or extend tests in `test/` for any new simulation behaviour (autopilot performance, physics, estimation accuracy and so on). Never weaken or delete a test just to make it pass. If the autopilot gets faster, tighten the lap-time assertion to lock the gain in.
   - `node --check` every changed browser script.
   - The workspace usually has no browser, so rendering changes can't be eyeballed. Be conservative in `main.js`: use only APIs that exist in the pinned three.js version (0.160.0, loaded from jsDelivr), keep the import map unchanged unless you deliberately bump it, and re-read your diff for runtime errors (undefined names, wrong method names, null elements). If a headless browser is available, use it to load the page and check the console.
5. **Document.** Add a dated entry at the top of `CHANGELOG.md` describing what changed in plain English, the visible differences first. Tick or update `ROADMAP.md`, leaving "Next steps" notes for anything unfinished. Keep `README.md` accurate.
6. **Ship.** Commit with a clear message and push to `main`. Fetch and rebase if the push is rejected. Never force-push.
7. **Final message** (sent to Dom as a push notification): 2–3 lines. What was added tonight, what to try in the demo, and the live link.

## Guardrails

- Static site only: HTML, CSS and ES modules, with no build step and no server. Libraries only from cdn.jsdelivr.net, unpkg.com or cdnjs.cloudflare.com, pinned to exact versions.
- Don't break existing features. If a change can't be made safe tonight, leave it on a branch, note it in the roadmap and ship nothing broken.
- Keep it fast: the page should load quickly and run smoothly on a phone.
- If something unrelated to tonight's work is broken (failing tests, console errors), fix that first.
- Never commit secrets and never add tracking or analytics.
