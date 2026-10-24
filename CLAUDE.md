# Rules for the overnight build runs

This repo is a pure test of how impressive a thing Claude can build on its own. A scheduled Claude run works on it three nights a week (Monday, Thursday, Saturday) with no human present. Dom, the owner, isn't using it to learn (his learning project is `dom-t23/vision-drone-racing`), so there is no teaching in it: optimise only for a result that impresses, both at a glance and on close inspection. Dom may edit this file; the latest version always wins.

The live demo is served by GitHub Pages from `main` (repo root) at https://dom-t23.github.io/drone-sim/. **Every push to `main` goes live**, so main must always work.

## What "impressive" means here

Each night, pick whatever adds the most. Balance two kinds of win, and alternate if one has had several nights in a row:
- **Visible wow**: something a stranger notices within ten seconds of opening the link, that looks and feels polished on a laptop and a phone.
- **Technical depth**: a real step up in the robotics, estimation, planning, vision or simulation that holds up when an engineer reads the code and the changelog.

The best features are both. Prefer depth that shows (an inset, an overlay, a chart, a mode you can toggle) over depth hidden in the code.

## Each run

1. **Orient.** `git pull`. Read `CHANGELOG.md` (what exists), `ROADMAP.md` (the backlog, any epic in progress and its "Next steps") and any `NOTES.md` or GitHub issues from Dom, which take priority. Check for open `epic/*` branches.
2. **Pick tonight's work.** Either one chunk that can be finished and verified tonight, or the next step of a multi-night epic (below). Prefer finishing a half-done item or epic over starting something new. You may add your own ideas to the roadmap.
3. **Build it.** Keep `src/sim.js` (and other simulation modules such as `src/eye.js`) free of DOM and rendering-library code so they stay testable in Node. Rendering lives in `src/main.js` and modules beside it.
4. **Verify. Nothing ships unverified.**
   - `npm test` must pass. Add or extend tests in `test/` for any new simulation behaviour. Never weaken or delete a test just to make it pass. If the autopilot gets faster, tighten the lap-time assertion to lock the gain in.
   - `node --check` every changed browser script.
   - Load the page in a headless browser and check the console and a screenshot of every view you changed, on a desktop and a phone-sized viewport (recipe below). Look at the screenshots: impressive means it has to look right, not just run. If no browser works, be conservative in `main.js`: only APIs in the pinned three.js (0.160.0, from jsDelivr), import map unchanged unless deliberately bumped, and re-read the diff for runtime errors.
   - For any new autopilot or estimation behaviour, check it over a wide sweep (e.g. 30 seeds per difficulty, calm and gusty) before trusting it, and report honestly what fails.
5. **Document.** Add a dated entry at the top of `CHANGELOG.md` (it feeds the intro card): visible differences first, in plain English, each run's headline bullets led with a bold sentence, then "How it works" and "Tests" bullets. Never overstate: numbers in the changelog must come from runs you actually did. Tick or update `ROADMAP.md`, with "Next steps" for anything unfinished. Keep `README.md` accurate.
6. **Ship.** Commit with a clear message and push to `main` (or to the epic branch, see below). Fetch and rebase if the push is rejected. Never force-push.
7. **Final message** (sent to Dom as a push notification): 1–2 lines, no test statistics. What's new and what to try, with a direct link to the demo (a `#...` link straight into the new feature when there is one). On a night that only advanced an unfinished epic, say so in one line.

## Multi-night epics

A feature too big for one night (e.g. a new vehicle, a learned controller, a full attitude estimator) can be built over several runs:
- Work on a branch named `epic/<slug>`, created from `main` and kept up to date by merging `main` into it. Push it every night so the next run can carry on.
- In `ROADMAP.md`, keep an **Epic in progress** section at the top: the goal, the plan as a checklist, what's done, and the exact next step.
- Only merge into `main` when the whole feature works and passes every check above. Main must never carry a half-built feature.
- Don't let an epic go stale: if it isn't converging after about four runs, ship the usable part behind a toggle or shelve it with notes.

## Guardrails

- Static site only: HTML, CSS and ES modules, with no build step and no server. Libraries only from cdn.jsdelivr.net, unpkg.com or cdnjs.cloudflare.com, pinned to exact versions.
- Don't break existing features or shared links (`#seed=...` URLs must keep meaning the same course).
- Keep it fast: the page should load quickly and run smoothly on a phone. Expensive modes must be opt-in.
- If something unrelated to tonight's work is broken (failing tests, console errors), fix that first.
- This is Claude's showcase, credited as built by Claude. Never copy its code, results or claims into `vision-drone-racing`, and never present it as Dom's own work.
- Never commit secrets and never add tracking or analytics.

## Workspace notes (learned on earlier runs)

- **Headless browser:** Chromium is at `/opt/pw-browsers/chromium`. jsDelivr is blocked from the workspace, but the npm registry isn't: `npm i three@0.160.0 playwright` in a scratch folder, serve the repo with a small static server, launch Playwright with `executablePath: '/opt/pw-browsers/chromium'` and `--use-gl=swiftshader --enable-unsafe-swiftshader`, and `page.route('https://cdn.jsdelivr.net/**', ...)` to serve the local `three.module.js`. "GPU stall due to ReadPixels" warnings come from screenshots under SwiftShader and can be ignored.
- **Long jobs:** a single shell command is cut off after 10 minutes, and `npm test` takes about 5 minutes on the 2-CPU workspace. Run long sweeps and the full suite detached (`setsid nohup bash script.sh &`) writing to a file, and poll it.
- Use the scratchpad for throwaway scripts, never the repo.
