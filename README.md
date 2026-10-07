# Drone Sim

A self-flying racing drone in the browser. A quadrotor flies itself around a procedurally generated gate course, with a chase camera, an onboard FPV view, lap timing and a light trail.

**Live demo:** https://dom-t23.github.io/drone-sim/

This is an experiment in what Claude can build on its own. A scheduled Claude run picks up the project three nights a week (Monday, Thursday and Saturday), adds the next feature from `ROADMAP.md` or an idea of its own, tests it, and pushes. GitHub Pages redeploys the demo automatically. `CHANGELOG.md` shows what each night added.

## Controls

| Control | Action |
|---|---|
| **Camera** (or `C`) | Chase → orbit → onboard |
| **Speed** | Simulation time 1× / 2× / 4× |
| **Pilot** (or `P`) | Racing line → original axis pursuit → **you** (race the autopilot) |
| **Course** (or `D`) | Difficulty: easy → normal → hard |
| **Nav** (or `V`) | Ground truth → vision → map only |
| **Wind** (or `W`) | Off → breezy → gusty |
| **Telemetry** (or `T`) | Live charts and gate splits |
| **Ghost** (or `G`) | Show or hide the gold ghost of the best lap |
| **New course** (or `N`) | Generate a new gate layout |
| **Share** | Copy a link to this exact course |

**Flying it yourself** (Pilot: you): `W`/`↑` speed, `A` `D`/`←` `→` turn, `Space`/`Shift` (or `E`/`Q`) climb and descend. On phones, on-screen sticks appear (left: speed and turn; right: climb and drift); gamepads work too. You race the racing-line autopilot, shown as a cyan ghost, and your own best lap on that course, shown as a gold ghost (kept in your browser).

The course is in the URL, e.g. https://dom-t23.github.io/drone-sim/#seed=12&d=hard (add `&nav=vision` for vision mode, `&wind=gusty` for wind, `&pilot=manual` to race it yourself), so any layout can be shared or revisited.

## How it works

- `src/sim.js` is the simulation core with no rendering dependencies: course generation (three difficulties; hard adds big climbs, angled gates and dive gates), quadrotor dynamics (thrust vector with tilt and thrust limits, first-order lag, drag), the racing-line planner, both autopilots, the assisted manual pilot, gate/lap detection, collisions with gate frames, legs and pillars (crash, tumble and respawn), and vision mode (camera model, PnP, gating and fusion, in-flight re-planning).
- `src/main.js` renders it with three.js and runs the sim at a fixed 120 Hz timestep; `src/telemetry.js` draws the telemetry panel, `src/splits.js` works out gate splits, `src/race.js` the gap to the rival when you race it, and `src/ghost.js` records the best lap and replays it.
- `test/` holds headless checks run with `npm test` (Node 18+). They fly a set of course seeds on every difficulty and nav mode and fail if the drone misses gates, crashes, strays from its line, gets slower, mis-estimates gates, or the state goes non-finite.

The default autopilot flies a **racing line**: a smooth closed spline through every gate, crossing each one square-on. Where it crosses each opening (up to 0.6 m off centre) and how hard it swings in are tuned by coordinate descent on the planned lap time. Each point on it gets the fastest speed whose turn still fits the drone's tilt and thrust limits, then forward and backward passes add acceleration and braking, using only what the turn leaves of that envelope (a friction circle), so it brakes early for corners over a crest. The tracker feeds forward the acceleration the path needs (turning, speeding up, beating drag) and corrects any position and velocity error with PD feedback. The line on the course shows the plan, coloured blue (slow) to orange (fast).

**Vision mode** (the **Nav** button) takes away the drone's perfect knowledge. Its map has every gate moved up to 2.6 m, so flying the map alone (**map only**) misses gates. With **vision**, a synthetic onboard camera finds the corners of the real gates in view, with pixel noise, lost corners and the odd wild one. Each detection is turned into a gate position by PnP (fitting the gate's known shape to the corners by Levenberg-Marquardt). It's checked against what the drone already believes (innovation gating, which throws out the wild ones) and fused into a running estimate. The racing line is re-planned through the corrected gates as it flies. The cyan frames show where the drone believes each gate is, and the onboard view shows the detections: green when used, red when rejected.

**Wind** pushes the drone through drag on its air-relative velocity: a steady wind plus random gusts. A disturbance observer compares the acceleration the drone gets with what its still-air model predicted, and cancels the difference. The planner also keeps back enough thrust for the forecast wind.

**Manual flight** works like a camera drone's velocity mode: the sticks ask for a forward speed, turn rate, climb rate and sideways drift, and the same thrust-vector loop delivers them within the airframe's limits. The turn rate is capped by speed so the drone follows its nose, and with the climb stick centred it eases to the next gate's height.

**Crashes**: gates, their legs and the scenery pillars are solid. A real hit cuts the motors and the drone tumbles to the ground, then respawns on the line just past the last gate it passed; a light touch just slides it along. Map-only mode is a ghost run and flies through frames.

**Ghost replay**: every lap is recorded (position, heading and tilt 20 times a second, plus the time at each gate), and the fastest one so far flies again as a translucent gold drone, in step with your current lap. The **Ghost** row in the stats shows how far ahead or behind it you were at the last gate. Your own best ghost is saved per course, difficulty and wind in this browser, so it is waiting for you next time.

The original autopilot, still available with the **Pilot** button, is pure pursuit along each gate's axis: the drone chases a point sliding along the line through the gate centre, so it lines up and flies straight through, slowing for sharp turns. If it predicts it would cross a gate off the opening, it swings wide and comes round for another try.

## Run locally

```bash
python3 -m http.server 8000   # then open http://localhost:8000
npm test                      # simulation checks, no install needed
```

Built entirely by Claude.
