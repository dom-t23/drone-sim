# Drone Sim

A self-flying racing drone in the browser. A quadrotor flies itself around a procedurally generated gate course, with a chase camera, an onboard FPV view, lap timing and a light trail.

**Live demo:** https://dom-t23.github.io/drone-sim/

This is an experiment in what Claude can build on its own. A scheduled Claude run picks up the project three nights a week (Monday, Thursday and Saturday), adds the next feature from `ROADMAP.md` or an idea of its own, tests it, and pushes. GitHub Pages redeploys the demo automatically. `CHANGELOG.md` shows what each night added.

## Controls

| Control | Action |
|---|---|
| **Camera** (or `C`) | Chase → orbit → onboard |
| **Speed** | Simulation time 1× / 2× / 4× |
| **Pilot** (or `P`) | Racing line ↔ original axis pursuit |
| **Course** (or `D`) | Difficulty: easy → normal → hard |
| **Nav** (or `V`) | Ground truth → vision → map only |
| **New course** (or `N`) | Generate a new gate layout |
| **Share** | Copy a link to this exact course |

The course is in the URL, e.g. https://dom-t23.github.io/drone-sim/#seed=12&d=hard (add `&nav=vision` for vision mode), so any layout can be shared or revisited.

## How it works

- `src/sim.js` is the simulation core with no rendering dependencies: course generation (three difficulties; hard adds big climbs, angled gates and dive gates), quadrotor dynamics (thrust vector with tilt and thrust limits, first-order lag, drag), the racing-line planner, both autopilots, gate/lap detection, and vision mode (camera model, PnP, gating and fusion, in-flight re-planning).
- `src/main.js` renders it with three.js and runs the sim at a fixed 120 Hz timestep.
- `test/` holds headless checks run with `npm test` (Node 18+). They fly a set of course seeds on every difficulty and nav mode and fail if the drone misses gates, strays from its line, gets slower, mis-estimates gates, or the state goes non-finite.

The default autopilot flies a **racing line**: a smooth closed spline through every gate, crossing each one square-on. Where it crosses each opening (up to 0.6 m off centre) and how hard it swings in are tuned by coordinate descent on the planned lap time. Each point on it gets the fastest speed whose turn still fits the drone's tilt and thrust limits, then forward and backward passes add acceleration and braking, using only what the turn leaves of that envelope (a friction circle), so it brakes early for corners over a crest. The tracker feeds forward the acceleration the path needs (turning, speeding up, beating drag) and corrects any position and velocity error with PD feedback. The line on the course shows the plan, coloured blue (slow) to orange (fast).

**Vision mode** (the **Nav** button) takes away the drone's perfect knowledge. Its map has every gate moved up to 2.6 m, so flying the map alone (**map only**) misses gates. With **vision**, a synthetic onboard camera finds the corners of the real gates in view, with pixel noise, lost corners and the odd wild one. Each detection is turned into a gate position by PnP (fitting the gate's known shape to the corners by Levenberg-Marquardt). It's checked against what the drone already believes (innovation gating, which throws out the wild ones) and fused into a running estimate. The racing line is re-planned through the corrected gates as it flies. The cyan frames show where the drone believes each gate is, and the onboard view shows the detections: green when used, red when rejected.

The original autopilot, still available with the **Pilot** button, is pure pursuit along each gate's axis: the drone chases a point sliding along the line through the gate centre, so it lines up and flies straight through, slowing for sharp turns.

## Run locally

```bash
python3 -m http.server 8000   # then open http://localhost:8000
npm test                      # simulation checks, no install needed
```

Built entirely by Claude.
