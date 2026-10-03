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
| **New course** (or `N`) | Generate a new gate layout |

## How it works

- `src/sim.js` is the simulation core with no rendering dependencies: course generation, quadrotor dynamics (thrust vector with tilt and thrust limits, first-order lag, drag), the racing-line planner, both autopilots, and gate/lap detection.
- `src/main.js` renders it with three.js and runs the sim at a fixed 120 Hz timestep.
- `test/` holds headless checks run with `npm test` (Node 18+). They fly every course seed with both pilots and fail if the drone misses gates, strays from its line, gets slower, or the state goes non-finite.

The default autopilot flies a **racing line**: a smooth closed spline through every gate centre, crossing each gate square-on. Each point on it gets the fastest speed whose turn still fits the drone's tilt and thrust limits, then forward and backward passes add acceleration and braking limits. The tracker feeds forward the acceleration the path needs (turning, speeding up, beating drag) and corrects any position and velocity error with PD feedback. The line on the course shows the plan, coloured blue (slow) to orange (fast).

The original autopilot, still available with the **Pilot** button, is pure pursuit along each gate's axis: the drone chases a point sliding along the line through the gate centre, so it lines up and flies straight through, slowing for sharp turns.

## Run locally

```bash
python3 -m http.server 8000   # then open http://localhost:8000
npm test                      # simulation checks, no install needed
```

Built entirely by Claude.
