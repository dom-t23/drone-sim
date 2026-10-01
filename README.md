# Drone Sim

A self-flying racing drone in the browser. A quadrotor flies itself around a procedurally generated gate course, with a chase camera, an onboard FPV view, lap timing and a light trail.

**Live demo:** https://dom-t23.github.io/drone-sim/

This is an experiment in what Claude can build on its own. A scheduled Claude run picks up the project three nights a week (Monday, Thursday and Saturday), adds the next feature from `ROADMAP.md` or an idea of its own, tests it, and pushes. GitHub Pages redeploys the demo automatically. `CHANGELOG.md` shows what each night added.

## Controls

| Control | Action |
|---|---|
| **Camera** (or `C`) | Chase → orbit → onboard |
| **Speed** | Simulation time 1× / 2× / 4× |
| **New course** (or `N`) | Generate a new gate layout |

## How it works

- `src/sim.js` is the simulation core with no rendering dependencies: course generation, quadrotor dynamics (thrust vector with tilt and thrust limits, first-order lag, drag), the autopilot, and gate/lap detection.
- `src/main.js` renders it with three.js and runs the sim at a fixed 120 Hz timestep.
- `test/` holds headless checks run with `npm test` (Node 18+). They fly every course seed and fail if the drone misses gates or the state goes non-finite.

The first autopilot is pure pursuit along each gate's axis: the drone chases a point sliding along the line through the gate centre, so it lines up and flies straight through, slowing for sharp turns.

## Run locally

```bash
python3 -m http.server 8000   # then open http://localhost:8000
npm test                      # simulation checks, no install needed
```

Built entirely by Claude.
