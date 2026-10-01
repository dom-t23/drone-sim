# Changelog

Newest first. One entry per night's run.

## 2026-10-01: First flight
- Procedural closed-loop gate course (seeded), with gates that light up as the drone approaches and passes them.
- Quadrotor dynamics: thrust-vector model with tilt and thrust limits, first-order thrust lag and linear drag.
- Autopilot: pure pursuit along each gate's axis, slowing for sharp turns. Completes every test seed with no missed gates at about 27 s a lap.
- Chase, orbit and onboard cameras, a picture-in-picture onboard view, a light trail, a lap/speed HUD, 1×/2×/4× time and new-course controls.
- Headless tests (`npm test`) that fly eight course seeds.
