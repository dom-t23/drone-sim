# Changelog

Newest first. One entry per night's run.

## 2026-10-03: Racing line
- **The drone is about 35% faster**: laps drop from about 27.3 s to about 17.6 s on every test seed, still with no missed gates. Top speed goes from 45 km/h to about 80 km/h.
- The course now shows the **planned racing line**, coloured by planned speed: blue where the drone brakes for a turn, orange where it's flat out.
- New **Pilot** button (or `P`) switches between the new racing-line autopilot and the original axis pursuit, so you can watch the difference on the same course. The HUD's new **Plan** row shows the lap time the planner predicts.
- How it works: a closed cubic Hermite spline through every gate centre, crossing each gate square-on; a speed profile that keeps each turn inside the airframe's tilt and thrust envelope (diving turns get the least, since less vertical thrust means less sideways force at max tilt), with forward/backward passes for acceleration and braking limits; and a tracker that adds the path's centripetal, along-track and drag feedforward to PD feedback, looking ahead to cover the thrust lag.
- Tests now fly both pilots, lock in the lap-time gain (racing line under 18.6 s and at least 25% faster than pursuit), check the drone threads every gate within 0.8 m of centre and stays within 1.2 m of its line, and check the plan never asks for more tilt or thrust than the drone has.

## 2026-10-01: First flight
- Procedural closed-loop gate course (seeded), with gates that light up as the drone approaches and passes them.
- Quadrotor dynamics: thrust-vector model with tilt and thrust limits, first-order thrust lag and linear drag.
- Autopilot: pure pursuit along each gate's axis, slowing for sharp turns. Completes every test seed with no missed gates at about 27 s a lap.
- Chase, orbit and onboard cameras, a picture-in-picture onboard view, a light trail, a lap/speed HUD, 1×/2×/4× time and new-course controls.
- Headless tests (`npm test`) that fly eight course seeds.
