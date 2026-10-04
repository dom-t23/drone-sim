# Roadmap

A rough backlog, not a contract. Each night picks the item that will make the biggest visible or technical difference, or something better that isn't listed. Tick items off here and record them in `CHANGELOG.md`. Big items can be split across nights; leave a short **Next steps** note under the item so the next run can carry on.

## Flying
- [x] **Harder courses.** Easy / normal / hard selector, shareable `#seed=12&d=hard` URLs, hard courses with big height changes, angled gates and dive gates (2026-10-04).
  - **Next steps:** a split-S (two stacked gates flown in opposite directions) and other set pieces need the course generator to stop being a single loop around the centre; tighter hairpins. The pursuit pilot misses the odd gate on hard; that's fine as a baseline, but it could slow down more for angled gates.
- [x] **Faster autopilot.** Racing line (Hermite spline through gate centres plus an envelope-aware speed profile) tracked with feedforward + PD, shown on the course. 27.3 s → 17.6 s a lap (2026-10-03).
  - Friction-circle coupling of turn and along-track budgets done (2026-10-04): ~17.3 s on normal.
  - **Next steps:** optimise the gate crossing points within each opening and the tangent lengths (e.g. a few rounds of coordinate descent on planned lap time) instead of always aiming at the centre; then try minimum-snap. Retighten the lap-time test after each gain.
- [ ] **Wind and disturbances.** Gusts, sensor noise, motor lag variation; the autopilot must stay robust.
- [ ] **Crashes.** Gate-frame and pillar collisions, with a tumble and respawn.

## Seeing
- [x] **Vision mode, step 1.** Map with every gate moved up to 2.6 m; synthetic onboard camera (corner noise, dropouts, wild corners); PnP by Levenberg-Marquardt; misfit check + innovation gating; per-gate information filter; in-flight re-planning; detection overlay, ghost gates and map-only comparison mode (2026-10-04).
  - **Next steps:** real occlusion (gate frames and pillars hiding corners) instead of random dropouts; fuse gate orientation as well as position; data association without known gate IDs (match detections to the nearest predicted gate); a motion-blur/latency model.
- [ ] **Vision mode, step 2.** Estimator: the drone's own state is still ground truth. Add an EKF fusing IMU-like acceleration with the gate detections (each detected gate is a known-ish landmark), reuse the innovation gating, and plot the estimate against the truth. Start the map error and the state error together so they have to be solved jointly.
- [ ] **Vision mode, step 3.** Detect the gate from the actual rendered pixels (colour segmentation and corner finding on a downscaled onboard frame).

## Racing
- [ ] **Ghost replay** of the best lap per course.
- [ ] **Multiple drones** racing with different controllers or tunings, and a live leaderboard.
- [ ] **Manual flight**: keyboard, gamepad and touch controls to race against the autopilot ghost.

## Showing off
- [ ] **Telemetry panel**: speed, thrust and tilt graphs, gate split times.
- [ ] **Night mode** with LED-lit gates, prop wash particles and motor sound.
- [ ] **Landing page polish**: intro overlay, "what Claude built last night" from the changelog, version and build date in the corner.
- [ ] **Performance**: holds 60 fps on a mid-range phone.
