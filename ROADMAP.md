# Roadmap

A rough backlog, not a contract. Each night picks the item that will make the biggest visible or technical difference, or something better that isn't listed. Tick items off here and record them in `CHANGELOG.md`. Big items can be split across nights; leave a short **Next steps** note under the item so the next run can carry on.

## Flying
- [ ] **Harder courses.** Tighter turns, big height changes, dive gates, a split-S, gates at an angle. Add a difficulty selector and a shareable seed in the URL (`#seed=12&d=hard`).
- [x] **Faster autopilot.** Racing line (Hermite spline through gate centres plus an envelope-aware speed profile) tracked with feedforward + PD, shown on the course. 27.3 s → 17.6 s a lap (2026-10-03).
  - **Next steps:** optimise the gate crossing points within each opening and the tangent lengths (e.g. a few rounds of coordinate descent on planned lap time) instead of always aiming at the centre; couple the turn and acceleration budgets (a friction-circle style limit) so the profile can be pushed harder; then try minimum-snap. Retighten the lap-time test after each gain.
- [ ] **Wind and disturbances.** Gusts, sensor noise, motor lag variation; the autopilot must stay robust.
- [ ] **Crashes.** Gate-frame and pillar collisions, with a tumble and respawn.

## Seeing
- [ ] **Vision mode, step 1.** A synthetic camera in `sim.js` that projects the next gate's corners into the onboard image with noise and occlusion, then estimates the gate pose (PnP) and flies from that estimate instead of ground truth. Overlay the detections on the onboard view.
- [ ] **Vision mode, step 2.** Estimator: fuse vision with IMU-like acceleration (an EKF), with innovation gating, and plot the estimate against the truth.
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
