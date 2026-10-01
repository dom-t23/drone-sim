# Roadmap

A rough backlog, not a contract. Each night picks the item that will make the biggest visible or technical difference, or something better that isn't listed. Tick items off here and record them in `CHANGELOG.md`. Big items can be split across nights; leave a short **Next steps** note under the item so the next run can carry on.

## Flying
- [ ] **Harder courses.** Tighter turns, big height changes, dive gates, a split-S, gates at an angle. Add a difficulty selector and a shareable seed in the URL (`#seed=12&d=hard`).
- [ ] **Faster autopilot.** Plan the racing line through upcoming gates (e.g. minimum-snap or time-optimal-ish splines) instead of axis pursuit, and track it with a proper attitude/thrust controller. Show the planned path.
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
