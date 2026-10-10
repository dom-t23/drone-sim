# Roadmap

A rough backlog, not a contract. Each night picks the item that will make the biggest visible or technical difference, or something better that isn't listed. Tick items off here and record them in `CHANGELOG.md`. Big items can be split across nights; leave a short **Next steps** note under the item so the next run can carry on.

## Flying
- [x] **Harder courses.** Easy / normal / hard selector, shareable `#seed=12&d=hard` URLs, hard courses with big height changes, angled gates and dive gates (2026-10-04).
  - **Next steps:** a split-S (two stacked gates flown in opposite directions) and other set pieces need the course generator to stop being a single loop around the centre; tighter hairpins. The pursuit pilot misses the odd gate on hard; that's fine as a baseline, but it could slow down more for angled gates.
- [x] **Faster autopilot.** Racing line (Hermite spline through gate centres plus an envelope-aware speed profile) tracked with feedforward + PD, shown on the course. 27.3 s → 17.6 s a lap (2026-10-03).
  - Friction-circle coupling of turn and along-track budgets done (2026-10-04): ~17.3 s on normal.
  - Gate crossing points and tangent lengths optimised by coordinate descent on planned lap time (2026-10-04): ~16.3 s on normal.
  - **Next steps:** minimum-snap or time-optimal (e.g. CPC-style) trajectories instead of Hermite segments; let the planner trade crossing offset against tracking margin per gate; speed up the optimiser (incremental re-planning of the two segments a gate touches) so vision can re-shape continuously. Retighten the lap-time test after each gain.
- [x] **Wind and disturbances.** Steady wind + capped Gauss-Markov gusts, disturbance observer, forecast-aware planning margins (2026-10-04).
  - **Next steps:** sensor noise and motor lag variation (best done with vision step 2's state estimator); spatially varying wind (gusts that sweep across the course); estimate the wind direction per lap and bias the line to use tailwinds.
- [x] **Crashes.** Gate frames, legs and scenery pillars are solid; a real hit tumbles the drone to the ground, then it respawns just past the last gate passed. The pursuit pilot learned to go round instead of scraping frames (2026-10-07).
  - **Next steps:** a ground strike at speed should crash too (today the ground just catches you); camera shake and a crash sound; a "crashes" toggle or a no-respawn hardcore mode; let the racing-line planner know the drone's real size and the legs, so shaping can clip tighter where it's safe; make map-only crash for real once it can recover (e.g. vision kicks in after its first crash).

## Seeing
- [x] **Vision mode, step 1.** Map with every gate moved up to 2.6 m; synthetic onboard camera (corner noise, dropouts, wild corners); PnP by Levenberg-Marquardt; misfit check + innovation gating; per-gate information filter; in-flight re-planning; detection overlay, ghost gates and map-only comparison mode (2026-10-04).
  - **Next steps:** real occlusion (gate frames and pillars hiding corners) instead of random dropouts; fuse gate orientation as well as position; data association without known gate IDs (match detections to the nearest predicted gate); a motion-blur/latency model.
- [x] **Vision mode, step 2.** New **Nav: vision + IMU** mode: a noisy, biased accelerometer plus the camera, fused by an EKF whose state holds the drone's position, velocity and accelerometer bias and every gate (EKF-SLAM); zero-velocity updates on the pad find the bias; estimate, 2σ ball and IMU-only dead reckoning drawn in the chase view; position error vs 2σ in telemetry (2026-10-10).
  - **Next steps:** attitude is still ground truth: add a gyro and estimate attitude too (an error-state EKF on the rotation), which makes the camera measurement properly nonlinear; fuse each corner's pixel position directly instead of the PnP gate centre (removes the small far-range PnP bias that forced the covariance inflation); a barometer; add a Vision + IMU entrant to the Field; a "camera blackout" button that cuts the camera for a few seconds so you can watch the estimate drift and snap back.
- [ ] **Vision mode, step 3.** Detect the gate from the actual rendered pixels (colour segmentation and corner finding on a downscaled onboard frame).

## Racing
- [x] **Ghost replay** of the best lap: a gold ghost flies the best lap so far in step with the current lap, with a live gap at each gate; yours are saved per course in the browser (2026-10-07).
  - **Next steps:** a faint gold trail behind the ghost; share a ghost in the URL (or a short code) so a friend can race your lap; let the cyan rival and the gold ghost trade places on a toggle; a "clear my ghost" button.
- [x] **Multiple drones**: a Field button races five autopilots with different controllers and tunings (racing line, vision, smooth, steady, pursuit) from a line-abreast grid, with an F1-style timing tower, name tags, lead-change toasts and tap-to-follow cameras. With Pilot: you, you race all of them (2026-10-08).
  - **Next steps:** drone-to-drone contact (today they fly through each other like ghosts; even a simple sphere check with a bounce would make starts and overtakes look real) and racing-line planners that pick an overtaking line; airframe differences (thrust, drag) as well as tunings; a race length with a chequered flag and a results card; per-drone coloured trails; a field-wide telemetry view (speed traces of everyone on one chart).
- [x] **Manual flight**: keyboard, gamepad and touch sticks in an assisted velocity mode, racing the live racing-line autopilot as a cyan ghost, with countdown, live gap, lap-vs-rival toasts and per-course personal bests (2026-10-05).
  - **Next steps:** a "rate"/acro mode for experienced pilots (sticks command tilt, no assists); ~~a ghost of *your* best lap~~ (done with Ghost replay, 2026-10-07); a short in-game tutorial lap; haptics on gate pass for phones; an assist slider (altitude assist off, lower top speed for beginners).

## Showing off
- [x] **Telemetry panel**: speed (vs plan), tilt and thrust (vs limits), distance off the line, gate splits vs best lap (2026-10-04).
  - **Next steps:** hover read-outs on the charts; a vision panel (estimate error per gate over time); export a lap's telemetry as CSV.
- [x] **Night mode** with LED-lit gates (glow, ground light pools, a real light on the next gate), stars, moon and a drone headlight; drone LEDs; prop wash particles with a ground-effect dust ring (momentum theory); synthesised motor sound with gate, lap and crash cues (2026-10-09).
  - **Next steps:** bloom (needs the three.js addons in the import map and a careful look at phone performance); sound for the other drones in a field race, panned and Doppler-shifted as they pass; a dusk setting between day and night; a camera shake on crash to go with the crunch; LED colour per drone in a field race.
- [ ] **Landing page polish**: intro overlay, "what Claude built last night" from the changelog, version and build date in the corner.
- [ ] **Performance**: holds 60 fps on a mid-range phone.
