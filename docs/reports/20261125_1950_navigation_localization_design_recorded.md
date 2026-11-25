# Navigation and localization design recorded (documents only)

Date: 2026-11-25, 19:50. Task: David's pasted task b31c. No code, config or tests changed. One commit, not pushed.
Written for: David (decider) and whoever builds the localization module later.

## Summary

David's approved design for how the robot finds itself in the room and where objects are is now in the design plan (new section 13.1, plus sections 8, 12 and 13) and in the decisions log (rows M326 to M335). All numbers are marked PROVISIONAL (sim placeholders). Nothing is built.

Surprises:
- The eagle camera is already fixed to the torso in the code and the model. The neck was removed in commit 8cd06f5 (M167). Section 14 of the plan is already the fixed-camera text. The stale neck text sits in other sections (list below).
- The "SLAM: [PENDING] Dan's call" text was in section 12, not 13. Section 13 already said SLAM was decided (M143). Both are updated now.
- The paper excerpts (docs/mpvi_paper_excerpts.md) say nothing about localization, maps, odometry, lidar or GroundingDINO. The new paper claims are marked "not in excerpts, ask David", as the standing rule requires.
- M143 (SLAM once to build the map) and the new decision (no SLAM while running) disagree about how the map is made. Recorded as written, and the map-making question is listed as PENDING.

## Files changed

- docs/design_plan.md
  - Section 8, fidelity table: new rows "Localization (where the robot is)", "Lab map"; the "Object finding" row updated. Short-form list of differences: items (3) and (4) added.
  - Section 12, "Robot position": real replacement rewritten (the AMCL text superseded, M330; SLAM question closed, M327).
  - Section 12, M143 bullet: marked SUPERSEDED IN PART (M327, M330).
  - Section 12, "Lab map and fixtures": real replacement rewritten (M327).
  - Section 12, "Object positions": split added (M326, M332).
  - Section 13, "Object detection": marked UPDATED (M326); M066 text kept as history.
  - Section 13, "Mapping and localization": rewritten (M327, M330); SLAM text replaced.
  - Section 13, "Depth for object positions": hotplate known-surface method decided (M332); other objects PENDING; neck reference replaced by the fixed mount (§14).
  - Section 13.1 (new): overall split, map and start, lidar, odometry error, particle filter, self-orientation, hotplate tracker, scoring, fidelity pointer, considered and dropped, open items (item 11).
- docs/mpvi_decisions.md
  - New rows M326 to M335, placed after M325 and before the SF rows.
  - M066 row: prefix "[UPDATED by M326 ...]" (the old text kept).
  - M143 row: prefix "[UPDATED by M327 and M330 ...]" (the old text kept).
- docs/progress.md: task entry at the top.
- docs/reports/20261125_1950_navigation_localization_design_recorded.md: this file.

Section 14 was not edited (per the task).

## Decision rows

| Decision | Item | Design plan section |
|---|---|---|
| M326 | Overall split: lidar, odometry and prebuilt map for robot position; classic vision for known fixtures; detector only for unknown objects | 13.1, 8, 12 |
| M327 | Room map known in advance; hotplate not in the map; start pose unknown; no SLAM while running | 13.1, 12, 13 |
| M328 | Lidar assumed (360 beams, 10 Hz, 0.15 to 12 m, 2 cm noise) | 13.1 |
| M329 | Odometry error levels (none, realistic, harsh); seeded; filter's believed error = twice the random error | 13.1 |
| M330 | Own particle filter mirroring AMCL's two core models | 13.1, 12 |
| M331 | Self-orientation procedure (guesses, turn check, confidence, retries, lost-while-driving rule) | 13.1 |
| M332 | Hotplate tracker (first find, 5 sightings, edge tracking, weighting, 180-degree heading, stance recheck) | 13.1, 12, 13 |
| M333 | Scoring: logs, measures, pass lines, test runs | 13.1 |
| M334 | Fidelity table rows (localization, lab map, object finding) | 8 |
| M335 | Considered and dropped: camera corners; EKF-SLAM | 13.1 |

Rows M066 and M143 were marked UPDATED, not deleted.

## Eagle camera: what the code and model use

The code and the model use the FIXED camera, with no neck.

- src/robot_specs.py:191-210: EAGLE_BODY_SPECS and EAGLE_MOUNT. The body is fixed to parent openarm_body_link0 (the torso), pos (0.03, 0.0, 0.9055) m, pitch 50 degrees down. Comment: "no neck" (M153, M158, M161).
- src/robot_specs.py:217-232: the eagle_cam entry in CAMERA_SPECS. Parent eagle_cam_body, 55 degrees vertical, 640 x 360.
- src/cameras.py:57-74: add_eagle_body adds a box to the torso with no joint.
- src/simulator.py:44: the load order says the eagle camera body is fixed to the torso, no neck.
- No src/neck*.py or src/neck_framing.py exists. The neck was removed in commit 8cd06f5 (M167, "Fixed camera step 2 part D").
- models/robot/: no neck text. The only "neck" hits in models/scenes are beaker and flask geometry names (ext_neck, flask_neck: models/scenes/chemistry_lab_beaker_hotplate.xml:155, :244; models/scenes/chemistry_lab_tracer.xml:149, :247). These are not the camera.
- Section 14 of the design plan (header at line 503) is already the fixed-camera text, so the task's premise ("section 14 describes a neck with a drive pose") does not match the file.

Stale neck text in the design plan (not edited except where noted):
- Line 10, top-of-plan note: says the neck code is still to be removed "until step 2". The code is removed already (8cd06f5). Model: no neck. Tests and baselines not verified.
- Line 336, section 9 order of work: step 2 "remove the neck (model, code, checks, tests, baselines...)" is still listed as to-do.
- Lines 118 and 122, section 3.4: the stance requirements still list a neck repeatability check and src/neck_framing.py, which no longer exists.
- Line 82, section 3.1: the collision note says the neck was at its drive pose (marked as history, M153, M160).
- Lines 505 and 510, section 14 (not edited, per the task): "Code today that step 2 removes" names the deleted files; "the placeholder is chosen in step 2" is already done (M161).
- Line 441, section 11: "SLAM is decided (§13, M143); the rest (the filter, the camera corrections) waits" is out of date.
- Lines 320 to 325, section 9: neck mentions in build history, not checked line by line.

## Not done / needs David

1. Lidar status. The task says the lidar is decided (PROVISIONAL), while section 11 (line 441) and M163 say it is "likely, to be confirmed", and M163 keeps Stage 4 deferred until it is confirmed. M328 is recorded as an assumption. M335 says the camera approaches were dropped "once the lidar was confirmed available", which the task asked me to write as is. The lidar is not confirmed. Please confirm the wording.
2. How the prebuilt map is made. The task says "No SLAM". M143 (2026-10-22) says "SLAM once to build the lab map", and M163 names slam_toolbox. I recorded "no SLAM while running" and marked the map-making question PENDING (section 13.1 item 11). Needs your decision.
3. Paper claims not in the excerpts: that the paper assumes accurate localization; that it assumes a prebuilt map; that it uses GroundingDINO for object localization. Marked "not in excerpts, ask David" in M326, M327, M334 and the fidelity rows. Needs confirmation or the paper section.
4. M066 and M143 were marked UPDATED (prefix on the row, old text kept). Not deleted. Confirm this marker style, or say if you want them rewritten.
5. Build timing (item 11): section 13 says the module is built after VLA manipulation works (B8); section 9 ("Before B4", your order of 2026-10-21) puts navigation first. Both texts stand. Not decided.
6. Glass beaker (item 11): the known-surface method is recorded for the sim beaker only. The real beaker may need its own treatment. Open.
7. Depth for objects other than the hotplate: still PENDING (section 13).
8. Config: no value is in src/task_config.py. The rows say "none yet (design only)". Values go to config when the module is built.
9. The task asked that section 14 be checked, not edited. Stale lines in section 14 are listed above.
10. The design plan's top note (line 10) is out of date (see above). Not edited.
11. CLAUDE.md says to commit after each step. The task asked for one commit, so I made one commit at the end.
12. Not pushed (the task says David decides on pushing).
13. Tests: none run (documents only). A tmux job named dryrun_suite is running experiments/run_tests.sh in two other worktrees (openarm-tracer-sim-main-suite and openarm-tracer-sim-dryrun-merge). It is not in this checkout and was not touched.
