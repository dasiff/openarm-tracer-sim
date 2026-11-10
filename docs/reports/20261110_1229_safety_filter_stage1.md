# Safety filter Stage 1 (branch safety-filter): shared layer, shapes, lab map, no-simulation checks

Written for: David (design review of Stage 1). Scope per the task brief: docs, rename, build tools, shared layer, no-simulation checks. No resolver (Stage 2), no offline bench (Stage 3), no control-loop plug-in (Stage 4). No simulation stepping. No push.

## 1. Location check

- `pwd`: /home/vboxuser/projects/openarm-tracer-sim-safety-filter (a git worktree)
- branch: safety-filter (main untouched)
- Python: /home/vboxuser/projects/openarm-tracer-sim/.venv/bin/python (the main folder's venv, not a new one), MuJoCo 3.13.0
- `import geo_kin`: ok (geo_kin 0.1.0 via the venv)
- Setup note: the worktree's `deps/` submodules were empty. They were initialised with `git submodule update --init` using the main checkout's local copies as the source (the `-c submodule.<name>.url=` override; `.gitmodules` unchanged), at the same commits as the gitlinks. `git status` is clean for them.

## 2. Commits (all on safety-filter, none pushed)

| hash | message | files |
|---|---|---|
| 928bc08 | safety filter design: SF-1..SF-7 decision rows, section 16 references and pending list, SEW-Mimic paper notes | docs/mpvi_decisions.md, docs/design_plan.md, docs/sew_mimic_paper_notes.md (new), docs/progress.md |
| 41778b5 | safety filter: rename Kong's wrapper src/safety_filter.py to src/geo_kin_self_collision_filter.py (behaviour unchanged) | src/geo_kin_self_collision_filter.py (renamed), experiments/safety_filter_roundtrip.py, experiments/branch_jump_demo.py, experiments/elbow_snap_readable.py, experiments/elbow_snap_sweep.py, README.md |
| 927722b | safety filter: capsule build tool (robot_shapes.json, fingerprinted) and lab map survey (lab_map.json) | tools/build_filter_shapes.py, tools/survey_lab_map.py, data/safety_filter/robot_shapes.json (force-added), data/safety_filter/lab_map.json (force-added), src/safety_filter/{__init__,config,robot_model}.py, docs/progress.md |
| ef02f65 | safety filter: shared layer and no-simulation checks (Stage 1) | src/safety_filter/{geometry,shapes,touch,world,config}.py, experiments/safety_filter_{distance,shapes,touch,structure}_test.py, experiments/run_tests.sh (4 entries added), tools/build_filter_shapes.py, data/safety_filter/robot_shapes.json (regenerated at full precision) |

This report is committed as the next commit (see section 10 and `git log`).

## 3. New decision rows (temporary labels, in docs/mpvi_decisions.md)

- **SF-1** (supersedes M286): all filter shapes come from our own MuJoCo model; only directions go to the geo_kin solver: upper arm = our joint 3 axis, forearm = our joint 5 axis, hand = our link 7 orientation times C_HAND. Evidence: docs/reports/safety_filter_frame_fix_20261109.md.
- **SF-2** (adds to M295, PROVISIONAL): after conversion, forward kinematics of the converted joints must reproduce the elbow and hand within 2 mm and 0.5 deg; otherwise shorten the step.
- **SF-3** (refines M287): hand = one capsule for link 7 plus two palm capsules (each fitted in the link 7 frame); one capsule per finger in its own body frame; upper arm fat (about 9.7 cm), accepted.
- **SF-4** (refines M289): a held object is never checked against its own hand, only against the other arm and the robot body.
- **SF-5**: shape sizes come from the loaded robot model via tools/build_filter_shapes.py, stamped with the robot fingerprint; the filter refuses to start on a mismatch.
- **SF-6**: rename of Kong's wrapper to src/geo_kin_self_collision_filter.py; our package is src/safety_filter/; SF labels until merge.
- **SF-7** (supersedes the stage order of M297): Stage 1 shared layer and no-simulation checks; Stage 2 resolver and Kong comparison; Stage 3 offline bench; Stage 4 plug-in with off/shadow/on. Stages 1 to 3 need no physics. Open: whether demo generation waits for Stage 4.

M286 is marked superseded, M297 marked "stage order superseded", M287 and M289 marked refined. Design plan section 16: references added; PENDING: palm item closed (SF-3), gravity item closed (held weight always from the robot's own weighing, never the model's beaker mass; commit bc339ab), "demo generation vs Stage 4 order" added as pending.

## 4. Rename (commit 41778b5)

Files whose import or reference changed (`from src.safety_filter import` became `from src.geo_kin_self_collision_filter import`): experiments/safety_filter_roundtrip.py, experiments/branch_jump_demo.py, experiments/elbow_snap_sweep.py, experiments/elbow_snap_readable.py. Also src/geo_kin_self_collision_filter.py (its own docstring example), README.md (tree line). models/policies/branch_jump_policy.py imports `experiments.safety_filter_roundtrip` (unchanged name), so it did not change. Historical reports in docs/reports/ were not edited.

Tests run: `python experiments/safety_filter_roundtrip.py` exits 0 (timing lines printed). The other changed modules compile (`py_compile`). No `_test.py` file imports the wrapper.

## 5. Shape table (data/safety_filter/robot_shapes.json)

Ends and boxes are stored body-local (body and local coordinates); the filter places them with its own forward kinematics. Upper arm and forearm ends are the joint anchors (the joint positions are at the body origins in this model, so the local ends are 0 in those bodies).

| arm | part | frame (body) | end points (body-local, m) | radius (m) or box half sizes (m) | geom that sets it |
|---|---|---|---|---|---|
| left | upper arm | joint 2 anchor (body link2) to joint 4 anchor (body link4) | (0,0,0) in link2 to (0,0,0) in link4 | r 0.096686 | links 1-3 collision meshes |
| left | forearm | joint 4 anchor (link4) to joint 7 anchor (link7) | (0,0,0) in link4 to (0,0,0) in link7 | r 0.076135 | links 4-6 collision meshes |
| left | shoulder mount | link 0 (fixed to torso) | (-0.0004,0.0668,0.0200) to (-0.0006,-0.0258,0.0368) | r 0.0581 | link0 collision mesh |
| left | link 7 | link 7 | (0.0000,0.0120,0.1324) to (-0.0000,-0.0236,-0.0601) | r 0.0329 | link7 collision mesh |
| left | palm 1 | link 7 | (0.0004,0.0810,0.1131) to (-0.0005,0.0261,0.0957) | r 0.0094 | hand collision mesh |
| left | palm 2 | link 7 | (-0.0012,-0.0581,0.1004) to (0.0005,0.0246,0.0941) | r 0.0285 | hand collision mesh |
| left | finger (right) | finger body openarm_left_right_finger | (-0.0128,0.0037,0.0644) to (0.0074,-0.0280,-0.0051) | r 0.0318 | finger collision mesh |
| left | finger (left) | finger body openarm_left_left_finger | (-0.0128,-0.0037,0.0644) to (0.0074,0.0280,-0.0051) | r 0.0318 | finger collision mesh |
| right | upper arm | joint 2 anchor (link2) to joint 4 anchor (link4) | as left | r 0.096686 | links 1-3 |
| right | forearm | joint 4 (link4) to joint 7 (link7) | as left | r 0.076135 | links 4-6 |
| right | shoulder mount | link 0 | (-0.0004,-0.0668,0.0200) to (-0.0006,0.0258,0.0368) | r 0.0581 | link0 collision |
| right | link 7 | link 7 | (0,-0.0120,0.1324) to (-0,0.0236,-0.0601) | r 0.0329 | link7 collision |
| right | palm 1 | link 7 | (0.0005,-0.1029,0.1251) to (-0.0003,-0.0220,0.0946) | r 0.0124 | hand collision |
| right | palm 2 | link 7 | (-0.0015,0.0574,0.1027) to (0.0006,-0.0224,0.0933) | r 0.0285 | hand collision |
| right | finger (right) | finger body openarm_right_right_finger | (-0.0128,0.0037,0.0644) to (0.0074,-0.0280,-0.0051) | r 0.0318 | finger collision |
| right | finger (left) | finger body openarm_right_left_finger | (-0.0128,-0.0037,0.0644) to (0.0074,0.0280,-0.0051) | r 0.0318 | finger collision |
| common | base (tracer chassis mesh) | base_link | box centre (-0.0108,0.0019,-0.0537), half (0.0719,0.3051,0.3533) | box | base_link_geom (tightest box in its own frame) |
| common | base wheel/link boxes (6) | right_link, left_Link, right/left front and rear links | boxes, half sizes 0.019/0.0625/0.0625 and 0.0409/0.039/0.0587 | box | the six chassis link meshes (each its own frame) |
| common | pedestal base | pedestal_base | centre (0,0,0.1524), half (0.06,0.06,0.1524) | box | pedestal_base_collision (exact) |
| common | pedestal top | pedestal_top | centre (0,0,-0.01), half (0.07,0.07,0.01) | box | pedestal_top_collision (exact) |
| common | torso | openarm_body_link0 | vertical capsule ends (0,0,0.008) to (0,0,0.758) | r 0.0416 | torso mesh body_link0_3 |
| common | eagle camera body | eagle_cam_body | centre 0, half (0.015,0.045,0.0125) | box | eagle_cam_body box (exact) |
| common | battery | battery | centre 0, half (0.192,0.104,0.122) | box | battery_geom (exact) |

Arm segment sampling (upper arm and forearm; the links move relative to the segment): joints 1 to 3 (upper arm) and 4 to 6 (forearm), joint-limit corners (8 each) plus random poses within the limits: 1000 random in the first set, 1000 more in the second. Other joints at zero.

| arm | part | radius at 1000 samples | radius at 2000 samples | still grows? |
|---|---|---|---|---|
| left and right | upper arm | 0.096686 m | 0.096686 m | no |
| left and right | forearm | 0.076135 m | 0.076135 m | no |

Fingerprint: 781198f99c7e46e8... (SHA-256 over the compiled geometry; full value in the file). The robot model also includes the battery and eagle camera body, which the simulator adds on top of the robot description (the build tool and the filter use the same loader).

## 6. Lab map (data/safety_filter/lab_map.json)

Scene: models/scenes/chemistry_lab_beaker_hotplate.xml (the task scene); fingerprint = SHA-256 of that file (ada206db631ff2e3...). Boxes at their world poses; cylinders and capsules are written as the box enclosing them in their own frame. Only collidable geoms (contype > 0, the simulator's own collision set) are taken.

| fixture | boxes written (name: half sizes, m) |
|---|---|
| bench (9) | bench_top 2.2, 0.8, 0.06; bench_frame_front and back 2.18, 0.02, 0.45; bench_frame_left and right 0.02, 0.78, 0.45; bench_leg_fl, fr, bl, br 0.06, 0.06, 0.45 |
| sink (3) | sink_basin_outer 0.35, 0.25, 0.03; faucet_post (cylinder) 0.025, 0.025, 0.1; faucet_spout (capsule) 0.015, 0.015, 0.115 |
| walls (4) | wall_n, wall_s 6.0, 0.08, 1.5; wall_e, wall_w 0.08, 6.0, 1.5 |
| back_counter (2) | back_counter_top 2.6, 0.55, 0.06; back_counter_base 2.6, 0.55, 0.45 |
| fume_hood (2) | hood_base 1.2, 0.9, 0.45; hood_worktop 1.2, 0.9, 0.05 |
| storage_cabinet (1) | cab_body 0.7, 0.6, 1.0 |
| safety_station (2) | safety_post (cylinder) 0.06, 0.06, 1.1; eyewash_bowl (cylinder) 0.2, 0.2, 0.05 |

Excluded (24 geoms, listed in the file with reasons): visual or non-colliding boxes (shelves, pegboard, the hood sides, back, sash and top, the cabinet doors and top, the sink hole, the light fixtures); the gas manifold and gas knobs on the bench (collidable, not in the fixture list: David to decide whether the bench's gas manifold is a fixed fixture). The hotplate, beakers, test-tube rack, flasks, stools, the rolling cart, the microscope, the scale, the fire extinguisher and the waste bin are not fixed fixtures and are not written.

## 7. Test table

| test | result | key number |
|---|---|---|
| experiments/safety_filter_distance_test.py | PASS | capsule-capsule, 2000 pairs (1231 outside, 769 deep): max error vs mj_geomDistance 1.2e-16 m; 0 sign disagreements; 0 push failures. Capsule-box, 2000 pairs (1486 outside, 514 deep): max error vs a 40001-sample brute force 3.5e-5 m (the filter's grid); on outside pairs vs mj_geomDistance 3.9e-8 m; 0 deep pairs where the push makes the gap worse. |
| experiments/safety_filter_shapes_test.py | PASS | 200 poses (both arms drawn in each pose; each arm 200 random configurations; random pedestal, base pose and finger travel). Largest stick-out 2.9e-15 m (floating-point noise; the check is <= 0 up to 1e-12 m). Placed upper arm and forearm ends equal MuJoCo's joint anchors (error 0). A changed model is refused (fingerprint). |
| experiments/safety_filter_touch_test.py | PASS (40 checks) | 12 pair-kind cases (one per kind: arm-world 1 cm; fingers vs world, held vs world, same arm, held vs own hand, torso, body-body, shoulder mounts, hand-hand 1 cm, held vs other hand 1 cm, robot-robot 3 cm, arm vs pedestal 3 cm); held attach (matched beaker removed from world), held follows the hand (offset error 1.9e-16 m), release (returns at the pose the hand left it), fallback (capsule of 8 cm, warning logged, nothing returned on release); broad phase (near candidate, far not, pushed pair kept until released); catalog sizes equal models/objects/beaker.xml and hotplate.xml; lab map fingerprint equals the scene file; adapter; config margins. |
| experiments/safety_filter_structure_test.py | PASS | no project import outside robot_specs, battery, cameras; no scene or model loaded from text or a scene path; only the robot description file is read at load; no public parameter named for a simulator, a scene or a data object; world.py does not import src.world_model. |
| experiments/safety_filter_roundtrip.py (rename check) | exit 0 | timing: solve median 0.067 ms with filter on |

Timing per tick (no stepping; 200 random states, median; script not committed): place 0.9 ms; broad phase 1.2 ms; pair checks 13.4 ms (median 7 checked pairs, max 22). Total median about 16 ms. This is above the 2 ms line of M297, which is a Stage 4 pass line, not a Stage 1 criterion. The cost is in the Python capsule-box routine (vectorised grid of 4001 samples per pair); not optimised here.

## 8. Things that differ from the brief, and why

1. **Upper arm and forearm sampling**: the brief asks for at least 2000 poses "within joint limits including the corners of the joints involved". I sampled the joints that change the geometry relative to each segment (1 to 3 for the upper arm, 4 to 6 for the forearm), plus their 8 corners each and 2000 random poses in total. The other joints are at zero; they do not change these radii (their rotations are about the segment ends). The maximum did not grow between 1000 and 2000 samples, so no inflation was added.
2. **Palm**: two capsules are fitted by a split along the principal axis, each enclosing its own points; the union covers the hand. The test checks the union, not each capsule (a single capsule of the pair does not cover the hand by design).
3. **Pedestal lift is a robot input**: the pedestal joint moves the torso, the shoulder mounts and the arms, so the filter input includes the pedestal height (RobotState.pedestal_m). It is a robot joint, not world data.
4. **Gripper input**: finger travel in metres per finger (both fingers of one hand take the same value).
5. **Cylinders** (the beaker, world cylinders) are checked as their enclosing box: conservative, Stage 1 only; the held shape uses the same box.
6. **MuJoCo's mj_geomDistance is not a reliable reference for a capsule that passes through a box**: one of 2000 pairs gave a positive distance for a segment with 11295 of its samples inside the box. The capsule-box reference is therefore a dense brute-force sample, with MuJoCo compared on outside pairs.
7. **MuJoCo's fromto points** lie on the capsule surfaces, not the centre lines; the test compares directions.
8. **Touch-table choices not in any decision row** (flagged, for David): (a) the torso is exempt from arm and held-object pairs (EXEMPT_TORSO_PAIRS); (b) both shoulder mounts are exempt from each other (fixed to the torso); (c) robot body against the world is not checked (M288 gives margins only for arm parts vs world); (d) a held object is exempt from the world and from its own arm (SF-4 and M289).
9. **Held fallback**: the default height of 8 cm and the radius of half the finger gap are PROVISIONAL placeholders; M289 does not give the height.
10. **Adapter**: the temporary WorldModel adapter takes a world model object by duck typing. M290 allows world information from the world model; the structure test forbids simulator, scene and data objects, not the world model.
11. **Robot body includes the wheel links**: the six chassis link meshes are boxes in the "base" part, in addition to base_link (the brief lists "base").
12. **Object ids must be unique in the world** (WorldView removes by id); the held object returns under the same id.
13. The filter's `src/safety_filter/robot_model.py` imports src/battery.py and src/cameras.py (robot-only helpers that the simulator also uses to add the battery and the eagle body). It does not import the simulator.
14. Paper notes: docs/sew_mimic_paper_notes.md was written from an automated summary of the arXiv HTML, not from the PDF. Algorithm and section numbers are as the summary gave them. Section VI-C numbers were in the first pass only and are unconfirmed; the note says so. David should check the note against the PDF.
15. Build sampling settings and the filter settings live in src/safety_filter/config.py (not src/task_config.py), as the brief asks.

## 9. Not done

- **Full test suite** (experiments/run_tests.sh): not run (it takes about an hour and needs the robot simulation; the four new tests are registered in run_tests.sh but were run individually).
- **Pass line for time per tick** (2 ms median, M297): not met (13 to 16 ms median); a Stage 4 issue, not optimised here.
- **Stage 2 (resolver, gap rule, conversion checks SF-2)**: not started, as the brief says. Config for them is in config.py (tagged PROVISIONAL).
- **Stage 3 (offline bench)** and **Stage 4 (control-loop plug-in)**: not started.
- **Robot body against the world** and the gas manifold: not checked / not written (section 8, items 8c and lab map).
- **Push to origin**: not done (the brief says do not push).
- **data/ outside the shapes and lab map**: not copied; not needed for Stage 1.

## 10. Commit of this report

Written before the reply, committed with docs/progress.md updated (crash rules 2 and 6).
