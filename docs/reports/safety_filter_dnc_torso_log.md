# Progress log: "blocked by the torso" extension of the divide-and-conquer never-close list

Branch safety-filter-dnc (from safety-filter-speed 9e6718e), worktree ~/projects/openarm-tracer-sim-safety-filter-dnc. Not pushed. Main worktree and branch safety-filter untouched. Temporary decision labels DNC-.

## Set-up (2026-11-16)
- The worktree and branch did not exist; made with `git worktree add -b safety-filter-dnc ... safety-filter-speed`. `.venv` links to the main checkout's; `deps/` initialised from the speed worktree's local copies (no .gitmodules or config change); `data/` entries are links to the speed worktree's, EXCEPT `data/safety_filter/never_close_dc.json` (own copy, git-ignored) and the tracked `robot_shapes.json`, `never_close_sweep.json`, `lab_map.json`, so this task's rewrites do not touch the other worktree.
- Joints 1 and 2 of each arm in the compiled model: openarm_left_joint1 = joint 4, openarm_left_joint2 = joint 5, openarm_right_joint1 = joint 13, openarm_right_joint2 = joint 14 (hinges). The mapping is unambiguous.

## Finding before step 1 (changes what the extension does)
Arm shape to torso capsule gap over each shape's full joint range (20000 random poses, left arm; the right arm mirrors):
- left_link1_3: -1.2 to +4.6 mm at EVERY joint 1 angle. It is always within 5 mm of the torso (the capsule over-covers the torso, known from SF-25; the pair is on the Stage 1 list by its real geometry, 64 mm).
- left_link1_1 38.5 to 45.3 mm, left_link1_2 11.7 to 15.2 mm, left_link2 8.5 to 66.4 mm: never within 5 mm.
- shoulder mount: -48.7 mm always (exempt from the torso by construction, TORSO_EXEMPT_PARTS).
- links 3 to 7, palm, fingers: can reach the torso (within 5 mm at 0 to 2 percent of random poses).
Consequences:
- With "any arm shape" taken literally (all arm shapes except the shoulder mount), every pose and every joint box is torso-proximate: all 219 pairs are torso-dependent and the run-time guard fires on every tick, so the Stage 2 list is never used.
- The shapes that depend only on joints 1 and 2 (links 1 and 2) are either always within 5 mm (link1_3) or never (the others), so a sweep over joints 1 and 2 alone gives all or nothing.
- In the golden recordings the closest arm shape to the torso is left_link1_3 on every sampled tick (2.9 to 4.6 mm; 0.4 mm on the cat stream).
Built as written (DNC-2: guard set "all_arm"), with the shape set a config value; the labels two other shape sets would give are stored with each pair for David's choice.

## Step 1 and 2: identification and label DONE (commits c4d13aa, d40095b)
- Code: tools/never_close_dc.py `--label-torso` (class TorsoBound, label_torso). For each declared pair the certifying boxes are produced again by the proof (same counts as stored). A box is torso-proximate when the lower bound (centre value minus travel, the proof's bound) of the smallest gap between a guard-set arm shape and the torso is at or below 5 mm; an arm variable the box does not hold ranges over its full limits. A pair is torso-dependent when every certifying box is torso-proximate.
- First run counted only the arms a pair involves: 217 of 219. That did not match the run-time guard and check A, which count every arm shape of both arms, so it was changed to both arms (my interpretation error, corrected before anything was merged).
- RESULT (log data/logs/dnc_label_20261115_235243.log, 19:44 wall, peak 2.5 GB): torso-dependent 219 of 219, not torso-dependent 0. For information, stored per pair: with the "checked" shape set 219; with the "checked" set restricted to shapes the box fully fixes 196.
- Written to data/safety_filter/never_close_dc.json: `torso_dependent` and `torso_label` on each declared entry, and a top-level `torso` block. The file is git-ignored (not committed); the merged result in robot_shapes.json is committed.

## Step 3: merge tool DONE (commit 35b47f3)
- tools/build_never_close.py writes `never_close_pairs` into robot_shapes.json. Output: "merged never-close list written to data/safety_filter/robot_shapes.json (never_close_pairs): Stage 1 147, Stage 2 219 (of which torso-dependent 219), total 366".
- Loader (src/safety_filter/shapes.py): reads `never_close_pairs` when present (else the old `never_close` block), refuses a block whose model fingerprint, stamp or torso settings do not match.

## Step 4: run-time guard DONE (commit 465ae9a)
- Resolver.begin calls `_torso_guard` before the sleep timers and the pair table: smallest arm-to-torso gap at the raw and the last command (RobotShapes.torso_gap, the filter's capsule distance); at or below 5 mm the tick is torso-proximate and the touch rules see the list without the torso-dependent pairs (RobotShapes.set_torso_proximate). Counter Resolver.torso_proximate_ticks; flags Resolver.torso_proximate, torso_gap_m; a debug log line (logging, logger src.safety_filter.resolver) when the guard switches on or off. No change of step()'s arguments, return values or FilterStatus.
- Filter tests: 12 of 13 as before after one test fix: safety_filter_touch_test compared "pairs not on the list" with the Stage 1 block only (it failed, and safety_filter_sequential_push_test with it, because that test runs the touch test); it now reads the list the filter uses and has two new checks for the merged list and the guard. safety_filter_spec_test is the known old failure.

## Step 5: checks DONE (scripts/safety_filter_dc_torso_check.py, commit 1eea057; log data/logs/dnc_torso_check_20261116_001607.log, 20 min)
- Check A: PASS — 219 torso-dependent pairs, 109500 samples, largest arm-to-torso gap 4.56 mm (limit 15.00 mm); 0 other pairs, 0 samples.
- Check B: PASS — 25295 ticks on 6 recordings, 25295 torso-proximate, 0 disagreements. Per recording, torso-proximate / ticks: punch original 1576 / 1576, punch reactive 1576 / 1576, shadow 7783 / 7783, cat 6436 / 6436, punch original layered 1576 / 1576, cat layered 6348 / 6348.
- What this means: the guard fires on every tick, so the 219 Stage 2 pairs are never trusted and the filter behaves exactly as with the Stage 1 list alone. Cause: left_link1_3's capsule is within 5 mm of the torso capsule at every joint 1 angle. Check A's second half had no pair to test.

## Decisions (temporary labels)
- DNC-1: Torso-dependent = every certifying box is torso-proximate by the proof's bound; both arms count; an arm variable outside the box ranges over its full limits. Setting key `torso_proximity_margin_m` 0.005 in config NEVER_CLOSE_DC (David's TORSO_PROXIMITY_MARGIN, written in the block's lower-case style with its unit).
- DNC-2: Guard shape set `torso_guard_shapes` = "all_arm": every arm shape except TORSO_EXEMPT_PARTS (the shoulder mounts, which overlap the torso by construction). Alternative "checked" (also without the shapes whose torso pair is on the Stage 1 list) is selectable but not used.
- DNC-3: The guard is evaluated at the raw and at the last command of a tick, not at intermediate poses; it is skipped when the list has no torso-dependent pair.
- DNC-4: Merge at build time into `never_close_pairs` (David's decision); the old `never_close` block (Stage 1 with its sweep numbers) is kept beside it; a pair on both lists is kept as Stage 1.
- DNC-5: The top-level `fingerprint` of robot_shapes.json stays the robot model's (the loader compares it with the compiled model). The stamp that covers the merged content is `never_close_pairs.fingerprint` = SHA-256 over the model fingerprint, the merged entries and the torso settings; the SHA-256 of both source files is stored beside it.
- DNC-6: Check A's loose bound is 5 mm + 1 cm = 15 mm (the task also says 6.5 cm; the tighter one is used).
- DNC-7: No logging pattern existed in the filter package; the guard uses the standard `logging` module at debug level, on switching only.
- DNC-8: never_close_dc.json stays git-ignored; robot_shapes.json (tracked) carries the merged list.

## For David
The extension as specified cancels Part B: every tick is torso-proximate. Options, none built: (a) guard set "checked" (drops links 1 and 2, whose capsules over-cover the torso): the guard would then fire only when links 3 to 7, palm or fingers come within 5 mm, but 219 of 219 pairs are still labelled torso-dependent, because one bound over a large box, or over an arm the pair does not involve, is too loose to prove any box torso-free; (b) count only shapes the box fully fixes: 196 of 219; (c) decide a box's torso proximity by subdividing it instead of one bound per box.
Not done: design plan and decision-row updates (temporary DNC labels only, as asked); the full test suite; timing with the merged list.
