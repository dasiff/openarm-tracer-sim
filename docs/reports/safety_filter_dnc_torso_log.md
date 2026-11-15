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

## Step 1 and 2: identification and label (in progress)
- Code: tools/never_close_dc.py `--label-torso` (class TorsoBound, label_torso); settings in config NEVER_CLOSE_DC (torso_proximity_margin_m 0.005, torso_guard_shapes "all_arm").
- RUN: tmux `dnc1`: `bash experiments/run_capped.sh data/logs/dnc_label_<stamp>.log timeout 2700 python tools/never_close_dc.py --label-torso --budget-min 40` (repeat while exit 3; checkpoint data/safety_filter/never_close_dc.json.torso.work.json).
