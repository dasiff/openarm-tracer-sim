# Trim handoff, z only: Run 1 (blend on) and Run 2 (control, blend off), with the cleanup (M230 to M234)

Written for: David (design review). Branch held-weight-ramp-sag-off. Build c52d09c (z-only handoff, begin_load_change removed, trim_blend_enabled switch). Launcher and metrics: experiments/bounce_handoff_run.py, experiments/bounce_handoff_metrics.py. Logs: data/logs/zonly_run1_20261102_194144.log and data/logs/zonly_run2_20261102_194144.log. Outputs: data/bounce_zonly_20261102/run1/, run2/, metrics_run1.md, metrics_run2.md. Acronyms: IN, CLOSE, UP, CHECK, OVER, DOWN, OPEN, BACK, HOME = the pick's and the place's steps; z = height; ref = the trim's z saved at the grasp; full = the trim's z latched on the first tick of the ramp-in; kg = kilograms; deg = degrees; mm = millimetres.

**Plain summary.** Both runs complete the plan (outcome SUCCESS). With the blend on (Run 1), the trim's z returns exactly to its grasp value at the end of the ramp, and the z the trim had learned during the lift (18.85 mm) is handed over to the feedforward's mass term to 0.30 mm. The x part of the trim, however, is not handed over: it keeps integrating during the ramp and winds up to −15.3 mm, while the feedforward gives x only 0.2 mm. The pad leaves its planned path by 10.9 mm in x in both runs. With the blend off (Run 2), z integrates through the ramp and falls to +4.9 mm, and the pad rises about 8 mm above the path in z.

## Step 0 (state at the start)
- Processes: no MuJoCo or teleop process (VERIFIED, pgrep); no tmux server (VERIFIED). One simulation ran at a time.
- git log -5 at the start: c52d09c (build), 7873ad5 (part 2 run result), 6b9ba1d (launcher), 817683e (part 1 report), 3d44e7d (trim blend build). git log -3 at the end: see the commit of this report.
- Other Claude processes: PID 253425 has exited. PID 278373 is still running, working directory /home/vboxuser/projects/openarm-tracer-sim, started 17:58:20 (VERIFIED from /proc and ps). I cannot tell whether it is this session (NOT FOUND).
- Uncommitted: the trace logging in src/mpvi_modules.py and the marker removals in src/mpvi_modules.py, src/task_config.py and src/gravity_ff.py. Saved as data/trim_zonly_20261102/uncommitted_part_build.diff (106 lines). Kept out of the commit, as asked. The logging reads state only.

## Build (what changed in code, M230 to M234)
| file | change | why |
|---|---|---|
| src/mpvi_modules.py `_sag_trim_step` | z only is blended (or frozen when no ref); x, y and rotation integrate during a ramp-in; ramp-out still holds all; the switch is checked here | z-only handoff (M230, M231); control switch (M233) |
| src/mpvi_modules.py `mark_trim_reference` | stores the z value only | M231 amended |
| src/held_mass_ramp.py | `begin_load_change` and its docstring mentions removed | M234 |
| src/task_config.py PICK_PLACE_CFG | `trim_blend_enabled` = True (test switch for a control run, not a design choice) | M233 |
| experiments/held_mass_ramp_test.py | rewritten: no begin_load_change; z-only and switch checks | the brief's tests |
| experiments/bounce_handoff_run.py | logs z-only reference and full; second argument `off` sets the switch for Run 2 | the runs |
| experiments/bounce_handoff_metrics.py | reads z-only values; item 6 base pose from the checkpoint; hotplate from the scene site | this report |

Unit tests: experiments/held_mass_ramp_test.py, 38 checks, all PASS (VERIFIED). They cover: z plus progress times (full z minus ref z) stays at full z within 1e-9; x, y and rotation change during the ramp-in; with no ref only z freezes; with the switch False nothing is frozen or blended; ramp-out freezes everything until end_load_change; a reset clears ref and full.

Forbidden-word check: no "pick", "place" or "gripper" in the trim helper (VERIFIED, grep).

## 1. Steps (first attempt; pad error = measured minus waypoint at the step's last tick)
| step | Run 1: passed first attempt | Run 1: final pad error (mm) | Run 1: time (s) | Run 2: passed first attempt | Run 2: final pad error (mm) | Run 2: time (s) |
|---|---|---|---|---|---|---|
| pick IN | yes | 1.288 | 3.54 | yes | 1.288 | 3.54 |
| pick CLOSE | yes | not a move | 1.01 | yes | not a move | 1.01 |
| pick UP | yes | 0.590 | 2.99 | yes | 0.590 | 2.99 |
| pick CHECK (weighing) | yes | not a move | 0.01 | yes | not a move | 0.01 |
| pick OVER | yes | 2.414 | 1.53 | yes | 2.414 | 1.53 |
| pick DOWN | yes | 0.403 | 3.68 | yes | 0.403 | 3.68 |
| place OVER | yes | 6.367 | 1.68 | yes | 6.687 | 1.68 |
| place DOWN | yes | 0.874 | 1.26 | yes | 0.320 | 2.03 |
| NAV | NOT AVAILABLE (no completion line in the log) | | | NOT AVAILABLE | | |

VERIFIED (run logs and the trace). No step timed out. Pick steps are identical in both runs because the blend acts only from the weighing on. The place time (PLACE: OVER to PLACE: BACK, episode log): Run 1 4.30 s; Run 2 5.10 s. Final beaker position: Run 1 x 0.6495, y 0.6602, z 1.0279 m; Run 2 x 0.6504, y 0.6601, z 1.0279 m. The hotplate top site in the scene is at x 0.6500, y 0.6600, z 1.0280 m (VERIFIED from the model at the final checkpoint). Horizontal distance from beaker to site: 0.5 mm (Run 1) and 0.4 mm (Run 2). INFERRED: the beaker rests on the hotplate, from the SUCCESS line and this position; the hotplate top height was not compared separately.

## 2. Metric 1 (baseline = pad height on the tick before the held mass; aligned on that tick; window 200 ticks)
| run | rise (mm) | drop (mm) | swing (mm) |
|---|---|---|---|
| Run 1 (z-only blend) | +2.98 | -4.44 | 7.42 |
| Run 2 (control, blend off) | +8.09 | -0.03 | 8.12 |
| earlier full blend run | +0.00 | -2.80 | 2.80 |
| sag on run (existing) | +17.70 | -18.45 | 36.14 |

VERIFIED (computed from the traces; the earlier numbers from the earlier metrics report). The baseline is 1120.06 mm absolute on tick 11599 in both runs. What it means: the bounce of the sag-on run is far smaller in all three handoff variants. Run 2 has the largest rise of the three handoff runs, because z is not handed over. INFERRED: the place's rise comes from the z part of the trim being left in the command when the mass ramps in.

## 3. Metric 2 (e = measured pad z minus reference pad z, from the CLOSE tick to 3 s after the end of DOWN)
| quantity | Run 1 | Run 2 |
|---|---|---|
| e just before the ramp-in | -0.442 mm (tick 11598; the CHECK tick has no reference) | -0.442 mm |
| maximum after the ramp-in starts | +2.982 mm (tick 11789) | +8.095 mm (tick 11716) |
| minimum after the ramp-in ends | -3.427 mm (tick 11956) | -4.044 mm (tick 11957) |
| e at the end of DOWN | +0.873 mm (tick 12041) | +0.301 mm (tick 12157) |
| zero crossings after the ramp-in ends | 10 | 9 |
| time between the first two crossings | 0.413 s | 0.013 s |

VERIFIED (trace). The e values are from the same tick ranges in both runs. What it means: the z error is smaller with the blend on (its largest value is 3.0 mm instead of 8.1 mm).

## 4. Excursion from the planned path (largest absolute value, ramp-in start to 3 s after the ramp end; move ticks)
| run | x (mm) | y (mm) | z (mm) | rotation (deg) |
|---|---|---|---|---|
| Run 1 (z-only blend) | +10.948 | -1.167 | -4.438 | +2.120 |
| Run 2 (control) | +10.865 | -1.360 | +8.095 | +2.241 |
| earlier full blend run | +16.511 | -2.044 | +3.984 | +2.522 |

VERIFIED. What it means: the x excursion (about 11 mm) is the same with and without the blend, so the blend does not cause it. INFERRED: the x trim winding up during the ramp (item 5) and the lag of the OVER move account for it, which is not separated here.

## 5. Trim components at the saved reference, at ramp-in start and at ramp-in end (trim in mm and deg)
Run 1 (blend on):
| component | at reference (tick 11000) | at ramp-in start (11599) | at ramp-in end (11750) | change over the ramp |
|---|---|---|---|---|
| x (mm) | -2.081 | +3.049 | -15.333 | -18.381 |
| y (mm) | -2.660 | -3.059 | -1.303 | +1.756 |
| z (mm) | -1.308 | +17.538 | -1.308 | -18.846 |
| rotation x, y, z (deg) | -0.569, -0.157, -0.786 | -3.201, +0.497, +0.549 | -1.696, -0.691, -2.478 | +1.505, -1.189, -3.026 |

Run 2 (control, blend off):
| component | at reference (tick 11000) | at ramp-in start (11599) | at ramp-in end (11750) | change over the ramp |
|---|---|---|---|---|
| x (mm) | -2.081 | +3.049 | -14.976 | -18.025 |
| y (mm) | -2.660 | -3.059 | -1.172 | +1.887 |
| z (mm) | -1.308 | +17.538 | +4.870 | -12.669 |
| rotation x, y, z (deg) | -0.569, -0.157, -0.786 | -3.201, +0.497, +0.549 | -1.346, -0.710, -2.381 | +1.854, -1.207, -2.929 |

VERIFIED (trace). Handoff values (Run 1): the saved z reference is -1.3077 mm; the latched full z is +17.5383 mm; full minus reference is +18.8460 mm. The trim's z at the end of the ramp minus the reference is +0.000000 mm, to within the brief's 1e-6 m. The handoff timing is unchanged from the full blend run: target set at tick 11599, first blended update at tick 11601 (two ticks later), progress 2.3e-5 there.

## 6. Corrected feedforward (read only), with the actual base pose
- Base pose used: the run's full state at checkpoint tick 11550 (the last checkpoint before tick 11599; 49 ticks earlier). Base x 0.6343 m, base y 1.1388 m, base z 0.1527 m, yaw 269.806 deg (equal to -90.194 deg, the episode log's value at 78.70 s, as a cross-check). Pedestal 0.1494 m. Frame: world, metres; rotation in degrees about the world axes.
- Check: the pad centre from these joints is (0.8503, 0.6599, 1.1201) m; the logged pad at tick 11550 is the same to 0.006 mm (VERIFIED).
- The feedforward removes, at the pose: 0.205 kg (the weighed reading): x +0.199 mm, y -2.401 mm, z +19.150 mm; rotation -3.715, -0.067, -0.020 deg. 0.200 kg (the true mass): x +0.195, y -2.343, z +18.687 mm; rotation -3.625, -0.066, -0.019 deg.
- The trim's change over the ramp (Run 1): x -18.381 mm, y +1.756 mm, z -18.846 mm; rotation +1.505, -1.189, -3.026 deg. Run 2: x -18.025, y +1.887, z -12.669; rotation +1.854, -1.207, -2.929 deg.

What it means: in z the blend gives back exactly the feedforward's share (18.85 mm removed against 19.15 mm the feedforward takes, a 0.30 mm difference), so the handover is right in z. In x the trim removes 18.4 mm while the feedforward share is 0.2 mm; y and rotation do not match either. So the x and y parts still integrate onto a load the feedforward does not carry. The control run shows that without the blend z ends at +4.9 mm, which is far from the feedforward's share.

## 7. Hotplate position: where it is read, and where the place gets it (read only, no change)
- Readers of hotplate_xy (file and line) and what each does:
  - src/task_config.py:51: the definition (0.80, 0.66) m, the unshifted default layout.
  - src/task_beaker_hotplate.py:85: apply_task_poses writes the hotplate body's x, y into the model from the config it is given. In the run that config is demo_video's MOVED layout (src/demo_video, experiments/demo_video.py:44, shift -0.15 m), so the scene's hotplate is at x 0.65 m. VERIFIED.
  - src/task_points.py:47: the finder and full check use TASK_CFG (unshifted) for the set-down points. Not used during the run; a layout other than the default would make these points wrong. INFERRED from the code.
  - src/world_model.py:361: StaticTaskSource (the configured world model; dry runs only, not used by the simulated run).
  - experiments: many scripts read TASK_CFG hotplate_xy or set the hotplate from it; they are not part of the run.
  - experiments/bounce_handoff_metrics.py: no longer reads it (it reads the scene site).
- The place skill takes the hotplate position from the world model (src/mpvi_modules.py:259, `hp = world.object_pose(dest)["position"]`), not from the config. The world model in the run is the simulator source (src/mpvi_modules.py:117 and src/world_model.py:259, SimGroundTruthSource), and it writes the hotplate entry from the scene site `hotplate_top` (src/world_model.py:317, hp_top; written at line 325). VERIFIED.
- World-model calls (src/mpvi_modules.py unless stated):
  - 123 refresh; 150 stance (the navigation target's stance).
  - 234 object_pose(beaker) (pick grasp); 239 object_height(beaker); 241 object_pose(hotplate, carry height); 242 bench_top_z (clearance).
  - 258 object_pose(beaker) (place); 259 object_pose(hotplate) (place destination).
  - 334 held_by (recovery).
  - 466 and 477 object_pose(beaker) (grasp height and lift).
  - 583 and 584 object_pose and object_height of the object (the grounding check).
  - src/orchestrator.py:121 refresh and 122 get(obj).
- What the world model returns for the hotplate position: the scene site `hotplate_top`, at (0.6500, 0.6600, 1.0280) m at the run's end (VERIFIED). It matches the scene site (0.65, 0.66), and it does not match TASK_CFG's 0.80.

## 8. begin_load_change: mentions before removal and now (grep)
Before the removal (this task's start, grep): src/held_mass_ramp.py (the definition at line 63 and the docstrings at lines 13 and 75), experiments/held_mass_ramp_test.py (lines 88, 105, 121, 124, 135), docs/mpvi_decisions.md (M231, line 243), docs/progress.md (line 486), and three reports and the metrics script of the earlier parts (historical or descriptive).

Now (grep, after the removal):
```
docs/progress.md:486             (historical log entry)
docs/mpvi_decisions.md:243       M231: "The sentence on begin_load_change is removed (M234)."
docs/mpvi_decisions.md:246       M234: the removal row
docs/design_plan.md:618          jj: the removal item
docs/reports/20261102_1806_*     (historical report)
docs/reports/20261102_1903_*     (historical report)
docs/reports/20261102_1926_*     (historical report)
experiments/bounce_handoff_metrics.py   (its note now says "removed (M234)")
```
No definition and no call remain in src/ or in the tests. The historical reports and the progress log keep their past wording by design; the live code, tests, decisions log and design plan are updated.

## Limits and NOT BUILT, needs decision
1. The x and y trim integrate onto a mass the feedforward does not carry, and they wind up during the ramp (x to -15.3 mm). Options: (a) hold x and y during the ramp as well (this is the rule David did not choose); (b) learn x and y as a separate share in the feedforward; (c) leave as is. Not decided.
2. The trace's `frozen` column is the ramp-state flag (true during any ramp), so during a ramp-in it reads 1 in both runs even though in Run 2 nothing is held. Read the change columns, not the flag, to see what was held.
3. The place's extra pad error (OVER 6.4 to 6.7 mm) is not separated between the handoff and the OVER move's lag. Not tested (no run without the handoff at this scene in this part).
4. NAV completion is not logged.
5. The control run uses the config switch by launcher argument (set in the process, not in the file); the file's default stays True.

## Files
- Commits: build c52d09c; launcher and metrics are committed with this report.
- Uncommitted logging: data/trim_zonly_20261102/uncommitted_part_build.diff.
- Outputs: data/bounce_zonly_20261102/ (run1, run2, metrics_run1.md, metrics_run2.md).
