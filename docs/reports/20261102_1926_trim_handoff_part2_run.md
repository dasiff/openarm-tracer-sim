# Trim handoff, part 2: one run of the committed build (decisions-log M230 to M232)

Written for: David (design review). Branch held-weight-ramp-sag-off. Build: 3d44e7d (committed). Launcher: 6b9ba1d. Run log: data/logs/handoff_run_20261102_191925.log. Outputs: data/bounce_handoff_20261102/run/ (per-tick trace bounce_ticks.csv, episode log, recording). Metrics: experiments/bounce_handoff_metrics.py, output data/bounce_handoff_20261102/metrics_part2.md. Acronyms: IN, CLOSE, UP, CHECK, NAV, OVER, DOWN, OPEN, BACK, HOME = pick and place steps (NAV = navigation); ref = trim saved at the grasp; full = trim latched on the first ramp-in tick; e = measured pad z minus reference pad z; kg = kilograms; deg = degrees.

**Plain summary.** The plan finished (outcome SUCCESS). The lift that failed in the freeze runs (20.2 mm off) now passes with 0.59 mm. The handoff works as designed: the trim moves from full to ref over exactly the mass ramp and ends equal to ref to zero difference. The pad is far off its planned path in x during the ramp (up to 16.5 mm), while z stays within 2.5 mm. The old place "bounce" (rise +17.7 mm in the sag-on run) does not appear: rise 0.00 mm. Before a claim about the bounce is made, a run without the handoff at this same scene is needed (not done).

## Step 0 (state at the start)
- Processes: no MuJoCo or teleop process (VERIFIED, pgrep); no tmux server (VERIFIED). One simulation ran at a time.
- The two other Claude processes: PID 253425, working directory /home/vboxuser/projects/openarm-tracer-sim, started 16:08:03; PID 278373, same directory, started 17:58:20 (VERIFIED from /proc and ps). Whether either is this session is not known (NOT FOUND).
- git log -5 at the start: 817683e (trim handoff report), 3d44e7d (trim handoff build), 70f5e37 (read-only sag check), aa530e8 (freeze runs report), 5ed1b28 (metrics).
- Uncommitted: the trace logging in src/mpvi_modules.py (extended for this run: handoff values, ramp direction and progress, freeze flag, pad rotation error, measured pad position) and the marker removals in src/mpvi_modules.py, src/task_config.py and src/gravity_ff.py. Saved as data/bounce_handoff_20261102/uncommitted_part_run.diff (107 lines). The logging only reads state; the run's behaviour is the committed build's (VERIFIED from the code; no separate baseline check).

## 1. Steps (first attempt; trace and run log; pad error is the measured pad minus its waypoint at the step's last tick)
| step | passed first attempt | pad error at end (mm) | time taken (s) | trim frozen at end |
|---|---|---|---|---|
| pick IN | yes | 1.288 | 3.54 | no |
| pick CLOSE | yes | not a move | 1.01 | no |
| pick UP | yes | 0.590 | 2.99 | no |
| pick CHECK (weighing) | yes | not a move | 0.01 | no |
| pick OVER | yes | 2.414 | 1.53 | no |
| pick DOWN | yes | 0.403 | 3.68 | no |
| place OVER | yes | 4.130 | 1.68 | no |
| place DOWN | yes | 0.459 | 2.02 | no |
| NAV (navigation segments) | NOT AVAILABLE (no arm waypoint; the log has the segment starts but no completion line) | NOT AVAILABLE | NOT AVAILABLE | - |

VERIFIED. No step timed out. The UP step that timed out at 20.2 mm in the freeze runs now passes at 0.59 mm. IN gives 1.288 mm, the same as the earlier sag-on run (1.29 mm). What it means: the trim now learns during the lift, which is what the lift needs.

## 2. Handoff timing (VERIFIED from the trace)
- Ramp-in target set at tick 11599 (78.7267 s).
- First blended trim update: tick 11601 (78.7400 s), the first move tick after the weighing (the held step has no move, so no trim update runs during it).
- Ticks between the target and that update: 2.
- Ramp progress on that tick: 2.3e-5 (minimum jerk, 0 to 1).
- The latched full value is first visible in the trace on tick 11602 (the trace logs before each tick's own update).
- Ramp end (duration 1.0 s): tick 11749.

## 3. Handoff values (VERIFIED from the trace; positions in mm, rotations in deg)
| quantity | x (mm) | y (mm) | z (mm) | rotation x, y, z (deg) |
|---|---|---|---|---|
| trim at the saved reference (tick 11000, the first traced tick after the grasp) | -2.0809 | -2.6599 | -1.3077 | -0.5693, -0.1571, -0.7863 |
| trim latched as full (tick 11602) | +3.0485 | -3.0585 | +17.5383 | -3.2009, +0.4973, +0.5485 |
| full minus reference | +5.1294 | -0.3985 | +18.8460 | -2.6316, +0.6544, +1.3348 |
| trim at the end of the ramp-in (tick 11750) minus reference | 0.000000 | 0.000000 | 0.000000 | 0.000000, 0.000000, 0.000000 |

The largest difference between the trim at the end of the ramp-in and the reference is 0 (VERIFIED), well inside the 1e-6 the brief asks for. What it means: the blend lands exactly on the reference, so the trim does not keep a step at the end of the ramp.

## 4. Pad error to the reference (measured minus reference; mm, deg; the nearest move tick is used where the step has no reference)
| when | tick used | x (mm) | y (mm) | z (mm) | rotation (deg) |
|---|---|---|---|---|---|
| ramp-in start (target set) | 11598 (last move tick before CHECK) | +0.375 | -0.114 | -0.442 | +0.408 |
| ramp progress 0.25 | 11653 | +9.626 | -0.141 | -0.254 | +1.193 |
| ramp progress 0.5 | 11674 | +14.576 | -0.595 | -0.960 | +2.070 |
| ramp progress 0.75 | 11696 | +16.453 | -1.201 | -2.457 | +2.520 |
| ramp progress 1.0 (ramp end) | 11749 | +13.942 | -2.042 | -2.333 | +2.079 |
| 1 s after the ramp end | 11899 | -1.898 | -0.978 | +3.980 | +0.428 |
| 3 s after the ramp end | 12155 (end of DOWN) | -0.143 | -0.124 | +0.418 | +0.141 |

Largest excursion from ramp start to 3 s after the ramp end (move ticks): x +16.511 mm, y -2.044 mm, z +3.984 mm, rotation +2.522 deg (VERIFIED from the trace).

What it means in plain words: during the ramp the pad moves about 14 to 16 mm in x away from the planned path. The z error stays within about 2.5 mm. The x error is much larger than the z error. The trim's x part (5.13 mm) is removed while the mass feedforward gives no matching x share (about 0.2 mm), so the x error is at least partly explained by the handoff (INFERRED). The rest may come from the OVER move's lag behind its reference (INFERRED; a run without the ramp would separate the two, not done).

## 5. Feedforward estimate at that pose (VERIFIED method; INFERRED pose)
The pose is checkpoint 11550, the last 150-tick checkpoint before the ramp-in (the joint angles of that checkpoint are used; the ramp started at 11599, 49 ticks later). The pad position from the checkpoint joints matches the logged pad at tick 11550 to 0.1 mm (VERIFIED), so the full state, including the base yaw, is correct.

| quantity | x (mm) | y (mm) | z (mm) | rotation x, y, z (deg) |
|---|---|---|---|---|
| feedforward estimate, 0.205 kg (the weighed reading) | +0.199 | -2.401 | +19.150 | -3.715, -0.067, -0.020 |
| feedforward estimate, 0.200 kg (the true mass) | +0.195 | -2.343 | +18.687 | -3.625, -0.066, -0.019 |
| full minus reference (section 3) | +5.129 | -0.399 | +18.846 | -2.632, +0.654, +1.335 |
| full minus reference minus the feedforward (0.205 kg) | +4.930 | +2.002 | -0.304 | +1.083, +0.722, +1.354 |

What it means: in z, the trim's learned part (18.85 mm) matches the feedforward's estimate (19.15 mm) to 0.30 mm. In x, y and rotation it does not: the trim has learned about 4.9 mm of x and 2.0 mm of y that the feedforward does not produce, and about 1 to 1.4 degrees of rotation. The likely source is the arm's posture and solver offsets, which the gravity model does not include (INFERRED). This is why the handoff leaves x and y with an error (section 4).

## 6. Metric 1 (baseline = pad height on the tick before the held mass; aligned on that tick)
| run | rise (mm) | drop (mm) | swing (mm) |
|---|---|---|---|
| sag on (existing) | +17.70 | -18.45 | 36.14 |
| sag off in place only (existing) | +2.99 | -4.05 | 7.04 |
| this run (handoff) | 0.00 | -2.80 | 2.80 |

This run's baseline is 1120.06 mm absolute (tick 11599). VERIFIED. The rise is zero: the pad never rises above its baseline in the 200 ticks after the held mass first reads nonzero. What it means: the place OVER did not bounce up in this run. INFERRED: the old sag-on bounce came from the trim's z part (17.5 mm) staying in the command while the mass feedforward ramped in; the handoff removes that part in step with the mass, and z matches the feedforward to 0.3 mm (section 5). This needs a run without the handoff in this same scene to confirm, which was not done.

## 6b. Metric 2 (e = measured pad z minus reference pad z; mm; from the CLOSE tick to 3 s after the end of DOWN)
- e just before the ramp-in starts: -0.442 mm (tick 11598; the CHECK tick has no reference).
- Maximum of e after the ramp-in starts: +3.984 mm (tick 11900, during OVER).
- Minimum of e after the ramp-in ends: -3.574 mm (tick 11957).
- e at the end of DOWN: +0.418 mm (tick 12155).
- Zero crossings of e after the ramp-in ends: 8; time between the first two: 0.820 s (VERIFIED).

## 7. Freeze and trim resets (VERIFIED from the trace)
| window (ticks) | what | trim at start (x, y, z mm; rot deg) | trim at end | largest change inside |
|---|---|---|---|---|
| 11601 to 11748 | ramp-in: frozen flag on, the trim blends from full to ref | +3.049, -3.058, +17.538 | -2.081, -2.660, -1.307 | rotation about x 0.0459 rad (2.63 deg), by design |
| 12156 to 12363 | ramp-out freeze (M226 c), ends at the gripper-open event | -1.071, -0.659, -3.418 | -1.071, -0.659, -3.418 | 0 |

- Ramp-out freeze: from tick 12156 to 12363 the trim is constant: its largest change inside the window is 0 (VERIFIED).
- Resets of the trim: run start (construction); the pick skill start at tick 9687 is not a reset (sag_reset_at_skill_start is False); the gripper-open event at tick 12363 zeroes the trim and clears ref and full: the trim at tick 12364 is 0 (VERIFIED from the trace). Recovery: none.
- Between the end of the ramp-in and the gripper open, the trim integrates normally (not frozen): it moves from -2.08, -2.66, -1.31 mm to -1.07, -0.66, -3.42 mm during the place (VERIFIED; INFERRED: the place's own residual sag).

## 8. Final state (VERIFIED unless marked)
- Pad height at the end of DOWN (tick 12155): 1084.81 mm absolute.
- Beaker at the end of the episode (last checkpoint, tick 13650): x 0.6503, y 0.6602, z 1.0279 m.
- Hotplate top site (the success check's site, from the model at the same tick): x 0.6500, y 0.6600, z 1.0280 m. The beaker is 0.2 mm from the site in the horizontal plane and sits at its height.
- Does the beaker rest on the hotplate? INFERRED yes: the plan's success line says SUCCESS, and the beaker's position is on the hotplate top site.
- Inconsistency, not resolved: TASK_CFG hotplate_xy is (0.80, 0.66) m, but the hotplate's top site in the scene is at x 0.65 m. Either the config value is stale or the scene places the hotplate elsewhere. The success check uses the scene site (VERIFIED from SUCCESS_CFG), so the run is consistent with the scene. Needs David to correct the config (NOT BUILT).
- Place duration (episode log, PLACE: OVER to PLACE: BACK): 5.10 s.
- Outcome line: "outcome SUCCESS; plan failure None; status ['done', 'done', 'done', 'done']".

## 9. Where the begin and end functions are called (VERIFIED by grep, working tree)
- begin_load_change: no call in src/ or in the launchers. It is kept in src/held_mass_ramp.py. Only the unit tests call it.
- end_load_change: three call sites in src/mpvi_modules.py: the skill-start reset (only when sag_reset_at_skill_start is on, line 293), the recovery start (line 320), and the gripper-open reset (line 466). It also has its definition in src/held_mass_ramp.py.

## Corrections to earlier reports
- docs/reports/20261102_1857_sag_feedforward_check.md: its feedforward x, y and rotation were computed with the base at its default yaw (the script set only the arm joints). The base is at -90 degrees. The corrected values are in that report's second correction note. The z values stand. The first correction there (close tick available) stands.

## NOT BUILT, needs decision
1. TASK_CFG hotplate_xy (0.80, 0.66) does not match the scene's hotplate top site (0.65, 0.66). Options: correct the config, or move the hotplate in the scene. Not changed here.
2. The x and y mismatch between the trim and the feedforward (4.9 mm x, 2.0 mm y at the handoff). Options: learn a separate x and y share in the feedforward, or accept the error and treat it as posture. Not decided.
3. A run without the handoff at this scene, to check the bounce claim and to separate the handoff's x error from the OVER move's lag. Not run here (a second simulation is a new task).
4. NAV completion is not logged, so NAV has no completion time. Option: log the navigation completion. Not built.

## Files
- Commit: this report, the metrics script, the design plan update, the docs (see the commit message). The trace logging stays uncommitted: data/bounce_handoff_20261102/uncommitted_part_run.diff.
- Outputs: data/bounce_handoff_20261102/run/ and data/bounce_handoff_20261102/metrics_part2.md.

## Correction (added in the z-only task)
The "inconsistency" in section 8 is not one. The run used demo_video's MOVED layout (experiments/demo_video.py:44, hotplate_xy shifted by -0.15 m), so the hotplate in the scene is at x 0.65 m. TASK_CFG hotplate_xy (0.80, 0.66) is the unshifted default. The earlier comparison used the unshifted default. The beaker's position is correct for the run. See docs/reports/20261102_1951_trim_zonly_runs.md, section 7.
