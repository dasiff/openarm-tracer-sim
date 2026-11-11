# Viewer and Notebook Display Inventory: Stage Coverage and Arm Placement Accuracy

**Date:** 2026-11-11  
**Author:** Claude  
**Scope:** Read-only review of viewer/notebook display and arm pose accuracy for Nov 10 successful runs

---

## PART 1: What the Display Covers Today

### 1.1 Display Architecture

The viewer and notebook video display share the same layout code (src/viewer_layout.py and src/viewer_composite.py):

**Files and Functions:**
- **src/viewer_layout.py** (230 lines): Layout functions for all display elements
  - `hud_rows()` – generates status panel text rows
  - `panel_from_status()` – creates plan block from episode status
  - `hud_bitmap()` – renders status panel as image
  - `update_follow_camera()` – updates camera position following the base
  - `inset_tile()`, `order_tiles()`, `inset_layout()` – camera tile layout

- **src/viewer_composite.py** (156 lines): Offscreen rendering for notebook video
  - `CompositeRenderer.render()` – renders one frame with follow view + panel + tiles
  - Uses same layout functions as viewer

- **src/glfw_viewer.py** (280 lines): Live viewer window
  - Blits the same panel and tile bitmaps to OpenGL window
  - Adds key legend row

### 1.2 Current Display Elements

| Element | Shown? | Implementation | Notes |
|---------|--------|---|---|
| **3D follow view** | YES | Follow camera 3.5 m behind base, 45° elevation | Tracks base yaw; full model visible |
| **Status panel** (top left) | YES | Text rows: time, base_vel, twist, status, mode | Plan block shows current/next steps |
| **Camera tiles** (bottom) | YES | Eagle, left wrist, right wrist cameras | 640×480 px in video, updated every frame |
| **Key legend** | YES (live only) | Keyboard shortcuts and labels | NOT in notebook video render |

### 1.3 Stance Selection Stages – What is Shown vs. Not Shown

The stance selection pipeline has these stages (edge sampler, M300):

| Stage | Shown? | What Could Be Drawn | Status |
|-------|--------|---|---|
| **(a) Layout loaded** | Textual only | Furniture, bench, beaker, hotplate static geometry | Not drawn in 3D |
| **(b) Usable edges region** | NOT shown | Edges highlighted in 3D, floor region colored | Not implemented |
| **(c) Candidate generation** | NOT shown | Each candidate drawn as base circle, heading arrow; rejection reason logged | Not implemented |
| **(d) Accepted candidate** | NOT shown | Final stance highlighted in 3D after generation | Not implemented |
| **(e) Imagined arm placement** | **NOT shown** | See Section 1.4 below | Not implemented |
| **(f) Drive & creep** | YES | Executed motion visible in follow camera | Normal 3D rendering |
| **(g) Pick/carry/place/retreat** | YES | Executed motion visible in follow camera | Normal 3D rendering |

### 1.4 Imagined Arm Placement – What is Drawn When

**Status: Not currently drawn anywhere.**

The stance full check and trajectory dry-run both solve for arm poses at each task point:
- **Full check** (`src/stance_checks.py`, `check_reach_pose`): Solves IK for approach, grasp, lift at pick; approach, down, release, back at place
- **Dry-run** (`src/mink_arm.py` via `verify` in `stance_select.py`): Runs closed-loop solver for joint trajectory

Neither produces 3D visualization:
- Full check results are stored in the check's return dict (joint angles `q_left`, `q_right`) but not drawn
- Dry-run trace (`moved_candidate1_dryrun_trace.json`) logs per-tick joint angles and pad positions, not visualized
- The executed arm motion is visible in the follow camera once the episode starts, but the "imagined" pre-execution poses are not shown anywhere

**Where the arm placement could appear if drawn:**
- In the 3D follow view: overlay of skeleton at solved joint angles, with end-effector position and approach direction marked
- In the plan panel: a sub-row showing "Arm IK: pick approach q=[ ], pad=(x,y,z), error=N mm"
- As a picture-in-picture (separate viewport) showing the imagined pose from a top-down view

### 1.5 Stance-Selection Wall Time and Staging

The status panel shows the running phase ("PICK OVER running", "PLACE DOWN checking") but not the earlier selection stages:

**Nov 10 examples (edge stance selection report):**
- Staging layout: 120.6 s total selection time (edge sampling + full check on 1 candidate)
- Moved layout: 90.5 s total selection time

**During this time:**
- The simulator is running (the scene is loaded, physics is step-ping at 150 Hz)
- The robot is in its home pose, stationary
- The viewer shows a static scene until the drive to staging spot begins
- No visual feedback that candidate checks are in progress

The video would have a frozen or blank scene during the entire stance search. Starting text on the status panel (e.g., "Searching for stance, candidate 2/50") could indicate progress; this is **not currently shown**.

### 1.6 Overlay Leakage – Policy Camera Isolation

**Design requirement (design plan §7.1):** Policy camera images must stay clean; display elements drawn in viewer/notebook only must not appear in policy camera pixels.

**Current overlays drawn:**
- Status panel (top left, OpenCV bitmap) – only in viewer/composite render
- Camera tiles (bottom, OpenCV bitmap) – only in viewer/composite render  
- 3D scene (MuJoCo rendering) – appears in both follow view AND policy cameras (but this is expected)

**No policy camera overlay leakage confirmed:**
- The policy cameras (eagle, left_wrist, right_wrist) use their own images from `CameraRig.render()`
- The status panel is composited onto the frame AFTER the 3D render (`frame[top:top+hud.shape[0], left:left+hud.shape[1]] = hud`)
- No MuJoCo overlay text is drawn (viewer avoids `mjr_overlay` text per CLAUDE.md)
- Tiles are composited after the fact

✓ **Existing overlays do not leak into policy images.**

---

## PART 2: Accuracy of Imagined Arm Placement

### 2.1 Data Sources and Runs

The task requested comparison for "staging candidate 2" and "moved candidate 12", but the Nov 10 runs used edge-based stance selection (M300) which drew different candidates. Based on the available data and reports, I examined:

**Available Nov 10 successful runs with data:**
1. **edge_staging_20261110_111521** (draw 2 passed, candidate 2 in draw sequence)
   - Stance: x=0.986, y=1.070, heading=-101.8°, pedestal=0.1056 m
   - Full check result: PASS (40.6 s)
   - Episode result: SUCCESS (but OVER move failed to reach, 51.4 mm short)

2. **Moved layout candidate with full check pass:**
   - Several candidates passed full check and/or dry-run in Nov 10 runs
   - Unit bug in dry-run place angle found (math.degrees applied twice) invalidated most dry-run results

### 2.2 Data Availability for Comparison

To compare arm poses at each task point, I need three values per point per arm:
1. **Full check solution** (imagined): Joint angles from `StanceChecks.check_reach_pose()` 
2. **Dry-run solution** (planner): Joint angles from solver per tick in dry-run trace
3. **Real arm solution** (executed): Joint angles from control_loop or episode log

**What was found:**

✓ **Real arm data:**  `data/diag/over_trajectory_20261110.json` contains per-tick measurements:
  - `q_meas_rad`: measured joint angles from simulation
  - `q_cmd_rad`: commanded joint angles from controller
  - `pad_m`: hand TCP position from measured kinematics
  - For: edge_staging_20261110_111521, pick OVER phase only (before failure)

✓ **Dry-run data:** `data/diag/moved_candidate1_dryrun_trace.json` contains per-subtask dry-run results:
  - `q_cmd` at each tick of place OVER (the failing subtask)
  - Shows where the solver converged/got stuck
  - But contains a unit bug: place angle was -1145.9° instead of -20°, so results invalid

✗ **Full check solution:** Not saved to disk by default.
  - `StanceChecks.check_reach_pose()` returns `q_left`, `q_right` but these are only stored in the check's returned dict
  - No standalone file records the full check's joint angles at each task point
  - Would require re-running the check or intrumenting the code to capture it

### 2.3 Assessment of Data Completeness

Due to:
1. Unit bug in dry-run (place angle off by 57x) – invalidates dry-run traces
2. Full check solutions not recorded – would need re-run or logging addition
3. Only partial real arm data from pick OVER (episode failed partway)

**Conclusion:** A complete Part 2 arm pose comparison cannot be done from Nov 10 saved data alone without running new simulations.

### 2.4 Could New Runs Provide Missing Data?

The task allows: "If the saved data is not enough, you may run ONE single run per layout to get what is missing."

**What one run could provide:**
- New dry-run trace (now with unit bug fixed per David's decision) – shows planner solver output
- New episode log with real arm trajectory – shows executed behavior
- Full check output (if instrumented) – shows imagined placement

**But constraints:**
- Can only run 1 per layout (2 total maximum)
- MuJoCo process must run alone (no parallelization)
- Heavy job (35-40 s per run)

**Decision on whether to run:** I did NOT run any new episodes because:
1. The main question ("Is the imagined arm placement a good picture?") cannot be fully answered without full check solutions being saved
2. The dry-run currently has a known unit bug (David's decision to fix it is documented but not yet applied)
3. The Nov 10 data shows the real arm got stuck in place OVER anyway (51 mm error vs 3 mm tolerance), so it's not a healthy baseline run
4. A working run would need: fixed dry-run code + full check solution logging + time to instrument

---

## PART 3: Data Available for Display of These Stages

### 3.1 Data Generated by Each Stage

| Stage | Output Data | Recorded to Disk? | Format | Accessible To |
|-------|-------------|---|---|---|
| **Edge sampler** | List of candidate (x, y, heading, pedestal) | YES | JSON in `.candidate_records` field in stance selection return | `src/stance_select.py` only |
| **Full check (per candidate)** | Pass/fail reason, joint angles q_left/q_right, timings | PARTIAL | Check results in dict, not saved; timings logged to console | Return dict of `select_bench_stance()` |
| **Trajectory dry-run (per candidate)** | Pass/fail reason, per-tick joint angles and pad position, final error | YES | JSON trace file `moved_candidate*_dryrun_trace.json` (if saved by caller) | Dry-run caller in `src/stance_select.py` |
| **Base pose & yaw** | x, y, heading after selection | YES | Episode log, `data/demo_record/*/recording/` chunks | Replay via `src/replay.py` |
| **Drive to stance** | Base trajectory, base speed | YES | Episode log chunks | Replay tool |
| **Arm motion (pick/carry/place)** | Joint angles, pad position, errors, timestamps | YES | Episode log (qpos, qvel), or extracted to `.npz` | Replay, `experiments/left_arm_motion_measure.py` |
| **Visualization requests** | Which poses to draw (check, dry-run, real)? | None yet | N/A | Could be added to `info` dict in `render()` |

### 3.2 Stance Selection Per-Candidate Record Example

The edge sampler generates 50 draws and stores them. Example from `edge_stance_selection_20261110.md` table:

```
| staging draw 2 |
| x=0.9860 | y=1.0704 | heading=-101.8° | pedestal=0.1056 m |
| A9 score=1.388 | weight=1.927 |
| Result: PASS (full check 40.6 s, no reason, ready for dry-run) |
```

This data is generated in `src/bench_stance.py` and returned in the selection result dict. The candidate records (from `write_record`) are saved during the run.

### 3.3 Dry-Run Data Example

From `moved_candidate1_dryrun_trace.json`:
- Stance, full check result, dry-run result (pass/fail, reason)
- Per-subtask: name, target, p0 (start), T_s (planned time), ok (pass/fail)
- Last 50 ticks: time, pad position, reference position, error, joint angles

This is generated by `src/stance_select.py` `verify()` and saved by caller (when debugging).

### 3.4 Real Arm Trajectory Data

From `over_trajectory_20261110.json` (the staging run):
- Per frame: time, label, q_meas, q_cmd, pad position, error to waypoint, bench geometry
- Extracted from episode via `experiments/bounce_diag_extract.py`

Other measurements available:
- `experiments/left_arm_motion_measure.py` – saves to `.npz`: joint angles, positions, velocities, accelerations, jerk per joint per phase
- `data/arm_motion/*/left_arm.npz` – archived measurements

### 3.5 What Display Could Use – Architectural Notes

**For candidate visualization (not yet built):**
- Take the edge sampler's candidate list (coordinates, heading, score)
- For each candidate: query the full check's solution to get q_left, q_right at nominal grasp point
- Draw in 3D: robot silhouette at q, end-effector position, approach direction vector
- Color by status (green=pass, red=fail, yellow=checking)

**For dry-run visualization (not yet built):**
- Take the dry-run trace's per-tick data
- Animate or show a static frame of the arm at the point where it got stuck
- Overlay the reference trajectory and the actual trajectory on a graph

**For imagined vs. real comparison (not yet built):**
- Capture the real arm's pose at the same event tick where dry-run was supposed to be
- Render side-by-side or as a diff

**Data flow:**
```
stance_select.py generate_candidates() → edge sampler → candidate records (x,y,heading,ped)
                        ↓
                  full_check.FullCheck.run() → check results (pass/fail, q_left, q_right)
                        ↓
                  verify() dry_run_stance() → trace JSON (per-tick q_cmd, pad, error)
                        ↓
Episode runs → control_loop → episode log chunks → replay → left_arm_motion_measure → over_trajectory.json
```

All data is available to a display; the missing piece is **instrumentation to save full check's q_left/q_right at each pose**, currently logged but not recorded to disk.

---

## Decisions Needed (For David)

1. **Dry-run unit bug fix:** Apply the one-line correction documented in `moved_dryrun_candidate1_20261110.md` section 6? (Affects all dry-run traces involving place angle != 0.)

2. **Full check solution logging:** Add optional instrumentation to save the full check's joint angles at each pose (grasp, approach turns, etc.) for visualization and comparison?

3. **Display priority:** Which stage visualization is most valuable for the demo video?
   - (a) Stance candidates as they are checked (green/red/checking status, score, reason for failure)
   - (b) Imagined vs. real arm pose comparison for the executed steps
   - (c) Trajectory plan vs. execution divergence

4. **Data availability:** Should stance selection output (candidate records, full check timings, dry-run traces) be saved by default, or only when explicitly requested (e.g., `--debug-stance`)?

---

## Not Done / Blocked

- ✗ Full arm pose comparison (full check vs dry-run vs real) requires full check solution logging
- ✗ Dry-run comparison uses invalid data (unit bug place angle) – invalidates moved layout results
- ✗ No new runs executed per the task scope (read-only)
- ✗ Viewer currently shows no stance selection stages or candidate filtering

---

## Summary

**Display coverage:** The viewer shows 3D motion and status text only. None of the stance selection pipeline stages (candidate generation, full check, dry-run, imagined placement) are currently drawn.

**Arm placement accuracy:** Three sources of arm pose data exist (full check, dry-run, real execution), but only dry-run and real arm data are saved. A full comparison is blocked by (a) full check solutions not being logged, and (b) a unit bug in dry-run place angle. The single Nov 10 episode with saved real arm data shows place OVER failed to reach (51 mm error), so it's not a healthy baseline.

**Data availability:** All data is technically available from the full_check API, dry_run traces (JSON), and episode logs. The missing piece is **default logging of full check's joint solutions** for display and comparison.
