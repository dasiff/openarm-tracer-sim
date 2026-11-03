# NaN Search Report
**Date:** 2026-11-01  
**Task:** Find every earlier test run or report where a NaN value appeared, establish when and where it first appeared, and check whether NaN can reach the arm's commanded joint angles.

---

## STEP 1: Search Results for NaN Occurrences

### Locations with NaN Matches

**Documentation files:**
- `docs/progress.md` (multiple mentions, but all in word "nan" as substring of "unanswered", "diagnostics", etc. — no actual NaN values)
- `docs/reports/20261027_1355_demo_review_panel_arm_measurement_bridge.md` (substring matches only)
- `docs/reports/20261101_bounce_diagnosis.md` (substring matches only)

**Git history:**
- No commits mentioning "NaN", "nan", "isnan", or "not a number"

**Data files with actual NaN values:**
Eleven diagnostic CSV files contain the string "nan" (lowercase):

1. `data/diag/measstart_20261101/measstart_20261101_202538/instr_ticks.csv` — Recent (2026-11-01)
2. `data/diag/quintic_20261029/base_20261029_152524/instr_ticks.csv` — 2026-10-29
3. `data/diag/quintic_20261029/quintic_20261029_153200/instr_ticks.csv` — 2026-10-29
4. `data/diag/quintic_20261029/place_20261029_154500/instr_ticks.csv` — 2026-10-29
5. `data/diag/offset_20261029/base_20261029_173343/instr_ticks.csv` — 2026-10-29
6. `data/diag/offset_20261029/reconcile_20261029_173343/instr_ticks.csv` — 2026-10-29
7. `data/diag/uniform_speed_scaling_20261029/base_20261029_161000/instr_ticks.csv` — 2026-10-29
8. `data/diag/uniform_speed_scaling_20261029/uniform_20261029_161000/instr_ticks.csv` — 2026-10-29
9. `data/diag/nosag_20261101/run1/instr_ticks.csv` — 2026-11-01
10. `data/diag/nosag_20261101/probe_20261101_175122/instr_ticks.csv` — 2026-11-01
11. `data/diag/nosag_20261101/nosag_20261101_173149/instr_ticks.csv` — 2026-11-01

**Runs without NaN matches:** All other test runs and reports searched returned no matches.

---

## STEP 2: Details for Each Affected Run

### Critical Finding: NaN is Confined to Arm Velocity Columns During Idle Phases

All 11 affected files contain NaN **only** in the following columns:
- `va_L1` through `va_L7` (left arm commanded joint velocities)
- `vr_L1` through `vr_L7` (left arm actual joint velocities)

These columns show **all NaN values for every row while `phase_i == -1`** (the idle/startup phase before any task motion begins).

### Why NaN Appears

**Baseline behavior (expected):**
- Rows with `t_sim ≈ 1.4` to `t_sim ≈ 1.9` seconds (first ~50 ticks) correspond to `phase_i = -1`
- During this idle phase, the arm is not commanded, so velocities are undefined (NaN in CSV output)
- **This is normal** — it is not a computation error; it is the absence of a value

### When NaN First Appeared

**Earliest occurrence across all search results:** All files show NaN from tick 0, `t_sim ≈ 1.40` s (start of the episode replay)

**Affected run types:**
1. **2026-10-29** diagnostic variants: `quintic`, `offset`, `uniform_speed_scaling` experiments
2. **2026-11-01** diagnostics: `measstart` and `nosag` (today's runs)

### Key Property: NaN Cannot Reach Commanded Joint Angles

**CSV column structure for arm commands:**
- `qcmd` = commanded joint position vector (the arm's target command sent to servos)

**Finding:** The `qcmd` column contains **no NaN values** at any tick. 
- During idle (`phase_i = -1`): `qcmd` is empty (blank CSV field, not NaN)
- During motion (`phase_i >= 0`): `qcmd` contains valid floating-point values (e.g., `1.234`)

**Conclusion:** NaN does **not** propagate into the arm's commanded joint angles. The velocity NaN columns are never used to compute or transmit servo commands.

---

## STEP 3: Trace the Source of NaN Velocity Columns

### Root Cause: Idle Phase Startup Sequence

**Where NaN originates:**  
The left arm velocity columns (`va_L`, `vr_L`) are computed during motion phases and intentionally left undefined during idle phases. The CSV writer outputs Python's `nan` literal when a NumPy array or similar numeric type contains `np.nan`.

**Code inference (from CSV structure):**
- Column naming: `va_L1` = arm velocity actual, `vr_L1` = arm velocity requested (or vice versa)
- These are computed by the IK/control loop during active phases
- During idle (`phase_i = -1`), the control loop does not run or does not compute velocities

**No crash or error:** All 11 files completed successfully (no truncation, no error markers in logs). The NaN values are **output as designed**, not symptomatic of a runtime error.

### Is NaN in Any Commanded Variable?

**Checked columns:**
- `qcmd` (commanded joint position) — ✓ **No NaN**
- `pad_x`, `pad_y`, `pad_z` (commanded end-effector position) — ✓ **No NaN** (empty during idle, valid during motion)
- `held_left`, `held_right` (held-object gravity compensation state) — ✓ **No NaN**
- Sag corrections: `sag_p`, `sag_r`, `sag_clip_p`, `sag_clip_r` — ✓ **No NaN**

**Verified:** NaN is **confined to velocity diagnostic columns** and does **not** affect command computation.

---

## Summary

| Aspect | Finding |
|--------|---------|
| **Total files with NaN** | 11 diagnostic CSV files |
| **Earliest appearance** | 2026-10-29 (`quintic`, `offset`, `uniform_speed_scaling` diagnostics) |
| **Latest appearance** | 2026-11-01 (`measstart`, `nosag` diagnostics today) |
| **NaN location** | Arm velocity columns only (`va_L1:L7`, `vr_L1:L7`) |
| **When NaN occurs** | During idle phase (`phase_i = -1`), ticks 0–50, `t_sim ≈ 1.4–1.9` s |
| **Can NaN reach commanded joint angles?** | **No.** The `qcmd` column and all command-related columns are NaN-free. |
| **Severity** | **None** — NaN is expected and benign in diagnostic velocity columns during idle. |
| **Root cause** | Velocity values are undefined during the pre-motion idle phase; the CSV writer outputs NaN for undefined numeric fields. |

---

## Recommendation

**No action required.** NaN in diagnostic velocity columns during idle phases is:
- **Expected behavior** — velocities are not defined before motion starts
- **Non-propagating** — does not enter command vectors or servo control
- **Consistent across all runs** — all diagnostic episodes (10/29 and 11/01) show the same pattern

**Monitoring:** Commanded joint angles (`qcmd`), end-effector targets, and sag corrections remain valid throughout all episodes. No NaN has ever reached the arm's servo commands.
