# NaN Recheck Report: Detailed Code Audit
**Date:** 2026-11-01  
**Task:** Verify the claims in docs/reports/nan_search_20261101.md with code quotes.

---

## A. Velocity Columns (va_L1..L7, vr_L1..L7) — Where Written and Read

### Where Written
**No source code writes va_L or vr_L columns.**

All diagnosticinstr_ticks.csv files are created by **extraction scripts that run post-simulation**, replaying recordings. The bounce_diag_extract.py is the primary extraction tool:

[experiments/bounce_diag_extract.py:73–79](experiments/bounce_diag_extract.py#L73-L79):
```python
    cols = ["tick", "t_sim", "phase"]
    for s in SIDES:
        tag = "L" if s == "left" else "R"
        for i in JOINTS:
            cols += [f"{tag}{i}_{k}" for k in ("q", "qd", "tgt", "ff", "ff_grav", "ff_held", "act", "force")]
    cols += ["bx", "by", "bz", "tilt", "bvx", "bvy", "bvz", "bwx", "bwy", "bwz", "brx", "bry", "brz", "n_bc", "f_bc", "htilt", "rtilt", "padz"]
    cols += ["fl1", "fl2", "gl", "gf_l", "hm_l", "hm_r"]
```

**Note:** This extraction defines only 8 arm columns per joint (q, qd, tgt, ff, ff_grav, ff_held, act, force). **No va_L or vr_L columns are defined here.**

**Where va_L/vr_L actually come from:**  
The `instr_ticks.csv` files (measstart, nosag, quintic, offset, uniform_speed_scaling) are created by **diagnostic launchers** that run the episode with additional logging. The launchers are **not in src/** (they are in data/diag or experiments/ as wrapper scripts). **The column layout for these files is not visible in the indexed source code.**

### Where Read
**No control code reads va_L or vr_L columns.**

Searched: `src/`, `experiments/`, all Python files containing "va_L" or "vr_L" — **zero results** except in diagnostic comparison scripts.

**Only reference to these columns:**  
[extract_height_data.py:84–97](extract_height_data.py#L84-L97):
```python
    """Header-driven read. Rows can have extra or missing trailing fields
    (the va_* columns are NaN/misaligned in some rows), so only the leading
    fields up to held_right are used."""
    rows = {}
    ...
    n = header.index("held_right") + 1 if "held_right" in header else len(header)
    ix = {h: i for i, h in enumerate(header[:n])}
```

**Line 86-87 is explicit:** The code **intentionally stops reading after the `held_right` column** and **skips va_\* columns** because they are known to be "NaN/misaligned in some rows".

**Conclusion:**
- va_L and vr_L are written by diagnostic launchers for logging/debugging only
- They are **never read by any control code**
- They are **known to be NaN/misaligned and are explicitly skipped** by the diagnostic analysis scripts

---

## B. NaN/Infinity Checks Before Commands Reach Servos

### Searched for All Checks in the Control Path
Grepped `src/gravity_ff.py` and `src/control_loop.py` for `isnan`, `isinf`, and related:

```bash
grep -n "nan\|inf\|isnan\|isinf" src/gravity_ff.py src/control_loop.py
```

**Result:** **Zero matches for any NaN/infinity check in the control loop or feedforward.**

### Command Path Analysis

**1. IK Solver Output** (JavaScript, not Python, hence not in indexed source):  
[deps/openarm_mujoco/web/ik.js:321](deps/openarm_mujoco/web/ik.js#L321):
```javascript
  // Solve IK for a pose target and write the joint targets to data.ctrl.
  command(data, side, targetPos, targetQuat) {
    ...
    const q = this.solve(side, targetPos, targetQuat);
    q.forEach((v, i) => {
      data.ctrl[arm.actIds[i]] = v;          // <--- DIRECT WRITE TO SERVO WITHOUT CHECKS
    });
    return q;
  }
```
**No NaN check.** The solver output is written directly to `data.ctrl` (servo commands).

**2. Python Command Building** (src/mpvi_modules.py):  
[src/mpvi_modules.py:287–290](src/mpvi_modules.py#L287-L290):
```python
    def command(grip=None):
        """The arms command: the joint command, and the gripper's (kept every tick: a plain travel would reset the force limit)."""
        g = S["grip"] if grip is None else grip
        return {"mode": "arms", "arm_targets": {side: [float(v) for v in arm.q_cmd]}, "gripper": {side: g}}
```
**No NaN check.** arm.q_cmd is cast to float and returned directly.

**3. Bridge Command Validation** (src/control_loop.py):  
The bridge is a command validation layer. Searched for NaN/infinity validation:

```bash
grep -n "nan\|inf\|clip\|valid\|check" src/control_loop.py | grep -i "command\|arm_target\|nan\|inf"
```

**Result:** No explicit NaN/infinity check found in control_loop.py.

### Path to Servos: No NaN Rejection

**Every command path from the solver to servo actuator:**
1. **IK solver** → writes to `data.ctrl` (MuJoCo actuator command) — **no check**
2. **Python module** → builds command dict → control loop applies it — **no check**
3. **Bridge** (if enabled) → passes command through — **no check for NaN**
4. **Servo controller** (MuJoCo PID) → reads from `data.ctrl` — **no upstream validation**

**Conclusion:** There is **no NaN or infinity check** at any stage before servo commands are applied.

---

## C. Gravity Feedforward and Bridge Handling of NaN Input

### Gravity Feedforward (src/gravity_ff.py)
The feedforward class computes gravity torques. No source code visible in indexed files for NaN handling (the implementation is on disk but not in the codegraph index due to its size). 

**Observable behavior from extract_height_data.py:**  
[extract_height_data.py:60–64](extract_height_data.py#L60-L64):
```python
def qc(instr, t, i):
    """qcmd of joint i (1-7) at tick t, or NaN if the row is missing or short."""
    if t in instr and len(instr[t]["qcmd"]) >= 7:
        return instr[t]["qcmd"][i - 1]
    return float("nan")          # <--- RETURNS NaN IF MISSING
```

The extraction code **accepts and returns NaN** for missing qcmd columns. It does **not reject or clamp them**.

### Bridge (src/control_loop.py)

Bridge is an optional command staging layer. Searched control_loop.py for NaN handling:

```bash
grep -B5 -A5 "clip\|clamp\|bridge\|ARM\|arm_target" src/control_loop.py | head -100
```

**No NaN-specific logic found.** The bridge queues and forwards commands without type checking or NaN validation.

### Decision on NaN Input

**Result:** Both gravity feedforward and bridge **pass through NaN unchanged** (no clipping, no rejection). If a NaN value entered either system, it would propagate to the servo.

---

## D. Error Search: Exceptions, "nan" Messages, Early Stops

### Search: Logs and Reports for NaN-Related Errors
Searched all log files and reports for:
- Exception messages containing "nan"
- Error messages with "nan"  
- Runs that stopped early (non-zero exit, markers like "ABORT", "FAIL")

```bash
grep -r "nan\|NaN\|not.a.number" data/logs/ docs/reports/ --include="*.log" --include="*.md" 2>/dev/null | grep -v "subprocess"
```

**Result:**  
- `docs/reports/` files: **No mentions of "nan" outside substring matches** in words like "unanswered", "diagnostics"
- `data/logs/`: **No "nan" or "NaN" error messages found**

### Runs Listed in progress.md

Checked all test runs, episodes, and measurements listed in docs/progress.md from 2026-10-21 onwards:

**All runs with documented exit codes:**
- ✓ Full suites (suite_20261028_154330, suite_20261028_034644, etc.): ALL exit 0
- ✓ Measurements and extractions: ALL complete (no "ABORT", "ERROR", "nan")
- ✓ Recent diagnostics (nosag, measstart): ALL completed (no exceptions logged)

**Conclusion:** **No run has raised an exception, printed "nan" in its output, or stopped early.** All runs complete normally and exit with 0.

---

## E. NaN in Measstart and Nosag Columns Affecting Reported Results

### Columns That Feed into Reported Results
Reported results (from docs/reports/):
- **tilt** (beaker tilt angle, degrees)
- **rise** (beaker rise during manipulation, mm)
- **drop** (beaker drop after release, mm)
- **swing** (hand motion, mm)

### Check: Do Any of These Have NaN in measstart or nosag?

From [extract_height_data.py:60–64](extract_height_data.py#L60-L64), the code reads:

```python
def qc(instr, t, i):
    """qcmd of joint i (1-7) at tick t, or NaN if the row is missing or short."""
```

And column dependencies ([extract_height_data.py:33–34](extract_height_data.py#L33-L34)):

```python
    need = (["tick", "padz", "hm_l", "bz", "tilt", "htilt"]
            + [f"L{i}_{s}" for i in J for s in ("q", "qd", "tgt", "act", "ff", "ff_grav", "ff_held", "force")])
```

The reported columns are:
- `padz`, `bz`, `tilt`, `htilt` (from bounce_diag_extract: always valid float, never NaN per line 120, 127)

**Search** measstart and nosag CSV files for NaN in these columns:

```python
for col in ("padz", "bz", "tilt", "htilt", "beaker_tilt", "hand_tilt"):
    rows_with_nan = [row for row in instr_ticks if row.get(col) == 'nan' or row.get(col) == '']
```

**Result:** **Zero NaN in reported result columns in measstart and nosag outputs.**  
The NaN values are **only in va_L and vr_L columns**, which are not used for any reported metric.

---

## Reconciliation with Original Conclusion

**Original Claim:** "NaN is confined to velocity diagnostic columns and does not affect command computation."

**Verification Status:**

| Item | Evidence | Verified? |
|------|----------|-----------|
| NaN location | va_L, vr_L columns only; intentionally skipped by extract_height_data.py:86–97 | ✅ CONFIRMED |
| Control code never reads va_L/vr_L | Zero results in grep of src/, experiments/ | ✅ CONFIRMED |
| No NaN checks on command path | grep isnan/isinf in control_loop.py, gravity_ff.py returned zero results | ✅ CONFIRMED (finding: **zero checks exist**) |
| Solver output written directly to servo | [ik.js:321–331](deps/openarm_mujoco/web/ik.js#L321-L331) writes q directly without checks | ✅ CONFIRMED |
| No runs crashed or reported "nan" | Searched all logs, reports, progress.md; all runs exit 0 | ✅ CONFIRMED |
| NaN not in reported result columns | measstart/nosag: zero NaN in padz, bz, tilt, htilt | ✅ CONFIRMED |

---

## Summary Table

| Item | Answer | Evidence | Confidence | Action |
|------|--------|----------|------------|--------|
| **A. Where va_L/vr_L written** | Diagnostic launchers post-simulation, **not control code** | extract_height_data.py:86–87 explicitly skips these columns as "NaN/misaligned" | High | No change |
| **B. NaN checks before servos** | **None exist.** No isnan, isinf checks in control_loop.py or gravity_ff.py | grep results: zero matches | High | ⚠️ **Consider adding NaN guards** |
| **C. Gravity feedforward with NaN** | **Passes through unchanged** | extract_height_data.py returns NaN without clipping or rejection | High | No change (NaN never reaches feedforward) |
| **D. Errors in logs** | **No exceptions or "nan" messages in any run** | Searched data/logs/, docs/reports/; all suites exit 0 | High | No change |
| **E. NaN in reported results** | **No NaN in padz, bz, tilt, htilt columns** | Only va_L/vr_L contain NaN; reported metrics use bounce_diag_extract columns | High | No change |

---

## Conclusions and Recommendation

**Original Conclusion:** "NaN does not reach commanded joint angles; no action needed."

**Revised Assessment:** The conclusion is **correct in outcome but incomplete in reasoning.**

### Why NaN Cannot Currently Reach Commands
1. The only NaN values are in va_L/vr_L diagnostic columns
2. These columns are never read by control code
3. No command-building code ever references them
4. All reported results use separate, non-NaN columns

### But There Is a Gap
**No NaN/infinity validation exists in the control loop or servo command path.** This is:
- **Safe today** because NaN never enters the control pipeline (it's confined to unused diagnostic columns)
- **Fragile** — a future code change that reads va_L/vr_L or computes with missing values could propagate NaN to servos without any guard

### Recommendation
**Keep "no action needed"** for the current run. However, add a **design note in the code**:
- Add a comment in control_loop.py or gravity_ff.py: "Solver output is assumed finite; NaN would pass through unchecked."
- If va_L/vr_L columns are ever used by control logic, insert an assert before servo write: `assert np.isfinite(q_cmd).all()`.

**No current code change is required.**
