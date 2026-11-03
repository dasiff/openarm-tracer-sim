# Records update: the command bridge is on by default (2026-11-03)

Written for: David (repo owner). Documentation edits only. No code, config or script was changed. No simulation was run. Nothing was regenerated, and `experiments/baselines/` was not touched. Nothing was pushed. This report is left uncommitted.

## Summary

- **What changed.** The project records now say the command bridge (a rate limiter on the arm commands) is ON by default. They say this since commit `6d9b542` (2026-10-29), which you confirmed as intended on 2026-11-03.
- **Files edited, and in one commit:**
  - `docs/mpvi_decisions.md`: new decision **M241** added after M240. A short note added to the end of the old M193 row, pointing to M241. The original words of M193 are unchanged.
  - `docs/design_plan.md`: lines 37 and 43 changed from "OFF by default" to "ON by default since commit 6d9b542 (decision M241)".
  - `docs/progress.md`: one progress entry added at the end.
- **New decision number:** M241. The next free number was M241. M240 was the last row in the log, and no other file uses M241 or higher (checked with a search of `docs/`, `src/`, `experiments/`, `README.md` and `CLAUDE.md`).
- **Commit:** `e127479`, "Records: command bridge is on by default (M241; commit 6d9b542, confirmed by David 2026-11-03)". It contains only the three files above, and nothing else.
- **Not regenerated.** The manual baseline `experiments/baselines/manual_reference.csv` is now recorded as stale. It has NOT been regenerated, and no decision was made about regenerating it.
- **A problem found along the way (not fixed).** `experiments/command_bridge_test.py` has a test case "1. bridge OFF (the default)" that now runs with the bridge ON. I expect it to fail. I did not run it. See "NOT BUILT, needs decision".

## 1. Step 1: uncommitted changes before editing

Commands: `git status --short -- docs/mpvi_decisions.md docs/design_plan.md docs/progress.md` and `git diff` on the same three files.

- **Result: none of the three files had uncommitted changes.** Both commands printed nothing for these files.
- The main repo had 82 changed or untracked entries before the edits. These are your uncommitted logging hunks and marker removals, plus untracked reports and scripts. None of them is in the three files.

## 2. Step 2: the new decision entry (M241) and the note on M193

Source for every fact in M241, which is what the commit did:
- The commit: `git show 6d9b542`. It changes two lines: `src/task_config.py` (`COMMAND_BRIDGE_CFG["enabled"]`, from `False` to `True`, with the comment "Default on from David's request, 2026-10-29 [row M193]") and `src/control_loop.py` (a comment, "off by default" to "on by default").
- The date, the setting name and the file come from that commit and from `src/task_config.py`.
- "It is the only commit since 5b7349e that changes the manual scenario output" comes from `docs/reports/baseline_drift_part2_20261103.md` (step 3 walk, 40 runs).
- "David confirmed on 2026-11-03" comes from your message in this session.

New row added after M240 (exact text):

```
| M241 | 2026-11-03 | The command bridge is ON by default. Commit 6d9b542 (2026-10-29, "Command bridge on by default (M193)") changed `COMMAND_BRIDGE_CFG["enabled"]` in `src/task_config.py` from `False` to `True`; the comment in that file gives the same date and says it is from David's request. David confirmed on 2026-11-03 that this change was intended. It is the only commit since 5b7349e (2026-10-26, the gravity compensation switch, M185) that changes the output of the manual bench scenario (`experiments/cc_bench_scenarios.sh manual`). The output of HEAD 1056ab6 is byte-identical to the output after 6d9b542 (`docs/reports/baseline_drift_part2_20261103.md`). Consequence: the manual baseline `experiments/baselines/manual_reference.csv` (from Oct 22) is stale, because it predates M185 and this change. The baseline has NOT been regenerated, and no decision has been made about regenerating it. The M193 row above describes the earlier default (OFF) and is kept unchanged as history. | decides | ON (`enabled` True) for every run; `--no-bridge` in `experiments/demo_video.py` still turns it off | David (confirmed 2026-11-03) | `src/task_config.py` COMMAND_BRIDGE_CFG; commit 6d9b542 |
```

Note: the "`--no-bridge` in `experiments/demo_video.py` still turns it off" part comes from `experiments/demo_video.py` line 158 (help text: "ON by default here ... --no-bridge") and line 85 (docstring). I did not run the demo.

Note added to the end of the M193 row. The change is an insertion before the final cell's closing bar. The original words are unchanged.

Before (the last cell of M193, end of line):
```
... | addition (design plan section 1) | David 2026-10-18 / 2026-10-20 (limits and OFF default); build is ours |
```
After:
```
... | addition (design plan section 1) | David 2026-10-18 / 2026-10-20 (limits and OFF default); build is ours. Note 2026-11-03: superseded by M241 (ON by default since commit 6d9b542; the rest of this row is unchanged). |
```

## 3. Step 3: design plan lines 37 and 43

Only the wrong words about the bridge were changed. Everything else in both lines is the same.

Line 37, before (the part changed):
```
... row M193: `COMMAND_BRIDGE_CFG` in `src/task_config.py`, loop setting `cfg["command_bridge"]`, OFF by default, ON in the demo video script; `experiments/command_bridge_test.py`]** [DECIDED 2026-10-18]
```
Line 37, after (the part changed):
```
... row M193: `COMMAND_BRIDGE_CFG` in `src/task_config.py`, loop setting `cfg["command_bridge"]`, ON by default since commit 6d9b542 (decision M241; before that OFF by default, ON in the demo video script); `experiments/command_bridge_test.py`]** [DECIDED 2026-10-18]
```

Line 43, before (the part changed):
```
... The bridge is a loop setting (`COMMAND_BRIDGE_CFG`), OFF by default (the keyboard, teleop and staging runs and the baselines are unchanged) and ON for D3 and for anything that later records training data or runs a policy. ...
```
Line 43, after (the part changed):
```
... The bridge is a loop setting (`COMMAND_BRIDGE_CFG`), ON by default since commit 6d9b542 (decision M241; under M193 it was OFF by default, when the keyboard, teleop and staging runs and the baselines were unchanged) and ON for D3 and for anything that later records training data or runs a policy. ...
```

Note: the "ON for D3 and for anything that later records training data or runs a policy" part was left as it was. It is still true, and it was not part of the wrong statement.

## 4. Step 4: progress entry

Two entries added at the end of `docs/progress.md`, in the same format as the others (a `TASK` line, then a `DONE` line):

```
TASK 2026-11-03 (records update, command bridge default; documentation only, no code, no config, no runs): the bridge default was changed in commit 6d9b542 (2026-10-29); David confirmed on 2026-11-03 that it was intended. Source: docs/reports/baseline_drift_part2_20261103.md.
DONE 2026-11-03 (records update): decision M241 added (ON by default since 6d9b542; the manual baseline is stale, NOT regenerated). M193 row given a note pointing to M241 (its text unchanged). docs/design_plan.md lines 37 and 43 changed from OFF to ON by default. Report docs/reports/records_update_20261103.md. Commit: the one that adds this entry (docs/mpvi_decisions.md, docs/design_plan.md, docs/progress.md). NOT PUSHED.
```

The progress entry does not have a clock time. The entries before it have one, and I did not invent one.

## 5. Step 5: other places that say the bridge is off

I searched the whole repo for "off by default", "OFF", "off everywhere", "except the demo video", "no-bridge", and bridge settings written as `False`, including comments and reports. I edited none of them.

### Standing statements that are now wrong or need a decision

1. **`experiments/command_bridge_test.py`, lines 8 and 66.** The test case is called "1. bridge OFF (the default)". Its config does not set the bridge, so the loop uses the default (`COMMAND_BRIDGE_CFG["enabled"]`, now `True`). By reading the code, the expected result ("applied target jumped by 0.6 rad at once, bridge log empty") will not hold with the bridge on. I did not run the test. This needs a decision, and it is a code/test change, so it is not part of this task. See "NOT BUILT".

### Dated reports and progress lines (history, left as written)

These were true when written, and are records of the day. Each is listed with its line.
- `docs/progress.md` lines 26, 36, 42, 44, 46, 50, 52, 118, 134, 136, 137, 138, 139, 140: "bridge OFF" in suite runs, and "off by default" when the bridge was built (Oct 27 and 28).
- `docs/reports/20261027_1403_session_start_status.md` line 9: "Command bridge built (M193, off by default; demo_video default ON)".
- `docs/reports/20261027_1355_demo_review_panel_arm_measurement_bridge.md` lines 66 and 107: the bridge block and "Full test suite with the bridge OFF (its default)".
- `docs/reports/20261027_2310_part_a_b_report.md` lines 8, 11 and 39: "bridge OFF except where a test turns it on", "OFF by default", "No suite-wide switch for the bridge exists".
- `docs/reports/20261028_0330_phase_start_ramp.md` line 47: "bridge OFF (default)".
- `docs/reports/20261028_0346_min_jerk_skill_start_ramp.md` line 40: "(bridge OFF, both ramps ON)".
- `docs/reports/20261020_1323_d3_design_proposal.md` line 147: the question "off by default and on for D3 (my proposal), or on everywhere now (new baselines)?". This was the question at the time, not a statement.
- `docs/reports/baseline_drift_part2_20261103.md` (my report, still uncommitted, written before this update): its sections 4 and "NOT BUILT" describe the decision log and the design plan as still saying OFF. That was true when written. I did not edit it.
- `docs/mpvi_decisions.md` row M193 itself: kept unchanged as history, with the note above.

### Places that already say ON, and are consistent

- `src/task_config.py` line 462: "Default on from David's request, 2026-10-29 [row M193]". Consistent with M241. (It says the request is David's. M241 quotes that comment, and the request itself is not in any other record.)
- `src/control_loop.py` line 35 (the module docstring): "on by default". Consistent.
- `src/control_loop.py` line 112 is NOT consistent. It is the default of the loop setting `"command_bridge"`, and its comment says "None: no bridge". The code at line 583 (`bcfg = {**COMMAND_BRIDGE_CFG, **(cfg.get("command_bridge") or {})}`) merges `COMMAND_BRIDGE_CFG` in when the value is `None`. With the default now `enabled: True`, `None` means the bridge is ON. So this comment is stale. It is a code comment, so I did not edit it. It is listed under "NOT BUILT, needs decision".
- `experiments/demo_video.py` lines 85 and 158: "ON unless --no-bridge" and "ON by default here, as for D3". Consistent.
- `README.md`: no mention of the command bridge. Nothing to change (no statement about it either way).

## Commands run

- `git status --short -- <three files>`, `git diff -- <three files>` (step 1; empty)
- `grep -n` on `docs/mpvi_decisions.md`, `docs/design_plan.md`, `docs/progress.md` for decision numbers and the M193 row (read only)
- `grep -rn -i -I` over the repo, excluding `.git`, `deps`, `data` and `.venv` (step 5; read only)
- `Read` on the exact lines before each `Edit`
- `Edit` on the three files (5 edits; each one matched a unique string)
- `git diff --stat` and `git diff` on the three files (review before commit)
- `git add -- docs/mpvi_decisions.md docs/design_plan.md docs/progress.md` and `git commit` (one commit, `e127479`)
- `git show --stat HEAD`, `git status --porcelain | wc -l` and `git diff --stat` (after commit)

## Check after the commit (step 7)

- The commit `e127479` lists exactly three files: `docs/design_plan.md` (4 lines changed), `docs/mpvi_decisions.md` (3 lines changed) and `docs/progress.md` (2 lines added). Nothing else is in it.
- The main repo's status count is 82 before and after the commit. The count includes your uncommitted hunks and the untracked reports. The commit did not change any of those entries, because the three files had no changes of yours before the commit.
- `git diff --stat` for the whole repo is the same as before this task: 66 files changed, 50 insertions, 69 deletions. Your uncommitted hunks are unchanged.

## NOT BUILT, needs decision

1. **`experiments/command_bridge_test.py` case 1** ("bridge OFF (the default)"). With the default now ON, I expect this case to fail, and it may make the test suite fail too. I did not run it. Decide whether the test should set `enabled` to False explicitly (a code change), or whether its expectation should change. Not done.
2. **Regenerating `experiments/baselines/manual_reference.csv`.** Not done. The file is recorded as stale in M241. Decide whether to regenerate it, and if so to what state.
3. **Teleop and policy baselines.** Not checked, and not mentioned in M241. They may also be affected by the bridge default; I did not test that.
4. **Dated reports and progress lines that say "OFF"** (the list in section 5). Left as history. Decide whether you want a note added to each one, or nothing.
5. **My earlier report `docs/reports/baseline_drift_part2_20261103.md`** still says the records are not yet updated (section 4 and "NOT BUILT" item 1). It is uncommitted and was accurate when written. Decide whether it should be updated.
6. **`src/control_loop.py` line 112 comment** ("None: no bridge"). Stale: with the default ON, `None` means the bridge is ON (code at line 583). A code comment, so not edited. Decide whether to fix the comment.
7. **Demo video help and docstring** (`experiments/demo_video.py` lines 85 and 158). Consistent now, so nothing done.
