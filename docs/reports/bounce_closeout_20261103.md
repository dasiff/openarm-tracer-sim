# Bounce close-out (2026-11-03)

Written for: David, to review the close-out of the bounce work. Plain language. Documentation only: no code, config, setting or number was changed, and no simulation was run.

## Summary

- **Decision number: M244.** A new row at the bottom of `docs/mpvi_decisions.md`, with a detail section after the table. It records David's stopping rule, the before and after numbers, KNOWN LIMITS, the tests, the guess about the lag, the untried options and the open decisions.
- **Commits (local, not pushed; the branch has no upstream):**
  - `75cea83`: the six reports and the two test-switch patches, with a README (commit 1).
  - `5c42375`: the three docs files, `docs/mpvi_decisions.md`, `docs/design_plan.md` and `docs/progress.md` (commit 2).
- **Saved:** `docs/reports/20261103_1640_bounce_check.md`, `docs/reports/bounce_analysis_20261103.md`, `docs/reports/bounce_tests_20261103.md`, `docs/reports/bounce_tests2_20261103.md`, `docs/reports/save_control_loop_20261103.md`, `docs/reports/save_heldback_20261103.md`; `experiments/test_patches/test_switches.patch`, `experiments/test_patches/test_switches2.patch`, `experiments/test_patches/README.txt`.
- **Still unsaved:** this report (`docs/reports/bounce_closeout_20261103.md`, to review first); `measstart`; `.claude/`; `compare_nosag.py`; `extract_height_data.py`; `deps/AgileX_Tracer` (none of these was decided, and none was changed).
- **Step 1:** the main tree had no tracked changes before the work started (`git diff --stat` empty).
- **Design plan:** one status note in the held-weight section (built, with known limits). No plan item was marked DECIDED or DONE; the existing markers are unchanged.
- **Progress log:** one entry at the bottom of `docs/progress.md`.
- **Numbers:** taken from the named reports. Two places where the reports round differently from the brief: the anchored swing is 4.61 to 4.65 mm in `bounce_analysis_20261103.md` (the brief says about 4.6); the Nov 1 swing is 36.14 mm in the report table (the brief says 36.1). The report's own values are used.

## Exact text added to each docs file

### `docs/mpvi_decisions.md`, new row (bottom of the table)

```
| M244 | 2026-11-03 | Bounce close-out (David, 2026-11-03, stopping rule). Run the last test batch, then close the bounce investigation with no further diagnostic rounds. Whatever still misses the 3 mm line is recorded as KNOWN LIMITS. Status: built, with known limits. The held-weight design (M215 to M240) stays as built; nothing in code or settings was changed by this entry. The before and after table, the tests, the open decisions and the untried options are in the section below this table (M244 detail). Patches of the test switches are saved, not applied, in experiments/test_patches/. | logs | paper silent | n/a | none (documentation only) | David (stopping rule, 2026-11-03) | docs/reports/20261103_1640_bounce_check.md; docs/reports/bounce_analysis_20261103.md; docs/reports/bounce_tests_20261103.md; docs/reports/bounce_tests2_20261103.md |
```

### `docs/mpvi_decisions.md`, detail section (after the table)

```
## M244 detail: bounce close-out (2026-11-03)

**Stopping rule (David, 2026-11-03).** Run the last test batch, then close out the bounce work with no further diagnostic rounds. Whatever still misses the 3 mm line is recorded here as KNOWN LIMITS. The next task is the distance experiment.

**What is built.** The held-weight design on branch held-weight-ramp-sag-off, as the rows M215 to M240 record: the lift-off weighing (M235 to M237); the 1.0 s minimum-jerk held-mass ramp (M218); the sag correction, which does not integrate during a held-mass ramp (M226, M238); and the gripper open waiting for the ramp (M221, M239 part a). Status: built, with known limits. Not built: the open-command gate (M239 part b).

**Before and after** (numbers as written in the named reports; the reports are the source):

| Measure | Nov 1 (old design) | Nov 3 (held-weight design) |
|---|---|---|
| Lift rise (mm) | 17.70 | +4.1 (150 g), +4.0 (200 g), +1.9 (250 g) |
| Lift drop (mm, against the start) | -18.45 | -2.1 (150 g), -2.1 (200 g), -2.2 (250 g) |
| Swing, Nov 1 definition (200 ticks from the held-weight start, mm) | 36.14 | Not comparable: the window closes before lift-off (0.07 to 0.09 mm). Anchored at the start of place OVER (an anchor chosen by Claude Code, not verified): 4.61 to 4.65 mm |
| Place DOWN swing, hand against plan, DOWN start to 3 s later (mm) | not measured this way | 9.0 (150 g), 9.4 (200 g), 9.1 (250 g) |
| Full pick and place completed | yes | yes, at all three masses |
| Weight estimate over-read (g) | not reported | +25.4 (150 g), +24.5 (200 g), +0.4 (250 g) |

Sources: docs/reports/20261103_1640_bounce_check.md (rise, drop, swing, over-read) and docs/reports/bounce_analysis_20261103.md (section 1c, anchored swing; section 4a, over-read).

**Tests** (200 g unless stated). The control run matched the existing run on all 58 shared columns, in all 14,381 rows, with zero differing cells (both test rounds).

| Test | Result |
|---|---|
| True mass given to the arm (simulator truth; test only) | Lift rise 2.81 mm (200 g) and 2.89 mm (150 g), both under 3 mm. DOWN swing 9.32 and 8.98 mm (unchanged). The over-read explained about 1.2 mm of the rise, about half the predicted 2.2 to 2.3 mm. Source: bounce_tests_20261103.md |
| Trim frozen in all directions during DOWN | Swing 6.41 mm, but the step failed: hand 3.45 mm from the waypoint; retry 5.6 mm. Source: bounce_tests_20261103.md |
| Trim frozen in z only during DOWN | Undershoot -4.40 to -1.28 mm; lag +4.99 to +6.31 mm; swing (DOWN plus 3 s) 9.39 to 7.59 mm; step completed, final distance 0.20 mm. Source: bounce_tests2_20261103.md |
| Place DOWN at half speed (planned time 1.0 s to 2.0 s) | Lag +4.99 to +2.80 mm; undershoot -4.40 to -1.15 mm; swing (DOWN plus 3 s) 9.39 to 4.32 mm; step completed, final distance 0.49 mm (DOWN label 2.2 s; timeout 12 s). Source: bounce_tests2_20261103.md |

**What the tests showed, in plain words.**
- A perfect weight fixes about half of the lift rise (the predicted drop was 2.2 to 2.3 mm, the measured drop about 1.2 mm). It does not change the DOWN swing.
- Freezing the whole trim removes the undershoot, but it also removes the sideways (x) correction that brings the hand to the waypoint, so the step fails.
- Freezing only the vertical trim removes most of the undershoot (71 %), the step still completes, and the lag grows.
- Half speed roughly halves the lag (the ratio is 0.56 against a speed ratio of 0.50, calculated) and cuts the undershoot further. The step completes.

**Guess (not a finding; consistent with both tests).** The lag is the root of the DOWN swing: the hand falls behind the planned descent, and the vertical trim winds up against that lag and overshoots below plan.

**KNOWN LIMITS (recorded, not fixed).**
1. Lift rise above the 3 mm line at 150 g (+4.1 mm) and 200 g (+4.0 mm). At 250 g it is +1.9 mm, under the line.
2. Place DOWN swing above the 3 mm line at all three masses (9.0, 9.4 and 9.1 mm) with the design as built.
3. Lift drop is under the line at all three masses (2.1 to 2.2 mm).

**Not tried.** Combining the z-only freeze with half speed; other speeds; repeat runs; 250 g in the two test rounds; the bridge target in the last batch.

**OPEN DECISIONS for David (listed, not decided).**
- (a) Whether to adopt a slower descent while holding a beaker. Half speed gave a DOWN swing of 4.3 mm (one run, 200 g).
- (b) What to do about the weight over-read. A perfect weight fixed only about half the rise.
- (c) Whether the 3 mm line applies to the DOWN swing. M206 says "each under 3 mm" for rise, drop and swing; the repo records no reason for the number.
```

### `docs/mpvi_decisions.md`, UPDATE notes on older rows (text of each row kept; note appended to the decision cell)

- **M206:** `UPDATE 2026-11-03 (see M244): the 3 mm pass line was run on the held-weight design at 150, 200 and 250 g as scripted lift-off runs, not as a VLA handoff test. Result: lift rise +4.1 / +4.0 / +1.9 mm (rise fails at 150 and 200 g); place DOWN swing 9.0 / 9.4 / 9.1 mm (fails at all three masses). Not met yet.`
- **M218:** `UPDATE 2026-11-03 (see M244): the 1.0 s ramp was used in all the Nov 3 runs and was not changed. The true-mass test changes the ramp's end value, not its duration.`
- **M221:** `UPDATE 2026-11-03 (see M244): timing superseded by M239: the ramp-out starts at the OPEN step, not at the end of DOWN.`
- **M235:** `UPDATE 2026-11-03 (see M244): run at 150, 200 and 250 g; all three completed the full pick and place. Corrected lift-off estimate: +16.98 %, +12.27 % and +0.17 % of the true mass (docs/reports/20261103_1640_bounce_check.md, table).`
- **M237:** `UPDATE 2026-11-03 (see M244): in the three Nov 3 runs lift-off came before the 1.0 kg cap, so the cap did not act.`
- **M239:** `UPDATE 2026-11-03 (see M244): part b (the open-command gate) is still not built; the ramp-out runs from OPEN, as built.`
- **M240:** `UPDATE 2026-11-03 (see M244): limit (2) was measured with the Nov 3 over-read of +25.4 / +24.5 / +0.4 g (150 / 200 / 250 g). A perfect weight fixed about half the predicted rise (M244).`

### `docs/design_plan.md`, one note under the held-weight section heading

```
**Status 2026-11-03 (M244, bounce close-out): built, with known limits.** Lift rise above the 3 mm line at 150 g (+4.1 mm) and 200 g (+4.0 mm); place DOWN swing above it at all three masses (9.0, 9.4 and 9.1 mm). Stopped by David's rule (no further diagnostic rounds). The open decisions are listed in M244 and are not decided. The item markers below are unchanged.
```

### `docs/progress.md`, one entry at the bottom

```
TASK 2026-11-03 (bounce close-out; documentation only, no code, no simulation, no push): David's stopping rule applied (run the last test batch, then close the bounce work; what still misses the 3 mm line is KNOWN LIMITS). Decision M244 added to docs/mpvi_decisions.md (row, a detail section after the table, and UPDATE notes on M206, M218, M221, M235, M237, M239, M240). docs/design_plan.md: one status note in the held-weight section (built, with known limits); item markers unchanged. Commit 75cea83 saved the six reports (20261103_1640_bounce_check, bounce_analysis_20261103, bounce_tests_20261103, bounce_tests2_20261103, save_control_loop_20261103, save_heldback_20261103) and experiments/test_patches/ (two test-switch patches, never applied, with a README). Secret search on those files: no keys, tokens, passwords or e-mail addresses; the only hits are in save_heldback_20261103.md, which describes the earlier search. No file over 1 MB. Left unsaved: docs/reports/bounce_closeout_20261103.md (this session's report, to review first), measstart, .claude/, compare_nosag.py, extract_height_data.py, deps/AgileX_Tracer (not decided). Open for David, not decided: (a) a slower descent while holding a beaker; (b) what to do about the weight over-read; (c) whether the 3 mm line applies to the DOWN swing. NOT PUSHED.
```

## Secret search

Run on the files committed in `75cea83` (the six reports, the README and the two patches) and on the lines added to the three docs files in `5c42375`.
- **Keywords** (`api key`, `secret`, `password`, `passwd`, `token`, `bearer`, `private key`, `key=`, Google `AIza`, `sk-`, AWS `AKIA`, GitHub `ghp_`, Slack `xox`): hits only in `docs/reports/save_heldback_20261103.md`, which describes the earlier search, and in the progress entry, where the words describe this search ("no keys, tokens or passwords"). No secret value.
- **E-mail addresses:** none in the committed files or the added lines.
- **Long random strings** (24 or more characters, mixed case and digits): none. The only long strings are file paths and identifiers.
- **Size:** every committed file is under 1 MB (largest: `docs/reports/bounce_analysis_20261103.md`, 28,885 bytes).
- **Left out:** nothing; no hit and no oversize file.

## Step 8: where the 3 mm line comes from

**Answer: the repo records the line, but not the reason for it.**
- `docs/mpvi_decisions.md`, M206 (2026-11-01, decided by David, DECIDED, not built, decides LATER): "Handoff test: scripted stand-in for the VLA hands off to the controller at different moments, 150/200/250 g. Metrics: rise, drop, swing. Pass: each under 3 mm." No reason is given for the 3 mm.
- `docs/design_plan.md`, held-weight section, item f (M206): "Pass criterion: each under 3 mm." Same wording; no reason.
- `src/task_config.py`, PICK_PLACE_CFG `tol_m` (0.003 m): a different quantity, the waypoint tolerance. Its comment says "[row M112 PENDING David: no decision recorded]". Not the same line.
- `src/task_config.py`, the drop monitor margin (`drop_margin_m` 0.003 m, row M044): a different quantity; its comment gives a reason ("3 mm is well above the 1 mm noise of a held grasp"). Not the same line.
- `CLAUDE.md`: no mention of the 3 mm line.
- Reports: `docs/reports/20261103_1640_bounce_check.md` says "Pass line (David): rise, drop and swing each under 3 mm", and gives no reason either.
- **Conclusion:** the decision is David's (M206). The repo has no record of why 3 mm was chosen. The open question (c) in M244 asks whether the line applies to the DOWN swing.

## Commands run

From the main repo `/home/vboxuser/projects/openarm-tracer-sim`:
- `git status --short`; `git diff --stat`; `git branch --show-current`; `git log -1 --oneline`.
- `git diff --stat -- docs/mpvi_decisions.md docs/design_plan.md docs/progress.md` (before each edit: clean).
- `grep -rn -E 'M244|M245' docs/ CLAUDE.md` (no clash).
- Secret search and size check: `grep -n -I -i -E '<keywords>' <files>`, `grep -n -I -E '<email pattern>'`, `grep -h -o -I -E '[A-Za-z0-9+/=]{24,}'` with mixed-case and digit filters, `stat -c %s` for sizes.
- Patches: `cp data/bounce_tests_20261103/test_switches.patch experiments/test_patches/`; `cp data/bounce_tests2_20261103/test_switches2.patch experiments/test_patches/`; README written with a short Python script.
- Edits: a Python script added the M244 row and detail section and appended the seven UPDATE notes (each row's text compared with its original, apart from the note: all seven match), and a second script added the design-plan note and the progress entry.
- Commit 1: `git add -- <the nine paths>` (explicit paths only), then `git commit -F -` with the message and the Co-Authored-By line.
- Commit 2: `git add -- docs/mpvi_decisions.md docs/design_plan.md docs/progress.md`, then `git commit -F -` with the message and the Co-Authored-By line.
- Checks: `git status -sb`; `git rev-parse --abbrev-ref @{u}` (no upstream); `git log origin/main..HEAD`; `git diff --stat` (empty). No `git push` was run.

## NOT BUILT, needs decision

1. **Open decision (a):** whether to adopt a slower descent while holding a beaker (half speed gave a DOWN swing of 4.3 mm; one run, 200 g). Not decided.
2. **Open decision (b):** what to do about the weight over-read (a perfect weight fixed only about half the rise). Not decided.
3. **Open decision (c):** whether the 3 mm line applies to the DOWN swing. M206 says "each under 3 mm"; no reason is recorded. Not decided.
4. **Plan item marker not changed:** `ll. Lift-off weighing WEIGH step (DECIDED, built, runs pending)` in `docs/design_plan.md` still says "runs pending", which is now out of date. The rule allowed only the status note; whether to change the marker is for David.
5. **M206 (the handoff test with a VLA)** is still not built. The bounce tests used scripted lift-off runs, not a VLA handoff.
6. **Not tried** (recorded in M244): combining the z-only freeze with half speed; other speeds; repeat runs; 250 g in the two test rounds; the bridge target in the last batch.
7. **Unknown** (recorded in the test report): why the x trim ends 1.2 mm further from zero in the z-only run than in the control. This is the line you selected in `bounce_tests2_20261103.md`; it stays unknown.
8. **Test patches saved, not applied** to the branch (by design; see the README).
9. **Four untracked items left out** (`measstart`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`) and `deps/AgileX_Tracer`. Not decided; not changed.
10. **Nothing pushed.** Two local commits (`75cea83`, `5c42375`) wait for David's decision to push.

