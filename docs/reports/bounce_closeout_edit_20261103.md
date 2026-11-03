# Bounce close-out edit (2026-11-03)

Written for: David. Plain language. Documentation only: no code, config, setting or number was changed, and no simulation was run.

## Summary

- Open decision (a) in M244 was reworded for the VLA case, as David approved on Nov 3. The old wording is kept as history under it.
- A short note on the place DOWN lag was added after the open-decision list. Shown, calculated and guess are kept apart.
- **Commit: `d7e7691`** (local; not pushed; the branch has no upstream). It changed exactly two files: `docs/mpvi_decisions.md` and `docs/progress.md`.
- Nothing was decided. No plan item was marked DECIDED or DONE.
- Preconditions checked first: the close-out docs commit `5c42375` exists, M244 is in the log, and both files were clean before the edit (so no `git add -p` was needed).

## Before and after: open decision (a)

**Before** (line 303 of `docs/mpvi_decisions.md`, as committed in `5c42375`):

```
- (a) Whether to adopt a slower descent while holding a beaker. Half speed gave a DOWN swing of 4.3 mm (one run, 200 g).
```

**After** (the same list item, with the history line directly under it):

```
- (a) How the speed of lowering a held beaker is handled when the VLA drives the arm. Open options: (1) slow demos, so the training data lowers the beaker slowly; (2) a hand-speed limit below the policy while holding; (3) a velocity-based lag correction below the policy. None is decided. Context: half speed on the SCRIPTED descent cut the place DOWN swing to 4.3 mm, but the VLA will not use that speed setting.
  (Reworded Nov 3, 2026 after David's question. Original wording: '(a) Whether to adopt a slower descent while holding a beaker. Half speed gave a DOWN swing of 4.3 mm (one run, 200 g).'.)
```

## The added note

```
**Note on the place DOWN lag (Nov 3, 2026).** Plain words. The labels are kept apart.

- **Shown by the data:** the hand lags the plan at the first peak by +4.89, +4.99 and +4.76 mm at 150, 200 and 250 g (about the same at all three masses). At half speed on the scripted descent the lag was +2.80 mm (200 g). The planned peak descent speed was 56.99 mm/s at normal speed and 28.67 mm/s at half speed. Sources: docs/reports/bounce_analysis_20261103.md (section 3b) and docs/reports/bounce_tests2_20261103.md (sections 2 and 4a).
- **Calculated from the data (two speeds only):** the lag ratio is 2.80 / 4.99 = 0.56, close to the speed ratio 28.67 / 56.99 = 0.50. The lag divided by the speed is 0.088 s at normal speed and 0.098 s at half speed. A fixed delay of about 0.09 s would give about 5.1 mm at normal speed and 2.6 mm at half speed. This is a two-point comparison, not a fit.
- **Guess:** the servo follows its target with an error that grows with speed (a "following error"), possibly with extra delay from the command bridge (which updated every 3 ticks in the test runs, that is 50 Hz at 150 Hz; shown by the bridge columns) and from the arm's inertia. The vertical sag trim then winds up against that lag and overshoots, which is the undershoot. Shown by the data: freezing only the vertical trim cut the undershoot from -4.40 to -1.28 mm and raised the lag from +4.99 to +6.31 mm. Whether that links the undershoot to the lag is the guess.
- **Not tested:** a descent with no beaker; a descent faster than normal (what a quicker VLA descent would do); a hand-speed limit; the effect of the command bridge's 0.5 rad/s joint-speed limit on hand speed (not calculated, so it is not known whether that limit would hold back a faster descent).
- **Link to the open item "what sits below the policy":** no item with that name is recorded in the repo (searched docs/ and CLAUDE.md). The nearest records are M240, limit (3) ("a policy run needs it below the policy"), and M228, limit (1) (the correction lives in the scripted skill code). This question belongs with that item, which David's team settles. Nothing is decided here.
```

## Commit

- Hash: `d7e7691`, message "Bounce close-out: reword open decision (a) for the VLA case; add lag note".
- Files in the commit: `docs/mpvi_decisions.md` (11 lines changed: 1 removed, 10 added) and `docs/progress.md` (2 lines added).
- Checks: `git status` shows only untracked files; `git diff --stat` is empty; no upstream is configured, and no push was run; no added line carries the withdrawn label.

## Commands run

- `git status --short`; `git log --oneline -3`; `git diff --stat -- docs/mpvi_decisions.md docs/progress.md` (both clean); `grep -n -E 'slower descent|holding a beaker' docs/mpvi_decisions.md`; `grep -rn -i -E 'below the policy|sits below' docs/ CLAUDE.md`.
- Python script: replaced the (a) line exactly once and inserted the history line under it; appended the note after the open-decision list (the last lines of the file).
- `git diff -- docs/mpvi_decisions.md` (checked that only the (a) line and the appended note changed).
- Python script: appended one progress entry to `docs/progress.md`.
- `git add -- docs/mpvi_decisions.md docs/progress.md`; `git diff --cached --stat`; `git commit -q -F -` (plain message and the Co-Authored-By line).
- `git show --stat --format= HEAD`; `git show --name-only --format= d7e7691`; `git rev-parse --abbrev-ref --symbolic-full-name @{u}` (no upstream).

## NOT BUILT, needs decision

1. **Open decision (a) stays open.** Options (1), (2) and (3) are listed, not chosen.
2. **"What sits below the policy" is not a named item in the repo.** The nearest records are M240, limit (3), and M228, limit (1). David's team settles that item; I did not add it to any plan.
3. **The lag mechanism is a guess.** The "following error" and the bridge delay are not tested. A descent with no beaker, a faster descent, a hand-speed limit, and the bridge's 0.5 rad/s limit on hand speed are all untested, as the note says.
4. **The 0.09 s delay figure is a two-point calculation**, not a fit, and is not a measured property of the servo.
5. **Nothing pushed.** Commit `d7e7691` (and the earlier `75cea83` and `5c42375`) are local.

