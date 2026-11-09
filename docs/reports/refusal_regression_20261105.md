# Refusal regression check: efc7d49 vs 444cc4f (2026-11-05)

Stance selection only (make_pipeline_task builds the task, no episode). Probe: scratch script outside the repo (refusal_probe.py); layouts set on an in-memory copy of TASK_CFG (y 0.66 for both objects). Each run through experiments/run_capped.sh, one at a time.

Commits: efc7d49 (scratch worktree ws_old, detached) and 444cc4f (scratch worktree ws, branch world-model-sources).

## Runs

| # | Commit | Layout (beaker x, hotplate x, gap) | Outcome | Chosen x, y, yaw, pedestal | Refusal text | Run time (s, probe) | Wall time (s) | Exit |
|---|---|---|---|---|---|---|---|---|
| 1 | efc7d49 | control: 0.85, 0.65, 0.20 | CHOSEN | 0.635, 1.135, -90 deg, 0.10 | none | 72.7 | 73.8 | 0 |
| 2 | efc7d49 | A: 0.81, 0.65, 0.16 | REFUSED | none | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them; blocked by ['hotplate'] | 43.2 | 44.3 | 0 (refusal recorded) |
| 3 | efc7d49 | B: 0.85, 0.69, 0.16 | REFUSED | none | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them; blocked by ['hotplate'] | 46.5 | 47.5 | 0 (refusal recorded) |
| 4 | 444cc4f | A: 0.81, 0.65, 0.16 | REFUSED | none | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them; blocked by ['hotplate'] | 47.9 | 48.9 | 0 (refusal recorded) |
| 5 | 444cc4f | B: 0.85, 0.69, 0.16 | REFUSED | none | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them; blocked by ['hotplate'] | 49.8 | 50.7 | 0 (refusal recorded) |

Notes:
- Exit 0 in runs 2 to 5 means the probe ran and recorded the refusal; the refusal is an exception inside make_pipeline_task that the probe caught.
- The control reproduces the earlier chosen stance (0.635, 1.135, pedestal 0.10) on the old commit, so the probe works there.

## Verdict

- Layout A: efc7d49 and 444cc4f give the same outcome (refused, same text, same blocker).
- Layout B: efc7d49 and 444cc4f give the same outcome (refused, same text, same blocker).
- So M266 did not cause the refusal. The old code refuses too.

## The rule that blocks the beaker

Code: src/bench_stance.py, filter_obstacles (lines 337-392), and self_blockers (lines 621-629). The message is built at bench_stance.py:733-734.

What it compares: for each bin of the beaker's reach region, the arm's chain (shoulder → pre-grasp point → grasp point, 24 samples per segment) against each other object as a vertical cylinder. A bin is removed when a chain sample is closer than the obstacle radius plus the margins to the cylinder's axis, within the obstacle's height range (lines 379-389).

Numbers, from the code and config (src/task_config.py):
- Obstacle radius: the hotplate's larger half-size, 0.09 m (HOTPLATE_SHAPE half (0.09, 0.09), src/world_model.py:34; _footprint in src/stance_select.py:123-124).
- obstacle_margin_m = 0.03 m (src/task_config.py:604, BENCH_STANCE_CFG, row M170).
- arm_radius_m = 0.06 m (src/task_config.py:605, row M170, placeholder).
- chain_samples = 24 (src/task_config.py:626). hand_standoff_m = 0.15 m (src/task_config.py:606), which sets the last segment's length and so its half sample step (0.15/23/2 ≈ 0.0033 m).
- The blocking threshold: a chain point is blocked when its distance to the obstacle axis is less than 0.09 (radius) + 0.03 + 0.06 + half step, i.e. about 0.18 m from the hotplate's centre (bench_stance.py:386).

Why the beaker is blocked: the cylinder axis is placed relative to each grasp point, so the grasp point is always at the layout gap from the hotplate's axis (bench_stance.py:379, `cx, cy = gx + ob["rel"][0], ...`). The grasp point's distance to the cylinder edge is gap − 0.09. It is blocked when gap − 0.09 < 0.18 + half step, i.e. gap < about 0.183 m.

Flip gap (derived from the code, not measured): about 0.183 m. Layouts A and B (gap 0.16 m) are below it, so every grasp point is blocked, the beaker's count is 0, and selection refuses. The demo MOVED layout (gap 0.20 m) is above it, and selection passes. The earlier passing runs (gap 0.24 m) are also above it. This is consistent with the observed outcomes but not checked by a sweep.

The rule's files are the same on both commits: src/bench_stance.py has no diff between efc7d49 and 444cc4f, and the obstacle values in src/task_config.py are unchanged. The only change to stance selection on the new branch is M266 (where the world comes from); it does not change this rule.

## Not done

- No spacing sweep, as asked, so the 0.183 m flip is derived, not measured.
- No repo change; the efc7d49 worktree was created detached and not modified.

## Decisions for David (default in brackets)

1. Keep the 16 cm refusal as a known limit until the obstacle rule is decided [keep].
2. Decide whether the hotplate's 0.09 m radius and the 0.03 m and 0.06 m margins should be revised (placeholders at M170) [keep for now].
