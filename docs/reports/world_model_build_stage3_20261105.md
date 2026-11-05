# World-model build, stage 3: cheap and slow passes (2026-11-05)

Branch world-model-build (stage 2 is c050fea). This stage adds the cheap and slow classification (src/stance_passes.py), the mode setting (stance_check_mode, default "current"), and this report. The mode is NOT yet read by the stance search: wiring the passes into the overlay and the clearance ranking key is blocked on the slow-pass candidate cap (NEEDS DECISION, §5). Default behaviour is unchanged.

## 1. What was built

- `src/stance_passes.py`: per stance, arm and bin, the three poses (grasp, pre-grasp, over-the-top) solved with the verification's IK and zombie start, the bin constraint, the 3 mm tolerance, and 150 iterations. The three poses of a bin are solved in one batch. The clearance is the smallest hand/finger/forearm distance to the world-model boxes (world_collision.py), uncapped for the tests and capped at the margin for the ranking.
- Classification: `hard` (grasp IK fails, or the grasp penetrates a box), `cheap` (grasp clears the margin), `a` (grasp within the margin; neither pre-grasp nor over clears it), `b` (grasp within the margin; pre-grasp or over clears it). Slow pass: `a` and `b` are accepted with the clearance (the smallest over the three poses, capped at the margin) as a soft penalty.
- `task_config.py`: `stance_check_mode = "current"` (row M272, PENDING David). Not read anywhere yet.
- Not built: wiring into `bench_stance.overlay` and `stance_select.select_bench_stance`; the clearance key (key 3.5 in the ranking); the candidate cap; the cost-bounded ordering by region and reach score. These need your decision (§5).

## 2. Results

### (b) Grid over stances (both arms; bins 9, 10, 11, 0, 1, 2, 3; grasp, pre-grasp and over-the-top)

Grid: 0.10 m steps, forward 0.1 to 0.9 m and lateral −0.6 to 0.6 m (117 stances per layout), pedestal 0.15 m. IK only: no reach map, no camera, no furniture, no support filter. Cells where the IK fails for every bin are not listed.

| Layout | Stances with a cheap bin (left / right) | Left: cheap (a, b) by bin | Right: cheap (a, b) by bin |
|---|---|---|---|
| Control (beaker 0.85, hotplate 0.65, gap 0.20) | 24 / 20 | bin 9: 10 (0, 0); bin 10: 2 (4, 11); bin 11: 22 (0, 0); bin 0: 5 (5, 16); bin 1: 0 (4, 6) | bin 9: 5 (0, 0); bin 10: 1 (4, 6); bin 11: 17 (0, 3); bin 0: 17 (2, 7); bin 1: 0 (6, 13) |
| A (beaker 0.81, hotplate 0.65, gap 0.16) | 10 / 5 | bin 9: 10 (0, 0); bin 10: 2 (4, 11); bin 11: 0 (6, 16); bin 0: 0 (1, 3); bin 1: 0 (0, 0) | bin 9: 5 (0, 0); bin 10: 1 (4, 6); bin 11: 0 (3, 17); bin 0: 0 (4, 12); bin 1: 0 (0, 0) |
| B (beaker 0.85, hotplate 0.69, gap 0.16) | 10 / 5 | identical to A | identical to A |

- At 0.16 m the cheap pass finds stances (left bins 9 and 10, right bins 9 and 10). The cheap pass is therefore not empty at the 0.16 m gap, and it succeeds in bins that the old padded circle refused.
- Slow candidates at A (if the cheap pass were empty): class (a) 11 left and 11 right; class (b) 30 left and 35 right. These are counted, not run to a stance, because the cheap pass already found candidates.
- The surviving bins at A are bins 9 and 10 (cheap). Bins 11 and 0 survive only through class (a) or (b) at this grid. Their soft clearances are in the working output; the raw cheap clearances were not stored, so they are not reported here.
- A and B are identical, as expected: the test depends only on the beaker-to-hotplate offset.
- Caveat: IK-only survivors with no reach-map check. Stage 1 showed that bins 9 and 10 are not reachable at the demo stance (the map and the verification agree), so a reach-map check could remove many of these cheap bins. This is the key open question for the cap.

### (a) Stored baseline stances with the new key

NOT RUN. The clearance key is not wired into the overlay, so there is no selection to rerun. The decision NEEDS DECISION (§5) comes before wiring. The stage 2 listing already shows that the stored chosen stances keep a workable grasp (left bin 11 at about 39 mm) under every robustness variant.

### 3. Contact check (runs 0, 3, 4)

Min distance of the left hand, fingers and forearm to the real hotplate geoms over every tick of the run (signal trace joints; base and pedestal from the episode log), with the checker's static clearance at the same stance and bins (margin 30 mm, uncapped distances):

| Run | Layout / stance | Trajectory min, hand / fingers / forearm (mm) | Time of min (s) | Static grasp clearance, bins 0 / 1 / 11 (mm) |
|---|---|---|---|---|
| 0 | moved (hotplate 0.65), (0.635, 1.135) | 23.6 / 23.4 / 26.5 | 80.8 to 85.7 | 9.6 / 4.5 / 38.5 |
| 3 | hotplate 0.61, (0.605, 1.145) | 25.8 / 23.7 / 26.1 | 86.5 | 40.8 / 38.7 / not solved |
| 4 | beaker 0.89, (0.635, 1.135) | 27.5 / 25.0 / 28.0 | 85.8 | 41.0 / 38.9 / 39.7 |

No run comes close to contact. The closest approach (23 to 28 mm) happens between 80 and 86 s, during the carry from the pick to the place, not at the grasp. The static grasp clearances of bins 0 and 1 in run 0 (9.6 and 4.5 mm) are the smallest in the table, but the arm does not stay at the grasp pose for long enough to matter. The trajectory margin is therefore larger than the grasp margin.

### 4. Robustness (boxes shifted and resized), with a workable / not workable classification

Workable: at least one bin has an accepted grasp with non-negative clearance (raw, uncapped).

| Variant (both boxes) | Workable (control stored stance / moved stored stance) | Flips |
|---|---|---|
| Nominal | workable / workable | — |
| Position x, y, z (±0.03 m) | workable / workable | none |
| Yaw ±0.333 rad (edge moves 0.03 m) | workable / workable | none |
| Yaw ±0.087 rad (5°, new) | workable / workable | none |
| Size x, y, z (±0.03 m) | workable / workable | none |
| Combined (sizes ±0.03, centre ∓0.03; yaw plus shift) | workable / workable | none |

No stance flips between workable and not workable. Where the flips start for the clear / overlap state (from stage 2, accepted poses): the left bins 0 and 1 grasp flip to overlap at a 3 cm x shift or about 19° of yaw. With the 5° yaw variant, no flip occurs: the fragile grasps survive a 5° yaw error. The z-shift and z-size flips come only from the right arm's over-the-top pose at bin 2. Ordering changes among workable stances are information only (stage 2 gave the details).

### 5. Timing, memory and budget

- Both arms, all seven bins and three poses, per stance: 2.25 to 2.64 s (grid mean, 117 stances per layout). Per arm: about 1.2 s.
- The cheap pass alone and the slow classes separately were not timed separately. The grasp-only cost is roughly a third of the total, by pose count. This is an estimate; NOT MEASURED.
- The grid run: 864 s for three layouts, peak RSS 2.66 GB under run_capped. Within the 5 GB budget.
- Budget: no more than 10 s over today's ~65 s. At 2.4 s per stance (both arms, all poses), 10 s allows about four stances. Proposed cap: four stances per selection call, ranked by the region and reach score (NEEDS DECISION).
- The budget is not yet tested end to end, because the passes are not wired into the search.

### 6. Existing test suites, mode off (default)

| Suite | Result | Expected |
|---|---|---|
| experiments/beaker_hollow_test.py | 21/21, rc 0 | 21/21 |
| experiments/stance_select_test.py | 14/15, rc 1 | 14/15 (the navigation-off check fails on the base commit too, stage 1 of the earlier task) |
| experiments/full_check_test.py | 31/31, rc 0 | 31/31 |
| experiments/bench_stance_test.py | 13/13, rc 0 | 13/13 |

All counts are unchanged. The default mode changes no code path: stance_check_mode is only a config key. The verification code is untouched.

## 7. What I did not do

- No wiring of the cheap and slow passes into the overlay or the stance selection. The setting exists but is read nowhere. This is a real gap against the brief's item 1, and it is deliberate: the candidate cap and the cost decision come first.
- No clearance ranking key. The stored baseline stances were therefore not rerun with it.
- No gating, no change to pedestal_override_m, the sag layer, the verification code, or the design plan.

## 8. Items for David (NEEDS DECISION)

1. **Slow-pass candidate cap**: propose four stances per selection call, ranked by region and reach score, at about 2.4 s each (about 10 s over budget). Your answer decides whether to wire the passes at all. Please confirm or give another number.
2. **Reach-map check in the cheap pass**: the grid's cheap survivors at 0.16 m are IK-only. The map says bins 9 and 10 are unreachable at the demo stance. Should the cheap pass use the map's reach as a gate, as well as the IK? Default: yes, reach from the map is trusted (stage 1), IK confirms.
3. **Wiring**: once the cap and the reach gate are decided, the passes go into the overlay behind stance_check_mode. The clearance ranking key is then enabled only after the baseline-stance comparison in §2(a), which needs your go.
4. **Row M272**: the mode setting is recorded as PENDING.
