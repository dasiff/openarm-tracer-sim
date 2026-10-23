# Addendum G: support chain, surface, target height, bench_max_raise_m

Written for: David. Code: `src/support.py` (new), `src/world_model.py`, `src/orchestrator.py` (grounding hook), `BENCH_STANCE_CFG` (surfaces, band file, limit), `experiments/g0_reach_band.py`, `g0_raised_check.py`, `support_test.py`. Row M174. C3 not started.

## G0 answers
- **(a) Pedestal travel (model):** joint range 0.0 to 0.418 m (the spec says the same).
- **(b) Reach band** (bench-approach bins, path points with standoff, over and lift, the finder's requirements: framing, grippers, robot-hides-target, zombie clearance to the object and the furniture; a beaker-sized object on a support swept 0.70 to 1.60 m in 1 cm steps, some floor spot reaching it): lowest and highest grasp-point height above the floor per pedestal height (every 2 cm shown; the 1 cm table is in `docs/reports/g0_reach_band.json`):

| pedestal (m) | lowest (m) | highest (m) |
|---|---|---|
| 0.00 | 0.853 | 1.043 |
| 0.02 | 0.873 | 1.083 |
| 0.04 | 0.893 | 1.143 |
| 0.06 | 0.802 | 1.253 |
| 0.08 | 0.823 | 1.273 |
| 0.10 | 0.843 | 1.293 |
| 0.12 | 0.863 | 1.313 |
| 0.14 | 0.883 | 1.333 |
| 0.16 | 0.903 | 1.353 |
| 0.18 | 0.923 | 1.373 |
| 0.20 | 0.943 | 1.393 |
| 0.22 | 0.963 | 1.413 |
| 0.24 | 0.983 | 1.433 |
| 0.26 | 1.003 | 1.453 |
| 0.28 | 1.023 | 1.473 |
| 0.30 | 1.043 | 1.493 |
| 0.32 | 1.063 | 1.513 |
| 0.34 | 1.083 | 1.533 |
| 0.36 | 1.103 | 1.553 |
| 0.38 | 1.123 | 1.573 |
| 0.40 | 1.143 | 1.573 |
| 0.41 | 1.153 | 1.583 |

  **Minimum over all pedestal heights 0.802 m, maximum 1.583 m** (the bench top is 0.960 m: the highest support served is 0.57 m above it). Both ends move 1 cm per 1 cm of pedestal height; the jump at 0.05 is where the column stops colliding with the bench. Stored in `docs/measured/bench_reach_band.json` (what the grounding check reads).
- **(c) Support in the world model before G:** a `location` property only ("bench" for both), no on-top-of relation. The hotplate had no height field. The place target for "the hotplate" is the hotplate's top (the `hotplate_top` site, 0.068 m above the bench top), not the bench top; the finder's 12 task points for the place already use the plate's top + 2 mm + the band.
- **(d) The hotplate as an obstacle:** not for its own place step (an object's obstacle list excludes itself); it is an obstacle for the beaker's pick (cylinder up to the plate top, 9 cm radius placeholder), and its height enters that filter through that cylinder. The full check's IK path check also keeps the open fingers 15 mm from it.
- **(f) What the bench removes from the maps** (2-map pilot without the bench against the current maps, left bins 0 and 9; counts by height above the bench top in 1 cm steps are in the json): voxels are removed only from about 5 cm below to 3 cm above the bench top (pad centre height): about 4,000 to 6,000 per cm between -3 and +2 cm, 862 at +3 cm; above that "removed" and "gained" are equal (build noise: 51 and 59 at +5 cm). **The bench removes none above about 3 cm.** The task's pick (5.25 cm) and place (12.25 cm) are above it.
- **(g) Full check at raised targets** (beaker on a made-up box under it, the box an object of the check, the stance the overlay picks for each):

| raise | stance | region | full check |
|---|---|---|---|
| 0.05 m | (0.705, 1.135), pedestal 0.13 | covers both, counts 2 / 2 | PASS |
| 0.10 m | (0.735, 1.115), pedestal 0.14 | covers both, counts 3 / 2 | PASS |
| 0.20 m | (0.775, 1.035), pedestal 0.17 | covers both, counts 1 / 1 | **FAIL, home_clearance**: the arms at home are 10 mm from the box (limit 40 mm) |

  Region and full check disagree at 0.20 m: the region does not know the support under the object as an obstacle for the arms at home. Not built (next filter: the chain's entries as boxes in the zombie clearance).

## The limit
`BENCH_STANCE_CFG["bench_max_raise_m"] = 0.1325 m`: the highest passing raise (0.10 m) + the grasp offset (0.0525 m: the target is the pad centre, the raise is the support's) - 2 cm. Inputs: (f) 0.03 m, (g) 0.10 m. **Deviation, PENDING David:** the literal "smaller of (f) and (g), minus 2 cm" is 0.01 m, which would stop the test scene's own pick (0.0525 m above the bench top) and place (0.1225 m). (f) says where the bench still matters to the maps, not how high a target can be, so it is not the upper bound. The test scene's place is 0.1225 m against 0.1325 m: inside, 1 cm to spare.

## What changed
- World model: `supported_by`, `height`, `surface` on the entries of both sources (the beaker is supported by the bench at the start and by the hotplate once it stands on the plate; the hotplate by the bench; the bench is the surface root).
- `src/support.py`: chain, tops, target height (pick: the object's chain; place: the destination's, the base on its top plus the set-down gap; "held" for a held object with no destination), the stops (surface unknown; chain does not end at a surface, loops and missing entries included; outside the reach band; no stance method for surface; no stance method: above the bench top by more than the limit), the stance method per root, runs of the same root surface only, the null step only for the same surface.
- Grounding adds `surface`, `support_chain`, `target_height` to a step (only when the world model carries support information) and stops with those reasons. Test scene log: pick: surface bench, chain [beaker, bench], 1.0125 m; place: [hotplate, bench], 1.0825 m.
- Settings: `BENCH_STANCE_CFG["surfaces"]`, `reach_band_file`, `bench_max_raise_m`. The region builder still reads the bin list from the key it had; reading the surface block is for C3.

## Tests
`experiments/support_test.py` 18/18: (1) beaker chain (beaker, hotplate, bench 1.0805 m; beaker, bench 1.0125 m; the place 1.0825 m), (2) 20 cm box 1.2125 m, (3) no chain: "surface unknown: the green flask", (4) outside the band, (5) loops and a missing support, (6) beyond the limit: "no stance method: the beaker is 0.252 m above the bench top, limit 0.150 m", (7) the test scene's place inside the configured band, root without settings, held, runs, null step.
Existing, one at a time with the hook active (`data/logs/suite_g_*`): stance_finder 28/28, mpvi_postprocess 54/54, grounding 65/65, orchestrator_retry 25/25, stance_select 23/23, stance_select_pipeline 7/7, mpvi_pipeline 14/14 (live plan, recording replay bit-identical). The default stance behaviour is unchanged: nothing reads the new fields yet.

## Needs David
1. The limit rule (above).
2. The beaker's initial support is the bench, not the hotplate as in your note; the chain beaker, hotplate, bench exists once it is on the plate and for the place's target.
3. The region ignores the support under an object for the arms' home clearance (the 0.20 m case): add it before C3?
