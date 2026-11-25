# Navigation stage 1 of 7: the simulated lidar (placeholder unit)

Date: 2026-11-25, about 22:50. Written for David (decider) and for the person who builds stage 2 (the map grid).
Design: docs/design_plan.md section 13.1. Decisions: M328 (placeholder unit), M336 (lidar confirmed), M340 (navigation built first), M341 (build choices).
Commits: 64d015f (WIP, settings and module), 5e42f93 (WIP, tests, picture script, runner entry, records), a623ac4 (the two fixes and the corrected picture). Not pushed.

## What was built

- `LIDAR_SPECS` in `src/robot_specs.py`: 360 beams (1 degree), 10 scans per second (every 15 ticks at 150 Hz), range 0.15 to 12 m, range noise 2 cm (Gaussian), scan plane 0.05 m above the deck top. Every value is PROVISIONAL (M328).
- `src/lidar.py`: the beams are cast in Python with `mujoco.mj_multiRay`. The lidar pose is the base body's pose plus a mount read from the compiled model (the chassis mesh's front centre and top). The robot model and the scene XML are not changed. Beam states: range, no return, too close, self-blocked. A self-blocked beam is flagged and dropped; it never passes through the robot. Noise is seeded (drawn per lidar, saved with its scans). `navigation_observation()` is the only place a scan is built; the policy observation has no lidar entry. `save_scans()` / `load_scans()` write one compressed file per run through a temporary name.
- `experiments/lidar_test.py`: the checks below. Added to `experiments/run_tests.sh`.
- `experiments/lidar_slice_picture.py`: the picture below.

## Test results (run 2, final code; log data/logs/lidar_test_20261125_224413.log)

Run 1 failed before any check: the installed MuJoCo needs a `normal` argument before `nray` in `mj_multiRay`. Fixed in a623ac4.

| Test | Expected | Measured | Result |
|---|---|---|---|
| 1 known distance, east wall, facing +x | beam 0 reads the slab distance to the wall within 1 mm | expected 6.073004 m, measured 6.073004 m | PASS |
| 1 known distance, south wall, facing -y | same, second pose and heading | expected 9.573004 m, measured 9.573004 m | PASS |
| 2 bench leg fl (beam 271 deg) | distance predicted by the leg's box geometry, within 1 mm | 2.707275 m predicted, 2.707275 m measured | PASS |
| 2 bench leg fr (beam 88 deg) | same | 1.173852 m predicted, 1.173852 m measured | PASS |
| 2 bench top never hit | 0 beams hit the bench top (it is above the plane) | 0 | PASS |
| 3 self-blocking, lowest pedestal (0 m) | no arm or hand in the scan plane | plane at 0.2217 m; none in the plane; 48 self-blocked beams | PASS |
| 3 self-blocking, highest pedestal (0.418 m) | same | same; none in the plane; 48 self-blocked beams | PASS |
| 4 noise, 200 static scans | about 0.02 m per beam (tolerance 0.002 m) | mean 0.01996 m, median 0.01993 m over 312 range beams | PASS |
| 5 replay, same seed | byte-identical scans | identical (ranges and states compared as bytes) | PASS |
| 5 replay, different seed | different scans | different | PASS |
| 5 save and load | loads back unchanged, with the seed | unchanged, seed 2024 | PASS |
| 6 timing, 200 scans with noise | worst under 0.1 s (the 10 Hz period) | average 1.177 ms, worst 4.531 ms | PASS |
| 8 model unchanged | model hash before = after | 5a880d94a948 before and after | PASS |
| 8 policy observation | no lidar key | none | PASS |
| 8 navigation observation | keys lidar and time | lidar, time | PASS |
| 8 scan rate | 10 scans in 150 ticks | 10 | PASS |
| 7 manual baseline (run by hand) | byte-identical to the CURRENT reference | SHA-256 3060ae41cc85... equals experiments/baselines/manual_reference_20261103.csv; compare max difference 0.000000 on every column | PASS |

Test 7 was run after the suite had ended (the `dryrun_suite` chain was gone; no other MuJoCo job was running). Output: data/logs/lidar_manual_20261125_224737.csv.

## Measured geometry

- Base origin 0.1527 m above the floor (settled). Chassis top 0.019 m above the base origin, so the deck top is 0.172 m above the floor.
- Mount, base frame: front 0.3470 m (front edge of the deck), centre 0.0019 m (y), height 0.0690 m above the base origin (deck top plus 0.05 m).
- Scan plane height above the floor: **0.2217 m (about 0.222 m)**.
- Lidar origin at the staging stance: (0.765, 0.738) m. That is under the bench top's footprint (bench edge at y = 0.8 m), as the design says: the scan sees the legs, not the top.

## Self-blocked beams (arms at home, lab scene)

Angles are in the lidar frame: 0 deg is the base's forward direction, counter-clockwise positive. The same at both pedestal heights, 0 and 0.418 m:

- 48 beams, 157 to 204 deg.
- Beams 157 to 168 and 193 to 204 deg (24 beams) hit the **battery pack** at about 0.47 m.
- Beams 169 to 192 deg (24 beams) hit the **pedestal base** at about 0.29 m.
- No arm or hand reaches the scan plane at either height.

The pedestal top and the torso are above the plane, so raising the pedestal does not change the set.

## Timing (this VM, 200 scans, noise on, nothing else running)

- Average 1.177 ms per scan; worst 4.531 ms. The 10 Hz period is 100 ms.

## Picture

docs/reports/20261125_lidar_scan_slice.png. Three panels at the scan plane height: the staging stance, the staging approach spot (0.2063 m behind it), and the middle of the room (0.0, -2.2) m, heading 0. Grey is what crosses the plane, computed from each primitive geom of the compiled model. The bench top is a dashed outline labelled "bench top, above scan plane". Self-blocked beams are orange. The picture shows the bin, the cart legs, the storage cabinet, the back counter, the fume hood, the bench legs and the walls.

Two bugs found while making it, both fixed in a623ac4: (1) a first version used ray parity; where two cabinet doors touch, and where a grid row lies on a face, the count was wrong and drew false lines across the room. The slice now tests each primitive exactly. (2) The `mj_multiRay` argument list (above).

## Decisions

- M340: navigation built first (design plan section 9, 13.1).
- M341: the build choices of this stage (Python ray casting; no model change; self-blocked beams flagged and dropped; scan taken all at once; lidar only in the navigation observation).
- Design plan 13.1: the lidar item is marked BUILT, with the known simplification and the navigation-only rule. Also corrected: 13.1 said MuJoCo ray casting excludes the robot's own bodies; it does not. The build flags those hits instead.

## Not done / needs David

1. All lidar values are placeholders (M328) until the hardware team gives the unit.
2. No run loop writes scans yet. `save_scans()` exists and is tested, but stage 1 is the sensor only. When a run writes scan files, the seed should also go into the run record (`meta.json` still has `"seed": None`). I did not change the run record: that would change its contents for every run. Needs David's OK.
3. Stage 4 (observation slots) and its build position: still open (M336, M339).
4. The dryrun_suite chain ended during this work. I did not read its results (they are in the other worktrees' data/logs); I only checked that it had stopped before I ran any MuJoCo job.
5. The lidar test writes data/logs/lidar_test/replay_test_scans.npz (git-ignored).
