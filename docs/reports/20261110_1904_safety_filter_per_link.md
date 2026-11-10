# Safety filter per-link shapes (2026-11-10)

Kinematics only. Branch safety-filter, not pushed.

## Outcome in one paragraph
Per-link capsules (links 1 to 6, both arms, own frame, exact enclosure over all mesh vertices) are in `data/safety_filter/robot_shapes.json`. SF-16 (arm vs world, M1-M9; robot vs robot, P1/P3/P4 at pedestals 0.155 and 0.120 m) holds for everything **except left link 1**: no 1, 2 or 3 capsule set for it passed (40 MARGIN rows, all left_link1 pieces 2 and 3 vs torso, shape 2.5 to 13 mm, real 64 mm). The right link 1 passes with 3 capsules. The file ships left link 1 as the minimum-volume 3-capsule fit.

An error on my side cost time: the first search reported "0 MARGIN rows" because left link 1 was left with a stale cache entry (a 2-piece fit written for a 3-piece count). The shapes test caught it (stick-out 9.6 mm). The second search (about an hour) used exact all-vertex radii; a targeted re-search of left link 1 (about 45 minutes) found no passing set, and a final build without search gave the numbers below.

## 1. Torso share (measured before changing shapes; pedestal 0.155 m)
A = filter shape of X against the REAL Y; B = REAL X against the filter shape of Y; Q = real X vs real Y; S = filter vs filter.

| pair, pose | S (mm) | Q (mm) | A: arm shape to real torso/finger | B: real arm to torso capsule/finger capsule | X-side share (Q-A) | Y-side share (Q-B) |
|---|---|---|---|---|---|---|
| right_upper_arm vs torso, P1 | +17.7 | +64.0 | +26.8 | +52.4 | 37.2 | 11.6 |
| left_forearm_1 vs torso, M9 | +15.1 | +72.4 | +15.9 | +72.3 | 56.5 | 0.1 |
| left_forearm_1 vs right_finger_right, M7 | +28.5 | +83.6 | +58.0 | +59.2 | 25.6 | 24.4 |

Torso decision: the torso capsule causes 9 to 12 mm (right upper arm) and 0 mm (left forearm), below the 15 mm threshold: **torso unchanged**. At M7 the other 24 mm comes from the right finger capsule (not a torso; fingers stay as they are).

## 2. Per-link shapes (final file)
Counts by search (1 to 3 capsules, minimum-volume split, first set with no SF-16 MARGIN involving the link).

| arm | link | capsules | radii (mm) | volume (cm^3) | what set the count |
|---|---|---|---|---|---|
| left | 1 | 3 (not met) | 13.9, 45.4, 56.1 | 2985 | left_link1_3 vs torso, +2.5 mm shape, real +64 mm (P3 M1); also left_link1_2 vs torso (+11.7 mm) |
| left | 2 | 1 | 49.2 | 1628 | rule met with 1 |
| left | 3 | 2 | 45.4, 36.3 | 1851 | 1 capsule vs bench, M5, ped 0.120 (earlier run: -19.5 mm) |
| left | 4 | 1 | 50.4 | 1978 | rule met with 1 |
| left | 5 | 2 | 44.4, 44.6 | 1961 | 1 capsule vs beaker, M3 (earlier run: -53.5 mm) |
| left | 6 | 1 | 38.0 | 337 | rule met with 1 |
| right | 1 | 3 | 46.0, 42.4, 34.8 | 3387 | right_link1 vs shoulder mounts and torso at P1 |
| right | 2 | 1 | 49.2 | 1628 | rule met with 1 |
| right | 3 | 1 | 45.0 | 2798 | rule met with 1 |
| right | 4 | 1 | 50.4 | 1978 | rule met with 1 |
| right | 5 | 2 | 46.3, 45.4 | 1724 | 1 capsule vs left palm 2, M7 |
| right | 6 | 1 | 37.8 | 348 | rule met with 1 |

Caveat on "what set the count": the set-by pairs for links 3 and 5 are from the first (hull-radius) search; counts were confirmed by the second search. Left link 1: closest pair left_link1_3 vs torso, shape +2.5 mm, real +64.0 mm (over-coverage 61 mm: the 56 mm radius piece's end cap reaches the torso side). The left link 1 mesh is the "flip" mesh, so it is not a mirror of the right link 1.

## 3. Torso exemption by real geometry (home, pedestal 0.155)
Real gap of each link's collision geometry to the real torso mesh (mm): shoulder mounts +1.0 (exempt); left/right link 1-3 (upper arm) +64; link 4-6 (forearm) +92; link 7 +142; palm +200; fingers +198 to +246. List: **shoulder mounts only** (`TORSO_EXEMPT_PARTS`). The SF-9 exemption of the left upper arm is withdrawn.

## 4. Arm vs world (M1-M9, both pedestals)
No MARGIN on a non-exempt shape. BAND flags on checked shapes: M2 link 7 vs bench (shape +16.2, real +16.4 mm), M2 palm vs bench (shape +14.9, real +16.4 mm). Fingers and the held beaker show MARGIN flags as before; both are exempt from the world (M289).

## 5. Robot vs robot (P1, P3, P4; both pedestals)
MARGIN rows: only left_link1_2 and left_link1_3 vs torso, at every pose (shape +11.7 to +13.4 mm and +2.5 to +4.6 mm; real +64.0 mm): loose shape, not REAL-INSIDE.
BAND rows: right_link1_2/_3 vs torso at every pose (shape +35.2 / +35.9 mm, real +64.0 mm); left_link1_1 vs torso at M7, M8 (+38.6 to +39.1 mm); left_link2 vs torso at M1, ped 0.120 (+38.0 mm, real +75.0 mm). No REAL-INSIDE flag anywhere.
Parked right arm at home: no MARGIN on a right-arm shape. Its link 1 pieces are at +35.2 and +35.9 mm from the torso (BAND), inside the 40 mm push-start distance, so a push would begin (to 50 mm) although the rule allows the band and the real clearance is 64 mm. Left arm at home: the left link 1 pieces violate the margin (see above), so the LEFT arm would be pushed at home.

## 6. Tests and timing
- Shapes test: PASS (200 poses, both arms, union enclosure per link; largest stick-out 2.9e-15 m).
- Touch test: PASS (new exemption list). Distance test: PASS (unchanged accuracy). Structure test: PASS.
- Timing, same 200 random states: place 0.35, broad 0.61, pairs 2.23, whole tick 3.28 ms (median). VM load average about 1.1 to 1.3 (my own runs only). Slower than the 0.87 ms quiet-window figure; more shapes (45) and more checked pairs (median 21).
- Fix: same-arm exempt reason label in touch.py corrected.

## 7. Not done
- Left link 1 does not meet SF-16 with 3 capsules. Options for David: allow BAND-like margin for the link 1 vs torso pair, exempt link 1 from the torso by a stated rule, split link 1 more finely (4+ capsules), or shrink the torso capsule (rejected by the 15 mm threshold).
- The search script prints only at the end; it should log progress per link.
- Pose-set check used cached IK poses (M1 and M9 at 0.120 m converged only to 4.0 and 1.3 mm).
