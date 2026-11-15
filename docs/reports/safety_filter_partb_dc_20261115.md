# Safety filter Part B: never-close pairs by divide and conquer (2026-11-15)

Branch safety-filter-speed, not pushed, main untouched. Written for David. Step log: docs/reports/safety_filter_speed_log.md ("PART B").

## Outcome
- `tools/never_close_dc.py` proves 219 of the 518 pairs that the Stage 1 list leaves checked can never come within margin + 1 cm inside the joint limits (200 arm vs body, 19 arm vs arm). 265 can come close (a real pose was found), 34 are unproven and stay checked.
- All soundness checks pass (certificate, golden replay, gate A and B; falsification: see the table below).
- The list is NOT in use. The file format and the loader are structural decisions and are left to David (SF-77). The gate and the profile ran on a scratch copy of the source tree that points at a trial shapes file; nothing in `src/` changed except a new settings block `NEVER_CLOSE_DC` in `src/safety_filter/config.py`, which only the offline tool reads.
- With the trial list the pair table shrinks from 518 to 299 robot pairs; the time per call improves by a few percent to about 15 percent, inside the run-to-run variation of this VM. No timing target changes status.

## Soundness (summary; full argument in docs/safety_filter_never_close_soundness.md)
For a box of joint values, the gap of a pair is at least (a value that is at most the gap at the box centre) minus the travel of each shape, where travel is the largest, over the shape's end points or corners, of the sum of half range x lever arm over its joints. The lever arm is a pose-independent upper bound of the point's distance from the joint axis, so one joint moves a point by at most lever arm x angle, joints add up by the triangle inequality, every point of a rigid capsule or box moves no further than its end points or corners, and the distance between two sets falls by at most the sum of their travels. A pair is declared only when every piece of the full joint limits has this bound at or above margin + 1 cm. It assumes hinges and slides only, shapes rigid in one body (both checked by the tool on the model), and joint values inside the model limits (the same assumption as Stage 1; the filter does not clip commands). The bound is loose (lever arms assume a stretched arm), which only leaves pairs checked that could have been dropped.

## Method (as built; settings in config NEVER_CLOSE_DC)
- Proof on the filter's own capsules and boxes (stricter than the real geometry, which they enclose). One certificate per shape pair.
- Variables of a pair as in the Stage 1 sweep. First 20000 random poses and a short local search: a pose below the threshold ends the pair as "reachable".
- Box test as above; an unsettled box is cut in half along the variable with the largest half range x lever arm among those wider than 0.05 rad (5 mm for a slide); a box with nothing left to cut ends the pair as "not certified"; 120 s cap per pair ("time cap"). The lowest-bound boxes are cut first, so a hopeless pair fails early.
- Before any pair: model and shapes assumptions asserted; the tool's batched placement compared with the filter's on 300 random states (largest difference 8.9e-16 m); lever arms compared with real distances from the axes (never smaller).

## Results
| pair type | certified | reachable | not certified | time cap | total |
|---|---|---|---|---|---|
| arm vs arm | 19 | 220 | 6 | 0 | 245 |
| arm vs body | 200 | 45 | 10 | 18 | 273 |
| total | 219 | 265 | 16 | 18 | 518 |

Certified by part: arm links 3 to 7, palm and fingers vs the seven base boxes 161; vs battery 15; vs pedestal base 13; vs pedestal top 10; link 3 vs eagle camera 1; arm vs arm: link 1 vs links 3, 4, 5 of the other arm 13, link 2 vs links 2, 3: 4, link 3 vs link 3: 2. By number of variables: 3: 1, 4: 39, 5: 27, 6: 42, 7: 20, 8: 62, 9: 28.
Tool cost: 3597 s of pair time (two pieces, 40 and 20 minutes wall), 122.9 million boxes, peak memory 2.17 GB.

## Soundness checks
| check | result |
|---|---|
| i. certificate | PASS, 219 of 219: proven again from scratch with no time cap, settled volume exactly 1 for every pair, same box counts and bounds as the first run (21.15 million settled boxes); 231,052 boxes re-checked with the filter's scalar functions at 1,848,416 random poses: gap never below the box bound (smallest surplus 21.6 mm) |
| ii. falsification | FALSIFY_ROW |
| iii. golden replay | PASS: six recordings, 75,885 poses (raw, last and filtered command of every call), no declared pair below its threshold; closest 131.5 mm (left_link3_2 vs right_link1_2, 91.5 mm above its threshold); no recorded command outside the joint limits |
| iv. gate A (trial list) | PASS: 0 failing ticks on six recordings, largest command difference 0, every counter identical; the informational "active set" differs on the same 1483 and 1302 ticks as without the list |
| iv. gate B (trial list) | PASS: punch original, punch reactive and cat closed loop identical to golden (commands and status) |

## Timing (ms per call; same session, replay of the recordings; before = working tree, after = trial list)
| stream | before mean | before p95 | after mean | after p95 |
|---|---|---|---|---|
| punch original | 55.7 | 98.4 | 54.3 | 96.1 |
| punch reactive | 9.21 | 16.3 | 7.42 | 14.3 |
| cat | 5.69 | 30.8 | 4.83 | 25.0 |
| shadow pass-through | 1.61 | 4.02 | 1.33 | 3.45 |

Closed loop with the trial list (gate B): punch original 52.8 mean / 91.6 p95, punch reactive 10.4 / 19.7, cat 5.43 / 23.2 (before, from the speed report: 60.2 / 101, 8.80 / 17.2, 5.33 / 23.1). Timing on this VM varies by about 20 percent between runs, so only the direction is reliable.
Pairs: robot pairs in the table 518 -> 299; mean pairs looked at per tick: punch original 472 -> 283, punch reactive 463 -> 281, cat 146 -> 126, shadow 38.4 -> 36.2.
Targets (mean 6.7 ms and p95 20 ms on punch, reactive punch, cat; pass-through 1 ms): unchanged. Met: cat mean, reactive p95 (19.7 in the closed loop, close to the line). Not met: punch original, reactive mean (closed loop 10.4), cat p95, pass-through 1.33.

## Tests
Filter test set with the working tree (the list is not in use, so the tests see the same code as before): 12 of 13 pass, `safety_filter_spec_test` is the known old failure. TESTS_AFTER

## Decisions for David
1. Loader and file format (SF-77, not built). Proposed: `tools/build_filter_shapes.py --merge-never-close` reads the sweep file, the DC file or both, refuses a stale fingerprint and writes the union into `robot_shapes.json` (`never_close.pairs`, each marked with its source); the runtime loader then needs no change. Alternative: `shapes.py` reads `never_close_dc.json` at run time.
2. Joint limits (assumption A3). The proof covers the model's joint ranges only and the filter does not clip commands. Option: prove over limits widened by a small pad (not built), or have the filter treat an out-of-limit command separately.
3. Threshold. A declared pair can never come within margin + 1 cm, but the gap rule can also slow a candidate pair that is further away when one tick would close more than 20 percent of its gap above the margin; a declared pair is no longer slowed that way (the Stage 1 list has the same property). No difference on the recorded streams. A larger threshold would make this provable; not built.
4. SF-27 parts not built: "tiny pieces use a real pose's gap" (left out on purpose: one pose proves nothing about the rest of a box), the "blocked by the torso" flood, and the cross-check of the Stage 1 list (the tool has `--include-listed`; not run).
5. The proof is on the filter's shapes, not the real geometry as in Stage 1 (stricter; SF-27 did not say).
6. Split rule: largest half range x lever arm instead of "largest range" (ranges mix radians and metres); slides stop at 5 mm.
7. The tool's settings live in `src/safety_filter/config.py` (`NEVER_CLOSE_DC`), so that file changed although it is not the loader.
8. 34 unproven pairs (palm, link 7 and fingers vs pedestal and battery, and 6 arm vs arm). Options, not built: lever arms computed per box (tighter, the proof extends), a longer cap.

## Not run
Cross-check of the 147 Stage 1 pairs with the DC tool; the full test suite (no filter code changed).
