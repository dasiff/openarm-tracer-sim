# P3 step 1: waypoints of the scripted pick and place against the full check's points

Read from `plan_pick` / `plan_place` (src/mpvi_modules.py) and `step_target` (src/stance_select.py). Heights for the test scene: bench top 0.96 m, plate top 1.028 m, beaker 0.10 m, grasp fraction 0.525 (pad centre 52.5 mm up the beaker), `clearance_m` 0.20, `carry_gap_m` 0.04, `standoff_m` 0.12, `retreat_m` 0.05. "a" is the robot's forward direction (the skill always approaches along it). Pad-centre heights are approximate where they depend on the pad's height at that moment.

| Skill | Waypoint | Where (pad centre) | In the full check before P3? | Added (P3 s2) |
|---|---|---|---|---|
| pick | OVER | standoff back from the beaker along a, bench top + 0.20 = 1.16 | no | "over" |
| pick | DOWN | same xy, grasp height 1.0125 | no (xy 12 cm off the grasp points) | "standoff" (band middle height) |
| pick | IN | beaker xy, grasp height | yes: "grasp" band points (1.005 to 1.020) | |
| pick | UP | beaker xy, carry height (about 1.12: plate top + 0.0525 + 0.04) | no | "lift" (band middle + 0.10, about 1.11, the region's definition; the true carry height depends on the destination, unknown at the pick) |
| pick | BACK (only when the next navigation step moves) | 5 cm back along a at the carry height | no | "back" |
| pick | HOME | the zombie pose | covered by the home clearance, not by the IK path | |
| place | OVER | hotplate xy, carry height (about 1.12) | no: the "above" points are at set-down + 0.10 = 1.18 to 1.19, not the carry height | "carry" |
| place | DOWN | hotplate xy, set-down height 1.075 to 1.09 | yes: "setdown" band points | |
| place | BACK | 5 cm back along a at the set-down height | no | "back" |
| place | HOME | the zombie pose | home clearance only | |

Observations (report only, nothing changed):
- The full check lets every point try any approach bin (`_bin_order`), while the skill always approaches along the robot's forward direction (bin 0). The new points keep `bin: None` like the old ones, so the check is not stricter about the approach direction than before.
- The moved-layout failure was at the place OVER waypoint, which was not in the full check's points before P3 (see the overnight report).
