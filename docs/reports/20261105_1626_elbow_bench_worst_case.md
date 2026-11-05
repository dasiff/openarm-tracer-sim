# Elbow vs bench: worst-case gap by pedestal height (geometry only)

Date: 2026-11-05, 16:26. Read-only: no code, model, map or stance_passes.py change. No motion simulation. Not pushed.
Written for: David (decides the pedestal height and the clearance requirement).

## Answer

- Using the bound you asked for (lowest elbow = shoulder z − 0.2243 − elbow radius, with the allowances), the gap is **above 20 mm only at 0.20 m**. At 0.15 m it is **−10 mm** (short), and at 0.05 and 0.10 m it is well below zero.
- The **smallest pedestal height with gap ≥ 20 mm is 0.18 m** (0.1796 m with L1 = 0.2243; 0.184 m with the largest L1 of 0.2285). On the 0.05 m grid that is 0.20 m.
- This is a much stricter test than the earlier study (`docs/reports/20261105_1613_pedestal_elbow_clearance.md`, which found the elbow clears from about 0.03 m). That study checked the elbow over reachable side-grasp poses. This bound assumes the upper arm points straight down, at any pedestal. The two answers answer different questions; see "Reconciling with the earlier study" below.
- **The elbow bound does not cover the forearm or hand.** With the arm straight, the forearm collision geometry reaches 0.41 m below the elbow. The check does not bound it (see Caveats).

## Inputs

**Elbow collision geometry (both arms, from `models/scenes/chemistry_lab_beaker_hotplate.xml` via `models/robot/combined_robot.xml`).** Each arm has one elbow collision geom, `openarm_left_link4_collision` and `openarm_right_link4_collision` (mesh `link4_collision`, body `link4`, rotating about joint 4 = the elbow). Its mesh vertices were transformed to world at zero config and at the zombie pose (`TASK_CFG["home_q_left"]` / `home_q_right`).

| Arm | Radius about elbow (joint 4), max over vertices | Lowest link4 point relative to elbow, zero config | Lowest link4 point, zombie pose |
|---|---|---|---|
| left | **0.1188 m** | −0.0965 m | −0.0327 m |
| right | **0.1188 m** | −0.0965 m | −0.0404 m |

Radius used: **0.1188 m**, the sphere about the elbow pivot that contains the whole geom. It covers the downward extent at every pose checked (the geom dips at most 0.0965 m below the elbow).

**Upper arm length (shoulder pivot = joint 2, elbow = joint 4).** L1 = **0.2243 m** at zero config (both arms). Over the joint-3 (upper-arm roll) range it varies 0.2200 to 0.2285 m. The brief's "fixed 0.2243 m" holds only at joint 3 = 0; the lowest elbow is at the largest L1, so the 0.2285 m case is shown as a sensitivity.

**Shoulder height.** Joint 2 world z = **1.1555 + q** at q = 0 (settled base 0.1527 m + torso 0.3048 m + shoulder 0.698 m), both arms. Verified by forward kinematics, both arms. Joint 1 is at the same height.

**Tilt allowance: 1° of base tilt = 7.0 mm.** Tilting the base by θ about a horizontal axis through two floor contact points moves the shoulder vertically by θ × (horizontal distance from the axis to the shoulder, perpendicular to the axis). The contact points are `robot_specs.BASE_CONTACT_SPECS["points"]` (two wheels, four casters). The largest such lever over all contact-pair axes is **0.402 m (left) and 0.403 m (right)**, so the allowance is 0.402 × sin 1° = 7.0 mm (7.03 mm right). This takes the worst axis, i.e. the one that tips the robot over a contact pair. It is a bound on tilt about a floor axis; it does not model the wheel-caster geometry or a 1° tilt about an arbitrary axis.

**Sag allowance: 5 mm**, subtracted directly.

**Total allowance: 12.0 mm** (7.0 tilt + 5.0 sag).

## Table: worst-case gap, per pedestal height and arm

Formula: gap = (shoulder z − 0.2243 − 0.1188) − 0.960 − 0.0070 (tilt) − 0.0050 (sag). Raw (before allowances) and final gap in mm. Both arms give the same numbers.

| q (m) | Shoulder z (m) | Raw gap | Final gap, left | Final gap, right | Final gap with L1 = 0.2285 (both) | ≥ 20 mm? |
|---|---|---|---|---|---|---|
| 0.05 | 1.2055 | −97.6 | −109.6 | −109.6 | −113.8 | no |
| 0.10 | 1.2555 | −47.6 | −59.6 | −59.6 | −63.8 | no |
| 0.15 | 1.3055 | +2.4 | **−9.6** | **−9.6** | −13.8 | no |
| 0.20 | 1.3555 | +52.4 | **+40.4** | **+40.4** | +36.2 | yes |

Smallest pedestal height with final gap ≥ 20 mm: **0.1796 m** (L1 = 0.2243), **0.1838 m** (L1 = 0.2285). On the grid: **0.20 m**.

## Reconciling with the earlier study

The earlier report (`docs/reports/20261105_1613_pedestal_elbow_clearance.md`) found the elbow clears the bench from about 0.03 m, but it checked only reachable side-grasp poses (level hand at the beaker's band). This check takes the elbow at the lowest point any upper arm pose allows (shoulder − L1, straight down), regardless of whether the arm can reach there with a hand on the beaker. Both are right for their own question. The straight-down bound is the stricter one: it treats any arm posture as possible, so it is conservative for the actual approach.

## Caveats (not checked)

- **Forearm and hand are not bounded.** The forearm (link5 onward, rotating about the elbow) reaches 0.41 m below the elbow when the arm is straight (joint 4 = 0), and 0.011 to 0.018 m below it at the zombie pose. This check covers the elbow only, as asked. A forearm or hand bound is needed for "zero chance of hitting the bench".
- **Right shoulder xy was taken from the mirror**, not measured: the right shoulder z and L1 were measured (both match the left), and the tilt lever uses the mirrored right shoulder (−0.0301, −0.1535) in the base frame.
- The L1 "fixed" assumption ignores the joint-3 variation, which moves L1 by ±4 mm. The sensitivity column shows its effect (−4 mm on the gap).
- The tilt allowance is a bound about a floor axis through two contact points. It does not account for the actual pitch or roll the base would see, and it assumes the base is level at rest.
- No motion, collision or joint-limit checks. Joint 2 and joint 4 limits are not applied to the bound (the bound is a height bound, not a reachability one).
- The sag allowance of 5 mm is a value you gave; it is not measured on this robot.

## Decisions needed from David

1. **Which requirement?** The straight-down bound needs a pedestal of at least 0.18 m (0.20 m on the grid), with 0.15 m (the current parked height, M197) short by 10 mm. Alternatively, the requirement could be checked over the actual approach paths (the earlier reachable-pose result: about 0.03 m) — say which.
2. **Forearm and hand:** should the same bound be extended to the forearm and hand (they reach 0.41 m below the elbow with a straight arm)? If yes, that needs a separate check.
3. **Allowances:** is 1° of tilt about a floor axis through two contact points the right model, and is 5 mm of sag the value to use?
4. **Right shoulder:** OK to accept the mirror assumption, or should I measure it?
