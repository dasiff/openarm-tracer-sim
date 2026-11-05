# Flared-elbow lookups for pedestal 0.16 m (read-only)

Date: 2026-11-05, 16:47. Read-only: no code, model, map or src/stance_passes.py change. Not pushed.
Written for: David (the 0.16 m flared-elbow rule).

## Summary

- **Sideways flare is joint 2 on both arms**, but the sign is opposite: outward flare is joint 2 **negative on the left** and **positive on the right**. Joint 1 is the forward/back swing and moves the elbow height most (about 1.6 mm per degree).
- **The brief's equivalence does not hold at the zombie pose.** "Elbow z ≥ 1.111 m" is the same as "25–30° from straight down" only if the shoulder is at 1.3155 m (joint 1 at zero). At the zombie pose the shoulder is at **1.3271 m** (joint 1 raises joint 2 by 11.6 mm). At that height, elbow z ≥ 1.111 is met at up to **15.6°** from straight down, not 25–30°.
- **Zombie pose is already inside the rule on the left**: upper arm 18.3° from straight down (elbow z 1.1152). The right arm is at 25.1° (elbow z 1.1229).
- **30° and 45° flares, pedestal 0.16 m:** no collision found with the torso, the other arm, the eagle camera (approximate box), the hotplate, the bench or the beaker. Smallest margins are 118 mm (other arm) and 158 mm (camera, approximate).
- **True lowest point of the elbow link (link4 collision geom) below the elbow joint over all sampled joint 1–4 orientations: −118.8 mm** (both arms). This equals the sphere radius used in the earlier report, so that bound is attained.

## 1. Joints (both arms, read from the scene model; world axes at zombie config, base yaw −90°)

Base frame used for "forward" and "lateral": base +x = world (0, −1, 0) (forward, toward the bench); base +y = world (1, 0, 0) (left side). The left arm sits at base +y, so outward for the left is +y, and for the right it is −y.

| Joint | Left: world axis, range (deg), zombie | Right: world axis, range (deg), zombie | Role |
|---|---|---|---|
| 1 | (1, 0, 0); [−200, +80]; +22.60 | (−1, 0, 0); [−80, +200]; −22.60 | Forward/back swing of the whole arm. Positive moves the elbow backward and up (left: +1° gives fwd −3.5 mm, z +1.6 mm). |
| 2 | (0, 0.923, 0.384); [−190, +10]; −14.71 | (0, 0.923, 0.384); [−10, +190]; +14.71 | **Sideways flare** (roll about the base fore-aft axis). Left: negative = outward (+1° gives lateral −3.9 mm). Right: positive = outward (+1° gives lateral −3.6 mm, outward for right). |
| 3 | (0.254, 0.372, −0.893); [−90, +90]; +15.53 | (−0.254, 0.372, −0.893); [−90, +90]; −15.53 | Upper-arm roll. Changes the upper arm length by ±4 mm and moves the elbow ~0.5 mm per degree. |
| 4 | (−0.932, 0.341, −0.123); [0, +140]; +126.12 | (−0.932, −0.341, 0.123); [0, +140]; +126.12 | Elbow flex. Does not move the elbow joint; it moves the forearm and hand. 0 = straight. |

Signs and limits are per arm, not mirrored: the right joint 1 and joint 2 ranges are the negation of the left's (right joint 2 has the same world axis as the left, so its sign convention is the opposite of the left's for the same outward flare).

**Which joints set the elbow height:** elbow z is set mainly by joint 1 (about +1.6 mm per degree, left) and by the pedestal (1:1). Joint 2 changes elbow z only slightly (−0.4 mm per degree, left) and sets the sideways angle. Joint 4 does not move the elbow. The upper arm's angle from straight down is set by joints 1 and 2 together; the forearm (joint 4) does not change it.

## 2. Zombie pose at pedestal 0.16 m, and 30° / 45° flares

Zombie values (`TASK_CFG["home_q_left"]`, `TASK_CFG["home_q_right"]`, in degrees): left (22.598, −14.709, 15.531, 126.122, −3.746, 27.308, 13.018); right (−22.598, 14.709, −15.531, 126.122, 3.746, −27.308, −13.018).

| Arm | Shoulder J2 z (m) | Elbow J4 z (m) | Upper arm from straight down |
|---|---|---|---|
| Left | 1.3271 | **1.1152** | 18.3° |
| Right | 1.3271 | **1.1229** | 25.1° |

Flare: joint 2 changed alone (other joints at zombie), solved so the upper arm is 30° or 45° from straight down, outward.

| Arm, flare | Joint 2 (deg) | Elbow z (m) | Upper arm |
|---|---|---|---|
| Left, 30° | −33.45 (zombie −14.71) | 1.1338 | (0.0959, 0.057, −0.1932) |
| Left, 45° | −51.06 | 1.1693 | (0.152, 0.042, −0.158) |
| Right, 30° | +20.52 (zombie +14.71) | 1.1319 | (−0.1055, 0.0395, −0.1952) |
| Right, 45° | +37.31 | 1.1677 | (−0.1575, 0.0246, −0.1594) |

All flared joint 2 values are inside the joint range.

## 3. Collision distances (minimum geom-to-geom distance, mm; positive = clear)

Other arm at zombie; pedestal 0.16 m; base at the stance (0.635, 1.135), yaw −90°.

| Check | Left zombie | Left 30° | Left 45° | Right zombie | Right 30° | Right 45° |
|---|---|---|---|---|---|---|
| Torso (`openarm_body_link0`, `pedestal_top`) | 64 | 64 | 64 | 64 | 64 | 64 |
| Other arm | 118 | 125 | 125 | 118 | 125 | 125 |
| Eagle camera (box, approximate) | 158 | 158 | 158 | 158 | 158 | 158 |
| Hotplate | 145 | 145 | 148 | 240 | 242 | 253 |
| Bench top | 181 | 179 | 181 | 181 | 180 | 179 |
| Beaker | 247 | 238 | 231 | >300 | >300 | >300 |

Notes on the pairs: torso minimum is always arm link 1 against the torso mesh (joint 1 unchanged, so unchanged by flare); other-arm minimum is the two hands at zombie and link 1 against the other arm's link 0 when flared; the closest hotplate point on the left is a finger, on the right the hand.

**Eagle camera** is not in the scene XML. It is added at load time by `src/cameras.py`. I used its box from `robot_specs.EAGLE_BODY_SPECS` (half sizes 0.015, 0.045, 0.0125 m) at `EAGLE_MOUNT` (0.03, 0, 0.9055) in the torso frame, and measured arm collision-mesh vertices to the box. This is a vertex-to-box bound, not a mesh-to-mesh distance. The value does not change with flare because the closest point is link 1.

## 4. True lowest point of the elbow link below the elbow joint

Elbow link = `openarm_{side}_link4_collision`, the only collision geom on link 4. Joint 4 is declared in link 4's body, so the geom rotates about the elbow joint too. I sampled joints 1–4 over their full limits (13 × 13 × 9 × 9 grid) and took the lowest vertex relative to the elbow joint.

| Arm | Lowest point below elbow (mm) | Joints at that point (deg, j1 j2 j3 j4) |
|---|---|---|
| Left | **−118.8** | 56.7, −73.3, 67.5, 87.5 |
| Right | **−118.8** | 36.7, 6.7, −90.0, 0.0 |

−118.8 mm equals the sphere radius (max vertex distance from the elbow joint), so the sphere bound is attained, not just an upper bound. At the zombie pose the same geom reaches only −33 mm (left) and −40 mm (right) below the elbow (from the earlier report).

**Not covered by this number:** the forearm and hand (link 5 onward) rotate about the elbow and reach 0.41 m below it when the arm is straight (earlier report). The elbow link number does not bound them.

## What was not checked

- Motion from zombie to the flare pose (path, speed, or the order of joint moves). Only end poses were checked.
- Forearm, wrist and hand against the torso, bench, hotplate or beaker at the flare poses: collision checks used all arm collision geoms, but only at the single flare pose.
- The other arm was held at zombie; combined flares of both arms were not checked.
- Base stance is assumed (0.635, 1.135, yaw −90°) from the stance report, not a verified stance for 0.16 m. Hotplate and beaker distances depend on it.
- The eagle camera check uses an approximate box (vertex-to-box), not the true geom.
- Joint 3 was held at zombie (±4 mm effect on upper arm length, not in the flare check).
- Pedestal 0.16 m only. The shoulder height moves with joint 1, so the earlier "shoulder z = 1.1555 + q" holds only at joint 1 = 0.
- Joint 2 flare beyond the 30° and 45° solutions.

## Decisions needed from David

1. **Define the rule in one unit.** Either "upper arm ≥ 25–30° from straight down", measured at the real shoulder (which is 1.3271 m at zombie), or "elbow z ≥ 1.111 m". At the zombie shoulder height these disagree: 1.111 m admits the upper arm at 15.6° from straight down.
2. **Flare joint:** use joint 2 for sideways flare (as checked here), or allow joint 1 as well (it moves the elbow height and fore/back position).
3. **Forearm and hand:** should the same bound be extended to them? They reach 0.41 m below the elbow when the arm is straight, and the elbow bound does not cover them.
4. **Stance for the collision check:** use the report stance (0.635, 1.135, −90°) or the reference stance (0.765, 1.085, −90°)?
5. **Next step:** a motion check from zombie to the 30°/45° flare (joint 2 moves: left 18.7° and 36.3°; right 5.8° and 22.6°), or stop here.

## Files

- Scratch scripts (not committed): `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/137e4ff7-1e6a-485b-baff-f71dff4caeb7/scratchpad/` (`flare.py`, `flare2.py`, `camcheck.py`, `sens.py`).
- Sources: `models/robot/combined_robot.xml`, `models/scenes/chemistry_lab_beaker_hotplate.xml`, `src/task_config.py` (`home_q_left`, `home_q_right`), `src/robot_specs.py` (`EAGLE_BODY_SPECS`, `EAGLE_MOUNT`), `src/cameras.py`.
