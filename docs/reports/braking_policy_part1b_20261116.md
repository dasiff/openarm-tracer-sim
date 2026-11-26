# Braking policy, Part 1 follow-up: pair pruning and sleep timers (f), fastest link (g), base and arms (h)

Date: 2026-11-16. Follows [braking_policy_part1_20261116.md](braking_policy_part1_20261116.md). Read-only: no code changed, no pruning changed, nothing pushed.

Code read from branch safety-filter (427f766 and this branch's history); the same `src/` tree is checked out on main. Measurement scripts were run from the session scratchpad (`followup/`), not committed; they can be added under `experiments/` if you want them in the repo.

## f) Pair pruning and sleep timers

### f.1 Is a pair sleep timer implemented? Yes.

`class SleepTimers` (src/safety_filter/tick.py:468-623, decision SF-72). Its docstring states the rule:

> "each popped pair gets wakeup_tick = now + max(1, floor((G - release) / (v x dt x (1 + safety_buffer)))), G a lower bound of its gap at the pose the filter returns, release = margin + slowing band, v the fastest the pair's two parts can approach each other"

The formula in code (tick.py:605-609):

```
rem = lb - tab.release[idx]
cost = np.maximum(self.vpair[idx] * t.res.dt * (1.0 + cfg.SLEEP["safety_buffer"]), cfg.SLEEP["min_cost_m"])
n = np.where(rem > 0.0, np.minimum(np.floor(rem / cost), cfg.SLEEP["stats_cap_ticks"]), 0).astype(np.int64)
n = np.maximum(n, 1)
self._push(idx, self.now + n)
```

The approach bound `v` for a pair is the sum of its two items' bounds (tick.py:584-585):

```
vitem = self._static_lever(t).sum(axis=1) * t.res.joint_speed_rad_s
self.vpair = vitem[tab.I] + vitem[tab.J]
```

- Enabled: [src/safety_filter/config.py:127](../../src/safety_filter/config.py#L127) `"enabled": True,`
- Active only when the resolver is given a joint speed bound. The plugin passes the command bridge limit (src/safety_filter_plugin.py:169-170): `speed = float(bcfg["max_joint_velocity_rad_s"]) if bcfg["enabled"] and cfg.get("replay") is None else None` and `Resolver(..., joint_speed_rad_s=speed)`.
- Speed bound value: 0.5 rad/s (src/task_config.py:466, COMMAND_BRIDGE_CFG).
- Safety buffer: 0.20 (config.py:129).

So a pair is skipped for n ticks, where n is the number of whole ticks its worst-case approach at the speed bound needs to close the gap from its lower bound G down to its release distance.

### f.2 The "slowing band"

There is no constant named "slowing band". The sleep's band is the release distance above the margin, which the code sets as:

- touch.py:46-47: `activation=margin + cfg.PUSH_START_BEYOND_M`, `release=margin + cfg.PUSH_STOP_BEYOND_M`
- config.py:41: `PUSH_START_BEYOND_M = 0.01      # m, PROVISIONAL [M288]`
- config.py:42: `PUSH_STOP_BEYOND_M = 0.02       # m, PROVISIONAL [M288]`

So the value the sleep uses is **0.02 m** (release = margin + 2 cm). The gap rule's docstring calls the margin-to-margin+1 cm strip the "slowing band" (src/safety_filter/gap_rule.py:5-6: "Inside the slowing band (margin to margin + 1 cm, released at 2 cm)"), which corresponds to PUSH_START_BEYOND_M = 0.01 m. The sleep docstring says "release = margin + slowing band" (tick.py:472) but uses the release value, so for the sleep the band is 2 cm, not 1 cm.

### f.3 What pruning exists, and the cutoffs

| Step | Where | Cutoff | What it drops |
|---|---|---|---|
| Pair cut in the object form | touch.py:172-175 | `max_act = max(MARGINS) + PUSH_START` = 0.04 m (0.06 m if a never-touch object is present) | pairs whose boxes are more than `max_act` apart |
| Activation test per pair (array form) | tick.py:238-240 `near = (ta.box_separation(tab, self._place(q)) <= tab.act) & ~self._in`; `near &= self._awake` | `tab.act` = margin + 0.01 m | any pair whose boxes are farther apart than its activation distance |
| Where the test is applied | resolver.py:102-103 `t.discover(last)`, `t.discover(t.raw)` | evaluated at the last command and the raw command only | nothing farther ahead is looked at |
| Sleep timer | tick.py:605-609 and 238-240 | n ticks from the speed bound | pairs that cannot reach release within n ticks |
| Never-close list | shapes.py:128-130 | fixed list, 366 pairs | pairs the real geometry cannot bring together |
| World broad phase | touch.py:210-229 `broad_phase()` | pad = largest radius + 0.01 + 0.01 m | **not called anywhere** (no callers in src/, experiments/, scripts/): dead code |

Activation cutoffs (object form, config values):

| Pair kind | Margin | Activation (cutoff) | Release |
|---|---|---|---|
| hand_hand_m | 0.010 m | 0.020 m | 0.030 m |
| robot_robot_m | 0.030 m | 0.040 m | 0.050 m |
| arm_world_m | 0.010 m | 0.020 m | 0.030 m |
| never_touch | 0.050 m | 0.060 m | — |

Measured with touch.pair_rule and box separation (f_pairs.py, robot shapes only, 366-pair never-close list loaded):

- Home pose: 299 touch-rule pairs checked, **0** pass the activation cutoff.
- 60 random poses (uniform in joint ranges): the cutoff keeps median **0**, maximum **53** of 299.

The "about 54 of 518" figure you quoted is from docs/reports/safety_filter_stage2c_diagnosis_20261112.md:42: "of 518 pairs that survive the never-close list, the broad phase passes only about 54 (at most 76) at the raw command". That was measured on 12 Nov, before the sleep timer and before the second never-close list (the table then had 518 robot pairs; docs/reports/safety_filter_speed_log.md:192 records 518 -> 299 after Part B). The current table has 299 robot pairs, which matches the count above.

Sleep statistics (docs/reports/safety_filter_speed_log.md:79), mean pairs looked at per tick: punch 473 of 518 (the punch commands are faster than the 0.5 rad/s bound, so every pair wakes), shadow 60 of 1214, cat 170 of 1247.

### f.4 Could pruning drop an obstacle farther than one tick of travel but still within braking range?

**Yes, for the activation cutoff.** A pair whose boxes are farther apart than its activation distance is not a candidate at the last or the raw command, however fast the arm is moving. The cutoffs on box separation (a lower bound of the gap) are 2 cm (hand-hand, arm-world), 4 cm (robot-robot) and 6 cm (never-touch), so a palm 3 cm above the hotplate is not checked, and the filter has no view of anything beyond those distances. One tick at 0.2 m/s is about 1.3 mm, so the cutoff is safe for the per-tick push, but it is not a look-ahead. A braking layer that needs obstacles out to its braking distance cannot use this candidate set.

Fingers and held objects are never checked against world objects at all (touch.py:69 and 71), so the cutoff does not matter for them except for never-touch objects.

**The sleep timer does not drop such obstacles while its bound holds.** A pair is skipped only for as many ticks as its worst-case approach at the bound needs to reach its release distance (tick.py:605-609); a command step faster than the bound wakes that arm's pairs (tick.py:557-558). Limits of that guarantee:

1. The bound is on **commanded** joint steps (tick.py:557-558). The arms' measured motion can lag the command by up to the bridge's 0.25 rad tracking-error limit (src/task_config.py:468), and the sleep length does not include that.
2. Base, pedestal and finger changes wake all pairs only if their change **in one tick** exceeds 2e-4 (tick.py:561 `if float(np.abs(sv - self.state_vec).sum()) > cfg.SLEEP["state_change_m"]`; config.py:130). Slow drift is not accumulated: a base moving below about 0.03 m/s, or turning below about 0.03 rad/s, never wakes the pairs, and the sleep length counts arm joints only (tick.py:584-585), so base motion during a sleep is not bounded.
3. The bound is loose (f.1 vs g below): the sleep is safe but short. That is conservative, not a risk.

## g) Fastest link

Inputs used:

- Joint speed limit: 0.5 rad/s per joint, the command bridge's `max_joint_velocity_rad_s` (src/task_config.py:466).
- Link lengths from the robot model (distance between consecutive arm joint anchors at the home pose): 0.0671, 0.0728, 0.1569, 0.1006, 0.1262, 0.0375 m (joint 1 to 2, 2 to 3, 3 to 4 (elbow), 4 to 5, 5 to 6, 6 to 7 (wrist)). Total shoulder-to-wrist about 0.56 m. Both arms identical.
- Hand: the link 7 origin, which is the filter's hand frame (src/safety_filter/shapes.py:240-243 `hand_frame`). Hand orientation is not constrained.
- Points checked: both ends of every arm capsule from robot_shapes.json (links 1 to 7, palm, finger).
- Hand speed 0.20 m/s, from the question.

Three bounds per part (m/s):

- **A. The filter's own sleep-timer bound**, replicated at the home pose from tick.py:505-523 and 584-585: v = 0.5 x sum of levers.
- **B. Joint limit only**, any pose: v = 0.5 x sum over joints of the Jacobian column norms, over 401 poses (home plus 400 random).
- **C. Hand at 0.20 m/s**: largest link speed over the feasible (pose, hand direction) pairs, with |w_i| <= 0.5 and J_hand w = 0.2 d. Exact by vertex enumeration of the feasible polytope; cross-checked against a linear program (the LP never exceeds the vertex result, which is the expected direction).

| Part | A: code bound, home (left / right) | B: joint limit only (left / right) | C: hand 0.20 m/s (left / right) |
|---|---|---|---|
| link 1 | 0.051 / 0.071 | 0.031 / 0.043 | 0.031 / 0.043 |
| link 2 | 0.124 / 0.124 | 0.087 / 0.087 | 0.062 / 0.062 |
| link 3 (ends at the elbow) | 0.336 / 0.673 | 0.097 / 0.069 | 0.071 / 0.049 |
| link 4 | 0.624 / 0.624 | 0.448 / 0.448 | 0.183 / 0.183 |
| link 5 | 0.837 / 0.819 | 0.581 / 0.580 | 0.222 / 0.240 |
| link 6 | 0.987 / 0.991 | 0.592 / 0.598 | 0.214 / 0.216 |
| **link 7 (wrist end)** | **1.300 / 1.300** | **0.733 / 0.733** | **0.309 / 0.310** |
| palm (hand, not a link) | 1.578 / 1.458 | 0.856 / 0.859 | 0.398 / 0.402 |
| finger (hand, not a link) | 1.557 / 1.557 | 0.872 / 0.871 | 0.411 / 0.407 |

Answer to the question:

- With the hand moving at 20 cm/s, the largest speed of any arm link (1 to 7) is about **0.31 m/s**, at the wrist end of link 7. That is about 1.5 times the hand speed, because the hand's orientation is free and rotation about the reference point moves the link-7 ends faster than the reference point.
- The elbow (link 3 ends at the elbow point; joint 4 anchor) stays below about **0.07 m/s** in that case. The elbow therefore does not move faster than the hand in this configuration.
- Without the hand constraint, the joint limit alone allows **0.73 m/s** on link 7 and **0.87 m/s** on the fingers and palm.

Caveat that matters more than the numbers: **20 cm/s is barely reachable under the bridge's joint cap.** Over the 401 sampled poses, 20 cm/s is feasible in only about 11% of (pose, direction) pairs. At the home pose it is feasible in 1 to 3 of 132 direction tests (two runs with different random direction sets). The hand's full speed capacity at home (linear program over the same bound, hand point = link 7 origin) is between **0.097 and 0.214 m/s**, median **0.134 m/s**, across 66 directions and both arms. So "the hand moving at 20 cm/s" is not a state the arms reach near home in most directions with 0.5 rad/s per joint. The values in column C are the maxima over the combinations that are reachable.

Where the link speeds can be obtained in the code:

- Joint speed limit: src/task_config.py:466 (COMMAND_BRIDGE_CFG), handed to the resolver at src/safety_filter_plugin.py:169-170.
- Per-item speed bound (A): `SleepTimers._static_lever` (src/safety_filter/tick.py:505-523) and `vitem` (tick.py:584).
- Link geometry per tick: `RobotShapes.place` and `place_arrays` (src/safety_filter/shapes.py:204-238) give every capsule's ends; `ArmKinematics.hand_pose_now` (src/safety_filter/arm_conversion.py:67) and `sew_now` (arm_conversion.py:76) give the hand and the shoulder, elbow and wrist.
- Actual link velocity: **not computed anywhere.** Options: a finite difference of the capsule ends between ticks (the sleep timer already keeps the previous state, tick.py:569 and 596), or MuJoCo's `mj_jac` on the robot-only model after `mj_comPos` (shapes.py `set_state` runs kinematics only, so `mj_comPos` is needed for Jacobians).

## h) Base and arms

Yes, in the default configuration. Quoted lines:

1. Exclusivity is off by default in the loop: src/control_loop.py:139 `"exclusive": False,  # False: only one subsystem moves at a time is OFF by default`.
2. Every field is routed to the keyboard source by default: src/control_loop.py:165 `DEFAULT_ROUTES = {k: "keyboard" for k in ROUTE_KEYS}`.
3. The keyboard source appends a base twist and an arm command to the same list in one tick: src/keyboard_source.py:233-234 `cmds.append({"mode": "base", "twist": (lin_vel, ang_vel)})`; src/keyboard_source.py:257-263 `if arm_targets or grip: cmd = {"mode": "arms"} ... cmds.append(cmd)`; the list is returned at src/keyboard_source.py:264 `return cmds`.
4. With exclusivity off, every non-idle command of the tick is applied: src/control_loop.py:1138 `to_apply = exclusive_select(cmds, t) if cfg["exclusive"] else [c for c in cmds if c["mode"] != "idle"]`.
5. The base is driven at every physics step while the arms are commanded: src/control_loop.py:983 `for k in range(steps_per_frame):`, :986-987 `if base_dof is not None: apply_base_velocity(data, base_dof, vx, vy, wz)`.

With exclusivity on, the base and arms are not meant to overlap:

6. The episode configuration sets it on: src/task_config.py:396 `"exclusive": True,  # one subsystem moves at a time`, in EPISODE_CFG (line 391), passed to the loop by src/episode_runner.py:259.
7. `exclusive_select` (src/control_loop.py:752-796): a command for the other subsystem waits (`ex["pending"] = cand`, line 793) while the active one moves, and a moving base is stopped when an arm command has to wait (control_loop.py:794-795 `if active == "base": stop_base()`).

Caveat: a waiting command starts once the active subsystem is "settled" (the settle thresholds, config `settle_*`), so a short overlap while the active one is still settling is possible. Not measured.

Related limit, not a steady cap: `arm_max_speed` = 1.0 rad/s per joint is a snap-back limit on the arm targets in the episode configuration (src/task_config.py:380). It is not the bridge's cap.

## Not done, and limits

- No measurement from recorded runs: actual hand and link speeds, command step sizes, how often the bridge rejects a step, and whether the base moves while arms are commanded in practice. Everything above is from the code and from the robot model.
- Link speeds are over sampled poses (401 random plus home). A higher maximum at a pose not sampled is possible; B is an upper bound by construction, C is the maximum over sampled poses.
- The hand point is the link 7 origin. A different reference point (for example the pad midpoint) gives different C values.
- Position limits of the joints are not applied in B and C (speed only), so these are upper bounds on what the arms can do.
- Scripts are in the session scratchpad, not committed (no repo code added).
- The commit for this report and the progress entry: see the progress log.

## Decisions needed from David

| # | Question | Why it matters |
|---|---|---|
| D1 | Does the braking layer get its own candidate distance (out to its braking range), or does it use the filter's activation cutoffs (1 cm past margin, 2 to 6 cm in total)? | The current cutoffs hide any obstacle beyond about 2 to 6 cm from a pair, at every speed (f.4). |
| D2 | Should the sleep bound include (a) measured arm lag beyond the command, and (b) accumulated base, pedestal and finger motion? | Today sleep is safe only for commanded arm steps and fast base changes (f.4 items 1 and 2). |
| D3 | Is 20 cm/s the intended hand speed, given that the 0.5 rad/s joint cap gives 0.10 to 0.21 m/s at home (median 0.13)? If not, what hand speed should the limit be set from? | At home the 20 cm/s hand speed is not reachable in most directions (g). |
| D4 | Which point is "the hand" for the limit: link 7 origin, the pad midpoint, or the held object's centre? | Link 7 ends move about 1.5 times the reference point's speed when the hand rotates (g). |
| D5 | Should braking output be allowed to reach the bridge's 0.5 rad/s cap, or should the braking limit sit below it? | The bridge's cap is the only steady joint-speed cap in the command chain; `arm_max_speed` (1.0 rad/s) is a snap-back limit, not a steady cap (h). |
| D6 | With exclusivity off (default loop), should the braking layer also cover base motion while the arms move? | Base and arm commands can be applied in the same tick (h, items 1 to 5). |

## Commit

Part 1 report and progress entry: commit 8a9ceb3 on branch safety-filter (not pushed). This follow-up: committed on branch safety-filter with the progress entry, not pushed.
