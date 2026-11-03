# Bridge identity check, STEP 0-6 (2026-11-02)

Do not decide pass/fail here. Script: scripts/diag/check_bridge_identity.py (not committed). Data: data/diag/logging_20261101/step6 (sim run 12:13-12:17 with current src; no sim run in this step).

## STEP 0 src/ evidence
- control_loop.py: ours (BRIDGE_ACCEPTED_LOG: +7 lines); other session removed 4 marker lines.
- gravity_ff.py: ours (+6 lines, -4 lines: tau_gravity_R slice fixed, one-time warning); other session removed 3 marker lines.
- All other 70 src/ files: only marker lines removed (other session).

## STEP 2 sha256
```
5db9794b7db714861dd4411e5df0337fabbc31b5c9a96e0f06b5ae1b49a0c0a5  data/diag/logging_20261101/pre/baseline/episode/episode_log.csv
5db9794b7db714861dd4411e5df0337fabbc31b5c9a96e0f06b5ae1b49a0c0a5  data/diag/logging_20261101/post_head/episode/episode_log.csv
5db9794b7db714861dd4411e5df0337fabbc31b5c9a96e0f06b5ae1b49a0c0a5  data/diag/logging_20261101/post_bridge/episode/episode_log.csv
```
post_bridge command (tmux post_bridge, 12:13): bash experiments/run_capped.sh data/logs/post_bridge_20261102_121311.log python experiments/demo_video.py --record-only data/diag/logging_20261101/post_bridge

## STEP 4 identity (script output)
```
=== arm L ===
update events checked: 2577
max |residual| = 2.22045e-15 rad at tick 12426
5 worst ticks:
  tick 12426: 2.22045e-15 rad
  tick 12425: 2.22045e-15 rad
  tick 12424: 1.9984e-15 rad
  tick 12423: 1.9984e-15 rad
  tick 12422: 1.9984e-15 rad
events where nearest f is (1/3, 2/3, 1) at the three ticks: 2577 of 2577
nearest-f patterns (tick+0, +1, +2): (0.333, 0.667, 1.0): 2577
row tick minus bridge_tick at the first changed row, over events: [1]
first 3 events (row tick, bridge_tick at that row, nearest f at the three ticks):
  tick 4 bridge_tick 3 nearest f [0.333, 0.667, 1.0]
  tick 7 bridge_tick 6 nearest f [0.333, 0.667, 1.0]
  tick 10 bridge_tick 9 nearest f [0.333, 0.667, 1.0]
=== arm R ===
update events checked: 1478
max |residual| = 5.92191e-16 rad at tick 3812
5 worst ticks:
  tick 3812: 5.92191e-16 rad
  tick 3521: 5.92191e-16 rad
  tick 4216: 5.92117e-16 rad
  tick 4375: 4.44089e-16 rad
  tick 4194: 4.44089e-16 rad
events where nearest f is (1/3, 2/3, 1) at the three ticks: 1478 of 1478
nearest-f patterns (tick+0, +1, +2): (0.333, 0.667, 1.0): 1478
row tick minus bridge_tick at the first changed row, over events: [1]
first 3 events (row tick, bridge_tick at that row, nearest f at the three ticks):
  tick 4 bridge_tick 3 nearest f [0.333, 0.667, 1.0]
  tick 7 bridge_tick 6 nearest f [0.333, 0.667, 1.0]
  tick 10 bridge_tick 9 nearest f [0.333, 0.667, 1.0]
```

## STEP 3 code (src/control_loop.py, current line numbers)
Target build, bridge_update (lines 648-666): the accepted slot is set only on acceptance (`bridge["accepted"][s] = tgt.copy()`), and the command that is sent is `cmd_q`, which moves toward the target by at most `bridge_step` per update:
```
for s, tgt in bridge["target"].items():
    d_ = tgt - cmd_q[s]
    if float(np.max(np.abs(d_))) > bcfg["max_target_step_rad"]:
        ...
        tgt = bridge["accepted"].get(s, cmd_q[s])                                      # the previous accepted target stays
        d_ = tgt - cmd_q[s]
    else:
        bridge["accepted"][s] = tgt.copy()
    cmd_q[s] = cmd_q[s] + np.clip(d_, -bridge_step, bridge_step)
    out[s] = [float(v) for v in cmd_q[s]]
...
apply_cmd({"mode": "arms", "arm_targets": out, "period_steps": bridge_every * steps_per_frame})
```
Ramp (lines 569-578): `hold_qpos_targets[arm_act[s]] = q` then `ramp_from[moved], ramp_k[moved], ramp_len[moved] = last_ctrl[moved], 0.0, cmd.get("period_steps", ...)`.
Per physics step (lines 434-452): `block = ramp_from + (hold_qpos_targets - ramp_from) * frac`, `frac = clip((ramp_k + k)/ramp_len, 0, 1)`.
Gravity (lines 925-930): `ff_off = gff.torques(data) / gff.kp`, then per physics step `data.ctrl[:] = ctrl_steps[k]` and `gff.apply_offsets(data.ctrl, ff_off)`. So the gravity term is added after the ramp value.
Constants: rate_hz 50 (task_config.py line 456), so bridge_every = 3 ticks; bridge_step = 0.5/50 = 0.01 rad per update (line 457).

Formula the code implements: per update, cmd_q moves toward the accepted target by at most 0.01 rad; the servo target ramps linearly from the previous ctrl (`last_ctrl`) to the new `cmd_q` over 3 ticks (15 physics steps), so at the three tick ends the fractions are 1/3, 2/3, 1; gravity is added after.
Disagreement with the hypothesis: the ramp starts from the previous ctrl, not from the previous accepted target, and it ramps to `cmd_q`, not to `accepted`. The two agree in this run because every accepted change is <= 0.01 rad (see below), so `cmd_q` equals `accepted` at each update.

## STEP 4 extra checks (run, not in the script)
- max |accepted change| per event: L 0.0100000000000002 rad (32 events exceed 0.01 only by float noise), R 0.00275 rad. So the 0.01 rate limit never binds on these events.
- Earlier "identity" check (ctrl - accepted - tau_ff) max 0.00667 rad (L) and 0.00183 rad (R) equals (2/3) x 0.01 and (2/3) x 0.00275: the expected lag of the 1/3, 2/3, 1 ramp, not an error.
- Sample event L, joint 1, first row tick 4: X_old -0.0009890923, X_new -0.0009890842; ctrl - tau_ff at ticks 4, 5, 6 = -0.00098908962, -0.00098908689, -0.00098908416, equal to the 1/3, 2/3, 1 expected values.
- bridge_tick vs the row where the new value first shows: row tick minus bridge_tick = 1 for every event on both arms (bridge_tick 3, first new value in row 4). NOT VERIFIED: the cause (order of on_tick and the bridge update within a tick); I did not check the call order.

## STEP 5 padz vs data/diag/height_20261101/extract_on.csv
Note: extract_on.csv has `padz_mm_rel` (padz minus its value at TREF = 11598, in mm), not absolute padz. The comparison uses (step6 padz - step6 padz at 11598) x 1000.

| tick | step6 padz_rel (mm) | extract_on padz_mm_rel |
|---|---|---|
| 11598 | 0.0 | 0.0 |
| 11599 | -0.000849 | -0.001 |
| 11600 | 0.041406 | 0.042 |
| 11601 | 0.18154 | 0.182 |
| 11602 | 0.405546 | 0.406 |

Max abs difference over common ticks (241): shift -1: 1.4088 mm; shift 0: 0.000835 mm; shift +1: 1.40916 mm. Shift 0 is smallest.
extract_on: padz starts to rise at tick 11600 (first tick after 11598 where padz_rel increases by more than 1e-6 mm per tick; definition mine). held_mass_ticks first nonzero at tick 11599 (value 0.205); held_left_instr first nonzero at tick 11600.
NOT VERIFIED: the two runs are separate sims; the agreement at shift 0 is shown, but the replay timing of padz in the height script was not compared to the live state read in this extract.

## STEP 6 blend_v
Writers: src/mpvi_modules.py line 365 `S["blend_v"], S["blend_handover"] = v_pad, True` (v_pad is the pad's 3-vector speed, m/s, line 360 comment "the pad's speed now (m/s)"); consumed at line 330-331 `st["v0"], S["blend_v"] = S["blend_v"], None`.
Extract script lines 200-208: `bv = S.get("blend_v")`, `row[f"blend_v_L{i}"] = float(bv[i - 1]) if bv is not None and i <= len(bv) else ""`.
Filled ticks (step6, 4 rows): 9916, 10468, 11853, 12517 (values: L1, L2, L3 = pad velocity x, y, z in m/s; L4..L7 always empty).
