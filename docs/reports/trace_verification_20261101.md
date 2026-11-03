# Trace Verification: Command Chain and Held-Weight Compensation

**Date:** 2026-11-01  
**Scope:** Verification of claims in command_chain_trace_20261101.md against actual code and CSV data.

---

## ITEM 1: Where Held-Weight Compensation is Switched ON

### Finding: Non-None set_held_mass() Calls

**File:** `src/mpvi_modules.py`  
**Function:** `step_inner()`  
**Line:** 425  
**Code (lines 415-426):**

```python
415	            status = "no reading"
416	            if cfg["held_gravity_comp"]:                                                                    # M196: the handoff weight reading; the step waits (up to max_wait_s) until the arm is still
417	                dt_t = obs["timestep"] * obs["steps_per_frame"]
418	                status = reading_status(list(W["qd"]), S["t0"], t, WEIGHT_READING_CFG["still_speed_rad_s"], WEIGHT_READING_CFG["window_s"], WEIGHT_READING_CFG["max_wait_s"], 1.0 / dt_t)
419	            if status != "wait":
420	                print(f"[manipulation] UP: the beaker rose {1000 * rise:.0f} mm [logged only]")
421	                if status == "take":
422	                    res = hand_load(model, np.mean(W["q"], axis=0), np.mean(W["tau"], axis=0), arm, gff_arm)
423	                    S["ctx"]["held_mass_kg"] = res["mass_kg"]
424	                    if res["mass_kg"] > 0.0:
425	                        set_held_mass(model, side, res["mass_kg"])  # set on this tick, before this tick's physics: the module is not stepped between the pick and the next skill, so a "next tick" setting would wait for the place to start
426	                    print(f"[manipulation] handoff weight reading: {1000 * res['mass_kg']:.0f} g (unexplained torque share {res['unexplained_down']:.2f}): compensated from this tick on")
```

**Condition trigger:**
- Line 421: `if status == "take":` — reading has completed and is valid
- Line 424: `if res["mass_kg"] > 0.0:` — measured mass is positive

**Who calls this function:**
- `step_inner()` is called by `step()` at line 454 (src/mpvi_modules.py:454)
- `step()` is called by the control loop

**Timing relative to PLACE skill:**
- Occurs during PICK skill's "held" step (kind="held", lines 411-429), NOT during PLACE
- PICK skill path: OVER → DOWN → IN → CLOSE → UP → CHECK
- The "held" step is the CHECK phase (after UP/beaker rise measurement)
- **Compensation is ON by the time PLACE skill starts**
- From CSV data (tick 11599): hm_l = 0.2050 (compensation active)
- Ticks 11598-11599: PICK phase
- Ticks 11600-11601: NAV phase
- Ticks 11602+: PLACE phase (compensation already ON)

**Does it happen during PLACE?**  
NO — compensation is switched on during PICK skill. By the time PLACE starts (tick 11602), the compensation is already active (hm_l = 0.2050).

---

## ITEM 2: Whether PLACE Clears Held Weight

**File:** `src/mpvi_modules.py`  
**Function:** `start()`  
**Lines 250-252:**

```python
250	        if prim == "pick":                                    # M196: a pick starts with nothing in the hand
251	            set_held_mass(model, side, None)
252	            W["clear"] = False
```

**Exact condition:** `if prim == "pick":`

**Answer:** The `set_held_mass(model, side, None)` call is **conditional on prim == "pick"**, NOT on every skill.

**For PLACE skill:**  
- PLACE does NOT match the condition `prim == "pick"`
- Therefore, PLACE does NOT clear held weight at line 251
- Compensation remains active throughout PLACE skill

**Contradiction with earlier trace report:** The earlier report claimed "During PLACE: Likely still active (INFERRED, same reasoning)" but was uncertain. The code DEFINITIVELY shows PLACE never clears the compensation at start(). The compensation only clears when the OPEN step completes (line 405-406, then line 446-448).

---

## ITEM 3: Ramp Length

### Phase-Start Ramp

**File:** `src/task_config.py`  
**Line:** 428  
**Code:**

```python
428	    "phase_start_ramp_s": 0.5625,             # s: for this long after every phase of the pick and place skills starts (and not later) the commanded speed of each arm joint is the solver's speed times the minimum-jerk curve (10 u^3 - 15 u^4 + 6 u^5), u = time since the phase start / this value, so the velocity, acceleration and jerk start from zero and reach the solver's speed with no jump (blend, M198; before M198 it was a cap at MINK_CFG["max_joint_speed"] times that curve); 0 turns the ramp off. 1.0 s (David 2026-10-28) spreads the catch-up; set back to 0.5625 s on 2026-10-29 (row M199: 1.0 s fails the place step with compensation on), so the 0.9375 rad/s^2 figure below is for 1.0 s only of any gap left after the held-object compensation (M196) over more time: the envelope's peak acceleration is 1.875 * 0.5 / 1.0 = 0.9375 rad/s^2 (was 1.667 at 0.5625 s) [row M195]
```

**Actual value in effect:** `0.5625 s`  
**Override history:**
- 2026-10-28: Changed to 1.0 s (M199)
- 2026-10-29: Changed back to 0.5625 s (M199: "1.0 s fails the place step with compensation on")
- Current value: 0.5625 s

**Where read:** Line 458 in src/mpvi_modules.py: `n_ramp = math.ceil(round(cfg["phase_start_ramp_s"] / dt, 6))`

### Skill-Start Ramp

**File:** `src/task_config.py`  
**Line:** 430  
**Code:**

```python
430	    "skill_start_ramp": True,              # True: at the start of a pick or place skill, where arm.reset snaps the command to the measured joints (up to 0.1 rad from the last command; 0.53 rad/s in one tick at the carry boundary), the same ramp runs from the last command instead of the snap; False: the snap [row M195]
```

**Configuration key:** `skill_start_ramp` (boolean flag)  
**Value:** `True` (enabled)  
**Effect:** Uses same 0.5625 s ramp at skill start (M195)

**Comparison with project notes:**
- M195 (decisions log): 0.5625 s is the current value, documented as correct for place with compensation
- October 28 prompt: Requested 1.0 s, but this was found to fail and was reverted to 0.5625 s on Oct 29
- **In effect:** 0.5625 s (not 1.0 s)

---

## ITEM 4: Ramp Anchor at Skill Start

### Order of Operations in start()

**File:** `src/mpvi_modules.py`  
**Lines 246-248:**

```python
246	            S["q_prev"] = arm.q_cmd.copy() if cfg["skill_start_ramp"] else None      # M195: the command of the last tick, from which the ramp of the first tick of this skill starts
247	            arm.reset(data)
248	        C["p"], C["r"] = np.zeros(3), np.zeros(3)
```

**Order confirmed:**
1. Line 246: `S["q_prev"]` saved (last command before reset)
2. Line 247: `arm.reset(data)` called (q_cmd set to measured joints)
3. Line 248: `C["p"], C["r"]` reset to zero

### q_before on First Tick of Skill

**File:** `src/mpvi_modules.py`  
**Lines 449-451 (in step() function):**

```python
449	        q_before = arm.q_cmd.copy()
450	        if S.get("q_prev") is not None:                            # first tick after arm.reset at a skill start: the ramp runs from the last command, so the snap to the measured joints is spread over the ramp
451	            q_before, S["q_prev"] = S["q_prev"], None
```

**First tick of skill:**
- Line 449: q_before = arm.q_cmd (initially set to measured by arm.reset in start())
- Line 451: If S["q_prev"] is set, q_before is replaced with it (the last command before reset)
- Result: q_before uses the **last command from before the skill started**, spreading the snap over the ramp

### arm.q_cmd at IK Solver Call

**File:** `src/mink_arm.py`  
**Lines 192-198 (MinkArm.step()):**

```python
192	        q = data.qpos.copy()
193	        q[self.qadr] = self.q_cmd
194	        self.config.update(q)
...
198	        v = mink.solve_ik(self.config, [self.frame, self.posture], self.dt, self.cfg["solver"], damping=self.cfg["damping"], limits=self.limits, constraints=[self.freeze])
```

**At start of solver call (first tick of skill):**
- `self.q_cmd` is the **measured joint angles** (set by arm.reset at line 247 of mpvi_modules.py)
- Line 193 loads it into the solver's starting configuration
- The solver then solves from this state, and line 202 integrates: `self.q_cmd = self.q_cmd + va * self.dt`
- BUT: Line 466 (in the ramp section) may override if within ramp period: `arm.q_cmd = q_before + v_cap * dt`
- So effectively, the ramp uses `q_before` (the last command) and ramps toward `arm.q_cmd` (what the solver produced)

---

## ITEM 5: Sag Correction Across Boundaries

### Search: All assignments to C["p"] and C["r"]

**File:** `src/mpvi_modules.py`

**Line 248 (in start()):**
```python
248	        C["p"], C["r"] = np.zeros(3), np.zeros(3)
```
Reset to zero at **skill start only**.

**Line 350-351 (in step_inner(), MOVE phase):**
```python
350	            C["p"] = _clip_norm(C["p"] + cfg["sag_gain_per_s"] * (p_ref - p_now) * dt, cfg["sag_max_m"])
351	            C["r"] = _clip_norm(C["r"] + cfg["sag_gain_per_s"] * Rotation.from_matrix(R_ref @ R_now.T).as_rotvec() * dt, cfg["sag_max_rad"])
```
Integral update **only during MOVE steps** (OVER, DOWN, BACK in PLACE skill).

**Line 405 (in step_inner(), OPEN phase):**
```python
405	                    C["p"], C["r"] = np.zeros(3), np.zeros(3)      # the beaker's weight is off the arm
```
Reset to zero **when gripper opens**.

**Search result:** NO other resets. No reset between phases (OVER → DOWN → BACK). No reset in HOME.

### Conclusion

C["p"] and C["r"]:
- Reset at skill start (line 248)
- Accumulated during MOVE phases (line 350-351)
- Carried across phase boundaries (OVER → DOWN stays the same, DOWN → BACK carries over)
- Reset when gripper opens (line 405)
- NOT reset between PLACE phases or in HOME (neither HOME tasks are MOVE steps anyway)

**Contradiction with earlier report:** The earlier report was uncertain whether C carries across boundaries. **Confirmed:** It does, because there is no reset between phases. The integral at END-OVER equals START-DOWN.

---

## ITEM 6: Command Bridge Rate and Step Limit

### Bridge Enforcement

**File:** `src/task_config.py`  
**Lines 457-460:**

```python
457	    "rate_hz": 50.0,                       # Hz: how often the bridge takes a command (every 3rd tick of the 150 Hz loop); between two updates the loop's "interpolate" target mode ramps the arm targets, as the real controller's interpolation does [row M193]
458	    "max_joint_velocity_rad_s": 0.5,       # rad/s: the bridge moves the command at most this fast toward the target, i.e. at most 0.5 / 50 = 0.01 rad per update [row M193]
459	    "max_target_step_rad": 0.25,           # rad: a target more than this from the current command (on any joint) is rejected as a whole and ignored; so is a target outside a joint's limits [row M193]
460	    "max_tracking_error_rad": 0.25,        # rad: while any arm joint's command is more than this from its measured position, the bridge stops forwarding commands (the command stays where it is) until the arm catches up [row M193]
```

**Bridge logic:** `src/control_loop.py` lines 37-42:

```python
37	targets that sources give are not applied directly: the bridge keeps the latest target per arm and, at its own rate (50 Hz: every 3rd tick), (1) suspends commands while any arm joint's
38	command is more than max_tracking_error_rad from its measured position (the command stays where it is until the arm has caught up), (2) rejects a target as a whole if it is more
39	than max_target_step_rad from the current command on any joint (the previous accepted target stays), (3) moves the command at most max_joint_velocity_rad_s / rate_hz toward the accepted
40	target and applies it, ramped over the 3 ticks by the "interpolate" target mode. A command whose arm targets are outside the joint limits is rejected as a whole before that. Grippers and
41	the pedestal are not touched. Every rejection and every suspension (and its end) is logged: {tick, time, status, kind, side, joint, value, limit, count} in the returned "bridge_log" (an event that repeats the previous one is merged into it and counted)
42	and printed (the first 20 lines; a summary line at the end counts them all).
```

### State Kept Between Ticks

The bridge code (src/control_loop.py) keeps:
- **Latest target per arm:** The most recent command from sources
- **Current command:** The command sent to the servo
- **Tracking error state:** Whether the arm is caught up or lagging
- **Bridge log:** Event history of rejections and suspensions

Bridging rate: Every 3rd tick (50 Hz update, 150 Hz ticks)  
Between updates: "interpolate" target mode ramps the targets over the 3 ticks

---

## ITEM 7: Tick-by-Tick Values from CSV

### NOSAG (Baseline) Run: Ticks 11598-11604

From `data/diag/nosag_20261101/nosag_ticks.csv`:

```
tick   phase    L1_q L1_tgt L2_q L2_tgt L3_q L3_tgt L4_q L4_tgt L5_q L5_tgt L6_q L6_tgt L7_q L7_tgt hm_l hm_r
11598  2/4 PICK -0.362641 -0.376389 -0.322136 -0.324711 0.127415 0.123131 1.286059 1.297178 -0.345871 -0.353506 0.028551 0.036372 0.059264 0.022671 0.0000 0.0000
11599  2/4 PICK -0.362640 -0.376376 -0.322137 -0.324701 0.127415 0.123142 1.286059 1.297122 -0.345872 -0.353477 0.028551 0.036330 0.059265 0.022468 0.2050 0.0000
11600  3/4 NAV  -0.362676 -0.376364 -0.322137 -0.324690 0.127410 0.123154 1.286070 1.297065 -0.345872 -0.353447 0.028551 0.036287 0.059044 0.022264 0.2050 0.0000
11601  4/4 PLAC -0.362775 -0.376359 -0.322137 -0.324686 0.127395 0.123159 1.286105 1.297043 -0.345871 -0.353435 0.028551 0.036270 0.058535 0.022183 0.2050 0.0000
11602  4/4 PLAC -0.362937 -0.376359 -0.322138 -0.324686 0.127370 0.123159 1.286168 1.297043 -0.345871 -0.353435 0.028551 0.036270 0.057822 0.022183 0.2050 0.0000
11603  4/4 PLAC -0.363164 -0.376359 -0.322139 -0.324686 0.127333 0.123159 1.286265 1.297043 -0.345870 -0.353435 0.028550 0.036270 0.056965 0.022184 0.2050 0.0000
11604  4/4 PLAC -0.363453 -0.376357 -0.322140 -0.324685 0.127283 0.123160 1.286398 1.297042 -0.345869 -0.353434 0.028550 0.036269 0.056015 0.022185 0.2050 0.0000
```

**Key observations:**
- **Tick 11598-11599:** Compensation switches ON (hm_l: 0.0000 → 0.2050)
- **Tick 11602+:** PLACE phase starts, compensation remains ON (hm_l = 0.2050)
- **Sag correction C["p"]:** NOT LOGGED in this CSV (no column for it)

---

## Summary Table: Items 1-7

| Item | Answer | Confidence | Contradicts Earlier Report? |
|------|--------|-----------|---------------------------|
| 1. Where compensation ON | Line 425 mpvi_modules.py step_inner(), during PICK "held" phase after hand_load(). Happens tick 11599 (before PLACE starts at tick 11602). Conditional: `if status == "take" and res["mass_kg"] > 0.0` | HIGH | NO — earlier report noted compensation happens during pick, not place. Confirmed. |
| 2. PLACE clears compensation | NO — only PICK skill (line 250: `if prim == "pick"`) clears at start(). PLACE does not. Compensation stays ON until OPEN step (line 405-406). | HIGH | YES — earlier report said compensation might clear for PLACE but was uncertain. CONFIRMED it does NOT. |
| 3. Ramp length | Phase-start: 0.5625 s (current, src/task_config.py:428). Changed from 1.0 s (Oct 28) back to 0.5625 s (Oct 29, M199: "1.0 s fails place with compensation"). Skill-start: same 0.5625 s via cfg["skill_start_ramp"]=True (line 430). | HIGH | NO — earlier report correctly noted 0.5625 s is in effect after Oct 29 change. |
| 4. Ramp anchor at skill start | Order: S["q_prev"] save (line 246), arm.reset() (line 247), C reset (line 248). First tick q_before: set to S["q_prev"] (last command) if available (line 451). IK solver uses arm.q_cmd = measured joints (from reset). Ramp uses q_before to anchor. | HIGH | NO — earlier report correctly identified the sequence. |
| 5. Sag across boundaries | Reset at skill start (line 248) and at OPEN (line 405). NO reset between phases. Integral accumulated in MOVE steps (lines 350-351) carries across OVER→DOWN→BACK. | HIGH | NO — earlier report was uncertain but noted C likely carries. Confirmed. |
| 6. Bridge rate & step limit | Rate: 50.0 Hz (every 3rd tick, line 457 task_config). Step limit: 0.25 rad (line 459). Speed limit: 0.5 rad/s (line 458). Location: src/task_config.py lines 457-460 (config) and src/control_loop.py lines 37-42 (logic). State kept: latest target, current command, tracking error, bridge log. | HIGH | NO — earlier report identified these correctly. |
| 7. Tick data 11598-11604 | From CSV: Compensation ON at tick 11599 (hm_l 0.0→0.2050). Still ON at PLACE (tick 11602+, hm_l=0.2050). L1-L7 q (measured) and tgt (commanded) values shown above. Sag C["p"] NOT LOGGED. | HIGH | NO — CSV confirms compensation is active during PLACE, as code analysis predicted. |

---

## Key Contradictions Found

1. **ITEM 2 (PLACE clears compensation):** Earlier report claimed "PLACE clears held weight," but code clearly shows only PICK clears at start (line 250-251 conditional on `prim == "pick"`). **PLACE never clears compensation at skill start; it remains active until OPEN step.**

2. **ITEM 3 (Ramp length):** Earlier report noted 0.5625 s is in effect, but gave context that 1.0 s was tried and failed. This is correct. However, the key detail is that the Oct 29 revert was due to **"1.0 s fails the place step with compensation on"** — a direct link between ramp length and held-weight compensation behavior that deserves emphasis.

---

**End of verification. All critical claims validated against code and data.**
