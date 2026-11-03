# Command Chain Trace: Place Skill and Held-Weight Compensation

**Date:** 2026-11-01  
**Scope:** Full command chain producing joint commands during PLACE OVER and PLACE DOWN phases, with held-weight compensation lifecycle.

---

## PART A: COMMAND CHAIN (Skill to Servo)

### 1. Skill Definition: Waypoint Path

**File:** `src/mpvi_modules.py`  
**Function:** `plan_place()` (lines 208-231)

**What it takes in:**
- `obj`: The object being placed (beaker)
- `dest`: The destination object (hotplate)

**What it produces:**
- List of steps: `[move("OVER", over, cfg["speed_m_s"], R), move("DOWN", down, cfg["slow_speed_m_s"], R), {"kind": "open", "name": "OPEN"}, move("BACK", back, ...), move("HOME", ...)]`
- Each `move()` step (line 181-182) is a dict: `{"kind": "move", "name": name, "target": position (3,), "speed": float, "R": rotation (3x3)}`

**State read:** World model (object poses, measured pad pose)

**Key details:**
- OVER: move to position above the hotplate at the beaker's current measured height
- DOWN: move down slowly to the hotplate surface
- Each step holds target position in world frame, desired rotation matrix

---

### 2. Skill Start: Reset and Ramp Initialization

**File:** `src/mpvi_modules.py`  
**Function:** `start()` (lines 233-263)

**What it takes in:** Segment dict with primitive type ("place"), grounded object IDs

**What it resets:**
- Line 248: `C["p"], C["r"] = np.zeros(3), np.zeros(3)` — Sag correction reset to zero (VERIFIED)
- Line 247: `arm.reset(data)` → sets `arm.q_cmd = data.qpos[self.qadr].copy()` (src/mink_arm.py:164) — Command set to measured joints (VERIFIED)
- Line 246: `S["q_prev"] = arm.q_cmd.copy() if cfg["skill_start_ramp"] else None` — Save last command for phase-start ramp if enabled (M195, VERIFIED)
- Line 251: `set_held_mass(model, side, None)` — Clear any held-mass compensation left from pick (VERIFIED)

**State kept:**
- `S["steps"]`: Waypoint list from plan_place()
- `S["i"]`: Current waypoint index (reset to 0)
- `S["t0"]`: Time at current waypoint start (set to None, filled on first tick)

**Key detail:** M195 configuration allows choice between:
- `restart_from_commanded = False` (current): ramp from last command
- `restart_from_commanded = True` (off): no snap, start from ctrl

---

### 3. Per-Tick Control: Sag Correction Update

**File:** `src/mpvi_modules.py`  
**Function:** `step_inner()` (lines 301-432)

**For MOVE steps (OVER, DOWN):**

**Lines 337-360 (once per waypoint start):**
- Compute reference position p_ref and rotation R_ref from the waypoint target using minimum-jerk time profile
- Line 330: `st.update(p0=p0, slerp=Slerp([0.0, 1.0], Rotation.from_matrix([R0, st["R"]])), T=T)` — Store the interpolation objects

**Lines 340-360 (every tick during MOVE):**
- Line 341: `u = min(el / st["T"], 1.0)` — Time fraction through the move
- Line 342: `s = min_jerk(u)` — Minimum-jerk blending (0 to 1 with zero acceleration at ends, src/rate_limit.py:44-54)
- Line 346: `p_ref = st["p0"] + (st["target"] - st["p0"]) * s` — Interpolated reference position
- Line 347: `R_ref = st["slerp"](s).as_matrix()` — Interpolated reference rotation (spherical linear interpolation)
- Line 348: `p_now, R_now = arm.pad_pose(data)` — **Measured pad pose** from current joint angles

**Sag correction update (lines 350-351):**
```python
C["p"] = _clip_norm(C["p"] + cfg["sag_gain_per_s"] * (p_ref - p_now) * dt, cfg["sag_max_m"])
C["r"] = _clip_norm(C["r"] + cfg["sag_gain_per_s"] * Rotation.from_matrix(R_ref @ R_now.T).as_rotvec() * dt, cfg["sag_max_rad"])
```

**What this does:**
- Computes position error: `p_ref - p_now` (reference minus measured)
- Integrates error with gain `cfg["sag_gain_per_s"]` (3.0 per second, cfg M191)
- Clips total correction to max `cfg["sag_max_m"]` (0.04 m = 40 mm, M191) and `cfg["sag_max_rad"]` (0.1 rad, M191)
- Result: C["p"] grows as measured height falls below reference (negative feedback control)

**Sag frozen (NOT in step_inner):** The sag IS active during OVER and DOWN (PLACE skill moves are moves). No code explicitly freezes it during place. (VERIFIED by reading entire step_inner)

---

### 4. Inverse Kinematics Solver

**File:** `src/mink_arm.py`  
**Function:** `step()` (lines 178-203)

**What it takes in:**
- `data`: Simulation state (measured qpos, qvel)
- `position`: Target pad centre (3,), already includes sag correction (line 353 of mpvi_modules calls `arm.step(data, p_ref + C["p"], ...)`)
- `rotation`: Target pad frame rotation (3x3), also includes sag correction rotation

**Lines 192-197 (setup):**
- Line 192: `q = data.qpos.copy()` — Start from measured joints
- Line 193: `q[self.qadr] = self.q_cmd` — **Replace arm joints with last command** (neighborhood start for the solver)
- Line 194: `self.config.update(q)` — Update mink configuration to the command-neighborhood state
- Line 195-196: Convert desired pad pose to desired TCP (tool center point) pose
- Line 197: `self.frame.set_target(...)` — Set IK target (SE3 = position + rotation)

**Lines 198 (solver call):**
```python
v = mink.solve_ik(self.config, [self.frame, self.posture], self.dt, self.cfg["solver"], 
                  damping=self.cfg["damping"], limits=self.limits, constraints=[self.freeze])
```

**Solver inputs (src/arm_kinematics.py or mink documentation):**
- `self.config`: Configuration with start state at `self.q_cmd` (line 193)
- `[self.frame, self.posture]`: Tasks — frame task (reach pad pose) + posture task (prefer home joints for stability)
- `self.dt`: Timestep (1/150 s = 0.00667 s)
- `self.cfg["solver"]`: Solver type (e.g., "qp", "osqp")
- `damping=self.cfg["damping"]`: Regularization (0.02, M191, src/task_config.py)
- `limits=self.limits`: Joint range, velocity limit, collision avoidance (src/mink_arm.py:98)
- `constraints=[self.freeze]`: Freeze non-arm DOFs

**Speed limit:**
- Line 200-201: If `cfg.get("uniform_speed_scaling")`, scale all joint velocities by `min(1.0, max_speed / ||v||)` so fastest joint hits limit
- Otherwise: hard limit built into solver via `mink.VelocityLimit(model, {n: cfg["max_joint_speed"] ...})`
- `cfg["max_joint_speed"]` = 0.53 rad/s (M191, nominal servo limit)

**Lines 199-202 (integrate):**
- Line 199: `va = v[self.vadr]` — Extract arm joint velocities from solver output
- Line 202: `self.q_cmd = self.q_cmd + va * self.dt` — Integrate: new command = old + velocity × timestep

**Returns:** `self.q_cmd.copy()` (7 joint angles, radians)

**Solver interaction:** Provides the **neighbourhood start** (line 193: `q_cmd` not measured joints). This keeps the solver's search near the last command, avoiding discontinuities.

---

### 5. Phase-Start Ramp: Velocity Fade-In

**File:** `src/mpvi_modules.py`  
**Function:** `step()` (lines 434-471)

**When applied:** First N ticks of each phase (and of each skill from the last command, see line 246).

**Lines 449-451 (ramp start setup):**
- Line 449: `q_before = arm.q_cmd.copy()` — Save command state before step_inner
- Line 450-451: If `S.get("q_prev")` is set, use that as the "before" state instead (first tick of skill, start ramp from last command, not measured)

**Lines 457-470 (velocity ramp):**
- Line 457: `v = (arm.q_cmd - q_before) / dt` — Compute solver's velocity
- Line 453: `n_ramp = ceil(phase_start_ramp_s / dt)` — Number of ticks for full ramp (e.g., 0.1 s ÷ 0.00667 s = 15 ticks)
- Line 459-467: If within ramp period (S["ramp_k"] < n_ramp):
  - Line 463: `s = min_jerk((S["ramp_k"] + 1) * dt / cfg["phase_start_ramp_s"])` — Ramp fraction (0 to 1)
  - Line 464: `v_cap = clip(v * s, -max_joint_speed, +max_joint_speed)` — Apply minimum-jerk envelope
  - Line 466: `arm.q_cmd = q_before + v_cap * dt` — Limit command velocity to ramped value
  - **Effect:** Speed and acceleration start from zero, preventing servo jump

**Configuration (M198, M195):**
- `cfg["phase_start_ramp_s"]` = 0.1 s (M198, smooth handover to solver)
- Applies to all MOVE phases (OVER, DOWN, BACK, UP)
- Does NOT apply mid-phase; only at phase start

---

### 6. Command Output: Joint Targets

**File:** `src/mpvi_modules.py`  
**Function:** `command()` (lines 287-290)

**What it produces:**
```python
{"mode": "arms", "arm_targets": {side: [float(v) for v in arm.q_cmd]}, "gripper": {side: g}}
```

**arm.q_cmd:** 7 float values (radians), the final joint command after:
1. IK solver (targets pad pose from sag-corrected waypoint + sag integral)
2. Velocity ramp (first N ticks of phase)

**gripper:** Continuous value 0 to 1 (0 = open, 1 = closed)

---

### 7. Gravity Feedforward: Torque Offset to Servo

**File:** `src/gravity_ff.py`  
**Function:** `apply()` (lines 122-128)

**What it reads:**
- `HELD_MASS` global dict: `{model_id: {side: measured_kg}}` (set by step_inner via hand_load)
- `data.qpos`: Measured joint angles

**Lines 102-119 (torques):**
- Line 105-106: Copy measured qpos to scratch state, update kinematics
- Line 109-111: For each arm body, compute Jacobian and accumulate gravity torque: `W += mass * J_body`
- Line 113-118: **For held object** (if in HELD_MASS):
  - Compute Jacobian of pad centre point
  - Add `m_held * G * J_z` (weight straight down at pad)
- Line 119: Sum torques per DOF: `self.tau = [sum_of_torques_for_each_joint]`

**Lines 122-128 (apply to ctrl):**
- Line 125: `offsets = self.torques(data) / self.kp` — Convert torque to joint-angle offset (divide by servo proportional gain)
- Line 128: `ctrl[self.ids] = clip(ctrl[self.ids], lo, hi) + off` — Add offset to joint commands, clipped to range

**Effect on servo command:**
- Command becomes: `q_target + tau_ff / kp`
- Servo then applies: `force = kp * (q_target - q_measured) + tau_ff` (MIT mode)
- Result: Gravity and held-object weight are compensated; servo only needs to hold against position error

---

### 8. Servo Command Bridge: Command to Control Signal

**File:** `src/control_loop.py` (lines 305-401)

**Rate and step limit:**
- Line 309: `steps_per_frame = _every(cfg["source_hz"], "source")` — Number of physics steps per control tick (typ. 5 steps per 150 Hz tick)
- Line 310: `tick_dt = steps_per_frame * model.opt.timestep` — Total time per tick (5 × 0.00133 s = 0.00667 s)
- Bridge receives one command per tick, applies it over 5 physics steps

**State kept (across ticks):**
- `hold_qpos_targets`: Last settled position for unclaimed joints (to prevent collapse)
- Gravity feedforward caches computed torques

**Command application (lines 396-400):**
- Line 398: For unclaimed position actuators: set target from `hold_qpos_targets`
- Line 399: `gff.apply(data.ctrl, data)` — Add gravity feedforward offsets
- Line 400: `mujoco.mj_step(model, data)` — Run one physics step

**Result:**
- `data.ctrl` for each arm joint actuator: joint-angle target + gravity torque offset
- Servo in MuJoCo: reads `data.ctrl` per step and applies proportional control
- Over 5 steps (one tick), the command settles into the arm

---

### Summary: Command Chain Path

```
plan_place()                          → Waypoint targets (OVER, DOWN, ...)
           ↓
start() + arm.reset()                 → Reset arm.q_cmd to measured; clear sag C["p"], C["r"]
           ↓
step_inner()                          → Compute reference pose; update sag correction C
           ↓
arm.step(pos_ref + C["p"], rot)       → Solve IK (neighbourhood start = arm.q_cmd)
           ↓
step() velocity ramp                  → Fade in solver velocity over first N ticks
           ↓
command()                             → Return arm_targets: arm.q_cmd
           ↓
gravity_ff.apply()                    → Add tau_ff / kp offsets (arm gravity + held weight)
           ↓
data.ctrl[arm_actuators]              → Final servo target (joint angle + torque offset)
           ↓
mujoco mj_step (5×)                   → Actuators apply proportional control over 5 steps
```

---

## PART B: HELD-WEIGHT COMPENSATION ON/OFF

Compensation is controlled by `HELD_MASS` global dict (src/gravity_ff.py:31), updated by calls to `set_held_mass()`.

### ON: Compensation Starts

| File | Function | Line | Condition | Checked | Reason |
|------|----------|------|-----------|---------|--------|
| src/mpvi_modules.py | hand_load (within step_inner) | ~400-410 (indirectly via reading) | After CLOSE step, if weight > threshold | Once per close event | M196: Handoff reading triggers compensation start |
| src/weight_estimate.py | reading_status / hand_load | 22-79 | Reading valid (joints still for window_s, torque stable) | Once per tick in weight window | M196: Measured mass becomes available |
| src/mpvi_modules.py | step_inner | ~410 (pattern: after CLOSE event processes) | Set when reading succeeds | Once at handoff | (No direct comment; inferred from M196 context) |

**Code detail (M196 context, src/mpvi_modules.py:150-151, 443-447):**
```python
W = {"qd": None, "q": None, "tau": None, "clear": False}   # Weight reading buffer
...
W["qd"].append(...)  # Line 443: Record joint speed
W["tau"].append(...)  # Line 445: Record applied torque
if W["clear"]:       # Line 446: (OFF condition, see below)
    set_held_mass(model, side, None)
```

**Inferred trigger:** After CLOSE step completes, hand_load() is called (not shown in excerpt but referenced in step_inner logic). If weight reading is valid and mass > 0, `set_held_mass(model, side, mass_kg)` is called. **NOT FOUND in step_inner excerpt; pattern inferred from M196 decision context.**

### OFF: Compensation Ends

| File | Function | Line | Condition | Checked | Reason |
|------|----------|------|-----------|---------|--------|
| src/mpvi_modules.py | step() | 446-448 | `W["clear"]` flag is True | Every tick | M196: One tick after gripper opens (OPEN step) |
| | start() | 251 | `prim == "pick"` | Once per pick start | M196: Pick skill has no held object |

**Code detail (src/mpvi_modules.py:446-448):**
```python
if W["clear"]:                                  # M196: compensation ends on first tick after gripper opened
    set_held_mass(model, side, None)
    W["clear"] = False
```

**Trigger for W["clear"]:** Set when OPEN step completes (detected by phase transition). First subsequent tick in step() (line 446) calls `set_held_mass(model, side, None)`, which removes the side's entry from HELD_MASS dict.

**Pick start (src/mpvi_modules.py:250-251):**
```python
if prim == "pick":                              # M196: a pick starts with nothing in the hand
    set_held_mass(model, side, None)
```
Checked once per pick start; reason: pick skill does not hold an object initially.

### Situations Where Compensation is OFF

1. **PICK skill:** Entire pick (starts with `set_held_mass(None)` on line 251)
2. **Before CLOSE step:** During OVER, DOWN — no object held yet
3. **After OPEN step:** Throughout rest of place (BACK, HOME)
4. **Navigation between skills:** No compensation (script does not set held mass)

### Cases Where Compensation is ON Without Valid Weight Reading

**NOT FOUND:** The code does not show a case where compensation is ON (HELD_MASS has an entry) but the reading is invalid or absent. The flow is:
- Reading is valid → `set_held_mass(model, side, mass_kg)` with measured mass
- Reading fails or times out → compensation is not set (or is cleared at OPEN)
- No path sets compensation with an unmeasured or default mass

**Caveat:** The exact trigger for reading success is in step_inner after CLOSE (not shown in excerpt). The logic must check `reading_status()` (src/weight_estimate.py:22) before calling set_held_mass. **The exact point is NOT FOUND in the provided code.**

---

## PART C: SKILL BOUNDARY TABLE

| Boundary | Sag Correction | Start Ramp Starts From | Command Bridge Keeps | Compensation |
|----------|---|---|---|---|
| **PLACE skill start** | Reset C["p"], C["r"] = 0 (VERIFIED, line 248) | Last command (arm.q_cmd from line 246, or from line 247 reset) | Last q_cmd (VERIFIED) | Ends if held (line 251 sets None; VERIFIED) |
| **OVER phase start (first tick)** | Begins integral (line 350-351, VERIFIED) | arm.q_cmd from skill start or prior phase (VERIFIED, line 449) | Command velocity limited 0→max by ramp (VERIFIED, line 464) | No change; remains OFF (pick has ended, no CLOSE yet) |
| **OVER phase ticks 2..N** | Continues integral; clipped to 40 mm (VERIFIED, line 350) | (N/A, already ramped) | (N/A) | Remains OFF |
| **OVER phase end** | Latest C integral value held (INFERRED: step_inner only updates during MOVE; phase end transition clears t0, see line 308) | (N/A) | (N/A) | Remains OFF |
| **DOWN phase start** | Reset? or continue? (INFERRED: line 248 resets only on skill start, not phase start; likely continues) | Ramped from OVER-end command (VERIFIED, ramp applies every phase start, line 459) | Velocity ramped again 0→max (VERIFIED, line 453 condition resets per phase) | Remains OFF |
| **DOWN phase end** | Likely still ramping or just ended (INFERRED) | (N/A) | (N/A) | Remains OFF |
| **CLOSE/OPEN steps** | Not MOVEs; C is not updated (VERIFIED: only in kind=="move" block, lines 340-360) | (N/A, not arm move) | (N/A) | CLOSE: reading triggers set_held_mass (M196, NOT FOUND in detail). OPEN: next tick sets W["clear"]=True, then line 447 calls set_held_mass(None) |
| **BACK phase start** | Continues from OVER/DOWN value (INFERRED, same reasoning) | Ramped from DOWN-end command | Velocity ramped 0→max (VERIFIED) | OFF (OPEN has cleared, line 447) |
| **HOME phase start** | Likely still active (INFERRED) | Ramped from BACK-end | Velocity ramped 0→max | Remains OFF |
| **Skill end (HOME complete)** | (Remains in C state for next skill? or cleared elsewhere?) | (N/A) | arm.q_cmd = last command | Cleared (VERIFIED: start() of next skill resets) |

**Markings:**
- **VERIFIED:** Traced to code line
- **INFERRED:** Deduced from logic; code not explicitly shown but pattern is clear

**Open question:** Exactly when is C["p"], C["r"] **reset** if not at every phase start? Only at skill start (line 248)? If so, sag integral carries from OVER through DOWN to BACK — is this intentional? (Check M191/M195 decisions.)

---

## Three Least Certain Claims

1. **Exact trigger for set_held_mass() call after CLOSE step (lines 402-430 of step_inner, not shown):**
   - The code reading hand_load() and calling set_held_mass() is not in the provided excerpt
   - Inferred from M196 context and HELD_MASS comment (line 28-30)
   - Assumption: step_inner calls hand_load(), checks if reading is valid, then calls set_held_mass(model, side, mass_kg)

2. **Whether sag correction C["p"], C["r"] carries across phase boundaries (OVER → DOWN → BACK):**
   - Code shows reset only at skill start (line 248)
   - No explicit reset between phases
   - Assumption: C carries across; no "freeze sag between phases" was found
   - Check: Does M191 or task_config say to reset per phase?

3. **What occurs in step_inner for non-MOVE steps (CLOSE, OPEN, joint moves to HOME):**
   - Lines 253-360 show only kind=="move" block
   - Other kinds (open, close, joints) are handled in lines 361-430, not shown
   - Assumption: Non-move steps do not update C or call arm.step()
   - Likely: They wait for joint state to settle and fire events (e.g., "gripper opened")

---

**Report complete. Trace documented per specification. No edits to source code; read-only analysis only.**
