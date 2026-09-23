# Session log

## 2026-09-23 — Shared teleop environment, MuJoCo 3.13.0 upgrade, version comparison

Two working sessions. The first (about 17:53–18:39 UTC) ended when the VM ran
out of memory and crashed. The second (from 18:56) resumed the work.

### Decisions
- **One environment shared with teleop.** The project gets its own `.venv`
  (uv, Python 3.12), separate from the earlier test environment. It uses the
  versions locked by `openarm_teleop`, exported with
  `uv export --frozen --no-hashes --no-emit-project --no-emit-local --extra test`.
  The teleop tested lock is not to be changed ("keep in mind im not changing
  the tele-op tested lock").
- **Python 3.12.** The VM's system Python is 3.14, but the teleop lock pins
  numpy 2.2.6 and scipy 1.15.3, which only have packages up to Python 3.13.
  With the lock fixed, that rules out 3.14. Within 3.10–3.13, 3.12 was picked
  as a conservative default, not for a technical reason; 3.13 would also
  work. (The pre-crash session said the README uses 3.12. It doesn't.)
- **Upgrade to MuJoCo 3.13.0** (from 3.3.4), because the teleop lock pins
  3.13.0 and both projects share one environment. This was on condition that
  we confirm everything still works on it (see the comparison below).
- **The 3.3.4 comparison environment pins numpy 2.2.6** to match the project,
  so MuJoCo is the only thing that differs between versions.
- **geo_kin is installed directly from the wheel**, because the provisioner
  rejects the openarm product (see Problems).
- **Keep commits separate:** environment, `.gitignore` and version changes go
  in one commit (d8179d6), controller work in others.
- **Compare versions before debugging the controller:** settle the
  3.3.4 vs 3.13.0 question before touching the arm-swing bug.
- **Push only to `origin`** (github.com/dasiff). The gatech mirror is not
  maintained. Commits from the VM use the repo-local author
  `David Siff <dasiff@gmail.com>`. The VM has no stored GitHub credentials.
- **Run MuJoCo jobs one at a time on this VM** (see Problems).

### Versions installed
| Where | What |
|---|---|
| `~/.local/bin` | uv 0.12.18. The system Python is 3.14 only, and curl isn't installed. |
| `openarm-tracer-sim/.venv` (Python 3.12.14) | mujoco 3.13.0, numpy 2.2.6, scipy 1.15.3, pandas 3.0.6, pytest 9.1.1 and the rest of the teleop lock; geo_kin 0.1.0 (licensed wheel); editable installs of `openarm_teleop`, `geo_kin_core` and `XRT_devices`. The activate script sets the geo_kin license variable. |
| `requirements.txt` | `mujoco==3.13.0` (was 3.3.4) |
| `~/projects/openarm_teleop` | Cloned at 66b5e85, with submodules `geo_kin_core` at 765738f and `XRT_devices` at f364006. Its `uv.lock` is untouched. |
| Throwaway environment | MuJoCo 3.3.4 with numpy 2.2.6, used only for the comparison. Not part of the project. |

### What was checked on 3.13.0
- **Passing:**
  - The geo_kin install check passes for openarm, and headless replay works.
  - `run_basic_sim`, `test_controller`, `empty_room_joint_test` (460 rows),
    `workspace_sweep` (78,125 poses) and `visualize_workspace` all pass.
- **Teleop test suite:** 10 passed, 1 failed (the provisioner issue below).
- **Scenes:** all three compile on both versions with the same model size.
- **Not tested:** the viewer scripts, which need a display.

### MuJoCo 3.3.4 vs 3.13.0: no physics regression
- **Zero control:** with no control applied, the arms ended up to 3.5 rad
  apart between versions after 2000 steps. With nothing holding them, that
  can't tell a regression apart from drift.
- **Held-pose rerun:** we reran it the way `experiments/combined_controller.py`
  runs with no keys pressed:
  - spawn at `[-1.0, 3.5, 0.22]`
  - 500-step warmup, then lock in the resting pose and re-settle
  - 2000 steps holding the pose
- **Runs:** `chemistry_lab`, `single_block` and `chemistry_lab_tracer` (Tracer
  base only, no arms), on both versions. Each was run as-is and with a 1e-9
  nudge to the starting joint angles (to the base position in the Tracer-only
  scene).
- **Results:**
  - Both versions lock in the same resting pose, agreeing to within 0.0001 rad.
  - Tracer-only scene: the versions end within 0.04 mm (base) and 3 mm (lab
    objects) of each other.
  - Full-robot scenes: final arm angles differ by 2.3–3.1 rad between versions.
    But a 1e-9 nudge within a *single* version changes them by 0.5–2.9 rad.
    The version gap is no bigger than the nudge gap. That is drift that keeps
    growing, not a change in the physics.
- **Decision:** we're staying on 3.13.0.

### Problems found
1. **Out-of-memory crash.** Each full-robot MuJoCo process peaks at about
   1.8–2.0 GB of RAM. Running five at once caused an out-of-memory kill
   (exit 137) on this 8 GB VM with no swap, and crashed the machine.
2. **Robot starts inside the furniture** at the scenes' default position:
   18 cm into the bench in `chemistry_lab`, 3–4 cm into the workbench in
   `single_block`. Results from that position are invalid. Use the
   controller's spawn position.
3. **Gripper fingers overlap** by 3.5 mm at the zero pose, in the model
   itself. Not addressed.
4. **Scenes won't load from relative paths.** Mesh paths resolve against the
   current directory, not the scene file. This happens on both versions, so
   it predates the upgrade. Workaround: load scenes by absolute path.
5. **The provisioner rejects the openarm product** ("product must begin with
   g1, rby1, or vega"), both at the pinned commit and upstream. This is the
   one failing teleop test. We worked around it with a direct wheel install;
   the upstream fix is still pending.
6. **The arm hold is weak even without collisions.** With a clear start and no
   keys pressed, the held arm joints still drift 1.6–2.0 rad from their locked
   targets in the main loop. They hold fine during warmup and re-settle, which
   apply control every physics step. The main loop updates control only once
   per rendered frame (about every 8 steps) and overwrites the base velocity
   every step. Either could be the cause; neither has been tested. The model's
   mixed actuator settings (gain and bias combinations differ across
   actuators) are another candidate.

### Still open
- The arm-swing-when-base-moves bug in `combined_controller.py`. Items 6 and 3
  above are the leads. Not started.
- Connect `q_goal_right/left` from geo_kin's `session.solve()` to the arm-hold
  PD in `combined_controller.py`. This was the original goal and comes after
  the bug fix.
- Report the provisioner issue upstream.
- The check for Windows-side commits that never reached GitHub wasn't
  finished. A fetch showed nothing new on origin, and `combined_controller.py`
  is in 088104b.
