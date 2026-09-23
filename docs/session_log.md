# Session log

## 2026-09-23 — MuJoCo 3.3.4 vs 3.13.0 held-pose comparison

### Question
An earlier zero-control test showed the arms ending up to 3.5 rad apart
between MuJoCo 3.3.4 and 3.13.0. Was that a real physics regression from the
version bump (d8179d6), or numerical drift that grows when nothing holds the
arms?

### What we ran
The same idle sequence `experiments/combined_controller.py` runs when no keys
are pressed, without the viewer: spawn at `[-1.0, 3.5, 0.22]`, a 500-step PD
warmup, lock in the resting pose, re-settle for 200 steps, then 2000 main-loop
steps holding that pose (base velocity pinned to zero, as the controller does).
We ran each of `chemistry_lab`, `single_block` and `chemistry_lab_tracer`
(Tracer base only, no arms) on both versions. Each was run once as-is and once
with a 1e-9 nudge to the starting joint angles (to the base position in the
Tracer-only scene), 12 runs in total, one at a time.

### What we found
- **No evidence of a physics regression.**
- Both versions lock in the same resting pose, agreeing to within 0.0001 rad.
- Tracer-only scene: the two versions end within 0.04 mm (base) and 3 mm
  (lab objects) of each other.
- Full-robot scenes: final arm angles differ by 2.3–3.1 rad between versions.
  But a 1e-9 rad nudge within a *single* version changes them by 0.5–2.9 rad.
  The version gap is no bigger than the nudge gap. That is drift that keeps
  growing, not a change in the physics.
- A first attempt was invalid: at the scenes' default position the robot
  starts inside the furniture (18 cm into the bench in `chemistry_lab`, 3–4 cm
  into the workbench in `single_block`). Use the controller's spawn position
  for any comparison. The gripper finger pairs also overlap by 3.5 mm in the
  model itself; we left that alone.

### Versions decided on
- Project environment (`.venv`, Python 3.12.14, shared with the teleop
  scripts): **MuJoCo 3.13.0**, numpy 2.2.6, pinned in `requirements.txt`.
  We're staying on it.
- MuJoCo 3.3.4 was installed only in a throwaway environment for this
  comparison. It is not part of the project.

### Observation for later: the hold is weak even without collisions
With a clear start and nothing pressed, the held arm joints still drift
1.6–2.0 rad away from their locked targets during the main loop. They hold
fine during warmup and re-settle, which apply control every physics step.
The main loop updates control only once per rendered frame (about every 8
steps) and overwrites the base velocity every step. One of these may be the
cause, but we haven't tested either. This is the lead for the pending
arm-swing-when-base-moves bug in `combined_controller.py`, which we haven't
started.

### Machine constraint
Each full-robot MuJoCo process peaks at about 1.8–2.0 GB of RAM. Running five
at once caused an out-of-memory kill (exit 137) on this 8 GB VM. Run MuJoCo
jobs one at a time on this machine.
