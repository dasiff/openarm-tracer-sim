"""Test of the held-mass ramp (rate and target ramps), the freeze of the load change, the lift-off weighing functions, and the trim's one guard
(src/held_mass_ramp.py, src/weight_estimate.py, src/gravity_ff.py, src/mpvi_modules._sag_trim_step; decisions-log M217 to M219, M222, M226, M235 to M239).
Exits 0 on pass. No simulation is stepped (the hand height check reads a model and does forward kinematics only).

    MUJOCO_GL=egl python experiments/held_mass_ramp_test.py

Checks, with a stand-in for the model (the ramp is keyed by id(model) and never reads it):
  1. no ramp: mass 0, finished at once
  2. target ramp in: minimum-jerk shape, from 0 to 1.0 kg, finished at the end
  3. retarget while running: restarts from the current mass over the full duration and prints one line
  4. the same target again: nothing changes and nothing is printed
  5. target 0 with mass 0: finished at once
  6. the two arms are independent (keyed by side)
  7. reset clears every ramp of the model
  8. the freeze flag (M226): not changing with no load (8a); changing during a target ramp in (8e); settled at its target (8f)
  9. a retarget mid-ramp: still changing until the new target is reached (9a, 9b)
 10. a pick start with no held load does not freeze (10)
 11. the flag: a ramp out holds it until end_load_change (11a, 11b)
 12. the ramp out of a held mass: changing during the ramp, still changing after it, ends at end_load_change (12a to 12c)
 13. the trim (M238): it does not integrate while a ramp runs (a target ramp out, 13a), and it integrates again after the ramp (13b)
 14. the rate mode (M236): the mass rises linearly at rate/g (14a), has no steps (14b), stop holds the value (14c), and the retarget after a stop is minimum jerk (14d)
 15. the lift-off detection (M235): no trigger below the threshold (15a), none for a rise shorter than the hold count (15b), trigger at the threshold (15c),
     and the count restarts after a drop (15d)
 16. the estimate (M236): mass at detection minus the rise divided by the hand height per kilogram (16a); a non-positive hand height raises (16b)
 17. the hand height per kilogram (M236): for 20 random poses, the joint offsets for 1 kg, applied to the joints and run through forward kinematics,
     give the pad z movement; the largest difference from held_pad_z_per_kg is printed as INFO (17), and the value is checked positive (17)
 18. the trim does not change during a rate ramp, and a reset clears its ramp (18a, 18b)
 19. the trim helper names no phase, skill, gripper or object (19)
"""

import contextlib
import inspect
import io
import sys

sys.path.insert(0, "/home/vboxuser/projects/openarm-tracer-sim")

import mujoco                                         # noqa: E402
import numpy as np                                    # noqa: E402

from src import held_mass_ramp as hmr                # noqa: E402
from src.gravity_ff import ArmGravityFeedforward, held_pad_z_per_kg  # noqa: E402
from src.mpvi_modules import _sag_trim_step           # noqa: E402
from src.rate_limit import min_jerk                  # noqa: E402
from src.run_record import load_recording            # noqa: E402
from src.replay import build_model                   # noqa: E402
from src.task_config import PICK_PLACE_CFG           # noqa: E402
from src.weight_estimate import G, liftoff_check, liftoff_estimate  # noqa: E402

DUR = PICK_PLACE_CFG["held_mass_ramp_s"]
CFG = PICK_PLACE_CFG
fails = []
quiet = io.StringIO()


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  ({detail})" if detail and not ok else ""))
    if not ok:
        fails.append(name)


class Model:
    """A stand-in for the compiled model: the ramp is keyed by id(model) only."""


def ramp_in(m, side, target, t0, tick=0):
    with contextlib.redirect_stdout(quiet):
        hmr.set_target(m, side, target, t0, tick)


# ---- 1 to 7: the ramp module ----
m = Model()
hmr.reset(m)
check("1. no ramp: mass 0 and finished", hmr.mass_at(m, "left", 0.3) == 0.0 and hmr.finished(m, "left", 0.3))

ramp_in(m, "left", 1.0, 0.0)
half = hmr.mass_at(m, "left", DUR / 2)
check("2a. target ramp in: half way is the minimum-jerk midpoint", abs(half - min_jerk(0.5)) < 1e-12, f"{half}")
check("2b. target ramp in: not finished before the duration", not hmr.finished(m, "left", DUR * 0.9))
check("2c. target ramp in: at the duration, mass 1.0 and finished", abs(hmr.mass_at(m, "left", DUR) - 1.0) < 1e-12 and hmr.finished(m, "left", DUR))

buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    hmr.set_target(m, "left", 0.0, 0.2, 30)          # retarget while running
lines = buf.getvalue().strip().splitlines()
mid = hmr.mass_at(m, "left", 0.2)
check("3a. retarget while running prints one line with the tick and both targets",
      len(lines) == 1 and "tick 30" in lines[0] and "old target 1.000 kg" in lines[0] and "new target 0.000 kg" in lines[0], f"{lines}")
check("3b. retarget restarts from the current mass over the full duration", abs(mid - min_jerk(0.2)) < 1e-12 and not hmr.finished(m, "left", 0.2 + DUR * 0.5))
check("3c. the retargeted ramp ends at 0 after the full duration", abs(hmr.mass_at(m, "left", 0.2 + DUR) - 0.0) < 1e-12 and hmr.finished(m, "left", 0.2 + DUR))

buf = io.StringIO()
before = dict(hmr.RAMP[id(m)]["left"])
with contextlib.redirect_stdout(buf):
    hmr.set_target(m, "left", 0.0, 0.5, 31)          # same target as the running ramp
check("4. same target: nothing changes, nothing printed", buf.getvalue() == "" and hmr.RAMP[id(m)]["left"]["t0"] == before["t0"])

ramp_in(m, "right", 0.0, 1.0, 40)                    # mass 0, target 0: finished at once
check("5. target 0 with mass 0: finished at once", hmr.finished(m, "right", 1.0) and hmr.mass_at(m, "right", 1.0) == 0.0)

ramp_in(m, "left", 0.8, 2.0, 50)
check("6. the arms are independent", hmr.mass_at(m, "right", 2.5) == 0.0 and abs(hmr.mass_at(m, "left", 2.0 + DUR) - 0.8) < 1e-12)

hmr.reset(m)
check("7. reset clears every ramp of the model", hmr.mass_at(m, "left", 5.0) == 0.0 and hmr.finished(m, "right", 5.0))

# ---- 8 to 12: the freeze flag (M226) ----
m2 = Model()
hmr.reset(m2)
check("8a. no load and no ramp: not changing", not hmr.is_load_changing(m2, "left", 0.0))
ramp_in(m2, "left", 0.9, 2.0, 60)
check("8e. target ramp in: changing during the ramp", hmr.is_load_changing(m2, "left", 2.0 + DUR * 0.5))
check("8f. target ramp in: settled at the end of the ramp (not changing)", not hmr.is_load_changing(m2, "left", 2.0 + DUR))
ramp_in(m2, "left", 0.0, 4.0, 70)                    # place's DOWN ends: ramp out of the held mass
check("12a. ramp out of a held mass: changing during the ramp", hmr.is_load_changing(m2, "left", 4.0 + DUR * 0.5))
check("12b. ramp out: still changing after the ramp (until the gripper opens)", hmr.is_load_changing(m2, "left", 4.0 + DUR + 1.0))
hmr.end_load_change(m2, "left")                      # the gripper-open event
check("12c. gripper-open event ends the freeze", not hmr.is_load_changing(m2, "left", 6.0))

m3 = Model()
hmr.reset(m3)
ramp_in(m3, "left", 1.0, 1.0, 80)
ramp_in(m3, "left", 0.4, 1.0 + DUR * 0.5, 81)        # retarget mid-ramp
check("9a. retarget mid-ramp: still changing", hmr.is_load_changing(m3, "left", 1.0 + DUR * 0.5 + DUR * 0.5))
check("9b. retarget mid-ramp: not changing once the new target is reached", not hmr.is_load_changing(m3, "left", 1.0 + DUR * 0.5 + DUR))

m4 = Model()
hmr.reset(m4)
ramp_in(m4, "left", 0.0, 0.0, 1)                     # pick start, hand empty
check("10. pick start with no held load: not changing (the approach learns)", not hmr.is_load_changing(m4, "left", 0.5))

m5 = Model()
hmr.reset(m5)
ramp_in(m5, "left", 0.2, 0.0, 90)
ramp_in(m5, "left", 0.0, 2.0, 91)                    # ramp out: changing
hmr.reset(m5)
check("11a. reset clears the freeze", not hmr.is_load_changing(m5, "left", 2.5))
ramp_in(m5, "left", 0.2, 3.0, 92)
ramp_in(m5, "left", 0.0, 5.0, 93)
check("11b0. after the ramp out, the flag holds the freeze until the gripper opens", hmr.is_load_changing(m5, "left", 6.5))
hmr.end_load_change(m5, "left")
check("11b. end_load_change clears the flag", not hmr.is_load_changing(m5, "left", 6.5))


def trim_state(p, r):
    return {"p": np.array(p, float), "r": np.array(r, float)}


P_ERR = np.array([0.001, 0.0005, -0.0008])
R_ERR = np.array([0.002, -0.001, 0.0015])
CFG_DT = 0.01

# 13: the trim does not integrate while a ramp runs (M238); it integrates again after the ramp
m6 = Model()
hmr.reset(m6)
ramp_in(m6, "left", 0.2, 0.0, 100)
ramp_in(m6, "left", 0.0, 2.0, 101)
C6 = trim_state([0.010, -0.004, 0.017], [0.002, 0.001, -0.003])
before6 = {k: v.copy() for k, v in C6.items()}
for t in np.arange(2.0, 2.0 + DUR - 0.01, 0.05):
    _sag_trim_step(C6, CFG, m6, "left", float(t), P_ERR, R_ERR, CFG_DT)
check("13a. during a ramp out the trim does not change (position and rotation)", all(np.array_equal(C6[k], before6[k]) for k in C6), f"{C6}")
_sag_trim_step(C6, CFG, m6, "left", 2.0 + DUR + 0.5, P_ERR, R_ERR, CFG_DT)
check("13b. after the ramp the trim integrates again (no release by the gripper needed)",
      not np.array_equal(C6["p"], before6["p"]) and not np.array_equal(C6["r"], before6["r"]))

# 14: the rate mode (M236)
RATE = 0.5 / G                                       # kg/s for 0.5 N/s
m7 = Model()
hmr.reset(m7)
with contextlib.redirect_stdout(quiet):
    hmr.start_rate(m7, "left", RATE, 0.0, 200)
ts = np.linspace(0.0, 2.0, 2001)
ms = np.array([hmr.mass_at(m7, "left", float(t)) for t in ts])
lin_err = float(np.abs(ms - RATE * ts).max())
check("14a. the mass rises linearly at rate/g (0.5 N per s over g)", lin_err < 1e-12, f"max error {lin_err:.3e} kg")
steps = float(np.abs(np.diff(ms)).max())
check("14b. no steps: the largest change over one millisecond step is the rate times the step", steps <= RATE * (ts[1] - ts[0]) + 1e-12, f"{steps}")
check("14c0. the rate ramp never finishes on its own", not hmr.finished(m7, "left", 100.0) and hmr.ramp_running(m7, "left", 100.0))
hmr.stop_rate(m7, "left", 1.0)
m_hold = hmr.mass_at(m7, "left", 1.0)
check("14c. stop holds the value", abs(hmr.mass_at(m7, "left", 5.0) - m_hold) < 1e-12 and abs(m_hold - RATE * 1.0) < 1e-12)
m_det = m_hold
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m7, "left", 0.205, 1.0, 210)
mid_est = min_jerk(0.5) * (0.205 - m_det) + m_det
check("14d. the retarget to the estimate is minimum jerk from the held value over 1.0 s",
      abs(hmr.mass_at(m7, "left", 1.5) - mid_est) < 1e-12 and abs(hmr.mass_at(m7, "left", 2.0) - 0.205) < 1e-12)

# 15: the lift-off detection (M235)
THR = 0.3e-3
st = {"n": 0}
trig = [liftoff_check(st, 0.25e-3, THR, 5) for _ in range(50)]
check("15a. a rise below the threshold never triggers", not any(trig))
st = {"n": 0}
trig = [liftoff_check(st, 0.31e-3, THR, 5) for _ in range(4)]
check("15b. a rise above the threshold but shorter than the hold count does not trigger", not any(trig))
st = {"n": 0}
trig = [liftoff_check(st, 0.3e-3, THR, 5) for _ in range(5)]
check("15c. a rise exactly at the threshold triggers on the hold count (the fifth tick)", trig == [False, False, False, False, True], f"{trig}")
st = {"n": 0}
seq = [0.31e-3, 0.31e-3, 0.0, 0.31e-3, 0.31e-3, 0.31e-3, 0.31e-3, 0.31e-3]
trig = [liftoff_check(st, r, THR, 5) for r in seq]
check("15d. a drop below the threshold restarts the count", trig[-1] and not any(trig[:-1]), f"{trig}")

# 16: the estimate (M236)
check("16a. estimate = mass at detection minus rise over hand height per kilogram",
      abs(liftoff_estimate(0.30, 0.0006, 0.1) - (0.30 - 0.006)) < 1e-12)
raised = False
try:
    liftoff_estimate(0.3, 0.001, 0.0)
except ValueError:
    raised = True
check("16b. a non-positive hand height raises", raised)

# 17: the hand height per kilogram (M236), against forward kinematics on the recorded model (no simulation step)
rec = load_recording("/home/vboxuser/projects/openarm-tracer-sim/data/closed_loop/on_20261029_1453/recording")
model, _d0, _r0 = build_model(rec["meta"])
ff = ArmGravityFeedforward.from_model(model, sides=("left",))
tcp, pad_local = ff.pads["left"][0], ff.pads["left"][1]
rng = np.random.default_rng(7)
qadr = [int(model.jnt_qposadr[model.actuator_trnid[int(a), 0]]) for a in ff.ids[:7]]
dofs = [ff.dofs[k][0] for k in range(7)]
jlo = np.array([model.jnt_range[model.actuator_trnid[int(a), 0]][0] for a in ff.ids[:7]])
jhi = np.array([model.jnt_range[model.actuator_trnid[int(a), 0]][1] for a in ff.ids[:7]])
worst_mm = 0.0
all_pos = True
worst_pred = []
for trial in range(20):
    d = mujoco.MjData(model)
    d.qpos[:] = _d0.qpos[:] if _d0 is not None else d.qpos
    q = jlo + (jhi - jlo) * rng.uniform(0.2, 0.8, 7)
    for k, a in enumerate(qadr):
        d.qpos[a] = q[k]
    mujoco.mj_forward(model, d)
    hpz_pred = held_pad_z_per_kg(model, d, "left")
    point0 = d.xpos[tcp] + d.xmat[tcp].reshape(3, 3) @ pad_local
    z0 = float(point0[2])
    jac = np.zeros((3, model.nv))
    mujoco.mj_jac(model, d, jac, None, point0, tcp)
    d2 = mujoco.MjData(model)
    d2.qpos[:] = d.qpos[:]
    for k, a in enumerate(qadr):
        d2.qpos[a] = q[k] + G * jac[2, dofs[k]] / ff.kp[k]          # the joint offset the 1 kg held torque makes (tau / kp)
    mujoco.mj_forward(model, d2)
    point1 = d2.xpos[tcp] + d2.xmat[tcp].reshape(3, 3) @ pad_local
    moved = float(point1[2] - z0)
    worst_mm = max(worst_mm, 1000.0 * abs(moved - hpz_pred))
    all_pos = all_pos and hpz_pred > 0.0
    worst_pred.append(round(hpz_pred, 4))
    if trial == 0:
        first = (hpz_pred, moved)
print(f"INFO 17. hand height per kilogram: largest difference from forward kinematics over 20 poses = {worst_mm:.3f} mm (the 1 kg offsets are not small for the low-gain joints, so the linear map is approximate)")
check("17. hand height per kilogram is positive at each of the 20 poses (the estimate's sign)", all_pos, f"{worst_pred}")
print(f"   hand height per kilogram (first pose): predicted {first[0]:.5f} m, forward kinematics {first[1]:.5f} m; largest difference over 20 poses {worst_mm:.4f} mm")

# 18: the trim does not change during a rate ramp; a reset clears the ramp (so the trim integrates again)
m8 = Model()
hmr.reset(m8)
with contextlib.redirect_stdout(quiet):
    hmr.start_rate(m8, "left", RATE, 0.0, 300)
C8 = trim_state([0.010, -0.004, 0.017], [0.002, 0.001, -0.003])
before8 = {k: v.copy() for k, v in C8.items()}
for t in np.arange(0.0, 1.0, 0.02):
    _sag_trim_step(C8, CFG, m8, "left", float(t), P_ERR, R_ERR, CFG_DT)
check("18a. the trim does not change during a rate ramp", all(np.array_equal(C8[k], before8[k]) for k in C8))
hmr.reset(m8)
_sag_trim_step(C8, CFG, m8, "left", 1.0, P_ERR, R_ERR, CFG_DT)
check("18b. after a reset the trim integrates again", not np.array_equal(C8["p"], before8["p"]))

# 19: the trim helper names no phase, skill, gripper or object
src_text = inspect.getsource(_sag_trim_step).lower()
bad = [w for w in ("pick", "place", "gripper", "beaker", "object", "skill", "phase", "hotplate") if w in src_text]
check("19. the trim helper has no phase, skill, gripper or object name", not bad, f"found {bad}")

print("\nRESULT:", "PASS" if not fails else f"FAIL ({len(fails)}: {fails})")
sys.exit(1 if fails else 0)
