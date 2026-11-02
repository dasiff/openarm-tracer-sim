"""Test of the held-mass ramp and its freeze (src/held_mass_ramp.py, decisions-log M217 to M219, M222, M226). Exits 0 on pass.

    MUJOCO_GL=egl python experiments/held_mass_ramp_test.py

Checks, with a stand-in for the model (the ramp is keyed by id(model) and never reads it):
  1. no ramp: mass 0, finished at once
  2. ramp in: from 0 to 1.0 kg over the duration, minimum-jerk shape, finished at the end
  3. retarget while running: restarts from the current mass over the full duration and prints one line
  4. the same target again: nothing changes and nothing is printed
  5. target 0 with mass 0: finished at once
  6. the two arms are independent (keyed by side)
  7. reset clears every ramp of the model
  8. the freeze (M226): starts at the CLOSE step's begin, holds through the ramp in, ends when the ramp in reaches its target
  9. the freeze holds during a retarget (restart from the current mass) until the new target is reached
 10. a pick start with no held load does not freeze (the empty hand approach still learns)
 11. reset clears the freeze; end_load_change clears the flag, but a ramp still running stays a change
 12. ramp out of a held mass starts the freeze; it ends at end_load_change (the gripper-open event)
 13. the sag trim step (src/mpvi_modules._sag_trim_step): a frozen step leaves the trim unchanged; an unfrozen step integrates
"""

import contextlib
import io
import sys

sys.path.insert(0, "/home/vboxuser/projects/openarm-tracer-sim")

from src import held_mass_ramp as hmr                # noqa: E402
from src.rate_limit import min_jerk                  # noqa: E402
from src.task_config import PICK_PLACE_CFG           # noqa: E402

DUR = PICK_PLACE_CFG["held_mass_ramp_s"]
fails = []


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  ({detail})" if detail and not ok else ""))
    if not ok:
        fails.append(name)


class Model:
    """A stand-in for the compiled model: the ramp is keyed by id(model) only."""


m = Model()
hmr.reset(m)
check("1. no ramp: mass 0 and finished", hmr.mass_at(m, "left", 0.3) == 0.0 and hmr.finished(m, "left", 0.3))

hmr.set_target(m, "left", 1.0, 0.0, 0)
half = hmr.mass_at(m, "left", DUR / 2)
check("2a. ramp in: half way is the minimum-jerk midpoint", abs(half - min_jerk(0.5)) < 1e-12, f"{half}")
check("2b. ramp in: not finished before the duration", not hmr.finished(m, "left", DUR * 0.9))
check("2c. ramp in: at the duration, mass 1.0 and finished", abs(hmr.mass_at(m, "left", DUR) - 1.0) < 1e-12 and hmr.finished(m, "left", DUR))

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

hmr.set_target(m, "right", 0.0, 1.0, 40)             # mass 0, target 0: finished at once
check("5. target 0 with mass 0: finished at once", hmr.finished(m, "right", 1.0) and hmr.mass_at(m, "right", 1.0) == 0.0)

hmr.set_target(m, "left", 0.8, 2.0, 50)
check("6. the arms are independent", hmr.mass_at(m, "right", 2.5) == 0.0 and abs(hmr.mass_at(m, "left", 2.0 + DUR) - 0.8) < 1e-12)

hmr.reset(m)
check("7. reset clears every ramp of the model", hmr.mass_at(m, "left", 5.0) == 0.0 and hmr.finished(m, "right", 5.0))

# --- the freeze of the sag correction (M226) and the trim step (M225): stand-ins only, no simulation ---
import numpy as np                                    # noqa: E402
from src.mpvi_modules import _sag_trim_step, _trim_handoff_clear   # noqa: E402

quiet = io.StringIO()
m2 = Model()
hmr.reset(m2)
check("8a. no load and no ramp: not changing", not hmr.is_load_changing(m2, "left", 0.0))
hmr.begin_load_change(m2, "left", 1.0)
check("8b. CLOSE step begin: changing at once (the ramp is at 0)", hmr.is_load_changing(m2, "left", 1.0))
check("8c. still changing later while no weighing has set a target", hmr.is_load_changing(m2, "left", 5.0))
check("8d. the other arm is not changing", not hmr.is_load_changing(m2, "right", 5.0))
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m2, "left", 0.9, 2.0, 60)          # the weighing reads 0.9 kg: ramp in
check("8e. ramp in: changing during the ramp", hmr.is_load_changing(m2, "left", 2.0 + DUR * 0.5))
check("8f. ramp in: freeze ends when the ramp reaches its target", not hmr.is_load_changing(m2, "left", 2.0 + DUR))
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m2, "left", 0.0, 4.0, 70)          # place's DOWN ends: ramp out of the held mass
check("12a. ramp out of a held mass: changing during the ramp", hmr.is_load_changing(m2, "left", 4.0 + DUR * 0.5))
check("12b. ramp out: still changing after the ramp (until the gripper opens)", hmr.is_load_changing(m2, "left", 4.0 + DUR + 1.0))
hmr.end_load_change(m2, "left")                       # the gripper-open event
check("12c. gripper-open event ends the freeze", not hmr.is_load_changing(m2, "left", 6.0))

m3 = Model()
hmr.reset(m3)
hmr.begin_load_change(m3, "left", 0.0)
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m3, "left", 1.0, 1.0, 80)
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m3, "left", 0.4, 1.0 + DUR * 0.5, 81)  # retarget mid-ramp
check("9a. retarget mid-freeze: still changing", hmr.is_load_changing(m3, "left", 1.0 + DUR * 0.5 + DUR * 0.5))
check("9b. retarget mid-freeze: freeze ends at the new target", not hmr.is_load_changing(m3, "left", 1.0 + DUR * 0.5 + DUR))

m4 = Model()
hmr.reset(m4)
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m4, "left", 0.0, 0.0, 1)           # pick start, hand empty
check("10. pick start with no held load: not changing (the approach learns)", not hmr.is_load_changing(m4, "left", 0.5))

m5 = Model()
hmr.reset(m5)
hmr.begin_load_change(m5, "left", 0.0)
hmr.reset(m5)
check("11a. reset clears the freeze", not hmr.is_load_changing(m5, "left", 0.5))
hmr.begin_load_change(m5, "left", 0.0)
hmr.end_load_change(m5, "left")
check("11b. end_load_change clears the flag", not hmr.is_load_changing(m5, "left", 0.5))

C = {"p": np.zeros(3), "r": np.zeros(3)}
H6 = {"ref": None, "full": None}
cfg = PICK_PLACE_CFG
p_err = np.array([0.002, 0.0, -0.001])
r_err = np.array([0.0, 0.01, 0.0])
m6 = Model()
hmr.reset(m6)
hmr.begin_load_change(m6, "left", 0.0)
before = {k: v.copy() for k, v in C.items()}
_sag_trim_step(C, H6, cfg, m6, "left", 0.5, p_err, r_err, 0.01)
check("13a. frozen step: the trim value is unchanged", all(np.array_equal(C[k], before[k]) for k in C), f"{C}")
hmr.end_load_change(m6, "left")
_sag_trim_step(C, H6, cfg, m6, "left", 0.5, p_err, r_err, 0.01)
check("13b. unfrozen step: the trim integrates (gain times error times step)",
      np.allclose(C["p"], cfg["sag_gain_per_s"] * p_err * 0.01) and np.allclose(C["r"], cfg["sag_gain_per_s"] * r_err * 0.01), f"{C}")

# --- the handoff blend of a ramp-in (M231): the trim moves from "full" to "ref" with the ramp, and does not integrate meanwhile ---
F_p, F_r = np.array([0.010, -0.004, 0.020]), np.array([0.002, 0.001, -0.003])
R_p, R_r = np.array([-0.002, -0.003, -0.001]), np.array([-0.009, -0.002, -0.013])
share_p, share_r = F_p - R_p, F_r - R_r
GRID = [DUR * k / 100.0 for k in range(0, 101)]

m9 = Model()
hmr.reset(m9)
C9 = {"p": F_p.copy(), "r": F_r.copy()}
H9 = {"ref": (R_p.copy(), R_r.copy()), "full": None}
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m9, "left", 0.2, 0.0, 90)                # ramp in from 0 kg
check("14a. ramp-in direction is +1 while it runs, 0 after it", hmr.ramp_direction(m9, "left", 0.5 * DUR) == 1 and hmr.ramp_direction(m9, "left", DUR + 0.1) == 0)
worst_p = worst_r = 0.0
for t in GRID[:-1]:
    _sag_trim_step(C9, H9, cfg, m9, "left", t, np.zeros(3), np.zeros(3), 0.01)
    k = hmr.ramp_progress(m9, "left", t)
    worst_p = max(worst_p, float(np.abs(C9["p"] + k * share_p - F_p).max()))
    worst_r = max(worst_r, float(np.abs(C9["r"] + k * share_r - F_r).max()))
check("14b. ramp in: trim + progress times (full minus ref) stays at full within 1e-9 (position and rotation)", worst_p < 1e-9 and worst_r < 1e-9, f"{worst_p} {worst_r}")
m9b = Model()
hmr.reset(m9b)
C9b = {"p": F_p.copy(), "r": F_r.copy()}
H9b = {"ref": (R_p.copy(), R_r.copy()), "full": None}
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m9b, "left", 0.2, 0.0, 91)
_sag_trim_step(C9b, H9b, cfg, m9b, "left", 0.0, np.array([0.001, 0, 0]), np.zeros(3), 0.01)
check("14c. progress 0: the trim equals full", np.allclose(C9b["p"], F_p) and np.allclose(C9b["r"], F_r), f"{C9b}")
_sag_trim_step(C9b, H9b, cfg, m9b, "left", DUR, np.array([0.001, 0, 0]), np.zeros(3), 0.01)
check("14d. ramp end: the trim equals ref and full is cleared", np.allclose(C9b["p"], R_p) and np.allclose(C9b["r"], R_r) and H9b["full"] is None, f"{C9b} {H9b}")
_sag_trim_step(C9b, H9b, cfg, m9b, "left", DUR + 1.0, np.array([0.001, 0, 0]), np.zeros(3), 0.01)
check("14e. after the ramp in the trim integrates again", not np.allclose(C9b["p"], R_p))

m10 = Model()
hmr.reset(m10)
C10 = {"p": F_p.copy(), "r": F_r.copy()}
H10 = {"ref": None, "full": None}
with contextlib.redirect_stdout(quiet):
    hmr.set_target(m10, "left", 0.2, 0.0, 92)
frozen_ok = True
for t in GRID[:-1]:
    _sag_trim_step(C10, H10, cfg, m10, "left", t, np.array([0.001, 0, 0]), np.zeros(3), 0.01)
    frozen_ok = frozen_ok and np.array_equal(C10["p"], F_p) and np.array_equal(C10["r"], F_r)
check("14f. no ramp reference: the trim only freezes during the ramp in", frozen_ok)

H11 = {"ref": (R_p.copy(), R_r.copy()), "full": (F_p.copy(), F_r.copy())}
_trim_handoff_clear(H11)
check("14g. a reset clears ref and full", H11["ref"] is None and H11["full"] is None)

print("\nRESULT:", "PASS" if not fails else f"FAIL ({len(fails)}: {fails})")
sys.exit(1 if fails else 0)
