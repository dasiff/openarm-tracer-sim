"""Test of the held-mass ramp, its freeze, and the z-only trim handoff (src/held_mass_ramp.py, src/mpvi_modules._sag_trim_step; decisions-log M217 to M219,
M222, M226, M230 to M233). Exits 0 on pass.

    MUJOCO_GL=egl python experiments/held_mass_ramp_test.py

Checks, with a stand-in for the model (the ramp is keyed by id(model) and never reads it):
  1. no ramp: mass 0, finished at once
  2. ramp in: from 0 to 1.0 kg over the duration, minimum-jerk shape, finished at the end
  3. retarget while running: restarts from the current mass over the full duration and prints one line
  4. the same target again: nothing changes and nothing is printed
  5. target 0 with mass 0: finished at once
  6. the two arms are independent (keyed by side)
  7. reset clears every ramp of the model
  8. a ramp in does not freeze by itself; the flag is False with no load (8a); ramp in: changing during the ramp, settled at its target (8e, 8f)
  9. a retarget mid-ramp: still changing until the new target is reached
 10. a pick start with no held load does not freeze (the empty hand approach still learns)
 11. reset clears the freeze; end_load_change clears the flag
 12. ramp out of a held mass starts the freeze; it ends at end_load_change (the gripper-open event)
 13. the trim step: a ramp out freezes every component; after end_load_change the components integrate again
 14. the z-only handoff (M230 to M233):
     14a ramp-in direction is +1 while it runs, 0 after it
     14b during a ramp in, z plus progress times (full z minus ref z) stays at full z within 1e-9 (the share equals full z minus ref z)
     14c x, y and rotation keep integrating during the ramp in (they are not held)
     14d progress 0 gives z equal to full z; the ramp end gives z equal to ref z and clears full
     14e with no ref only z freezes during the ramp in; x, y and rotation integrate
     14f with trim_blend_enabled False nothing is held during the ramp in
     14g a reset clears ref and full
     14h after the ramp in z integrates again
"""

import contextlib
import io
import sys

sys.path.insert(0, "/home/vboxuser/projects/openarm-tracer-sim")

import numpy as np                                    # noqa: E402

from src import held_mass_ramp as hmr                # noqa: E402
from src.mpvi_modules import _sag_trim_step, _trim_handoff_clear  # noqa: E402
from src.rate_limit import min_jerk                  # noqa: E402
from src.task_config import PICK_PLACE_CFG           # noqa: E402

DUR = PICK_PLACE_CFG["held_mass_ramp_s"]
CFG = PICK_PLACE_CFG
CFG_OFF = dict(PICK_PLACE_CFG, trim_blend_enabled=False)
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


# ---- 1 to 7: the ramp module as before ----
m = Model()
hmr.reset(m)
check("1. no ramp: mass 0 and finished", hmr.mass_at(m, "left", 0.3) == 0.0 and hmr.finished(m, "left", 0.3))

ramp_in(m, "left", 1.0, 0.0)
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

ramp_in(m, "right", 0.0, 1.0, 40)                    # mass 0, target 0: finished at once
check("5. target 0 with mass 0: finished at once", hmr.finished(m, "right", 1.0) and hmr.mass_at(m, "right", 1.0) == 0.0)

ramp_in(m, "left", 0.8, 2.0, 50)
check("6. the arms are independent", hmr.mass_at(m, "right", 2.5) == 0.0 and abs(hmr.mass_at(m, "left", 2.0 + DUR) - 0.8) < 1e-12)

hmr.reset(m)
check("7. reset clears every ramp of the model", hmr.mass_at(m, "left", 5.0) == 0.0 and hmr.finished(m, "right", 5.0))

# ---- 8 to 13: the freeze of the load change ----
m2 = Model()
hmr.reset(m2)
check("8a. no load and no ramp: not changing", not hmr.is_load_changing(m2, "left", 0.0))
ramp_in(m2, "left", 0.9, 2.0, 60)
check("8e. ramp in: changing during the ramp", hmr.is_load_changing(m2, "left", 2.0 + DUR * 0.5))
check("8f. ramp in: settled at the end of the ramp (not changing)", not hmr.is_load_changing(m2, "left", 2.0 + DUR))
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

# 13: a ramp out holds every component; after end_load_change they integrate again
m6 = Model()
hmr.reset(m6)
ramp_in(m6, "left", 0.2, 0.0, 100)
ramp_in(m6, "left", 0.0, 2.0, 101)
C6 = trim_state([0.010, -0.004, 0.017], [0.002, 0.001, -0.003])
H6 = {"ref": None, "full": None}
before6 = {k: v.copy() for k, v in C6.items()}
for t in np.arange(2.0, 2.0 + DUR + 0.5, 0.05):
    _sag_trim_step(C6, H6, CFG, m6, "left", float(t), P_ERR, R_ERR, 0.01)
check("13a. ramp out: every component of the trim is frozen", all(np.array_equal(C6[k], before6[k]) for k in C6), f"{C6}")
hmr.end_load_change(m6, "left")
_sag_trim_step(C6, H6, CFG, m6, "left", 2.0 + DUR + 1.0, P_ERR, R_ERR, 0.01)
check("13b. after end_load_change the trim integrates (position and rotation)",
      not np.array_equal(C6["p"], before6["p"]) and not np.array_equal(C6["r"], before6["r"]))

# 14: z-only handoff
F_Z, R_Z = 0.0175, -0.0013                            # full z and ref z (m)
FULL_P, REF_P = np.array([0.0050, -0.0030, F_Z]), np.array([-0.0020, -0.0026, R_Z])
FULL_R, REF_R = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
t0 = 0.0
GRID = [t0 + DUR * k / 100.0 for k in range(0, 101)]

m7 = Model()
hmr.reset(m7)
ramp_in(m7, "left", 0.2, t0, 110)
check("14a. ramp-in direction is +1 while it runs, 0 after it",
      hmr.ramp_direction(m7, "left", t0 + DUR * 0.5) == 1 and hmr.ramp_direction(m7, "left", t0 + DUR + 0.1) == 0)

C7 = trim_state(FULL_P.copy(), FULL_R.copy())
H7 = {"ref": float(REF_P[2]), "full": None}
worst = 0.0
xy_moved = False
for t in GRID[:-1]:
    _sag_trim_step(C7, H7, CFG, m7, "left", t, P_ERR, R_ERR, 0.01)
    k = hmr.ramp_progress(m7, "left", t)
    worst = max(worst, abs(float(C7["p"][2]) + k * (F_Z - R_Z) - F_Z))
    xy_moved = xy_moved or not np.isclose(C7["p"][0], FULL_P[0]) or not np.isclose(C7["p"][1], FULL_P[1])
check("14b. ramp in: z plus progress times (full z minus ref z) stays at full z within 1e-9", worst < 1e-9, f"{worst}")
check("14c. ramp in: x and y keep integrating (they change during the ramp)", xy_moved)
check("14c2. ramp in: rotation keeps integrating (it changes during the ramp)", not np.array_equal(C7["r"], FULL_R))

m8 = Model()
hmr.reset(m8)
ramp_in(m8, "left", 0.2, t0, 111)
C8 = trim_state(FULL_P.copy(), FULL_R.copy())
H8 = {"ref": float(REF_P[2]), "full": None}
_sag_trim_step(C8, H8, CFG, m8, "left", t0, np.zeros(3), np.zeros(3), 0.01)
check("14d1. progress 0: z equals full z", abs(C8["p"][2] - F_Z) < 1e-12, f"{C8['p'][2]}")
_sag_trim_step(C8, H8, CFG, m8, "left", t0 + DUR, np.zeros(3), np.zeros(3), 0.01)
check("14d2. ramp end: z equals ref z and full is cleared", abs(C8["p"][2] - R_Z) < 1e-12 and H8["full"] is None, f"{C8['p'][2]} {H8}")

m9 = Model()
hmr.reset(m9)
ramp_in(m9, "left", 0.2, t0, 112)
C9 = trim_state(FULL_P.copy(), FULL_R.copy())
H9 = {"ref": None, "full": None}
z_frozen = True
for t in GRID[:-1]:
    _sag_trim_step(C9, H9, CFG, m9, "left", t, P_ERR, R_ERR, 0.01)
    z_frozen = z_frozen and abs(C9["p"][2] - F_Z) < 1e-15
check("14e1. no ref: z only freezes during the ramp in", z_frozen, f"{C9['p'][2]}")
check("14e2. no ref: x and y integrate", not np.isclose(C9["p"][0], FULL_P[0]) or not np.isclose(C9["p"][1], FULL_P[1]))
check("14e3. no ref: rotation integrates", not np.array_equal(C9["r"], FULL_R))

m10 = Model()
hmr.reset(m10)
ramp_in(m10, "left", 0.2, t0, 113)
C10 = trim_state(FULL_P.copy(), FULL_R.copy())
H10 = {"ref": float(REF_P[2]), "full": None}
moved_z = False
for t in GRID[:-1]:
    _sag_trim_step(C10, H10, CFG_OFF, m10, "left", t, P_ERR, R_ERR, 0.01)
    moved_z = moved_z or abs(C10["p"][2] - F_Z) > 1e-12
check("14f1. switch off: z integrates during the ramp in (no blend, no hold)", moved_z)
check("14f2. switch off: no blend record is kept", H10["full"] is None, f"{H10}")

H11 = {"ref": float(REF_P[2]), "full": float(FULL_P[2])}
_trim_handoff_clear(H11)
check("14g. a reset clears ref and full", H11["ref"] is None and H11["full"] is None)

m12 = Model()
hmr.reset(m12)
ramp_in(m12, "left", 0.2, t0, 114)
C12 = trim_state(FULL_P.copy(), FULL_R.copy())
H12 = {"ref": float(REF_P[2]), "full": None}
for t in GRID:
    _sag_trim_step(C12, H12, CFG, m12, "left", t, P_ERR, R_ERR, 0.01)
z_end = C12["p"][2]
pre = C12["p"].copy()
_sag_trim_step(C12, H12, CFG, m12, "left", t0 + DUR + 1.0, P_ERR, R_ERR, 0.01)
check("14h. after the ramp in z integrates again", not np.isclose(C12["p"][2], z_end) and abs(z_end - R_Z) < 1e-12 and not np.array_equal(pre, C12["p"]))

print("\nRESULT:", "PASS" if not fails else f"FAIL ({len(fails)}: {fails})")
sys.exit(1 if fails else 0)
