"""Test of the held-mass ramp (src/held_mass_ramp.py, decisions-log M217 to M219, M222). Exits 0 on pass.

    MUJOCO_GL=egl python experiments/held_mass_ramp_test.py

Checks, with a stand-in for the model (the ramp is keyed by id(model) and never reads it):
  1. no ramp: mass 0, finished at once
  2. ramp in: from 0 to 1.0 kg over the duration, minimum-jerk shape, finished at the end
  3. retarget while running: restarts from the current mass over the full duration and prints one line
  4. the same target again: nothing changes and nothing is printed
  5. target 0 with mass 0: finished at once
  6. the two arms are independent (keyed by side)
  7. reset clears every ramp of the model
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

print("\nRESULT:", "PASS" if not fails else f"FAIL ({len(fails)}: {fails})")
sys.exit(1 if fails else 0)
