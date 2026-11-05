"""Test of the elbow height guard (src/elbow_guard.py, decision M269). Exit 0 on pass.

Checks, on the scene model at the settled base height:
  1. the minimum is 1.1048 m (bench top 0.960 + housing 0.1188 + allowance 0.012 + margin 0.014);
  2. the zombie pose at pedestal 0.15 m is above the minimum on both arms, so the clamp passes it unchanged;
  3. the zombie pose at pedestal 0.14 m is already below the minimum on the left arm, so the clamp holds the measured angles;
  4. a target that would put the elbow below the minimum is clamped onto the segment from the measured angles, ending at or above the minimum;
  5. with the switch off, targets pass unchanged.

    MUJOCO_GL=egl python experiments/elbow_guard_test.py
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.elbow_guard import ElbowGuard                        # noqa: E402
from src.task_config import ELBOW_GUARD_CFG, TASK_CFG          # noqa: E402

BASE_Z = 0.1527                                                # settled base height (stance_finder robot_geometry), metres

model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "scenes" / "chemistry_lab_beaker_hotplate.xml"))
data = mujoco.MjData(model)
SIDES = ("left", "right")
ARM_QADR = {s: [int(model.joint(f"openarm_{s}_joint{k}").qposadr[0]) for k in range(1, 8)] for s in SIDES}
PED_QADR = int(model.joint("pedestal_joint").qposadr[0])
BASE_QADR = int(model.joint("base_float").qposadr[0])
failures = []


def state(ped):
    """A settled-base qpos with the arms at the zombie pose and the given pedestal height."""
    mujoco.mj_resetData(model, data)
    data.qpos[:] = model.qpos0
    data.qpos[BASE_QADR:BASE_QADR + 3] = [0.0, 0.0, BASE_Z]
    data.qpos[BASE_QADR + 3:BASE_QADR + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[PED_QADR] = ped
    for s in SIDES:
        data.qpos[ARM_QADR[s]] = TASK_CFG["home_q_" + s]
    return data.qpos.copy()


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(name)


guard = ElbowGuard(model, SIDES, {s: ARM_QADR[s] for s in SIDES}, PED_QADR)
min_z = ELBOW_GUARD_CFG["min_joint_z_m"]

# 1. the minimum
check("minimum is 1.1048 m", abs(min_z - 1.1048) < 1e-9, f"{min_z}")

# 2. zombie at 0.15 m is safe on both arms; the clamp returns the target itself
q = state(0.15)
targets = {s: np.asarray(TASK_CFG["home_q_" + s], float) for s in SIDES}
z = {s: guard.elbow_z(q, s, targets[s], 0.15) for s in SIDES}
check("zombie 0.15 m: left elbow above minimum", z["left"] >= min_z, f"left {z['left']:.4f} m")
check("zombie 0.15 m: right elbow above minimum", z["right"] >= min_z, f"right {z['right']:.4f} m")
out = guard.clamp(q, targets, 0.15)
check("zombie 0.15 m: clamp leaves the target unchanged", all(out[s] is targets[s] for s in SIDES))

# 3. zombie at 0.14 m is below the minimum on the left: the clamp holds the measured angles
q = state(0.14)
out = guard.clamp(q, targets, 0.14)
check("zombie 0.14 m: left below minimum", guard.elbow_z(q, "left", targets["left"], 0.14) < min_z)
check("zombie 0.14 m: clamp holds the measured left angles", np.allclose(out["left"], q[ARM_QADR["left"]]))

# 4. a target that goes below the minimum is clamped onto the segment, at or above the minimum
q = state(0.15)
q_now = q[ARM_QADR["left"]].copy()
lo_j2, hi_j2 = model.jnt_range[model.joint("openarm_left_joint2").id]
bad = None
for j2 in np.linspace(lo_j2, hi_j2, 400):
    cand = targets["left"].copy()
    cand[1] = j2
    if guard.elbow_z(q, "left", cand, 0.15) < min_z:
        bad = cand
        break
check("found a target below the minimum (joint 2 sweep)", bad is not None)
if bad is not None:
    out = guard.clamp(q, {"left": bad, "right": targets["right"]}, 0.15)
    z_out = guard.elbow_z(q, "left", out["left"], 0.15)
    check("clamped target is at or above the minimum", z_out >= min_z - 1e-6, f"{z_out:.6f} m")
    d = bad - q_now
    lam = float(np.dot(out["left"] - q_now, d) / np.dot(d, d))
    on_segment = np.allclose(out["left"], q_now + lam * d, atol=1e-9) and -1e-9 <= lam <= 1.0 + 1e-9
    check("clamped target lies on the segment from the measured angles", on_segment, f"fraction {lam:.4f}")
    check("clamp counts the hit", guard.hits["left"] >= 1)

# 5. switch off: no clamp
off = ElbowGuard(model, SIDES, {s: ARM_QADR[s] for s in SIDES}, PED_QADR, cfg={**ELBOW_GUARD_CFG, "enabled": False})
if bad is not None:
    out = off.clamp(q, {"left": bad, "right": targets["right"]}, 0.15)
    check("switch off: target passes unchanged", out["left"] is bad)

print("FAILED: " + ", ".join(failures) if failures else "ALL PASS")
sys.exit(1 if failures else 0)
