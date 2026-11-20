"""Sag solver warm start after a failed solve (SF-117).

The arm solver (MinkArm.step) builds its configuration from q_cmd, the warm start, not from the measured joints. SagLayer sets q_cmd
from the measured joints on its first call and advances it only on success. A failed solve used to leave q_cmd at the failing pose
forever, so every later call linearised around that stale pose and failed too (diagnosed on the held run, report section 14).
With SF-117 the next call after a failure takes q_cmd from the measured joints again.

Uses the real solver on the scene model the run loads, with the run's base and pedestal (no fake solver).
  A. reproduce: the first call at the zero pose fails (the elbow is outside its margin window); the arm then moves to the IN pose and the
     next call succeeds with the fix. With the reset disabled (the old behaviour) the same call fails.
  B. streak: several failing calls at the zero pose, then one call at the IN pose: the first good-pose call succeeds, and the streak ends.
  C. success path unchanged: consecutive successful calls advance q_cmd by the solver's step and never reset it.

    python experiments/sag_warm_start_reset_test.py      (exit code 0 when every check passes)
"""

import os
import sys

import numpy as np
import mujoco

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.sag_layer import SagLayer  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)
IN = np.radians([14.83, -50.76, 0.29, 80.08, -38.15, -34.43, -25.42])     # a reachable pose from the held run's probe log
NOMINAL = 1.0 / 150.0
FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


MODEL = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "scenes", "chemistry_lab_beaker_hotplate.xml"))
ADR = MODEL.joint("openarm_left_joint1").qposadr[0]


def make(left_q=None):
    """The scene at the run's base (0.7, 3.4, 0.153 m, yaw 90 deg) and pedestal (0.0704 m); the left arm at left_q (rad) or at zero."""
    d = mujoco.MjData(MODEL)
    d.qpos[0:3] = [0.7, 3.4, 0.153]
    d.qpos[3:7] = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
    d.qpos[9] = 0.0704
    if left_q is not None:
        d.qpos[ADR:ADR + 7] = left_q
    mujoco.mj_forward(MODEL, d)
    return d


def set_arm(d, left_q):
    d.qpos[ADR:ADR + 7] = left_q
    mujoco.mj_forward(MODEL, d)


def fails(layer):
    return sum(1 for e in layer.log if e["kind"] == "solver_no_solution")


def case_a():
    # with the fix
    d = make(None)
    layer = SagLayer(MODEL, d, "left", HOME)
    t = 1.400
    layer.correct(HOME, d, t, NOMINAL)
    first_failed = fails(layer) == 1
    set_arm(d, IN)
    t += NOMINAL
    before = fails(layer)
    layer.correct(IN, d, t, NOMINAL)
    fixed_ok = fails(layer) == before and layer._fail_streak == 0
    advanced = np.allclose(layer.q_cmd, layer.arm.q_cmd)
    check("SF-117 (A1) the first call at the zero pose fails (the diagnosed case)", first_failed, f"failures after call 1: {fails(layer)}")
    check("SF-117 (A2) with the fix: the next call at the IN pose succeeds (the warm start was reset from the measured joints)", fixed_ok,
          f"failures unchanged {fails(layer) == before}, streak {layer._fail_streak}")
    check("SF-117 (A3) the warm start is advanced by the solver's step on that success", advanced)

    # without the fix: the same sequence with the reset disabled (the old code path: the warm start stays at the first call's zero pose)
    d2 = make(None)
    old = SagLayer(MODEL, d2, "left", HOME)
    old.correct(HOME, d2, 1.400, NOMINAL)
    old._reset_warm = False                     # the old behaviour: no reset after the failure
    set_arm(d2, IN)
    before2 = fails(old)
    old.correct(IN, d2, 1.400 + NOMINAL, NOMINAL)
    check("SF-117 (A4) without the reset the same call at the IN pose fails (the failure the held run shows on every call)", fails(old) == before2 + 1,
          f"failures added by the call: {fails(old) - before2}")


def case_b():
    d = make(None)
    layer = SagLayer(MODEL, d, "left", HOME)
    t = 1.400
    for k in range(5):
        layer.correct(HOME, d, t, NOMINAL)
        t += NOMINAL
    streak_failed = fails(layer) == 5 and layer._fail_streak == 5
    streak_before_good = layer._fail_streak
    set_arm(d, IN)
    before = fails(layer)
    layer.correct(IN, d, t, NOMINAL)
    recovered = fails(layer) == before and layer._fail_streak == 0
    ends = [e for e in layer.log if e["kind"] == "solver_recovered"]
    check("SF-117 (B1) five failing calls at the zero pose: each one fails and the streak counts them", streak_failed, f"failures {5}, streak before the good pose {streak_before_good}")
    check("SF-117 (B2) the first good-pose call succeeds and ends the streak", recovered and len(ends) == 1 and ends[0]["streak"] == 5,
          f"recovery entries {[e.get('streak') for e in ends]}")


def case_c():
    d = make(IN)
    layer = SagLayer(MODEL, d, "left", HOME)
    qs, resets = [], []
    t = 1.400
    for k in range(4):
        q_src = IN + np.array([0.02 * np.sin(0.1 * k), 0, 0, 0, 0, 0, 0])
        layer.correct(q_src, d, t, NOMINAL)
        qs.append(np.array(layer.q_cmd, float))
        resets.append(layer._reset_warm)
        t += NOMINAL
    advancing = all(not np.allclose(qs[i], qs[i + 1]) for i in range(len(qs) - 1))
    check("SF-117 (C1) consecutive successful calls advance q_cmd and never set the reset flag", advancing and not any(resets) and fails(layer) == 0,
          f"failures {fails(layer)}, q_cmd moved on each call {advancing}, reset flags {resets}")


def main():
    case_a()
    case_b()
    case_c()
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
