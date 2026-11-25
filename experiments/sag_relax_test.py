"""Relaxation test (stage 2 follow-up, David 2026-11-25, answer 1): after a motion that left a wrong estimate, with NO new arm commands (the arm holds its
last target, and the sag layer runs every tick on it), the estimate relaxes toward the at-rest droop with the 2 s smoothing.

Harness: the solver-free test's rig (sag_solver_free_test.rig/run): a fixed droop in the hand (2 mm), the arm at rest on a constant held target.
The estimate starts at twice the droop (a wrong estimate left by a motion). The expected curve is the first-order low-pass of the at-rest sag:
est(t) = d + (est0 - d) * exp(-t / T), with d the joint droop that gives the pad droop and T the 2 s smoothing.

    python experiments/sag_relax_test.py      (exit code 0 when every check passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import sag_solver_free_test as T  # noqa: E402
from src.task_config import SAG_LAYER_CFG  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)
    if not ok:
        FAILS.append(name)


def main():
    PAD = np.array([0.0, 0.0, 0.002, 0.0, 0.0, 0.0])          # a 2 mm droop fixed in the hand
    model, data, layer, qadr, emu, delta = T.rig(T.HOME, None)
    J0 = layer._jacobian(data)
    d = np.linalg.pinv(J0) @ PAD                                # the joint droop that gives the pad droop
    est0 = 2.0 * d                                              # a wrong estimate left by a motion
    layer.est = est0.copy()
    n = int(round(10.0 / T.NOMINAL))                            # 10 s, no new commands: the source's target stays at HOME
    recs = T.run(model, data, layer, qadr, emu, delta, lambda k: T.HOME.copy(), n, pad_droop=PAD)
    Tc = float(SAG_LAYER_CFG["measure_T_s"])
    ests = np.array([r["est"] for r in recs])
    ratio = []
    for sec in (1.0, 2.0, 4.0, 6.0, 8.0):
        k = min(int(round(sec / T.NOMINAL)), n - 1)
        err = np.linalg.norm(ests[k] - d)
        err0 = np.linalg.norm(est0 - d)
        expected = np.exp(-sec / Tc)
        ratio.append((sec, err / err0, expected))
    print("INFO  relaxation of the estimate toward the droop (fraction of the initial error left; model exp(-t/T)):", flush=True)
    for sec, got, exp in ratio:
        print(f"INFO    t = {sec:4.1f} s: measured {got:.4f}  model {exp:.4f}", flush=True)
    # the test: at 2T (4 s) and 3T (6 s) the remaining fraction matches the first-order model within 0.02 (absolute)
    got2 = [g for s, g, e in ratio if s == 4.0][0]
    got3 = [g for s, g, e in ratio if s == 6.0][0]
    check("after 2T (4 s) with no new commands the estimate has relaxed by the 2 s smoothing (remaining fraction e^-2 = 0.135, within 0.02)",
          abs(got2 - np.exp(-2.0)) < 0.02, f"remaining fraction {got2:.4f}")
    check("after 3T (6 s) it has relaxed further, to e^-3 = 0.050 (within 0.02)", abs(got3 - np.exp(-3.0)) < 0.02, f"remaining fraction {got3:.4f}")
    # the predictor starts from the measured joints, so the first ~6 ticks (0.04 s) carry a start-up transient (~3e-5 rad); the monotonic check starts at 0.1 s
    dist = np.array([np.linalg.norm(e - d) for e in ests[: int(round(8.0 / T.NOMINAL))]])
    mono = bool(np.all(np.diff(dist[int(round(0.1 / T.NOMINAL)):]) <= 1e-9))
    check("after the 0.1 s start-up the distance from the droop falls monotonically while the arm holds still", mono,
          f"largest start-up increase before 0.1 s {float(np.max(np.diff(dist[:int(round(0.1 / T.NOMINAL))]))):.2e} rad")
    print(f"{len(FAILS)} failed of the checks above", flush=True)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
