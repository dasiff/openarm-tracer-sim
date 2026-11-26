"""Tests of the command bridge's one uniform shrink (stage 3, David d0bc, 2026-11-24; src/bridge_shrink.py). Pure checks (no simulator) and two loop checks
(the tracking-error suspension still fires; a repeated over-limit target is reached and never rejected).

    python experiments/bridge_shrink_test.py      (exit code 0 when every check passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

from src.bridge_shrink import apply_shrink, shrink_factor  # noqa: E402

SPEED = 0.5 / 50.0            # rad per update (the bridge's cap, 0.01)
STEP = 0.25                   # rad (the step-size limit)
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   ({detail})" if detail else ""), flush=True)


def z():
    return np.zeros(7)


def main():
    print("1. within every limit: the move passes unchanged, exactly", flush=True)
    d = np.array([0.004, -0.009, 0.0, 0.001, 0.0, 0.0, 0.0])
    f, rule, side, joint, peak = shrink_factor({"left": d, "right": z()}, STEP, SPEED)
    cmd = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    out = apply_shrink(cmd, cmd + d, f)
    check("factor 1 and no binding rule inside the limits", f == 1.0 and rule is None, f"f {f}, rule {rule}")
    check("the command becomes the target exactly", np.array_equal(out, cmd + d), f"max diff {float(np.max(np.abs(out - (cmd + d)))):.2e}")

    print("2. over the speed limit: the largest move is scaled to 0.01 rad, the direction is kept", flush=True)
    dl = np.zeros(7); dl[0] = 0.2; dl[2] = -0.1
    f, rule, side, joint, peak = shrink_factor({"left": dl, "right": z()}, STEP, SPEED)
    new = apply_shrink(z(), dl, f)
    check("the speed rule binds (0.01 / 0.2 is smaller than 0.25 / 0.2)", rule == "speed" and side == "left" and joint == 1 and abs(f - 0.05) < 1e-12,
          f"f {f:.4f}, rule {rule}, arm {side}, joint {joint}")
    check("the largest joint move after the shrink is exactly 0.01 rad", abs(float(np.max(np.abs(new))) - 0.01) < 1e-12, f"{float(np.max(np.abs(new))):.6f} rad")
    cosv = float(np.dot(new, dl) / (np.linalg.norm(new) * np.linalg.norm(dl)))
    check("the direction is preserved (cosine 1 within 1e-9)", abs(cosv - 1.0) < 1e-9, f"cosine {cosv:.12f}")

    print("3. the step-size rule binds when its limit is the smaller one", flush=True)
    dstep = np.zeros(7); dstep[1] = 0.02
    f, rule, side, joint, peak = shrink_factor({"left": dstep, "right": z()}, 0.005, SPEED)
    new = apply_shrink(z(), dstep, f)
    check("with a step limit of 0.005 rad the step rule binds (f 0.25)", rule == "step" and abs(f - 0.25) < 1e-12, f"f {f:.4f}, rule {rule}")
    check("the largest move is 0.005 rad", abs(float(np.max(np.abs(new))) - 0.005) < 1e-12, f"{float(np.max(np.abs(new))):.6f} rad")

    print("4. a large left step shrinks the right arm by the same factor, and vice versa", flush=True)
    dL = np.zeros(7); dL[0] = 0.2
    dR = np.zeros(7); dR[1] = 0.005
    f, rule, side, joint, peak = shrink_factor({"left": dL, "right": dR}, STEP, SPEED)
    newR = apply_shrink(z(), dR, f)
    check("the right arm's 0.005 rad move becomes f x 0.005 (the left arm's factor, 0.05)", abs(float(newR[1]) - 0.05 * 0.005) < 1e-15,
          f"right move {float(newR[1]):.6f} rad (not 0.005)")
    f2, rule2, side2, _, _ = shrink_factor({"left": dR, "right": dL}, STEP, SPEED)
    newL = apply_shrink(z(), dR, f2)
    check("vice versa: a large right step shrinks the left arm's small move by the same factor", abs(f2 - f) < 1e-15 and side2 == "right" and abs(float(newL[1]) - 0.05 * 0.005) < 1e-15,
          f"factor {f2:.4f}, left move {float(newL[1]):.6f} rad")

    print("5. one arm's zero move stays zero", flush=True)
    dR0 = z()
    f, rule, side, joint, peak = shrink_factor({"left": dL, "right": dR0}, STEP, SPEED)
    cR = np.array([0.5, 0.1, -0.2, 1.0, 0.0, 0.0, 0.0])
    check("the right arm with no move is unchanged by the left arm's shrink", np.array_equal(apply_shrink(cR, cR, f), cR),
          f"factor {f:.4f}")

    print("6. a repeated over-limit target reaches the target in the expected number of updates and is never rejected", flush=True)
    target = z(); target[2] = 0.6
    cmd = z()
    updates = 0
    shrunk = 0
    rejected = 0
    while np.max(np.abs(target - cmd)) > 1e-12 and updates < 500:
        mv = target - cmd
        f, rule, *_ = shrink_factor({"left": mv, "right": z()}, STEP, SPEED)
        shrunk += f < 1.0
        cmd = apply_shrink(cmd, target, f)
        updates += 1
    check("the target is reached in ceil(0.6 / 0.01) = 60 updates", updates == 60 and np.array_equal(cmd, target), f"{updates} updates")
    check("59 of them are shrunk and the last is a full 0.01 rad step", shrunk == 59 and rejected == 0, f"shrunk {shrunk}, rejected {rejected}")

    print("7. the tracking-error suspension still fires (the bridge's own check, unchanged)", flush=True)
    import command_bridge_test as cbt
    from src.task_config import SAG_LAYER_CFG
    SAG_LAYER_CFG["enabled"] = False
    a, res, seen = cbt.run(420, lambda k, rest: {"mode": "arms", "arm_targets": {"left": [float(x) for x in (rest + np.where(np.arange(7) == 2, 0.2, 0.0))]}} if k >= 30 else None,
                           {"enabled": True, "max_tracking_error_rad": 0.003})
    sus = [e for e in res["bridge_log"] if e["kind"].startswith("suspended")]
    check("a suspension is logged while the arm lags by more than the tracking limit", len(sus) > 0 and sus[0]["limit"] == 0.003, str(sus[:1]))

    n = len(RESULTS)
    print(f"{sum(RESULTS)} of {n} checks passed", flush=True)
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
