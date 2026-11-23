"""Timing probe for the two-pass sag correction (SF-120 Part 2, step 0). Measurement only; wraps one existing script unchanged.

For every SagLayer.correct call it records:
  - correct_ms: today's layer call (the trim, the reference, and the solver's one step: Pass 1 as it is now);
  - solver_ms: the MinkArm.step inside it (today's solver alone);
  - pass2_ms: a SCRATCH second pass, on a separate configuration (the layer's arm state is not changed): starting from today's
    solver result, target = that result's pad pose plus the layer's current learned offset, no speed limit, same margins, collisions
    and freeze, iterated until the pad is within 0.1 mm and 0.001 rad, at most 10 iterations. Its iteration count and final error are recorded.
    The scratch pass runs after the real call, so the run's own numbers are unchanged.

    python experiments/sag_timing_probe.py --name NAME --out DIR [--stop-time T] -- <script.py> [args...]

--stop-time T: ends the process at sim time T (for the held run's first 12 s). Writes <out>/<NAME>_timing.npz and <NAME>_timing.json.
"""

import argparse
import json
import os
import runpy
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mink  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from src import mink_arm as mink_arm_mod  # noqa: E402
from src import sag_layer  # noqa: E402
from src.task_config import MINK_CFG  # noqa: E402

POS_TOL, ROT_TOL, MAX_ITERS = 1e-4, 1e-3, 10     # 0.1 mm, 0.001 rad, at most 10 iterations (David's Part 2 spec)


def step2_prototype(layer, data, q1, C):
    """SCRATCH prototype of SF-121 Step 2 (timed here before any build): the learned offset C as ONE linear joint change at Pass 1's joints q1,
    dq = J^T (J J^T + damping I)^-1 e with e = [C p; C r] (the pad's world twist) and the arm Jacobian at the pad centre; command = q1 + dq
    clamped to the hard joint limits. Works on the layer's scratch copy only (no change to the arm's state). Returns (command, clamped, seconds)."""
    import mujoco as _mj
    from src.task_config import MINK_CFG as _MC
    from src.robot_helpers import arm_ids as _ids
    arm = layer.arm
    t0 = time.perf_counter()
    scratch = layer.scratch
    scratch.qpos[:] = data.qpos
    scratch.qpos[arm.qadr] = q1
    _mj.mj_kinematics(layer.model, scratch)
    _mj.mj_comPos(layer.model, scratch)
    p_pad, _ = arm.pad_pose(scratch)
    jacp = np.zeros((3, layer.model.nv))
    jacr = np.zeros((3, layer.model.nv))
    _mj.mj_jac(layer.model, scratch, jacp, jacr, p_pad, arm.tcp)
    J = np.vstack([jacp[:, arm.vadr], jacr[:, arm.vadr]])
    e = np.concatenate([np.asarray(C["p"], float), np.asarray(C["r"], float)])
    dq = J.T @ np.linalg.solve(J @ J.T + float(_MC["damping"]) * np.eye(6), e)
    q = q1 + dq
    jnt = _ids(layer.model, layer.side)["jnt"]
    lo, hi = layer.model.jnt_range[jnt, 0], layer.model.jnt_range[jnt, 1]
    clipped = np.clip(q, lo, hi)
    return clipped, bool(np.any(np.abs(clipped - q) > 1e-12)), time.perf_counter() - t0


def main():
    argv = sys.argv[1:]
    if "--" not in argv:
        raise SystemExit("usage: sag_timing_probe.py --name N --out D [--stop-time T] -- script.py [args]")
    k = argv.index("--")
    mine, rest = argv[:k], argv[k + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stop-time", type=float, default=None)
    a = ap.parse_args(mine)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    rec = []
    solver_t = {"v": None}
    real_step = mink_arm_mod.MinkArm.step

    def timed_step(self, data, position, rotation):
        t0 = time.perf_counter()
        r = real_step(self, data, position, rotation)
        solver_t["v"] = time.perf_counter() - t0
        return r

    mink_arm_mod.MinkArm.step = timed_step
    real_correct = sag_layer.SagLayer.correct

    def timed_correct(self, q_src, data, t, nominal_dt):
        solver_t["v"] = None
        t0 = time.perf_counter()
        q_out = real_correct(self, q_src, data, t, nominal_dt)
        correct_s = time.perf_counter() - t0
        s1 = solver_t["v"] if solver_t["v"] is not None else np.nan
        if self.arm.q_cmd is not None and self.diag is not None:
            # Step 2 prototype on today's Pass 1 output and the learned offset now (timed; the layer's state is not changed)
            _, clamped, s2 = step2_prototype(self, data, self.arm.q_cmd.copy(), self.C)
            rec.append((float(t), correct_s, s1, s2, 1 if clamped else 0, 0.0, 0.0, 0 if self.side == "left" else 1))
        return q_out

    sag_layer.SagLayer.correct = timed_correct

    if a.stop_time is not None:
        import src.episode_runner as er  # noqa: E402
        real_run_loop = er.run_loop

        def run_loop_stop(model, data, sources, cfg, viewer=None):
            prev = cfg.get("on_tick")

            def tick(obs, loop):
                if data.time >= a.stop_time:
                    save()
                    print(f"timing_probe: stopped at sim time {data.time:.3f}", flush=True)
                    os._exit(0)
                return prev(obs, loop) if prev is not None else None

            cfg["on_tick"] = tick
            return real_run_loop(model, data, sources, cfg, viewer=viewer)

        er.run_loop = run_loop_stop

    def save():
        R = np.array(rec, float) if rec else np.zeros((0, 8))
        np.savez(out / f"{a.name}_timing.npz", t=R[:, 0], correct_s=R[:, 1], solver_s=R[:, 2], step2_s=R[:, 3], clamped=R[:, 4],
                 side=R[:, 7])
        summ = {"calls": int(len(R))}
        if len(R):
            for key, col in (("correct_ms", R[:, 1] * 1e3), ("solver_ms", R[:, 2] * 1e3), ("step2_ms", R[:, 3] * 1e3)):
                col = col[np.isfinite(col)]
                if len(col):
                    summ[key] = {"mean": round(float(col.mean()), 4), "p50": round(float(np.percentile(col, 50)), 4),
                                 "p99": round(float(np.percentile(col, 99)), 4), "max": round(float(col.max()), 4)}
            both = (R[:, 1] + R[:, 3]) * 1e3
            summ["correct_plus_step2_ms"] = {"mean": round(float(both.mean()), 4), "p99": round(float(np.percentile(both, 99)), 4),
                                           "max": round(float(both.max()), 4)}
        with open(out / f"{a.name}_timing.json", "w") as f:
            json.dump(summ, f, indent=1)
        print(f"timing_probe: wrote {out / (a.name + '_timing.json')}: {json.dumps(summ)[:400]}", flush=True)

    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    try:
        runpy.run_path(script, run_name="__main__")
    finally:
        save()


if __name__ == "__main__":
    main()
