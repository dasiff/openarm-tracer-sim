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


def pass2(layer, data, q1, p_t, R_t, dt, cache):
    """Scratch second pass: from q1 toward the pad pose (p_t, R_t). Returns (q, iterations, pos_err_m, rot_err_rad, seconds)."""
    arm = layer.arm
    t0 = time.perf_counter()
    if "config" not in cache:
        cache["config"] = mink.Configuration(arm.model, q=data.qpos.copy())
    config = cache["config"]
    q = data.qpos.copy()
    q[arm.qadr] = q1
    config.update(q)
    R_tcp = np.asarray(R_t, float) @ arm.R_pad_in_tcp.T
    p_tcp = np.asarray(p_t, float) - R_tcp @ arm.pad_in_tcp
    frame = mink.FrameTask(f"openarm_{layer.side}_hand_tcp", "body", position_cost=MINK_CFG["position_cost"], orientation_cost=MINK_CFG["orientation_cost"])
    frame.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(R_tcp), p_tcp))
    it = 0
    pos_err = rot_err = np.inf
    while True:
        e = frame.compute_error(config)                     # mink's body twist: translation (3), rotation (3)
        pos_err, rot_err = float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
        if (pos_err < POS_TOL and rot_err < ROT_TOL) or it >= MAX_ITERS:
            break
        v = mink.solve_ik(config, [frame, arm.posture], dt, MINK_CFG["solver"], damping=MINK_CFG["damping"], limits=arm.pose_limits,
                          constraints=[arm.freeze])
        config.integrate_inplace(v, dt)
        it += 1
    return config.q[arm.qadr].copy(), it, pos_err, rot_err, time.perf_counter() - t0


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
    caches = {}

    def timed_correct(self, q_src, data, t, nominal_dt):
        solver_t["v"] = None
        t0 = time.perf_counter()
        q_out = real_correct(self, q_src, data, t, nominal_dt)
        correct_s = time.perf_counter() - t0
        s1 = solver_t["v"] if solver_t["v"] is not None else np.nan
        if self.arm.q_cmd is not None and self.diag is not None:
            # the scratch pass 2: target = the pad pose of today's solver result, plus the learned offset now
            p1, R1 = self._pose(data.qpos, q_out)
            R_t = (Rotation.from_rotvec(self.C["r"]) * Rotation.from_matrix(R1)).as_matrix()
            q2, it, pe, re_, s2 = pass2(self, data, q_out, p1 + self.C["p"], R_t, self.arm.dt, caches.setdefault(self.side, {}))
            rec.append((float(t), correct_s, s1, s2, it, pe, re_, 0 if self.side == "left" else 1))
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
        np.savez(out / f"{a.name}_timing.npz", t=R[:, 0], correct_s=R[:, 1], solver_s=R[:, 2], pass2_s=R[:, 3], iters=R[:, 4],
                 pos_err_m=R[:, 5], rot_err_rad=R[:, 6], side=R[:, 7])
        summ = {"calls": int(len(R))}
        if len(R):
            for key, col in (("correct_ms", R[:, 1] * 1e3), ("solver_ms", R[:, 2] * 1e3), ("pass2_ms", R[:, 3] * 1e3)):
                col = col[np.isfinite(col)]
                if len(col):
                    summ[key] = {"mean": round(float(col.mean()), 4), "p50": round(float(np.percentile(col, 50)), 4),
                                 "p99": round(float(np.percentile(col, 99)), 4), "max": round(float(col.max()), 4)}
            both = (R[:, 1] + R[:, 3]) * 1e3
            summ["pass1_plus_pass2_ms"] = {"mean": round(float(both.mean()), 4), "p99": round(float(np.percentile(both, 99)), 4),
                                           "max": round(float(both.max()), 4)}
            summ["pass2_iters"] = {"mean": round(float(R[:, 4].mean()), 3), "max": int(R[:, 4].max()),
                                   "share_at_cap": round(float(np.mean(R[:, 4] >= MAX_ITERS)), 4)}
            summ["pass2_final_pos_err_mm_max"] = round(float(np.max(R[:, 5]) * 1e3), 5)
            summ["pass2_final_rot_err_rad_max"] = round(float(np.max(R[:, 6])), 6)
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
