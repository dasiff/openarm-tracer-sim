"""Scene probe for the ARMS stall (SF-120, 2026-11-23). Measurement only; wraps one existing script unchanged.

Records, from the run's own live scene (the scene the test or the held shim builds), every 0.05 s from --log-from to --stop-time:
  - the pedestal joint, the base pose, the bench and the task hotplate (positions, the bench top's height);
  - for the left arm's solver collision geoms (the same sets as src/mink_arm.py _collision_limits: links 3 to 7, hand and fingers),
    the nearest distance to the bench / task hotplate and to the right arm's collision geoms, and every pair closer than 0.10 m;
    a pair closer than MINK_CFG detect_distance (0.05 m) is inside the solver's collision constraint;
  - the left arm's last solver call (q_src, q_sent, q_meas) and the constraint letters per joint (M margin, C cap, X other).

    python experiments/scene_stall_probe.py --name NAME --out DIR [--log-from 3.0] [--stop-time 8.0] -- <script.py> [args...]

The run ends at the stop time (os._exit after saving: the run loop catches exceptions and would otherwise continue).
Writes <out>/<NAME>_scene.csv, <NAME>_pairs.json and <NAME>_calls.npz.
"""

import argparse
import csv
import json
import os
import runpy
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from src import sag_layer  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_config import MINK_CFG  # noqa: E402

LOG_HZ = 20.0           # every 0.05 s
DETECT = float(MINK_CFG["detect_distance"])
MARGIN = float(MINK_CFG["joint_margin_rad"])
CAP = float(MINK_CFG["max_joint_speed"]) / 150.0


def _apply_start_offset(model, data, off):
    """Scratch change (SF-121 Part 1 a): left joints 2 and 4 set `off` rad inside their margin bounds, qpos and actuator command, at the first tick."""
    from src.task_config import MINK_CFG as _MC
    jnt = arm_ids(model, "left")["jnt"]
    qadr = arm_ids(model, "left")["qadr"]
    targets = {1: -off, 3: float(model.jnt_range[jnt[3], 0] + _MC["joint_margin_rad"]) + off}
    for j, q in targets.items():
        data.qpos[qadr[j]] = q
        for act in range(model.nu):
            if model.actuator_trnid[act, 0] == jnt[j]:
                data.ctrl[act] = q
    mujoco.mj_forward(model, data)
    print(f"start_offset: left joints 2 and 4 set to {[round(v, 4) for v in targets.values()]} at the first tick (scratch change)", flush=True)


def main():
    argv = sys.argv[1:]
    if "--" not in argv:
        raise SystemExit("usage: scene_stall_probe.py --name N --out D [--log-from T] [--stop-time T] -- script.py [args]")
    k = argv.index("--")
    mine, rest = argv[:k], argv[k + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--log-from", type=float, default=3.0)
    ap.add_argument("--stop-time", type=float, default=8.0)
    ap.add_argument("--hotplate-x", type=float, default=None,
                    help="scratch change: the task hotplate's x (TASK_CFG and demo_video's MOVED layout, which the held shim uses)")
    ap.add_argument("--start-margin-offset", type=float, default=None,
                    help="scratch change: joints 2 and 4 of the left arm start this many rad inside their margin bounds (after the start-up)")
    a = ap.parse_args(mine)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    if a.hotplate_x is not None:
        from src.task_config import TASK_CFG as _TC
        _TC["hotplate_xy"] = (a.hotplate_x, _TC["hotplate_xy"][1])
        import demo_video as _dv
        _dv.MOVED["hotplate_xy"] = (a.hotplate_x, _dv.MOVED["hotplate_xy"][1])

    calls = {"left": [], "right": []}
    real_correct = sag_layer.SagLayer.correct

    def capture(self, q_src, data, t, nominal_dt):
        q_sent = real_correct(self, q_src, data, t, nominal_dt)
        calls[self.side].append((float(t), np.array(q_src, float), np.array(q_sent, float),
                                 np.array(data.qpos[np.asarray(self.arm.qadr)], float)))
        return q_sent

    sag_layer.SagLayer.correct = capture

    import src.episode_runner as er  # noqa: E402

    state = {"geo": [], "pairs": {}, "names": None}
    real_run_loop = er.run_loop

    def run_loop_probe(model, data, sources, cfg, viewer=None):
        prev_tick = cfg.get("on_tick")
        ctx = {"model": model, "data": data, "next": a.log_from}

        def tick(obs, loop):
            t = float(data.time)
            if a.start_margin_offset is not None and not ctx.get("start_done"):
                ctx["start_done"] = True
                _apply_start_offset(model, data, a.start_margin_offset)
            if t >= a.stop_time:
                save(model)
                print(f"scene_probe: stopped at sim time {t:.3f} (stop-time {a.stop_time})", flush=True)
                os._exit(0)
            if t >= ctx["next"] - 1e-9:
                ctx["next"] += 1.0 / LOG_HZ
                sample(model, data, t)
            return prev_tick(obs, loop) if prev_tick is not None else None

        cfg["on_tick"] = tick
        return real_run_loop(model, data, sources, cfg, viewer=viewer)

    er.run_loop = run_loop_probe

    def body_name(model, b):
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "world"

    def sets(model):
        col = [g for g in range(model.ngeom) if model.geom_contype[g] or model.geom_conaffinity[g]]
        bn = lambda g: body_name(model, model.geom_bodyid[g])
        left = [g for g in col if bn(g).startswith("openarm_left_") and not bn(g).endswith(("link0", "link1", "link2"))]
        right = [g for g in col if bn(g).startswith("openarm_right_") and not bn(g).endswith("link0")]
        furn = [g for g in col if bn(g) in ("bench", "task_hotplate")]
        return left, right, furn

    def sample(model, data, t):
        if state["names"] is None:
            state["names"] = sorted({body_name(model, b) for b in range(model.nbody)})
            state["sets"] = sets(model)
        left, right, furn = state["sets"]
        ped = float(data.qpos[model.jnt_qposadr[model.joint("pedestal_joint").id]])
        base = data.xpos[model.body("base_link").id]
        R = data.xmat[model.body("base_link").id].reshape(3, 3)
        yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        bench = data.xpos[model.body("bench").id]
        hot_ids = [b for b in range(model.nbody) if "hotplate" in body_name(model, b)]
        hot = data.xpos[hot_ids[0]] if hot_ids else np.full(3, np.nan)
        hot_name = body_name(model, hot_ids[0]) if hot_ids else None
        pairs_all = []
        nearest = {}
        for label, others in (("furniture", furn), ("right_arm", right)):
            best = (np.inf, None, None)
            for g in left:
                for og in others:
                    d = mujoco.mj_geomDistance(model, data, g, og, 0.10, np.zeros(6))
                    if d < 0.10:
                        pairs_all.append({"label": label, "left": body_name(model, model.geom_bodyid[g]),
                                          "other": body_name(model, model.geom_bodyid[og]), "d_m": round(float(d), 5)})
                    if d < best[0]:
                        best = (d, body_name(model, model.geom_bodyid[g]), body_name(model, model.geom_bodyid[og]))
            nearest[label] = best
        active = [p for p in pairs_all if p["d_m"] < DETECT]
        last = {}
        for side in ("left", "right"):
            prior = [c for c in calls[side] if c[0] <= t + 1e-9]
            last[side] = prior[-1] if prior else None
        lc = last["left"]
        if lc is not None:
            tc, qs, qe, qm = lc
            prev = [c for c in calls["left"] if c[0] < tc - 1e-9]
            qe_prev = prev[-1][2] if prev else None
            lo = model.jnt_range[arm_ids(model, "left")["jnt"], 0] + MARGIN
            hi = model.jnt_range[arm_ids(model, "left")["jnt"], 1] - MARGIN
            letters = ""
            for j in range(7):
                on = abs(qe[j] - hi[j]) < 1e-3 or abs(qe[j] - lo[j]) < 1e-3
                capd = qe_prev is not None and abs(qe[j] - qe_prev[j]) >= 0.99 * CAP
                letters += "M" if on else ("C" if capd else ("X" if abs(qe[j] - qs[j]) > 0.02 else "."))
        else:
            qs = qe = qm = np.full(7, np.nan)
            letters = ""
        state["geo"].append({
            "t": round(t, 3), "pedestal_m": round(ped, 5), "base_x": round(float(base[0]), 4), "base_y": round(float(base[1]), 4),
            "base_yaw_deg": round(yaw, 2), "bench_x": round(float(bench[0]), 4), "bench_y": round(float(bench[1]), 4),
            "bench_z": round(float(bench[2]), 4), "hot_name": hot_name, "hot_x": round(float(hot[0]), 4), "hot_y": round(float(hot[1]), 4),
            "hot_z": round(float(hot[2]), 4), "min_left_to_furniture_m": None if not np.isfinite(nearest["furniture"][0]) else round(float(nearest["furniture"][0]), 4),
            "furniture_pair": nearest["furniture"][1:], "min_left_to_right_m": None if not np.isfinite(nearest["right_arm"][0]) else round(float(nearest["right_arm"][0]), 4),
            "right_pair": nearest["right_arm"][1:], "n_active_pairs": len(active), "letters_j1_to_j7": letters,
            "j2_src": round(float(qs[1]), 4), "j2_sent": round(float(qe[1]), 4), "j2_meas": round(float(qm[1]), 4),
            "j2_margin_hi": round(float(model.jnt_range[arm_ids(model, "left")["jnt"][1], 1] - MARGIN), 4),
        })
        state["pairs"][f"{t:.3f}"] = {"active": active, "within_0.10": pairs_all}
        # the solver's own limit objects: the inequality rows each limit gives at the last solve (rows > 0 = the constraint is active)
        lim_rows = {}
        layer = sag_layer.SAG_LAYERS.get("left")
        if layer is not None:
            arm = layer.arm
            for lim in arm.limits:
                name = type(lim).__name__
                try:
                    G, h = lim.compute_qp_inequalities(arm.config, arm.dt)
                    G = np.atleast_2d(G)
                    rows = int(G.shape[0]) if G.size else 0
                    lim_rows[name] = {"rows": rows, "min_h": round(float(np.min(h)), 6) if rows else None}
                except Exception as e:  # logging only: a limit that cannot report its rows is recorded as such
                    lim_rows[name] = {"error": f"{type(e).__name__}: {e}"}
        state.setdefault("limits", {})[f"{t:.3f}"] = lim_rows

    def save(model):
        if state["geo"]:
            with open(out / f"{a.name}_scene.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(state["geo"][0].keys()))
                w.writeheader()
                w.writerows(state["geo"])
        with open(out / f"{a.name}_pairs.json", "w") as f:
            json.dump({"body_names": state["names"], "pairs_by_time": state["pairs"], "limits_by_time": state.get("limits", {})}, f)
        arrs = {}
        for side in ("left", "right"):
            C = calls[side]
            arrs[f"{side}_t"] = np.array([c[0] for c in C])
            arrs[f"{side}_q_src"] = np.array([c[1] for c in C]) if C else np.zeros((0, 7))
            arrs[f"{side}_q_sent"] = np.array([c[2] for c in C]) if C else np.zeros((0, 7))
            arrs[f"{side}_q_meas"] = np.array([c[3] for c in C]) if C else np.zeros((0, 7))
        np.savez(out / f"{a.name}_calls.npz", **arrs)
        print(f"scene_probe: wrote {out / (a.name + '_scene.csv')} ({len(state['geo'])} samples)", flush=True)

    import atexit
    atexit.register(lambda: save(None))           # a normal end of the script (e.g. the test's own timeout) also writes the outputs
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
