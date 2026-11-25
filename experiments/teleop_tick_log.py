"""Teleop run with a log of EVERY control tick (stage 2 item E, David d0bc 2026-11-24): the arm's measured joints, the bridge output (REF), the
learned correction, dq and the pad error against the bridge-accepted reference, logged on every tick, including the ticks in which the teleop
loses tracking and no arm command is sent (the per-command recorder of climb_perturb_diag.py writes nothing on those ticks).

Analysis and logging only: it wraps src.control_loop.run_loop (the hook cfg["on_tick"] is chained, not replaced) and then runs the bench script
unchanged. The sag correction on/off switch is SAG_LAYER_CFG["enabled"].

    python experiments/teleop_tick_log.py --out DIR/run.npz --correction on -- experiments/combined_controller.py --mode teleop --no-preview

Environment for the bench (as the stage 2 queue sets it): CC_HEADLESS=1 CC_INSETS=0 CC_HUD=0 CC_FAKE_TELEOP=1 CC_FAKE_TELEOP_SIMCLOCK=1
CC_BENCH_FRAMES, CC_BENCH_KEYS, CC_BENCH_LOG as in experiments/cc_bench_*.
"""

import argparse
import atexit
import json
import os
import runpy
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--correction", choices=["on", "off"], required=True)
    a, rest = ap.parse_known_args(argv)
    if rest and rest[0] == "--":
        rest = rest[1:]
    import mujoco  # noqa: F401
    import src.control_loop as cl
    from src import sag_layer
    from src.robot_helpers import arm_ids
    from src.task_config import SAG_LAYER_CFG

    SAG_LAYER_CFG["enabled"] = a.correction == "on"
    rec = {"t": [], "q": [], "ref": [], "C_p": [], "C_r": [], "dq": [], "est": [], "err_m": [], "err_rot": [], "susp": [], "skips": [],
           "pred": [], "sag": [], "applied": [], "ctrl": [], "clamp_updates": [], "src": []}
    holder = {}
    orig_run_loop = cl.run_loop

    def run_loop_logged(model, data, sources, cfg, viewer=None, logger=None):
        ids = arm_ids(model, "left")
        qadr = np.asarray(ids["qadr"])
        prev_hook = cfg.get("on_tick")

        def on_tick(obs, loop):
            layer = sag_layer.SAG_LAYERS.get("left")
            if layer is not None:
                rec["t"].append(float(obs["time"]))
                rec["q"].append(np.asarray(data.qpos[qadr], float).copy())
                ref = layer.refs[-1][1] if layer.refs else np.full(7, np.nan)
                rec["ref"].append(np.asarray(ref, float).copy())
                rec["C_p"].append(layer.C["p"].copy())
                rec["C_r"].append(layer.C["r"].copy())
                rec["dq"].append(np.asarray(layer.dq, float).copy())
                rec["est"].append(np.asarray(layer.est, float).copy())
                rec["susp"].append(int(bool(layer.suspended)))
                rec["pred"].append(np.full(7, np.nan) if layer.pred is None else np.asarray(layer.pred, float).copy())
                rec["sag"].append(np.full(7, np.nan) if getattr(layer, "last_sag", None) is None else np.asarray(layer.last_sag, float).copy())
                rec["applied"].append(np.full(7, np.nan) if layer.applied_offset is None else np.asarray(layer.applied_offset, float).copy())
                rec["ctrl"].append(np.asarray(data.ctrl[layer.acts], float).copy())
                rec["clamp_updates"].append(np.asarray(cl.BRIDGE_CLAMP["updates"]["left"], float).copy())
                rec["src"].append(np.asarray(layer.last_cmd if layer.last_cmd is not None else np.full(7, np.nan), float).copy())
                rec["skips"].append(json.dumps(layer.skips))
                if layer.refs:
                    pad, Rp = layer.arm.pad_pose(data)
                    rpad, Rr = layer._pose(data.qpos, ref)
                    rec["err_m"].append(np.asarray(rpad - pad, float))
                    rec["err_rot"].append(np.asarray(_rotvec(Rr @ Rp.T), float))
                else:
                    rec["err_m"].append(np.full(3, np.nan))
                    rec["err_rot"].append(np.full(3, np.nan))
            if prev_hook is not None:
                return prev_hook(obs, loop)
            return None

        cfg = {**cfg, "on_tick": on_tick}
        res = orig_run_loop(model, data, sources, cfg, viewer=viewer, logger=logger)
        holder["bridge_log"] = res.get("bridge_log", []) if isinstance(res, dict) else []
        holder["bridge_shrink"] = res.get("bridge_shrink", []) if isinstance(res, dict) else []      # stage 3: the shrunk updates
        return res

    cl.run_loop = run_loop_logged

    def save():
        out = {k: np.array(v) for k, v in rec.items() if len(v)}
        out["bridge_log"] = np.array(json.dumps(holder.get("bridge_log", []), default=str))
        out["bridge_shrink"] = np.array(json.dumps(holder.get("bridge_shrink", []), default=str))
        info = {"correction": a.correction, "ticks": len(rec["t"])}
        out["info"] = np.array(json.dumps(info))
        np.savez(a.out, **out)
        print(f"teleop_tick_log: wrote {a.out} ({len(rec['t'])} ticks)", flush=True)

    atexit.register(save)
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


def _rotvec(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_rotvec()


if __name__ == "__main__":
    main(sys.argv[1:])
