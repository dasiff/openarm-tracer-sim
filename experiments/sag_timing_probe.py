"""Timing probe for the staged rebuild (SF-122): seconds per call of the sag layer (SagLayer.correct, every arms command) and of the command
bridge (control_loop BRIDGE_TIME: bridge_take per arms command and bridge_update per update). Measurement only; it wraps one existing script
unchanged, in the same process.

    python experiments/sag_timing_probe.py --name NAME --out DIR [--stop-time T] -- <script.py> [args...]

--stop-time T: ends the process at sim time T (the held run's first 12 s). Writes <out>/<NAME>_timing.npz and <NAME>_timing.json.
The JSON has mean, median and 99th percentile in milliseconds, per call, for the sag layer and for the bridge.
"""

import argparse
import atexit
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

import numpy as np  # noqa: E402

from src import sag_layer  # noqa: E402
from src.task_config import COMMAND_BRIDGE_CFG, PICK_PLACE_CFG  # noqa: E402


def stats(x):
    x = np.asarray(x, float) * 1e3
    if not len(x):
        return None
    return {"n": int(len(x)), "mean": round(float(x.mean()), 4), "p50": round(float(np.percentile(x, 50)), 4),
            "p99": round(float(np.percentile(x, 99)), 4), "max": round(float(x.max()), 4)}


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

    sag_t = []
    real_correct = sag_layer.SagLayer.correct
    clip = {}   # SF-123 logging: per side, the peak offset (|C_p| in mm, |C_r| in mrad) and the ticks at the clip limit (sag_max_m, sag_max_rad)

    def timed_correct(self, q_src, data, t, nominal_dt):
        t0 = time.perf_counter()
        r = real_correct(self, q_src, data, t, nominal_dt)
        sag_t.append(time.perf_counter() - t0)
        pn, rn = float(np.linalg.norm(self.C["p"])), float(np.linalg.norm(self.C["r"]))
        c = clip.setdefault(self.side, {"peak_p_mm": 0.0, "peak_r_mrad": 0.0, "ticks_at_p_clip": 0, "ticks_at_r_clip": 0, "ticks": 0})
        c["ticks"] += 1
        c["peak_p_mm"] = round(max(c["peak_p_mm"], 1e3 * pn), 4)
        c["peak_r_mrad"] = round(max(c["peak_r_mrad"], 1e3 * rn), 4)
        c["ticks_at_p_clip"] += int(pn >= PICK_PLACE_CFG["sag_max_m"] * (1 - 1e-9))
        c["ticks_at_r_clip"] += int(rn >= PICK_PLACE_CFG["sag_max_rad"] * (1 - 1e-9))
        return r

    sag_layer.SagLayer.correct = timed_correct
    saved = {"done": False}

    def save():
        if saved["done"]:
            return
        saved["done"] = True
        import src.control_loop as cl  # the bridge's own timing list (logging only; filled by the run)
        br = np.asarray(cl.BRIDGE_TIME, float)
        sg = np.asarray(sag_t, float)
        np.savez(out / f"{a.name}_timing.npz", sag_s=sg, bridge_s=br)
        summ = {"sag_layer_call_ms": stats(sg), "bridge_call_ms": stats(br)}
        summ["bridge_clamp_target_counts_per_joint"] = {side: [int(v) for v in arr] for side, arr in cl.BRIDGE_CLAMP["target"].items()}
        upd = {side: [int(v) for v in arr] for side, arr in cl.BRIDGE_CLAMP["updates"].items()}
        bridge_dt = 1.0 / COMMAND_BRIDGE_CFG["rate_hz"]
        summ["bridge_clamp_update_counts_per_joint"] = upd
        summ["bridge_clamp_seconds_per_joint"] = {side: [round(v * bridge_dt, 3) for v in arr] for side, arr in upd.items()}
        summ["sag_offset_and_clip_per_side"] = clip
        from src import sag_layer as _sl  # SF-122 stage 2: the learned offset at the end (both arms), and the skip counts of the solver-free layer
        summ["learned_offset_at_end"] = {side: {"p_mm": [round(1e3 * float(v), 4) for v in L.C["p"]], "r_mrad": [round(1e3 * float(v), 4) for v in L.C["r"]]}
                                         for side, L in _sl.SAG_LAYERS.items() if hasattr(L, "C")}
        summ["sag_skips"] = {side: dict(L.skips) for side, L in _sl.SAG_LAYERS.items() if hasattr(L, "skips")}
        with open(out / f"{a.name}_timing.json", "w") as f:
            json.dump(summ, f, indent=1)
        print(f"timing_probe: wrote {out / (a.name + '_timing.json')}: {json.dumps(summ)[:600]}", flush=True)

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

    atexit.register(save)
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
