"""Records, per control tick and per arm, the source's joint targets (q_src), the joint targets the layer sent on (q_sent) and the
measured joints (q_meas), while running one existing script unchanged (SF-120 Part 4, 2026-11-23). Measurement only.

    python experiments/gap_split_instrument.py --name NAME --out DIR [--trim zero|on] [--stop-time T] -- <script.py> [args...]

--trim zero (default): the sag trim output is forced to zero in this process (the layer runs, its solver included).
--trim on: the layer runs as it is.
--stop-time T: the process ends at the first layer call at or after sim time T (for the held run's handover).
Writes <out>/<NAME>.npz at exit: for each side, arrays t, q_src, q_sent, q_meas (one row per layer call).
"""

import argparse
import atexit
import os
import runpy
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np  # noqa: E402

from src import sag_layer  # noqa: E402


def main():
    argv = sys.argv[1:]
    if "--" not in argv:
        raise SystemExit("usage: gap_split_instrument.py --name N --out D [--trim zero|on] [--stop-time T] -- script.py [args]")
    k = argv.index("--")
    mine, rest = argv[:k], argv[k + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--trim", default="zero", choices=["zero", "on"])
    ap.add_argument("--stop-time", type=float, default=None)
    a = ap.parse_args(mine)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    if a.trim == "zero":
        real_step = sag_layer._sag_trim_step

        def zeroed(C, cfg, model, side, t, p_err, r_err, dt):
            real_step(C, cfg, model, side, t, p_err, r_err, dt)
            C["p"][:] = 0.0
            C["r"][:] = 0.0

        sag_layer._sag_trim_step = zeroed

    rec = {"left": [], "right": []}
    real_correct = sag_layer.SagLayer.correct

    class Stop(Exception):
        pass

    def capture(self, q_src, data, t, nominal_dt):
        q_sent = real_correct(self, q_src, data, t, nominal_dt)
        q_meas = np.array(data.qpos[np.asarray(self.arm.qadr)], float)
        rec[self.side].append((float(t), np.array(q_src, float), np.array(q_sent, float), q_meas))
        return q_sent

    sag_layer.SagLayer.correct = capture

    import src.episode_runner as er  # the held run's shim calls er.run_loop; the stop is checked on the sim clock here

    real_run_loop = er.run_loop

    def run_loop_stop(model, data, sources, cfg, viewer=None):
        prev_tick = cfg.get("on_tick")

        def tick(obs, loop):
            if a.stop_time is not None and data.time >= a.stop_time:
                save()
                print(f"instrument: stopped at sim time {data.time:.3f} (stop-time {a.stop_time})", flush=True)
                os._exit(0)  # the run loop catches exceptions and would keep going, so end the process here
            return prev_tick(obs, loop) if prev_tick is not None else None

        cfg["on_tick"] = tick
        return real_run_loop(model, data, sources, cfg, viewer=viewer)

    er.run_loop = run_loop_stop

    def save():
        arrs = {}
        for side in ("left", "right"):
            R = rec[side]
            arrs[f"{side}_t"] = np.array([r[0] for r in R])
            arrs[f"{side}_q_src"] = np.array([r[1] for r in R]) if R else np.zeros((0, 7))
            arrs[f"{side}_q_sent"] = np.array([r[2] for r in R]) if R else np.zeros((0, 7))
            arrs[f"{side}_q_meas"] = np.array([r[3] for r in R]) if R else np.zeros((0, 7))
        np.savez(out / f"{a.name}.npz", **arrs)
        print(f"instrument: wrote {out / (a.name + '.npz')} (left {len(rec['left'])} calls, right {len(rec['right'])} calls)", flush=True)

    atexit.register(save)
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    try:
        runpy.run_path(script, run_name="__main__")
    except Stop:
        print(f"instrument: stopped at sim time {a.stop_time}", flush=True)


if __name__ == "__main__":
    main()
