"""Analysis of the SF-121 Part 1 runs (start pose and hotplate tests). Reads the scene probe outputs (scene_stall_probe.py): the per-call
arrays (<name>_calls.npz) and the 0.05 s scene rows (<name>_scene.csv). For each run:
  - the arms-to-home result: the first time every measured joint of the left and right arms is within arms_tol (0.03 rad) of home, or never;
  - the time each left-arm joint sits on its solver margin bound (from the constraint letters, M);
  - the first tick where the run's solver output differs from the reference run (1e-6 rad), on either arm.

    python experiments/start_hotplate_analyse.py --run LABEL=DIR/NAME --ref LABEL=DIR/NAME ... --out FILE.json
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.task_config import TASK_CFG  # noqa: E402

HOME = {"left": np.asarray(TASK_CFG["home_q_left"], float), "right": np.asarray(TASK_CFG["home_q_right"], float)}
ARMS_TOL = 0.03


def load(prefix):
    z = np.load(f"{prefix}_calls.npz")
    rows = list(csv.DictReader(open(f"{prefix}_scene.csv")))
    return z, rows


def summarise(z, rows):
    out = {}
    for side in ("left", "right"):
        t, meas = z[f"{side}_t"], z[f"{side}_q_meas"]
        dev = np.max(np.abs(meas - HOME[side]), axis=1)
        within = np.where(dev < ARMS_TOL)[0]
        out[f"{side}_home_within_tol_t"] = round(float(t[within[0]]), 3) if len(within) else None
        out[f"{side}_max_dev_home_last"] = round(float(dev[-1]), 3)
    # margin bounds: letters M per joint from the scene rows (left arm)
    dt_row = 0.05
    on = {f"j{j + 1}": [] for j in range(7)}
    for r in rows:
        for j, ch in enumerate(r["letters_j1_to_j7"]):
            if ch == "M":
                on[f"j{j + 1}"].append(float(r["t"]))
    out["margin_time_left"] = {k: {"samples": len(v), "seconds": round(len(v) * dt_row, 2),
                                    "first_t": v[0] if v else None, "last_t": v[-1] if v else None} for k, v in on.items() if v}
    return out


def divergence(z, zr):
    first = {}
    for side in ("left", "right"):
        n = min(len(z[f"{side}_t"]), len(zr[f"{side}_t"]))
        d = np.max(np.abs(z[f"{side}_q_sent"][:n] - zr[f"{side}_q_sent"][:n]), axis=1)
        idx = np.where(d > 1e-6)[0]
        first[side] = round(float(z[f"{side}_t"][idx[0]]), 4) if len(idx) else None
    cand = [v for v in first.values() if v is not None]
    return {"first_t_per_arm": first, "first_t": min(cand) if cand else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", required=True, help="LABEL=PREFIX (the probe's --out DIR/NAME)")
    ap.add_argument("--ref", action="append", required=True, help="LABEL=PREFIX of the reference run for each --run, in the same order")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    result = {}
    for spec, rspec in zip(a.run, a.ref):
        label, prefix = spec.split("=", 1)
        rlabel, rprefix = rspec.split("=", 1)
        z, rows = load(prefix)
        zr, _ = load(rprefix)
        s = summarise(z, rows)
        s["calls"] = int(len(z["left_t"]))
        s["t_last"] = round(float(z["left_t"][-1]), 3)
        s["vs_reference"] = rlabel
        s["divergence"] = divergence(z, zr)
        result[label] = s
    with open(a.out, "w") as f:
        json.dump(result, f, indent=1)
    print(json.dumps(result, indent=1)[:4000], flush=True)


if __name__ == "__main__":
    main()
