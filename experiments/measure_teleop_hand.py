"""Teleop hand error and dq peak for the SF-123 test copy (analysis only; loads MuJoCo for forward kinematics, run with no other simulation).

    python experiments/measure_teleop_hand.py --root data/logs/stage2k --out data/logs/stage2k/teleop_hand.md

Hand error = the pad position of the accepted reference (REF = bridge output minus dq) minus the pad position of the measured joints, mean over
the run (both arms reported; the right arm is in the teleop log too). dq peak per joint (left arm), from the run's dq log.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "experiments"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    from experiments_loader import load
    from src.sag_layer import _PadGeometry
    from src.robot_helpers import arm_ids
    model, data = load()
    geo = {s: _PadGeometry(model, s) for s in ("left", "right")}
    qadr = {s: np.asarray(arm_ids(model, s)["qadr"]) for s in ("left", "right")}
    rows = ["| run | hand error left (mm) | hand error right (mm) | dq peak left (mrad, joints 1-7) |", "|---|---|---|---|"]
    names = ["teleop_off", "teleop_integ", "teleop_meas_T0.25", "teleop_meas_T0.5", "teleop_meas_T1", "teleop_meas_T2"]
    for nm in names:
        p = Path(a.root) / nm / "run.npz"
        if not p.exists():
            rows.append(f"| {nm} | not run | | |")
            continue
        z = np.load(p)
        he = {}
        for s in ("left", "right"):
            Q = z[f"{s}_q"]
            REF = z[f"{s}_ref"]
            vals = []
            for k in range(len(Q)):
                if not np.all(np.isfinite(REF[k])):
                    continue
                data.qpos[:] = Q[k]
                mujoco.mj_forward(model, data)
                pm = geo[s].pad_pose(data)[0]
                data.qpos[qadr[s]] = REF[k][:7]
                mujoco.mj_forward(model, data)
                pr = geo[s].pad_pose(data)[0]
                vals.append(np.linalg.norm(pr - pm))
                data.qpos[:] = Q[k]
            he[s] = 1e3 * float(np.mean(vals)) if vals else float("nan")
        dqp = 1e3 * np.max(np.abs(z["left_dq"][:, :7]), axis=0) if "left_dq" in z.files else np.full(7, np.nan)
        rows.append(f"| {nm} | {he['left']:.2f} | {he['right']:.2f} | {np.round(dqp, 1).tolist()} |")
        print(rows[-1], flush=True)
    Path(a.out).write_text("\n".join(rows) + "\n")
    print(f"measure_teleop_hand: wrote {a.out}")


if __name__ == "__main__":
    main()
