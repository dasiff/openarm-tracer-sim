"""Analysis of the lag measurement (experiments/lag_measure.py) and the policy-stream gap fit (SF-120 pre-work, 2026-11-23).

    python experiments/lag_analyze.py <lag_measure_dir> [--policy-dir DIR] [--out report.json]

For each pose (home, mid, near) in <dir>:
  rest droop        mean gap (position, mm; rotation vector, rad) over the last 1 s of the rest hold.
  ramps (trans)     mean gap over the last 1 s of each 1.5 s ramp (t in [0.5, 1.5] s from the ramp's start), minus the rest droop.
                    along = component along the motion direction; lag time = along / commanded speed (ms).
                    perp = the component perpendicular to the motion (should be small if the model is pure lag).
  ramps (rot)       the same for rotation about each axis (rad); lag time = along / rate (ms).
  steps             20 mm step in each direction: the pad's displacement along the step, from the step onset; t63 = first time it
                    reaches 63 % of the step (ms); the value at 2 s (mm) shows whether the response settles on the step.
Commanded speed of a ramp is taken from the reference (forward kinematics of the commanded targets) over the same window;
measured speed from the pad over the same window.

Policy-stream fit (--policy-dir): the variant B and C runs with the correction off (policy_stream_test outputs
B_s1_off_nobridge.csv and C_s1_off_nobridge.csv). Per tick, gap = ref - pad (after the 10.3 s setup), commanded speed = |ref(t+0.1)
- ref(t-0.1)| / 0.2 s, measured speed = |pad(t+0.1) - pad(t-0.1)| / 0.2 s. The gap norm is regressed on each speed with an intercept
(the intercept plays the rest droop); R-squared for the commanded speed and for the measured speed.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

HZ = 150.0
SETUP_S = 10.3333


def load_ticks(path):
    rows = list(csv.DictReader(open(path)))
    out = {k: [] for k in ("i", "t", "phase", "seg")}
    for r in rows:
        out["i"].append(int(r["i"]))
        out["t"].append(float(r["t"]))
        out["phase"].append(r["phase"])
        out["seg"].append(r["seg"])
    arr = {c: np.array([[float(r[f"{c}_{a}"]) for a in "xyz"] for r in rows]) for c in ("pad", "ref", "gap", "rgap")}
    return out, arr


def seg_indices(ticks, seg, phase):
    return [k for k, (s, p) in enumerate(zip(ticks["seg"], ticks["phase"])) if s == seg and p == phase]


def window(idx, t0, t1):
    """Ticks of a contiguous segment block with local time in [t0, t1) seconds from its first tick."""
    if not idx:
        return []
    start = idx[0]
    return [k for k in idx if t0 <= (k - start) / HZ < t1]


def rest_droop(ticks, arr):
    idx = [k for k, p in enumerate(ticks["phase"]) if p == "rest"]
    last = idx[-int(round(HZ)):]
    return arr["gap"][last].mean(axis=0), arr["rgap"][last].mean(axis=0)


def analyse_pose(out_dir, pose):
    seg_json = json.load(open(Path(out_dir) / f"{pose}_segments.json"))
    ticks, arr = load_ticks(Path(out_dir) / f"{pose}_ticks.csv")
    gap0, rgap0 = rest_droop(ticks, arr)
    cells = []
    for s in seg_json["segments"]:
        name = s["name"]
        complete = float(s.get("ramp_s_done", 0.0)) >= RAMP_S - 1e-6 and not s.get("ik_fail")
        if s.get("kind") == "trans":
            idx = seg_indices(ticks, name, "ramp")
            w = window(idx, MEAS_FROM, MEAS_TO)
            if not complete:
                cells.append({"seg": name, "kind": "trans", "dir": name.split("_")[1], "speed_mps": float(s["value"]), "valid": False,
                              "why": "joint limit" if s.get("limit_at_s") is not None else "IK residual", "limit_at_s": s.get("limit_at_s"),
                              "ik_worst": s.get("ik_worst_residual"), "ramp_s_done": s.get("ramp_s_done")})
                continue
            if not w:
                cells.append({"seg": name, "kind": "trans", "note": "no ramp samples", "limit_at_s": s.get("limit_at_s"), "ik_fail": s.get("ik_fail")})
                continue
            d = np.array(s["dir_or_axis"], float)
            v = float(s["value"])
            gap = arr["gap"][w].mean(axis=0) - gap0
            along = float(gap @ d)
            perp = float(np.linalg.norm(gap - along * d))
            pad_speed = float(np.linalg.norm(arr["pad"][w[-1]] - arr["pad"][w[0]]) / ((w[-1] - w[0]) / HZ))
            ref_speed = float(np.linalg.norm(arr["ref"][w[-1]] - arr["ref"][w[0]]) / ((w[-1] - w[0]) / HZ))
            half = len(w) // 2
            drift = float(np.linalg.norm(arr["gap"][w[half:]].mean(axis=0) - gap0) - np.linalg.norm(arr["gap"][w[:half]].mean(axis=0) - gap0))
            cells.append({"valid": True, "gap_drift_mm": 1000 * drift, "seg": name, "kind": "trans", "dir": name.split("_")[1], "speed_mps": v, "ref_speed_mps": ref_speed,
                          "pad_speed_mps": pad_speed, "gap_along_mm": 1000 * along, "gap_perp_mm": 1000 * perp,
                          "lag_ms": 1000 * along / v, "limit_at_s": s.get("limit_at_s"), "ik_fail": s.get("ik_fail"),
                          "ik_worst": s.get("ik_worst_residual")})
        elif s.get("kind") == "rot":
            idx = seg_indices(ticks, name, "ramp")
            w = window(idx, MEAS_FROM, MEAS_TO)
            if not complete:
                cells.append({"seg": name, "kind": "rot", "axis": name.split("_")[1], "rate_radps": float(s["value"]), "valid": False,
                              "why": "joint limit" if s.get("limit_at_s") is not None else "IK residual", "limit_at_s": s.get("limit_at_s"),
                              "ik_worst": s.get("ik_worst_residual"), "ramp_s_done": s.get("ramp_s_done")})
                continue
            if not w:
                cells.append({"seg": name, "kind": "rot", "note": "no ramp samples", "limit_at_s": s.get("limit_at_s"), "ik_fail": s.get("ik_fail")})
                continue
            ax = np.array(s["dir_or_axis"], float)
            w_rate = float(s["value"])
            rg = arr["rgap"][w].mean(axis=0) - rgap0
            along = float(rg @ ax)
            perp = float(np.linalg.norm(rg - along * ax))
            cells.append({"valid": True, "seg": name, "kind": "rot", "axis": name.split("_")[1], "rate_radps": w_rate,
                          "gap_along_rad": along, "gap_perp_rad": perp, "lag_ms": 1000 * along / w_rate,
                          "limit_at_s": s.get("limit_at_s"), "ik_fail": s.get("ik_fail"), "ik_worst": s.get("ik_worst_residual")})
        elif s.get("kind") == "step":
            if s.get("ik_fail"):
                cells.append({"seg": name, "kind": "step", "valid": False, "why": "IK residual", "ik_worst": s.get("ik_worst_residual")})
                continue
            idx = seg_indices(ticks, name, "step")
            if not idx:
                cells.append({"seg": name, "kind": "step", "note": "no step samples"})
                continue
            dvec = np.array(s["dir_or_axis"], float)
            p0 = arr["pad"][idx[0]]
            disp = (arr["pad"][idx] - p0) @ dvec
            target = STEP_M
            hit = np.nonzero(disp >= 0.63 * target)[0]
            t63 = float(hit[0] / HZ) if len(hit) else None
            cells.append({"valid": True, "seg": name, "kind": "step", "dir": name.split("_")[1], "step_mm": 1000 * target,
                          "t63_ms": None if t63 is None else 1000 * t63, "disp_at_2s_mm": 1000 * float(disp[-1]),
                          "disp_max_mm": 1000 * float(disp.max()), "ik_fail": s.get("ik_fail"), "ik_worst": s.get("ik_worst_residual")})
    return {"pose": pose, "rest_droop_pos_mm": (1000 * gap0).tolist(), "rest_droop_rot_rad": rgap0.tolist(),
            "stopped_by": seg_json.get("stopped_by"), "cells": cells}


MEAS_FROM, MEAS_TO = 0.5, 1.5
STEP_M = 0.02
RAMP_S = 1.5


def regress(y, x):
    """Least squares y = a + b x; returns a, b, R2."""
    A = np.column_stack([np.ones_like(x), x])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return float(coef[0]), float(coef[1]), (1 - ss_res / ss_tot) if ss_tot > 0 else float("nan")


def policy_fit(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t"]) for r in rows])
    pad = np.array([[float(r[f"pad_{a}"]) for a in "xyz"] for r in rows])
    ref = np.array([[float(r[f"ref_{a}"]) for a in "xyz"] for r in rows])
    ok = np.all(np.isfinite(ref), axis=1)
    k = int(round(0.1 * HZ))
    n = len(t)
    gap_norm, cmd_speed, meas_speed = [], [], []
    t_start = t[0] + SETUP_S                      # the variant starts after the setup move
    for j in range(k, n - k):
        if t[j] < t_start or not (ok[j - k] and ok[j + k]):
            continue
        gap_norm.append(float(np.linalg.norm(ref[j] - pad[j])))
        cmd_speed.append(float(np.linalg.norm(ref[j + k] - ref[j - k]) / (2 * k / HZ)))
        meas_speed.append(float(np.linalg.norm(pad[j + k] - pad[j - k]) / (2 * k / HZ)))
    g, cs, ms = np.array(gap_norm), np.array(cmd_speed), np.array(meas_speed)
    a_c, b_c, r2_c = regress(g, cs)
    a_m, b_m, r2_m = regress(g, ms)
    return {"file": str(path), "ticks_used": int(len(g)),
            "commanded_speed": {"intercept_mm": 1000 * a_c, "lag_s": b_c, "R2": r2_c},
            "measured_speed": {"intercept_mm": 1000 * a_m, "lag_s": b_m, "R2": r2_m}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--policy-dir", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = {"poses": [analyse_pose(a.dir, p) for p in ("home", "mid", "near") if (Path(a.dir) / f"{p}_segments.json").exists()]}
    if a.policy_dir:
        res["policy"] = [policy_fit(Path(a.policy_dir) / f"{v}_s1_off_nobridge.csv") for v in ("B", "C")]
    text = json.dumps(res, indent=2)
    if a.out:
        Path(a.out).write_text(text)
    print(text)


if __name__ == "__main__":
    main()
