"""Fit the Gaussian in r with an asymmetric bearing term to the filtered left maps (data/reach_map_before_bench). numpy/scipy only, no MuJoCo.

    source .venv/bin/activate && python experiments/reach_model_fit_asym.py

count(r, theta, z) = C exp(-((r - r0)/sr)^2) exp(-(theta / sigma(theta))^2), 0 below z_cut      r = sqrt(d^2 + z^2); theta = atan2(y, x), theta > 0 is to the robot's left
 A6: C, r0, sr, s_neg (theta < 0), s_pos (theta >= 0), z_cut                    (6 parameters; David's "sigma_left" is the theta < 0 width, "sigma_right" the theta >= 0 width)
 A8: the same with s_neg and s_pos each linear in z (8 parameters), tried only if A6 misses a target
Same protocol as the earlier rounds: fit on the even z-slices, test on the odd ones; precision, recall, within-1-bin on cells with count >= 1.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))

from reach_model_fit import CUBE, cell_coords, load_counts, scores  # noqa: E402
from src.reach_model import predict_count_asym  # noqa: E402

TARGET = 0.80


def fit_form(r, th, z, y, fit_set, test_set, cuts, free_slope):
    best = None
    for zc in cuts:
        def unpack(v):
            return (v[0], v[1], v[2], v[3], v[4] if free_slope else 0.0, v[5 if free_slope else 4], v[6] if free_slope else 0.0)

        def res(v, zc=zc):
            return predict_count_asym(r[fit_set], th[fit_set], z[fit_set], unpack(v), zc) - y[fit_set]
        n = 7 if free_slope else 5
        for v0 in ([3.0, 0.4, 0.25, 0.5, 0.0, 2.0, 0.0], [3.0, 0.4, 0.25, 1.0, 0.0, 1.0, 0.0], [4.0, 0.35, 0.2, 0.3, 0.0, 3.0, 0.0]):
            v0 = np.array([v0[0], v0[1], v0[2], v0[3], v0[4], v0[5], v0[6]] if free_slope else [v0[0], v0[1], v0[2], v0[3], v0[5]])
            f = least_squares(res, v0, x_scale="jac", max_nfev=200)
            rmse = float(np.sqrt(np.mean(f.fun ** 2)))
            if best is None or rmse < best[2]:
                best = (unpack(f.x), zc, rmse)
    p, zc, rmse = best
    pf = predict_count_asym(r[fit_set], th[fit_set], z[fit_set], p, zc)
    pt = predict_count_asym(r[test_set], th[test_set], z[test_set], p, zc)
    return {"C_bins": float(p[0]), "r0_m": float(p[1]), "sigma_r_m": float(p[2]), "sigma_theta_neg_rad": float(p[3]), "sigma_theta_neg_per_m_rad": float(p[4]), "sigma_theta_pos_rad": float(p[5]),
            "sigma_theta_pos_per_m_rad": float(p[6]), "z_cut_m": zc, "rmse_fit_bins": rmse, "fitted_slices": scores(pf, y[fit_set].astype(int)), "held_out_slices": scores(pt, y[test_set].astype(int))}


def met(form):
    s = form["held_out_slices"]
    return all(s[k] >= TARGET for k in ("precision", "recall", "within_1_bin"))


def main():
    cnt, origin = load_counts()
    d, th, z, iz = cell_coords(cnt, origin)
    y = cnt.ravel().astype(float)
    r = np.hypot(d, z)
    fit_set, test_set = iz % 2 == 0, iz % 2 == 1
    zs = origin[2] + CUBE * (np.arange(cnt.shape[2]) + 0.5)
    reach_z = zs[(cnt >= 1).sum(axis=(0, 1)) > 0]
    cuts = [float(v) for v in reach_z[3:9]]                      # around the earlier round's best cut (-0.395 m)
    out = {"forms": {}}
    out["forms"]["A6 asymmetric bearing (6 parameters)"] = fit_form(r, th, z, y, fit_set, test_set, cuts, False)
    print(json.dumps(out["forms"], indent=1), flush=True)
    if not met(out["forms"]["A6 asymmetric bearing (6 parameters)"]):
        out["forms"]["A8 asymmetric bearing, widths linear in z (8 parameters)"] = fit_form(r, th, z, y, fit_set, test_set, cuts, True)
        print(json.dumps(out["forms"], indent=1), flush=True)
    path = "docs/reports/reach_model_fit_asym.json"
    Path(path + ".tmp").write_text(json.dumps(out, indent=1))
    os.replace(path + ".tmp", path)


if __name__ == "__main__":
    main()
