"""Fit a Gaussian in the 3D distance r = sqrt(d^2 + z^2) from the shoulder to the filtered left maps (data/reach_map_before_bench). numpy/scipy only, no MuJoCo.

    source .venv/bin/activate && python experiments/reach_model_fit_r.py

Model R1: count = C * exp(-((r - r0) / sr)^2)                       (3 parameters)
Model R2: R1 times a height cutoff, count = 0 below z_cut           (4 parameters; the table: the filtered maps hold the bench, so nothing is reachable below it)
No bearing term: the reachable cells are not concentrated around the heading (see docs/reports/reach_model.md).
Same protocol as reach_model_fit.py: fit on the even z-slices, test on the odd ones; precision, recall and within-1-bin on cells with count >= 1.
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


def predict_r(r, z, C, r0, sr, z_cut=None):
    out = np.clip(C * np.exp(-((r - r0) / max(sr, 1e-3)) ** 2), 0.0, 12.0)
    return out if z_cut is None else np.where(z >= z_cut, out, 0.0)


def main():
    cnt, origin = load_counts()
    d, th, z, iz = cell_coords(cnt, origin)
    y = cnt.ravel().astype(float)
    r = np.hypot(d, z)
    fit_set, test_set = iz % 2 == 0, iz % 2 == 1
    zs = origin[2] + CUBE * (np.arange(cnt.shape[2]) + 0.5)
    per_z = (cnt >= 1).sum(axis=(0, 1))
    reach_z = zs[per_z > 0]
    facts = {"lowest_reachable_z_m": float(reach_z.min()), "highest_reachable_z_m": float(reach_z.max()),
             "reachable_cells_per_slice_at_lowest_5_slices": [int(v) for v in per_z[per_z > 0][:5]], "reachable_r_m": [float(r[y >= 1].min()), float(r[y >= 1].max())]}
    print(json.dumps(facts, indent=1), flush=True)

    def fit(mask, z_cut):
        res = lambda v: predict_r(r[mask], z[mask], v[0], v[1], v[2], z_cut) - y[mask]      # noqa: E731
        best = None
        for v0 in ([3.0, 0.4, 0.2], [2.0, 0.5, 0.1]):
            f = least_squares(res, v0, x_scale="jac", max_nfev=300)
            if best is None or f.cost < best.cost:
                best = f
        return best.x, float(np.sqrt(np.mean(best.fun ** 2)))

    out = {"facts": facts, "forms": {}}
    # R2's cut: scan the slice heights near the lowest reachable one, keep the cut with the smallest RMSE on the fit slices
    cuts = [None] + [float(v) for v in reach_z[:15]]
    for name, cand in (("R1 gaussian in r", [None]), ("R2 gaussian in r with height cutoff", cuts[1:])):
        best = None
        for zc in cand:
            v, rmse = fit(fit_set, zc)
            if best is None or rmse < best[2]:
                best = (v, zc, rmse)
        v, zc, rmse = best
        pf, pt = predict_r(r[fit_set], z[fit_set], *v, zc), predict_r(r[test_set], z[test_set], *v, zc)
        out["forms"][name] = {"C_bins": float(v[0]), "r0_m": float(v[1]), "sigma_r_m": float(v[2]), "z_cut_m": zc, "rmse_fit_bins": rmse,
                              "fitted_slices": scores(pf, cnt.ravel()[fit_set]), "held_out_slices": scores(pt, cnt.ravel()[test_set])}
        print(name, json.dumps(out["forms"][name], indent=1), flush=True)
    p = "docs/reports/reach_model_fit_r.json"
    Path(p + ".tmp").write_text(json.dumps(out, indent=1))
    os.replace(p + ".tmp", p)


if __name__ == "__main__":
    main()
