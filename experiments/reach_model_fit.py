"""Fit the Gaussian reach model (src/reach_model.py) to the unfiltered left maps (data/reach_map) and score it. numpy/scipy only, no MuJoCo.

    source .venv/bin/activate && python experiments/reach_model_fit.py [--out docs/reports/reach_model_fit.json]

Part A: the 12 left maps are read straight from the .npz files (packed voxels -> 1 cm cubes: all 8 voxels reachable) and put on one common 1 cm grid in the
shoulder frame (each bin has its own bounding box; every origin is a multiple of 1 cm). count[ix, iy, iz] = number of bins whose map holds the cube (0..12).
Part B: fit on the even z-slices, test on the odd ones; two forms: the specified one (angular centre 0) and ONE alternative (a constant angular centre th0).
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.reach_model import PARAM_NAMES, predict_count  # noqa: E402

MAPS_DIR = PROJECT_ROOT / "data" / "reach_map"        # the unfiltered maps (no bench)
CUBE = 0.01
MAX_D = 1.5                                            # m: the evaluation region (the maps' boxes are smaller, so every cell qualifies)


def load_counts():
    """The count grid (nx, ny, nz) int8 and its shoulder-frame origin, from the 12 left maps."""
    maps = []
    for k in range(12):
        z = np.load(MAPS_DIR / f"left_bin{k}.npz")
        shape = tuple(int(v) for v in z["shape"])
        r = np.unpackbits(z["reached"])[:int(np.prod(shape))].reshape(shape).astype(bool)
        a, b, c = r.shape
        cubes = r.reshape(a // 2, 2, b // 2, 2, c // 2, 2).all(axis=(1, 3, 5))
        off = np.rint(z["origin"] / CUBE).astype(int)
        assert np.allclose(off * CUBE, z["origin"], atol=1e-6) and float(z["voxel"]) == 0.005 and str(z["side"]) == "left"
        maps.append((cubes, off))
    lo = np.min([m[1] for m in maps], axis=0)
    hi = np.max([m[1] + np.array(m[0].shape) for m in maps], axis=0)
    cnt = np.zeros(tuple(hi - lo), np.int8)
    for cubes, off in maps:
        s = off - lo
        cnt[s[0]:s[0] + cubes.shape[0], s[1]:s[1] + cubes.shape[1], s[2]:s[2] + cubes.shape[2]] += cubes
    return cnt, lo * CUBE


def cell_coords(cnt, origin):
    """d, theta, z of every cell centre, flattened like cnt.ravel()."""
    nx, ny, nz = cnt.shape
    x = origin[0] + CUBE * (np.arange(nx) + 0.5)
    y = origin[1] + CUBE * (np.arange(ny) + 0.5)
    z = origin[2] + CUBE * (np.arange(nz) + 0.5)
    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")
    return np.hypot(X, Y).ravel(), np.arctan2(Y, X).ravel(), Z.ravel(), np.broadcast_to(np.arange(nz), cnt.shape).ravel()


def scores(pred, true):
    """Precision, recall and within-one-bin share (see the task): reachable = count >= 1."""
    pr, tr = pred >= 1.0, true >= 1
    tp = int((pr & tr).sum())
    return {"precision": tp / max(int(pr.sum()), 1), "recall": tp / max(int(tr.sum()), 1), "within_1_bin": float((np.abs(pred[tr] - true[tr]) <= 1.0).mean()),
            "cells": int(true.size), "truly_reachable": int(tr.sum()), "predicted_reachable": int(pr.sum())}


def fit(d, th, z, y, p0, th0_free):
    def resid(v):
        p, t0 = (v[:8], v[8]) if th0_free else (v, 0.0)
        return predict_count(d, th, z, p, t0) - y
    lo = np.full(len(p0), -np.inf)
    hi = np.full(len(p0), np.inf)
    r = least_squares(resid, p0, bounds=(lo, hi), method="trf", x_scale="jac", max_nfev=300)
    return r.x, float(np.sqrt(np.mean(r.fun ** 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/reports/reach_model_fit.json")
    args = ap.parse_args()
    cnt, origin = load_counts()
    nx, ny, nz = cnt.shape
    d, th, z, iz = cell_coords(cnt, origin)
    y = cnt.ravel().astype(float)
    keep = d <= MAX_D
    fit_set = keep & (iz % 2 == 0)
    test_set = keep & (iz % 2 == 1)
    facts = {"cube_grid": [nx, ny, nz], "cube_m": CUBE, "voxel_m": 0.005, "origin_shoulder_frame_m": [round(float(v), 3) for v in origin],
             "extent_m": {"x": [float(origin[0]), float(origin[0] + CUBE * nx)], "y": [float(origin[1]), float(origin[1] + CUBE * ny)], "z": [float(origin[2]), float(origin[2] + CUBE * nz)]},
             "cells": int(cnt.size), "cells_reachable_by_some_bin": int((cnt >= 1).sum()), "max_count": int(cnt.max()), "count_histogram": np.bincount(cnt.ravel()).tolist(),
             "fit_slices": int(len(np.unique(iz[fit_set]))), "test_slices": int(len(np.unique(iz[test_set])))}
    rb = (cnt >= 1).ravel()
    facts["reachable_bearing_deg_mean"] = float(np.degrees(th[rb]).mean())
    facts["reachable_d_m"] = [float(d[rb].min()), float(d[rb].max())]
    out = {"facts": facts, "forms": {}}
    starts = [np.array([3.0, 0.0, 0.4, 0.0, 0.2, 0.0, 1.5, 0.0]), np.array([4.0, 2.0, 0.5, 0.2, 0.15, 0.0, 1.0, 0.0])]
    for name, free in (("specified (angular centre 0)", False), ("alternative (constant angular centre th0)", True)):
        best = None
        for s in starts:
            for t0 in ((0.0,) if not free else (0.0, 0.8)):
                v0 = np.concatenate([s, [t0]]) if free else s
                v, rmse = fit(d[fit_set], th[fit_set], z[fit_set], y[fit_set], v0, free)
                if best is None or rmse < best[1]:
                    best = (v, rmse)
        v = best[0]
        p, t0 = (v[:8], float(v[8])) if free else (v, 0.0)
        pred_fit = predict_count(d[fit_set], th[fit_set], z[fit_set], p, t0)
        pred_test = predict_count(d[test_set], th[test_set], z[test_set], p, t0)
        out["forms"][name] = {"params": dict(zip(PARAM_NAMES, [float(a) for a in p])), "th0_rad": t0, "rmse_fit_bins": best[1],
                              "fitted_slices": scores(pred_fit, cnt.ravel()[fit_set]), "held_out_slices": scores(pred_test, cnt.ravel()[test_set])}
        print(name, json.dumps(out["forms"][name], indent=1), flush=True)
    tmp = args.out + ".tmp"
    Path(tmp).write_text(json.dumps({**out, "facts": facts}, indent=1))
    os.replace(tmp, args.out)
    print(json.dumps(facts, indent=1))


if __name__ == "__main__":
    main()
