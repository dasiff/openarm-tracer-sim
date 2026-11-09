"""PNG: the maps' reachable region against the Gaussian model's, in the shoulder frame, at the vertical offsets of the standard beaker and hotplate.

    source .venv/bin/activate && python experiments/reach_model_plot.py     (reads docs/reports/reach_model_fit.json; no MuJoCo)

The offsets are those of the standard stance (TASK_CFG staging_xy, staging_yaw, pedestal_q): the shoulder world position comes from the
MuJoCo read saved by docs/reports/reach_beaker_slider.py --geometry (data/reach_heatmap/shoulder_q016.json: base-frame offset at pedestal 0.16 m; the pedestal only moves it vertically).
"""
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))

from reach_model_fit import CUBE, load_counts  # noqa: E402
from src.reach_model import PARAM_NAMES, predict_count  # noqa: E402
from src.task_config import BEAKER_CFG, PICK_PLACE_CFG, TASK_CFG  # noqa: E402
from src.task_beaker_hotplate import task_heights  # noqa: E402


def standard_offsets():
    """Shoulder-frame offsets (x, y, z) of the beaker's grasp point and the hotplate's place point at the standard stance."""
    g = json.loads((PROJECT_ROOT / "data/reach_heatmap/shoulder_q016.json").read_text())
    sx, sy, sz_q016 = g["shoulder_base_frame"]
    base_z, surface = g["robot_geometry_base_z"], g["bench"]["surface_z"]
    h, q = TASK_CFG["staging_yaw"], TASK_CFG["pedestal_q"]
    c, s = math.cos(h), math.sin(h)
    base = np.array(TASK_CFG["staging_xy"], float)
    sh_xy = base + np.array([c * sx - s * sy, s * sx + c * sy])
    sh_z = base_z + sz_q016 + (q - g["read_pedestal_m"])
    pad = PICK_PLACE_CFG["grasp_height_frac"] * BEAKER_CFG["height"]
    pts = {"beaker (grasp)": (*TASK_CFG["beaker_xy"], surface + pad), "hotplate (place)": (*TASK_CFG["hotplate_xy"], surface + task_heights()["plate_top"] - task_heights()["bench_top"] + pad)}
    out = {}
    for name, (x, y, zz) in pts.items():
        dx, dy = x - sh_xy[0], y - sh_xy[1]
        out[name] = (c * dx + s * dy, -s * dx + c * dy, zz - sh_z)          # R(-h) (P - shoulder), the rotation of stance_finder._region
    return out


def main():
    fit = json.loads((PROJECT_ROOT / "docs/reports/reach_model_fit.json").read_text())
    cnt, origin = load_counts()
    nx, ny, nz = cnt.shape
    xs = origin[0] + CUBE * (np.arange(nx) + 0.5)
    ys = origin[1] + CUBE * (np.arange(ny) + 0.5)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    D, TH = np.hypot(X, Y), np.arctan2(Y, X)
    forms = list(fit["forms"].items())
    offs = standard_offsets()
    fig, axes = plt.subplots(len(offs), 3, figsize=(13, 8.5), squeeze=False)
    for r, (name, (ox, oy, oz)) in enumerate(offs.items()):
        iz = int(np.clip(np.floor((oz - origin[2]) / CUBE), 0, nz - 1))
        true = cnt[:, :, iz].astype(float)
        panels = [("maps: bins reaching (0-12)", true)]
        for fname, f in forms:
            p = [f["params"][k] for k in PARAM_NAMES]
            panels.append((f"model, {fname.split(' (')[0]}", predict_count(D, TH, origin[2] + CUBE * (iz + 0.5), p, f["th0_rad"])))
        for c, (title, arr) in enumerate(panels):
            ax = axes[r][c]
            im = ax.pcolormesh(xs, ys, arr.T, cmap="viridis", vmin=0, vmax=7, shading="auto")
            ax.contour(xs, ys, true.T, levels=[0.5], colors="white", linewidths=1.2)           # the maps' reachable edge, on every panel
            if c > 0:
                ax.contour(xs, ys, arr.T, levels=[1.0], colors="red", linewidths=1.2)           # the model's count = 1 edge
            ax.plot(ox, oy, "w+", ms=12, mew=2)
            ax.set_aspect("equal")
            ax.set_title(f"{name}: {title}\nz = {oz:+.3f} m, offset ({ox:.2f}, {oy:.2f})", fontsize=8)
            ax.set_xlabel("x forward, m")
            ax.set_ylabel("y left, m")
    fig.colorbar(im, ax=axes, label="bins", shrink=0.6)
    fig.suptitle("Filtered left maps vs Gaussian model in the shoulder frame (white line: maps count >= 1; red line: model count >= 1; + : object offset)", fontsize=9)
    fig.savefig(PROJECT_ROOT / "docs/reports/reach_model_slices.png", dpi=110, bbox_inches="tight")
    print({k: [round(float(v), 3) for v in o] for k, o in offs.items()})


if __name__ == "__main__":
    main()
