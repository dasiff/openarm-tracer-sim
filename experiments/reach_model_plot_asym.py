"""PNG: the filtered maps' reachable region against the asymmetric Gaussian models (A6 and A8), in the shoulder frame, at the vertical offsets of the standard beaker and hotplate.

    source .venv/bin/activate && python experiments/reach_model_plot_asym.py     (reads docs/reports/reach_model_fit_asym.json; no MuJoCo)
"""
import json
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
from reach_model_plot import standard_offsets  # noqa: E402
from src.reach_model import predict_count_asym  # noqa: E402


def main():
    fit = json.loads((PROJECT_ROOT / "docs/reports/reach_model_fit_asym.json").read_text())
    cnt, origin = load_counts()
    nx, ny, nz = cnt.shape
    xs = origin[0] + CUBE * (np.arange(nx) + 0.5)
    ys = origin[1] + CUBE * (np.arange(ny) + 0.5)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    D, TH = np.hypot(X, Y), np.arctan2(Y, X)
    offs = standard_offsets()
    forms = list(fit["forms"].items())
    fig, axes = plt.subplots(len(offs), 1 + len(forms), figsize=(4.4 * (1 + len(forms)), 8.5), squeeze=False)
    for r, (name, (ox, oy, oz)) in enumerate(offs.items()):
        iz = int(np.clip(np.floor((oz - origin[2]) / CUBE), 0, nz - 1))
        zc = origin[2] + CUBE * (iz + 0.5)
        true = cnt[:, :, iz].astype(float)
        panels = [("maps: bins reaching (0-12)", true)]
        for fname, f in forms:
            p = (f["C_bins"], f["r0_m"], f["sigma_r_m"], f["sigma_theta_neg_rad"], f["sigma_theta_neg_per_m_rad"], f["sigma_theta_pos_rad"], f["sigma_theta_pos_per_m_rad"])
            panels.append((f"model {fname.split(' ')[0]}", predict_count_asym(np.hypot(D, zc), TH, zc, p, f["z_cut_m"])))
        for c, (title, arr) in enumerate(panels):
            ax = axes[r][c]
            im = ax.pcolormesh(xs, ys, arr.T, cmap="viridis", vmin=0, vmax=7, shading="auto")
            ax.contour(xs, ys, true.T, levels=[0.5], colors="white", linewidths=1.2)
            if c > 0:
                ax.contour(xs, ys, arr.T, levels=[1.0], colors="red", linewidths=1.2)
            ax.plot(ox, oy, "w+", ms=12, mew=2)
            ax.set_aspect("equal")
            ax.set_title(f"{name}: {title}\nz = {oz:+.3f} m, offset ({ox:.2f}, {oy:.2f})", fontsize=8)
            ax.set_xlabel("x forward, m")
            ax.set_ylabel("y left, m")
    fig.colorbar(im, ax=axes, label="bins", shrink=0.6)
    fig.suptitle("Filtered left maps vs asymmetric Gaussian (white: maps count >= 1; red: model count >= 1; +: object offset)", fontsize=9)
    fig.savefig(PROJECT_ROOT / "docs/reports/reach_model_asym_slices.png", dpi=105, bbox_inches="tight")


if __name__ == "__main__":
    main()
