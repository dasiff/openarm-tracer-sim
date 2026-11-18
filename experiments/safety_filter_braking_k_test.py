"""Braking Phase K (profile the remaining cost): one check per fix. Kinematics only (no simulator).

Fix 2 (SF-97): the compiled gap-only batch distances equal the numpy versions (1e-10).

Fix 1 (SF-96): the static lever of the pair sleep timers is computed as arrays; it equals the Python loop it replaces.

Run: python experiments/safety_filter_braking_k_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.resolver import Resolver  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.tick import _Tick  # noqa: E402
from src.safety_filter.world import LabMapSource  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


rs = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
res = Resolver(rs, 1.0 / float(cfg.BRAKING["tick_hz"]))
world = LabMapSource(str(ROOT / "data" / "safety_filter" / "lab_map.json")).objects()
rec = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / "brkZ8_phaseI_20261117_193334.npz", allow_pickle=True)

# ---- fix 1 (SF-96): the static lever as arrays equals the loop ----
worst, worst_rows = 0.0, []
for i in range(2000, 5800, 400):
    b = rec["base"][i]
    st = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][i]),
                    q_left=tuple(rec["q_meas"][i, 0]), q_right=tuple(rec["q_meas"][i, 1]),
                    finger_left=float(rec["fingers"][i, 0]), finger_right=float(rec["fingers"][i, 1]))
    raw = np.asarray(rec["raw"][i], float)
    last = np.asarray(rec["sent"][i - 1], float)
    t = _Tick(res, raw, last, st, world, None, awake=None)
    sl = res.statics.sleep
    a = sl._static_lever(t)
    b_ = sl._static_lever_loop(t)
    d = float(np.max(np.abs(a - b_)))
    worst = max(worst, d)
    worst_rows.append(i)
check("fix 1 (SF-96): the static lever as arrays equals the Python loop it replaces (1e-12) at 10 recorded poses",
      worst < 1e-12, f"largest difference {worst:.2e} over rows {worst_rows}")

# ---- fix 2 (SF-97): the compiled gap-only batch distances equal the numpy versions they replace (1e-10) ----
from src.safety_filter import geometry as _geo  # noqa: E402
from src.safety_filter import batch_jit as _bj  # noqa: E402

rng = np.random.default_rng(3)
N = 600
A1 = rng.normal(0, 0.3, (N, 3)); B1 = A1 + rng.normal(0, 0.2, (N, 3))
A2 = rng.normal(0, 0.3, (N, 3)); B2 = A2 + rng.normal(0, 0.2, (N, 3))
B2[:20] = A2[:20]                                   # degenerate second segments
r1 = rng.uniform(0.01, 0.05, N); r2 = rng.uniform(0.01, 0.05, N)
g_np = _geo.capsule_capsule_batch(A1, B1, r1, A2, B2, r2, gap_only=True)
g_jit = _bj.capsule_capsule_gap_batch(A1, B1, r1, A2, B2, r2)
d_cc = float(np.max(np.abs(g_np - g_jit)))
check("fix 2 (SF-97): capsule-capsule gaps, compiled equal numpy (1e-10) over 600 random pairs (20 degenerate)", d_cc < 1e-10, f"largest difference {d_cc:.2e}")

Cb = rng.normal(0, 0.3, (N, 3)); hb = rng.uniform(0.01, 0.2, (N, 3))
Rb = np.zeros((N, 3, 3))
for k in range(N):
    Q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    Rb[k] = Q
Ac = rng.normal(0, 0.4, (N, 3)); Bc = Ac + rng.normal(0, 0.3, (N, 3))
rc = rng.uniform(0.005, 0.05, N)
g_np = _geo.capsule_box_batch(Ac, Bc, rc, Cb, Rb, hb, gap_only=True)
g_jit = _bj.capsule_box_gap_batch(Ac, Bc, rc, Cb, Rb, hb)
d_cb = float(np.max(np.abs(g_np - g_jit)))
check("fix 2 (SF-97): capsule-box gaps, compiled equal numpy (1e-10) over 600 random pairs (grid 33, 14 golden steps)", d_cb < 1e-10, f"largest difference {d_cb:.2e}")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
