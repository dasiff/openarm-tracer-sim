"""Braking Phase M (performance, the candidate machinery and the arm kinematics): one check per fix. Kinematics only (no simulator).

Fix 1 (SF-102): the pair box separations and the lever arms of the resolver's tick are compiled; they equal the numpy references.

Run: python experiments/safety_filter_braking_m_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import tick_arrays as ta  # noqa: E402
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

# ---- fix 1 (SF-102): the compiled box separations and lever arms equal the numpy references ----
worst_sep, worst_lev, n_pairs = 0.0, 0.0, 0
for i in (2000, 3000, 4000, 5000):
    b = rec["base"][i]
    st = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][i]),
                    q_left=tuple(rec["q_meas"][i, 0]), q_right=tuple(rec["q_meas"][i, 1]),
                    finger_left=float(rec["fingers"][i, 0]), finger_right=float(rec["fingers"][i, 1]))
    t = _Tick(res, np.asarray(rec["raw"][i], float), np.asarray(rec["sent"][i - 1], float), st, world, None, awake=None)
    P = t._place(np.asarray(rec["sent"][i - 1], float))
    sep_jit = ta.box_separation(t.tab, P)
    sep_np = ta._box_separation_numpy(t.tab, P)
    worst_sep = max(worst_sep, float(np.max(np.abs(sep_jit - sep_np))) if len(sep_np) else 0.0)
    lev_jit = t.lever_at(P)
    lev_np = t._lever_at_numpy(P)
    worst_lev = max(worst_lev, float(np.max(np.abs(lev_jit - lev_np))))
    n_pairs += len(sep_np)
check("fix 1 (SF-102): the compiled pair box separations equal the numpy reference (1e-12) at 4 recorded poses",
      worst_sep < 1e-12, f"largest difference {worst_sep:.2e} over {n_pairs} pairs")
check("fix 1 (SF-102): the compiled lever arms equal the numpy reference (1e-12) at 4 recorded poses",
      worst_lev < 1e-12, f"largest difference {worst_lev:.2e}")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
