"""Braking Phase L (performance, continued): one check per fix. Kinematics only (no simulator) except where noted.

Fix 3 (SF-101): the compiled static lever equals the Python loop (1e-12).

Fix 2 (SF-100): the compiled exact pair gap equals the Python reference (1e-6).

Fix 1 (SF-99): the plugin's per-call recorder holds the braking layer's time next to the resolver's time, so the full call (resolver plus
braking) is in the recording.

Run: python experiments/safety_filter_braking_l_test.py   (exit 0 = all pass).
"""
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter.resolver import FilterStatus  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402
from src.safety_filter_plugin import SafetyFilterPlugin  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


# ---- fix 1 (SF-99): the recorder holds the braking time next to the resolver time ----
HOME = np.zeros((2, 7))


def slow_resolver(raw, last, state, world, held):
    time.sleep(0.005)                                  # the resolver's share of the tick: about 5 ms
    return np.asarray(raw, float), FilterStatus(status="PASS")


class SlowBraking:
    """Stands in for the braking layer: about 10 ms, returns the command as it is with the fields the plugin reads."""

    errors = []

    def process(self, last, filtered, state, obstacle_fn=None, hold=False, call_dt=None):
        time.sleep(0.010)
        binfo = {"lookahead": [None, None], "stop": [False, False], "override": [False, False],
                 "limit": [np.inf, np.inf], "speed": [0.0, 0.0]}
        return np.asarray(filtered, float), binfo


state = RobotState(q_left=(0.0,) * 7, q_right=(0.0,) * 7)
plugin = SafetyFilterPlugin("on", slow_resolver, lambda holding: (state, [], None), run_name="test_phase_l",
                            braking=SlowBraking(), obstacle_fn=None)
arm_targets = {"left": [0.0] * 7, "right": [0.0] * 7}
current = {"left": [0.0] * 7, "right": [0.0] * 7}
for k in range(5):
    plugin.process(arm_targets, {"left": False, "right": False}, current, t=0.01 * k)
rec = plugin.rec
ft = np.asarray(rec["filter_time"], float)
bt = np.asarray(rec["braking_time"], float)
check("fix 1 (SF-99): the recorder has the braking time, one value per call", len(bt) == len(ft) == 5, f"calls {len(ft)}, braking values {len(bt)}")
check("fix 1 (SF-99): the resolver time is recorded as before (about 5 ms per call)", bool(np.all(ft >= 0.004)), f"mean {ft.mean() * 1e3:.2f} ms")
check("fix 1 (SF-99): the braking time is recorded (about 10 ms per call)", bool(np.all(bt >= 0.009)), f"mean {bt.mean() * 1e3:.2f} ms")
full = ft + bt
check("fix 1 (SF-99): the full call (resolver plus braking) is the sum: about 15 ms per call", bool(np.all(full >= 0.013)),
      f"full call mean {full.mean() * 1e3:.2f} ms")
plugin_off = SafetyFilterPlugin("on", slow_resolver, lambda holding: (state, [], None), run_name="test_phase_l_nb", braking=None)
plugin_off.process(arm_targets, {"left": False, "right": False}, current, t=0.0)
check("fix 1 (SF-99): without a braking layer the braking time is recorded as zero", plugin_off.rec["braking_time"][-1] == 0.0)

# ---- fix 2 (SF-100): the compiled exact pair gap equals the Python reference (1e-6), over the tick's pairs at recorded poses ----
from src import task_config as _tc  # noqa: E402,F401
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import tick_arrays as ta  # noqa: E402
from src.safety_filter.resolver import Resolver  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes  # noqa: E402
from src.safety_filter.tick import _Tick  # noqa: E402
from src.safety_filter.world import LabMapSource  # noqa: E402

rs = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
res = Resolver(rs, 1.0 / float(cfg.BRAKING["tick_hz"]))
world = LabMapSource(str(ROOT / "data" / "safety_filter" / "lab_map.json")).objects()
rec = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / "brkZ8_phaseI_20261117_193334.npz", allow_pickle=True)
worst_gap, n_pairs = 0.0, 0
for i in (2000, 3000, 4000, 5000):
    b = rec["base"][i]
    st = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][i]),
                    q_left=tuple(rec["q_meas"][i, 0]), q_right=tuple(rec["q_meas"][i, 1]),
                    finger_left=float(rec["fingers"][i, 0]), finger_right=float(rec["fingers"][i, 1]))
    t = _Tick(res, np.asarray(rec["raw"][i], float), np.asarray(rec["sent"][i - 1], float), st, world, None, awake=None)
    P = t._place(np.asarray(rec["sent"][i - 1], float))
    for p in range(min(len(t.tab), 150)):
        cfg.GEOMETRY["compiled"] = True
        g_jit = ta.exact_gap(t.tab, P, p)
        cfg.GEOMETRY["compiled"] = False
        g_py = ta.exact_gap(t.tab, P, p)
        cfg.GEOMETRY["compiled"] = True
        if np.isfinite(g_py):
            worst_gap = max(worst_gap, abs(g_jit - g_py))
        n_pairs += 1
check("fix 2 (SF-100): the compiled exact pair gap equals the Python reference (1e-6) over the table's pairs at 4 recorded poses",
      worst_gap < 1e-6, f"largest difference {worst_gap:.2e} over {n_pairs} pair evaluations")

# ---- fix 3 (SF-101): the compiled static lever equals the Python loop (1e-12) at recorded poses ----
worst_lever, lever_rows = 0.0, []
for i in (2000, 3000, 4000, 5000):
    b = rec["base"][i]
    st = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][i]),
                    q_left=tuple(rec["q_meas"][i, 0]), q_right=tuple(rec["q_meas"][i, 1]),
                    finger_left=float(rec["fingers"][i, 0]), finger_right=float(rec["fingers"][i, 1]))
    t = _Tick(res, np.asarray(rec["raw"][i], float), np.asarray(rec["sent"][i - 1], float), st, world, None, awake=None)
    sl = res.statics.sleep
    d = float(np.max(np.abs(sl._static_lever(t) - sl._static_lever_loop(t))))
    worst_lever = max(worst_lever, d)
    lever_rows.append(i)
check("fix 3 (SF-101): the compiled static lever equals the Python loop it replaces (1e-12) at 4 recorded poses",
      worst_lever < 1e-12, f"largest difference {worst_lever:.2e} over rows {lever_rows}")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
