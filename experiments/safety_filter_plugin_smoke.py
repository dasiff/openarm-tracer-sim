"""Plug-in smoke test (mode ON, measurement only): the left hand reaches along a straight joint-space path; ONE made-up box (in the world list
only, not in the simulator) sits in the hand's path. Reports statuses, the smallest gap to the box during the run (from the filter's own shapes at the
commands actually sent, and for the raw path), and whether the hand stopped short. Passes if the run completes and is recorded.
Run: bash experiments/run_capped.sh <log> python experiments/safety_filter_plugin_smoke.py   (no viewer, one MuJoCo process)"""
import os
import sys
from collections import Counter
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src import robot_specs  # noqa: E402
from src import task_config as tc  # noqa: E402
from src.control_loop import run_loop  # noqa: E402
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes  # noqa: E402
from src.safety_filter.touch import as_placed, gap_between  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402
from src.simulator import RobotSimulator  # noqa: E402

RUN = "smoke_on_" + __import__("time").strftime("%Y%m%d_%H%M%S")
FRAMES = 450                                           # ticks (3 s at 150 Hz) after the warm-up
DELTA = np.array([0.0, -0.5, 0.0, -0.9, 0.0, 0.0, 0.0])  # joint change of the left arm at the end of the reach (rad), my choice: straightens the folded elbow (stays inside the joint limits; the bridge rejects targets outside them)
SCENE = "models/scenes/chemistry_lab_combined.xml"

sim = RobotSimulator(model_path=str(ROOT / SCENE), cameras=None, pedestal_lift=robot_specs.PEDESTAL_SPECS)
model, data = sim.model, sim.data
q0 = np.array(tc.TASK_CFG["home_q_left"], float)
shapes = RobotShapes(load_robot_only_model(), ROOT / sfcfg.SHAPES_FILE)
box = {}
state_seen = {}


def extra(state):
    """The made-up box: centred on the left hand's position at 80% of the reach, 4 cm half size. Made once, from the first state."""
    state_seen.setdefault("s", state)
    if not box:
        from dataclasses import replace
        st = replace(state, q_left=tuple(q0 + 0.8 * DELTA))
        shapes.place(st)
        pos, _ = shapes.hand_frame("left")
        box["obj"] = WorldObject(id="made_up_box", shape="box", size=(0.04, 0.04, 0.04), centre=tuple(pos), rot=tuple(np.eye(3).ravel()))
    return [box["obj"]]


t_start = [None]


def step(obs):
    t = obs["tick"]
    if t_start[0] is None:
        t_start[0] = t
    s = min(1.0, (t - t_start[0]) / 300.0)
    return {"mode": "arms", "arm_targets": {"left": [float(v) for v in q0 + s * DELTA]}}


src = {"reach": {"name": "reach", "step": step}}
cfg = {"source_hz": 150.0, "cameras": False, "inset_cameras": False, "spawn_pos": [-1.0, 3.5, 0.22], "bench_frames": FRAMES,
       "routes": {"arms.left": "reach"}, "safety_filter_mode": "on", "safety_filter_run_name": RUN, "safety_filter_extra_objects": extra}
res = run_loop(model, data, src, cfg, viewer=None)

z = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / f"{RUN}.npz")
print("ticks recorded:", len(z["t"]), " statuses:", dict(Counter(z["status"].tolist())), " reasons:", dict(Counter(z["reason"][z["reason"] != ""].tolist())))
st0 = state_seen["s"]
from dataclasses import replace  # noqa: E402
bshape = as_placed(box["obj"])


def min_gap(arrs):
    best = (9.0, "")
    for q in arrs:
        placed = shapes.place(replace(st0, q_left=tuple(q[0]), q_right=tuple(q[1])))
        for p in placed:
            if p.arm == "left" and p.part not in ("torso",):
                g = gap_between(p, bshape)[0]
                if g < best[0]:
                    best = (float(g), p.id)
    return best


g0 = min_gap(z["sent"][:1])
print(f"gap to the box at the first command: {g0[0] * 1e3:.1f} mm ({g0[1]})")
gs = min_gap(z["sent"][::5])
gr = min_gap(z["raw"][::5])
print(f"smallest gap to the box, command sent: {gs[0] * 1e3:.1f} mm ({gs[1]});  raw path would have: {gr[0] * 1e3:.1f} mm ({gr[1]})   (every 5th tick)")
fin = np.abs(z["sent"][-1][0] - (q0 + DELTA)).max()
print(f"last sent command differs from the reach goal by up to {fin:.3f} rad -> hand {'stopped short' if fin > 0.02 else 'reached the goal'}")
print(f"filter time per call: mean {z['filter_time'].mean() * 1e3:.1f} ms, max {z['filter_time'].max() * 1e3:.1f} ms")
sys.exit(0 if len(z["t"]) > 0 else 1)
