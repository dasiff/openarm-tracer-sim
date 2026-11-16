"""One still picture of the cat (card, capsule, shell) in the viewer, from the stance-search overview camera, to look at the picture (no run). Writes the png given.
usage: MUJOCO_GL=egl python experiments/safety_filter_cat_still.py out.png [azimuth] [distance]"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")
import cv2  # noqa: E402
import mujoco  # noqa: E402

from demo_video import MOVED  # noqa: E402
from src import robot_specs  # noqa: E402
from src.cat_demo import CAT_CFG, cat_object, cat_position, draw_cat  # noqa: E402
from src.control_loop import _headless_context  # noqa: E402
from src.task_beaker_hotplate import bench_top_z, load_task  # noqa: E402
from src.task_config import SEARCH_DISPLAY_CFG as SD  # noqa: E402
from src.viewer_composite import CompositeRenderer  # noqa: E402

task = {**MOVED, "scene": CAT_CFG["scene"]}
sim = load_task(task, cameras=True)
model, data = sim.model, sim.data
cat = cat_object(cat_position(task, bench_top_z(model)))
W, H = SD["size"]
model.vis.global_.offwidth, model.vis.global_.offheight = max(model.vis.global_.offwidth, W), max(model.vis.global_.offheight, H)
gl, ctx = _headless_context(model)
comp = CompositeRenderer(model, ctx, robot_specs.CAMERA_SPECS, size=(W, H))
c = list(cat.centre)
az = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0
dist = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
info = {"time": 0.0, "base_vel": (0, 0, 0), "twist": (0, 0), "status": None, "mode": None, "fps": 30, "camera": {"lookat": c, "distance": dist, "azimuth": az, "elevation": -15.0},
        "decor": lambda scene, m: draw_cat(scene, m, cat, (data.qpos[0], data.qpos[1]), 1.0), "caption": "SAFETY FILTER ACTIVE"}
cv2.imwrite(sys.argv[1], cv2.cvtColor(comp.render(data, info), cv2.COLOR_RGB2BGR))
print("cat centre", c, "robot xy", data.qpos[0:2])
gl.free()
