"""Where the export process's memory goes: resident set size (RSS) after each setup step of the search part and after the first rendered frames.

    MUJOCO_GL=egl python experiments/video_export_timing/memory_profile.py <run folder>

For the question whether two renderer processes would fit beside each other under the 6 GB cap (decision for David, report of 2026-11-16).
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")


def rss_mb():
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("VmRSS"):
                return int(line.split()[1]) / 1024


def main(run_dir):
    print(f"{'after':48s} {'RSS MB':>8s}")
    print(f"{'python start':48s} {rss_mb():8.0f}")
    import cv2  # noqa: F401
    import mujoco
    import src.replay as replay_mod
    from src.control_loop import _headless_context
    from src.run_record import STATE_SPEC, load_recording
    from src.search_timeline import SearchRun
    from src.task_config import SEARCH_DISPLAY_CFG as CFG
    from src.viewer_composite import CompositeRenderer
    print(f"{'imports (cv2, mujoco, src)':48s} {rss_mb():8.0f}")
    run = SearchRun(Path(run_dir), CFG)
    rec = load_recording(Path(run_dir) / "recording")
    print(f"{'timeline and recording loaded':48s} {rss_mb():8.0f}")
    model, data, reset = replay_mod.build_model(rec["meta"])
    reset(model, data)
    print(f"{'model built':48s} {rss_mb():8.0f}")
    W, H = CFG["size"]
    model.vis.global_.offwidth, model.vis.global_.offheight = max(model.vis.global_.offwidth, W), max(model.vis.global_.offheight, H)
    gl, ctx = _headless_context(model)
    print(f"{'GL context and MjrContext':48s} {rss_mb():8.0f}")
    mujoco.mj_setState(model, data, rec["initial_state"], STATE_SPEC)
    mujoco.mj_forward(model, data)
    comp = CompositeRenderer(model, ctx, rec["meta"]["robot_specs"]["CAMERA_SPECS"], size=(W, H), panel_static=(rec["meta"].get("task") or {}).get("panel"), shadows=CFG["shadows"])
    print(f"{'CompositeRenderer (scene, rig)':48s} {rss_mb():8.0f}")
    st = run.state_at(0.0)
    g = st["ghost"]
    info = {"time": data.time, "base_vel": (0.0, 0.0, 0.0), "twist": (0.0, 0.0), "status": None, "mode": None, "fps": CFG["fps"], "clock_s": st["clock_s"], "clock_scale": CFG["clock_text_scale"],
            "search_rows": st["panel"], "camera": CFG["overview_camera"], "ghost": ({"x": g["x"], "y": g["y"], "yaw": g["yaw"], "pedestal": g["pedestal"], "left_q": g["left_q"], "right_q": g["right_q"]}, run.rgba(st))}
    for i in range(5):
        comp.render(data, info)
        print(f"{'frame %d rendered' % (i + 1):48s} {rss_mb():8.0f}")
    gl.free()


if __name__ == "__main__":
    main(sys.argv[1])
