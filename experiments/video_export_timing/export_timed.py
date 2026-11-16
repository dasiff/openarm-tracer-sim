"""The demo-video export (experiments/search_video.py) with a stop-watch on every stage, timers kept out of src/ (task of 2026-11-16).

    MUJOCO_GL=egl python experiments/video_export_timing/export_timed.py <run folder> --out DIR [--name moved] [--search-frames N] [--real-max-ticks T]

It calls the same functions search_video.py calls (render_search, replay_run through render_real's arguments, concat) and wraps, with time.perf_counter(),
the functions those call: the camera rig, the follow-view scene build and render, the pixel read, the panel/tile/clock bitmaps, the RGB to BGR conversion,
the frame bytes copy, the pipe write to ffmpeg, the physics step, the ghost pose, the timeline state, the writer's release (the encoder draining) and the
final concat. The output file is the same as search_video.py's (<out>/<name>_full_<stamp>.mp4) unless --search-frames or --real-max-ticks cut the run.

Output: <out>/<name>_timing_<stamp>.json (totals and per-call means per stage) and the same printed as a table. Wall-clock totals of the stages are
exclusive where they nest (a stage's own time excludes the timed stages it calls), so they add up to the whole.
"""

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import src.replay as replay_mod  # noqa: E402
import src.viewer_composite as vc  # noqa: E402
import src.cameras as cameras_mod  # noqa: E402
import src.control_loop as cl  # noqa: E402
import src.ghost_robot as ghost_mod  # noqa: E402
import src.search_timeline as tl  # noqa: E402
from src.run_record import load_recording  # noqa: E402
from src.task_config import SEARCH_DISPLAY_CFG as CFG  # noqa: E402
import search_video as sv  # noqa: E402
import demo_video as dv  # noqa: E402


class Timers:
    """Exclusive wall-clock totals per stage name (nested timed calls are subtracted from the caller's own time)."""

    def __init__(self):
        self.total, self.calls, self.stack, self.part, self.sub = {}, {}, [], "setup", ""

    def wrap(self, name, fn, sub=None):
        """A timed copy of fn; `sub` tags the stages called inside it (e.g. the mujoco calls made by the camera rig)."""
        def timed(*a, **k):
            key = f"{self.part}:{self.sub}{name}"
            t0 = time.perf_counter()
            self.stack.append(0.0)
            old = self.sub
            if sub is not None:
                self.sub = sub + "/"
            try:
                return fn(*a, **k)
            finally:
                self.sub = old
                dt = time.perf_counter() - t0
                inner = self.stack.pop()
                self.total[key] = self.total.get(key, 0.0) + dt - inner
                self.calls[key] = self.calls.get(key, 0) + 1
                if self.stack:
                    self.stack[-1] += dt
        return timed

    def table(self):
        rows = sorted(self.total.items(), key=lambda kv: -kv[1])
        lines = [f"{'stage':44s} {'total s':>10s} {'calls':>7s} {'ms/call':>9s}"]
        for k, v in rows:
            n = self.calls[k]
            lines.append(f"{k:44s} {v:10.2f} {n:7d} {1000 * v / max(n, 1):9.2f}")
        return "\n".join(lines)


T = Timers()


def install():
    """Replaces the functions the export calls with timed copies (module attributes only; nothing in src/ is edited)."""
    # the follow view (src/viewer_composite.py) and the camera rig (src/cameras.py) call mujoco.* by attribute, so the mujoco module is patched
    for name in ("mjv_updateScene", "mjr_render", "mjr_readPixels", "mj_step", "mj_kinematics", "mjv_addGeoms"):
        setattr(mujoco, name, T.wrap(name, getattr(mujoco, name)))
    cameras_mod.CameraRig.render = T.wrap("rig.render (own: 3 cameras' buffers)", cameras_mod.CameraRig.render, sub="rig")
    vc.CompositeRenderer.render = T.wrap("composite.render (own: numpy copies)", vc.CompositeRenderer.render, sub="follow")
    ghost_mod.GhostRobot.set_pose = T.wrap("ghost.set_pose (own)", ghost_mod.GhostRobot.set_pose, sub="ghost")
    ghost_mod.GhostRobot.draw = T.wrap("ghost.draw (own)", ghost_mod.GhostRobot.draw, sub="ghost")
    replay_mod.run_loop = T.wrap("run_loop (own: control replay, loop overhead)", replay_mod.run_loop, sub="loop")
    for name in ("hud_bitmap", "text_bitmap", "inset_tile", "clock_bitmap", "update_follow_camera", "settle_renderer"):
        setattr(vc, name, T.wrap(name, getattr(vc, name)))
    vc.np.flipud = T.wrap("np.flipud", np.flipud)
    vc.np.ascontiguousarray = T.wrap("np.ascontiguousarray", np.ascontiguousarray)
    tl.SearchRun.state_at = T.wrap("timeline.state_at", tl.SearchRun.state_at)
    cv2.cvtColor = T.wrap("cv2.cvtColor RGB->BGR", cv2.cvtColor)
    sv.cv2.cvtColor = cv2.cvtColor
    replay_mod.cv2.cvtColor = cv2.cvtColor
    dv.X264Writer.write = T.wrap("pipe write to ffmpeg (incl. tobytes)", dv.X264Writer.write)
    dv.X264Writer.release = T.wrap("writer.release (encoder drains)", dv.X264Writer.release)
    dv.X264Writer.__init__ = T.wrap("writer start (ffmpeg process)", dv.X264Writer.__init__)
    sv.X264Writer = dv.X264Writer
    cl._headless_context = T.wrap("GL context", cl._headless_context)
    sv._headless_context = cl._headless_context
    replay_mod._headless_context = cl._headless_context
    replay_mod.build_model = T.wrap("build_model", replay_mod.build_model)
    replay_mod.load_recording = T.wrap("load_recording", replay_mod.load_recording)
    sv.concat = T.wrap("ffmpeg concat", sv.concat)


def main(a):
    install()
    run_dir, out = Path(a.run_dir), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    name = a.name or run_dir.name
    stamp = time.strftime("%Y%m%d_%H%M%S")
    wall0 = time.perf_counter()
    T.part = "setup"
    run = T.wrap("SearchRun (timeline from the logs)", tl.SearchRun)(run_dir, CFG)
    rec = T.wrap("load_recording", load_recording)(run_dir / "recording")
    if a.search_frames:
        run.duration_s = a.search_frames / CFG["fps"]                 # cuts the search part to this many frames (timing only)
    T.part = "search"
    t0 = time.perf_counter()
    search_tmp, search_final = out / f".{name}_search_{stamp}.mp4.tmp", out / f"{name}_search_{stamp}.mp4"
    n = sv.render_search(run, rec, search_tmp)
    os.replace(search_tmp, search_final)
    t_search = time.perf_counter() - t0
    print(f"search part: {n} frames in {t_search:.1f} s ({1000 * t_search / max(n, 1):.0f} ms/frame)", flush=True)
    T.part = "real"
    t0 = time.perf_counter()
    real = out / f".{name}_real_{stamp}.mp4.tmp"
    work = real.parent / (real.stem + "_work")
    replay_mod.open_video = lambda path, fps, size, codec="mp4v", quality=None: dv.X264Writer(path, fps, size, str(path) + ".ffmpeg.log")
    r = replay_mod.replay_run(run_dir / "recording", out_dir=work, video="viewer", hz=float(CFG["fps"]), size=tuple(CFG["size"]), name="real", quiet=True, intro=None,
                              clock_offset_s=run.search_end_s, verify=False, info_hook=None, max_ticks=a.real_max_ticks)   # render_real's call, plus max_ticks
    shutil.move(str(work / "real.mp4"), str(real))
    shutil.rmtree(work, ignore_errors=True)
    t_real = time.perf_counter() - t0
    print(f"real part: {r.get('frames_rendered')} frames, {r['ticks']} ticks in {t_real:.1f} s ({1000 * t_real / max(r.get('frames_rendered') or 1, 1):.0f} ms/frame)", flush=True)
    T.part = "concat"
    t0 = time.perf_counter()
    full_final, full_tmp = out / f"{name}_full_{stamp}.mp4", out / f".{name}_full_{stamp}.mp4.tmp"
    sv.concat([search_final, real], full_tmp)
    os.replace(full_tmp, full_final)
    real.unlink()
    t_concat = time.perf_counter() - t0
    wall = time.perf_counter() - wall0
    summary = {"run": str(run_dir), "full_video": str(full_final), "search_frames": n, "search_wall_s": t_search, "real_frames": r.get("frames_rendered"), "real_ticks": r["ticks"],
               "real_wall_s": t_real, "concat_wall_s": t_concat, "wall_s": wall, "timed_sum_s": sum(T.total.values()), "stages": {k: {"total_s": v, "calls": T.calls[k]} for k, v in T.total.items()},
               "cut": {"search_frames": a.search_frames, "real_max_ticks": a.real_max_ticks}}
    (out / f"{name}_timing_{stamp}.json").write_text(json.dumps(summary, indent=1))
    print(T.table())
    print(f"\nwall {wall:.1f} s = search {t_search:.1f} + real {t_real:.1f} + concat {t_concat:.1f} (+ setup); timed stages sum {summary['timed_sum_s']:.1f} s; video {full_final}", flush=True)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="moved")
    ap.add_argument("--search-frames", type=int, default=None, help="render only this many search frames (a cut run, timing only)")
    ap.add_argument("--real-max-ticks", type=int, default=None, help="replay only this many control ticks of the real part (a cut run, timing only)")
    sys.exit(main(ap.parse_args()))
