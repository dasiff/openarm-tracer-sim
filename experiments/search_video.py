"""Renders the stance-search video of a successful run (decisions-log row M321): the ghost robot imagining candidate stances while the real robot sits at home, then the real run.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/search_video.py <run folder> [--out data/videos] [--stills-only] [--search-only] [--name staging]

The run folder is one that holds candidates_list.json, full_check_solutions.json, dryrun_trajectories.json and recording/ (e.g. data/demo_record/rerun_staging_20261113_000814). Nothing is re-run: it
only reads the logs and replays the recording (verified bit-identical, as demo_video.py does). All timing, colour and camera numbers are in task_config.SEARCH_DISPLAY_CFG: change one and render again.

  --stills-only   render five pictures (mid first other candidate, its red hold, mid accepted key poses, mid dry-run replay, green hold) to <out>/<name>_still_*.png, no video
  --search-only   write only the search part (parts 1 and 2) as its own file; without it the file with the real run (part 3) is written as well
Files (not in Git): <out>/<name>_search.mp4 and <out>/<name>_full.mp4 (under temporary names until complete), <name>_search_info.json.

Parts 1 and 2 use a fixed overview camera and the three camera tiles of the notebook layout; part 3 is the existing notebook layout with the clock continuing (search end plus the run's simulated time).
Shadows follow SEARCH_DISPLAY_CFG["shadows"] (off since 2026-11-16, row M322: the shadow maps were most of the render time; the picture is otherwise the same).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import imageio_ffmpeg  # noqa: E402
import mujoco  # noqa: E402

import src.replay as replay_mod  # noqa: E402
from src.control_loop import _headless_context  # noqa: E402
from src.robot_specs import SIMULATION_TIMESTEP  # noqa: E402
from src.run_record import STATE_SPEC, load_recording  # noqa: E402
from src.search_timeline import SearchRun  # noqa: E402
from src.task_config import SEARCH_DISPLAY_CFG as CFG  # noqa: E402
from src.viewer_composite import CompositeRenderer  # noqa: E402
from demo_video import X264Writer  # noqa: E402


def stills_times(run):
    """The five moments to look at: mid first other candidate, its red hold, mid accepted key poses, mid dry-run replay, green hold (video seconds)."""
    cfg = CFG
    keys = next(i for i in run.items if i["kind"] == "keys")
    dry = next(i for i in run.items if i["kind"] == "dryrun")
    hold = next(i for i in run.items if i["kind"] == "hold")
    first = run.items[0]
    return {"1_mid_first_other": first["t0"] + cfg["other_step_s"] / 2, "2_red_hold": first["t0"] + cfg["other_step_s"] + (cfg["other_candidate_s"] - cfg["other_step_s"]) / 2,
            "3_mid_accepted_poses": keys["t0"] + keys["dur"] / 2, "4_mid_dryrun": dry["t0"] + dry["dur"] / 2, "5_green_hold": hold["t0"] + hold["dur"] / 2}


def render_search(run, rec, out_path, only_times=None, log=print, frames=None, on_built=None):
    """Draws parts 1 and 2 frame by frame. Returns the number of frames (or {label: image} when only_times is given).

    Args:
        frames (tuple or None): (first, end) frame numbers of the part to draw (end not included); None draws all of it (the parallel export draws a piece per worker).
        on_built (callable or None): Called with no arguments once the model is built and its build memory released.
    """
    meta = rec["meta"]
    model, data, reset = replay_mod.build_model(meta)
    replay_mod.release_build_memory()
    if on_built is not None:
        on_built()
    reset(model, data)
    W, H = CFG["size"]
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, W)
    model.vis.global_.offheight = max(model.vis.global_.offheight, H)
    gl, ctx = _headless_context(model)
    mujoco.mj_setState(model, data, rec["initial_state"], STATE_SPEC)
    mujoco.mj_forward(model, data)
    comp = CompositeRenderer(model, ctx, meta["robot_specs"]["CAMERA_SPECS"], size=(W, H), panel_static=(meta.get("task") or {}).get("panel"), shadows=CFG["shadows"], tile_reuse_tol=CFG["tile_reuse_tol"])
    writer = None if only_times else X264Writer(out_path, CFG["fps"], (W, H), str(out_path) + ".ffmpeg.log")

    def frame_at(t):
        st = run.state_at(t)
        g = st["ghost"]
        rgba = run.rgba(st)
        info = {"time": data.time, "base_vel": (0.0, 0.0, 0.0), "twist": (0.0, 0.0), "status": None, "mode": None, "fps": CFG["fps"], "clock_s": st["clock_s"], "clock_scale": CFG["clock_text_scale"],
                "search_rows": st["panel"], "camera": CFG["overview_camera"],
                "ghost": ({"x": g["x"], "y": g["y"], "yaw": g["yaw"], "pedestal": g["pedestal"], "left_q": g["left_q"], "right_q": g["right_q"]}, rgba) if rgba and rgba[3] > 0 else None}
        return cv2.cvtColor(comp.render(data, info), cv2.COLOR_RGB2BGR)

    try:
        if only_times:
            return {label: frame_at(t) for label, t in only_times.items()}
        total = int(round(run.duration_s * CFG["fps"]))
        first, end = frames if frames else (0, total)
        for i in range(first, end):
            writer.write(frame_at(i / CFG["fps"]))
            if (i - first) % 60 == 0:
                log(f"  search frame {i}/{end}")
        return end - first
    finally:
        if writer:
            writer.release()
        gl.free()


def real_frame_tick(meta, frame):
    """The control tick in which real-part frame number `frame` is drawn (replay_run draws one every physics_hz / fps steps, at the end of a step)."""
    every = int(round(1.0 / (SIMULATION_TIMESTEP * CFG["fps"])))
    return (every * (frame + 1) - 1) // meta["steps_per_tick"]


def render_real(rec_dir, out_path, clock_offset_s, verify=False, log=print, info_hook=None, frames=None, on_built=None):
    """Part 3: the recording replayed in the notebook layout (no intro), the clock continuing from the search's end. Returns the replay report.

    Args:
        frames (tuple or None): (first, end) real-part frame numbers to draw (end not included); None draws all. The physics is replayed from the start up to the last of them
            (the parallel export gives each worker a piece).
        on_built (callable or None): Called with no arguments once the model is rebuilt and its build memory released.
    """
    extra = {}
    if frames:
        meta = json.loads((Path(rec_dir) / "meta.json").read_text())
        last = real_frame_tick(meta, frames[1] - 1)
        extra = {"render_ticks": (real_frame_tick(meta, frames[0]), last), "max_ticks": last + 1}
    work = Path(out_path).parent / (Path(out_path).stem + "_work")
    replay_mod.open_video = lambda path, fps, size, codec="mp4v", quality=None: X264Writer(path, fps, size, str(path) + ".ffmpeg.log")
    r = replay_mod.replay_run(rec_dir, out_dir=work, video="viewer", hz=float(CFG["fps"]), size=tuple(CFG["size"]), name="real", quiet=True, intro=None, clock_offset_s=clock_offset_s, verify=verify, info_hook=info_hook,
                              shadows="on" if CFG["shadows"] else "off", on_model_built=on_built, tile_reuse_tol=CFG["tile_reuse_tol"], **extra)
    log(f"  replay ok {r['ok']} ({r.get('n_ticks_recorded')} ticks, mismatch {r.get('mismatch')}); frames {r.get('frames_rendered')}")
    if verify and not r["ok"]:
        raise RuntimeError(f"the replay did not reproduce the run: {r.get('mismatch')}")
    shutil.move(str(work / "real.mp4"), str(out_path))
    shutil.rmtree(work, ignore_errors=True)
    return r


def concat(parts, out_path):
    """Joins MP4 files of the same codec settings without re-encoding."""
    lst = Path(str(out_path) + ".list.txt")
    lst.write_text("".join(f"file '{Path(p).resolve()}'\n" for p in parts))
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "warning", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", "-movflags", "+faststart", "-f", "mp4", str(out_path)], check=True)
    lst.unlink()


PHYSICS_COST = 0.08         # one real-part frame of physics replay (about 20 ms) as a fraction of one frame's drawing time (about 250 ms): how much later pieces cost more
BUILD_COST = 24             # a second model build in one worker (about 6 s) in frames of drawing time


def plan_pieces(n_search, n_real, workers):
    """Cuts the frames [search part | real part] into one contiguous piece per worker so that the workers finish together.

    A piece that reaches into the real part also pays the physics replay from the recording's start up to its last frame (PHYSICS_COST per real frame up to there),
    and a piece that holds both parts pays a second model build (BUILD_COST). Finds, by bisection, the smallest finishing time at which a greedy fill uses all frames.

    Returns:
        list[tuple]: (first, end) global frame numbers (the search frames come first), one per worker; an empty piece is (end, end).
    """
    total = n_search + n_real

    def cost(a, b):
        c = b - a
        if b > n_search:
            c += PHYSICS_COST * (b - n_search)
            if a < n_search:
                c += BUILD_COST
        return c

    def fill(limit):
        pieces, g = [], 0
        for _ in range(workers):
            lo, hi = g, total               # the largest end with cost(g, end) <= limit (cost grows with end)
            while lo < hi:
                mid = (lo + hi + 1) // 2
                lo, hi = (mid, hi) if cost(g, mid) <= limit else (lo, mid - 1)
            pieces.append((g, lo))
            g = lo
        return pieces, g >= total

    lo, hi = 0.0, float(total) * (1 + PHYSICS_COST) + BUILD_COST
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if fill(mid)[1] else (mid, hi)
    return fill(hi)[0]


def render_piece(run_dir, jobs, seg_dir, built, log=print):
    """One worker's share: renders its search frames and real frames (each as its own MP4) and returns the file names in video order.

    Args:
        run_dir (pathlib.Path): The run folder.
        jobs (dict): {"search": (first, end) or None, "real": (first, end) or None}, piece-local frame numbers of the two parts.
        seg_dir (pathlib.Path): Where the pieces go.
        built (callable): Called when a model build is finished (the parent starts the next worker after the first call).
    """
    out = []
    run = SearchRun(run_dir, CFG)
    tag = f"{jobs['id']:02d}"
    if jobs["search"]:
        rec = load_recording(run_dir / "recording")
        path = seg_dir / f"seg_{tag}_a_search.mp4"
        render_search(run, rec, path, log=log, frames=jobs["search"], on_built=built)
        out.append(str(path))
    if jobs["real"]:
        path = seg_dir / f"seg_{tag}_b_real.mp4"
        render_real(run_dir / "recording", path, run.search_end_s, log=log, frames=jobs["real"], on_built=built)
        out.append(str(path))
    return out


def worker_main(run_dir, jobs, seg_dir, built_event, results):
    """Entry point of a worker process (see render_piece); puts (job id, file names or an error text) in `results`."""
    try:
        names = render_piece(Path(run_dir), jobs, Path(seg_dir), built_event.set, log=lambda m: print(f"[worker {jobs['id']}] {m}", flush=True))
        results.put((jobs["id"], names, None))
    except BaseException as e:  # noqa: BLE001  (the parent reports it)
        results.put((jobs["id"], None, f"{type(e).__name__}: {e}"))
        raise
    finally:
        built_event.set()


def render_parallel(run_dir, run, workers, seg_dir):
    """Draws the whole video with `workers` processes and returns the MP4 pieces in video order. Starts the workers one after the other (each after the previous
    one has built its model) because a model build briefly needs about 2 GB."""
    import multiprocessing as mp
    n_search, n_real = int(round(run.duration_s * CFG["fps"])), None
    rec_meta = json.loads((Path(run_dir) / "recording" / "meta.json").read_text())
    n_real = int(rec_meta["n_ticks"] * rec_meta["steps_per_tick"] // int(round(1.0 / (SIMULATION_TIMESTEP * CFG["fps"]))))
    pieces = plan_pieces(n_search, n_real, workers)
    seg_dir.mkdir(parents=True, exist_ok=True)
    ctx = mp.get_context("spawn")
    results, procs = ctx.Queue(), []
    for k, (a, b) in enumerate(pieces):
        if b <= a:
            continue
        jobs = {"id": k, "search": (a, min(b, n_search)) if a < n_search else None, "real": (max(a, n_search) - n_search, b - n_search) if b > n_search else None}
        ev = ctx.Event()
        pr = ctx.Process(target=worker_main, args=(str(run_dir), jobs, str(seg_dir), ev, results))
        pr.start()
        procs.append((pr, ev, jobs))
        print(f"worker {k}: global frames {a} to {b} ({b - a}); search {jobs['search']}, real {jobs['real']}", flush=True)
        ev.wait(timeout=300)                                  # the next worker starts after this one's first model build
    names = {}
    for _ in procs:
        k, files, err = results.get()
        if err:
            for pr, _e, _j in procs:
                pr.terminate()
            raise RuntimeError(f"worker {k} failed: {err}")
        names[k] = files
    for pr, _e, _j in procs:
        pr.join()
    return [f for k in sorted(names) for f in names[k]], n_search, n_real


def main(a):
    run_dir = Path(a.run_dir)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    name = a.name or run_dir.name
    run = SearchRun(run_dir, CFG)
    rec = load_recording(run_dir / "recording")
    if a.stills_only:
        imgs = render_search(run, rec, None, only_times=stills_times(run))
        for label, img in imgs.items():
            cv2.imwrite(str(out / f"{name}_still_{label}.png"), img)
        print("stills written to", out)
        return 0
    stamp = time.strftime("%Y%m%d_%H%M%S")
    workers = a.workers if a.workers else CFG["workers"]
    if workers > 1 and not a.search_only and not a.search_file:
        t0 = time.time()
        seg_dir = out / f".{name}_pieces_{stamp}"
        files, n_search, n_real = render_parallel(run_dir, run, workers, seg_dir)
        full_final, full_tmp = out / f"{name}_full_{stamp}.mp4", out / f".{name}_full_{stamp}.mp4.tmp"
        concat(files, full_tmp)
        os.replace(full_tmp, full_final)
        shutil.rmtree(seg_dir, ignore_errors=True)
        info = {"run": str(run_dir), "full_video": str(full_final), "workers": workers, "search_frames": n_search, "real_frames": n_real, "search_end_s": run.search_end_s, "other_candidates": run.other_numbers,
                "accepted": run.accepted, "settings": {k: (list(v) if isinstance(v, tuple) else v) for k, v in CFG.items()}, "render_wall_s": time.time() - t0}
        (out / f"{name}_search_info_{stamp}.json").write_text(json.dumps(info, indent=1, default=str))
        print(f"full video: {full_final} ({n_search + n_real} frames, {workers} workers, {time.time() - t0:.0f} s wall)", flush=True)
        return 0
    search_tmp, search_final = out / f".{name}_search_{stamp}.mp4.tmp", out / f"{name}_search_{stamp}.mp4"
    t0 = time.time()
    if a.search_file:
        search_final, n = Path(a.search_file), int(round(run.duration_s * CFG["fps"]))
    else:
        n = render_search(run, rec, search_tmp)
        os.replace(search_tmp, search_final)
    info = {"run": str(run_dir), "search_video": str(search_final), "search_frames": n, "search_duration_s": run.duration_s, "search_end_s": run.search_end_s, "other_candidates": run.other_numbers,
            "accepted": run.accepted, "settings": {k: (list(v) if isinstance(v, tuple) else v) for k, v in CFG.items()}, "search_render_wall_s": time.time() - t0}
    print(f"search part done: {search_final} ({n} frames, {time.time() - t0:.0f} s wall)", flush=True)
    if not a.search_only:
        real = out / f".{name}_real_{stamp}.mp4.tmp"
        r = render_real(run_dir / "recording", real, run.search_end_s, verify=a.verify)
        full_final = out / f"{name}_full_{stamp}.mp4"
        full_tmp = out / f".{name}_full_{stamp}.mp4.tmp"
        concat([search_final, real], full_tmp)
        os.replace(full_tmp, full_final)
        real.unlink()
        info.update(full_video=str(full_final), real_frames=r.get("frames_rendered"), real_ticks=r.get("n_ticks_recorded"), real_replay_bit_identical=r["ok"], real_mismatch=r.get("mismatch"), clock_at_real_start_s=run.search_end_s + 1.4)
        print(f"full video: {full_final}", flush=True)
    (out / f"{name}_search_info_{stamp}.json").write_text(json.dumps(info, indent=1, default=str))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("run_dir")
    ap.add_argument("--out", default=str(ROOT / "data" / "videos"))
    ap.add_argument("--name", default=None)
    ap.add_argument("--stills-only", action="store_true")
    ap.add_argument("--search-only", action="store_true")
    ap.add_argument("--search-file", default=None, help="use this already rendered search part instead of rendering it again")
    ap.add_argument("--workers", type=int, default=None, help="render with this many processes (default: SEARCH_DISPLAY_CFG[\"workers\"])")
    ap.add_argument("--verify", action="store_true", help="stop if the replay is not bit-identical to the recording (the recordings of 2026-11-13 are not: see the report)")
    sys.exit(main(ap.parse_args()))
