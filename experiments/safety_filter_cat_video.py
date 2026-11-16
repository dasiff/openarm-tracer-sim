"""The cat demo video: the stance-search display, then the manipulation with the cat, the red ghost arms at the raw command and the caption, ending when the run ended (5 s after the arm stopped).

    MUJOCO_GL=egl python experiments/safety_filter_cat_video.py <cat run folder> [--out data/videos]

Reuses experiments/search_video.py (search part from the run's logs, real part replayed from the recording with the display hook of src/cat_demo.py), joins the two parts with
ffmpeg without re-encoding. Output: <out>/safety_filter_cat_<YYYYMMDD>.mp4 (written under a temporary name, then renamed). Offline, one process.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np  # noqa: E402

import search_video as sv  # noqa: E402
import src.replay as replay_mod  # noqa: E402
from src.cat_demo import CAT_CFG, make_info_hook  # noqa: E402
from src.run_record import load_recording  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402
from src.search_timeline import SearchRun  # noqa: E402
from src.task_config import SEARCH_DISPLAY_CFG as CFG  # noqa: E402


def main(a):
    run_dir = Path(a.run)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    info = json.loads((run_dir / "run_info.json").read_text())
    z = np.load(ROOT / info["plugin_recording"])
    k0 = int(np.searchsorted(z["t"], info["cat_t_start"] or 0.0))
    face_xy = z["base"][min(k0, len(z["t"]) - 1)][:2]                       # the robot's base at the start of the manipulation: the card faces it
    cat = WorldObject(id="cat", shape="capsule", size=(CAT_CFG["radius_m"], CAT_CFG["half_axis_m"]), centre=tuple(info["cat_centre"]), rot=tuple(np.eye(3).ravel()), never_touch=True)
    run = SearchRun(run_dir, CFG)
    rec = load_recording(run_dir / "recording")
    model, _, _ = replay_mod.build_model(rec["meta"])
    hook = make_info_hook(model, ROOT / info["plugin_recording"], cat, face_xy, CAT_CFG, show_cat=True, t_appear=0.0)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    t0 = time.time()
    search_tmp = out / f".cat_search_{stamp}.mp4"
    n = sv.render_search(run, rec, search_tmp)
    print(f"search part: {n} frames, {time.time() - t0:.0f} s wall", flush=True)
    real = out / f".cat_real_{stamp}.mp4"
    r = sv.render_real(run_dir / "recording", real, run.search_end_s, verify=False, info_hook=hook)
    final = out / f"safety_filter_cat_{time.strftime('%Y%m%d')}.mp4"
    tmp = out / f".cat_full_{stamp}.mp4"
    sv.concat([search_tmp, real], tmp)
    os.replace(tmp, final)
    search_tmp.unlink()
    real.unlink()
    meta = {"video": str(final), "run": str(run_dir), "search_frames": n, "real_frames": r.get("frames_rendered"), "real_ticks": r.get("n_ticks_recorded"),
            "replay_ok_bit_identical": r["ok"], "wall_s": time.time() - t0, "fps": CFG["fps"], "duration_s_estimate": (n + (r.get("frames_rendered") or 0)) / CFG["fps"]}
    (out / f"safety_filter_cat_{time.strftime('%Y%m%d')}.json").write_text(json.dumps(meta, indent=1, default=str))
    print(f"video: {final} ({meta['duration_s_estimate']:.1f} s)", flush=True)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", default=str(ROOT / "data" / "videos"))
    sys.exit(main(ap.parse_args()))
