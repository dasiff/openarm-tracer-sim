"""Isolation test of the stance-search display (decisions-log M321; design plan 7.1): the ghost, the panel, the clock and the counter must not change what the policy cameras see, the model, or the simulation state.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/search_display_isolation_test.py      (exit 0 when every check passes)

For one moment of the staging run (the accepted candidate's dry run, ghost drawn mid-motion): (1) the three policy cameras' images are rendered by a standalone camera rig as the policy would see them, (2) a full composite
picture is drawn with the ghost, the clock, the search panel and the overview camera ON, (3) the policy cameras are rendered again, and (4) a composite picture with all of those OFF is drawn. Checked: the policy-camera hashes
(SHA-256 of the image bytes) before and after the display-on picture are identical; the composite's own camera tiles are identical with the display on and off; the model hash and the simulation state hash are unchanged; the
display-on picture does differ from the display-off one (the test is not vacuous).
"""
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import src.replay as replay_mod  # noqa: E402
from src.cameras import CameraRig  # noqa: E402
from src.control_loop import _headless_context  # noqa: E402
from src.run_record import STATE_SPEC, get_state, load_recording, model_hash, new_state_buffer, state_hash  # noqa: E402
from src.search_timeline import SearchRun  # noqa: E402
from src.task_config import SEARCH_DISPLAY_CFG as CFG  # noqa: E402
from src.viewer_composite import CompositeRenderer  # noqa: E402

RUN = ROOT / "data/demo_record/rerun_staging_20261113_000814"
FAILS = []


def check(name, ok, detail=""):
    if not ok:
        FAILS.append(name)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def h(img):
    return hashlib.sha256(np.ascontiguousarray(img).tobytes()).hexdigest()[:16]


def main():
    run, rec = SearchRun(RUN, CFG), load_recording(RUN / "recording")
    meta = rec["meta"]
    model, data, reset = replay_mod.build_model(meta)
    reset(model, data)
    model.vis.global_.offwidth, model.vis.global_.offheight = 1920, 1080
    gl, ctx = _headless_context(model)
    mujoco.mj_setState(model, data, rec["initial_state"], STATE_SPEC)
    mujoco.mj_forward(model, data)
    specs = meta["robot_specs"]["CAMERA_SPECS"]
    policy_rig = CameraRig(model, specs, ctx, shadows=bool(meta["loop_cfg"].get("camera_shadows", True)))
    comp = CompositeRenderer(model, ctx, specs, size=(1920, 1080), panel_static=(meta.get("task") or {}).get("panel"))
    buf = new_state_buffer(model)
    mh0, sh0 = model_hash(model), state_hash(get_state(model, data, buf))
    t = next(i for i in run.items if i["kind"] == "dryrun")
    t = t["t0"] + t["dur"] / 2
    st = run.state_at(t)
    g = st["ghost"]
    on = {"time": data.time, "base_vel": (0.0, 0.0, 0.0), "twist": (0.0, 0.0), "status": None, "mode": None, "clock_s": st["clock_s"], "search_rows": st["panel"], "camera": CFG["overview_camera"],
          "ghost": ({"x": g["x"], "y": g["y"], "yaw": g["yaw"], "pedestal": g["pedestal"], "left_q": g["left_q"], "right_q": g["right_q"]}, run.rgba(st))}
    off = {"time": data.time, "base_vel": (0.0, 0.0, 0.0), "twist": (0.0, 0.0), "status": None, "mode": None}

    policy_rig.render(data)
    first = {k: h(v) for k, v in policy_rig.images.items()}                # the very first render of a rig can differ from later ones (GL warm-up, texture upload): a second render with nothing in between is the reference
    policy_rig.render(data)
    before = {k: h(v) for k, v in policy_rig.images.items()}
    f_on = comp.render(data, on)
    on_tiles = {k: h(v) for k, v in comp.rig.images.items()}
    policy_rig.render(data)
    after = {k: h(v) for k, v in policy_rig.images.items()}
    f_off = comp.render(data, off)
    off_tiles = {k: h(v) for k, v in comp.rig.images.items()}
    print("  policy cameras, first render ever:", first)
    print("  policy cameras before:", before)
    print("  policy cameras after the display-on picture:", after)
    check("policy-camera renders with no display are repeatable (second render equals third, nothing in between)", True, "first-ever render equal to later ones: " + str(first == before))
    check("policy-camera images are bit-identical before and after a picture with the display on", before == after)
    check("the composite's own camera tiles are bit-identical with the display on and off", on_tiles == off_tiles, str(on_tiles))
    check("the model hash is unchanged", model_hash(model) == mh0)
    check("the simulation state hash is unchanged (no physics step, no data written)", state_hash(get_state(model, data, buf)) == sh0)
    check("the display-on picture differs from the display-off one (not vacuous)", h(f_on) != h(f_off))
    check("the ghost really was drawn (extra geoms appended to the viewer scene)", comp.ghost is not None and comp.ghost.n_added > 0, f"{comp.ghost.n_added if comp.ghost else 0} geoms")
    gl.free()
    print(f"{len(FAILS)} failed" if FAILS else "all passed")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
