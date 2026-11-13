"""The cat demo run (headless, no render): the scripted pick and place of the moved layout (the layout of the stance-search videos), safety filter ON, with the cat
(src/cat_demo.py) joining the world list when the manipulation starts. Writes the run folder (search logs, recording, episode, plug-in recording, cat_run.json).

    bash experiments/run_capped.sh <log> python experiments/safety_filter_cat_run.py <out dir> [--no-cat] [--max-minutes 240] [--hold-stop-s 5]

--no-cat runs the same in mode on without the cat (to show the filter alone does not break the run). Stops cleanly when the arm has been held for --hold-stop-s simulated
seconds (status not PASS and the command sent unchanged), or at the time cap.
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

from demo_video import MOVED, search_seeds  # noqa: E402
from src.cat_demo import CAT_CFG, CatInjector, cat_object, cat_position  # noqa: E402
from src.episode_runner import run_episode  # noqa: E402
from src.llm_gemini import GeminiCaller  # noqa: E402
from src.mpvi_pipeline import make_pipeline_task  # noqa: E402
from src.mpvi_planner import make_llm_planner  # noqa: E402
from src.task_beaker_hotplate import bench_top_z, load_task  # noqa: E402
from src.task_config import LLM_CFG  # noqa: E402

CAT_TASK = {**MOVED, "scene": CAT_CFG["scene"]}     # the moved layout on the scene that has the cat's visual geoms (nothing else differs)


def main(a):
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    instruction = "put the beaker on the hotplate"
    caller = GeminiCaller(mode="cache_only")
    plan = make_llm_planner(caller.bind("plan", LLM_CFG["planner_prompt_version"]))(instruction)
    sim = load_task(CAT_TASK, cameras=True)
    bottom = cat_position(CAT_TASK, bench_top_z(sim.model))
    cat = cat_object(bottom)
    inj = CatInjector(cat)
    task = make_pipeline_task(plan, task_cfg=CAT_TASK, stop_on_disagreement=True, checker="oracle", instruction=instruction, manipulation="scripted", run_dir=out)
    loop = {"command_bridge": {"enabled": True}, "safety_filter_mode": "on", "safety_filter_run_name": f"{out.name}",
            "safety_filter_max_minutes": a.max_minutes, "safety_filter_hold_stop_s": a.hold_stop_s}
    if not a.no_cat:
        loop["safety_filter_extra_objects"] = inj
    loop = {**(task.get("loop_cfg") or {}), **loop}
    t0 = time.perf_counter()
    s = run_episode(task, sim.model, sim.data, out_dir=out / "episode", record_dir=out / "recording", loop_cfg=loop, extra_sources=task.get("extra_sources"))
    state = task["pipeline"]["orchestrator"]["state"]
    seeds = search_seeds(task.get("stance_search_dir"))
    info = {"outcome": s["outcome"], "stance_search_dir": task.get("stance_search_dir"), **seeds, "no_cat": a.no_cat, "plugin_recording": f"data/safety_filter/plugin_runs/{out.name}.npz",
            "cat_bottom": list(map(float, bottom)), "cat_centre": list(cat.centre), "cat_t_start": inj.t_start, "plan_failure": state["failure"], "status": state["status"],
            "wall_s": time.perf_counter() - t0, "recording": "recording", "episode_summary": "episode/episode_summary.json",
            "base_xy_at_manip_start": None}
    (out / "run_info.json").write_text(json.dumps(info, indent=1, default=str))
    print(f"outcome {s['outcome']}; plan failure {state['failure']}; status {state['status']}; cat joined at t={inj.t_start}", flush=True)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--no-cat", action="store_true")
    ap.add_argument("--max-minutes", type=float, default=240.0)
    ap.add_argument("--hold-stop-s", type=float, default=5.0)
    sys.exit(main(ap.parse_args()))
