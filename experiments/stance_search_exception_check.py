"""Real search with one injected failure in the second full check: the three files must still be written and the original error must reach the caller.

    source .venv/bin/activate && export MUJOCO_GL=egl && python experiments/stance_search_exception_check.py
"""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from src import full_check  # noqa: E402
from src.mpvi_postprocess import postprocess  # noqa: E402
from src.stance_select import apply_stance_selection  # noqa: E402
from src.task_config import FULLCHECK_CFG, TASK_CFG  # noqa: E402
from stance_search_proof import BEAKER_ON_HOTPLATE_PLAN, SAMPLER_SEED  # noqa: E402

FULLCHECK_CFG["seed_pin"] = 4242
orig_check = full_check.FullCheck.check
calls = {"n": 0}


def failing_check(self, *a, **k):
    calls["n"] += 1
    if calls["n"] == 2:
        raise RuntimeError("injected failure in the second full check (exception-path test)")
    return orig_check(self, *a, **k)


full_check.FullCheck.check = failing_check
from src import edge_sampling  # noqa: E402
orig_iter = edge_sampling.iter_verified_stances


def pinned(*a, **k):
    k["seed"] = SAMPLER_SEED
    return orig_iter(*a, **k)


edge_sampling.iter_verified_stances = pinned

run_dir = ROOT / "data" / "demo_record" / f"stance_exception_check_{time.strftime('%Y%m%d_%H%M%S')}"
run_dir.mkdir(parents=True)
tagged = postprocess(BEAKER_ON_HOTPLATE_PLAN, False, say=lambda *a, **k: None)
raised = None
try:
    apply_stance_selection(tagged, TASK_CFG, run_dir=run_dir, say=lambda *a, **k: None)
except RuntimeError as e:
    raised = str(e)
print("RAISED TO CALLER:", raised, flush=True)
for name in ("candidates_list.json", "full_check_solutions.json", "dryrun_trajectories.json"):
    p = run_dir / name
    print(name, p.exists(), p.stat().st_size if p.exists() else None, "bytes", flush=True)
c = json.loads((run_dir / "candidates_list.json").read_text())
print("accepted", c["accepted"], "drawn", len(c["candidates"]))
print("statuses", sorted({x["status"] for x in c["candidates"]}))
for x in c["candidates"]:
    if x["status"] == "interrupted":
        print("interrupted candidate", x["number"], {k: v for k, v in x["checks"].items()}, flush=True)
