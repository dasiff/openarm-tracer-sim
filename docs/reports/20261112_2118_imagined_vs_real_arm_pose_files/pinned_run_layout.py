"""Scratch (2026-11-12): one recorded run of the Nov 10 run_layout launcher with the stance search's seeds pinned.
    python scratch/pinned_run_layout.py staging|moved <record folder> <sampler seed> <full check seed>
Nothing in src/ is changed; only the module attribute / config value are set for this process (as experiments/stance_search_proof.py does)."""
import argparse, sys
from pathlib import Path
ROOT = Path("/home/vboxuser/projects/openarm-tracer-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "experiments"))
from src.task_config import TASK_CFG, FULLCHECK_CFG
from src import edge_sampling
import demo_video
layout, out, sseed, fseed = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
FULLCHECK_CFG["seed_pin"] = fseed
orig = edge_sampling.iter_verified_stances
def pinned(*a, **k):
    k["seed"] = sseed
    return orig(*a, **k)
edge_sampling.iter_verified_stances = pinned
if layout == "staging":
    demo_video.MOVED = TASK_CFG
elif layout != "moved":
    raise SystemExit("layout is staging or moved")
sys.exit(demo_video.main(argparse.Namespace(instruction="put the beaker on the hotplate", hz=30, name=layout, no_bridge=False, record_only=out)))
