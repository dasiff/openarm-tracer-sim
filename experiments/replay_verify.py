"""Replays a recording without any rendering and says whether every tick's state hash matches the recorded one: python experiments/replay_verify.py <recording folder>  (exit 0 if it does)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import src.replay as replay_mod

r = replay_mod.replay_run(sys.argv[1], video="none", verify=True, quiet=False)
print("replay ok:", r["ok"], "mismatch:", r["mismatch"])
sys.exit(0 if r["ok"] else 1)
