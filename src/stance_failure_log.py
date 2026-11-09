"""Logs of the stance checks (M281): the generator state before each computed check, and a full record of each failed check with every check group run on its own.

Only reads and writes files. Nothing here draws a random number or changes an answer: src/full_check.py calls it after the answer is fixed and puts the state back afterwards.
The records are read back by experiments/stance_failure_replay.py, which reruns one check from its saved state.
"""
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def jsonable(x):
    """Plain JSON types: numpy arrays and scalars become lists and numbers (exact, so a value read back is the same float), tuples become lists, dict keys become strings."""
    if isinstance(x, np.ndarray):
        return jsonable(x.tolist())
    if isinstance(x, np.generic):
        return x.item()
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    return x


def append(path, obj):
    """Appends one JSON line and flushes it to disk, so a crash loses at most the line in progress."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a") as f:
        f.write(json.dumps(jsonable(obj), separators=(",", ":"), default=str) + "\n")
        f.flush()
        os.fsync(f.fileno())


def git_commit():
    """The commit hash of the code, and whether tracked files differ from it (the run may have used uncommitted edits). (None, None) when git is not available."""
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip() != ""
        return head, dirty
    except (OSError, subprocess.CalledProcessError):
        return None, None


def run_header(seed):
    """The generator state at the start of a run: the state of default_rng(seed) before any draw. Printed in the header of each pipeline or mock run's log."""
    return {"fullcheck_rng_seed": seed, "fullcheck_rng_state_at_start": np.random.default_rng(seed).bit_generator.state,
            "python_random": "not seeded (OS entropy); used only for the LLM retry delay (src/llm_gemini.py), never in a check", "time": time.strftime("%Y-%m-%d %H:%M:%S")}
