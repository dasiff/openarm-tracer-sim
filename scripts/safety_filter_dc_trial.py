"""Part B trial set-up (measurement only; nothing here is part of the filter): the extended never-close list "in use" WITHOUT a loader change.

The file format and the loader for data/safety_filter/never_close_dc.json are David's decision (not built). So that the gate and the
profile can still run with the extended list, this script makes
  1. a trial shapes file: data/safety_filter/robot_shapes.json with the pairs listed by tools/never_close_dc.py appended to its existing
     `never_close.pairs` block (each marked "source": "dc"); the existing loader reads it as it is and checks the fingerprint;
  2. a scratch copy of the source tree (git archive of HEAD's src/ into data/safety_filter/dc_trial_tree, with models, data, deps,
     experiments and assets linked to the working tree, as the speed work's reference tree) whose src/safety_filter/config.py points SHAPES_FILE at the trial file. The gate and the profile take
     it with --src-root. The working tree's src/ is not touched.
Run: python scripts/safety_filter_dc_trial.py [--dc FILE]
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402

DC_FILE = ROOT / "data" / "safety_filter" / "never_close_dc.json"
TRIAL_SHAPES = "data/safety_filter/robot_shapes_dc_trial.json"      # relative to the project root, as config.SHAPES_FILE is
TRIAL_TREE = ROOT / "data" / "safety_filter" / "dc_trial_tree"
LINKED = ("models", "data", "deps", "experiments", "assets")         # links to the working tree, as in the speed work's reference tree


def make_shapes(dc_path):
    doc = json.loads((ROOT / cfg.SHAPES_FILE).read_text(encoding="utf-8"))
    dc = json.loads(Path(dc_path).read_text(encoding="utf-8"))
    if dc["fingerprint"] != doc["fingerprint"] or doc["never_close"]["fingerprint"] != doc["fingerprint"]:
        raise SystemExit("fingerprints differ: the DC file or the never-close block is for another robot model")
    have = {frozenset((p["a"], p["b"])) for p in doc["never_close"]["pairs"]}
    before = len(have)
    for e in dc["entries"]:
        a, b = e["shape_pairs"][0]
        if e["listed"] and frozenset((a, b)) not in have:
            have.add(frozenset((a, b)))
            doc["never_close"]["pairs"].append({"a": a, "b": b, "variables": e["variables"], "margin_m": e["margin_m"], "source": "dc",
                                                "min_lower_bound_m": e["min_lower_bound_m"]})
    out = ROOT / TRIAL_SHAPES
    tmp = out.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, out)
    print(f"wrote {out}: never-close pairs {before} -> {len(have)}")
    return before, len(have)


def make_tree():
    if TRIAL_TREE.exists():
        raise SystemExit(f"{TRIAL_TREE} exists: move it away by hand to build it again")
    TRIAL_TREE.mkdir(parents=True)
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    ar = subprocess.run(["git", "archive", "HEAD", "src"], cwd=ROOT, capture_output=True, check=True).stdout
    subprocess.run(["tar", "-x", "-C", str(TRIAL_TREE)], input=ar, check=True)
    for d in LINKED:
        os.symlink(ROOT / d, TRIAL_TREE / d)
    p = TRIAL_TREE / "src" / "safety_filter" / "config.py"
    s = p.read_text(encoding="utf-8")
    old = f'SHAPES_FILE = "{cfg.SHAPES_FILE}"'
    if s.count(old) != 1:
        raise SystemExit("config.py: the SHAPES_FILE line was not found exactly once")
    p.write_text(s.replace(old, f'SHAPES_FILE = "{TRIAL_SHAPES}"'), encoding="utf-8")
    (TRIAL_TREE / "HEAD").write_text(head + "\n", encoding="utf-8")
    print(f"wrote {TRIAL_TREE} (HEAD {head[:8]}, SHAPES_FILE -> {TRIAL_SHAPES})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dc", default=str(DC_FILE))
    a = ap.parse_args()
    make_shapes(a.dc)
    make_tree()
